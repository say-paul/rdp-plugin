"""Blender add-on for discovering and placing Robot Library."""

from __future__ import annotations

import math
import json
import shutil
import subprocess
import urllib.error
import urllib.request
import xml.etree.ElementTree as ElementTree
from pathlib import Path
from typing import Any, Iterable, Mapping

import bpy
from bpy.props import BoolProperty, CollectionProperty, EnumProperty, FloatProperty, FloatVectorProperty, IntProperty, PointerProperty, StringProperty
from bpy.types import Node, NodeTree, Operator, Panel, PropertyGroup, UIList
from bpy_extras.io_utils import ExportHelper, ImportHelper
from mathutils import Euler, Matrix, Quaternion, Vector

from .asset_sources import AssetSource, discover_from_sources
from .behavior_graph import BehaviorGraph, BehaviorLink, BehaviorNode, PortSpec
from .container_builder import DEFAULT_BASE_IMAGE
from .port_catalog import catalog_node_template
from . import mounting
from .physics import blender as live_physics
from .setup_parser import discover_sensor_definitions
from .world_exporter import RobotInstance, SceneMeshInstance, SensorMount, WorldOptions, write_world_export


bl_info = {
    "name": "MuJoCo Robot Arm Library",
    "author": "Caryam",
    "version": (0, 9, 3),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar > Robot Library",
    "description": "Discover robot arms from a MuJoCo setup and drag them into the scene",
    "category": "3D View",
}


DEFAULT_SETUP_PATH = str(Path.home() / "git_repos" / "mujoco_menagerie")
SUPPORTED_IMPORT_EXTENSIONS = {".blend", ".dae", ".fbx", ".glb", ".gltf", ".obj", ".ply", ".stl", ".xml"}
TYPED_DATA_NODE_KINDS = {
    "application",
    "virtual_lidar",
    "virtual_rgb_camera",
    "virtual_depth_camera",
    "virtual_imu",
    "virtual_tactile_array",
    "joint_controller",
    "joint_target",
}
PORT_CONFIGURABLE_NODE_KINDS = {"application"}


class RobotLibraryItem(PropertyGroup):
    robot_id: StringProperty()
    name: StringProperty()
    model_path: StringProperty()
    source_path: StringProperty()
    description: StringProperty()
    category: StringProperty()
    instance_count: IntProperty(default=0)


class SensorLibraryItem(PropertyGroup):
    sensor_id: StringProperty()
    name: StringProperty()
    category: StringProperty()
    source_path: StringProperty()
    description: StringProperty()


class RobotAssetSource(PropertyGroup):
    path: StringProperty(name="Path", default="")
    kind: EnumProperty(
        name="Kind",
        items=(("auto", "Auto", ""), ("robot", "Robot", ""), ("sensor", "Sensor", "")),
        default="auto",
    )
    label: StringProperty(name="Label", default="")
    enabled: BoolProperty(name="Enabled", default=True)


class CaryamBehaviorTree(NodeTree):
    bl_idname = "CARYAM_BEHAVIOR_TREE"
    bl_label = "Caryam Data Graph"
    bl_icon = "NODETREE"

    def update(self) -> None:
        """Keep only links whose declared application-port shapes agree."""
        for link in list(self.links):
            source = link.from_node
            target = link.to_node
            if not isinstance(source, CaryamBehaviorNode) or not isinstance(target, CaryamBehaviorNode):
                continue
            if source.kind not in TYPED_DATA_NODE_KINDS or target.kind not in TYPED_DATA_NODE_KINDS:
                continue
            source_port = next((port for port in source.output_ports if port.port_name == link.from_socket.name), None)
            target_port = next((port for port in target.input_ports if port.port_name == link.to_socket.name), None)
            if source_port is None or target_port is None:
                continue
            if source_port.to_spec().compatibility_error(target_port.to_spec()):
                self.links.remove(link)


class CaryamPortSpec(PropertyGroup):
    port_name: StringProperty(name="Name", default="value")
    shape_csv: StringProperty(
        name="Shape",
        default="",
        description="Comma-separated tensor shape, e.g. 1,3,480,640",
    )
    data_type: EnumProperty(
        name="Type",
        items=(
            ("any", "Any", "Any serializable value"),
            ("image", "Image", "Image tensor or frame"),
            ("joint_state", "Joint State", "Current joint positions"),
            ("joint_positions", "Joint Positions", "Requested joint positions"),
            ("joint_target", "Joint Target", "Validated controller command"),
            ("pose", "Pose", "Position and orientation"),
            ("boolean", "Boolean", "Boolean state"),
            ("point_cloud2", "ROS PointCloud2", "sensor_msgs/PointCloud2 scan"),
            ("image_rgb8", "RGB8 Image", "sensor_msgs/Image encoded as rgb8"),
            ("image_depth32f", "Depth Image", "sensor_msgs/Image encoded as 32FC1"),
            ("imu", "IMU", "sensor_msgs/Imu"),
            ("tactile_array", "Tactile Array", "Per-cell tactile contact values"),
        ),
        default="any",
    )
    delivery: EnumProperty(
        name="Mode",
        items=(
            ("stream", "Stream", "Continuous sensor or application values"),
            ("state", "State", "Most recent state value"),
            ("event", "Event", "Discrete event"),
            ("command", "Command", "Validated command proposal"),
        ),
        default="stream",
    )
    dtype: StringProperty(name="Dtype", default="")
    semantic: StringProperty(name="Semantic", default="")
    unit: StringProperty(name="Unit", default="")
    rate_hz: FloatProperty(name="Rate (Hz)", default=0.0, min=0.0)
    variable: BoolProperty(name="Variable length", default=False)

    def shape_tuple(self) -> tuple[int, ...]:
        if not self.shape_csv.strip():
            return ()
        try:
            return tuple(int(part.strip()) for part in self.shape_csv.split(","))
        except ValueError:
            return ()

    def to_spec(self) -> PortSpec:
        return PortSpec(
            self.port_name, self.shape_tuple(), self.data_type, self.delivery,
            dtype=self.dtype, semantic=self.semantic, unit=self.unit,
            rate_hz=self.rate_hz if self.rate_hz > 0 else None, variable=self.variable,
        )


class CaryamBehaviorNode(Node):
    bl_idname = "CARYAM_BEHAVIOR_NODE"
    bl_label = "Data Node"

    node_id: StringProperty(name="Id", default="node")

    def _on_kind_update(self, context: bpy.types.Context) -> None:
        self.sync_sockets()

    def _on_camera_resolution_update(self, context: bpy.types.Context) -> None:
        output_name = "rgb_image" if self.kind == "virtual_rgb_camera" else "depth_image"
        channels = 3 if self.kind == "virtual_rgb_camera" else 1
        output = next((port for port in self.output_ports if port.port_name == output_name), None)
        if output is not None:
            output.shape_csv = f"1,{channels},{self.camera_height},{self.camera_width}"

    def camera_output_label(self) -> str:
        if self.kind == "virtual_rgb_camera":
            return f"Output: rgb_image [3X{self.camera_width}X{self.camera_height}] (RGB8)"
        return f"Output: depth_image [1X{self.camera_width}X{self.camera_height}] (32FC1)"

    kind: EnumProperty(
        name="Kind",
        items=(
            ("start", "Start", "Begin execution"),
            ("stop", "Stop", "Stop the data flow"),
            ("wait", "Wait", "Wait for the next data-flow tick"),
            ("retry", "Retry", "Retry on a later data-flow tick"),
            ("success", "Success", "Mark the current control branch successful"),
            ("failure", "Failure", "Mark the current control branch failed"),
            ("sensor", "Virtual Sensor", "Read a Blender or MuJoCo value"),
            ("ai_model", "AI Model", "Transform sensor data into a decision"),
            ("joint_controller", "Joint Controller", "Validate requested robot joint positions"),
            ("joint_target", "Joint Target", "Publish a validated joint command"),
            ("condition", "Condition", "Evaluate a boolean value"),
            ("action", "Robot Action", "Write a robot joint command"),
            ("application", "Application", "A named application with typed input and output ports"),
            ("virtual_lidar", "Virtual Lidar", "Emit a ROS PointCloud2 scan"),
            ("virtual_rgb_camera", "RGB Camera", "Emit a fixed RGB image stream"),
            ("virtual_depth_camera", "Depth Camera", "Emit a fixed depth image stream"),
            ("virtual_imu", "IMU", "Emit acceleration and angular velocity"),
            ("virtual_tactile_array", "Tactile Array", "Emit tactile contact readings"),
            ("end", "End", "Finish execution"),
        ),
        default="sensor",
        update=_on_kind_update,
    )
    sensor_path: StringProperty(name="Sensor", default="joint.qpos")
    model_name: StringProperty(name="Model", default="rule_based")
    robot_id: StringProperty(name="Robot")
    joint_name: StringProperty(name="Joint")
    target: FloatProperty(name="Target", default=0.0)
    lidar_max_range_m: FloatProperty(name="Max Range (m)", default=10.0, min=0.01)
    lidar_samples: IntProperty(name="Horizontal Samples", default=360, min=1)
    sensor_mount_frame: StringProperty(name="Mount Frame", default="")
    sensor_rate_hz: FloatProperty(name="Rate (Hz)", default=30.0, min=0.1)
    camera_width: IntProperty(name="Width", default=640, min=1, update=_on_camera_resolution_update)
    camera_height: IntProperty(name="Height", default=480, min=1, update=_on_camera_resolution_update)
    camera_fov_degrees: FloatProperty(name="Vertical FOV", default=60.0, min=1.0, max=179.0)
    depth_min_m: FloatProperty(name="Min Depth (m)", default=0.1, min=0.0)
    depth_max_m: FloatProperty(name="Max Depth (m)", default=10.0, min=0.01)
    imu_accel_noise: FloatProperty(name="Accel Noise", default=0.0, min=0.0)
    imu_gyro_noise: FloatProperty(name="Gyro Noise", default=0.0, min=0.0)
    imu_include_magnetometer: BoolProperty(name="Include Magnetometer", default=False)
    tactile_geom: StringProperty(name="Target Geom", default="")
    tactile_mesh: StringProperty(name="Tactile Mesh", default="")
    tactile_threshold: FloatProperty(name="Contact Threshold", default=0.0, min=0.0)
    handler: StringProperty(name="Handler", default="")
    catalog_locked: BoolProperty(name="Catalog Ports", default=False)
    catalog_data_json: StringProperty(name="Catalog Data", default="")
    input_ports: CollectionProperty(type=CaryamPortSpec)
    output_ports: CollectionProperty(type=CaryamPortSpec)

    def sync_sockets(self) -> None:
        if self.kind in TYPED_DATA_NODE_KINDS:
            wanted_inputs = [port.port_name for port in self.input_ports if port.port_name]
            wanted_outputs = [port.port_name for port in self.output_ports if port.port_name]
        else:
            wanted_inputs = ["value"]
            wanted_outputs = ["value"]

        for socket in list(self.inputs):
            if socket.name not in wanted_inputs:
                self.inputs.remove(socket)
        for name in wanted_inputs:
            if self.inputs.get(name) is None:
                self.inputs.new("NodeSocketFloat", name)

        for socket in list(self.outputs):
            if socket.name not in wanted_outputs:
                self.outputs.remove(socket)
        for name in wanted_outputs:
            if self.outputs.get(name) is None:
                self.outputs.new("NodeSocketFloat", name)

    def init(self, context: bpy.types.Context) -> None:
        self.sync_sockets()

    def draw_label(self) -> str:
        return self.label or self.kind.replace("_", " ").title()

    def draw_buttons(self, context: bpy.types.Context, layout: bpy.types.UILayout) -> None:
        header = layout.row(align=True)
        remove = header.operator("robot_library.delete_data_node", text="", icon="TRASH")
        remove.node_name = self.name
        if self.kind == "sensor":
            layout.prop(self, "sensor_path")
        elif self.kind == "ai_model":
            layout.prop(self, "model_name")
        elif self.kind == "action":
            layout.prop(self, "robot_id")
            layout.prop(self, "joint_name")
            layout.prop(self, "target")
        elif self.kind == "virtual_lidar":
            layout.prop(self, "lidar_max_range_m")
            layout.prop(self, "lidar_samples")
            layout.label(text="Output: point_cloud (ROS PointCloud2)")
        elif self.kind in {"virtual_rgb_camera", "virtual_depth_camera"}:
            layout.prop(self, "sensor_mount_frame")
            size = layout.row(align=True)
            size.prop(self, "camera_width")
            size.prop(self, "camera_height")
            layout.prop(self, "camera_fov_degrees")
            layout.prop(self, "sensor_rate_hz")
            if self.kind == "virtual_depth_camera":
                layout.prop(self, "depth_min_m")
                layout.prop(self, "depth_max_m")
                layout.label(text=self.camera_output_label())
            else:
                layout.label(text=self.camera_output_label())
        elif self.kind == "virtual_imu":
            layout.prop(self, "sensor_mount_frame")
            layout.prop(self, "sensor_rate_hz")
            layout.prop(self, "imu_accel_noise")
            layout.prop(self, "imu_gyro_noise")
            layout.prop(self, "imu_include_magnetometer")
            layout.label(text="Output: imu (sensor_msgs/Imu)")
        elif self.kind == "virtual_tactile_array":
            layout.prop(self, "tactile_geom")
            layout.prop(self, "tactile_mesh")
            layout.prop(self, "sensor_rate_hz")
            layout.prop(self, "tactile_threshold")
            layout.label(text="Output: tactile_array")
        elif self.kind in TYPED_DATA_NODE_KINDS:
            layout.prop(self, "handler")
            if self.catalog_locked:
                layout.label(text="Catalog ports", icon="LOCKED")
            columns = layout.row()
            columns.enabled = not self.catalog_locked
            input_column = columns.column()
            input_column.label(text="Inputs")
            for index, port in enumerate(self.input_ports):
                port_row = input_column.row(align=True)
                port_row.prop(port, "port_name", text="")
                port_row.prop(port, "shape_csv", text="")
                port_row.prop(port, "data_type", text="")
                port_row.prop(port, "delivery", text="")
                if not self.catalog_locked:
                    port_row.prop(port, "dtype", text="Dtype")
                    port_row.prop(port, "semantic", text="Semantic")
                    port_row.prop(port, "unit", text="Unit")
                remove = port_row.operator("robot_library.remove_behavior_port", text="", icon="X")
                remove.direction = "input"
                remove.port_index = index
            add_input = input_column.operator("robot_library.add_behavior_port", text="Add Input", icon="ADD")
            add_input.direction = "input"

            output_column = columns.column()
            output_column.label(text="Outputs")
            for index, port in enumerate(self.output_ports):
                port_row = output_column.row(align=True)
                port_row.prop(port, "port_name", text="")
                port_row.prop(port, "shape_csv", text="")
                port_row.prop(port, "data_type", text="")
                port_row.prop(port, "delivery", text="")
                if not self.catalog_locked:
                    port_row.prop(port, "dtype", text="Dtype")
                    port_row.prop(port, "semantic", text="Semantic")
                    port_row.prop(port, "unit", text="Unit")
                remove = port_row.operator("robot_library.remove_behavior_port", text="", icon="X")
                remove.direction = "output"
                remove.port_index = index
            add_output = output_column.operator("robot_library.add_behavior_port", text="Add Output", icon="ADD")
            add_output.direction = "output"


def _behavior_tree(context: bpy.types.Context) -> CaryamBehaviorTree | None:
    edit_tree = getattr(getattr(context, "space_data", None), "edit_tree", None)
    if isinstance(edit_tree, CaryamBehaviorTree):
        return edit_tree
    tree = getattr(context.scene, "robot_behavior_tree", None)
    return tree if isinstance(tree, CaryamBehaviorTree) else None


def _node_data(node: CaryamBehaviorNode) -> dict[str, object]:
    if node.kind == "application" and node.catalog_data_json:
        try:
            value = json.loads(node.catalog_data_json)
            return value if isinstance(value, dict) else {}
        except ValueError:
            return {}
    if node.kind == "sensor":
        return {"sensor": node.sensor_path}
    if node.kind == "ai_model":
        return {"model": node.model_name}
    if node.kind == "action":
        return {"robot_id": node.robot_id, "joint": node.joint_name, "target": node.target}
    if node.kind == "virtual_lidar":
        return {"max_range_m": node.lidar_max_range_m, "horizontal_samples": node.lidar_samples}
    if node.kind in {"virtual_rgb_camera", "virtual_depth_camera"}:
        data: dict[str, object] = {
            "mount_frame": node.sensor_mount_frame,
            "width": node.camera_width,
            "height": node.camera_height,
            "vertical_fov_degrees": node.camera_fov_degrees,
            "rate_hz": node.sensor_rate_hz,
        }
        if node.kind == "virtual_depth_camera":
            data.update({"min_depth_m": node.depth_min_m, "max_depth_m": node.depth_max_m})
        return data
    if node.kind == "virtual_imu":
        return {
            "mount_frame": node.sensor_mount_frame,
            "rate_hz": node.sensor_rate_hz,
            "accel_noise": node.imu_accel_noise,
            "gyro_noise": node.imu_gyro_noise,
            "include_magnetometer": node.imu_include_magnetometer,
        }
    if node.kind == "virtual_tactile_array":
        return {
            "geom": node.tactile_geom,
            "mesh": node.tactile_mesh,
            "rate_hz": node.sensor_rate_hz,
            "contact_threshold": node.tactile_threshold,
        }
    return {}


def _flow_dof(context: bpy.types.Context) -> int:
    root = _control_root(context)
    if root is None:
        return 6
    return len(_joint_controls(root)) or int(root.get("joint_count", 6)) or 6


def _set_data_ports(
    node: CaryamBehaviorNode,
    kind: str,
    inputs: tuple[PortSpec, ...],
    outputs: tuple[PortSpec, ...],
) -> None:
    node.kind = kind
    node.input_ports.clear()
    node.output_ports.clear()
    for spec in inputs:
        port = node.input_ports.add()
        _set_port_spec(port, spec)
    for spec in outputs:
        port = node.output_ports.add()
        _set_port_spec(port, spec)
    node.sync_sockets()


def _set_port_spec(port: CaryamPortSpec, spec: PortSpec) -> None:
    port.port_name = spec.name
    port.shape_csv = ",".join(str(value) for value in spec.shape)
    port.data_type = spec.data_type
    port.delivery = spec.delivery
    port.dtype = spec.dtype
    port.semantic = spec.semantic
    port.unit = spec.unit
    port.rate_hz = spec.rate_hz or 0.0
    port.variable = spec.variable


def _configure_flow_preset(node: CaryamBehaviorNode, preset: str, dof: int) -> None:
    presets = {
        "application": ("Application", "application", (), (PortSpec("output"),), (0.20, 0.32, 0.55)),
        "virtual_camera": ("RGB Camera", "virtual_rgb_camera", (), (PortSpec("rgb_image", (1, 3, 480, 640), "image_rgb8"),), (0.12, 0.36, 0.52)),
        "virtual_rgb_camera": ("RGB Camera", "virtual_rgb_camera", (), (PortSpec("rgb_image", (1, 3, 480, 640), "image_rgb8"),), (0.12, 0.36, 0.52)),
        "virtual_depth_camera": ("Depth Camera", "virtual_depth_camera", (), (PortSpec("depth_image", (1, 1, 480, 640), "image_depth32f"),), (0.10, 0.42, 0.50)),
        "virtual_imu": ("IMU", "virtual_imu", (), (PortSpec("imu", data_type="imu"),), (0.16, 0.45, 0.30)),
        "virtual_tactile_array": ("Tactile Array", "virtual_tactile_array", (), (PortSpec("tactile_array", data_type="tactile_array"),), (0.48, 0.30, 0.12)),
        "virtual_lidar": (
            "Virtual Lidar",
            "virtual_lidar",
            (),
            (PortSpec("point_cloud", data_type="point_cloud2"),),
            (0.12, 0.36, 0.52),
        ),
        "ai_model": (
            "AI Model",
            "application",
            (PortSpec("rgb", (1, 3, 480, 640), "image"),),
            (PortSpec("joint_positions", (1, dof), "joint_positions"),),
            (0.44, 0.25, 0.12),
        ),
        "actuator_system": (
            f"{dof}-Actuator System",
            "application",
            (PortSpec("joint_positions", (1, dof), "joint_positions"),),
            (),
            (0.38, 0.14, 0.12),
        ),
        "joint_controller": (
            "Joint Controller",
            "joint_controller",
            (PortSpec("target", data_type="joint_positions"),),
            (PortSpec("joint_target", data_type="joint_target", delivery="command"),),
            (0.38, 0.14, 0.12),
        ),
        "joint_target": (
            "Joint Target",
            "joint_target",
            (PortSpec("joint_target", data_type="joint_target", delivery="command"),),
            (PortSpec("joint_target", data_type="joint_target", delivery="command"),),
            (0.50, 0.16, 0.10),
        ),
    }
    label, kind, inputs, outputs, color = presets[preset]
    _set_data_ports(node, kind, inputs, outputs)
    node.label = label
    node.use_custom_color = True
    node.color = color
    node.width = 220.0


def _graph_from_tree(tree: CaryamBehaviorTree) -> BehaviorGraph:
    nodes: list[BehaviorNode] = []
    for node in tree.nodes:
        if not isinstance(node, CaryamBehaviorNode):
            continue
        inputs = ()
        outputs = ()
        handler = ""
        if node.kind in TYPED_DATA_NODE_KINDS:
            inputs = tuple(port.to_spec() for port in node.input_ports if port.port_name)
            outputs = tuple(port.to_spec() for port in node.output_ports if port.port_name)
            handler = node.handler
        nodes.append(
            BehaviorNode(
                node_id=node.node_id,
                kind=node.kind,
                label=node.label or node.name,
                data=_node_data(node),
                position=(float(node.location.x), float(node.location.y)),
                handler=handler,
                inputs=inputs,
                outputs=outputs,
            )
        )
    links: list[BehaviorLink] = []
    for link in tree.links:
        if not isinstance(link.from_node, CaryamBehaviorNode) or not isinstance(link.to_node, CaryamBehaviorNode):
            continue
        source_port = link.from_socket.name if link.from_node.kind in TYPED_DATA_NODE_KINDS else "value"
        target_port = link.to_socket.name if link.to_node.kind in TYPED_DATA_NODE_KINDS else "value"
        links.append(BehaviorLink(link.from_node.node_id, link.to_node.node_id, source_port, target_port))
    return BehaviorGraph(tree.name, nodes, links)


def _populate_behavior_tree(tree: CaryamBehaviorTree, graph: BehaviorGraph) -> None:
    tree.nodes.clear()
    node_map: dict[str, CaryamBehaviorNode] = {}
    for item in graph.nodes:
        node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
        node.node_id = item.node_id
        node.kind = item.kind if item.kind in {"start", "stop", "wait", "retry", "success", "failure", "sensor", "ai_model", "joint_controller", "joint_target", "condition", "action", "end", "application", "virtual_lidar", "virtual_rgb_camera", "virtual_depth_camera", "virtual_imu", "virtual_tactile_array"} else "sensor"
        node.label = item.label
        node.location = item.position
        node.handler = item.handler
        node.catalog_data_json = json.dumps(item.data) if item.kind == "application" and item.data else ""
        node.catalog_locked = bool(item.data.get("catalog_category"))
        data = item.data
        node.sensor_path = str(data.get("sensor", "joint.qpos"))
        node.model_name = str(data.get("model", "rule_based"))
        node.robot_id = str(data.get("robot_id", ""))
        node.joint_name = str(data.get("joint", ""))
        node.target = float(data.get("target", 0.0))
        node.lidar_max_range_m = float(data.get("max_range_m", 10.0))
        node.lidar_samples = int(data.get("horizontal_samples", 360))
        node.sensor_mount_frame = str(data.get("mount_frame", ""))
        node.sensor_rate_hz = float(data.get("rate_hz", 30.0))
        node.camera_width = int(data.get("width", 640))
        node.camera_height = int(data.get("height", 480))
        node.camera_fov_degrees = float(data.get("vertical_fov_degrees", 60.0))
        node.depth_min_m = float(data.get("min_depth_m", 0.1))
        node.depth_max_m = float(data.get("max_depth_m", 10.0))
        node.imu_accel_noise = float(data.get("accel_noise", 0.0))
        node.imu_gyro_noise = float(data.get("gyro_noise", 0.0))
        node.imu_include_magnetometer = bool(data.get("include_magnetometer", False))
        node.tactile_geom = str(data.get("geom", ""))
        node.tactile_mesh = str(data.get("mesh", ""))
        node.tactile_threshold = float(data.get("contact_threshold", 0.0))
        if node.kind in TYPED_DATA_NODE_KINDS:
            node.input_ports.clear()
            for port in item.inputs:
                entry = node.input_ports.add()
                _set_port_spec(entry, port)
            node.output_ports.clear()
            for port in item.outputs:
                entry = node.output_ports.add()
                _set_port_spec(entry, port)
        node.sync_sockets()
        node_map[item.node_id] = node
    for link in graph.links:
        source = node_map.get(link.source)
        target = node_map.get(link.target)
        if source is not None and target is not None:
            source_socket = source.outputs.get(link.source_port) if source.kind in TYPED_DATA_NODE_KINDS else source.outputs[0]
            target_socket = target.inputs.get(link.target_port) if target.kind in TYPED_DATA_NODE_KINDS else target.inputs[0]
            if source_socket is not None and target_socket is not None:
                tree.links.new(source_socket, target_socket)


def _ensure_behavior_tree(scene: bpy.types.Scene) -> CaryamBehaviorTree:
    tree = scene.robot_behavior_tree
    if tree is None:
        tree = bpy.data.node_groups.new("Caryam Data Graph", "CARYAM_BEHAVIOR_TREE")
        scene.robot_behavior_tree = tree
    return tree


def _blend_directory() -> Path:
    if bpy.data.filepath:
        return Path(bpy.data.filepath).resolve().parent
    return Path.cwd()


def _addon_directory() -> Path:
    return Path(__file__).resolve().parent


def _resolve_setup_path(raw_path: str) -> Path:
    candidate = Path(raw_path).expanduser()
    if not candidate.is_absolute():
        candidate = _blend_directory() / candidate
    candidate = candidate.resolve()
    if candidate.exists():
        return candidate

    alternatives = (
        Path.home() / "git_repos" / "mujoco_menagerie",
        _blend_directory() / "deep_mind" / "mujoco_manager" / "robot_setup",
        _blend_directory() / "deep_mind" / "mujoco_managerire" / "robot_setup",
        _blend_directory() / "deep_mind" / "mujoco_manager" / "robot_setup.py",
        _blend_directory() / "deep_mind" / "mujoco_managerire" / "robot_setup.py",
    )
    for alternative in alternatives:
        if alternative.exists():
            return alternative.resolve()
    if raw_path in {"", DEFAULT_SETUP_PATH}:
        demo_path = _addon_directory() / "demo_robot_setup.json"
        if demo_path.exists():
            return demo_path.resolve()
    return candidate


def _refresh_libraries(scene: bpy.types.Scene) -> None:
    setup_path = _resolve_setup_path(scene.robot_setup_path)
    sources = [AssetSource(str(setup_path), "auto", "Primary")]
    for item in scene.robot_asset_sources:
        if item.path.strip():
            extra_path = Path(bpy.path.abspath(item.path)).expanduser().resolve()
            sources.append(AssetSource(str(extra_path), item.kind, item.label or item.path, item.enabled))
    scene.robot_library.clear()
    scene.sensor_library.clear()
    try:
        definitions, sensors = discover_from_sources(sources)
    except Exception as error:
        scene.robot_library_status = f"Cannot read setup: {error}"
        return

    for definition in definitions:
        item = scene.robot_library.add()
        item.robot_id = definition.robot_id
        item.name = definition.name
        item.model_path = definition.model_path
        item.source_path = definition.source_path
        item.description = definition.description
        item.category = definition.category

    for definition in sensors:
        item = scene.sensor_library.add()
        item.sensor_id = definition.sensor_id
        item.name = definition.name
        item.category = definition.category
        item.source_path = definition.source_path
        item.description = definition.description

    if definitions:
        scene.robot_library_status = f"{len(definitions)} robot(s), {len(sensors)} sensor(s) found"
        scene.robot_library_index = min(scene.robot_library_index, len(definitions) - 1)
    else:
        scene.robot_library_status = f"No robot arms found in {setup_path}; {len(sensors)} sensor(s) found"
        scene.robot_library_index = 0
    scene.sensor_library_index = min(scene.sensor_library_index, max(len(sensors) - 1, 0))


def _refresh_library(scene: bpy.types.Scene) -> None:
    """Keep the existing internal single-source refresh entry point."""
    _refresh_libraries(scene)


def _refresh_active_library() -> None:
    scene = getattr(bpy.context, "scene", None)
    if scene is not None and hasattr(scene, "robot_library"):
        _refresh_libraries(scene)
    return None


def _asset_path(item: RobotLibraryItem) -> Path | None:
    if not item.model_path or item.model_path.startswith("builtin://"):
        return None
    path = Path(item.model_path).expanduser()
    if not path.is_absolute():
        path = Path(item.source_path).parent / path
    return path.resolve()


def _new_objects(before: set[bpy.types.Object]) -> list[bpy.types.Object]:
    return [obj for obj in bpy.data.objects if obj not in before]


def _import_mesh_file(path: Path) -> list[bpy.types.Object]:
    before = set(bpy.data.objects)
    suffix = path.suffix.lower()

    if suffix == ".obj":
        if hasattr(bpy.ops.wm, "obj_import"):
            bpy.ops.wm.obj_import(filepath=str(path))
        else:
            bpy.ops.import_scene.obj(filepath=str(path))
    elif suffix == ".stl":
        if hasattr(bpy.ops.wm, "stl_import"):
            bpy.ops.wm.stl_import(filepath=str(path))
        else:
            bpy.ops.import_mesh.stl(filepath=str(path))
    elif suffix == ".ply":
        if hasattr(bpy.ops.wm, "ply_import"):
            bpy.ops.wm.ply_import(filepath=str(path))
        else:
            bpy.ops.import_mesh.ply(filepath=str(path))
    elif suffix == ".fbx":
        bpy.ops.import_scene.fbx(filepath=str(path))
    elif suffix in {".glb", ".gltf"}:
        bpy.ops.import_scene.gltf(filepath=str(path))
    elif suffix == ".dae":
        bpy.ops.wm.collada_import(filepath=str(path))
    else:
        return []
    return _new_objects(before)


def _import_blend_file(path: Path) -> list[bpy.types.Object]:
    imported: list[bpy.types.Object] = []
    with bpy.data.libraries.load(str(path), link=False) as (data_from, data_to):
        data_to.collections = list(data_from.collections)
    for collection in data_to.collections:
        if collection is None:
            continue
        if collection.name not in bpy.context.scene.collection.children:
            bpy.context.scene.collection.children.link(collection)
        imported.extend(collection.all_objects)
    return imported


def _mjcf_values(value: str | None, count: int, defaults: tuple[float, ...]) -> tuple[float, ...]:
    if not value:
        return defaults
    try:
        values = tuple(float(part) for part in value.replace(",", " ").split())
    except ValueError:
        return defaults
    return values if len(values) == count else defaults


def _mjcf_transform(element: ElementTree.Element) -> Matrix:
    position = Vector(_mjcf_values(element.get("pos"), 3, (0.0, 0.0, 0.0)))
    if element.get("quat"):
        quat_values = _mjcf_values(element.get("quat"), 4, (1.0, 0.0, 0.0, 0.0))
        rotation = Quaternion(quat_values)
    elif element.get("euler"):
        euler = tuple(math.radians(value) for value in _mjcf_values(element.get("euler"), 3, (0.0, 0.0, 0.0)))
        rotation = Euler(euler).to_quaternion()
    elif element.get("axisangle"):
        axisangle = _mjcf_values(element.get("axisangle"), 4, (0.0, 0.0, 1.0, 0.0))
        rotation = Quaternion(Vector(axisangle[:3]), math.radians(axisangle[3]))
    else:
        rotation = Quaternion((1.0, 0.0, 0.0, 0.0))
    return Matrix.Translation(position) @ rotation.to_matrix().to_4x4()


def _resolve_mjcf_file(base_directory: Path, filename: str, compiler_directories: list[Path]) -> Path | None:
    requested = Path(filename).expanduser()
    candidates = [requested] if requested.is_absolute() else [base_directory / requested]
    candidates.extend(base_directory / directory / requested for directory in compiler_directories)
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.exists():
            return resolved
    return None


def _read_mjcf_documents(path: Path) -> tuple[list[tuple[ElementTree.Element, Path]], dict[str, tuple[Path, Matrix]]]:
    documents: list[tuple[ElementTree.Element, Path]] = []
    mesh_assets: dict[str, tuple[Path, Matrix]] = {}
    pending = [path.resolve()]
    visited: set[Path] = set()
    while pending:
        document_path = pending.pop()
        if document_path in visited:
            continue
        visited.add(document_path)
        root = ElementTree.parse(document_path).getroot()
        documents.append((root, document_path))
        compiler = root.find("compiler")
        compiler_directories = []
        if compiler is not None:
            compiler_directories = [
                Path(compiler.get(attribute))
                for attribute in ("assetdir", "meshdir")
                if compiler.get(attribute)
            ]
        for mesh in root.findall(".//asset/mesh"):
            filename = mesh.get("file")
            mesh_name = mesh.get("name") or Path(filename).stem if filename else ""
            if not filename or not mesh_name:
                continue
            mesh_path = _resolve_mjcf_file(document_path.parent, filename, compiler_directories)
            if mesh_path is None:
                continue
            mesh_transform = _mjcf_transform(mesh)
            scale = _mjcf_values(mesh.get("scale"), 3, (1.0, 1.0, 1.0))
            mesh_transform = mesh_transform @ Matrix.Diagonal((*scale, 1.0))
            mesh_assets[mesh_name] = (mesh_path, mesh_transform)
        for include in root.findall(".//include"):
            filename = include.get("file")
            if not filename:
                continue
            include_path = _resolve_mjcf_file(document_path.parent, filename, compiler_directories)
            if include_path is not None:
                pending.append(include_path)
    return documents, mesh_assets


def _collect_mjcf_geometries(
    element: ElementTree.Element,
    parent_transform: Matrix,
    mesh_assets: dict[str, tuple[Path, Matrix]],
    geometries: list[tuple[Path, Matrix]],
) -> None:
    for geom in element.findall("geom"):
        transform = parent_transform @ _mjcf_transform(geom)
        mesh_name = geom.get("mesh")
        if mesh_name in mesh_assets:
            mesh_path, mesh_transform = mesh_assets[mesh_name]
            geometries.append((mesh_path, transform @ mesh_transform))
    for body in element.findall("body"):
        _collect_mjcf_geometries(body, parent_transform @ _mjcf_transform(body), mesh_assets, geometries)


def _import_mjcf_meshes(path: Path) -> list[bpy.types.Object]:
    try:
        documents, mesh_assets = _read_mjcf_documents(path)
    except (ElementTree.ParseError, OSError):
        return []

    geometries: list[tuple[Path, Matrix]] = []
    for root, _document_path in documents:
        worldbody = root.find("worldbody")
        if worldbody is not None:
            _collect_mjcf_geometries(worldbody, Matrix.Identity(4), mesh_assets, geometries)
    if not geometries:
        geometries = [(mesh_path, mesh_transform) for mesh_path, mesh_transform in mesh_assets.values()]

    imported: list[bpy.types.Object] = []
    for mesh_path, transform in geometries:
        try:
            objects = _import_mesh_file(mesh_path)
        except RuntimeError:
            continue
        for obj in objects:
            obj.matrix_world = transform @ obj.matrix_world
        imported.extend(objects)
    return imported


def _robot_collection() -> bpy.types.Collection:
    collection = bpy.data.collections.get("Robot Library")
    if collection is None:
        collection = bpy.data.collections.new("Robot Library")
        bpy.context.scene.collection.children.link(collection)
    return collection


def _create_placeholder(name: str) -> list[bpy.types.Object]:
    before = set(bpy.data.objects)
    bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=0.18, depth=0.7, location=(0, 0, 0.35))
    bpy.context.object.name = f"{name} Base"
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 0.95), scale=(0.28, 0.18, 0.12))
    bpy.context.object.name = f"{name} Shoulder"
    bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=0.12, depth=0.9, location=(0, 0, 1.4))
    bpy.context.object.name = f"{name} Link"
    bpy.ops.mesh.primitive_cube_add(location=(0, 0, 1.95), scale=(0.2, 0.14, 0.1))
    bpy.context.object.name = f"{name} Wrist"
    return _new_objects(before)


def _parent_objects(objects: Iterable[bpy.types.Object], root: bpy.types.Object) -> None:
    collection = _robot_collection()
    for obj in objects:
        world_matrix = root.matrix_world @ obj.matrix_world
        for current_collection in list(obj.users_collection):
            current_collection.objects.unlink(obj)
        collection.objects.link(obj)
        obj.parent = root
        obj.matrix_world = world_matrix


def _add_location(scene: bpy.types.Scene) -> Vector:
    origin = scene.cursor.location.copy()
    if not scene.robot_library_auto_spread:
        return origin
    collection = bpy.data.collections.get("Robot Library")
    if collection is None:
        return origin
    placed_count = sum(obj.type == "EMPTY" and "robot_id" in obj for obj in collection.objects)
    if placed_count == 0:
        return origin
    columns = 4
    spacing = 2.5
    return origin + Vector(((placed_count % columns) * spacing, (placed_count // columns) * spacing, 0.0))


def _place_robot(item: RobotLibraryItem, location: Vector) -> bpy.types.Object:
    bpy.ops.object.empty_add(type="PLAIN_AXES", location=location)
    root = bpy.context.object
    root.name = f"{item.name} {item.instance_count + 1}"
    root["robot_id"] = item.robot_id
    root["source_path"] = item.source_path
    robot_collection = _robot_collection()
    for current_collection in list(root.users_collection):
        current_collection.objects.unlink(root)
    robot_collection.objects.link(root)

    asset_path = _asset_path(item)
    root["model_path"] = str(asset_path) if asset_path is not None else ""
    imported: list[bpy.types.Object] = []
    hierarchy_created = False
    if asset_path is not None and asset_path.exists():
        try:
            if asset_path.suffix.lower() == ".blend":
                imported = _import_blend_file(asset_path)
            elif asset_path.suffix.lower() == ".xml":
                try:
                    from .compiled_importer import import_compiled_mjcf

                    imported = import_compiled_mjcf(
                        asset_path,
                        root,
                        robot_collection,
                        include_collisions=bpy.context.scene.robot_library_include_collisions,
                        mesh_detail=bpy.context.scene.robot_library_mesh_detail,
                        preview_mode=bpy.context.scene.robot_library_preview_mode,
                    )
                    hierarchy_created = bool(imported)
                except (ImportError, ModuleNotFoundError, RuntimeError, ValueError) as error:
                    if bpy.context.scene.robot_library_preview_mode == "DETAILED":
                        imported = _import_mjcf_meshes(asset_path)
                        root["import_mode"] = "xml_fallback"
                    else:
                        root["import_mode"] = "proxy_unavailable"
                        root["import_error"] = str(error)
            elif asset_path.suffix.lower() in SUPPORTED_IMPORT_EXTENSIONS:
                imported = _import_mesh_file(asset_path)
        except (OSError, RuntimeError):
            imported = []

    if not imported:
        imported = _create_placeholder(item.name)
        if "import_mode" not in root:
            root["import_mode"] = "placeholder"
    if not hierarchy_created:
        _parent_objects(imported, root)
    item.instance_count += 1
    bpy.ops.object.select_all(action="DESELECT")
    root.select_set(True)
    bpy.context.view_layer.objects.active = root
    bpy.context.scene.robot_library_control_root = root
    return root


def _world_robot_instances() -> list[RobotInstance]:
    instances: list[RobotInstance] = []
    for root in sorted((obj for obj in bpy.context.scene.objects if obj.get("robot_id")), key=lambda obj: obj.name):
        source_path = str(root.get("model_path", ""))
        if not source_path or Path(source_path).suffix.lower() != ".xml":
            continue
        qpos = {
            str(joint.get("mujoco_joint_name", joint.name.removeprefix("Joint: "))): float(joint["qpos"])
            for joint in _joint_controls(root)
        }
        position = tuple(float(value) for value in root.matrix_world.translation)
        quaternion = tuple(float(value) for value in root.matrix_world.to_quaternion())
        instances.append(
            RobotInstance(
                instance_id=root.name,
                robot_id=str(root.get("robot_id", root.name)),
                source_path=source_path,
                position=position,
                quaternion=quaternion,
                qpos=qpos,
                sensors=tuple(_sensor_mounts(root)),
                mount=mounting.export_mount(root),
            )
        )
    return instances


def _sensor_mounts(root: bpy.types.Object) -> list[SensorMount]:
    mounts: list[SensorMount] = []
    for obj in sorted(bpy.context.scene.objects, key=lambda item: item.name):
        if not obj.get("sensor_id"):
            continue
        if _selected_robot_root(obj) is not root:
            continue
        source_path = str(obj.get("sensor_source_path", ""))
        payload = str(obj.get("sensor_definition_json", ""))
        try:
            if payload:
                from .sensor_spec import SensorDefinition
                definition = SensorDefinition.from_dict(str(obj["sensor_id"]), json.loads(payload), source_path)
            else:
                definition = next(
                    sensor for sensor in discover_sensor_definitions(source_path)
                    if sensor.sensor_id == str(obj["sensor_id"])
                )
        except (OSError, ValueError, StopIteration, TypeError):
            continue
        parent_body = obj.parent
        while (
            parent_body is not None and parent_body is not root
            and "mujoco_body_id" not in parent_body and not parent_body.name.startswith("Body: ")
        ):
            parent_body = parent_body.parent
        body_name = str(parent_body.get("mujoco_body_name", parent_body.name.removeprefix("Body: "))) if parent_body is not None and parent_body is not root else ""
        frame = parent_body if parent_body is not None else root
        local_matrix = frame.matrix_world.inverted() @ obj.matrix_world
        mounts.append(SensorMount(
            definition,
            obj.name.removeprefix("Sensor: ").lower().replace(" ", "_").replace(".", "_"),
            tuple(float(value) for value in local_matrix.translation),
            tuple(float(value) for value in local_matrix.to_quaternion()),
            body_name,
        ))
    return mounts


def _is_robot_library_object(obj: bpy.types.Object) -> bool:
    ancestor = obj
    while ancestor is not None:
        if ancestor.get("robot_id"):
            return True
        if ancestor.type == "MESH" and ancestor.caryam_mount.enabled:
            return False
        ancestor = ancestor.parent
    return False


def _write_scene_obj(objects: Iterable[bpy.types.Object], output_path: Path, world_space: bool = True) -> bool:
    lines: list[str] = ["# Caryam Blender scene export"]
    vertex_offset = 0
    face_count = 0
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for obj in objects:
        evaluated = obj.evaluated_get(depsgraph)
        mesh = evaluated.to_mesh()
        try:
            mesh.calc_loop_triangles()
            if not mesh.vertices or not mesh.loop_triangles:
                continue
            lines.append(f"o {obj.name}")
            for vertex in mesh.vertices:
                coordinate = evaluated.matrix_world @ vertex.co if world_space else vertex.co
                lines.append(f"v {coordinate.x:.9g} {coordinate.y:.9g} {coordinate.z:.9g}")
            for triangle in mesh.loop_triangles:
                vertices = [vertex_offset + vertex_index + 1 for vertex_index in triangle.vertices]
                lines.append(f"f {vertices[0]} {vertices[1]} {vertices[2]}")
                face_count += 1
            vertex_offset += len(mesh.vertices)
        finally:
            evaluated.to_mesh_clear()
    if not face_count:
        return False
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return True


def _is_dynamic_scene_object(obj: bpy.types.Object) -> bool:
    rigid_body = obj.rigid_body
    return bool(obj.get("mujoco_dynamic", False)) or (
        rigid_body is not None and rigid_body.enabled and rigid_body.type == "ACTIVE" and not rigid_body.kinematic
    )


def _scene_object_rgba(obj: bpy.types.Object) -> tuple[float, float, float, float]:
    material = obj.active_material
    if material is None:
        return (0.62, 0.66, 0.7, 1.0)
    return tuple(float(value) for value in material.diffuse_color)


def _world_scene_meshes(output: Path) -> list[SceneMeshInstance]:
    objects = [
        obj
        for obj in sorted(bpy.context.scene.objects, key=lambda item: item.name)
        if obj.type == "MESH" and not obj.hide_render and not _is_robot_library_object(obj)
    ]
    asset_directory = output.with_name(f"{output.stem}_assets")
    scene_meshes: list[SceneMeshInstance] = []
    for index, obj in enumerate(objects):
        source_path = asset_directory / f"mesh_{index:03d}.obj"
        if not _write_scene_obj([obj], source_path, world_space=False):
            continue
        position, quaternion, scale = obj.matrix_world.decompose()
        rigid_body = obj.rigid_body
        mass = float(rigid_body.mass) if rigid_body is not None else 1.0
        scene_meshes.append(
            SceneMeshInstance(
                obj.name,
                str(source_path),
                _scene_object_rgba(obj),
                position=tuple(float(value) for value in position),
                quaternion=tuple(float(value) for value in quaternion),
                scale=tuple(abs(float(value)) for value in scale),
                dynamic=_is_dynamic_scene_object(obj),
                mass=mass,
                collision_shape=rigid_body.collision_shape if rigid_body is not None else "MESH",
                collision_enabled=rigid_body is None or rigid_body.enabled or bool(obj.get("mujoco_dynamic", False)),
                friction=float(rigid_body.friction) if rigid_body is not None else None,
                restitution=float(rigid_body.restitution) if rigid_body is not None else 0.0,
                mount=mounting.export_mount(obj),
            )
        )
    return scene_meshes


def _export_usd_scene(output: Path) -> Path:
    usd_path = output.with_suffix(".usd")
    result = bpy.ops.wm.usd_export(
        filepath=str(usd_path),
        export_meshes=True,
        export_materials=True,
        export_lights=True,
        export_cameras=True,
    )
    if "FINISHED" not in result:
        raise RuntimeError("Blender could not export the USD scene")
    return usd_path


def _export_world_file(filepath: str) -> tuple[Path, Path, Path]:
    if live_physics.active():
        raise RuntimeError("Stop live MuJoCo physics before exporting or launching Play")
    output = Path(bpy.path.abspath(filepath)).expanduser().resolve()
    scene = bpy.context.scene
    executable = scene.robot_mujoco_python.strip() or shutil.which("python") or "python"
    exported = write_world_export(
        output, _world_robot_instances(), _world_scene_meshes(output), python_executable=executable,
        world_options=WorldOptions(gravity=tuple(scene.robot_mujoco_gravity), timestep=scene.robot_mujoco_timestep),
    )
    _export_usd_scene(output)
    return exported


def _behavior_joint(robot_id: str, joint_name: str) -> bpy.types.Object | None:
    collection = bpy.data.collections.get("Robot Library")
    if collection is None:
        return None
    for root in collection.objects:
        if root.get("robot_id") != robot_id:
            continue
        for joint in _joint_controls(root):
            if joint.name.removeprefix("Joint: ") == joint_name:
                return joint
    return None


def _run_behavior_graph(graph: BehaviorGraph) -> dict[str, object]:
    fallback_root = _control_root(bpy.context)

    def sensor_reader(data: dict[str, object]) -> object:
        sensor = str(data.get("sensor", ""))
        if sensor == "joint.qpos":
            joint = _joint_controls(fallback_root)[0] if fallback_root is not None and _joint_controls(fallback_root) else None
            return float(joint["qpos"]) if joint is not None else 0.0
        if sensor.startswith("object:"):
            object_name, _, property_name = sensor.removeprefix("object:").partition(".")
            obj = bpy.data.objects.get(object_name)
            return obj.get(property_name, 0.0) if obj is not None else 0.0
        return 0.0

    def ai_runner(data: dict[str, object], context: dict[str, object]) -> object:
        model = str(data.get("model", "rule_based"))
        if model != "rule_based":
            raise ValueError(f"AI model is not configured in Blender: {model}")
        return context.get("input")

    def action_sink(data: dict[str, object], value: object) -> None:
        joint = _behavior_joint(str(data.get("robot_id", "")), str(data.get("joint", "")))
        if joint is None:
            raise ValueError("Robot Action needs a robot and joint from the imported library")
        target = value
        if isinstance(value, dict):
            target = value.get("target", data.get("target", 0.0))
        if isinstance(target, (int, float)):
            joint["qpos"] = float(target)
            joint.update_tag(refresh={"OBJECT"})

    def application_runner(node: BehaviorNode, inputs: dict[str, object]) -> dict[str, object]:
        if node.handler:
            raise ValueError(
                f"Application node '{node.node_id}' declares an external handler "
                f"({node.handler}); running it is not implemented inside Blender yet"
            )
        first_input = next(iter(inputs.values()), None)
        return {port.name: first_input for port in node.outputs}

    values = graph.run(sensor_reader, ai_runner, action_sink, application_runner)
    bpy.context.scene.frame_set(bpy.context.scene.frame_current)
    bpy.context.view_layer.update()
    return values


def _selected_robot_root(obj: bpy.types.Object | None) -> bpy.types.Object | None:
    while obj is not None:
        if "robot_id" in obj:
            return obj
        obj = obj.parent
    return None


def _joint_controls(root: bpy.types.Object) -> list[bpy.types.Object]:
    controls: list[bpy.types.Object] = []
    for obj in bpy.data.objects:
        if "qpos" not in obj:
            continue
        if _selected_robot_root(obj) == root:
            controls.append(obj)
    return sorted(controls, key=lambda obj: int(obj.get("mujoco_joint_id", 0)))


def _control_root(context: bpy.types.Context) -> bpy.types.Object | None:
    selected_root = _selected_robot_root(context.active_object)
    if selected_root is not None and selected_root.get("import_mode") == "mujoco_compiled":
        return selected_root
    stored_root = context.scene.robot_library_control_root
    if stored_root is not None and stored_root.get("import_mode") == "mujoco_compiled":
        return stored_root
    return None


def _draw_joint_controls(layout: bpy.types.UILayout, root: bpy.types.Object, controls: list[bpy.types.Object]) -> None:
    layout = layout.column()
    running = live_physics.active()
    layout.label(text=root.name, icon="CONSTRAINT_BONE")
    if running:
        layout.label(text="Move a target to enable its effort-limited servo", icon="PHYSICS")
    for joint in controls:
        row = layout.row(align=True)
        info = live_physics.joint_info(joint) if running else None
        metadata, actual = info if info is not None else (None, None)
        row.enabled = not running or (live_physics.joint_controls_enabled() and metadata is not None and metadata["available"])
        row.prop(joint, '["qpos"]', text=joint.name.removeprefix("Joint: ") + (" target" if running else ""), slider=True)
        if running and actual is not None:
            row.label(text=f"Actual: {actual:.3f}")
        if running and metadata is not None and not metadata["available"]:
            layout.label(text=metadata["reason"], icon="INFO")
    action_row = layout.row(align=True)
    action_row.enabled = not running
    reset_operator = action_row.operator("robot_library.reset_pose", text="Reset", icon="LOOP_BACK")
    reset_operator.root_name = root.name
    keyframe_operator = action_row.operator("robot_library.keyframe_pose", text="Keyframe", icon="KEY_HLT")
    keyframe_operator.root_name = root.name


def _viewport_hit(event: bpy.types.Event) -> Vector | None:
    window = bpy.context.window
    if window is None or window.screen is None:
        return None
    for area in window.screen.areas:
        if area.type != "VIEW_3D":
            continue
        if not (area.x <= event.mouse_x < area.x + area.width and area.y <= event.mouse_y < area.y + area.height):
            continue
        region = next((region for region in area.regions if region.type == "WINDOW"), None)
        if region is None or area.spaces.active.region_3d is None:
            return None
        from bpy_extras import view3d_utils

        coordinate = (event.mouse_x - region.x, event.mouse_y - region.y)
        region_3d = area.spaces.active.region_3d
        origin = view3d_utils.region_2d_to_origin_3d(region, region_3d, coordinate)
        direction = view3d_utils.region_2d_to_vector_3d(region, region_3d, coordinate)
        if abs(direction.z) < 0.0001:
            return None
        distance = -origin.z / direction.z
        if distance < 0:
            return None
        return origin + direction * distance
    return None


class ROBOT_OT_refresh(Operator):
    bl_idname = "robot_library.refresh"
    bl_label = "Refresh Robot Library"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        _refresh_libraries(context.scene)
        return {"FINISHED"}


class ROBOT_OT_add(Operator):
    bl_idname = "robot_library.add_robot"
    bl_label = "Add Robot Arm"
    bl_options = {"REGISTER", "UNDO"}

    robot_index: IntProperty()

    def execute(self, context: bpy.types.Context) -> set[str]:
        items = context.scene.robot_library
        if self.robot_index < 0 or self.robot_index >= len(items):
            self.report({"WARNING"}, "Select a robot arm first")
            return {"CANCELLED"}
        item = items[self.robot_index]
        root = _place_robot(item, _add_location(context.scene))
        if root.get("import_mode") == "proxy_unavailable":
            self.report({"WARNING"}, "Fast Assembly needs MuJoCo in Blender; added a placeholder")
            return {"FINISHED"}
        self.report(
            {"INFO"},
            f"{item.name}: {root.get('joint_count', 0)} joints, {root.get('collision_count', 0)} collision geoms",
        )
        return {"FINISHED"}


class ROBOT_OT_drag(Operator):
    bl_idname = "robot_library.drag_robot"
    bl_label = "Drag Robot Arm Into Viewport"
    bl_options = {"REGISTER", "UNDO"}

    robot_index: IntProperty()
    moved: bool

    def invoke(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        if self.robot_index < 0 or self.robot_index >= len(context.scene.robot_library):
            self.report({"WARNING"}, "Select a robot arm first")
            return {"CANCELLED"}
        self.moved = False
        context.window_manager.modal_handler_add(self)
        context.workspace.status_text_set("Drag into the 3D viewport and release, or press Esc to cancel")
        return {"RUNNING_MODAL"}

    def modal(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        if event.type == "ESC":
            context.workspace.status_text_set(None)
            return {"CANCELLED"}
        if event.type == "MOUSEMOVE":
            self.moved = True
            return {"RUNNING_MODAL"}
        if event.type == "LEFTMOUSE" and event.value == "RELEASE":
            location = _viewport_hit(event) if self.moved else context.scene.cursor.location.copy()
            context.workspace.status_text_set(None)
            if location is None:
                self.report({"WARNING"}, "Release over a 3D viewport")
                return {"CANCELLED"}
            item = context.scene.robot_library[self.robot_index]
            root = _place_robot(item, location)
            if root.get("import_mode") == "proxy_unavailable":
                self.report({"WARNING"}, "Fast Assembly needs MuJoCo in Blender; added a placeholder")
                return {"FINISHED"}
            self.report(
                {"INFO"},
                f"{item.name}: {root.get('joint_count', 0)} joints, {root.get('collision_count', 0)} collision geoms",
            )
            return {"FINISHED"}
        return {"RUNNING_MODAL"}


class ROBOT_OT_clear_filter(Operator):
    bl_idname = "robot_library.clear_filter"
    bl_label = "Clear Robot Filter"

    def execute(self, context: bpy.types.Context) -> set[str]:
        context.scene.robot_library_filter = ""
        return {"FINISHED"}


class ROBOT_OT_add_asset_source(Operator):
    bl_idname = "robot_library.add_asset_source"
    bl_label = "Add Asset Source"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        item = context.scene.robot_asset_sources.add()
        item.path = str(Path.home() / "git_repos" / "sensor_menagerie")
        item.kind = "sensor"
        item.label = "Sensor Menagerie"
        return {"FINISHED"}


class ROBOT_OT_remove_asset_source(Operator):
    bl_idname = "robot_library.remove_asset_source"
    bl_label = "Remove Asset Source"
    bl_options = {"REGISTER", "UNDO"}

    source_index: IntProperty()

    def execute(self, context: bpy.types.Context) -> set[str]:
        sources = context.scene.robot_asset_sources
        if 0 <= self.source_index < len(sources):
            sources.remove(self.source_index)
            return {"FINISHED"}
        return {"CANCELLED"}


class ROBOT_OT_reset_pose(Operator):
    bl_idname = "robot_library.reset_pose"
    bl_label = "Reset Robot Pose"
    bl_options = {"REGISTER", "UNDO"}

    root_name: StringProperty()

    def execute(self, context: bpy.types.Context) -> set[str]:
        root = bpy.data.objects.get(self.root_name)
        if root is None:
            return {"CANCELLED"}
        for joint in _joint_controls(root):
            joint["qpos"] = joint["home_qpos"]
            if joint.animation_data is not None and joint.animation_data.action is not None:
                joint.keyframe_insert(data_path='["qpos"]', frame=context.scene.frame_current)
            joint.update_tag(refresh={"OBJECT"})
        context.scene.frame_set(context.scene.frame_current)
        context.view_layer.update()
        return {"FINISHED"}


class ROBOT_OT_keyframe_pose(Operator):
    bl_idname = "robot_library.keyframe_pose"
    bl_label = "Keyframe Robot Pose"
    bl_options = {"REGISTER", "UNDO"}

    root_name: StringProperty()

    def execute(self, context: bpy.types.Context) -> set[str]:
        root = bpy.data.objects.get(self.root_name)
        if root is None:
            return {"CANCELLED"}
        for joint in _joint_controls(root):
            joint.keyframe_insert(data_path='["qpos"]', frame=context.scene.frame_current)
        return {"FINISHED"}


class ROBOT_OT_export_world(Operator, ExportHelper):
    bl_idname = "robot_library.export_world"
    bl_label = "Export MuJoCo World"
    bl_options = {"REGISTER"}

    filename_ext = ".xml"
    filter_glob: StringProperty(default="*.xml", options={"HIDDEN"})

    def invoke(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        self.filepath = bpy.path.abspath("//caryam_world.xml")
        return ExportHelper.invoke(self, context, event)

    def execute(self, context: bpy.types.Context) -> set[str]:
        try:
            world_path, manifest_path, render_path = _export_world_file(self.filepath)
        except (OSError, RuntimeError, ValueError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Exported {world_path.name}, {manifest_path.name}, and {render_path.name}")
        return {"FINISHED"}


class ROBOT_OT_export_and_render(Operator, ExportHelper):
    bl_idname = "robot_library.export_and_render"
    bl_label = "Export and Render in MuJoCo"
    bl_options = {"REGISTER"}

    filename_ext = ".xml"
    filter_glob: StringProperty(default="*.xml", options={"HIDDEN"})

    def invoke(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        self.filepath = bpy.path.abspath("//caryam_world.xml")
        return ExportHelper.invoke(self, context, event)

    def execute(self, context: bpy.types.Context) -> set[str]:
        try:
            world_path, _manifest_path, render_path = _export_world_file(self.filepath)
            executable = context.scene.robot_mujoco_python.strip() or shutil.which("python") or "python"
            subprocess.Popen([executable, str(render_path)], cwd=str(render_path.parent), start_new_session=True)
        except (OSError, RuntimeError, ValueError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Exported and launched MuJoCo for {world_path.name}")
        return {"FINISHED"}


class ROBOT_OT_launch_mujoco(Operator):
    bl_idname = "robot_library.launch_mujoco"
    bl_label = "Launch MuJoCo Simulation"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        output_path = bpy.path.abspath("//caryam_world.xml")
        try:
            world_path, _manifest_path, render_path = _export_world_file(output_path)
            executable = context.scene.robot_mujoco_python.strip() or shutil.which("python") or "python"
            subprocess.Popen([executable, str(render_path)], cwd=str(render_path.parent), start_new_session=True)
        except (OSError, RuntimeError, ValueError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Launched MuJoCo simulation for {world_path.name}")
        return {"FINISHED"}


def _core_bridge_request(
    bridge_url: str,
    method: str,
    path: str,
    payload: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    request = urllib.request.Request(
        f"{bridge_url.rstrip('/')}{path}",
        data=data,
        method=method,
        headers={"Content-Type": "application/json"} if data is not None else {},
    )
    with urllib.request.urlopen(request, timeout=2.0) as response:
        value = json.loads(response.read().decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("Core bridge returned an invalid response")
    if "error" in value:
        raise RuntimeError(str(value["error"]))
    return value


def _update_core_status(scene: bpy.types.Scene, status: Mapping[str, Any]) -> None:
    endpoint = str(status.get("core_endpoint", f"{scene.robot_core_host}:{scene.robot_core_port}"))
    state = str(status.get("state", "unknown"))
    scene.robot_core_status = f"{state.title()} at {endpoint}"
    blackboard = status.get("blackboard", {})
    scene.robot_core_command_valid = bool(blackboard.get("caryam.control.command_valid", False)) if isinstance(blackboard, dict) else False
    logs = status.get("logs", [])
    scene.robot_core_log = "\n".join(str(entry) for entry in logs[-5:]) if isinstance(logs, list) else ""


class ROBOT_OT_core_status(Operator):
    bl_idname = "robot_library.core_status"
    bl_label = "Refresh Core Status"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        scene = context.scene
        try:
            _update_core_status(scene, _core_bridge_request(scene.robot_core_bridge_url, "GET", "/status"))
        except (OSError, RuntimeError, ValueError, urllib.error.URLError) as error:
            scene.robot_core_status = f"Bridge unavailable: {error}"
            scene.robot_core_command_valid = False
            return {"CANCELLED"}
        return {"FINISHED"}


class ROBOT_OT_run_core_simulation(Operator):
    bl_idname = "robot_library.run_core_simulation"
    bl_label = "Run in Core Simulation"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        scene = context.scene
        graph = _graph_from_tree(_ensure_behavior_tree(scene))
        errors = graph.validate()
        if errors:
            self.report({"ERROR"}, "; ".join(errors))
            return {"CANCELLED"}
        try:
            status = _core_bridge_request(
                scene.robot_core_bridge_url,
                "POST",
                "/run",
                {
                    "graph": graph.to_dict(),
                    "core_host": scene.robot_core_host,
                    "core_port": scene.robot_core_port,
                },
            )
            _update_core_status(scene, status)
        except (OSError, RuntimeError, ValueError, urllib.error.URLError) as error:
            scene.robot_core_status = f"Bridge unavailable: {error}"
            scene.robot_core_command_valid = False
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, "Core simulation flow started")
        return {"FINISHED"}


class ROBOT_OT_stop_core_simulation(Operator):
    bl_idname = "robot_library.stop_core_simulation"
    bl_label = "Stop Core Simulation"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        scene = context.scene
        try:
            _update_core_status(scene, _core_bridge_request(scene.robot_core_bridge_url, "POST", "/stop", {}))
        except (OSError, RuntimeError, ValueError, urllib.error.URLError) as error:
            scene.robot_core_status = f"Bridge unavailable: {error}"
            scene.robot_core_command_valid = False
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class ROBOT_OT_open_behavior_editor(Operator):
    bl_idname = "robot_library.open_behavior_editor"
    bl_label = "Open Data Graph"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _ensure_behavior_tree(context.scene)
        if context.area is not None:
            context.area.type = "NODE_EDITOR"
            space = context.area.spaces.active
            space.tree_type = "CARYAM_BEHAVIOR_TREE"
            space.pin = True
            space.path.start(tree)
        return {"FINISHED"}


class ROBOT_OT_new_behavior(Operator):
    bl_idname = "robot_library.new_behavior"
    bl_label = "Clear Data Graph"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _ensure_behavior_tree(context.scene)
        tree.links.clear()
        tree.nodes.clear()
        return {"FINISHED"}


class ROBOT_OT_add_behavior_node(Operator):
    bl_idname = "robot_library.add_behavior_node"
    bl_label = "Add Data Node"
    bl_options = {"REGISTER", "UNDO"}

    kind: EnumProperty(
        name="Kind",
        items=(
            ("start", "Start", "Begin a graph"),
            ("stop", "Stop", "Stop the current data flow"),
            ("wait", "Wait", "Wait for the next data-flow tick"),
            ("retry", "Retry", "Retry on a later data-flow tick"),
            ("success", "Success", "Mark the current control branch successful"),
            ("failure", "Failure", "Mark the current control branch failed"),
            ("sensor", "Virtual Sensor", ""),
            ("ai_model", "AI Model", ""),
            ("joint_controller", "Joint Controller", ""),
            ("joint_target", "Joint Target", ""),
            ("condition", "Condition", ""),
            ("action", "Robot Action", ""),
            ("application", "Application", ""),
            ("end", "End", "Finish a graph"),
        ),
        default="sensor",
    )
    preset: EnumProperty(
        name="Preset",
        items=(
            ("legacy", "Legacy", "Use the selected data node kind"),
            ("application", "Application", "A custom application node with editable ports"),
            ("virtual_camera", "Virtual RGB Camera", "A 480p RGB virtual sensor"),
            ("virtual_rgb_camera", "RGB Camera", "A 480p RGB virtual sensor"),
            ("virtual_depth_camera", "Virtual Depth Camera", "A 480p depth virtual sensor"),
            ("virtual_imu", "IMU", "An accelerometer and gyroscope sensor"),
            ("virtual_tactile_array", "Tactile Array", "A tactile mesh sensor"),
            ("virtual_lidar", "Virtual Lidar", "A 360-sample virtual sensor"),
            ("ai_model", "AI Model", "Consumes RGB and emits robot joint positions"),
            ("actuator_system", "N-Actuator System", "Consumes joint positions for the selected robot DOF"),
        ),
        default="legacy",
        options={"HIDDEN"},
    )

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
        if self.preset == "legacy":
            node.kind = self.kind
            node.node_id = f"{self.kind}_{len(tree.nodes)}"
            node.label = node.draw_label()
        else:
            _configure_flow_preset(node, self.preset, _flow_dof(context))
            node.node_id = f"{self.preset}_{len(tree.nodes)}"
        cursor = getattr(getattr(context, "space_data", None), "cursor_location", None)
        node.location = cursor if cursor is not None else (len(tree.nodes) * 180.0, 0.0)
        return {"FINISHED"}


class ROBOT_OT_add_catalog_node(Operator):
    bl_idname = "robot_library.add_catalog_node"
    bl_label = "Add Catalog Node"
    bl_options = {"REGISTER", "UNDO"}

    template: EnumProperty(
        name="Template",
        items=(
            ("robot_state", "Robot State", "Robot joint state outputs"),
            ("robot_command", "Robot Command", "Robot joint command input"),
            ("selected_sensor", "Selected Sensor", "Ports from the selected sensor definition"),
        ),
        default="selected_sensor",
    )
    sensor_index: IntProperty(default=-1)

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        node_id = f"{self.template}_{len(tree.nodes) + 1}"
        if self.template == "selected_sensor":
            index = self.sensor_index if self.sensor_index >= 0 else context.scene.sensor_library_index
            if not 0 <= index < len(context.scene.sensor_library):
                self.report({"WARNING"}, "Select a catalog sensor first")
                return {"CANCELLED"}
            item = context.scene.sensor_library[index]
            definition = next(
                (sensor for sensor in discover_sensor_definitions(item.source_path) if sensor.sensor_id == item.sensor_id),
                None,
            )
            if definition is None:
                self.report({"ERROR"}, "Sensor definition is no longer available")
                return {"CANCELLED"}
            entry = definition.to_dict()
            node_id = f"{item.sensor_id}_{len(tree.nodes) + 1}"
        else:
            entry = {"category": self.template, "name": self.template.replace("_", " ").title(), "dof": _flow_dof(context)}
        template = catalog_node_template(node_id, entry)
        node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
        node.node_id = template.node_id
        node.label = template.label
        _set_data_ports(node, "application", template.inputs, template.outputs)
        node.catalog_locked = True
        node.catalog_data_json = json.dumps(template.data)
        node.location = (len(tree.nodes) * 180.0, 0.0)
        return {"FINISHED"}


def _draw_behavior_add_menu(menu: bpy.types.Menu, context: bpy.types.Context) -> None:
    space_data = getattr(context, "space_data", None)
    if getattr(space_data, "tree_type", "") != "CARYAM_BEHAVIOR_TREE":
        return
    layout = menu.layout
    layout.separator()
    layout.label(text="Caryam Flow")
    for preset, label, icon in (
        ("application", "Application", "SCRIPT"),
        ("virtual_rgb_camera", "RGB Camera", "OUTLINER_OB_CAMERA"),
        ("virtual_depth_camera", "Depth Camera", "CAMERA_DATA"),
        ("virtual_lidar", "Virtual Lidar", "FORCE_TURBULENCE"),
        ("virtual_imu", "IMU", "EMPTY_ARROWS"),
        ("virtual_tactile_array", "Tactile Array", "MOD_SOFT"),
        ("ai_model", "AI Model", "LIGHT_DATA"),
        ("actuator_system", "N-Actuator System", "CONSTRAINT_BONE"),
        ("start", "Start", "PLAY"),
        ("stop", "Stop", "PAUSE"),
        ("wait", "Wait", "TIME"),
        ("retry", "Retry", "FILE_REFRESH"),
        ("success", "Success", "CHECKMARK"),
        ("failure", "Failure", "CANCEL"),
    ):
        operator = layout.operator("robot_library.add_behavior_node", text=label, icon=icon)
        operator.preset = preset


class ROBOT_OT_add_port(Operator):
    bl_idname = "robot_library.add_behavior_port"
    bl_label = "Add Behavior Port"
    bl_options = {"REGISTER", "UNDO"}

    direction: EnumProperty(
        items=(("input", "Input", ""), ("output", "Output", "")),
        default="input",
    )

    def execute(self, context: bpy.types.Context) -> set[str]:
        node = context.active_node
        if not isinstance(node, CaryamBehaviorNode) or node.kind not in PORT_CONFIGURABLE_NODE_KINDS:
            self.report({"WARNING"}, "Select an Application node first")
            return {"CANCELLED"}
        if node.catalog_locked:
            self.report({"WARNING"}, "Catalog ports are read-only")
            return {"CANCELLED"}
        ports = node.input_ports if self.direction == "input" else node.output_ports
        entry = ports.add()
        entry.port_name = f"{self.direction}_{len(ports)}"
        node.sync_sockets()
        return {"FINISHED"}


class ROBOT_OT_remove_port(Operator):
    bl_idname = "robot_library.remove_behavior_port"
    bl_label = "Remove Behavior Port"
    bl_options = {"REGISTER", "UNDO"}

    direction: EnumProperty(
        items=(("input", "Input", ""), ("output", "Output", "")),
        default="input",
    )
    port_index: IntProperty()

    def execute(self, context: bpy.types.Context) -> set[str]:
        node = context.active_node
        if not isinstance(node, CaryamBehaviorNode) or node.kind not in PORT_CONFIGURABLE_NODE_KINDS:
            return {"CANCELLED"}
        if node.catalog_locked:
            return {"CANCELLED"}
        ports = node.input_ports if self.direction == "input" else node.output_ports
        if 0 <= self.port_index < len(ports):
            ports.remove(self.port_index)
        node.sync_sockets()
        return {"FINISHED"}


class ROBOT_OT_delete_data_node(Operator):
    bl_idname = "robot_library.delete_data_node"
    bl_label = "Delete Data Node"
    bl_options = {"REGISTER", "UNDO"}

    node_name: StringProperty(options={"HIDDEN"})

    def invoke(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        node = tree.nodes.get(self.node_name)
        if node is None:
            self.report({"WARNING"}, "Data node no longer exists")
            return {"CANCELLED"}
        tree.nodes.remove(node)
        return {"FINISHED"}


class ROBOT_OT_save_behavior(Operator, ExportHelper):
    bl_idname = "robot_library.save_behavior"
    bl_label = "Save Data Graph"
    bl_options = {"REGISTER"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json;*.yaml;*.yml", options={"HIDDEN"})

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        graph = _graph_from_tree(tree)
        errors = graph.validate()
        if errors:
            self.report({"ERROR"}, "; ".join(errors))
            return {"CANCELLED"}
        path = Path(self.filepath)
        try:
            if path.suffix.lower() in {".yaml", ".yml"}:
                graph.save_flow_yaml(path)
            else:
                graph.save(path)
        except (OSError, RuntimeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Saved {path.name}")
        return {"FINISHED"}


class ROBOT_OT_load_behavior(Operator, ImportHelper):
    bl_idname = "robot_library.load_behavior"
    bl_label = "Load Data Graph"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json;*.yaml;*.yml", options={"HIDDEN"})

    def execute(self, context: bpy.types.Context) -> set[str]:
        try:
            path = Path(self.filepath)
            graph = BehaviorGraph.load_flow_yaml(path) if path.suffix.lower() in {".yaml", ".yml"} else BehaviorGraph.load(path)
        except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        _populate_behavior_tree(tree, graph)
        return {"FINISHED"}


class ROBOT_OT_run_behavior(Operator):
    bl_idname = "robot_library.run_behavior"
    bl_label = "Run Data Graph"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        try:
            _run_behavior_graph(_graph_from_tree(tree))
        except (ValueError, KeyError, TypeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, "Data graph executed")
        return {"FINISHED"}


class ROBOT_OT_joint_window(Operator):
    bl_idname = "robot_library.joint_window"
    bl_label = "Robot Joint Controller"

    root_name: StringProperty()

    def invoke(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        root = _control_root(context)
        if root is None:
            self.report({"WARNING"}, "Import or select a compiled MuJoCo robot first")
            return {"CANCELLED"}
        self.root_name = root.name
        return context.window_manager.invoke_props_dialog(self, width=520)

    def draw(self, context: bpy.types.Context) -> None:
        root = bpy.data.objects.get(self.root_name)
        if root is None:
            self.layout.label(text="Robot is no longer available", icon="ERROR")
            return
        _draw_joint_controls(self.layout, root, _joint_controls(root))

    def execute(self, context: bpy.types.Context) -> set[str]:
        return {"FINISHED"}


class ROBOT_OT_attach_sensor(Operator):
    bl_idname = "robot_library.attach_sensor"
    bl_label = "Attach Sensor to Body"
    bl_options = {"REGISTER", "UNDO"}

    sensor_index: IntProperty(default=-1)

    def execute(self, context: bpy.types.Context) -> set[str]:
        scene = context.scene
        index = self.sensor_index if self.sensor_index >= 0 else scene.sensor_library_index
        if not 0 <= index < len(scene.sensor_library):
            self.report({"WARNING"}, "Select a sensor first")
            return {"CANCELLED"}
        target = context.active_object
        robot_root = _selected_robot_root(target)
        if target is None or robot_root is None:
            self.report({"WARNING"}, "Select an imported robot body first")
            return {"CANCELLED"}
        if target is robot_root:
            top_bodies = [child for child in robot_root.children if child.name.startswith("Body: ")]
            if len(top_bodies) == 1:
                target = top_bodies[0]
            elif len(top_bodies) > 1:
                self.report({"WARNING"}, "Select a specific robot body")
                return {"CANCELLED"}
        item = scene.sensor_library[index]
        definition = next(
            (sensor for sensor in discover_sensor_definitions(item.source_path) if sensor.sensor_id == item.sensor_id),
            None,
        )
        if definition is None:
            self.report({"ERROR"}, "Sensor definition is no longer available")
            return {"CANCELLED"}
        mount = bpy.data.objects.new(f"Sensor: {item.name}", None)
        mount.empty_display_type = "SPHERE"
        mount.empty_display_size = 0.03
        mount["sensor_id"] = item.sensor_id
        mount["sensor_category"] = item.category
        mount["sensor_source_path"] = item.source_path
        mount["sensor_definition_json"] = json.dumps(definition.to_dict())
        _robot_collection().objects.link(mount)
        mount.parent = target
        mount.matrix_parent_inverse = Matrix.Identity(4)
        mount.location = (0.0, 0.0, 0.0)
        mount.rotation_euler = (0.0, 0.0, 0.0)
        return {"FINISHED"}


class ROBOT_OT_ros_list_topics(Operator):
    bl_idname = "robot_library.ros_list_topics"
    bl_label = "List ROS Topics"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import ros_bridge
        try:
            topics = ros_bridge.list_topics(context.scene.robot_ros_distro)
        except (ros_bridge.RosCommandError, ValueError) as error:
            context.scene.robot_ros_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        context.scene.robot_ros_status = "; ".join(topics) if topics else "No topics found"
        return {"FINISHED"}


class ROBOT_OT_ros_echo_topic(Operator):
    bl_idname = "robot_library.ros_echo_topic"
    bl_label = "Echo ROS Topic Once"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import ros_bridge
        scene = context.scene
        try:
            scene.robot_ros_status = ros_bridge.echo_topic(scene.robot_ros_distro, scene.robot_ros_topic)
        except (ros_bridge.RosCommandError, ValueError) as error:
            scene.robot_ros_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


class ROBOT_OT_ros_publish_topic(Operator):
    bl_idname = "robot_library.ros_publish_topic"
    bl_label = "Publish ROS Topic Once"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import ros_bridge
        scene = context.scene
        try:
            ros_bridge.publish_topic(
                scene.robot_ros_distro, scene.robot_ros_topic,
                scene.robot_ros_message_type, scene.robot_ros_payload,
            )
        except (ros_bridge.RosCommandError, ValueError) as error:
            scene.robot_ros_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        scene.robot_ros_status = f"Published to {scene.robot_ros_topic}"
        return {"FINISHED"}


class ROBOT_OT_ros_launch_rviz(Operator):
    bl_idname = "robot_library.ros_launch_rviz"
    bl_label = "Launch RViz"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import ros_bridge
        try:
            ros_bridge.launch_rviz(context.scene.robot_ros_distro)
        except (ros_bridge.RosCommandError, ValueError) as error:
            context.scene.robot_ros_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        context.scene.robot_ros_status = "RViz launched"
        return {"FINISHED"}


class ROBOT_OT_generate_containerfile(Operator):
    bl_idname = "robot_library.generate_containerfile"
    bl_label = "Generate Containerfile"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import container_builder
        scene = context.scene
        try:
            content = container_builder.render_containerfile(
                scene.robot_build_base_image, [], "/opt/caryam/entrypoint.sh",
            )
            build_dir = _blend_directory() / "build"
            path = container_builder.write_containerfile(build_dir, content)
            payload = build_dir / "caryam_payload"
            payload.mkdir(exist_ok=True)
            entrypoint = payload / "entrypoint.sh"
            if not entrypoint.exists():
                entrypoint.write_text("#!/bin/sh\nexec /sbin/init \"$@\"\n", encoding="utf-8")
            entrypoint.chmod(0o755)
        except (OSError, ValueError) as error:
            scene.robot_build_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        scene.robot_build_status = f"Wrote {path}"
        return {"FINISHED"}


class ROBOT_OT_build_container_image(Operator):
    bl_idname = "robot_library.build_container_image"
    bl_label = "Build Container Image"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import container_builder
        scene = context.scene
        build_dir = _blend_directory() / "build"
        containerfile = build_dir / "Containerfile"
        if not containerfile.is_file():
            self.report({"ERROR"}, "Generate the Containerfile first")
            return {"CANCELLED"}
        try:
            tag = container_builder.image_tag(scene.robot_build_oci_name)
            command = container_builder.build_image_command(scene.robot_build_tool, tag, containerfile, build_dir)
            subprocess.Popen(command, cwd=str(build_dir), start_new_session=True)
        except (OSError, ValueError) as error:
            scene.robot_build_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        scene.robot_build_status = f"Building {tag}"
        return {"FINISHED"}


class ROBOT_OT_flash_bootc_image(Operator):
    bl_idname = "robot_library.flash_bootc_image"
    bl_label = "Flash bootc Image to Disk"
    bl_options = {"REGISTER"}
    confirmed: BoolProperty(default=False, options={"HIDDEN"})

    def invoke(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        self.confirmed = True
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context: bpy.types.Context) -> set[str]:
        if not self.confirmed:
            self.report({"WARNING"}, "Confirm the target disk in the dialog before flashing")
            return {"CANCELLED"}
        from . import container_builder
        scene = context.scene
        try:
            tag = container_builder.image_tag(scene.robot_build_oci_name)
            command = container_builder.bootc_install_command(scene.robot_build_tool, tag, scene.robot_build_target_disk)
            subprocess.Popen(command, start_new_session=True)
        except (OSError, ValueError) as error:
            scene.robot_build_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        scene.robot_build_status = f"Flashing {tag} to {scene.robot_build_target_disk}"
        return {"FINISHED"}


class ROBOT_UL_library(UIList):
    def draw_item(
        self,
        context: bpy.types.Context,
        layout: bpy.types.UILayout,
        data: object,
        item: RobotLibraryItem,
        icon: int,
        active_data: object,
        active_propname: str,
        index: int,
        flt_flag: int = 0,
    ) -> None:
        layout.label(text=f"{index + 1:03d}  {item.name}", icon="OUTLINER_OB_ARMATURE")
        if item.instance_count:
            layout.label(text=str(item.instance_count))

    def filter_items(self, context: bpy.types.Context, data: object, property_name: str):
        items = getattr(data, property_name)
        query = context.scene.robot_library_filter.strip().lower()
        flags = [
            self.bitflag_filter_item
            if not query or query in f"{item.name} {item.category}".lower()
            else 0
            for item in items
        ]
        return flags, []


class ROBOT_PT_library(Panel):
    bl_label = "Robot Library"
    bl_idname = "ROBOT_PT_library"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Library"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        library = getattr(scene, "robot_library", None)
        if library is None:
            layout.label(text="Robot Library is not registered", icon="ERROR")
            layout.label(text="Disable and re-enable the add-on")
            return
        path_row = layout.row(align=True)
        path_row.prop(scene, "robot_setup_path", text="Setup")
        path_row.operator("robot_library.refresh", text="", icon="FILE_REFRESH")
        filter_row = layout.row(align=True)
        filter_row.prop(scene, "robot_library_filter", text="", icon="VIEWZOOM")
        filter_row.operator("robot_library.clear_filter", text="", icon="X")
        sources_box = layout.box()
        sources_box.label(text="Additional Asset Sources")
        for index, source in enumerate(scene.robot_asset_sources):
            row = sources_box.row(align=True)
            row.prop(source, "enabled", text="")
            row.prop(source, "path", text="")
            row.prop(source, "kind", text="")
            remove = row.operator("robot_library.remove_asset_source", text="", icon="X")
            remove.source_index = index
        sources_box.operator("robot_library.add_asset_source", text="Add Source", icon="ADD")
        visible_count = sum(
            not scene.robot_library_filter.strip().lower()
            or scene.robot_library_filter.strip().lower() in f"{item.name} {item.category}".lower()
            for item in scene.robot_library
        )
        layout.label(text=f"{visible_count} visible / {len(scene.robot_library)} detected")
        layout.label(text=scene.robot_library_status)

        layout.template_list(
            "ROBOT_UL_library",
            "robot_arm_library",
            scene,
            "robot_library",
            scene,
            "robot_library_index",
            rows=4,
        )
        if not scene.robot_library:
            layout.label(text="No robots available", icon="INFO")
            layout.operator("robot_library.refresh", text="Refresh Robot Library", icon="FILE_REFRESH")
            return

        item = scene.robot_library[scene.robot_library_index]
        layout.label(text=item.model_path or "No model path; placeholder will be used", icon="FILE")
        if item.category:
            layout.label(text=item.category.replace("_", " ").title(), icon="OUTLINER_OB_ARMATURE")
        options_box = layout.box()
        options_box.label(text="Import Options")
        options_box.prop(scene, "robot_library_auto_spread", text="Auto-space Add")
        options_box.prop(scene, "robot_library_include_collisions", text="Load Collision Geoms")
        options_box.prop(scene, "robot_library_preview_mode", text="Robot Geometry")
        if scene.robot_library_preview_mode == "DETAILED":
            options_box.prop(scene, "robot_library_mesh_detail", text="Mesh Detail")
        add_row = layout.row(align=True)
        add_operator = add_row.operator("robot_library.add_robot", text="Add to World", icon="ADD")
        add_operator.robot_index = scene.robot_library_index
        drag_operator = add_row.operator("robot_library.drag_robot", text="Drag into Viewport", icon="HAND")
        drag_operator.robot_index = scene.robot_library_index
        layout.operator("robot_library.joint_window", text="Open Joint Controller", icon="CONSTRAINT_BONE")


class ROBOT_UL_sensor_library(UIList):
    def draw_item(
        self, context: bpy.types.Context, layout: bpy.types.UILayout, data: object,
        item: SensorLibraryItem, icon: int, active_data: object, active_propname: str,
        index: int, flt_flag: int = 0,
    ) -> None:
        icon_name = "OUTLINER_OB_CAMERA" if item.category == "depth_camera" else "OUTLINER_OB_FORCE_FIELD"
        layout.label(text=f"{index + 1:03d}  {item.name}", icon=icon_name)


class ROBOT_PT_sensor_library(Panel):
    bl_label = "Sensor Library"
    bl_idname = "ROBOT_PT_sensor_library"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Library"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        if not scene.sensor_library:
            layout.label(text="No sensors discovered", icon="INFO")
            layout.label(text="Add a sensor source and refresh", icon="INFO")
            return
        layout.template_list(
            "ROBOT_UL_sensor_library", "sensor_library", scene, "sensor_library",
            scene, "sensor_library_index", rows=4,
        )
        index = min(scene.sensor_library_index, len(scene.sensor_library) - 1)
        item = scene.sensor_library[index]
        layout.label(text=item.category.replace("_", " ").title())
        attach = layout.operator("robot_library.attach_sensor", text="Attach to Selected Body", icon="CONSTRAINT_BONE")
        attach.sensor_index = index


class ROBOT_PT_joint_controls(Panel):
    bl_label = "Robot Joints"
    bl_idname = "ROBOT_PT_joint_controls"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Control"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        robot_root = _control_root(context)
        if robot_root is None:
            layout.label(text="Import or select a robot", icon="INFO")
            return
        controls = _joint_controls(robot_root)
        if not controls:
            layout.label(text="No hinge or slide joints", icon="INFO")
            return
        _draw_joint_controls(layout, robot_root, controls)


class ROBOT_PT_mujoco_world(Panel):
    bl_label = "MuJoCo"
    bl_idname = "ROBOT_PT_mujoco_world"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Sim"
    bl_parent_id = "ROBOT_PT_simulation"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        row = layout.row(align=True)
        row.enabled = not live_physics.active()
        row.operator("robot_library.export_world", text="Export", icon="EXPORT")
        row.operator("robot_library.launch_mujoco", text="Play", icon="PLAY")
        live_physics.draw(layout, scene)
        obj = context.active_object
        if obj is None:
            layout.label(text="Select a mesh to inspect physics", icon="INFO")
            return
        if _is_robot_library_object(obj):
            layout.label(text="Robot physics comes from its MJCF", icon="INFO")
        elif obj.type != "MESH":
            layout.label(text="Selected item is not a mesh", icon="INFO")
        elif obj.hide_render:
            layout.label(text="Selected mesh is excluded from export", icon="ERROR")
        elif obj.rigid_body is not None and not obj.rigid_body.enabled and not obj.get("mujoco_dynamic", False):
            layout.label(text="Selected mesh: Dynamic is off", icon="INFO")
        elif obj.rigid_body is not None and obj.rigid_body.kinematic and not obj.get("mujoco_dynamic", False):
            layout.label(text="Selected mesh: Animated is on", icon="INFO")
        elif _is_dynamic_scene_object(obj):
            layout.label(text="Selected mesh: dynamic in MuJoCo", icon="CHECKMARK")
        else:
            layout.label(text="Selected mesh: fixed collider", icon="INFO")


class ROBOT_PT_core_simulation(Panel):
    bl_label = "Intrinsic Core"
    bl_idname = "ROBOT_PT_core_simulation"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Sim"
    bl_parent_id = "ROBOT_PT_simulation"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        layout.prop(scene, "robot_core_bridge_url", text="Bridge")
        endpoint_row = layout.row(align=True)
        endpoint_row.prop(scene, "robot_core_host", text="Host")
        endpoint_row.prop(scene, "robot_core_port", text="Port")
        controls = layout.row(align=True)
        controls.operator("robot_library.core_status", text="", icon="FILE_REFRESH")
        controls.operator("robot_library.run_core_simulation", text="Run", icon="PLAY")
        controls.operator("robot_library.stop_core_simulation", text="", icon="PAUSE")
        layout.label(text=scene.robot_core_status, icon="INFO")
        command_row = layout.row()
        command_row.enabled = False
        command_row.prop(scene, "robot_core_command_valid", text="Command Valid")
        if scene.robot_core_log:
            log_box = layout.box()
            for line in scene.robot_core_log.splitlines():
                log_box.label(text=line, icon="CONSOLE")


class ROBOT_PT_gazebo_world(Panel):
    bl_label = "Gazebo"
    bl_idname = "ROBOT_PT_gazebo_world"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Sim"
    bl_parent_id = "ROBOT_PT_simulation"

    def draw(self, context: bpy.types.Context) -> None:
        self.layout.label(text="Gazebo support is planned", icon="INFO")


class ROBOT_PT_simulation(Panel):
    bl_label = "Sim"
    bl_idname = "ROBOT_PT_simulation"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Sim"

    def draw(self, context: bpy.types.Context) -> None:
        self.layout.label(text="Choose a simulator", icon="WORLD")


class ROBOT_PT_behavior_graph(Panel):
    bl_label = "Robot Data Graph"
    bl_idname = "ROBOT_PT_behavior_graph"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Control"

    def draw(self, context: bpy.types.Context) -> None:
        self.layout.operator("robot_library.open_behavior_editor", text="Open Data Graph", icon="NODETREE")


class ROBOT_PT_rcs_control(Panel):
    bl_label = "ROS Control"
    bl_idname = "ROBOT_PT_rcs_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Control"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        layout.prop(scene, "robot_ros_distro", text="ROS")
        layout.operator("robot_library.ros_launch_rviz", text="Launch RViz", icon="RESTRICT_VIEW_OFF")
        layout.operator("robot_library.ros_list_topics", text="List Topics", icon="FILE_REFRESH")
        layout.prop(scene, "robot_ros_topic", text="Topic")
        layout.operator("robot_library.ros_echo_topic", text="Echo Once", icon="PLAY")
        layout.prop(scene, "robot_ros_message_type", text="Type")
        layout.prop(scene, "robot_ros_payload", text="Payload")
        layout.operator("robot_library.ros_publish_topic", text="Publish Once", icon="EXPORT")
        layout.label(text=scene.robot_ros_status)


class ROBOT_PT_robot_build(Panel):
    bl_label = "Container Build"
    bl_idname = "ROBOT_PT_robot_build"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Build"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        layout.prop(scene, "robot_build_oci_name", text="Image Name")
        layout.prop(scene, "robot_build_base_image", text="Base Image")
        layout.prop(scene, "robot_build_tool", text="Tool")
        layout.operator("robot_library.generate_containerfile", text="Generate Containerfile", icon="FILE_NEW")
        layout.operator("robot_library.build_container_image", text="Build Image", icon="PACKAGE")
        layout.separator()
        layout.prop(scene, "robot_build_target_disk", text="Target Disk")
        layout.operator("robot_library.flash_bootc_image", text="Flash to Disk", icon="EXPORT")
        layout.label(text=scene.robot_build_status)


class ROBOT_PT_behavior_editor(Panel):
    bl_label = "Caryam Data Graph"
    bl_idname = "ROBOT_PT_behavior_editor"
    bl_space_type = "NODE_EDITOR"
    bl_region_type = "UI"
    bl_category = "Data Graph"

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return context.space_data.tree_type == "CARYAM_BEHAVIOR_TREE"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        layout.label(text=f"{len(tree.nodes)} nodes, {len(tree.links)} links", icon="NODETREE")
        layout.operator("robot_library.new_behavior", text="Clear Canvas", icon="TRASH")
        layout.label(text="Add Flow Node")
        for preset, label, icon in (
            ("application", "Application", "SCRIPT"),
            ("virtual_rgb_camera", "RGB Camera", "OUTLINER_OB_CAMERA"),
            ("virtual_depth_camera", "Depth Camera", "CAMERA_DATA"),
            ("virtual_lidar", "Virtual Lidar", "FORCE_TURBULENCE"),
            ("virtual_imu", "IMU", "EMPTY_ARROWS"),
            ("virtual_tactile_array", "Tactile Array", "MOD_SOFT"),
            ("ai_model", "AI Model", "LIGHT_DATA"),
            ("actuator_system", "N-Actuator System", "CONSTRAINT_BONE"),
        ):
            operator = layout.operator("robot_library.add_behavior_node", text=label, icon=icon)
            operator.preset = preset
        layout.label(text="Catalog Nodes")
        for template, label in (("robot_state", "Robot State"), ("robot_command", "Robot Command")):
            operator = layout.operator("robot_library.add_catalog_node", text=label, icon="NODE")
            operator.template = template
        if context.scene.sensor_library:
            operator = layout.operator("robot_library.add_catalog_node", text="Selected Sensor", icon="OUTLINER_OB_CAMERA")
            operator.template = "selected_sensor"
        layout.separator()
        layout.label(text="Add Control Node")
        for kind, label, icon in (
            ("start", "Start", "PLAY"),
            ("stop", "Stop", "PAUSE"),
            ("wait", "Wait", "TIME"),
            ("retry", "Retry", "FILE_REFRESH"),
            ("success", "Success", "CHECKMARK"),
            ("failure", "Failure", "CANCEL"),
        ):
            operator = layout.operator("robot_library.add_behavior_node", text=label, icon=icon)
            operator.kind = kind
        layout.separator()
        file_row = layout.row(align=True)
        file_row.operator("robot_library.load_behavior", text="Load", icon="IMPORT")
        file_row.operator("robot_library.save_behavior", text="Save", icon="EXPORT")
        layout.operator("robot_library.run_behavior", text="Run Graph", icon="PLAY")


CLASSES = (
    RobotLibraryItem,
    SensorLibraryItem,
    RobotAssetSource,
    CaryamBehaviorTree,
    CaryamPortSpec,
    CaryamBehaviorNode,
    ROBOT_OT_refresh,
    ROBOT_OT_add,
    ROBOT_OT_drag,
    ROBOT_OT_clear_filter,
    ROBOT_OT_add_asset_source,
    ROBOT_OT_remove_asset_source,
    ROBOT_OT_reset_pose,
    ROBOT_OT_keyframe_pose,
    ROBOT_OT_export_world,
    ROBOT_OT_export_and_render,
    ROBOT_OT_launch_mujoco,
    ROBOT_OT_core_status,
    ROBOT_OT_run_core_simulation,
    ROBOT_OT_stop_core_simulation,
    ROBOT_OT_open_behavior_editor,
    ROBOT_OT_new_behavior,
    ROBOT_OT_add_behavior_node,
    ROBOT_OT_add_catalog_node,
    ROBOT_OT_add_port,
    ROBOT_OT_remove_port,
    ROBOT_OT_delete_data_node,
    ROBOT_OT_save_behavior,
    ROBOT_OT_load_behavior,
    ROBOT_OT_run_behavior,
    ROBOT_OT_joint_window,
    ROBOT_OT_attach_sensor,
    ROBOT_OT_ros_list_topics,
    ROBOT_OT_ros_echo_topic,
    ROBOT_OT_ros_publish_topic,
    ROBOT_OT_ros_launch_rviz,
    ROBOT_OT_generate_containerfile,
    ROBOT_OT_build_container_image,
    ROBOT_OT_flash_bootc_image,
    ROBOT_UL_library,
    ROBOT_UL_sensor_library,
    ROBOT_PT_library,
    ROBOT_PT_sensor_library,
    ROBOT_PT_joint_controls,
    ROBOT_PT_simulation,
    ROBOT_PT_mujoco_world,
    ROBOT_PT_core_simulation,
    ROBOT_PT_gazebo_world,
    ROBOT_PT_behavior_graph,
    ROBOT_PT_rcs_control,
    ROBOT_PT_robot_build,
    ROBOT_PT_behavior_editor,
    *mounting.MOUNT_CLASSES,
    *live_physics.CLASSES,
)


def _guard_authoring_operator(cls) -> None:
    original = getattr(cls, "poll", None)

    def poll(operator_cls, context):
        return not live_physics.active() and (original(context) if original is not None else True)

    cls.poll = classmethod(poll)


for _operator in CLASSES:
    if issubclass(_operator, Operator) and _operator not in live_physics.CLASSES and _operator is not ROBOT_OT_joint_window:
        _guard_authoring_operator(_operator)


def _remove_scene_properties() -> None:
    mounting.unregister_properties()
    for name in (
        "robot_library",
        "robot_library_index",
        "robot_library_status",
        "robot_library_filter",
        "robot_setup_path",
        "robot_asset_sources",
        "sensor_library",
        "sensor_library_index",
        "robot_library_auto_spread",
        "robot_library_include_collisions",
        "robot_library_preview_mode",
        "robot_library_mesh_detail",
        "robot_library_control_root",
        "robot_mount_distance",
        "robot_magnet_alignment",
        "robot_mujoco_python",
        "robot_mujoco_gravity",
        "robot_mujoco_timestep",
        "robot_mujoco_rate",
        "robot_mujoco_servo_kp",
        "robot_mujoco_servo_kd",
        "robot_mujoco_servo_effort",
        "robot_core_bridge_url",
        "robot_core_host",
        "robot_core_port",
        "robot_core_status",
        "robot_core_log",
        "robot_core_command_valid",
        "robot_behavior_tree",
        "robot_ros_distro",
        "robot_ros_topic",
        "robot_ros_message_type",
        "robot_ros_payload",
        "robot_ros_status",
        "robot_build_oci_name",
        "robot_build_base_image",
        "robot_build_tool",
        "robot_build_target_disk",
        "robot_build_status",
    ):
        if hasattr(bpy.types.Scene, name):
            delattr(bpy.types.Scene, name)


def _unregister_existing_classes() -> None:
    for cls in reversed(CLASSES):
        registered_classes = [cls]
        registered_class = getattr(bpy.types, cls.__name__, None)
        if registered_class is not None:
            registered_classes.append(registered_class)
        for candidate in registered_classes:
            try:
                bpy.utils.unregister_class(candidate)
            except RuntimeError:
                continue


def register() -> None:
    live_physics.unregister_handlers()
    _remove_scene_properties()
    _unregister_existing_classes()
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.robot_setup_path = StringProperty(name="Robot setup", default=DEFAULT_SETUP_PATH)
    bpy.types.Scene.robot_library_filter = StringProperty(name="Filter")
    bpy.types.Scene.robot_library_status = StringProperty(name="Status", default="Refresh to discover robot arms")
    bpy.types.Scene.robot_library_index = IntProperty(name="Robot", default=0, min=0)
    bpy.types.Scene.robot_library = CollectionProperty(type=RobotLibraryItem)
    bpy.types.Scene.robot_asset_sources = CollectionProperty(type=RobotAssetSource)
    bpy.types.Scene.sensor_library = CollectionProperty(type=SensorLibraryItem)
    bpy.types.Scene.sensor_library_index = IntProperty(name="Sensor", default=0, min=0)
    bpy.types.Scene.robot_library_auto_spread = BoolProperty(name="Auto-space Add", default=True)
    bpy.types.Scene.robot_library_include_collisions = BoolProperty(name="Load Collision Geoms", default=False)
    bpy.types.Scene.robot_library_preview_mode = EnumProperty(
        name="Robot Geometry",
        items=(
            ("PROXY", "Fast Assembly", "Use small bounding boxes for robot geometry in Blender"),
            ("DETAILED", "Detailed Mesh", "Load the robot's visual meshes into Blender"),
        ),
        default="PROXY",
    )
    bpy.types.Scene.robot_library_mesh_detail = FloatProperty(
        name="Mesh Detail", description="Fraction of source triangles kept in Detailed Mesh mode",
        default=1.0, min=0.01, max=1.0,
    )
    bpy.types.Scene.robot_library_control_root = PointerProperty(name="Controlled Robot", type=bpy.types.Object)
    mounting.register_properties()
    bpy.types.Scene.robot_mount_distance = FloatProperty(
        name="Snap Distance", description="Maximum distance from the robot base to a surface or mount point",
        default=0.1, min=0.001, subtype="DISTANCE", unit="LENGTH",
    )
    bpy.types.Scene.robot_magnet_alignment = EnumProperty(
        name="Magnetic Alignment",
        items=(
            ("OPPOSE", "Face to Face", "Mate outward-facing point normals without overlapping the surfaces"),
            ("MATCH", "Match Frames", "Match both points' axes"),
            ("POSITION", "Position Only", "Keep the moving object's orientation"),
        ),
        default="OPPOSE",
    )
    bpy.types.Scene.robot_mujoco_python = StringProperty(name="MuJoCo Python", default="python")
    bpy.types.Scene.robot_mujoco_gravity = FloatVectorProperty(
        name="Gravity", size=3, default=(0.0, 0.0, -9.81), subtype="ACCELERATION",
    )
    bpy.types.Scene.robot_mujoco_timestep = FloatProperty(
        name="Physics Timestep", default=0.002, min=0.00001, max=0.1, precision=5,
    )
    bpy.types.Scene.robot_mujoco_rate = IntProperty(name="Viewport Updates", default=30, min=1, max=120)
    bpy.types.Scene.robot_mujoco_servo_kp = FloatProperty(name="Joint Stiffness", default=40, min=.01, max=100000)
    bpy.types.Scene.robot_mujoco_servo_kd = FloatProperty(name="Joint Damping", default=4, min=0, max=10000)
    bpy.types.Scene.robot_mujoco_servo_effort = FloatProperty(
        name="Maximum Joint Effort", description="Temporary servo cap in Nm for hinges or N for slides; MJCF force limits may reduce it",
        default=20, min=.001, max=100000,
    )
    bpy.types.Scene.robot_core_bridge_url = StringProperty(name="Core Bridge", default="http://127.0.0.1:8765")
    bpy.types.Scene.robot_core_host = StringProperty(name="Core Host", default="localhost")
    bpy.types.Scene.robot_core_port = IntProperty(name="Core Port", default=17080, min=1, max=65535)
    bpy.types.Scene.robot_core_status = StringProperty(name="Core Status", default="Bridge not checked")
    bpy.types.Scene.robot_core_log = StringProperty(name="Core Log")
    bpy.types.Scene.robot_core_command_valid = BoolProperty(name="Core Command Valid", default=False)
    bpy.types.Scene.robot_behavior_tree = PointerProperty(name="Data Graph", type=CaryamBehaviorTree)
    bpy.types.Scene.robot_ros_distro = EnumProperty(
        name="ROS", items=(("ros1", "ROS 1", ""), ("ros2", "ROS 2", "")), default="ros2",
    )
    bpy.types.Scene.robot_ros_topic = StringProperty(name="Topic", default="/joint_states")
    bpy.types.Scene.robot_ros_message_type = StringProperty(name="Type", default="std_msgs/Float64")
    bpy.types.Scene.robot_ros_payload = StringProperty(name="Payload", default="data: 0.0")
    bpy.types.Scene.robot_ros_status = StringProperty(name="ROS Status", default="Not checked")
    bpy.types.Scene.robot_build_oci_name = StringProperty(name="Image", default="caryam-robot")
    bpy.types.Scene.robot_build_base_image = StringProperty(name="Base Image", default=DEFAULT_BASE_IMAGE)
    bpy.types.Scene.robot_build_tool = EnumProperty(
        name="Tool", items=(("podman", "Podman", ""), ("docker", "Docker", "")), default="podman",
    )
    bpy.types.Scene.robot_build_target_disk = StringProperty(name="Target Disk", default="")
    bpy.types.Scene.robot_build_status = StringProperty(name="Build Status", default="Not built")
    bpy.types.NODE_MT_add.append(_draw_behavior_add_menu)
    bpy.app.timers.register(_refresh_active_library, first_interval=0.1)
    live_physics.register_handlers()


def unregister() -> None:
    live_physics.unregister_handlers()
    if bpy.app.timers.is_registered(_refresh_active_library):
        bpy.app.timers.unregister(_refresh_active_library)
    try:
        bpy.types.NODE_MT_add.remove(_draw_behavior_add_menu)
    except (RuntimeError, ValueError):
        pass
    _remove_scene_properties()
    _unregister_existing_classes()


if __name__ == "__main__":
    register()
