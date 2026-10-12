"""Validate dynamic application-node sockets in Blender."""

import sys
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon
from robot_arm_library.behavior_graph import PortSpec


robot_arm_library.register()
try:
    scene = bpy.context.scene
    result = bpy.ops.robot_library.new_behavior()
    assert result == {"FINISHED"}, result
    assert len(scene.robot_behavior_tree.nodes) == 0
    canvas = scene.robot_behavior_tree

    for kind in ("application", "sensor", "ai_model", "action", "condition", "start", "end"):
        result = bpy.ops.robot_library.add_behavior_node(kind=kind)
        assert result == {"FINISHED"}, (kind, result)
    assert {node.kind for node in scene.robot_behavior_tree.nodes} == {
        "application",
        "sensor",
        "ai_model",
        "action",
        "condition",
        "start",
        "end",
    }
    removable = canvas.nodes.new("CARYAM_BEHAVIOR_NODE")
    removable.node_id = "remove_me"
    result = bpy.ops.robot_library.delete_data_node(node_name=removable.name)
    assert result == {"FINISHED"}, result
    assert canvas.nodes.get(removable.name) is None
    result = bpy.ops.robot_library.new_behavior()
    assert result == {"FINISHED"}, result
    assert scene.robot_behavior_tree == canvas
    assert len(canvas.nodes) == 0

    result = bpy.ops.robot_library.add_behavior_node(preset="virtual_rgb_camera")
    assert result == {"FINISHED"}, result
    camera_preset = canvas.nodes[-1]
    assert camera_preset.kind == "virtual_rgb_camera"
    assert camera_preset.output_ports[0].port_name == "rgb_image"
    assert camera_preset.output_ports[0].shape_tuple() == (1, 3, 480, 640)
    assert camera_preset.output_ports[0].data_type == "image_rgb8"
    assert camera_preset.camera_output_label() == "Output: rgb_image [3X640X480] (RGB8)"
    camera_preset.camera_width = 800
    camera_preset.camera_height = 600
    assert camera_preset.output_ports[0].shape_tuple() == (1, 3, 600, 800)
    assert camera_preset.camera_output_label() == "Output: rgb_image [3X800X600] (RGB8)"

    result = bpy.ops.robot_library.add_behavior_node(preset="virtual_depth_camera")
    assert result == {"FINISHED"}, result
    depth_preset = canvas.nodes[-1]
    assert depth_preset.kind == "virtual_depth_camera"
    assert [(port.port_name, port.data_type) for port in depth_preset.output_ports] == [
        ("depth_image", "image_depth32f"),
    ]
    depth_preset.camera_width = 320
    depth_preset.camera_height = 240
    assert depth_preset.output_ports[0].shape_tuple() == (1, 1, 240, 320)
    assert depth_preset.camera_output_label() == "Output: depth_image [1X320X240] (32FC1)"

    result = bpy.ops.robot_library.add_behavior_node(preset="virtual_imu")
    assert result == {"FINISHED"}, result
    imu_preset = canvas.nodes[-1]
    assert imu_preset.kind == "virtual_imu"
    assert [(port.port_name, port.data_type) for port in imu_preset.output_ports] == [("imu", "imu")]

    result = bpy.ops.robot_library.add_behavior_node(preset="virtual_tactile_array")
    assert result == {"FINISHED"}, result
    tactile_preset = canvas.nodes[-1]
    assert tactile_preset.kind == "virtual_tactile_array"
    assert [(port.port_name, port.data_type) for port in tactile_preset.output_ports] == [
        ("tactile_array", "tactile_array"),
    ]
    camera_preset.sensor_mount_frame = "wrist_camera"
    camera_preset.sensor_rate_hz = 15.0
    depth_preset.depth_max_m = 4.0
    imu_preset.imu_include_magnetometer = True
    tactile_preset.tactile_geom = "finger_pad"
    sensor_graph = blender_addon._graph_from_tree(canvas)
    sensor_data = {node.node_id: node.data for node in sensor_graph.nodes}
    assert sensor_data[camera_preset.node_id]["mount_frame"] == "wrist_camera"
    assert sensor_data[depth_preset.node_id]["max_depth_m"] == 4.0
    assert sensor_data[imu_preset.node_id]["include_magnetometer"] is True
    assert sensor_data[tactile_preset.node_id]["geom"] == "finger_pad"

    result = bpy.ops.robot_library.add_behavior_node(preset="virtual_lidar")
    assert result == {"FINISHED"}, result
    lidar_preset = canvas.nodes[-1]
    assert lidar_preset.kind == "virtual_lidar"
    assert [(port.port_name, port.data_type) for port in lidar_preset.output_ports] == [
        ("point_cloud", "point_cloud2"),
    ]
    assert lidar_preset.lidar_max_range_m == 10.0
    assert lidar_preset.lidar_samples == 360

    result = bpy.ops.robot_library.add_behavior_node(preset="ai_model")
    assert result == {"FINISHED"}, result
    ai_preset = canvas.nodes[-1]
    assert ai_preset.input_ports[0].shape_tuple() == (1, 3, 480, 640)

    result = bpy.ops.robot_library.add_behavior_node(preset="actuator_system")
    assert result == {"FINISHED"}, result
    actuator_preset = canvas.nodes[-1]
    assert actuator_preset.input_ports[0].port_name == "joint_positions"
    assert actuator_preset.input_ports[0].shape_tuple() == (1, 6)

    area = next(area for area in bpy.context.screen.areas if area.type == "PROPERTIES")
    with bpy.context.temp_override(area=area):
        result = bpy.ops.robot_library.open_behavior_editor()
    assert result == {"FINISHED"}, result
    space = area.spaces.active
    assert space.type == "NODE_EDITOR"
    assert space.edit_tree == canvas

    tree = bpy.data.node_groups.new("Port Sync Test", "CARYAM_BEHAVIOR_TREE")
    scene.robot_behavior_tree = tree
    node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
    node.kind = "application"

    port = node.input_ports.add()
    port.port_name = "oak1"
    port.shape_csv = "1,3,480,640"
    node.sync_sockets()

    assert list(node.inputs.keys()) == ["oak1"], list(node.inputs.keys())
    assert list(node.outputs.keys()) == [], list(node.outputs.keys())
    assert port.shape_tuple() == (1, 3, 480, 640)

    node.kind = "sensor"
    assert list(node.inputs.keys()) == ["value"], list(node.inputs.keys())
    assert list(node.outputs.keys()) == ["value"], list(node.outputs.keys())

    node.kind = "application"
    output_port = node.output_ports.add()
    output_port.port_name = "decision"
    node.sync_sockets()
    assert list(node.outputs.keys()) == ["decision"], list(node.outputs.keys())
    node.output_ports.remove(0)
    node.sync_sockets()
    assert list(node.outputs.keys()) == [], list(node.outputs.keys())

    node.node_id = "camera"
    node.output_ports.add().port_name = "oak1"
    node.output_ports[0].shape_csv = "1,3,480,640"
    node.sync_sockets()
    transform = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
    transform.node_id = "image_transform"
    transform.kind = "application"
    transform.input_ports.add().port_name = "oak1"
    transform.output_ports.add().port_name = "smolvla"
    transform.sync_sockets()
    tree.links.new(node.outputs["oak1"], transform.inputs["oak1"])

    graph = blender_addon._graph_from_tree(tree)
    assert graph.validate() == [], graph.validate()
    assert graph.node_map()["camera"].outputs == (PortSpec("oak1", (1, 3, 480, 640)),)
    link = graph.links[0]
    assert (link.source, link.source_port, link.target, link.target_port) == (
        "camera",
        "oak1",
        "image_transform",
        "oak1",
    )

    incompatible = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
    incompatible.node_id = "incompatible_model"
    incompatible.kind = "application"
    incompatible.input_ports.add().port_name = "oak1"
    incompatible.input_ports[0].shape_csv = "1,3,224,224"
    incompatible.sync_sockets()
    tree.links.new(node.outputs["oak1"], incompatible.inputs["oak1"])
    tree.update()
    assert all(link.to_node != incompatible for link in tree.links)

    values = blender_addon._run_behavior_graph(graph)
    assert values["image_transform"] == {"smolvla": None}
    print("Application node socket sync test passed")
finally:
    robot_arm_library.unregister()