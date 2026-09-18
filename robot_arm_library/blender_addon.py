"""Blender add-on for discovering and placing Robot Library."""

from __future__ import annotations

import math
import json
import shutil
import subprocess
import xml.etree.ElementTree as ElementTree
from pathlib import Path
from typing import Iterable

import bpy
from bpy.props import BoolProperty, CollectionProperty, EnumProperty, FloatProperty, IntProperty, PointerProperty, StringProperty
from bpy.types import Node, NodeTree, Operator, Panel, PropertyGroup, UIList
from bpy_extras.io_utils import ExportHelper, ImportHelper
from mathutils import Euler, Matrix, Quaternion, Vector

from .setup_parser import discover_robot_definitions
from .behavior_graph import BehaviorGraph, BehaviorLink, BehaviorNode, default_behavior_graph
from .world_exporter import RobotInstance, write_world_export


bl_info = {
    "name": "MuJoCo Robot Arm Library",
    "author": "Caryam",
    "version": (0, 8, 6),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar > Robot Library",
    "description": "Discover robot arms from a MuJoCo setup and drag them into the scene",
    "category": "3D View",
}


DEFAULT_SETUP_PATH = str(Path.home() / "git_repos" / "mujoco_menagerie")
SUPPORTED_IMPORT_EXTENSIONS = {".blend", ".dae", ".fbx", ".glb", ".gltf", ".obj", ".ply", ".stl", ".xml"}


class RobotLibraryItem(PropertyGroup):
    robot_id: StringProperty()
    name: StringProperty()
    model_path: StringProperty()
    source_path: StringProperty()
    description: StringProperty()
    category: StringProperty()
    instance_count: IntProperty(default=0)


class CaryamBehaviorTree(NodeTree):
    bl_idname = "CARYAM_BEHAVIOR_TREE"
    bl_label = "Caryam Behavior Graph"
    bl_icon = "NODETREE"


class CaryamBehaviorNode(Node):
    bl_idname = "CARYAM_BEHAVIOR_NODE"
    bl_label = "Behavior Node"

    node_id: StringProperty(name="Id", default="node")
    kind: EnumProperty(
        name="Kind",
        items=(
            ("start", "Start", "Begin execution"),
            ("sensor", "Virtual Sensor", "Read a Blender or MuJoCo value"),
            ("ai_model", "AI Model", "Transform sensor data into a decision"),
            ("condition", "Condition", "Evaluate a boolean value"),
            ("action", "Robot Action", "Write a robot joint command"),
            ("end", "End", "Finish execution"),
        ),
        default="sensor",
    )
    sensor_path: StringProperty(name="Sensor", default="joint.qpos")
    model_name: StringProperty(name="Model", default="rule_based")
    robot_id: StringProperty(name="Robot")
    joint_name: StringProperty(name="Joint")
    target: FloatProperty(name="Target", default=0.0)

    def init(self, context: bpy.types.Context) -> None:
        self.inputs.new("NodeSocketFloat", "Value")
        self.outputs.new("NodeSocketFloat", "Value")

    def draw_label(self) -> str:
        return self.label or self.kind.replace("_", " ").title()

    def draw_buttons(self, context: bpy.types.Context, layout: bpy.types.UILayout) -> None:
        layout.prop(self, "kind", text="")
        if self.kind == "sensor":
            layout.prop(self, "sensor_path")
        elif self.kind == "ai_model":
            layout.prop(self, "model_name")
        elif self.kind == "action":
            layout.prop(self, "robot_id")
            layout.prop(self, "joint_name")
            layout.prop(self, "target")


def _behavior_tree(context: bpy.types.Context) -> CaryamBehaviorTree | None:
    edit_tree = getattr(getattr(context, "space_data", None), "edit_tree", None)
    if isinstance(edit_tree, CaryamBehaviorTree):
        return edit_tree
    tree = getattr(context.scene, "robot_behavior_tree", None)
    return tree if isinstance(tree, CaryamBehaviorTree) else None


def _node_data(node: CaryamBehaviorNode) -> dict[str, object]:
    if node.kind == "sensor":
        return {"sensor": node.sensor_path}
    if node.kind == "ai_model":
        return {"model": node.model_name}
    if node.kind == "action":
        return {"robot_id": node.robot_id, "joint": node.joint_name, "target": node.target}
    return {}


def _graph_from_tree(tree: CaryamBehaviorTree) -> BehaviorGraph:
    nodes = [
        BehaviorNode(
            node.node_id,
            node.kind,
            node.label or node.name,
            _node_data(node),
            (float(node.location.x), float(node.location.y)),
        )
        for node in tree.nodes
        if isinstance(node, CaryamBehaviorNode)
    ]
    links = [
        BehaviorLink(link.from_node.node_id, link.to_node.node_id)
        for link in tree.links
        if isinstance(link.from_node, CaryamBehaviorNode) and isinstance(link.to_node, CaryamBehaviorNode)
    ]
    return BehaviorGraph(tree.name, nodes, links)


def _populate_behavior_tree(tree: CaryamBehaviorTree, graph: BehaviorGraph) -> None:
    tree.nodes.clear()
    node_map: dict[str, CaryamBehaviorNode] = {}
    for item in graph.nodes:
        node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
        node.node_id = item.node_id
        node.kind = item.kind if item.kind in {"start", "sensor", "ai_model", "condition", "action", "end"} else "sensor"
        node.label = item.label
        node.location = item.position
        data = item.data
        node.sensor_path = str(data.get("sensor", "joint.qpos"))
        node.model_name = str(data.get("model", "rule_based"))
        node.robot_id = str(data.get("robot_id", ""))
        node.joint_name = str(data.get("joint", ""))
        node.target = float(data.get("target", 0.0))
        node_map[item.node_id] = node
    for link in graph.links:
        source = node_map.get(link.source)
        target = node_map.get(link.target)
        if source is not None and target is not None:
            tree.links.new(source.outputs[0], target.inputs[0])


def _ensure_behavior_tree(scene: bpy.types.Scene) -> CaryamBehaviorTree:
    tree = scene.robot_behavior_tree
    if tree is None:
        tree = bpy.data.node_groups.new("Robot Behavior", "CARYAM_BEHAVIOR_TREE")
        scene.robot_behavior_tree = tree
        _populate_behavior_tree(tree, default_behavior_graph())
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


def _refresh_library(scene: bpy.types.Scene) -> None:
    setup_path = _resolve_setup_path(scene.robot_setup_path)
    scene.robot_library.clear()
    try:
        definitions = discover_robot_definitions(setup_path)
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

    if definitions:
        scene.robot_library_status = f"{len(definitions)} robot(s) found in {setup_path}"
        scene.robot_library_index = min(scene.robot_library_index, len(definitions) - 1)
    else:
        scene.robot_library_status = f"No robot arms found in {setup_path}"
        scene.robot_library_index = 0


def _refresh_active_library() -> None:
    scene = getattr(bpy.context, "scene", None)
    if scene is not None and hasattr(scene, "robot_library"):
        _refresh_library(scene)
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
                    )
                    hierarchy_created = bool(imported)
                except (ImportError, ModuleNotFoundError, RuntimeError, ValueError):
                    imported = _import_mjcf_meshes(asset_path)
                    root["import_mode"] = "xml_fallback"
            elif asset_path.suffix.lower() in SUPPORTED_IMPORT_EXTENSIONS:
                imported = _import_mesh_file(asset_path)
        except (OSError, RuntimeError):
            imported = []

    if not imported:
        imported = _create_placeholder(item.name)
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
    collection = bpy.data.collections.get("Robot Library")
    if collection is None:
        return []
    instances: list[RobotInstance] = []
    for root in sorted((obj for obj in collection.objects if obj.get("robot_id")), key=lambda obj: obj.name):
        source_path = str(root.get("model_path", ""))
        if not source_path or Path(source_path).suffix.lower() != ".xml":
            continue
        qpos = {
            joint.name.removeprefix("Joint: "): float(joint["qpos"])
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
            )
        )
    return instances


def _export_world_file(filepath: str) -> tuple[Path, Path, Path]:
    output = Path(bpy.path.abspath(filepath)).expanduser().resolve()
    return write_world_export(output, _world_robot_instances())


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

    values = graph.run(sensor_reader, ai_runner, action_sink)
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
        ancestor = obj
        while ancestor.parent is not None and ancestor != root:
            ancestor = ancestor.parent
        if ancestor == root:
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
    layout.label(text=root.name, icon="CONSTRAINT_BONE")
    for joint in controls:
        layout.prop(joint, '["qpos"]', text=joint.name.removeprefix("Joint: "), slider=True)
    action_row = layout.row(align=True)
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
        _refresh_library(context.scene)
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
        except (OSError, ValueError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Exported {world_path.name}, {manifest_path.name}, and {render_path.name}")
        return {"FINISHED"}


class ROBOT_OT_export_and_render(ROBOT_OT_export_world):
    bl_idname = "robot_library.export_and_render"
    bl_label = "Export and Render in MuJoCo"

    def execute(self, context: bpy.types.Context) -> set[str]:
        try:
            world_path, _manifest_path, render_path = _export_world_file(self.filepath)
            executable = context.scene.robot_mujoco_python.strip() or shutil.which("python") or "python"
            subprocess.Popen([executable, str(render_path)], cwd=str(render_path.parent), start_new_session=True)
        except (OSError, ValueError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Exported and launched MuJoCo for {world_path.name}")
        return {"FINISHED"}


class ROBOT_OT_open_behavior_editor(Operator):
    bl_idname = "robot_library.open_behavior_editor"
    bl_label = "Open Behavior Editor"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        _ensure_behavior_tree(context.scene)
        if context.area is not None:
            context.area.type = "NODE_EDITOR"
            context.area.ui_type = "CARYAM_BEHAVIOR_TREE"
        return {"FINISHED"}


class ROBOT_OT_new_behavior(Operator):
    bl_idname = "robot_library.new_behavior"
    bl_label = "New Behavior Graph"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = bpy.data.node_groups.new("Robot Behavior", "CARYAM_BEHAVIOR_TREE")
        context.scene.robot_behavior_tree = tree
        _populate_behavior_tree(tree, default_behavior_graph())
        return {"FINISHED"}


class ROBOT_OT_add_behavior_node(Operator):
    bl_idname = "robot_library.add_behavior_node"
    bl_label = "Add Behavior Node"
    bl_options = {"REGISTER", "UNDO"}

    kind: EnumProperty(
        name="Kind",
        items=(
            ("sensor", "Virtual Sensor", ""),
            ("ai_model", "AI Model", ""),
            ("condition", "Condition", ""),
            ("action", "Robot Action", ""),
        ),
        default="sensor",
    )

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
        node.kind = self.kind
        node.node_id = f"{self.kind}_{len(tree.nodes)}"
        node.label = node.draw_label()
        node.location = (len(tree.nodes) * 180.0, 0.0)
        return {"FINISHED"}


class ROBOT_OT_save_behavior(Operator, ExportHelper):
    bl_idname = "robot_library.save_behavior"
    bl_label = "Save Behavior Graph"
    bl_options = {"REGISTER"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={"HIDDEN"})

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        graph = _graph_from_tree(tree)
        errors = graph.validate()
        if errors:
            self.report({"ERROR"}, "; ".join(errors))
            return {"CANCELLED"}
        graph.save(self.filepath)
        self.report({"INFO"}, f"Saved {Path(self.filepath).name}")
        return {"FINISHED"}


class ROBOT_OT_load_behavior(Operator, ImportHelper):
    bl_idname = "robot_library.load_behavior"
    bl_label = "Load Behavior Graph"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={"HIDDEN"})

    def execute(self, context: bpy.types.Context) -> set[str]:
        try:
            graph = BehaviorGraph.load(self.filepath)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        _populate_behavior_tree(tree, graph)
        return {"FINISHED"}


class ROBOT_OT_run_behavior(Operator):
    bl_idname = "robot_library.run_behavior"
    bl_label = "Run Behavior Graph"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        try:
            _run_behavior_graph(_graph_from_tree(tree))
        except (ValueError, KeyError, TypeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, "Behavior graph executed")
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
        add_row = layout.row(align=True)
        add_operator = add_row.operator("robot_library.add_robot", text="Add to World", icon="ADD")
        add_operator.robot_index = scene.robot_library_index
        drag_operator = add_row.operator("robot_library.drag_robot", text="Drag into Viewport", icon="HAND")
        drag_operator.robot_index = scene.robot_library_index
        layout.operator("robot_library.joint_window", text="Open Joint Controller", icon="CONSTRAINT_BONE")


class ROBOT_PT_sensor_library(Panel):
    bl_label = "Sensor Library"
    bl_idname = "ROBOT_PT_sensor_library"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Library"

    def draw(self, context: bpy.types.Context) -> None:
        self.layout.label(text="Sensor library is not configured", icon="INFO")


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
    bl_label = "MuJoCo Sim"
    bl_idname = "ROBOT_PT_mujoco_world"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MuJoCo World Simulation"
    bl_parent_id = "ROBOT_PT_world_simulation"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        layout.prop(scene, "robot_mujoco_python", text="Python")
        row = layout.row(align=True)
        row.operator("robot_library.export_world", text="Export", icon="EXPORT")
        row.operator("robot_library.export_and_render", text="Render", icon="PLAY")


class ROBOT_PT_world_simulation(Panel):
    bl_label = "MuJoCo World Simulation"
    bl_idname = "ROBOT_PT_world_simulation"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "MuJoCo World Simulation"

    def draw(self, context: bpy.types.Context) -> None:
        self.layout.label(text="Export or render the composed world", icon="WORLD")


class ROBOT_PT_behavior_graph(Panel):
    bl_label = "Robot Flow"
    bl_idname = "ROBOT_PT_behavior_graph"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Control"

    def draw(self, context: bpy.types.Context) -> None:
        self.layout.operator("robot_library.open_behavior_editor", text="Open Behavior Editor", icon="NODETREE")


class ROBOT_PT_rcs_control(Panel):
    bl_label = "RCS Control"
    bl_idname = "ROBOT_PT_rcs_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Control"

    def draw(self, context: bpy.types.Context) -> None:
        self.layout.label(text="RCS control is not configured", icon="INFO")


class ROBOT_PT_behavior_editor(Panel):
    bl_label = "Caryam Flow"
    bl_idname = "ROBOT_PT_behavior_editor"
    bl_space_type = "NODE_EDITOR"
    bl_region_type = "UI"
    bl_category = "Robot Flow"

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return context.space_data.tree_type == "CARYAM_BEHAVIOR_TREE"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        layout.label(text=f"{len(tree.nodes)} nodes, {len(tree.links)} links", icon="NODETREE")
        layout.operator("robot_library.new_behavior", text="New Graph", icon="FILE_NEW")
        add_row = layout.row(align=True)
        for kind, icon in (("sensor", "DRIVER_SENSOR"), ("ai_model", "LIGHT_DATA"), ("action", "CONSTRAINT_BONE")):
            operator = add_row.operator("robot_library.add_behavior_node", text="", icon=icon)
            operator.kind = kind
        layout.separator()
        file_row = layout.row(align=True)
        file_row.operator("robot_library.load_behavior", text="Load", icon="IMPORT")
        file_row.operator("robot_library.save_behavior", text="Save", icon="EXPORT")
        layout.operator("robot_library.run_behavior", text="Run Graph", icon="PLAY")


CLASSES = (
    RobotLibraryItem,
    CaryamBehaviorTree,
    CaryamBehaviorNode,
    ROBOT_OT_refresh,
    ROBOT_OT_add,
    ROBOT_OT_drag,
    ROBOT_OT_clear_filter,
    ROBOT_OT_reset_pose,
    ROBOT_OT_keyframe_pose,
    ROBOT_OT_export_world,
    ROBOT_OT_export_and_render,
    ROBOT_OT_open_behavior_editor,
    ROBOT_OT_new_behavior,
    ROBOT_OT_add_behavior_node,
    ROBOT_OT_save_behavior,
    ROBOT_OT_load_behavior,
    ROBOT_OT_run_behavior,
    ROBOT_OT_joint_window,
    ROBOT_UL_library,
    ROBOT_PT_library,
    ROBOT_PT_sensor_library,
    ROBOT_PT_joint_controls,
    ROBOT_PT_world_simulation,
    ROBOT_PT_mujoco_world,
    ROBOT_PT_behavior_graph,
    ROBOT_PT_rcs_control,
    ROBOT_PT_behavior_editor,
)


def _remove_scene_properties() -> None:
    for name in ("robot_library", "robot_library_index", "robot_library_status", "robot_library_filter", "robot_setup_path", "robot_library_auto_spread", "robot_library_include_collisions", "robot_library_control_root", "robot_mujoco_python", "robot_behavior_tree"):
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
    _remove_scene_properties()
    _unregister_existing_classes()
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.robot_setup_path = StringProperty(name="Robot setup", default=DEFAULT_SETUP_PATH)
    bpy.types.Scene.robot_library_filter = StringProperty(name="Filter")
    bpy.types.Scene.robot_library_status = StringProperty(name="Status", default="Refresh to discover robot arms")
    bpy.types.Scene.robot_library_index = IntProperty(name="Robot", default=0, min=0)
    bpy.types.Scene.robot_library = CollectionProperty(type=RobotLibraryItem)
    bpy.types.Scene.robot_library_auto_spread = BoolProperty(name="Auto-space Add", default=True)
    bpy.types.Scene.robot_library_include_collisions = BoolProperty(name="Load Collision Geoms", default=False)
    bpy.types.Scene.robot_library_control_root = PointerProperty(name="Controlled Robot", type=bpy.types.Object)
    bpy.types.Scene.robot_mujoco_python = StringProperty(name="MuJoCo Python", default="python")
    bpy.types.Scene.robot_behavior_tree = PointerProperty(name="Behavior Graph", type=CaryamBehaviorTree)
    bpy.app.timers.register(_refresh_active_library, first_interval=0.1)


def unregister() -> None:
    if bpy.app.timers.is_registered(_refresh_active_library):
        bpy.app.timers.unregister(_refresh_active_library)
    _remove_scene_properties()
    _unregister_existing_classes()


if __name__ == "__main__":
    register()