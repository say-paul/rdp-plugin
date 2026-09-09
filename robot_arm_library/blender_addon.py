"""Blender add-on for discovering and placing MuJoCo robot arms."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ElementTree
from pathlib import Path
from typing import Iterable

import bpy
from bpy.props import BoolProperty, CollectionProperty, IntProperty, StringProperty
from bpy.types import Operator, Panel, PropertyGroup, UIList
from mathutils import Euler, Matrix, Quaternion, Vector

from .setup_parser import discover_robot_definitions


bl_info = {
    "name": "MuJoCo Robot Arm Library",
    "author": "Caryam",
    "version": (0, 5, 0),
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
    except OSError as error:
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
    collection = bpy.data.collections.get("MuJoCo Robot Arms")
    if collection is None:
        collection = bpy.data.collections.new("MuJoCo Robot Arms")
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
    collection = bpy.data.collections.get("MuJoCo Robot Arms")
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
    imported: list[bpy.types.Object] = []
    hierarchy_created = False
    if asset_path is not None and asset_path.exists():
        try:
            if asset_path.suffix.lower() == ".blend":
                imported = _import_blend_file(asset_path)
            elif asset_path.suffix.lower() == ".xml":
                try:
                    from .compiled_importer import import_compiled_mjcf

                    imported = import_compiled_mjcf(asset_path, root, robot_collection)
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
    return root


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
        _place_robot(items[self.robot_index], _add_location(context.scene))
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
            _place_robot(context.scene.robot_library[self.robot_index], location)
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
            if query and query not in f"{item.name} {item.category}".lower()
            else 0
            for item in items
        ]
        return flags, []


class ROBOT_PT_library(Panel):
    bl_label = "MuJoCo Robot Arms"
    bl_idname = "ROBOT_PT_library"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Library"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
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
            if scene.robot_library_status == "Refresh to discover robot arms":
                _refresh_library(scene)
            if not scene.robot_library:
                return

        if not scene.robot_library:
            return

        item = scene.robot_library[scene.robot_library_index]
        layout.label(text=item.model_path or "No model path; placeholder will be used", icon="FILE")
        if item.category:
            layout.label(text=item.category.replace("_", " ").title(), icon="OUTLINER_OB_ARMATURE")
        layout.prop(scene, "robot_library_auto_spread", text="Auto-space Add")
        add_row = layout.row(align=True)
        add_operator = add_row.operator("robot_library.add_robot", text="Add to World", icon="ADD")
        add_operator.robot_index = scene.robot_library_index
        drag_operator = add_row.operator("robot_library.drag_robot", text="Drag into Viewport", icon="HAND")
        drag_operator.robot_index = scene.robot_library_index

        robot_root = _selected_robot_root(context.active_object)
        if robot_root is None or robot_root.get("import_mode") != "mujoco_compiled":
            return
        controls = _joint_controls(robot_root)
        if not controls:
            return
        joint_box = layout.box()
        joint_box.label(text=f"Joint Controls: {robot_root.name}", icon="CONSTRAINT_BONE")
        for joint in controls:
            joint_box.prop(joint, '["qpos"]', text=joint.name.removeprefix("Joint: "), slider=True)
        action_row = joint_box.row(align=True)
        reset_operator = action_row.operator("robot_library.reset_pose", text="Reset", icon="LOOP_BACK")
        reset_operator.root_name = robot_root.name
        keyframe_operator = action_row.operator("robot_library.keyframe_pose", text="Keyframe", icon="KEY_HLT")
        keyframe_operator.root_name = robot_root.name


CLASSES = (
    RobotLibraryItem,
    ROBOT_OT_refresh,
    ROBOT_OT_add,
    ROBOT_OT_drag,
    ROBOT_OT_clear_filter,
    ROBOT_OT_reset_pose,
    ROBOT_OT_keyframe_pose,
    ROBOT_UL_library,
    ROBOT_PT_library,
)


def _remove_scene_properties() -> None:
    for name in ("robot_library", "robot_library_index", "robot_library_status", "robot_library_filter", "robot_setup_path", "robot_library_auto_spread"):
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


def unregister() -> None:
    _remove_scene_properties()
    _unregister_existing_classes()


if __name__ == "__main__":
    register()