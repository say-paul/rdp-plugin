"""Sticky robot placement and persistent rigid mount relationships in Blender."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import bpy
from bpy.props import BoolProperty, EnumProperty, PointerProperty
from bpy.types import Operator, Panel, PropertyGroup
from mathutils import Matrix, Quaternion, Vector
from mathutils.bvhtree import BVHTree

from . import sensor_visuals


_KEYMAP_ITEMS = []
_AXIS_VECTORS = {
    "Z": Vector((0, 0, 1)), "NEG_Z": Vector((0, 0, -1)),
    "X": Vector((1, 0, 0)), "NEG_X": Vector((-1, 0, 0)),
    "Y": Vector((0, 1, 0)), "NEG_Y": Vector((0, -1, 0)),
}


class CaryamRobotMount(PropertyGroup):
    enabled: BoolProperty(default=False)
    target: PointerProperty(name="Mount Target", type=bpy.types.Object)
    source_anchor: PointerProperty(name="Moving Anchor", type=bpy.types.Object)
    uses_source_anchor: BoolProperty(default=False)


def robot_root(obj: bpy.types.Object | None) -> bpy.types.Object | None:
    while obj is not None:
        if "robot_id" in obj:
            return obj
        if obj.type == "MESH" and obj.caryam_mount.enabled and not obj.get("sensor_id"):
            return None
        obj = obj.parent
    return None


def robot_body(obj: bpy.types.Object | None) -> bpy.types.Object | None:
    while obj is not None and "robot_id" not in obj:
        if obj.type == "MESH" and obj.caryam_mount.enabled and not obj.get("sensor_id"):
            return None
        if "mujoco_body_id" in obj:
            return obj
        obj = obj.parent
    return None


def base_frame(root: bpy.types.Object) -> bpy.types.Object:
    bodies = [
        obj for obj in root.children_recursive
        if "mujoco_body_id" in obj and robot_root(obj) == root
    ]
    bases = [obj for obj in bodies if robot_body(obj.parent) is None]
    if len(bases) != 1:
        raise ValueError("Sticky Move requires a compiled MJCF robot with exactly one base body")
    return bases[0]


def _rigid_matrix(matrix: Matrix) -> Matrix:
    return Matrix.LocRotScale(matrix.translation, matrix.to_quaternion(), Vector((1, 1, 1)))


def _validate_robot(root: bpy.types.Object) -> None:
    if root.get("import_mode") != "mujoco_compiled" or Path(str(root.get("model_path", ""))).suffix.lower() != ".xml":
        raise ValueError("Select an imported, compiled MJCF robot for Sticky Move")
    if any(abs(value - 1.0) > 1e-5 for value in root.matrix_world.to_scale()):
        raise ValueError("Sticky Move requires a robot with unit scale")
    base_frame(root)


def _physical_target(target: bpy.types.Object) -> bpy.types.Object:
    if target.get(sensor_visuals.PREVIEW_KEY) and not target.get(sensor_visuals.HOUSING_KEY):
        raise ValueError("Sensor guides are overlays, not mount surfaces")
    sensor = sensor_visuals.selected_mount(target)
    if sensor is not None:
        return sensor
    obj = target
    while obj is not None:
        if obj.type == "MESH" and obj.caryam_mount.enabled:
            return obj
        body = robot_body(obj)
        if body is not None:
            return body
        if "robot_id" in obj:
            return base_frame(obj)
        if obj.type == "MESH":
            if robot_root(obj) is not None:
                raise ValueError("The target geometry has no compiled robot body")
            return obj
        obj = obj.parent
    raise ValueError("Mount points must belong to an exported mesh or robot body")


def _is_descendant(obj: bpy.types.Object, root: bpy.types.Object) -> bool:
    while obj is not None:
        if obj == root:
            return True
        obj = obj.parent
    return False


def _validate_target(root: bpy.types.Object, target: bpy.types.Object) -> bpy.types.Object:
    if _is_descendant(target, root):
        raise ValueError("A robot cannot mount to itself or one of its descendants")
    physical = _physical_target(target)
    if physical.get("sensor_id"):
        raise ValueError("Choose an exported mesh or robot body as mount target")
    target_root = robot_root(physical)
    if target_root is not None and target_root.get("import_mode") != "mujoco_compiled":
        raise ValueError("The target robot must be a compiled MJCF model")
    if physical.type == "MESH" and physical.hide_render:
        raise ValueError("The mount surface is excluded from MuJoCo export")
    return physical


def is_anchor(obj: bpy.types.Object | None) -> bool:
    return obj is not None and bool(obj.get("caryam_mount_point"))


def moving_object(obj: bpy.types.Object | None) -> bpy.types.Object | None:
    if obj is None:
        return None
    sensor = sensor_visuals.selected_mount(obj)
    if sensor is not None:
        return sensor
    root = robot_root(obj)
    if root is not None:
        return root
    if is_anchor(obj):
        return _physical_target(obj)
    return obj if obj.type == "MESH" else None


def _validate_moving(root: bpy.types.Object) -> None:
    if "robot_id" in root:
        _validate_robot(root)
    elif not root.get("sensor_id") and (root.type != "MESH" or root.hide_render):
        raise ValueError("Select a sensor, exported mesh or compiled robot to move")
    if abs(root.matrix_world.determinant()) < 1e-10:
        raise ValueError("Magnetic snapping requires nonzero object scale")


def _source_anchor(
    root: bpy.types.Object, selected: bpy.types.Object | None, require_enabled: bool = True,
) -> bpy.types.Object | None:
    anchor = selected if is_anchor(selected) else root.caryam_mount.source_anchor
    if anchor is None:
        anchors = [obj for obj in root.children_recursive if is_anchor(obj) and moving_object(obj) == root and obj.caryam_anchor_enabled]
        if len(anchors) == 1:
            anchor = anchors[0]
        elif len(anchors) > 1:
            raise ValueError("Select the magnetic point on the object that should snap")
    if anchor is not None:
        if not is_anchor(anchor) or moving_object(anchor) != root or (require_enabled and not anchor.caryam_anchor_enabled):
            raise ValueError("Choose an enabled magnetic point belonging to the moving object")
        if "robot_id" in root and _physical_target(anchor) != base_frame(root):
            raise ValueError("Move a robot using an anchor on its base; articulated-link anchors can receive attachments")
    elif "robot_id" not in root:
        raise ValueError("Create or select a magnetic point on the moving object first")
    return anchor


def mount_robot(root: bpy.types.Object, target: bpy.types.Object, source_anchor: bpy.types.Object | None = None) -> None:
    _validate_moving(root)
    _validate_target(root, target)
    world = root.matrix_world.copy()
    root.parent = target
    root.matrix_parent_inverse = target.matrix_world.inverted()
    root.matrix_basis = world
    root.caryam_mount.target = target
    root.caryam_mount.enabled = True
    root.caryam_mount.uses_source_anchor = source_anchor is not None
    if source_anchor is not None:
        root.caryam_mount.source_anchor = source_anchor


def detach_robot(root: bpy.types.Object) -> None:
    world = root.matrix_world.copy()
    root.parent = None
    root.matrix_world = world
    root.caryam_mount.enabled = False
    root.caryam_mount.target = None
    root.caryam_mount.uses_source_anchor = False


def export_mount(root: bpy.types.Object):
    from .world_exporter import RobotMount

    if not root.caryam_mount.enabled:
        return None
    _validate_moving(root)
    if root.caryam_mount.uses_source_anchor:
        source = root.caryam_mount.source_anchor
        if source is None or source.name not in bpy.context.scene.objects:
            raise ValueError(f"Object '{root.name}' has a missing source anchor; detach or remount it")
        _source_anchor(root, source, require_enabled=False)
    target = root.caryam_mount.target
    if target is None or target.name not in bpy.context.scene.objects:
        raise ValueError(f"Object '{root.name}' has a missing mount target; detach or remount it")
    if root.parent != target:
        raise ValueError(f"Robot '{root.name}' mount parenting changed; detach or remount it")
    physical = _validate_target(root, target)
    relative = _rigid_matrix(physical.matrix_world).inverted() @ _rigid_matrix(root.matrix_world)
    target_root = robot_root(physical)
    position = relative.translation
    rotation = relative.to_quaternion()
    return RobotMount(
        target_kind="robot" if target_root is not None else "scene",
        target_instance_id=target_root.name if target_root is not None else physical.name,
        target_body_name=str(physical.get("mujoco_body_name", physical.name.removeprefix("Body: "))) if target_root else "",
        position=(float(position.x), float(position.y), float(position.z)),
        quaternion=(float(rotation.w), float(rotation.x), float(rotation.y), float(rotation.z)),
    )


@dataclass
class SnapTarget:
    target: bpy.types.Object
    position: Vector
    rotation: Quaternion


@dataclass
class SnapSurface:
    target: bpy.types.Object
    tree: BVHTree


def snap_candidates(context: bpy.types.Context, root: bpy.types.Object) -> tuple[list[SnapSurface], list[bpy.types.Object]]:
    surfaces = []
    points = []
    depsgraph = context.evaluated_depsgraph_get()
    for obj in context.scene.objects:
        if _is_descendant(obj, root) or not obj.visible_get():
            continue
        try:
            _validate_target(root, obj)
        except ValueError:
            continue
        if is_anchor(obj) and obj.caryam_anchor_enabled:
            points.append(obj)
        elif obj.type == "MESH" and not obj.hide_render:
            evaluated = obj.evaluated_get(depsgraph)
            mesh = evaluated.to_mesh()
            try:
                vertices = [evaluated.matrix_world @ vertex.co for vertex in mesh.vertices]
                polygons = [tuple(polygon.vertices) for polygon in mesh.polygons]
                if polygons:
                    surfaces.append(SnapSurface(obj, BVHTree.FromPolygons(vertices, polygons)))
            finally:
                evaluated.to_mesh_clear()
    return surfaces, points


def _nearest_axis_rotation(target_rotation: Quaternion, source_rotation: Quaternion, axis: str) -> Quaternion:
    source_normal = source_rotation @ Vector((0, 0, 1))
    if axis == "AUTO":
        normal = max(_AXIS_VECTORS.values(), key=lambda value: round((target_rotation @ value).dot(source_normal), 7))
    else:
        normal = _AXIS_VECTORS[axis]
    candidates = []
    for tangent in _AXIS_VECTORS.values():
        if abs(tangent.dot(normal)) < .5:
            local = Matrix((tangent, normal.cross(tangent), normal)).transposed().to_quaternion()
            candidates.append(target_rotation @ local)
    return max(candidates, key=lambda candidate: round(abs(candidate.dot(source_rotation)), 7))


def nearest_snap(
    position: Vector,
    rotation: Quaternion,
    radius: float,
    surfaces: list[SnapSurface],
    points: list[bpy.types.Object],
    alignment: str = "MATCH",
) -> SnapTarget | None:
    nearby_points = [point for point in points if (point.matrix_world.translation - position).length <= radius]
    if nearby_points:
        point = min(nearby_points, key=lambda item: (item.matrix_world.translation - position).length)
        target_rotation = point.matrix_world.to_quaternion()
        if alignment == "AXIS":
            target_rotation = _nearest_axis_rotation(target_rotation, rotation, point.caryam_magnet_axis)
        elif alignment == "OPPOSE":
            target_rotation = target_rotation @ Quaternion((1, 0, 0), 3.141592653589793)
        elif alignment == "POSITION":
            target_rotation = rotation.copy()
        return SnapTarget(point, point.matrix_world.translation.copy(), target_rotation)
    closest = None
    distance = radius
    for surface in surfaces:
        location, normal, _, candidate_distance = surface.tree.find_nearest(position, radius)
        if location is not None and candidate_distance <= distance:
            up = rotation @ Vector((0, 0, 1))
            aligned = up.rotation_difference(normal) @ rotation
            closest = SnapTarget(surface.target, location, aligned)
            distance = candidate_distance
    return closest


def snap_transform(base_local: Matrix, snap: SnapTarget) -> Matrix:
    return Matrix.LocRotScale(snap.position, snap.rotation, Vector((1, 1, 1))) @ base_local.inverted()


class _MountMove:
    magnetic_points = False

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        if context.mode != "OBJECT":
            return False
        try:
            return moving_object(context.active_object) is not None
        except ValueError:
            return False

    def _prepare(self, context: bpy.types.Context) -> None:
        root = moving_object(context.active_object)
        if root is None:
            raise ValueError("Select a mesh, robot, or its magnetic point first")
        context.view_layer.update()
        _validate_moving(root)
        self.root = root
        self.use_magnets = getattr(self, "magnetic_points", False)
        self.source_anchor = _source_anchor(root, context.active_object) if self.use_magnets else None
        if self.use_magnets and self.source_anchor is None:
            raise ValueError("Create or select a magnetic point on the moving object first")
        self.alignment = context.scene.robot_magnet_alignment if self.use_magnets else "MATCH"
        self.original_parent = self.root.parent
        self.original_inverse = self.root.matrix_parent_inverse.copy()
        self.original_basis = self.root.matrix_basis.copy()
        self.original_world = self.root.matrix_world.copy()
        anchor_frame = self.source_anchor
        if anchor_frame is None:
            anchor_frame = base_frame(root) if "robot_id" in root else root
        self.base_local = self.original_world.inverted() @ _rigid_matrix(anchor_frame.matrix_world)
        self.base_world = self.original_world @ self.base_local
        self.surfaces, self.points = snap_candidates(context, self.root)
        if self.use_magnets:
            self.surfaces = []
        else:
            self.points = [point for point in self.points if not point.get("caryam_magnetic_anchor")]
        self.snap = nearest_snap(
            self.base_world.translation, self.base_world.to_quaternion(),
            context.scene.robot_mount_distance, self.surfaces, self.points, self.alignment,
        )

    def execute(self, context: bpy.types.Context) -> set[str]:
        try:
            self._prepare(context)
            snap = nearest_snap(
                self.base_world.translation, self.base_world.to_quaternion(),
                context.scene.robot_mount_distance, self.surfaces, self.points, self.alignment,
            )
            if snap is None:
                message = "No magnetic point within the snap distance" if self.use_magnets else "No surface or mount point within the snap distance"
                self.report({"WARNING"}, message)
                return {"CANCELLED"}
            self.root.matrix_world = snap_transform(self.base_local, snap)
            mount_robot(self.root, snap.target, self.source_anchor)
            context.view_layer.update()
        except (ValueError, RuntimeError) as error:
            if hasattr(self, "original_world"):
                self._restore(context)
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}

    def invoke(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        if context.area is None or context.area.type != "VIEW_3D":
            self.report({"WARNING"}, "Start the placement tool in a 3D viewport")
            return {"CANCELLED"}
        try:
            self._prepare(context)
        except ValueError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.region = next(region for region in context.area.regions if region.type == "WINDOW")
        self.region_3d = context.space_data.region_3d
        self.mouse_offset = Vector((0, 0, 0)) if self.use_magnets else self.base_world.translation - self._mouse_position(event)
        world = self.root.matrix_world.copy()
        self.root.parent = None
        self.root.matrix_world = snap_transform(self.base_local, self.snap) if self.snap is not None else world
        context.view_layer.update()
        context.window_manager.modal_handler_add(self)
        self._status(context)
        return {"RUNNING_MODAL"}

    def _mouse_position(self, event: bpy.types.Event) -> Vector:
        from bpy_extras import view3d_utils

        coordinate = (event.mouse_x - self.region.x, event.mouse_y - self.region.y)
        return view3d_utils.region_2d_to_location_3d(
            self.region, self.region_3d, coordinate, self.base_world.translation,
        )

    def _status(self, context: bpy.types.Context) -> None:
        tool = "Magnetic Move" if self.use_magnets else "Sticky Move"
        missing = "No nearby magnetic point" if self.use_magnets else "No nearby surface"
        target = f"Rigid mount: {self.snap.target.name}" if self.snap else missing
        context.workspace.status_text_set(f"{tool} — {target} | Click: place | Esc / Right-click: cancel")
        context.area.tag_redraw()

    def _restore(self, context: bpy.types.Context) -> None:
        self.root.parent = self.original_parent
        self.root.matrix_parent_inverse = self.original_inverse
        self.root.matrix_basis = self.original_basis
        context.view_layer.update()

    def modal(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        if event.type in {"ESC", "RIGHTMOUSE"}:
            self._restore(context)
            context.workspace.status_text_set(None)
            return {"CANCELLED"}
        if self.use_magnets and event.type in {"MOUSEMOVE", "LEFTMOUSE"}:
            if not (self.region.x <= event.mouse_x < self.region.x + self.region.width
                    and self.region.y <= event.mouse_y < self.region.y + self.region.height):
                return {"RUNNING_MODAL"}
        if event.type == "MOUSEMOVE":
            position = self._mouse_position(event) + self.mouse_offset
            rotation = self.base_world.to_quaternion()
            self.snap = nearest_snap(position, rotation, context.scene.robot_mount_distance, self.surfaces, self.points, self.alignment)
            if self.snap is not None:
                self.root.matrix_world = snap_transform(self.base_local, self.snap)
            else:
                world = self.original_world.copy()
                world.translation += position - self.base_world.translation
                self.root.matrix_world = world
            context.view_layer.update()
            self._status(context)
        elif event.type == "LEFTMOUSE" and event.value == "PRESS":
            try:
                if self.snap is not None:
                    mount_robot(self.root, self.snap.target, self.source_anchor)
                else:
                    detach_robot(self.root)
            except ValueError as error:
                self._restore(context)
                self.report({"ERROR"}, str(error))
                context.workspace.status_text_set(None)
                return {"CANCELLED"}
            context.view_layer.update()
            context.workspace.status_text_set(None)
            return {"FINISHED"}
        elif event.type in {"MIDDLEMOUSE", "WHEELUPMOUSE", "WHEELDOWNMOUSE"}:
            return {"PASS_THROUGH"}
        return {"RUNNING_MODAL"}

    def cancel(self, context: bpy.types.Context) -> None:
        self._restore(context)
        context.workspace.status_text_set(None)


class ROBOT_OT_sticky_move(_MountMove, Operator):
    bl_idname = "robot_library.sticky_move"
    bl_label = "Sticky Move"
    bl_description = "Snap the robot base to a surface without requiring magnetic points"
    bl_options = {"REGISTER", "UNDO", "BLOCKING"}


class ROBOT_OT_magnetic_move(_MountMove, Operator):
    bl_idname = "robot_library.magnetic_move"
    bl_label = "Magnetic Move"
    bl_description = "Snap a custom point to another magnetic point; Ctrl+Shift+M in the 3D viewport"
    bl_options = {"REGISTER", "UNDO", "BLOCKING"}
    magnetic_points = True


class ROBOT_OT_detach_mount(Operator):
    bl_idname = "robot_library.detach_mount"
    bl_label = "Detach Mount"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        try:
            root = moving_object(context.active_object)
            return root is not None and root.caryam_mount.enabled
        except ValueError:
            return False

    def execute(self, context: bpy.types.Context) -> set[str]:
        detach_robot(moving_object(context.active_object))
        context.view_layer.update()
        return {"FINISHED"}


def _anchor_visual(context: bpy.types.Context, name: str) -> bpy.types.Object:
    point = bpy.data.objects.new(name, None)
    context.collection.objects.link(point)
    point.empty_display_type = "SPHERE"
    point.empty_display_size = 0.025
    point.show_in_front = True
    point.show_name = True
    point.show_axis = True
    point.color = (0.1, 0.85, 1.0, 1.0)
    return point


def create_magnetic_point(
    context: bpy.types.Context, target: bpy.types.Object, world: Matrix,
    point: bpy.types.Object | None = None,
) -> bpy.types.Object:
    physical = _physical_target(target)
    if physical.type == "MESH" and physical.hide_render and not physical.get("sensor_id"):
        raise ValueError("Choose a sensor or surface included in MuJoCo export")
    parent_frame = _rigid_matrix(physical.matrix_world)
    inverse = physical.matrix_world.inverted() @ parent_frame
    local = parent_frame.inverted() @ _rigid_matrix(world)
    if point is None:
        point = _anchor_visual(context, "Magnetic Point")
    assert point is not None
    point.name = "Magnetic Point"
    point["caryam_mount_point"] = True
    point["caryam_magnetic_anchor"] = True
    point.caryam_anchor_enabled = True
    point.parent = physical
    point.matrix_parent_inverse = inverse
    point.matrix_basis = local
    context.view_layer.update()
    owner = moving_object(point)
    if owner is not None and not owner.caryam_mount.enabled:
        if "robot_id" not in owner or robot_body(physical.parent) is None:
            owner.caryam_mount.source_anchor = point
    for obj in context.selected_objects:
        obj.select_set(False)
    point.select_set(True)
    context.view_layer.objects.active = point
    return point


def surface_hit(context: bpy.types.Context, origin: Vector, direction: Vector) -> tuple[bpy.types.Object, Matrix] | None:
    hit, location, normal, _, obj, _ = context.scene.ray_cast(
        context.evaluated_depsgraph_get(), origin, direction,
    )
    if not hit:
        return None
    obj = obj.original
    try:
        physical = _physical_target(obj)
    except ValueError:
        return None
    if physical.type == "MESH" and physical.hide_render and not physical.get("sensor_id"):
        return None
    rotation = normal.to_track_quat("Z", "Y")
    return physical, Matrix.LocRotScale(location, rotation, Vector((1, 1, 1)))


class ROBOT_OT_add_magnetic_point(Operator):
    bl_idname = "robot_library.add_magnetic_point"
    bl_label = "Add Magnetic Point"
    bl_description = "Click any object surface to place an editable magnetic anchor; Esc cancels"
    bl_options = {"REGISTER", "UNDO", "BLOCKING"}

    @classmethod
    def poll(cls, context: bpy.types.Context) -> bool:
        return context.mode == "OBJECT" and context.area is not None and context.area.type == "VIEW_3D"

    def invoke(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        self.region = next(region for region in context.area.regions if region.type == "WINDOW")
        self.region_3d = context.space_data.region_3d
        self.preview = _anchor_visual(context, "Magnetic Point Preview")
        self.preview.hide_viewport = True
        self.hit = None
        context.window_manager.modal_handler_add(self)
        context.workspace.status_text_set("Add Magnetic Point — click a surface | Esc / Right-click: cancel")
        return {"RUNNING_MODAL"}

    def _hover(self, context: bpy.types.Context, event: bpy.types.Event) -> None:
        from bpy_extras import view3d_utils

        preview = self.preview
        if preview is None:
            return
        coordinate = (event.mouse_x - self.region.x, event.mouse_y - self.region.y)
        if not (0 <= coordinate[0] < self.region.width and 0 <= coordinate[1] < self.region.height):
            self.hit = None
        else:
            origin = view3d_utils.region_2d_to_origin_3d(self.region, self.region_3d, coordinate)
            direction = view3d_utils.region_2d_to_vector_3d(self.region, self.region_3d, coordinate)
            self.hit = surface_hit(context, origin, direction)
        preview.hide_viewport = self.hit is None
        if self.hit is not None:
            preview.matrix_world = self.hit[1]
        context.area.tag_redraw()

    def _cleanup(self, context: bpy.types.Context) -> None:
        if self.preview is not None:
            bpy.data.objects.remove(self.preview, do_unlink=True)
            self.preview = None
        context.workspace.status_text_set(None)
        context.area.tag_redraw()

    def modal(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        if event.type in {"ESC", "RIGHTMOUSE"}:
            self._cleanup(context)
            return {"CANCELLED"}
        if event.type == "MOUSEMOVE":
            self._hover(context, event)
        elif event.type == "LEFTMOUSE" and event.value == "PRESS":
            self._hover(context, event)
            if self.hit is None:
                self.report({"WARNING"}, "Click a sensor housing, exported mesh or robot surface")
                return {"RUNNING_MODAL"}
            try:
                point = create_magnetic_point(context, self.hit[0], self.hit[1], self.preview)
            except ValueError as error:
                self.report({"ERROR"}, str(error))
                self._cleanup(context)
                return {"CANCELLED"}
            point.hide_viewport = False
            self.preview = None
            self._cleanup(context)
            return {"FINISHED"}
        elif event.type in {"MIDDLEMOUSE", "WHEELUPMOUSE", "WHEELDOWNMOUSE"}:
            return {"PASS_THROUGH"}
        return {"RUNNING_MODAL"}

    def cancel(self, context: bpy.types.Context) -> None:
        self._cleanup(context)


class ROBOT_OT_add_mount_point(Operator):
    bl_idname = "robot_library.add_mount_point"
    bl_label = "Add Mount Point at Cursor"
    bl_description = "Create a sticky mount point at the 3D cursor; its local +Z points away from the surface"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        target = context.active_object
        if target is None:
            self.report({"WARNING"}, "Select a surface or robot body first")
            return {"CANCELLED"}
        try:
            physical = _physical_target(target)
            if physical.type == "MESH" and physical.hide_render:
                raise ValueError("The mount surface is excluded from MuJoCo export")
        except ValueError as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}
        point = bpy.data.objects.new("Mount Point", None)
        context.collection.objects.link(point)
        point.empty_display_type = "ARROWS"
        point.empty_display_size = 0.08
        point.show_in_front = True
        point["caryam_mount_point"] = True
        point.parent = physical
        point.matrix_parent_inverse = physical.matrix_world.inverted()
        point.matrix_basis = context.scene.cursor.matrix.copy()
        context.view_layer.update()
        for obj in context.selected_objects:
            obj.select_set(False)
        point.select_set(True)
        context.view_layer.objects.active = point
        return {"FINISHED"}


class ROBOT_PT_mounting(Panel):
    bl_label = "Snap & Mount"
    bl_idname = "ROBOT_PT_mounting"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Library"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        layout.prop(context.scene, "robot_mount_distance", text="Snap Distance")
        layout.operator("robot_library.sticky_move", icon="SNAP_ON")
        layout.label(text="Base to surface; no points needed", icon="INFO")
        magnets = layout.box()
        magnets.label(text="Magnetic Points (Optional)")
        magnets.operator("robot_library.add_magnetic_point", icon="ADD")
        magnets.prop(context.scene, "robot_magnet_alignment", text="Alignment")
        obj = context.active_object
        if is_anchor(obj):
            box = magnets.box()
            box.prop(obj, "name", text="Point")
            box.prop(obj, "caryam_anchor_enabled", text="Magnetic")
            box.prop(obj, "caryam_magnet_axis", text="Destination Axis")
            box.prop(obj, "location", text="Offset")
            box.prop(obj, "rotation_euler", text="Rotation")
        try:
            root = moving_object(obj)
        except ValueError:
            root = None
        if root is not None:
            magnets.prop(root.caryam_mount, "source_anchor", text="Moving Point")
        magnets.operator("robot_library.magnetic_move", text="Magnetic Move (Ctrl+Shift+M)", icon="SNAP_ON")
        magnets.label(text="Select a moving point for Magnetic Move", icon="INFO")
        if root is not None and root.caryam_mount.enabled:
            target = root.caryam_mount.target
            layout.label(text=f"Rigid mount: {target.name if target else 'Missing target'}", icon="CONSTRAINT")
            layout.operator("robot_library.detach_mount", icon="UNLINKED")


MOUNT_CLASSES = (
    CaryamRobotMount,
    ROBOT_OT_sticky_move,
    ROBOT_OT_magnetic_move,
    ROBOT_OT_detach_mount,
    ROBOT_OT_add_magnetic_point,
    ROBOT_OT_add_mount_point,
    ROBOT_PT_mounting,
)


def register_keymaps() -> None:
    unregister_keymaps()
    config = bpy.context.window_manager.keyconfigs.addon
    if config is not None:
        keymap = config.keymaps.new(name="Object Mode", space_type="EMPTY")
        item = keymap.keymap_items.new("robot_library.magnetic_move", "M", "PRESS", ctrl=True, shift=True)
        _KEYMAP_ITEMS.append((keymap, item))


def unregister_keymaps() -> None:
    for keymap, item in _KEYMAP_ITEMS:
        keymap.keymap_items.remove(item)
    _KEYMAP_ITEMS.clear()


def register_properties() -> None:
    bpy.types.Object.caryam_mount = PointerProperty(type=CaryamRobotMount)
    bpy.types.Object.caryam_anchor_enabled = BoolProperty(name="Magnetic", default=True)
    bpy.types.Object.caryam_magnet_axis = EnumProperty(
        name="Destination Axis", description="Moving point +Z aligns to this axis in the destination point's local frame (Nearest Axes mode)",
        items=(
            ("AUTO", "Automatic (Nearest)", "Choose the nearest signed destination axis and nearest quarter-turn twist"),
            ("X", "+X", "Align moving +Z with destination +X"), ("NEG_X", "-X", "Align moving +Z with destination -X"),
            ("Y", "+Y", "Align moving +Z with destination +Y"), ("NEG_Y", "-Y", "Align moving +Z with destination -Y"),
            ("Z", "+Z", "Align moving +Z with destination +Z"), ("NEG_Z", "-Z", "Align moving +Z with destination -Z"),
        ), default="AUTO",
    )


def unregister_properties() -> None:
    if hasattr(bpy.types.Object, "caryam_mount"):
        del bpy.types.Object.caryam_mount
    if hasattr(bpy.types.Object, "caryam_anchor_enabled"):
        del bpy.types.Object.caryam_anchor_enabled
    if hasattr(bpy.types.Object, "caryam_magnet_axis"):
        del bpy.types.Object.caryam_magnet_axis
