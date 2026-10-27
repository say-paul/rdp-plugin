"""Generic sensor housings and aiming guides for Blender authoring only."""

from __future__ import annotations

import json
import math

import bpy
from bpy.app.handlers import persistent
from bpy.props import BoolProperty
from mathutils import Matrix, Vector

from .sensor_spec import CAMERA_CATEGORIES, CameraSensorSpec, LidarSensorSpec, SensorDefinition
from .setup_parser import discover_sensor_definitions

PREVIEW_KEY = "caryam_sensor_preview"
HOUSING_KEY = "caryam_sensor_housing"
PREVIEW_VERSION = 3
_DRAW_HANDLE = None
_OVERLAY_MENU_REGISTERED = False
GUIDE_LENGTH = 0.2
COLORS = (
    (0.16, 0.19, 0.23, 1.0),
    (0.025, 0.04, 0.065, 1.0),
    (0.08, 0.55, 0.85, 1.0),
    (0.9, 0.08, 0.06, 1.0),
    (0.08, 0.8, 0.16, 1.0),
    (0.1, 0.25, 1.0, 1.0),
    (1.0, 0.65, 0.05, 1.0),
)


class _Geometry:
    def __init__(self):
        self.vertices = []
        self.faces = []
        self.materials = []

    def add(self, vertices, faces, material):
        offset = len(self.vertices)
        self.vertices.extend(vertices)
        self.faces.extend(tuple(offset + index for index in face) for face in faces)
        self.materials.extend([material] * len(faces))

    def box(self, center, half_size, material):
        vertices = [
            tuple(center[axis] + signs[axis] * half_size[axis] for axis in range(3))
            for signs in ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                          (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1))
        ]
        self.add(vertices, [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                            (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)], material)

    def cylinder(self, start, end, radius, material, *, cone=False):
        start, end = Vector(start), Vector(end)
        rotation = Vector((0, 0, 1)).rotation_difference(end - start)
        count = 24
        ring = [rotation @ Vector((radius * math.cos(i * math.tau / count),
                                   radius * math.sin(i * math.tau / count), 0)) for i in range(count)]
        vertices = [start + point for point in ring]
        faces = [tuple(reversed(range(count)))]
        if cone:
            vertices.append(end)
            faces.extend((i, (i + 1) % count, count) for i in range(count))
        else:
            vertices.extend(end + point for point in ring)
            faces.append(tuple(range(count, count * 2)))
            faces.extend((i, (i + 1) % count, (i + 1) % count + count, i + count) for i in range(count))
        self.add(vertices, faces, material)


def _material(index):
    name = f"Caryam Sensor Preview {index}"
    material = bpy.data.materials.get(name)
    if material is None:
        material = bpy.data.materials.new(name)
        material.diffuse_color = COLORS[index]
    return material


def module_mesh(category):
    geometry = _Geometry()
    if category in CAMERA_CATEGORIES:
        geometry.box((0, 0, 0.02), (0.045, 0.014, 0.014), 0)
        geometry.box((0, 0, 0.005), (0.043, 0.012, 0.002), 1)
        for x in ((0,) if category == "rgb_camera" else (-0.027, 0.027)):
            geometry.cylinder((x, 0, 0.006), (x, 0, -0.005), 0.01, 1)
            geometry.cylinder((x, 0, -0.005), (x, 0, -0.006), 0.007, 2)
    elif category == "imu":
        geometry.box((0, 0, -0.003), (0.02, 0.015, 0.002), 4)
        geometry.box((0, 0, 0.001), (0.006, 0.006, 0.003), 1)
        for x in (-0.016, 0.016):
            for y in (-0.011, 0.011):
                geometry.cylinder((x, y, -0.001), (x, y, 0.001), 0.002, 6)
    else:
        geometry.cylinder((0, 0, -0.034), (0, 0, -0.028), 0.04, 0)
        geometry.cylinder((0, 0, -0.028), (0, 0, -0.009), 0.034, 0)
        geometry.cylinder((0, 0, -0.009), (0, 0, 0.009), 0.035, 1)
        geometry.cylinder((0, 0, 0.009), (0, 0, 0.014), 0.033, 2)
    mesh = bpy.data.meshes.new("Caryam Sensor Module")
    mesh.from_pydata(geometry.vertices, [], geometry.faces)
    for index in range(len(COLORS)):
        mesh.materials.append(_material(index))
    for polygon, material in zip(mesh.polygons, geometry.materials):
        polygon.material_index = material
    mesh.update()
    return mesh


def guide_lines(definition):
    """Return local-space guides; the short preview length is not sensor range."""
    if definition.category in CAMERA_CATEGORIES:
        spec = definition.camera or CameraSensorSpec()
        half_height = GUIDE_LENGTH * math.tan(math.radians(spec.vertical_fov_degrees()) / 2)
        half_width = half_height * spec.resolution_width / spec.resolution_height
        corners = [(x * half_width, y * half_height, -GUIDE_LENGTH)
                   for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        return [[(0, 0, 0), corner] for corner in corners] + [corners + [corners[0]]]
    if definition.category == "imu":
        return []
    spec = definition.lidar or LidarSensorSpec()
    pitches = (-spec.vertical_fov / 2, spec.vertical_fov / 2) if spec.channels > 1 and spec.vertical_fov else (0,)
    lines = []
    for pitch in pitches:
        pitch = math.radians(pitch)
        arc = [
            (GUIDE_LENGTH * math.cos(pitch) * math.cos(yaw),
             GUIDE_LENGTH * math.cos(pitch) * math.sin(yaw), GUIDE_LENGTH * math.sin(pitch))
            for yaw in (math.radians(-spec.horizontal_fov / 2 + spec.horizontal_fov * i / 48) for i in range(49))
        ]
        lines.append(arc)
        if spec.horizontal_fov < 360:
            lines.extend([[(0, 0, 0), arc[0]], [(0, 0, 0), arc[-1]]])
    return lines


def selected_mount(obj):
    while obj is not None:
        if obj.get("sensor_id"):
            return obj
        obj = obj.parent
    return None


def mount_frame(mount, root):
    parent = mount.parent
    while parent is not None and parent is not root and "mujoco_body_id" not in parent and not parent.name.startswith("Body: "):
        parent = parent.parent
    return parent if parent is not None else root


def housing_object(mount):
    if mount.type == "MESH":
        return mount
    return next((child for child in mount.children if child.get(HOUSING_KEY)), None)


def _remove_helpers(mount):
    for child in reversed(list(mount.children_recursive)):
        if child.get(PREVIEW_KEY):
            data = child.data
            kind = child.type
            bpy.data.objects.remove(child, do_unlink=True)
            if data is not None and data.users == 0:
                (bpy.data.meshes if kind == "MESH" else bpy.data.curves).remove(data)


def _child(mount, name, data):
    obj = bpy.data.objects.new(name, data)
    mount.users_collection[0].objects.link(obj)
    obj.parent = mount
    obj.matrix_parent_inverse = Matrix.Identity(4)
    obj[PREVIEW_KEY] = True
    obj.hide_render = True
    obj.show_in_front = False
    return obj


def update_preview(mount, definition):
    housing = housing_object(mount)
    # Versions before 3 forced X-ray on rather than recording an intentional choice.
    in_front = bool(housing is not None and housing.show_in_front and mount.get("caryam_sensor_visual_version", 0) >= 3)
    mesh = module_mesh(definition.category)
    _remove_helpers(mount)
    if mount.type == "MESH":
        previous = mount.data
        mount.data = mesh
        if mount.get("caryam_sensor_visual_version") and previous.users == 0:
            bpy.data.meshes.remove(previous)
    else:
        mount.empty_display_type = "PLAIN_AXES"
        mount.empty_display_size = 0.03
        housing = _child(mount, "Module: " + mount.name, mesh)
        housing[HOUSING_KEY] = True
        housing.hide_select = True
        housing.show_in_front = in_front
    mount["caryam_sensor_visual_version"] = PREVIEW_VERSION
    mount.hide_render = True
    mount.show_in_front = in_front
    mount.show_name = True
    mount.show_axis = False
    mount.color = COLORS[0]


def create_mount(collection, name, definition):
    mount = bpy.data.objects.new(name, bpy.data.meshes.new("Caryam Sensor Module"))
    collection.objects.link(mount)
    mount["caryam_sensor_visual_version"] = PREVIEW_VERSION
    mount.lock_scale = (True, True, True)
    update_preview(mount, definition)
    return mount


def sensor_definition(mount):
    source_path = str(mount.get("sensor_source_path", ""))
    payload = str(mount.get("sensor_definition_json", ""))
    if payload:
        return SensorDefinition.from_dict(str(mount["sensor_id"]), json.loads(payload), source_path)
    return next(sensor for sensor in discover_sensor_definitions(source_path) if sensor.sensor_id == str(mount["sensor_id"]))


def _arrow_lines(direction, length):
    direction = Vector(direction).normalized()
    tip = direction * length
    neck = tip * .78
    side = direction.orthogonal().normalized() * length * .07
    up = direction.cross(side)
    return [Vector((0, 0, 0)), tip, tip, neck + side, tip, neck - side, tip, neck + up, tip, neck - up]


def overlay_lines(definition, *, axes=True, aim=True, fov=True):
    if axes:
        for direction, index in (((1, 0, 0), 3), ((0, 1, 0), 4), ((0, 0, 1), 5)):
            yield COLORS[index], _arrow_lines(direction, .07)
        origin = [Vector(direction) * sign * .003 for direction in ((1, 0, 0), (0, 1, 0), (0, 0, 1)) for sign in (-1, 1)]
        yield COLORS[6], origin
    if aim and definition.category != "imu":
        direction = (0, 0, -1) if definition.category in CAMERA_CATEGORIES else (1, 0, 0)
        yield COLORS[6], _arrow_lines(direction, .13)
    if fov:
        edges = [point for line in guide_lines(definition) for edge in zip(line, line[1:]) for point in edge]
        if edges:
            yield COLORS[2], edges


def overlay_batches(context):
    space = context.space_data
    if space is None or space.type != "VIEW_3D" or not space.overlay.show_overlays or not space.overlay.show_extras:
        return
    axes, aim, fov = (getattr(context.scene, name) for name in ("caryam_sensor_axes", "caryam_sensor_aim", "caryam_sensor_fov"))
    if not any((axes, aim, fov)):
        return
    for mount in context.scene.objects:
        if not mount.get("sensor_id") or not mount.visible_get(view_layer=context.view_layer, viewport=space):
            continue
        try:
            definition = sensor_definition(mount)
        except (OSError, ValueError, StopIteration, TypeError, KeyError):
            continue
        for color, points in overlay_lines(definition, axes=axes, aim=aim, fov=fov):
            yield mount, color, [mount.matrix_world @ Vector(point) for point in points]


def _draw_overlays():
    batches = list(overlay_batches(bpy.context))
    if not batches:
        return
    import gpu
    from gpu_extras.batch import batch_for_shader

    shader = gpu.shader.from_builtin("UNIFORM_COLOR" if bpy.app.version >= (4, 0, 0) else "3D_UNIFORM_COLOR")
    blend = gpu.state.blend_get()
    depth = gpu.state.depth_test_get()
    depth_mask = gpu.state.depth_mask_get()
    width = gpu.state.line_width_get()
    try:
        gpu.state.blend_set("ALPHA")
        gpu.state.depth_mask_set(False)
        gpu.state.line_width_set(1.5)
        for mount, color, points in batches:
            housing = housing_object(mount)
            gpu.state.depth_test_set("NONE" if housing is not None and housing.show_in_front else "LESS_EQUAL")
            shader.bind()
            shader.uniform_float("color", color)
            batch_for_shader(shader, "LINES", {"pos": points}).draw(shader)
    finally:
        gpu.state.blend_set(blend)
        gpu.state.depth_test_set(depth)
        gpu.state.depth_mask_set(depth_mask)
        gpu.state.line_width_set(width)


def _redraw_viewports(_self, context):
    if context is not None and context.window_manager is not None:
        for window in context.window_manager.windows:
            for area in window.screen.areas:
                if area.type == "VIEW_3D":
                    area.tag_redraw()


def _draw_overlay_settings(self, context):
    column = self.layout.column()
    overlay = context.space_data.overlay
    column.active = overlay.show_overlays and overlay.show_extras
    column.label(text="Caryam Sensors")
    column.prop(context.scene, "caryam_sensor_axes", text="Axes / Origin")
    column.prop(context.scene, "caryam_sensor_aim", text="Aim Direction")
    column.prop(context.scene, "caryam_sensor_fov", text="Field of View")


@persistent
def _upgrade_previews(_unused=None):
    for mount in list(bpy.data.objects):
        if mount.library is not None or not mount.get("sensor_id") or mount.get("caryam_sensor_visual_version", 0) >= PREVIEW_VERSION:
            continue
        try:
            definition = sensor_definition(mount)
        except (OSError, ValueError, StopIteration, TypeError, KeyError):
            continue
        update_preview(mount, definition)


def register():
    global _DRAW_HANDLE, _OVERLAY_MENU_REGISTERED
    unregister()
    for name, label in (("caryam_sensor_axes", "Sensor Axes"), ("caryam_sensor_aim", "Sensor Aim Direction"), ("caryam_sensor_fov", "Sensor Field of View")):
        setattr(bpy.types.Scene, name, BoolProperty(name=label, default=True, update=_redraw_viewports))
    bpy.types.VIEW3D_PT_overlay.append(_draw_overlay_settings)
    _OVERLAY_MENU_REGISTERED = True
    bpy.app.handlers.load_post.append(_upgrade_previews)
    # Blender restricts scene data while enabling an add-on.
    bpy.app.timers.register(_upgrade_previews, first_interval=.1)
    if not bpy.app.background:
        _DRAW_HANDLE = bpy.types.SpaceView3D.draw_handler_add(_draw_overlays, (), "WINDOW", "POST_VIEW")
    _redraw_viewports(None, bpy.context)


def unregister():
    global _DRAW_HANDLE, _OVERLAY_MENU_REGISTERED
    if bpy.app.timers.is_registered(_upgrade_previews):
        bpy.app.timers.unregister(_upgrade_previews)
    if _DRAW_HANDLE is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_DRAW_HANDLE, "WINDOW")
        _DRAW_HANDLE = None
    if _OVERLAY_MENU_REGISTERED:
        bpy.types.VIEW3D_PT_overlay.remove(_draw_overlay_settings)
        _OVERLAY_MENU_REGISTERED = False
    for handler in list(bpy.app.handlers.load_post):
        if handler.__module__ == __name__:
            bpy.app.handlers.load_post.remove(handler)
    for name in ("caryam_sensor_axes", "caryam_sensor_aim", "caryam_sensor_fov"):
        if hasattr(bpy.types.Scene, name):
            delattr(bpy.types.Scene, name)
    _redraw_viewports(None, bpy.context)
