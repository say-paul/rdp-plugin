"""Sensor guides are viewport overlays, never mountable scene geometry."""
# ruff: noqa: E402

import json
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import mounting, sensor_visuals as visuals
from robot_arm_library.sensor_spec import CameraSensorSpec, SensorDefinition


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    definition = SensorDefinition("camera", "Camera", "rgb_camera", "snapshot.json", camera=CameraSensorSpec())
    mount = visuals.create_mount(scene.collection, "Sensor: Camera", definition)
    mount["sensor_id"] = definition.sensor_id
    mount["sensor_category"] = definition.category
    mount["sensor_definition_json"] = json.dumps(definition.to_dict())
    bpy.context.view_layer.update()

    assert mounting.surface_hit(bpy.context, Vector((.06, 0, 1)), Vector((0, 0, -1))) is None, "An axis arrow is still a mountable solid"
    assert mounting.surface_hit(bpy.context, Vector((1, 0, -.1)), Vector((-1, 0, 0))) is None, "The aim arrow is still a mountable solid"
    hit = mounting.surface_hit(bpy.context, Vector((.01, .005, 1)), Vector((0, 0, -1)))
    assert hit is not None and hit[0] is mount
    source = mounting.create_magnetic_point(bpy.context, *hit)
    assert source.parent is mount
    assert not any(obj.type == "CURVE" and obj.get(visuals.PREVIEW_KEY) for obj in scene.objects)
    assert not mount.show_axis

    space = next(area.spaces.active for area in bpy.context.screen.areas if area.type == "VIEW_3D")
    context = SimpleNamespace(scene=scene, view_layer=bpy.context.view_layer, space_data=space)
    space.overlay.show_overlays = True
    space.overlay.show_extras = True
    assert list(visuals.overlay_batches(context))
    space.overlay.show_overlays = False
    assert not list(visuals.overlay_batches(context))
    assert not mount.hide_get() and mounting.surface_hit(bpy.context, Vector((.01, .005, 1)), Vector((0, 0, -1)))
    space.overlay.show_overlays = True
    space.overlay.show_extras = False
    assert not list(visuals.overlay_batches(context))
    space.overlay.show_extras = True
    scene.caryam_sensor_axes = False
    scene.caryam_sensor_aim = False
    scene.caryam_sensor_fov = False
    assert not list(visuals.overlay_batches(context))
    scene.caryam_sensor_fov = True
    batches = list(visuals.overlay_batches(context))
    assert len(batches) == 1
    assert len(batches[0][2]) == 16, "A camera frustum should contain eight wire edges"
    mount.hide_set(True)
    assert not list(visuals.overlay_batches(context))
    mount.hide_set(False)
    mount.location = (1, 2, 3)
    bpy.context.view_layer.update()
    assert (list(visuals.overlay_batches(context))[0][2][0] - mount.matrix_world.translation).length < 1e-6

    # Old combined meshes and solid guide children are upgraded when a saved file is opened.
    old_mesh = mount.data
    vertices = [tuple(vertex.co) for vertex in old_mesh.vertices]
    faces = [tuple(face.vertices) for face in old_mesh.polygons]
    offset = len(vertices)
    vertices.extend([(.055, -.001, 0), (.065, -.001, 0), (.065, .001, 0), (.055, .001, 0)])
    faces.append(tuple(offset + index for index in range(4)))
    old_mesh.clear_geometry()
    old_mesh.from_pydata(vertices, [], faces)
    old_mesh.update()
    mount["caryam_sensor_visual_version"] = 1
    curve = bpy.data.curves.new("Legacy solid frustum", "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = .001
    guide = visuals._child(mount, "Legacy aim guide", curve)
    try:
        mounting.create_magnetic_point(bpy.context, guide, guide.matrix_world)
    except ValueError as error:
        assert "not mount surfaces" in str(error)
    else:
        raise AssertionError("A legacy guide was accepted as a magnetic surface")
    mount.show_axis = True
    bpy.context.view_layer.update()
    assert mounting.surface_hit(bpy.context, Vector((1.06, 2, 4)), Vector((0, 0, -1))) is not None
    names = mount.name, source.name
    before = mount.matrix_world.copy()
    source_world = source.matrix_world.copy()
    with tempfile.TemporaryDirectory() as directory:
        blend = str(Path(directory) / "old_sensor.blend")
        bpy.ops.wm.save_as_mainfile(filepath=blend)
        bpy.ops.wm.open_mainfile(filepath=blend)
        mount, source = [bpy.data.objects[name] for name in names]
        assert mount.get("caryam_sensor_visual_version") == visuals.PREVIEW_VERSION
        assert source.parent is mount and list(mount.children) == [source]
        assert not mount.show_axis
        assert mount["sensor_definition_json"] == json.dumps(definition.to_dict())
        assert max(abs(mount.matrix_world[r][c] - before[r][c]) for r in range(4) for c in range(4)) < 1e-5
        assert max(abs(source.matrix_world[r][c] - source_world[r][c]) for r in range(4) for c in range(4)) < 1e-5
        assert mounting.surface_hit(bpy.context, Vector((1.06, 2, 4)), Vector((0, 0, -1))) is None
        assert bpy.context.scene.caryam_sensor_fov and not bpy.context.scene.caryam_sensor_axes
    robot_arm_library.register()
    assert bpy.app.handlers.load_post.count(visuals._upgrade_previews) == 1
    print("Sensor housing-only picking, viewport overlay controls and saved-preview migration passed")
finally:
    robot_arm_library.unregister()
