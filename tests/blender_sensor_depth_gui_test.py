"""Opposite-side viewport occlusion and opt-in X-ray for sensor housings/guides."""
# ruff: noqa: E402

import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import traceback

import addon_utils
import bpy
from mathutils import Quaternion

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import sensor_visuals as visuals
from robot_arm_library.sensor_spec import CameraSensorSpec, SensorDefinition

assert not bpy.app.background
bpy.context.preferences.view.show_splash = False
assert addon_utils.enable("robot_arm_library", default_set=False) is robot_arm_library
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
definition = SensorDefinition("camera", "Camera", "rgb_camera", "snapshot.json", camera=CameraSensorSpec())
mount = visuals.create_mount(scene.collection, "Sensor: Camera", definition)
mount["sensor_id"] = definition.sensor_id
mount["sensor_category"] = definition.category
mount["sensor_definition_json"] = json.dumps(definition.to_dict())
mount.show_name = False
mount.data.materials[0].diffuse_color = (.9, .02, .02, 1)
pose = mount.matrix_world.copy()
bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, .25))
occluder = bpy.context.object
occluder.scale = (.6, .6, .1)
bpy.ops.object.select_all(action="DESELECT")
scene.caryam_sensor_axes = False
scene.caryam_sensor_aim = False
area = next(area for area in bpy.context.screen.areas if area.type == "VIEW_3D")
space = area.spaces.active
space.show_region_ui = False
space.show_gizmo = False
space.overlay.show_floor = False
space.overlay.show_axis_x = False
space.overlay.show_axis_y = False
space.overlay.show_cursor = False
space.overlay.show_text = False
space.overlay.show_relationship_lines = False
space.shading.type = "SOLID"
space.shading.color_type = "MATERIAL"
space.shading.light = "FLAT"
space.shading.background_type = "VIEWPORT"
space.shading.background_color = (.3, .3, .3)
space.region_3d.view_location = (0, 0, 0)
space.region_3d.view_rotation = Quaternion((1, 0, 0, 0))
space.region_3d.view_distance = 1.2
space.region_3d.view_perspective = "ORTHO"
assets = tempfile.TemporaryDirectory(prefix="caryam-sensor-depth-")
frames = {}
stage = 0
legacy = None
started = time.monotonic()


def capture():
    path = str(Path(assets.name) / "frame.png")
    bpy.ops.screen.screenshot(filepath=path)
    image = bpy.data.images.load(path)
    width = image.size[0]
    pixels = image.pixels[:]
    bpy.data.images.remove(image)
    region = next(region for region in area.regions if region.type == "WINDOW")
    red = blue = 0
    for y in range(region.y + 20, region.y + region.height - 20):
        for x in range(region.x + 70, region.x + region.width - 20):
            index = (y * width + x) * 4
            r, g, b = pixels[index:index + 3]
            red += r > .6 and r > g * 2 and r > b * 2
            blue += b > .5 and b > r * 2 and b > g * 1.2
    return red, blue


def check():
    global stage, legacy
    try:
        assert time.monotonic() - started < 25, f"Depth smoke timed out: {frames}"
        frames[stage] = capture()
        red, blue = frames[stage]
        # A few native viewport accent pixels remain even with sensor guides hidden.
        assert max(abs(mount.matrix_world[r][c] - pose[r][c]) for r in range(4) for c in range(4)) < 1e-6
        if stage == 0:
            assert red == 0 and blue < 32, f"The hidden camera/guide draws through the occluder: {frames}"
            mount.show_in_front = True
        elif stage == 1:
            assert red > 50 and blue > 50, f"Explicit X-ray did not show the housing and wire guide: {frames}"
            mount.show_in_front = False
            space.region_3d.view_rotation = Quaternion((1, 0, 0), math.pi)
        elif stage == 2:
            # The black lens plate hides most of the red housing from this side.
            assert red > 10 and blue > 50, f"The camera was not visible from the unoccluded side: {frames}"
            space.region_3d.view_rotation = Quaternion((1, 0, 0, 0))
        elif stage == 3:
            assert red == 0 and blue < 32, f"Orbiting back did not restore occlusion: {frames}"
            mount.hide_set(True)
            legacy = bpy.data.objects.new("Sensor: Legacy", None)
            scene.collection.objects.link(legacy)
            legacy["sensor_id"] = definition.sensor_id
            legacy["sensor_category"] = definition.category
            legacy["sensor_definition_json"] = mount["sensor_definition_json"]
            visuals.update_preview(legacy, definition)
            legacy.show_name = False
        elif stage == 4:
            assert red == 0 and blue < 32, f"Legacy sensor housing bypasses depth: {frames}"
            visuals.housing_object(legacy).show_in_front = True
        elif stage == 5:
            assert red > 50 and blue > 50, f"Legacy housing X-ray control does not control both previews: {frames}"
            visuals.housing_object(legacy).show_in_front = False
        else:
            assert red == 0 and blue < 32, f"Legacy X-ray did not turn off: {frames}"
            addon_utils.disable("robot_arm_library", default_set=False)
            assets.cleanup()
            print(f"GRAPHICAL SENSOR DEPTH, OPPOSITE VIEWS AND X-RAY PASSED: {frames}", flush=True)
            os._exit(0)
        stage += 1
        area.tag_redraw()
    except Exception:
        traceback.print_exc()
        print("GRAPHICAL SENSOR DEPTH FAILED", flush=True)
        addon_utils.disable("robot_arm_library", default_set=False)
        assets.cleanup()
        os._exit(1)
    return .3


bpy.app.timers.register(check, first_interval=.8)
