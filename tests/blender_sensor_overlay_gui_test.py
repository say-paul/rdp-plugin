"""Graphical sensor overlay smoke test; run Blender without --background."""
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
from mathutils import Euler

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import sensor_visuals as visuals
from robot_arm_library.sensor_spec import CameraSensorSpec, SensorDefinition

assert not bpy.app.background, "Run this test without --background"
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
mount["caryam_sensor_visual_version"] = 1
mount.show_name = False
area = next(area for area in bpy.context.screen.areas if area.type == "VIEW_3D")
space = area.spaces.active
space.show_region_ui = False
space.show_gizmo = False
space.overlay.show_floor = False
space.overlay.show_axis_x = False
space.overlay.show_axis_y = False
space.overlay.show_cursor = False
space.overlay.show_text = False
space.shading.type = "SOLID"
space.shading.color_type = "MATERIAL"
space.shading.light = "FLAT"
space.shading.background_type = "VIEWPORT"
space.shading.background_color = (.3, .3, .3)
space.region_3d.view_location = (0, 0, -.05)
space.region_3d.view_rotation = Euler((math.radians(150), 0, math.radians(30))).to_quaternion()
space.region_3d.view_distance = .6
stage = 0
frames = {}
started = time.monotonic()
assets = tempfile.TemporaryDirectory(prefix="caryam-sensor-overlay-")


def capture():
    # Read the composed window: POST_PIXEL's framebuffer can contain only the overlay pass.
    path = str(Path(assets.name) / "frame.png")
    bpy.ops.screen.screenshot(filepath=path)
    image = bpy.data.images.load(path)
    width = image.size[0]
    pixels = image.pixels[:]
    bpy.data.images.remove(image)
    region = next(region for region in area.regions if region.type == "WINDOW")
    green = blue = colored = 0
    for y in range(region.y + 20, region.y + region.height - 20):
        for x in range(region.x + 70, region.x + region.width - 20):
            index = (y * width + x) * 4
            red, g, b = pixels[index:index + 3]
            green += g > .6 and g > red * 2 and g > b * 2
            blue += b > .5 and b > red * 2 and b > g * 1.2
            colored += max(red, g, b) - min(red, g, b) > .02
    return green, blue, colored


def check():
    global stage
    try:
        assert time.monotonic() - started < 25, f"Overlay drawing timed out: {frames}"
        frames[stage] = capture()
        green, blue, colored = frames[stage]
        if stage == 0:
            assert mount["caryam_sensor_visual_version"] == visuals.PREVIEW_VERSION
            assert not bpy.app.timers.is_registered(visuals._upgrade_previews)
            if os.environ.get("CARYAM_SCREENSHOT"):
                bpy.ops.screen.screenshot(filepath=os.environ["CARYAM_SCREENSHOT"])
            assert green > 10, f"The RGB axes were not rendered: {frames}"
            space.overlay.show_overlays = False
        elif stage == 1:
            assert green == 0 and colored > 20, f"Overlays did not hide independently of the housing: {frames}"
            space.overlay.show_overlays = True
            scene.caryam_sensor_axes = False
            scene.caryam_sensor_aim = False
        elif stage == 2:
            assert green == 0 and blue > frames[1][1] + 10, f"The wire FOV was not rendered independently: {frames}"
            scene.caryam_sensor_fov = False
        else:
            # Native outlines/origins can differ with Blender's master overlay switch.
            assert green == 0 and blue < frames[1][1] + 32, f"Sensor guides remain visible: {frames}"
            assert abs(colored - frames[1][2]) < max(50, frames[1][2] * .05), f"Disabling sensor guides changed the housing: {frames}"
            addon_utils.disable("robot_arm_library", default_set=False)
            assert visuals._DRAW_HANDLE is None
            assets.cleanup()
            print(f"GRAPHICAL SENSOR OVERLAY SMOKE PASSED: {frames}", flush=True)
            # Do not replace the user's shared quit.blend recovery file.
            os._exit(0)
        stage += 1
        area.tag_redraw()
    except Exception:
        traceback.print_exc()
        print("GRAPHICAL SENSOR OVERLAY SMOKE FAILED", flush=True)
        addon_utils.disable("robot_arm_library", default_set=False)
        assets.cleanup()
        os._exit(1)
    return .3


bpy.app.timers.register(check, first_interval=.8)
