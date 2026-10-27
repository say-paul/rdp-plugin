"""Exercise Ctrl+Shift+M using --enable-event-simulate in a fresh Blender window."""
# ruff: noqa: E402

import math
import os
from pathlib import Path
import sys
import time
import traceback

import addon_utils
import bpy
from bpy_extras.view3d_utils import location_3d_to_region_2d
from mathutils import Matrix, Quaternion, Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import mounting

assert not bpy.app.background
bpy.context.preferences.view.show_splash = False
assert addon_utils.enable("robot_arm_library", default_set=False) is robot_arm_library
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.mesh.primitive_cube_add(size=.1)
moving = bpy.context.object
source = mounting.create_magnetic_point(bpy.context, moving, Matrix.Translation((0, 0, .05)))
bpy.ops.mesh.primitive_cube_add(size=.1, location=(.5, 0, 0))
target_body = bpy.context.object
target_rotation = Quaternion((0, 1, 0), math.radians(20))
target = mounting.create_magnetic_point(
    bpy.context, target_body, Matrix.LocRotScale(Vector((.5, 0, .05)), target_rotation, Vector((1, 1, 1))),
)
target.caryam_magnet_axis = "NEG_X"
bpy.ops.object.select_all(action="DESELECT")
source.select_set(True)
bpy.context.view_layer.objects.active = source
area = next(area for area in bpy.context.screen.areas if area.type == "VIEW_3D")
region = next(region for region in area.regions if region.type == "WINDOW")
space = area.spaces.active
space.show_region_ui = False
space.region_3d.view_rotation = Quaternion((1, 0, 0, 0))
space.region_3d.view_location = (.25, 0, 0)
space.region_3d.view_distance = 1.5
space.region_3d.view_perspective = "ORTHO"
window = bpy.context.window
stage = 0
started = time.monotonic()
coordinates = {}


def event(kind, value, coordinate, **modifiers):
    window.event_simulate(type=kind, value=value, x=int(coordinate.x + region.x), y=int(coordinate.y + region.y), **modifiers)


def check():
    global stage
    try:
        assert time.monotonic() - started < 20, f"Magnetic shortcut timed out at stage {stage}"
        if stage == 0:
            coordinates["source"] = location_3d_to_region_2d(region, space.region_3d, source.matrix_world.translation)
            coordinates["target"] = location_3d_to_region_2d(region, space.region_3d, target.matrix_world.translation)
            event("MOUSEMOVE", "NOTHING", coordinates["source"])
            event("M", "PRESS", coordinates["source"], ctrl=True, shift=True)
            event("M", "RELEASE", coordinates["source"])
        elif stage == 1:
            event("MOUSEMOVE", "NOTHING", coordinates["target"])
        elif stage == 2:
            assert (source.matrix_world.translation - target.matrix_world.translation).length < 1e-4, "The shortcut did not start cursor-centered magnetic placement"
            event("LEFTMOUSE", "PRESS", coordinates["target"])
            event("LEFTMOUSE", "RELEASE", coordinates["target"])
        elif stage == 3:
            assert moving.parent is target and moving.caryam_mount.enabled
            normal = source.matrix_world.to_quaternion() @ Vector((0, 0, 1))
            assert normal.dot(target_rotation @ Vector((-1, 0, 0))) > .9999
            event("M", "PRESS", coordinates["target"], ctrl=True, shift=True)
            event("M", "RELEASE", coordinates["target"])
        elif stage == 4:
            coordinates["away"] = coordinates["target"] + Vector((100, 0))
            event("MOUSEMOVE", "NOTHING", coordinates["away"])
        elif stage == 5:
            assert (source.matrix_world.translation - target.matrix_world.translation).length > .1
            event("ESC", "PRESS", coordinates["away"])
            event("ESC", "RELEASE", coordinates["away"])
        else:
            assert moving.parent is target and moving.caryam_mount.enabled
            assert (source.matrix_world.translation - target.matrix_world.translation).length < 1e-4
            addon_utils.disable("robot_arm_library", default_set=False)
            assert not mounting._KEYMAP_ITEMS
            print("GRAPHICAL MAGNETIC SHORTCUT, AXIS OVERRIDE AND CANCEL PASSED", flush=True)
            os._exit(0)
        stage += 1
        area.tag_redraw()
    except Exception:
        traceback.print_exc()
        print("GRAPHICAL MAGNETIC SHORTCUT FAILED", flush=True)
        addon_utils.disable("robot_arm_library", default_set=False)
        os._exit(1)
    return .25


bpy.app.timers.register(check, first_interval=.7)
