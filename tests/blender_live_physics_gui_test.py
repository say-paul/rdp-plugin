"""Graphical timer/panel smoke test; run Blender without --background."""
# ruff: noqa: E402

import os
from pathlib import Path
import sys
import time

import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import robot_arm_library
from robot_arm_library.physics import blender as live

robot_arm_library.register()
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
bpy.ops.mesh.primitive_cube_add(size=.4, location=(0, 0, 2))
cube = bpy.context.object
bpy.ops.rigidbody.object_add()
cube.rigid_body.collision_shape = "BOX"
bpy.ops.object.select_all(action="DESELECT")
scene = bpy.context.scene
scene.robot_mujoco_python = os.environ.get("CARYAM_TEST_MUJOCO_PYTHON", str(ROOT / ".venv/bin/python"))
for area in bpy.context.screen.areas:
    if area.type == "PROPERTIES":
        area.spaces.active.context = "SCENE"
    if area.type == "VIEW_3D":
        area.spaces.active.show_region_ui = True
        area.spaces.active.region_3d.view_distance = 6

stage = 0
callbacks = 0
started = time.monotonic()


def check():
    global stage, callbacks
    callbacks += 1
    try:
        assert time.monotonic() - started < 25, f"GUI smoke timed out: {live.status()}"
        if stage == 0:
            assert bpy.ops.robot_library.live_physics(action="START") == {"FINISHED"}
            stage = 1
        elif stage == 1 and live.state() == "RUNNING" and live._session.time > .4:
            assert cube.matrix_world.translation.z < 1.8
            bpy.ops.robot_library.live_physics(action="PAUSE")
            stage = 2
        elif stage == 2 and live.state() == "PAUSED":
            if os.environ.get("CARYAM_SCREENSHOT"):
                bpy.ops.screen.screenshot(filepath=os.environ["CARYAM_SCREENSHOT"])
            bpy.ops.robot_library.live_physics(action="RESET")
            stage = 3
        elif stage == 3 and live.state() == "PAUSED" and live._session.time == 0:
            assert abs(cube.matrix_world.translation.z - 2) < 1e-5
            bpy.ops.robot_library.live_physics(action="RESUME")
            stage = 4
        elif stage == 4 and live.state() == "RUNNING" and live._session.time > .1:
            bpy.ops.robot_library.live_physics(action="STOP")
            assert abs(cube.matrix_world.translation.z - 2) < 1e-5
            assert callbacks > 4
            print("GRAPHICAL LIVE PHYSICS SMOKE PASSED", flush=True)
            robot_arm_library.unregister()
            bpy.ops.wm.quit_blender()
            return None
        elif stage > 0:
            assert live.active(), live.status()
    except Exception:
        import traceback
        traceback.print_exc()
        print("GRAPHICAL LIVE PHYSICS SMOKE FAILED", flush=True)
        live.stop()
        bpy.ops.wm.quit_blender()
        return None
    return .1


bpy.app.timers.register(check, first_interval=.5)
