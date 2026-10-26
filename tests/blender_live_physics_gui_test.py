"""Graphical timer/panel smoke test; run Blender without --background."""
# ruff: noqa: E402

import os
from pathlib import Path
import sys
import tempfile
import time

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import robot_arm_library
from robot_arm_library import blender_addon as addon
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
assets = tempfile.TemporaryDirectory(prefix="caryam-gui-")
source = Path(assets.name) / "servo.xml"
source.write_text('<mujoco><compiler angle="radian"/><worldbody><body name="base">'
                  '<geom size=".04"/><body name="wrist" pos="0 0 .2"><joint name="hinge" range="-1 1"/>'
                  '<geom type="capsule" fromto="0 0 0 .4 0 0" size=".04"/></body></body></worldbody></mujoco>')
item = scene.robot_library.add()
item.robot_id = "servo"
item.name = "Live servo arm"
item.model_path = str(source)
item.source_path = str(source)
root = addon._place_robot(item, Vector((1, 0, 1)))
joint = addon._joint_controls(root)[0]
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
        if stage > 0 and live._session is not None:
            # Viewport/custom-property refreshes can arrive after the physics timer returns.
            for obj in (cube, root, joint):
                obj.update_tag()
            bpy.context.view_layer.update()
            assert live._session is not None, live.status()
        if stage == 0:
            assert bpy.ops.robot_library.live_physics(action="START") == {"FINISHED"}
            stage = 1
        elif stage == 1 and live.state() == "RUNNING" and live._session.time > .4:
            assert cube.matrix_world.translation.z < 1.8
            joint["qpos"] = .6
            joint.update_tag(refresh={"OBJECT"})
            stage = 2
        elif stage == 2 and live.joint_info(joint)[1] > .55:
            bpy.ops.robot_library.live_physics(action="PAUSE")
            stage = 3
        elif stage == 3 and live.state() == "PAUSED":
            bpy.ops.robot_library.joint_window("INVOKE_DEFAULT")
            stage = 4
        elif stage == 4:
            if os.environ.get("CARYAM_SCREENSHOT"):
                bpy.ops.screen.screenshot(filepath=os.environ["CARYAM_SCREENSHOT"])
            bpy.ops.robot_library.live_physics(action="RESET")
            stage = 5
        elif stage == 5 and live.state() == "PAUSED" and live._session.time == 0:
            assert abs(cube.matrix_world.translation.z - 2) < 1e-5
            assert abs(live.joint_info(joint)[1]) < 1e-5
            bpy.ops.robot_library.live_physics(action="RESUME")
            stage = 6
        elif stage == 6 and live.state() == "RUNNING" and live._session.time > .1:
            bpy.ops.robot_library.live_physics(action="STOP")
            assert abs(cube.matrix_world.translation.z - 2) < 1e-5
            assert callbacks > 4
            print("GRAPHICAL LIVE PHYSICS SMOKE PASSED", flush=True)
            robot_arm_library.unregister()
            assets.cleanup()
            bpy.ops.wm.quit_blender()
            return None
        elif stage > 0:
            assert live.active(), live.status()
    except Exception:
        import traceback
        traceback.print_exc()
        print("GRAPHICAL LIVE PHYSICS SMOKE FAILED", flush=True)
        live.stop(wait=True)
        assets.cleanup()
        bpy.ops.wm.quit_blender()
        return None
    return .1


bpy.app.timers.register(check, first_interval=.5)
