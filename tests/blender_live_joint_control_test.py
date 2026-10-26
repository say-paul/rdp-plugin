"""Blender sliders command physical servos, not viewport/qpos teleportation."""
# ruff: noqa: E402

import os
from pathlib import Path
import sys
import struct
import tempfile
import time
from unittest.mock import patch

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import robot_arm_library
from robot_arm_library import blender_addon as addon
from robot_arm_library.physics import blender as live


def pump(predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        live._tick()
        live._reap_clients()
        if predicate():
            return
        assert live.active(), live.status()
        time.sleep(.01)
    raise AssertionError(live.status())


def close_matrix(left, right):
    assert max(abs(left[r][c] - right[r][c]) for r in range(4) for c in range(4)) < 3e-5


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.robot_mujoco_python = os.environ.get("CARYAM_TEST_MUJOCO_PYTHON", str(ROOT / ".venv/bin/python"))
    scene.robot_mujoco_gravity = (0, 0, 0)
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "servo.xml"
        source.write_text('<mujoco><compiler angle="radian"/><worldbody><body name="base">'
                          '<geom size=".04" contype="0" conaffinity="0"/>'
                          '<body name="wrist" pos="0 0 .2"><joint name="hinge" range="-1.69 1.69" actuatorfrcrange="-2 2"/>'
                          '<geom type="capsule" fromto="0 0 0 .2 0 0" size=".02" contype="0" conaffinity="0"/>'
                          '</body></body></worldbody><actuator><position joint="hinge" kp="100"/></actuator></mujoco>')
        roots = []
        for x in (0, 1):
            item = scene.robot_library.add()
            item.robot_id = "servo"
            item.name = "Servo Arm"
            item.model_path = str(source)
            item.source_path = str(source)
            roots.append(addon._place_robot(item, Vector((x, 0, 1))))
        joint = addon._joint_controls(roots[0])[0]
        other_joint = addon._joint_controls(roots[1])[0]
        body = next(obj for obj in roots[0].children_recursive if obj.get("mujoco_body_name") == "wrist")
        original = {obj: (obj.matrix_basis.copy(), obj.matrix_parent_inverse.copy()) for obj in scene.objects}
        with patch.dict(sys.modules, {"mujoco": None}):
            live.start(scene)
            pump(lambda: live.state() == "RUNNING" and live._session.applied)
            pose = body.matrix_world.copy()
            joint["qpos"] = .6
            joint.update_tag(refresh={"OBJECT"})
            bpy.context.view_layer.update()
            close_matrix(body.matrix_world, pose)
            pump(lambda: abs(live.joint_info(joint)[1] - .6) < .03)
            assert abs(live.joint_info(other_joint)[1]) < .01
            assert live.joint_info(joint)[0]["effort_range"] == [-2, 2]
            live.command("pause")
            pump(lambda: live.state() == "PAUSED")
            frozen_time = live._session.time
            frozen_pose = body.matrix_world.copy()
            joint["qpos"] = -.4
            joint.update_tag(refresh={"OBJECT"})
            live._tick()
            time.sleep(.1)
            live._tick()
            assert live._session.time == frozen_time
            close_matrix(body.matrix_world, frozen_pose)
            live.command("resume")
            pump(lambda: abs(live.joint_info(joint)[1] + .4) < .03)
            # Coalesce a drag's rapidly changing target values without moving Blender directly.
            for target in (.1, .2, .3, .4):
                joint["qpos"] = target
                live._tick()
            pump(lambda: abs(live.joint_info(joint)[1] - .4) < .03)
            live.command("pause")
            pump(lambda: live.state() == "PAUSED")
            endpoint_pose = body.matrix_world.copy()
            endpoint_time = live._session.time
            for limit in (-1.69, 1.69):
                joint["qpos"] = struct.unpack("f", struct.pack("f", limit))[0]
                joint.update_tag(refresh={"OBJECT"})
                pump(lambda: float(joint["qpos"]) == limit and live._session.target_command is None)
                assert live.state() == "PAUSED" and live._session.time == endpoint_time
                close_matrix(body.matrix_world, endpoint_pose)
            live.command("resume")
            pump(lambda: abs(live.joint_info(joint)[1] - 1.69) < .03)
            live.command("reset")
            pump(lambda: live.state() == "PAUSED" and live._session.time == 0)
            assert abs(float(joint["qpos"])) < 1e-6
            assert abs(live.joint_info(joint)[1]) < 1e-6
            live.stop(wait=True)
        for obj, (basis, inverse) in original.items():
            close_matrix(obj.matrix_basis, basis)
            close_matrix(obj.matrix_parent_inverse, inverse)
        assert abs(float(joint["qpos"])) < 1e-6
        assert all(not curve.mute for obj in scene.objects if obj.animation_data for curve in obj.animation_data.drivers)
    print("Live joint servo targets, pause/reset, duplicate mapping and authoring restoration passed")
finally:
    robot_arm_library.unregister()
