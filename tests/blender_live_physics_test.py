"""Headless integration of external MuJoCo, viewport transforms and restoration."""
# ruff: noqa: E402

from __future__ import annotations

import os
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch

import bpy
from mathutils import Matrix, Quaternion, Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import blender_addon as addon, mounting
from robot_arm_library.physics import blender as live


def close_matrix(actual, expected, tolerance=4e-5):
    assert max(abs(actual[r][c] - expected[r][c]) for r in range(4) for c in range(4)) < tolerance, (actual, expected)


def pump(predicate, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        live._tick()
        live._reap_clients()
        if predicate():
            return
        assert live.active(), live.status()
        time.sleep(.015)
    raise AssertionError(f"Live physics timed out: {live.status()}")


def clear():
    live.stop(wait=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.context.scene.robot_library.clear()


def cube(name, location, *, dynamic=False, scale=(1, 1, 1)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    if dynamic:
        bpy.ops.rigidbody.object_add()
        obj.rigid_body.collision_shape = "BOX"
    bpy.context.view_layer.update()
    return obj


def robot(path, name, location):
    item = bpy.context.scene.robot_library.add()
    item.robot_id = name
    item.name = name
    item.model_path = str(path)
    item.source_path = str(path)
    obj = addon._place_robot(item, Vector(location))
    assert obj["import_mode"] == "mujoco_compiled"
    return obj


def capture():
    return {obj: (obj.matrix_basis.copy(), obj.matrix_parent_inverse.copy(), obj.parent,
                  float(obj["qpos"]) if "qpos" in obj else None)
            for obj in bpy.context.scene.objects}


def restored(saved):
    for obj, (basis, inverse, parent, qpos) in saved.items():
        try:
            close_matrix(obj.matrix_basis, basis)
            close_matrix(obj.matrix_parent_inverse, inverse)
            assert obj.parent == parent
            if qpos is not None:
                assert float(obj["qpos"]) == qpos
        except ReferenceError:
            pass
    assert not bpy.app.timers.is_registered(live._tick)


def check_frame():
    session = live._session
    frame = session.last_frame
    poses = {name: Matrix.LocRotScale(Vector(p), Quaternion(q), Vector((1, 1, 1)))
             for name, p, q in zip(session.body_names, frame["positions"], frame["quaternions"])}
    bpy.context.view_layer.update()
    for binding in session.bindings:
        expected = poses[binding.body] @ binding.offset @ Matrix.Diagonal((*binding.scale, 1.0))
        close_matrix(binding.obj.matrix_world, expected)


def begin():
    assert bpy.ops.robot_library.live_physics(action="START") == {"FINISHED"}
    pump(lambda: live.state() == "RUNNING" and live._session.applied)


robot_arm_library.register()
try:
    scene = bpy.context.scene
    scene.robot_mujoco_python = os.environ.get("CARYAM_TEST_MUJOCO_PYTHON", str(ROOT / ".venv/bin/python"))
    with tempfile.TemporaryDirectory(prefix="caryam live test ") as directory:
        path = Path(directory)
        arm_path = path / "arm.xml"
        arm_path.write_text(
            '<mujoco><worldbody><body name="base" pos="0 0 .15">'
            '<geom type="box" size=".08 .08 .03" contype="0" conaffinity="0"/>'
            '<body name="link" pos="0 0 .2"><joint name="hinge" axis="0 1 0" damping=".05"/>'
            '<geom type="capsule" fromto="0 0 0 0 0 -.2" size=".025" contype="0" conaffinity="0"/>'
            '</body></body></worldbody></mujoco>', encoding="utf-8")
        free_path = path / "floating.xml"
        free_path.write_text(
            '<mujoco><worldbody><body name="base" pos="0 0 .1"><freejoint name="free"/>'
            '<geom type="sphere" size=".05"/></body></worldbody></mujoco>', encoding="utf-8")

        clear()
        table = cube("Table", (0, 0, .25), scale=(3, 3, .5))
        falling = cube("Falling", (0, 0, 2), dynamic=True, scale=(.2, .2, .2))
        # A nonuniformly scaled real parent must not introduce local-transform drift.
        parent = bpy.data.objects.new("Scaled parent", None)
        scene.collection.objects.link(parent)
        parent.scale = (2, 3, 4)
        bpy.context.view_layer.update()
        world = falling.matrix_world.copy()
        falling.parent = parent
        falling.matrix_parent_inverse = parent.matrix_world.inverted()
        falling.matrix_basis = world
        bpy.context.view_layer.update()
        original = capture()
        with patch.dict(sys.modules, {"mujoco": None}):
            begin()
            assert not bpy.ops.robot_library.launch_mujoco.poll()
            assert not bpy.ops.robot_library.sticky_move.poll()
            pump(lambda: live._session.time >= 1)
            check_frame()
            assert .55 < falling.matrix_world.translation.z < .65
            close_matrix(table.matrix_world, Matrix.LocRotScale(Vector((0, 0, .25)), Quaternion(), Vector((3, 3, .5))))
            live.command("pause")
            pump(lambda: live.state() == "PAUSED")
            frozen = live._session.time
            time.sleep(.15)
            live._tick()
            assert live._session.time == frozen
            live.command("resume")
            pump(lambda: live._session.time > frozen + .05)
            live.command("reset")
            pump(lambda: live.state() == "PAUSED" and live._session.time == 0)
            close_matrix(falling.matrix_world, world)
            owned = live._session.client.directory
            live.stop(wait=True)
            restored(original)
            assert scene.rigidbody_world.enabled
            assert not owned.exists(), "Session files must be removed on Stop"
            live.stop()

        clear()
        platform = cube("Moving platform", (0, 0, 2), dynamic=True, scale=(1, 1, .1))
        arm = robot(arm_path, "Mounted arm", (0, 0, 2))
        mounting.mount_robot(arm, platform)
        joint = addon._joint_controls(arm)[0]
        joint["qpos"] = .8
        joint.update_tag(refresh={"OBJECT"})
        bpy.context.view_layer.update()
        link = next(obj for obj in arm.children_recursive if obj.get("mujoco_body_name") == "link")
        attachment = cube("Link payload", (0, 0, 2.4), scale=(.04, .04, .04))
        mounting.mount_robot(attachment, link)
        child_arm = robot(arm_path, "Nested arm", (0, 0, 2.6))
        mounting.mount_robot(child_arm, attachment)
        marker = mounting.create_magnetic_point(bpy.context, link, link.matrix_world @ Matrix.Translation((0, 0, -.1)))
        marker_local = marker.matrix_local.copy()
        # Duplicate original body/joint names and a floating root are mapped by prefix, not source IDs.
        other = robot(arm_path, "Mounted arm", (2, 0, .1))
        other_joint = addon._joint_controls(other)[0]
        other_joint["qpos"] = .8
        other_joint.update_tag(refresh={"OBJECT"})
        other_link = next(obj for obj in other.children_recursive if obj.get("mujoco_body_name") == "link")
        floating = robot(free_path, "Floating", (-2, 0, 2))
        floating.rotation_euler.z = .4
        bpy.context.view_layer.update()
        link_world = other_link.matrix_world.copy()
        saved = capture()
        driver_states = [(driver, driver.mute) for obj in scene.objects if obj.animation_data for driver in obj.animation_data.drivers]
        with patch.dict(sys.modules, {"mujoco": None}):
            begin()
            pump(lambda: live._session.time > .25)
            check_frame()
            assert platform.matrix_world.translation.z < 1.9
            assert not all(abs(other_link.matrix_world[r][c] - link_world[r][c]) < 1e-4 for r in range(3) for c in range(3))
            assert mounting.base_frame(floating).matrix_world.translation.z < 2
            close_matrix(marker.matrix_world, link.matrix_world @ marker_local)
            assert all(driver.mute for driver, _ in driver_states)
            live.stop(wait=True)
        restored(saved)
        assert all(driver.mute == muted for driver, muted in driver_states)
        assert arm.caryam_mount.target == platform and child_arm.caryam_mount.target == attachment
        assert other.name != arm.name

        # Failures and lifecycle transitions must restore surviving authoring objects.
        begin()
        process = live._session.client.process
        process.kill()
        pump(lambda: not live.active())
        assert live.status().startswith("Error"), live.status()
        restored(saved)
        begin()
        live._stop_for_lifecycle()
        restored(saved)
        begin()
        bpy.data.objects.remove(marker, do_unlink=True)
        pump(lambda: not live.active())
        assert not live.active()
        restored(saved)
        begin()
        platform.rigid_body.mass = 2
        pump(lambda: not live.active())
        assert not live.active() and "settings" in live.status()
        begin()
        other_scene = bpy.data.scenes.new("Other physics scene")
        assert tuple(other_scene.robot_mujoco_gravity) == (0, 0, float(scene.robot_mujoco_gravity[2]))
        bpy.context.window.scene = other_scene
        pump(lambda: not live.active())
        assert not live.active()
        bpy.context.window.scene = scene
        restored(saved)
        begin()
        bpy.ops.wm.save_as_mainfile(filepath=str(path / "restored.blend"))
        assert not live.active()
        restored(saved)
        scene.robot_mujoco_python = str(path / "missing-python")
        try:
            live.start(scene)
            pump(lambda: not live.active())
        except OSError:
            pass
        assert not live.active()
        restored(saved)

        # Reject unsupported previews and shear before touching authoring transforms.
        clear()
        fallback = bpy.data.objects.new("Fallback", None)
        scene.collection.objects.link(fallback)
        fallback["robot_id"] = "fallback"
        fallback["model_path"] = str(arm_path)
        fallback["import_mode"] = "xml_fallback"
        try:
            live.start(scene)
        except ValueError as error:
            assert "compiled" in str(error)
        else:
            raise AssertionError("Fallback preview must not run partial physics")
        clear()
        sheared = cube("Sheared", (0, 0, 2))
        ancestor = bpy.data.objects.new("Nonuniform", None)
        scene.collection.objects.link(ancestor)
        ancestor.scale = (2, 1, 1)
        sheared.parent = ancestor
        sheared.rotation_euler.z = .5
        bpy.context.view_layer.update()
        try:
            live.start(scene)
        except ValueError as error:
            assert "shear" in str(error)
        else:
            raise AssertionError("Shear must be rejected")
        print("Live MuJoCo viewport, mounts, restoration and lifecycle regression passed")
finally:
    robot_arm_library.unregister()
