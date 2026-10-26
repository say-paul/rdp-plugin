"""Dependency-graph refreshes must not masquerade as live geometry edits."""
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


def begin():
    live.start(bpy.context.scene)
    pump(lambda: live.state() == "RUNNING" and live._session.applied)


def refresh(obj):
    obj.update_tag()
    bpy.context.view_layer.update()
    assert live._session is not None, live.status()
    live._tick()
    assert live._session is not None, live.status()


def check_real_edit(edit, name, body):
    begin()
    session = live._session
    worker_directory = session.client.directory
    original = next(basis for obj, _, basis, *_ in session.authoring.objects if obj == body)
    edit()
    bpy.context.view_layer.update()
    # The handler must not restore objects or terminate the worker during graph evaluation.
    assert live._session is session
    pump(lambda: not live.active())
    assert "Geometry changed:" in live.status() and name in live.status(), live.status()
    assert max(abs(body.matrix_basis[r][c] - original[r][c]) for r in range(4) for c in range(4)) < 3e-5
    assert not worker_directory.exists()


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.robot_mujoco_python = os.environ.get("CARYAM_TEST_MUJOCO_PYTHON", str(ROOT / ".venv/bin/python"))
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "arm.xml"
        source.write_text('<mujoco><worldbody><body name="base"><geom size=".04"/>'
                          '<body name="wrist" pos="0 0 .2"><joint name="hinge"/>'
                          '<geom type="capsule" fromto="0 0 0 .2 0 0" size=".02"/>'
                          '</body></body></worldbody></mujoco>')
        item = scene.robot_library.add()
        item.robot_id = "geometry-test"
        item.name = "Geometry test arm"
        item.model_path = str(source)
        item.source_path = str(source)
        robot = addon._place_robot(item, Vector((1, 0, 1)))
        joint = addon._joint_controls(robot)[0]
        robot_mesh = next(obj for obj in robot.children_recursive if obj.type == "MESH")
        bpy.ops.mesh.primitive_cube_add(size=.4, location=(0, 0, 2))
        cube = bpy.context.object
        bpy.ops.rigidbody.object_add()
        cube.rigid_body.collision_shape = "BOX"
        modifier = cube.modifiers.new("Bevel", "BEVEL")
        modifier.width = .02
        shared = bpy.data.objects.new("Shared cube", cube.data)
        scene.collection.objects.link(shared)
        shared.location = (-1, 0, 1)
        bpy.context.view_layer.update()
        begin()
        # These real depsgraph geometry flags do not change local-space geometry.
        initial_time = live._session.time
        for iteration in range(20):
            for obj in (cube, robot, joint, robot_mesh):
                refresh(obj)
            cube.data.update()
            joint["qpos"] = .01 * iteration
            joint.update_tag()
            cube.select_set(bool(iteration % 2))
            bpy.context.view_layer.update()
            live._tick()
            assert live._session is not None, live.status()
            time.sleep(.01)
        assert live._session.time > initial_time
        live.command("pause")
        pump(lambda: live.state() == "PAUSED")
        frozen_time = live._session.time
        for obj in (cube, robot, joint, robot_mesh):
            refresh(obj)
        assert live._session.time == frozen_time
        live.stop(wait=True)

        def deform():
            cube.data.vertices[0].co.x += .05
            cube.data.update()

        check_real_edit(deform, "Cube", cube)
        assert cube.data.vertices[0].co.x > -.2, "User mesh edits must not be undone on Stop"

        check_real_edit(lambda: setattr(modifier, "width", .05), cube.name, cube)

        def replace_mesh():
            cube.data = cube.data.copy()
            cube.data.vertices[0].co.y += .05
            cube.data.update()

        check_real_edit(replace_mesh, cube.name, cube)

        def topology_edit():
            cube.data.clear_geometry()
            cube.data.from_pydata([(-.2, -.2, -.2), (.2, -.2, -.2), (0, .2, -.2), (0, 0, .2)], [],
                                 [(0, 2, 1), (0, 1, 3), (1, 2, 3), (2, 0, 3)])
            cube.data.update()

        check_real_edit(topology_edit, cube.name, cube)
        cube.shape_key_add(name="Basis")
        shape = cube.shape_key_add(name="Deformation")
        shape.data[0].co.x += .1
        shape.value = 0
        bpy.context.view_layer.update()
        check_real_edit(lambda: setattr(shape, "value", 1), cube.name, cube)

        def robot_visual_edit():
            robot_mesh.data.vertices[0].co.x += .05
            robot_mesh.data.update()

        check_real_edit(robot_visual_edit, robot_mesh.name, cube)
    print("Live geometry refreshes, joint targets, mesh/modifier/topology/shape-key edits and deferred cleanup passed")
finally:
    robot_arm_library.unregister()
