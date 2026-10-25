"""Exercise sticky placement, persistent mounts, and exported MuJoCo motion."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from types import MethodType, SimpleNamespace

import bpy
from mathutils import Matrix, Vector

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
from robot_arm_library import blender_addon as addon, mounting


def assert_matrix(actual, expected, tolerance=1e-5):
    assert max(abs(actual[row][column] - expected[row][column]) for row in range(4) for column in range(4)) < tolerance


def active(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def modal_state():
    state = SimpleNamespace(report=lambda *_args: None)
    for name in ("_prepare", "_restore", "_status", "_mouse_position"):
        setattr(state, name, MethodType(getattr(mounting.ROBOT_OT_sticky_move, name), state))
    return state


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        model_path = path / "arm.xml"
        model_path.write_text(
            '<mujoco><worldbody><body name="base" pos="0 0 0.15">'
            '<geom type="box" size="0.08 0.08 0.03" contype="0" conaffinity="0"/>'
            '<body name="wrist" pos="0 0 0.2"><joint name="hinge" type="hinge"/>'
            '<geom type="sphere" size="0.04" contype="0" conaffinity="0"/>'
            '</body></body></worldbody></mujoco>', encoding="utf-8",
        )
        item = bpy.context.scene.robot_library.add()
        item.robot_id = "test-arm"
        item.name = "Test Arm"
        item.model_path = str(model_path)
        item.source_path = str(model_path)
        root = addon._place_robot(item, Vector((0, 0, 0.4)))
        assert root["import_mode"] == "mujoco_compiled"
        base = mounting.base_frame(root)
        assert base["mujoco_body_name"] == "base"
        joint = addon._joint_controls(root)[0]
        joint["qpos"] = 0.25
        bpy.context.view_layer.update()

        bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0))
        platform = bpy.context.object
        platform.name = "Platform"
        platform.scale = (2, 2, 1)
        bpy.context.view_layer.update()
        bpy.context.scene.robot_mount_distance = 0.1
        active(root)
        assert bpy.ops.robot_library.sticky_move("EXEC_DEFAULT") == {"FINISHED"}
        assert root.parent == platform
        assert root.caryam_mount.enabled
        assert abs(base.matrix_world.translation.z - 0.5) < 1e-5
        assert abs(float(joint["qpos"]) - 0.25) < 1e-8

        # Optional magnets must never change the default base-to-surface tool.
        mounting.detach_robot(root)
        source_magnet = mounting.create_magnetic_point(
            bpy.context, base, base.matrix_world @ Matrix.Translation((.04, 0, .04)),
        )
        target_magnet = mounting.create_magnetic_point(
            bpy.context, platform, Matrix.Translation((.1, 0, .5)),
        )
        active(root)
        assert bpy.ops.robot_library.sticky_move("EXEC_DEFAULT") == {"FINISHED"}
        assert root.parent == platform, "Sticky Move must ignore optional magnetic points"
        assert not root.caryam_mount.uses_source_anchor
        assert abs(base.matrix_world.translation.z - .5) < 1e-5
        source_magnet.caryam_anchor_enabled = False
        mounting.detach_robot(root)
        bpy.context.view_layer.update()
        assert bpy.ops.robot_library.sticky_move("EXEC_DEFAULT") == {"FINISHED"}
        assert root.parent == platform
        source_magnet.caryam_anchor_enabled = True
        assert target_magnet.parent == platform

        mount = mounting.export_mount(root)
        assert mount.target_kind == "scene" and mount.target_instance_id == platform.name

        local = root.matrix_world.copy()
        platform.location.x = 0.8
        platform.rotation_euler.z = 0.4
        bpy.context.view_layer.update()
        assert_matrix(root.matrix_world, Matrix.Translation((0.8, 0, 0)) @ Matrix.Rotation(0.4, 4, "Z") @ local)
        platform.name = "Renamed Platform"
        assert mounting.export_mount(root).target_instance_id == "Renamed Platform"

        saved_world = root.matrix_world.copy()
        active(root)
        assert bpy.ops.robot_library.detach_mount() == {"FINISHED"}
        assert root.parent is None and not root.caryam_mount.enabled
        assert_matrix(root.matrix_world, saved_world)
        root.location = (5, 5, 5)
        bpy.context.view_layer.update()
        assert bpy.ops.robot_library.sticky_move("EXEC_DEFAULT") == {"CANCELLED"}
        assert root.parent is None

        active(platform)
        bpy.context.scene.cursor.location = (0.8, 0, 0.8)
        bpy.context.scene.cursor.rotation_euler = (0.4, 0, 0.2)
        assert bpy.ops.robot_library.add_mount_point() == {"FINISHED"}
        point = bpy.context.object
        assert point["caryam_mount_point"] and point.parent == platform
        base_local = root.matrix_world.inverted() @ base.matrix_world
        root.matrix_world = Matrix.Translation((0.01, 0, 0)) @ point.matrix_world @ base_local.inverted()
        bpy.context.view_layer.update()
        active(root)
        assert bpy.ops.robot_library.sticky_move("EXEC_DEFAULT") == {"FINISHED"}
        assert root.parent == point
        assert_matrix(base.matrix_world, point.matrix_world)
        assert mounting.export_mount(root).target_instance_id == platform.name

        state = modal_state()
        state._prepare(bpy.context)
        state.mouse_offset = Vector((0, 0, 0))
        state._mouse_position = lambda _event: point.matrix_world.translation + Vector((0.01, 0, 0))
        original_world = root.matrix_world.copy()
        root.parent = None
        root.location += Vector((1, 2, 3))
        context = SimpleNamespace(
            workspace=SimpleNamespace(status_text_set=lambda _value: None),
            view_layer=bpy.context.view_layer,
            scene=bpy.context.scene,
            area=SimpleNamespace(tag_redraw=lambda: None),
        )
        assert mounting.ROBOT_OT_sticky_move.modal(state, context, SimpleNamespace(type="MOUSEMOVE")) == {"RUNNING_MODAL"}
        assert state.snap.target == point
        assert_matrix(base.matrix_world, point.matrix_world)
        assert mounting.ROBOT_OT_sticky_move.modal(state, context, SimpleNamespace(type="ESC")) == {"CANCELLED"}
        assert root.parent == point and root.caryam_mount.enabled
        assert_matrix(root.matrix_world, original_world)
        state = modal_state()
        state._prepare(bpy.context)
        state.mouse_offset = Vector((0, 0, 0))
        state._mouse_position = lambda _event: point.matrix_world.translation
        root.parent = None
        assert mounting.ROBOT_OT_sticky_move.modal(state, context, SimpleNamespace(type="MOUSEMOVE")) == {"RUNNING_MODAL"}
        assert mounting.ROBOT_OT_sticky_move.modal(state, context, SimpleNamespace(type="LEFTMOUSE", value="PRESS")) == {"FINISHED"}
        assert root.parent == point and root.caryam_mount.enabled
        state = modal_state()
        state._prepare(bpy.context)
        state.mouse_offset = Vector((0, 0, 0))
        state._mouse_position = lambda _event: Vector((5, 5, 5))
        root.parent = None
        assert mounting.ROBOT_OT_sticky_move.modal(state, context, SimpleNamespace(type="MOUSEMOVE")) == {"RUNNING_MODAL"}
        assert state.snap is None
        assert (base.matrix_world.translation - Vector((5, 5, 5))).length < 1e-5
        assert mounting.ROBOT_OT_sticky_move.modal(state, context, SimpleNamespace(type="LEFTMOUSE", value="PRESS")) == {"FINISHED"}
        assert root.parent is None and not root.caryam_mount.enabled
        root.matrix_world = original_world
        bpy.context.view_layer.update()
        assert bpy.ops.robot_library.sticky_move("EXEC_DEFAULT") == {"FINISHED"}

        # A mounted child robot must not contribute its joints or sensors to its parent.
        child = addon._place_robot(item, Vector((0, 0, 2)))
        wrist = next(obj for obj in root.children_recursive if obj.get("mujoco_body_name") == "wrist")
        child_base = mounting.base_frame(child)
        child_base_local = child.matrix_world.inverted() @ child_base.matrix_world
        child.matrix_world = wrist.matrix_world @ Matrix.Translation((0, 0, 0.1)) @ child_base_local.inverted()
        mounting.mount_robot(child, wrist)
        bpy.context.view_layer.update()
        assert len(addon._joint_controls(root)) == 1
        assert len(addon._joint_controls(child)) == 1
        instances = addon._world_robot_instances()
        assert all(set(instance.qpos) == {"hinge"} for instance in instances)
        child_mount = mounting.export_mount(child)
        assert child_mount.target_kind == "robot" and child_mount.target_body_name == "wrist"
        try:
            mounting.mount_robot(root, child_base)
        except ValueError as error:
            assert "descendants" in str(error)
        else:
            raise AssertionError("Cyclic mounting was accepted")

        active(platform)
        bpy.ops.rigidbody.object_add()
        platform.rigid_body.collision_shape = "BOX"
        world_path, manifest_path, renderer_path = addon._export_world_file(str(path / "mounted.xml"))
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        assert all(robot["mount"] is not None for robot in manifest["robots"])
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(world_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        parent_instance = next(instance for instance in instances if instance.instance_id == root.name)
        child_instance = next(instance for instance in instances if instance.instance_id == child.name)
        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, parent_instance.prefix + "base")
        wrist_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, parent_instance.prefix + "wrist")
        child_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, child_instance.prefix + "base")
        assert int(model.body_parentid[child_id]) == wrist_id or int(model.body_parentid[int(model.body_parentid[child_id])]) == wrist_id
        for _ in range(20):
            mujoco.mj_step(model, data)
        assert model.nq >= 9
        assert base_id > 0 and child_id > 0
        from unittest.mock import patch

        namespace = {"__file__": str(renderer_path)}
        with patch("sys.argv", [str(renderer_path), "--sensors-only"]):
            try:
                exec(compile(renderer_path.read_text(), str(renderer_path), "exec"), namespace)
            except SystemExit as stopped:
                assert stopped.code == 0
        rendered_data = namespace["data"]
        assert (Vector(rendered_data.xpos[base_id]) - base.matrix_world.translation).length < 1e-5
        assert (Vector(rendered_data.xpos[child_id]) - child_base.matrix_world.translation).length < 1e-5

        root_name, point_name, child_name = root.name, point.name, child.name
        blend_path = path / "mount.blend"
        bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))
        bpy.ops.wm.open_mainfile(filepath=str(blend_path))
        root = bpy.data.objects[root_name]
        child = bpy.data.objects[child_name]
        point = bpy.data.objects[point_name]
        assert root.caryam_mount.enabled and root.caryam_mount.target == point
        assert root.parent == point
        assert mounting.export_mount(child).target_body_name == "wrist"
        bpy.data.objects.remove(point, do_unlink=True)
        try:
            mounting.export_mount(root)
        except ValueError as error:
            assert "missing mount target" in str(error)
        else:
            raise AssertionError("Missing target was silently exported")
        active(root)
        assert bpy.ops.robot_library.detach_mount() == {"FINISHED"}
    print("Sticky mount UI, persistence, and MuJoCo export test passed")
finally:
    robot_arm_library.unregister()
