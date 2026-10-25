"""Click-created magnetic anchors on arbitrary objects and robot bases."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import patch

import bpy
from mathutils import Matrix, Vector

repository_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(repository_root))

import robot_arm_library
from robot_arm_library import blender_addon as addon, mounting


def active(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def near(moving, source, target):
    delta = target.matrix_world.translation - source.matrix_world.translation + Vector((0, 0, .01))
    moving.location += delta
    bpy.context.view_layer.update()
    active(source)


def assert_mated(source, target):
    assert (source.matrix_world.translation - target.matrix_world.translation).length < 1e-5
    source_normal = source.matrix_world.to_quaternion() @ Vector((0, 0, 1))
    target_normal = target.matrix_world.to_quaternion() @ Vector((0, 0, 1))
    assert source_normal.dot(target_normal) < -.9999


def creation_state(hit):
    state = SimpleNamespace(
        preview=mounting._anchor_visual(bpy.context, "Preview"), hit=hit,
        report=lambda *_args: None,
    )
    state._hover = lambda *_args: None
    state._cleanup = MethodType(mounting.ROBOT_OT_add_magnetic_point._cleanup, state)
    return state


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0))
    table = bpy.context.object
    table.name = "Target Table"
    table.scale = (2, 2, 1)
    bpy.context.view_layer.update()
    scene.cursor.location = (99, 99, 99)
    hit = mounting.surface_hit(bpy.context, Vector((.2, .1, 3)), Vector((0, 0, -1)))
    assert hit is not None and hit[0] == table
    assert abs(hit[1].translation.z - .5) < 1e-6
    assert (hit[1].to_quaternion() @ Vector((0, 0, 1))).z > .999
    context = SimpleNamespace(
        collection=bpy.context.collection, scene=scene, view_layer=bpy.context.view_layer,
        selected_objects=bpy.context.selected_objects,
        workspace=SimpleNamespace(status_text_set=lambda _text: None),
        area=SimpleNamespace(tag_redraw=lambda: None),
    )
    state = creation_state(hit)
    assert mounting.ROBOT_OT_add_magnetic_point.modal(state, context, SimpleNamespace(type="LEFTMOUSE", value="PRESS")) == {"FINISHED"}
    target = bpy.context.object
    assert target.parent == table and mounting.is_anchor(target)
    assert target.empty_display_type == "SPHERE" and target.show_name
    assert (target.matrix_world.translation - hit[1].translation).length < 1e-5
    assert (target.matrix_world.translation - scene.cursor.location).length > 10
    target.name = "Table Magnet"
    state = creation_state(None)
    preview_name = state.preview.name
    assert mounting.ROBOT_OT_add_magnetic_point.modal(state, context, SimpleNamespace(type="LEFTMOUSE", value="PRESS")) == {"RUNNING_MODAL"}
    assert mounting.ROBOT_OT_add_magnetic_point.modal(state, context, SimpleNamespace(type="ESC")) == {"CANCELLED"}
    assert preview_name not in bpy.data.objects

    bpy.ops.mesh.primitive_cube_add(size=1, location=(3, 0, 2))
    part = bpy.context.object
    part.name = "Loose Part"
    part.scale = (.4, .6, .2)
    bpy.context.view_layer.update()
    hit = mounting.surface_hit(bpy.context, Vector((3.05, 0, 1)), Vector((0, 0, 1)))
    assert hit is not None and hit[0] == part
    source = mounting.create_magnetic_point(bpy.context, *hit)
    source.name = "Part Magnet"
    near(part, source, target)
    source_rotation = source.matrix_world.to_quaternion()
    for mode in ("MATCH", "POSITION"):
        candidate = mounting.nearest_snap(
            source.matrix_world.translation, source_rotation, scene.robot_mount_distance, [], [target], mode,
        )
        expected = target.matrix_world.to_quaternion() if mode == "MATCH" else source_rotation
        assert candidate is not None and abs(candidate.rotation.dot(expected)) > .9999
    original_scale = part.matrix_world.to_scale().copy()
    target.caryam_anchor_enabled = False
    assert bpy.ops.robot_library.magnetic_move("EXEC_DEFAULT") == {"CANCELLED"}
    assert not part.caryam_mount.enabled
    target.caryam_anchor_enabled = True
    assert bpy.ops.robot_library.magnetic_move("EXEC_DEFAULT") == {"FINISHED"}
    assert_mated(source, target)
    assert (part.matrix_world.to_scale() - original_scale).length < 1e-5
    assert part.parent == target and part.caryam_mount.source_anchor == source
    assert mounting.export_mount(part).target_instance_id == table.name
    table.rotation_euler.z = .6
    table.location.x = .4
    bpy.context.view_layer.update()
    assert_mated(source, target)
    source.caryam_anchor_enabled = False
    assert mounting.export_mount(part) is not None
    source.caryam_anchor_enabled = True
    active(source)
    before = part.matrix_world.copy()
    assert bpy.ops.robot_library.detach_mount() == {"FINISHED"}
    assert part.parent is None
    assert (part.matrix_world.translation - before.translation).length < 1e-5

    # Arbitrary anchor offsets, not the part origin, determine the matching pose.
    source.location.x += .07
    bpy.context.view_layer.update()
    near(part, source, target)
    assert bpy.ops.robot_library.magnetic_move("EXEC_DEFAULT") == {"FINISHED"}
    assert_mated(source, target)

    with tempfile.TemporaryDirectory() as directory:
        root_dir = Path(directory)
        model_path = root_dir / "arm.xml"
        model_path.write_text(
            '<mujoco><worldbody><body name="base"><geom type="box" size=".1 .1 .02"/>'
            '<body name="wrist" pos="0 0 .3"><joint name="hinge"/><geom type="sphere" size=".04"/>'
            '</body></body></worldbody></mujoco>', encoding="utf-8",
        )
        item = scene.robot_library.add()
        item.robot_id = "arm"
        item.name = "Arm"
        item.model_path = str(model_path)
        item.source_path = str(model_path)
        robot = addon._place_robot(item, Vector((4, 0, 0)))
        base = mounting.base_frame(robot)
        robot_source = mounting.create_magnetic_point(
            bpy.context, base,
            Matrix.LocRotScale(Vector((4.04, .03, -.02)), Vector((0, 0, -1)).to_track_quat("Z", "Y"), Vector((1, 1, 1))),
        )
        near(robot, robot_source, target)
        assert bpy.ops.robot_library.magnetic_move("EXEC_DEFAULT") == {"FINISHED"}
        assert_mated(robot_source, target)
        assert (base.matrix_world.translation - target.matrix_world.translation).length > .02
        wrist = next(obj for obj in robot.children_recursive if obj.get("mujoco_body_name") == "wrist")
        wrist_target = mounting.create_magnetic_point(bpy.context, wrist, wrist.matrix_world)
        try:
            mounting._source_anchor(robot, wrist_target)
        except ValueError as error:
            assert "articulated-link" in str(error)
        else:
            raise AssertionError("An articulated link was silently treated as a fixed base")
        active(source)
        assert bpy.ops.robot_library.detach_mount() == {"FINISHED"}
        near(part, source, wrist_target)
        assert bpy.ops.robot_library.magnetic_move("EXEC_DEFAULT") == {"FINISHED"}
        assert_mated(source, wrist_target)
        assert mounting.export_mount(part).target_kind == "robot"
        joint = addon._joint_controls(robot)[0]
        joint["qpos"] = .4
        joint.update_tag(refresh={"OBJECT"})
        scene.frame_set(scene.frame_current)
        bpy.context.view_layer.update()
        assert_mated(source, wrist_target)
        active(table)
        bpy.ops.rigidbody.object_add()
        table.rigid_body.collision_shape = "BOX"
        world, manifest_path, renderer = addon._export_world_file(str(root_dir / "magnets.xml"))
        manifest = json.loads(manifest_path.read_text())
        part_entry = next(mesh for mesh in manifest["scene_meshes"] if mesh["instance_id"] == part.name)
        assert part_entry["mount"]["target_kind"] == "robot"
        assert len(manifest["robots"]) == 1
        namespace = {"__file__": str(renderer)}
        with patch("sys.argv", [str(renderer), "--sensors-only"]):
            try:
                exec(compile(renderer.read_text(), str(renderer), "exec"), namespace)
            except SystemExit as stopped:
                assert stopped.code == 0
        import mujoco
        model, data = namespace["model"], namespace["data"]
        part_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, part_entry["asset_name"])
        assert part_id > 0
        assert (Vector(data.xpos[part_id]) - part.matrix_world.translation).length < 1e-5
        assert world.exists()
        names = (robot.name, part.name, source.name, wrist_target.name)
        blend = root_dir / "magnetic.blend"
        bpy.ops.wm.save_as_mainfile(filepath=str(blend))
        bpy.ops.wm.open_mainfile(filepath=str(blend))
        robot, part, source, wrist_target = [bpy.data.objects[name] for name in names]
        assert part.caryam_mount.source_anchor == source and part.caryam_mount.target == wrist_target
        assert part.parent == wrist_target
        assert_mated(source, wrist_target)
        bpy.data.objects.remove(source, do_unlink=True)
        try:
            mounting.export_mount(part)
        except ValueError as error:
            assert "missing source anchor" in str(error)
        else:
            raise AssertionError("Deleted source anchor was silently accepted")
    print("Magnetic point creation, arbitrary object snapping, persistence and MuJoCo export passed")
finally:
    robot_arm_library.unregister()
