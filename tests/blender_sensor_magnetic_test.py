"""Magnetic anchors move sensor instances, not their robot or preview helpers."""
# ruff: noqa: E402

import json
import sys
import tempfile
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import blender_addon as addon, mounting, sensor_visuals


def active(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def matrix_close(actual, expected):
    assert max(abs(actual[r][c] - expected[r][c]) for r in range(4) for c in range(4)) < 1e-5


def snap(sensor, source, target):
    sensor.matrix_world.translation += target.matrix_world.translation - source.matrix_world.translation + Vector((0, 0, .01))
    bpy.context.view_layer.update()
    active(source)
    assert bpy.ops.robot_library.magnetic_move("EXEC_DEFAULT") == {"FINISHED"}
    assert sensor.parent is target and sensor.caryam_mount.source_anchor is source
    assert (source.matrix_world.translation - target.matrix_world.translation).length < 1e-5
    source_normal = source.matrix_world.to_quaternion() @ Vector((0, 0, 1))
    target_normal = target.matrix_world.to_quaternion() @ Vector((0, 0, 1))
    assert source_normal.dot(target_normal) < -.9999


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.robot_magnet_alignment = "OPPOSE"
    scene.robot_setup_path = "/nonexistent/robot/source"
    assert bpy.ops.robot_library.refresh() == {"FINISHED"}
    sensors = []
    for index, item in enumerate(scene.sensor_library):
        scene.cursor.location = (3 + index, 0, 2)
        assert bpy.ops.robot_library.add_sensor(sensor_index=index) == {"FINISHED"}
        sensor = bpy.context.object
        bpy.context.view_layer.update()
        assert sensor.hide_render
        hit = mounting.surface_hit(
            bpy.context, sensor.matrix_world.translation + Vector((.01, .005, 1)), Vector((0, 0, -1)),
        )
        assert hit is not None and hit[0] is sensor, item.category
        source = mounting.create_magnetic_point(bpy.context, *hit)
        assert source.parent is sensor and mounting.moving_object(source) is sensor
        assert mounting.moving_object(sensor) is sensor
        assert sensor.caryam_mount.source_anchor is source
        sensors.append((sensor, source))

    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        model_path = path / "arm.xml"
        model_path.write_text(
            '<mujoco><worldbody><body name="base"><geom size=".03"/>'
            '<body name="wrist" pos="0 0 .2"><joint name="hinge"/><geom size=".02"/>'
            '</body></body></worldbody></mujoco>', encoding="utf-8",
        )
        item = scene.robot_library.add()
        item.robot_id = "arm"
        item.name = "Arm"
        item.model_path = str(model_path)
        item.source_path = str(model_path)
        robot = addon._place_robot(item, Vector((0, 0, 1)))
        wrist = next(obj for obj in robot.children_recursive if obj.get("mujoco_body_name") == "wrist")
        target = mounting.create_magnetic_point(bpy.context, wrist, wrist.matrix_world @ Matrix.Translation((.1, 0, 0)))
        robot_pose = robot.matrix_world.copy()
        wrist_pose = wrist.matrix_world.copy()
        for sensor, source in sensors:
            snapshot = sensor["sensor_definition_json"]
            snap(sensor, source, target)
            assert mounting.robot_root(sensor) is robot
            assert sensor["sensor_definition_json"] == snapshot and sensor.hide_render
            assert mounting.moving_object(source) is sensor
            assert mounting.moving_object(sensor) is sensor
            if sensor.children:
                assert mounting.moving_object(sensor.children[0]) is sensor
            matrix_close(robot.matrix_world, robot_pose)
            matrix_close(wrist.matrix_world, wrist_pose)
        assert len(addon._sensor_mounts(robot)) == len(sensors)
        assert not addon._world_scene_sensors()
        world, manifest_path, _ = addon._export_world_file(str(path / "robot_sensors.xml"))
        manifest = json.loads(manifest_path.read_text())
        assert len(manifest["robots"][0]["sensors"]) == len(sensors)
        assert not manifest["scene_meshes"]
        import mujoco
        compiled = mujoco.MjModel.from_xml_path(str(world))
        assert compiled.ngeom == 3, "Sensor housings and anchors must not become collision geometry"
        prefix = manifest["robots"][0]["prefix"]
        for mount in addon._sensor_mounts(robot):
            assert mount.body_name == "wrist"
            assert compiled.body_parentid[compiled.body(prefix + mount.mount_name).id] == compiled.body(prefix + "wrist").id

        # Replacing/adding an anchor on an attached sensor must not resolve to the robot body.
        sensor, source = sensors[0]
        extra = mounting.create_magnetic_point(bpy.context, sensor, source.matrix_world)
        assert extra.parent is sensor and mounting.moving_object(extra) is sensor
        assert sensor.caryam_mount.source_anchor is source
        sensor_visuals.update_preview(sensor, addon._sensor_definition(sensor))
        assert source.parent is sensor and extra.parent is sensor
        bpy.data.objects.remove(extra, do_unlink=True)
        active(source)
        before = sensor.matrix_world.copy()
        assert bpy.ops.robot_library.detach_mount() == {"FINISHED"}
        matrix_close(sensor.matrix_world, before)
        assert sensor.parent is None and not sensor.caryam_mount.enabled
        assert not robot.caryam_mount.enabled

        bpy.ops.mesh.primitive_cube_add(size=.2, location=(1, 0, 1))
        cube = bpy.context.object
        bpy.context.view_layer.update()
        cube_target = mounting.create_magnetic_point(bpy.context, cube, cube.matrix_world @ Matrix.Translation((0, 0, .1)))
        snap(sensor, source, cube_target)
        exported = next(mount for mount in addon._world_scene_sensors() if mount.object_name == sensor.name)
        assert exported.body_name.startswith("scene_")
        world, manifest_path, _ = addon._export_world_file(str(path / "scene_sensor.xml"))
        compiled = mujoco.MjModel.from_xml_path(str(world))
        assert compiled.body_parentid[compiled.body(exported.mount_name).id] == compiled.body(exported.body_name).id
        manifest = json.loads(manifest_path.read_text())
        assert len(manifest["scene_meshes"]) == 1

        # The sensor-specific attachment tools can replace and detach a magnetic mount.
        active(sensor)
        sensor.caryam_sensor_target = wrist
        assert bpy.ops.robot_library.mount_scene_sensor() == {"FINISHED"}
        assert sensor.parent is wrist and not sensor.caryam_mount.enabled
        assert sensor.caryam_mount.target is None
        assert not sensor.caryam_mount.uses_source_anchor
        snap(sensor, source, cube_target)
        before = sensor.matrix_world.copy()
        active(sensor)
        assert bpy.ops.robot_library.detach_scene_sensor() == {"FINISHED"}
        bpy.context.view_layer.update()
        matrix_close(sensor.matrix_world, before)
        assert sensor.parent is None and not sensor.caryam_mount.enabled
        assert sensor.caryam_mount.target is None

        # Saved sphere-empty sensors use their housing as a surface, not as the moving object.
        legacy = bpy.data.objects.new("Sensor: Legacy", None)
        scene.collection.objects.link(legacy)
        legacy["sensor_id"] = sensor["sensor_id"]
        legacy["sensor_definition_json"] = sensor["sensor_definition_json"]
        legacy.location = (8, 0, 2)
        sensor_visuals.update_preview(legacy, addon._sensor_definition(legacy))
        bpy.context.view_layer.update()
        hit = mounting.surface_hit(bpy.context, Vector((8.01, .005, 3)), Vector((0, 0, -1)))
        assert hit is not None and hit[0] is legacy
        legacy_source = mounting.create_magnetic_point(bpy.context, *hit)
        assert legacy_source.parent is legacy
        assert mounting.moving_object(legacy.children[0]) is legacy
        snap(legacy, legacy_source, target)
        assert mounting.robot_root(legacy) is robot

        names = (legacy.name, legacy_source.name, target.name, robot.name)
        blend = path / "sensor_magnets.blend"
        bpy.ops.wm.save_as_mainfile(filepath=str(blend))
        bpy.ops.wm.open_mainfile(filepath=str(blend))
        legacy, legacy_source, target, robot = [bpy.data.objects[name] for name in names]
        assert legacy.parent is target and legacy_source.parent is legacy
        assert legacy.caryam_mount.enabled and legacy.caryam_mount.uses_source_anchor
        assert legacy.caryam_mount.source_anchor is legacy_source
        assert legacy.caryam_mount.target is target
        assert mounting.moving_object(legacy_source) is legacy
        assert mounting.robot_root(legacy) is robot
        assert len(addon._sensor_mounts(robot)) == len(sensors)
    print("Sensor magnetic surface picking, ownership, mounting, detach, persistence and MuJoCo export passed")
finally:
    robot_arm_library.unregister()
