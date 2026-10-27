"""Depth-tested sensor housings, optional X-ray and live parent motion."""
# ruff: noqa: E402

import json
import os
from pathlib import Path
import sys
import time

import bpy
from mathutils import Matrix

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import sensor_visuals as visuals
from robot_arm_library.physics import blender as live
from robot_arm_library.sensor_spec import CameraSensorSpec, SensorDefinition


def close(actual, expected):
    assert max(abs(actual[r][c] - expected[r][c]) for r in range(4) for c in range(4)) < 2e-5


def pump(predicate):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        live._tick()
        live._reap_clients()
        if predicate():
            return
        assert live.active(), live.status()
        time.sleep(.01)
    raise AssertionError(live.status())


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    definition = SensorDefinition("camera", "Camera", "rgb_camera", "snapshot.json", camera=CameraSensorSpec())
    mount = visuals.create_mount(scene.collection, "Sensor: Camera", definition)
    mount["sensor_id"] = definition.sensor_id
    mount["sensor_category"] = definition.category
    mount["sensor_definition_json"] = json.dumps(definition.to_dict())
    assert not mount.show_in_front, "Sensor geometry is forced to draw over the robot"
    mount.show_in_front = True
    visuals.update_preview(mount, definition)
    assert mount.show_in_front, "An intentional X-ray choice was lost on preview refresh"
    mount.show_in_front = False
    visuals.update_preview(mount, definition)
    assert not mount.show_in_front

    legacy = bpy.data.objects.new("Sensor: Legacy Camera", None)
    scene.collection.objects.link(legacy)
    legacy["sensor_id"] = definition.sensor_id
    legacy["sensor_category"] = definition.category
    legacy["sensor_definition_json"] = mount["sensor_definition_json"]
    legacy.location = (1, 0, 1)
    legacy.show_in_front = True
    legacy["caryam_sensor_visual_version"] = 2
    visuals.update_preview(legacy, definition)
    housing = visuals.housing_object(legacy)
    assert housing is not None and not housing.show_in_front and not legacy.show_in_front
    housing.show_in_front = True
    visuals.update_preview(legacy, definition)
    assert visuals.housing_object(legacy).show_in_front
    visuals.housing_object(legacy).show_in_front = False

    bpy.ops.mesh.primitive_cube_add(size=.2, location=(0, 0, 2))
    body = bpy.context.object
    bpy.ops.rigidbody.object_add()
    body.rigid_body.collision_shape = "BOX"
    mount.matrix_world = Matrix.Translation((0, 0, 2.12))
    bpy.context.view_layer.objects.active = mount
    mount.caryam_sensor_target = body
    assert bpy.ops.robot_library.mount_scene_sensor() == {"FINISHED"}
    bpy.context.view_layer.update()
    before = mount.matrix_world.copy()
    snapshot = mount["sensor_definition_json"]
    mount["caryam_sensor_visual_version"] = 2
    mount.show_in_front = True
    visuals._upgrade_previews()
    assert not mount.show_in_front and mount.parent is body
    close(mount.matrix_world, before)
    assert mount["sensor_definition_json"] == snapshot
    local = body.matrix_world.inverted() @ mount.matrix_world
    body_before = body.matrix_world.copy()
    scene.robot_mujoco_python = os.environ.get("CARYAM_TEST_MUJOCO_PYTHON", str(ROOT / ".venv/bin/python"))
    assert bpy.ops.robot_library.live_physics(action="START") == {"FINISHED"}
    pump(lambda: live.state() == "RUNNING" and live._session.time > .2)
    assert body.matrix_world.translation.z < body_before.translation.z - .05
    close(mount.matrix_world, body.matrix_world @ local)
    assert not mount.show_in_front
    live.stop(wait=True)
    close(mount.matrix_world, before)
    close(body.matrix_world, body_before)
    print("Sensor default occlusion, X-ray opt-in, legacy upgrade and live parent motion passed")
finally:
    live.stop(wait=True)
    robot_arm_library.unregister()
