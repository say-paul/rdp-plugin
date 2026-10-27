"""Add independent scene sensors, mount later, and keep instances out of the catalog UI."""
# ruff: noqa: E402

import json
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import patch

import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import robot_arm_library
from robot_arm_library import blender_addon as addon, sensor_visuals
from robot_arm_library.physics import blender as live

assert Path(addon.__file__).resolve() == ROOT / "robot_arm_library/blender_addon.py"


def close(left, right):
    assert max(abs(left[r][c] - right[r][c]) for r in range(4) for c in range(4)) < 2e-5


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


class Layout:
    enabled = True

    def __init__(self):
        self.labels = []
        self.operators = []

    def label(self, *, text, **kwargs):
        self.labels.append(text)

    def box(self):
        return self

    def column(self):
        return self

    def prop(self, *args, **kwargs):
        pass

    def template_list(self, *args, **kwargs):
        pass

    def operator(self, name, **kwargs):
        self.operators.append(name)
        return SimpleNamespace()


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.robot_setup_path = "/nonexistent/robot/source"
    assert bpy.ops.robot_library.refresh() == {"FINISHED"}
    assert len(scene.sensor_library) == 5
    scene.cursor.location = (0.3, 0.4, 2)
    scene.cursor.rotation_euler = (0.2, -0.1, 0.3)
    original_catalog = [(item.sensor_id, item.source_path) for item in scene.sensor_library]
    instances = {}
    for index, item in enumerate(scene.sensor_library):
        assert bpy.ops.robot_library.add_sensor(sensor_index=index) == {"FINISHED"}
        obj = bpy.context.active_object
        assert obj.parent is None and obj.type == "MESH"
        assert list(obj.users_collection) == [scene.collection]
        close(obj.matrix_world, scene.cursor.matrix)
        instances[item.category] = obj
    assert len(scene.sensor_library) == 5
    assert [(item.sensor_id, item.source_path) for item in scene.sensor_library] == original_catalog
    imu = instances["imu"]
    assert not imu.children and sensor_visuals.guide_lines(addon._sensor_definition(imu)) == []
    assert addon.ROBOT_PT_sensor_object.bl_space_type == "PROPERTIES"
    assert addon.ROBOT_PT_sensor_object.bl_context == "object"
    assert addon.ROBOT_PT_sensor_object.poll(bpy.context)
    catalog_layout = Layout()
    with patch.object(addon, "_draw_sensor_mount", side_effect=AssertionError("Catalog must not draw instances")):
        addon.ROBOT_PT_sensor_library.draw(SimpleNamespace(layout=catalog_layout), bpy.context)
    assert "robot_library.add_sensor" in catalog_layout.operators
    assert "robot_library.mount_scene_sensor" not in catalog_layout.operators
    layout = Layout()
    addon._draw_sensor_mount(layout, bpy.context)
    assert "World sensor (fixed)" in layout.labels
    assert "robot_library.mount_scene_sensor" in layout.operators

    with tempfile.TemporaryDirectory(prefix="independent sensor scene ") as directory:
        path = Path(directory)
        sensors = addon._world_scene_sensors()
        assert len(sensors) == 5 and all(not sensor.body_name for sensor in sensors)
        assert addon._world_scene_meshes(path / "world.xml") == []
        world, manifest, _ = addon._export_world_file(str(path / "world.xml"))
        manifest_value = json.loads(manifest.read_text())
        assert len(manifest_value["sensors"]) == 5 and not manifest_value["robots"]
        import mujoco
        compiled = mujoco.MjModel.from_xml_path(str(world))
        assert compiled.ncam == 3 and compiled.nsensor == 362 and compiled.ngeom == 1

        # A sensor-only scene must support the same live lifecycle as a robot scene.
        scene.robot_mujoco_python = os.environ.get("CARYAM_TEST_MUJOCO_PYTHON", str(ROOT / ".venv/bin/python"))
        sensor_poses = {obj: obj.matrix_world.copy() for obj in instances.values()}
        assert bpy.ops.robot_library.live_physics(action="START") == {"FINISHED"}
        pump(lambda: live.state() == "RUNNING")
        assert bpy.ops.robot_library.live_physics(action="PAUSE") == {"FINISHED"}
        pump(lambda: live.state() == "PAUSED")
        live.stop(wait=True)
        for obj, matrix in sensor_poses.items():
            close(obj.matrix_world, matrix)

        model = path / "arm.xml"
        model.write_text('<mujoco><worldbody><body name="base"><geom size=".03"/>'
                         '<body name="wrist" pos="0 0 .2"><joint name="joint"/><geom size=".02"/>'
                         '</body></body></worldbody></mujoco>')
        item = scene.robot_library.add()
        item.robot_id = "robot"
        item.name = "Robot"
        item.model_path = str(model)
        item.source_path = str(model)
        robot = addon._place_robot(item, Vector((1, 2, 3)))
        wrist = next(obj for obj in robot.children_recursive if obj.get("mujoco_body_name") == "wrist")
        bpy.context.view_layer.objects.active = imu
        before = imu.matrix_world.copy()
        snapshot = imu["sensor_definition_json"]
        imu.caryam_sensor_target = wrist
        assert bpy.ops.robot_library.mount_scene_sensor() == {"FINISHED"}
        bpy.context.view_layer.update()
        close(imu.matrix_world, before)
        assert imu.parent is wrist and imu["sensor_definition_json"] == snapshot
        assert scene.collection in imu.users_collection
        assert len(addon._world_scene_sensors()) == 4
        assert addon._sensor_mounts(robot)[0].sensor.category == "imu"
        world, manifest, _ = addon._export_world_file(str(path / "attached.xml"))
        compiled = mujoco.MjModel.from_xml_path(str(world))
        assert compiled.nsensor == 362
        value = json.loads(manifest.read_text())
        assert len(value["sensors"]) == 4 and len(value["robots"][0]["sensors"]) == 1
        assert bpy.ops.robot_library.detach_scene_sensor() == {"FINISHED"}
        bpy.context.view_layer.update()
        close(imu.matrix_world, before)
        assert imu.parent is None and len(addon._world_scene_sensors()) == 5

        # Mounting to a normal scene object follows the exported scene body too.
        bpy.ops.mesh.primitive_cube_add(size=.1, location=(0, 0, 2))
        cube = bpy.context.object
        bpy.ops.rigidbody.object_add()
        cube.rigid_body.collision_shape = "BOX"
        bpy.context.view_layer.objects.active = imu
        imu.caryam_sensor_target = cube
        assert bpy.ops.robot_library.mount_scene_sensor() == {"FINISHED"}
        bpy.context.view_layer.update()
        close(imu.matrix_world, before)
        exported = next(sensor for sensor in addon._world_scene_sensors() if sensor.object_name == imu.name)
        assert exported.body_name.startswith("scene_")
        world, _, _ = addon._export_world_file(str(path / "scene_attached.xml"))
        compiled = mujoco.MjModel.from_xml_path(str(world))
        assert compiled.body_parentid[compiled.body(exported.mount_name).id] == compiled.body(exported.body_name).id
        assert compiled.nsensor == 362
        # Scene-body and independent sensors remain visible and restore after simulation.
        before = imu.matrix_world.copy()
        assert bpy.ops.robot_library.live_physics(action="START") == {"FINISHED"}
        pump(lambda: live.state() == "RUNNING" and live._session.time > .05)
        live.stop(wait=True)
        close(imu.matrix_world, before)

        # A scene mesh mounted on a robot is still its own exported sensor parent.
        addon.mounting.mount_robot(cube, wrist)
        bpy.context.view_layer.update()
        assert addon._sensor_attachment_target(cube) is cube
        assert len(addon._world_scene_sensors()) == 5 and not addon._sensor_mounts(robot)
        world, _, _ = addon._export_world_file(str(path / "mounted_scene_sensor.xml"))
        compiled = mujoco.MjModel.from_xml_path(str(world))
        assert compiled.body_parentid[compiled.body(exported.mount_name).id] == compiled.body(exported.body_name).id
        bpy.context.view_layer.objects.active = imu

        # Standard Blender parenting to a transformed empty is a fixed-world sensor.
        assert bpy.ops.robot_library.detach_scene_sensor() == {"FINISHED"}
        anchor = bpy.data.objects.new("Scene anchor", None)
        scene.collection.objects.link(anchor)
        anchor.location = (1, 2, 3)
        imu.parent = anchor
        imu.matrix_parent_inverse = Matrix.Identity(4)
        bpy.context.view_layer.update()
        exported = next(sensor for sensor in addon._world_scene_sensors() if sensor.object_name == imu.name)
        assert exported.body_name == ""
        assert max(abs(a - b) for a, b in zip(exported.position, imu.matrix_world.translation)) < 1e-5
        assert not addon._sensor_target_poll(imu, imu)
        assert not addon._sensor_target_poll(imu, instances["lidar"])
        print("Scene sensors, IMU, catalog-only sidebar, Object Properties, mount/detach/export and live lifecycle passed")
finally:
    live.stop(wait=True)
    robot_arm_library.unregister()
