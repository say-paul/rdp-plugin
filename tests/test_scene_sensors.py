"""Independent world/body sensors and physically defined MuJoCo IMU output."""

import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from xml.etree import ElementTree

from robot_arm_library.sensor_spec import SensorDefinition
from robot_arm_library.world_exporter import (
    RobotInstance, RobotMount, SceneMeshInstance, SensorMount, WorldOptions,
    _write_world_export_external, build_world_xml, write_world_export,
)
from tests.test_world_physics import _cube_obj


class SceneSensorTests(unittest.TestCase):
    def sensor(self, category, name="sensor_test", **kwargs):
        definition = SensorDefinition.from_dict(category, {"category": category}, "snapshot.json")
        return SensorMount(definition, name, (0, 0, 2), (1, 0, 0, 0), **kwargs)

    def test_fixed_world_export_contains_all_sensor_categories_and_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            mounts = [self.sensor(category, "sensor_" + category) for category in
                      ("rgb_camera", "depth_camera", "rgbd_camera", "lidar", "imu")]
            world, manifest, renderer = write_world_export(Path(directory) / "world.xml", [], scene_sensors=mounts)
            root = ElementTree.parse(world).getroot()
            self.assertEqual(len(root.findall("worldbody/body")), 5)
            self.assertEqual(len(root.findall(".//camera")), 3)
            self.assertEqual(len(root.findall(".//sensor/accelerometer")), 1)
            self.assertEqual(len(root.findall(".//sensor/gyro")), 1)
            value = json.loads(manifest.read_text())
            self.assertEqual(len(value["sensors"]), 5)
            self.assertEqual(value["robots"], [])
            compile(renderer.read_text(), str(renderer), "exec")

    def test_missing_parent_and_duplicate_names_are_not_silently_dropped(self):
        with self.assertRaisesRegex(ValueError, "parent body"):
            build_world_xml([], "world.xml", scene_sensors=[self.sensor("imu", body_name="missing")])
        sensor = self.sensor("imu")
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            build_world_xml([], "world.xml", scene_sensors=[sensor, sensor])

    def test_export_worker_reads_optional_scene_sensors_and_one_shot_imu(self):
        if not importlib.util.find_spec("mujoco"):
            self.skipTest("MuJoCo is not installed")
        with tempfile.TemporaryDirectory() as directory:
            world, manifest, renderer = _write_world_export_external(
                Path(directory) / "world.xml", [], [], sys.executable, WorldOptions(), [self.sensor("imu")],
            )
            self.assertTrue(world.is_file())
            self.assertEqual(json.loads(manifest.read_text())["sensors"][0]["sensor"]["category"], "imu")
            result = subprocess.run([sys.executable, str(renderer), "--sensors-only"],
                                    capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("accelerometer (m/s^2, site axes)", result.stdout)
            self.assertIn("gyroscope (rad/s, site axes)", result.stdout)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
    def test_world_welded_imu_uses_native_mujoco_zero_readings(self):
        import mujoco
        import numpy as np

        mount = self.sensor("imu")
        mount = SensorMount(mount.sensor, mount.mount_name, mount.position, (math.sqrt(.5), math.sqrt(.5), 0, 0))
        with tempfile.TemporaryDirectory() as directory:
            world, _, _ = write_world_export(Path(directory) / "world.xml", [], scene_sensors=[mount])
            model = mujoco.MjModel.from_xml_path(str(world))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            acceleration = model.sensor("sensor_test_accelerometer").id
            gyro = model.sensor("sensor_test_gyroscope").id
            address = model.sensor_adr[acceleration]
            np.testing.assert_allclose(data.sensordata[address:address + 3], 0, atol=1e-8)
            address = model.sensor_adr[gyro]
            np.testing.assert_allclose(data.sensordata[address:address + 3], 0, atol=1e-8)
            self.assertEqual(model.ngeom, 1)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
    def test_robot_imu_specific_force_uses_site_axes_on_supported_dynamic_body(self):
        import mujoco
        import numpy as np

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "robot.xml"
            source.write_text('<mujoco><worldbody><body name="base"><joint type="hinge" axis="0 0 1"/>'
                              '<geom size=".1"/></body></worldbody></mujoco>')
            mount = self.sensor("imu")
            mount = SensorMount(mount.sensor, "imu", (0, 0, 0), (math.sqrt(.5), math.sqrt(.5), 0, 0), "base")
            robot = RobotInstance("robot", "robot", str(source), (0, 0, 2), (1, 0, 0, 0), {}, (mount,))
            world, _, _ = write_world_export(root / "world.xml", [robot])
            model = mujoco.MjModel.from_xml_path(str(world))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            site = model.site("robot_imu_imu_site").id
            expected = data.site_xmat[site].reshape(3, 3).T @ -model.opt.gravity
            address = model.sensor_adr[model.sensor("robot_imu_accelerometer").id]
            np.testing.assert_allclose(data.sensordata[address:address + 3], expected, atol=1e-8)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
    def test_body_attached_imu_reports_freefall_and_angular_rate(self):
        import mujoco
        import numpy as np

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "body.obj"
            _cube_obj(source)
            mesh = SceneMeshInstance("body", str(source), (1, 0, 0, 1), position=(0, 0, 5), dynamic=True)
            imu = self.sensor("imu", body_name=mesh.asset_name)
            world, _, _ = write_world_export(root / "world.xml", [], [mesh], scene_sensors=[imu])
            model = mujoco.MjModel.from_xml_path(str(world))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            address = model.sensor_adr[model.sensor("sensor_test_accelerometer").id]
            np.testing.assert_allclose(data.sensordata[address:address + 3], 0, atol=1e-8)
            data.qvel[5] = 2
            mujoco.mj_forward(model, data)
            address = model.sensor_adr[model.sensor("sensor_test_gyroscope").id]
            np.testing.assert_allclose(data.sensordata[address:address + 3], [0, 0, 2], atol=1e-8)

    @unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
    def test_scene_sensors_work_through_the_mounted_world_builder(self):
        import mujoco

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "body.obj"
            _cube_obj(source)
            base = SceneMeshInstance("base", str(source), (1, 0, 0, 1))
            child = SceneMeshInstance("child", str(source), (0, 1, 0, 1), mount=RobotMount("scene", "base"))
            imu = self.sensor("imu", body_name=child.asset_name)
            world, _, _ = write_world_export(root / "world.xml", [], [child, base], scene_sensors=[imu])
            model = mujoco.MjModel.from_xml_path(str(world))
            self.assertEqual(model.body_parentid[model.body("sensor_test").id], model.body(child.asset_name).id)
            self.assertEqual(model.nsensor, 2)


if __name__ == "__main__":
    unittest.main()
