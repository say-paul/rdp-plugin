import json
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

from robot_arm_library.sensor_spec import CameraSensorSpec, LidarSensorSpec, SensorDefinition
from robot_arm_library.world_exporter import RobotInstance, SensorMount, _model_has_free_joint, write_world_export


class WorldExporterSensorTests(unittest.TestCase):
    def test_free_joint_in_included_model_is_detected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "body.xml").write_text(
                "<mujoco><worldbody><body name='base'><freejoint/></body></worldbody></mujoco>",
                encoding="utf-8",
            )
            model = root / "model.xml"
            model.write_text("<mujoco><include file='body.xml'/></mujoco>", encoding="utf-8")
            self.assertTrue(_model_has_free_joint(model))

    def test_body_mount_is_written_into_a_model_copy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "arm.xml"
            source.write_text(
                "<mujoco><worldbody><body name='base'><body name='wrist'/></body></worldbody></mujoco>",
                encoding="utf-8",
            )
            lidar = SensorDefinition("lidar", "Front", "lidar", "registry.json", lidar=LidarSensorSpec(horizontal_samples=4))
            instance = RobotInstance("arm_1", "arm", str(source), (0, 0, 0), (1, 0, 0, 0), {}, (
                SensorMount(lidar, "front", (0, 0, 0), (1, 0, 0, 0), "wrist"),
            ))
            world_path, _, _ = write_world_export(root / "world.xml", [instance])
            generated = root / "world_assets" / "arm_1_sensorized.xml"
            self.assertTrue(generated.is_file())
            sensorized = ElementTree.parse(generated).getroot()
            self.assertIsNotNone(sensorized.find(".//body[@name='wrist']/body[@name='front']/site"))
            self.assertEqual(len(sensorized.findall(".//sensor/rangefinder")), 4)
            self.assertIn("sensorized.xml", world_path.read_text(encoding="utf-8"))
            self.assertNotIn("front_rf", source.read_text(encoding="utf-8"))

    def test_export_contains_lidar_depth_and_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "arm.xml"
            source.write_text("<mujoco><worldbody><body name='arm'/></worldbody></mujoco>", encoding="utf-8")
            lidar = SensorDefinition("lidar", "Front", "lidar", "registry.json", lidar=LidarSensorSpec(horizontal_samples=8))
            depth = SensorDefinition("depth", "Chest", "depth_camera", "registry.json", camera=CameraSensorSpec())
            instance = RobotInstance("arm_1", "arm", str(source), (0, 0, 0), (1, 0, 0, 0), {}, (
                SensorMount(lidar, "front", (0, 0, 0.2), (1, 0, 0, 0)),
                SensorMount(depth, "chest", (0, 0, 0.3), (1, 0, 0, 0)),
            ))
            world_path, manifest_path, render_path = write_world_export(root / "world.xml", [instance])
            world = ElementTree.parse(world_path).getroot()
            self.assertEqual(len(world.findall(".//sensor/rangefinder")), 8)
            self.assertIsNotNone(world.find(".//body/body/camera"))
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["robots"][0]["sensors"][0]["sensor"]["category"], "lidar")
            self.assertIn("_depth.npy", render_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
