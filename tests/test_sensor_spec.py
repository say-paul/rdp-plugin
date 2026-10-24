import json
import tempfile
import unittest
from pathlib import Path

from robot_arm_library.sensor_spec import CameraSensorSpec, LidarSensorSpec, SensorDefinition
from robot_arm_library.setup_parser import discover_sensor_definitions


class SensorSpecTests(unittest.TestCase):
    def test_specs_round_trip_and_derive_ports(self):
        lidar = LidarSensorSpec(channels=2, horizontal_samples=8, range_max=20)
        self.assertEqual(LidarSensorSpec.from_dict(lidar.to_dict()), lidar)
        camera = CameraSensorSpec(resolution_width=320, resolution_height=240)
        self.assertEqual(CameraSensorSpec.from_dict(camera.to_dict()), camera)
        self.assertGreater(camera.vertical_fov_degrees(), 0)
        definition = SensorDefinition("depth", "Depth", "depth_camera", "registry.json", camera=camera, backend_capabilities={"mujoco": "camera_depth"})
        restored = SensorDefinition.from_dict("depth", definition.to_dict(), "registry.json")
        self.assertEqual(restored, definition)
        self.assertEqual(restored.output_ports()[0].shape, (1, 1, 240, 320))
        with self.assertRaises(ValueError):
            SensorDefinition.from_dict("x", {"category": "sonar"}, "registry.json")

    def test_discovery_reads_registry_and_tolerates_missing_file(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(discover_sensor_definitions(root), [])
            (root / "registry.json").write_text(json.dumps({"sensors": {
                "front": {"category": "lidar", "name": "Front", "lidar": {"horizontal_samples": 180}},
                "bad": {"category": "sonar"},
            }}), encoding="utf-8")
            sensors = discover_sensor_definitions(root)
            self.assertEqual([sensor.sensor_id for sensor in sensors], ["front"])
            self.assertEqual(sensors[0].lidar.horizontal_samples, 180)


if __name__ == "__main__":
    unittest.main()
