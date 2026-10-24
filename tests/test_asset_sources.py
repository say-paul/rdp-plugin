import json
import tempfile
import unittest
from pathlib import Path

from robot_arm_library.asset_sources import AssetSource, discover_from_sources


class AssetSourceTests(unittest.TestCase):
    def test_sources_merge_and_deduplicate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "robot.json").write_text(json.dumps({"name": "Arm", "model_path": "arm.xml"}), encoding="utf-8")
            (root / "registry.json").write_text(json.dumps({"sensors": {"lidar": {"category": "lidar"}}}), encoding="utf-8")
            robots, sensors = discover_from_sources([AssetSource(directory), AssetSource(directory)])
            self.assertEqual(len(robots), 1)
            self.assertEqual(len(sensors), 1)
            robots, sensors = discover_from_sources([AssetSource(directory, "sensor")])
            self.assertEqual(robots, [])
            self.assertEqual(len(sensors), 1)
            self.assertEqual(discover_from_sources([AssetSource(directory, enabled=False)]), ([], []))
        with self.assertRaises(ValueError):
            discover_from_sources([AssetSource("/tmp", "unknown")])


if __name__ == "__main__":
    unittest.main()
