"""Caryam adapters consume the separately distributed sensor library."""

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
import warnings
from dataclasses import asdict
from pathlib import Path
from xml.etree import ElementTree

import mujoco_sensor_library as sensors

from robot_arm_library.asset_sources import AssetSource, discover_from_sources
from robot_arm_library.sensor_compilers.mujoco import compile_camera, compile_lidar
from robot_arm_library.sensor_spec import CameraSensorSpec, LidarSensorSpec, SensorDefinition
from robot_arm_library.setup_parser import SensorCatalogWarning, discover_sensor_definitions
from robot_arm_library.world_export_worker import _sensor_mount
from robot_arm_library.world_exporter import RobotInstance, RobotMount, SensorMount, write_world_export


class SensorLibraryIntegrationTests(unittest.TestCase):
    def test_specs_and_compiler_are_imported_not_duplicated(self):
        self.assertIs(CameraSensorSpec, sensors.CameraSensorSpec)
        self.assertIs(LidarSensorSpec, sensors.LidarSensorSpec)
        self.assertIs(compile_camera, sensors.compile_camera)
        self.assertIs(compile_lidar, sensors.compile_lidar)
        self.assertTrue(issubclass(SensorDefinition, sensors.SensorDefinition))

    def test_installed_library_works_without_checkout_or_blender(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, "-I", "-c", (
                    "import sys; import mujoco_sensor_library as lib; "
                    "catalog = lib.load_catalog(); "
                    "assert len(catalog.sensors) == 5 and not catalog.diagnostics; "
                    "assert not {'robot_arm_library', 'bpy', 'rclpy', 'mujoco'} & sys.modules.keys()"
                )],
                cwd=directory, capture_output=True, text=True, timeout=30,
            )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_catalog_sources_deduplicate_and_derive_graph_ports(self):
        path = sensors.default_catalog_path()
        robots, definitions = discover_from_sources([
            AssetSource(str(path.parent), "sensor"), AssetSource(str(path), "sensor"),
        ])
        self.assertEqual(robots, [])
        self.assertEqual(len(definitions), 5)
        self.assertEqual({item.category for item in definitions}, {
            "lidar", "rgb_camera", "depth_camera", "rgbd_camera", "imu",
        })
        by_category = {item.category: item for item in definitions}
        self.assertEqual([port.semantic for port in by_category["rgb_camera"].output_ports()], ["rgb_image"])
        self.assertEqual([port.semantic for port in by_category["depth_camera"].output_ports()], ["depth"])
        self.assertEqual([port.semantic for port in by_category["rgbd_camera"].output_ports()], ["rgb_image", "depth"])
        for definition in definitions:
            self.assertIsInstance(definition.source_path, str)
            mount = SensorMount(definition, definition.sensor_id, (0, 0, 0), (1, 0, 0, 0), "wrist")
            self.assertEqual(_sensor_mount(asdict(mount)), mount)

    def test_invalid_registry_entries_are_visible_without_losing_valid_entries(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            path.write_text(json.dumps({"schema_version": 1, "sensors": {
                "valid": {"category": "rgb_camera"},
                "bad": {"category": "lidar", "lidar": {"horizontal_samples": True}},
            }}), encoding="utf-8")
            with warnings.catch_warnings(record=True) as messages:
                warnings.simplefilter("always", SensorCatalogWarning)
                definitions = discover_sensor_definitions(path)
            self.assertEqual([item.sensor_id for item in definitions], ["valid"])
            self.assertTrue(any("bad" in str(item.message) for item in messages))
            path.write_text(json.dumps({"schema_version": 999, "sensors": {}}), encoding="utf-8")
            with self.assertWarns(SensorCatalogWarning):
                self.assertEqual(discover_sensor_definitions(path), [])

    def test_robot_only_registry_does_not_generate_sensor_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "registry.json"
            path.write_text(json.dumps({"robots": {}}), encoding="utf-8")
            with warnings.catch_warnings(record=True) as messages:
                warnings.simplefilter("always")
                self.assertEqual(discover_sensor_definitions(path), [])
            self.assertEqual(messages, [])

    def test_all_camera_categories_export_under_bodies_and_roots(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "arm.xml"
            source.write_text(
                '<mujoco><worldbody><body name="base"><geom type="sphere" size=".1"/>'
                '<body name="wrist" pos="0 0 .2"/></body></worldbody></mujoco>', encoding="utf-8",
            )
            original = source.read_bytes()
            mounts = tuple(
                SensorMount(
                    SensorDefinition(category, category, category, "registry.json", camera=CameraSensorSpec()),
                    f"{category}_{location}", (0, 0, .1), (1, 0, 0, 0), location,
                )
                for category in ("rgb_camera", "depth_camera", "rgbd_camera")
                for location in ("", "wrist")
            )
            robot = RobotInstance("arm_1", "arm", str(source), (0, 0, 0), (1, 0, 0, 0), {}, mounts)
            world, manifest_path, _ = write_world_export(root / "world.xml", [robot])
            generated = root / "world_assets" / "arm_1_sensorized.xml"
            self.assertEqual(len(ElementTree.parse(generated).findall(".//camera")), 3)
            self.assertEqual(len(ElementTree.parse(world).findall(".//camera")), 3)
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(len(manifest["robots"][0]["sensors"]), 6)
            self.assertEqual(source.read_bytes(), original)
            if importlib.util.find_spec("mujoco") is not None:
                import mujoco

                self.assertEqual(mujoco.MjModel.from_xml_path(str(world)).ncam, 6)
                mounted = RobotInstance(
                    "arm_2", "arm", str(source), (0, 0, 0), (1, 0, 0, 0), {}, mounts,
                    RobotMount("robot", "arm_1", "wrist"),
                )
                parent = RobotInstance("arm_1", "arm", str(source), (0, 0, 0), (1, 0, 0, 0), {})
                world, _, _ = write_world_export(root / "mounted.xml", [parent, mounted])
                self.assertEqual(mujoco.MjModel.from_xml_path(str(world)).ncam, 6)


if __name__ == "__main__":
    unittest.main()
