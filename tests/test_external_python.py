"""Worker imports must not depend on Blender's site-packages or a virtualenv."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from robot_arm_library import world_exporter
from robot_arm_library.external_python import worker_environment
from robot_arm_library.physics.client import SimulationClient
from robot_arm_library.sensor_spec import _sensors


class WorkerEnvironmentTests(unittest.TestCase):
    def test_dependency_available_without_site_packages_and_native_path_leaks(self):
        with tempfile.TemporaryDirectory(prefix="worker imports with spaces ") as directory:
            owner = Path(directory)
            with patch.dict(os.environ, {"PYTHONPATH": "/user/configured", "PYTHONHOME": "/blender/python"}):
                environment = worker_environment(owner)
            paths = environment["PYTHONPATH"].split(os.pathsep)
            self.assertEqual(Path(paths[0]), Path(__file__).resolve().parents[1])
            imports = Path(paths[1])
            self.assertEqual(imports.parent, owner)
            self.assertEqual([item.name for item in imports.iterdir()], ["mujoco_sensor_library"])
            self.assertEqual(paths[2:], ["/user/configured"])
            self.assertNotIn(str(Path(_sensors.__file__).resolve().parents[1]), paths)
            self.assertNotIn("PYTHONHOME", environment)
            result = subprocess.run([
                sys.executable, "-B", "-S", "-c",
                "import importlib.util, sys; "
                "import mujoco_sensor_library as sensors; "
                "import robot_arm_library.physics.worker; "
                "import robot_arm_library.world_export_worker; "
                "assert len(sensors.load_catalog().sensors) == 5; "
                "assert importlib.util.find_spec('numpy') is None; "
                "assert 'bpy' not in sys.modules and 'mujoco' not in sys.modules",
            ], cwd=owner, env=environment, capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(Path(_sensors.__file__).is_file(), "Session cleanup must not remove the library")

    def test_copy_fallback_when_symlinks_are_unavailable(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(Path, "symlink_to", side_effect=OSError("Symlinks unavailable")):
                environment = worker_environment(Path(directory))
            imports = Path(environment["PYTHONPATH"].split(os.pathsep)[1])
            package = imports / "mujoco_sensor_library"
            self.assertFalse(package.is_symlink())
            self.assertEqual((package / "__init__.py").read_bytes(), Path(_sensors.__file__).read_bytes())
            self.assertTrue((package / "registry.json").is_file())
            self.assertFalse((package / "__pycache__").exists())
        self.assertTrue(Path(_sensors.__file__).is_file())

    def test_live_worker_receives_isolated_dependency_and_cleans_up_after_spawn_failure(self):
        client = SimulationClient(sys.executable)
        imports = None

        def fail_to_start(*args, **kwargs):
            nonlocal imports
            imports = Path(kwargs["env"]["PYTHONPATH"].split(os.pathsep)[1])
            self.assertEqual(imports.parent, client.directory)
            self.assertTrue((imports / "mujoco_sensor_library" / "__init__.py").is_file())
            raise OSError("Cannot start selected Python")

        try:
            with patch("robot_arm_library.physics.client.subprocess.Popen", side_effect=fail_to_start):
                with self.assertRaises(OSError):
                    client.start({"version": 1, "robots": [], "scene_meshes": []}, [])
        finally:
            client.close()
        self.assertIsNotNone(imports)
        self.assertFalse(client.directory.exists())
        self.assertTrue(Path(_sensors.__file__).is_file())

    def test_export_worker_receives_isolated_dependency_and_cleans_up_after_spawn_failure(self):
        imports = None

        def fail_to_start(*args, **kwargs):
            nonlocal imports
            imports = Path(kwargs["env"]["PYTHONPATH"].split(os.pathsep)[1])
            self.assertTrue((imports / "mujoco_sensor_library" / "__init__.py").is_file())
            raise OSError("Cannot start selected Python")

        with tempfile.TemporaryDirectory() as directory:
            with patch.object(world_exporter.subprocess, "run", side_effect=fail_to_start):
                with self.assertRaisesRegex(RuntimeError, "Could not start MuJoCo export"):
                    world_exporter._write_world_export_external(Path(directory) / "world.xml", [], [], sys.executable)
        self.assertIsNotNone(imports)
        self.assertFalse(imports.exists())
        self.assertTrue(Path(_sensors.__file__).is_file())

    def test_repeated_preparation_uses_separate_owned_import_directories(self):
        with tempfile.TemporaryDirectory() as directory:
            first = worker_environment(Path(directory))
            second = worker_environment(Path(directory))
            self.assertNotEqual(first["PYTHONPATH"], second["PYTHONPATH"])


if __name__ == "__main__":
    unittest.main()
