import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import install_blender_addon as installer


class BlenderInstallTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        self.library = root / "sensor-library"
        self.library.mkdir()
        (self.library / "pyproject.toml").write_text("[project]\nname = 'mujoco-sensor-library'\n")
        self.archive = root / "robot_arm_library.zip"
        self.archive.touch()
        self.modules = root / "blender" / "scripts" / "modules"
        self.bpy = SimpleNamespace(
            utils=Mock(), context=SimpleNamespace(preferences=SimpleNamespace(addons={installer.ADDON: object()})),
            ops=SimpleNamespace(preferences=Mock(), wm=Mock()),
        )
        self.bpy.utils.user_resource.return_value = str(self.modules)
        for name in ("addon_disable", "addon_install", "addon_enable"):
            getattr(self.bpy.ops.preferences, name).return_value = {"FINISHED"}
        self.bpy.ops.wm.save_userpref.return_value = {"FINISHED"}
        self.addon_utils = Mock()
        self.addon_utils.check.return_value = (True, True)
        run_patch = patch.object(installer.subprocess, "run")
        self.run = run_patch.start()
        self.addCleanup(run_patch.stop)
        forget_patch = patch.object(installer, "_forget_addon_modules")
        self.forget = forget_patch.start()
        self.addCleanup(forget_patch.stop)
        path_patch = patch.object(installer.sys, "path", list(installer.sys.path))
        path_patch.start()
        self.addCleanup(path_patch.stop)

    def install(self):
        return installer.install_addon(
            self.bpy, self.addon_utils, python_executable="/chosen/python",
            sensor_library=self.library, addon_zip=self.archive,
        )

    def test_installs_separate_dependency_and_saves_only_after_enable(self):
        self.assertEqual(self.install(), self.modules)
        self.run.assert_called_once_with([
            "/chosen/python", "-m", "pip", "install", "--upgrade", "--no-deps", "--no-compile",
            "--target", str(self.modules), str(self.library),
        ], check=True)
        self.bpy.utils.user_resource.assert_called_once_with("SCRIPTS", path="modules", create=True)
        self.bpy.ops.preferences.addon_disable.assert_called_once_with(module=installer.ADDON)
        self.bpy.ops.preferences.addon_install.assert_called_once_with(overwrite=True, filepath=str(self.archive))
        self.bpy.ops.preferences.addon_enable.assert_called_once_with(module=installer.ADDON)
        self.addon_utils.check.assert_called_once_with(installer.ADDON)
        self.bpy.ops.wm.save_userpref.assert_called_once_with()
        self.forget.assert_called_once_with()
        self.assertIn(str(self.modules), installer.sys.path)

    def test_dependency_failure_does_not_disable_or_save_addon(self):
        self.run.side_effect = subprocess.CalledProcessError(1, ["pip"])
        with self.assertRaises(subprocess.CalledProcessError):
            self.install()
        self.bpy.ops.preferences.addon_disable.assert_not_called()
        self.bpy.ops.preferences.addon_install.assert_not_called()
        self.bpy.ops.wm.save_userpref.assert_not_called()

    def test_cancelled_install_does_not_enable_or_save(self):
        self.bpy.ops.preferences.addon_install.return_value = {"CANCELLED"}
        with self.assertRaisesRegex(RuntimeError, "Installing add-on failed"):
            self.install()
        self.bpy.ops.preferences.addon_enable.assert_not_called()
        self.bpy.ops.wm.save_userpref.assert_not_called()

    def test_failed_load_is_not_reported_as_success(self):
        self.addon_utils.check.return_value = (True, False)
        with self.assertRaisesRegex(RuntimeError, "did not successfully load"):
            self.install()
        self.bpy.ops.wm.save_userpref.assert_not_called()

    def test_first_install_does_not_disable_an_uninstalled_addon(self):
        self.bpy.context.preferences.addons.clear()
        self.install()
        self.bpy.ops.preferences.addon_disable.assert_not_called()

    def test_invalid_paths_fail_before_installation(self):
        self.archive.unlink()
        with self.assertRaises(FileNotFoundError):
            self.install()
        self.run.assert_not_called()
        self.bpy.ops.wm.save_userpref.assert_not_called()

    def test_unavailable_modules_directory_does_not_install_to_current_directory(self):
        self.bpy.utils.user_resource.return_value = ""
        with self.assertRaisesRegex(RuntimeError, "Cannot create"):
            self.install()
        self.run.assert_not_called()


@unittest.skipUnless(shutil.which("make"), "make is not installed")
class SensorProjectLocationTests(unittest.TestCase):
    def test_make_install_uses_sibling_or_explicit_sensor_project(self):
        root = Path(installer.__file__).resolve().parent
        environment = dict(os.environ)
        environment.pop("SENSOR_LIBRARY", None)
        for override, expected in (
            (None, root.parent / "mujoco_sensor_library"),
            ("/custom/sensor-project", Path("/custom/sensor-project")),
        ):
            with self.subTest(override=override):
                command = ["make", "--no-print-directory", "-n", "install"]
                if override is not None:
                    command.append(f"SENSOR_LIBRARY={override}")
                result = subprocess.run(command, cwd=root, env=environment, capture_output=True, text=True, check=True)
                self.assertIn(f'--sensor-library "{expected}"', result.stdout)
                self.assertNotIn("libraries/mujoco_sensor_library", result.stdout)


class ModuleCacheTests(unittest.TestCase):
    def test_only_addon_and_sensor_modules_are_reloaded(self):
        modules = {name: object() for name in (
            "robot_arm_library", "robot_arm_library.sensor_spec", "mujoco_sensor_library",
            "mujoco_sensor_library.specs", "another_addon", "robot_arm_library_extra",
        )}
        with patch.object(installer.sys, "modules", modules):
            installer._forget_addon_modules()
        self.assertEqual(set(modules), {"another_addon", "robot_arm_library_extra"})


if __name__ == "__main__":
    unittest.main()
