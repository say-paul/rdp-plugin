"""Install the add-on and its separate sensor dependency from inside Blender."""

from __future__ import annotations

import argparse
import importlib
import subprocess
import sys
from pathlib import Path

ADDON = "robot_arm_library"
SENSOR_PACKAGE = "mujoco_sensor_library"


def _finished(operation: str, result: set[str]) -> None:
    if "FINISHED" not in result:
        raise RuntimeError(f"{operation} failed: {result}")


def _forget_addon_modules() -> None:
    for name in list(sys.modules):
        if any(name == package or name.startswith(package + ".") for package in (ADDON, SENSOR_PACKAGE)):
            del sys.modules[name]
    importlib.invalidate_caches()


def install_addon(bpy, addon_utils, *, python_executable: str, sensor_library: Path, addon_zip: Path) -> Path:
    if not (sensor_library / "pyproject.toml").is_file():
        raise FileNotFoundError(f"Sensor package project not found: {sensor_library}")
    if not addon_zip.is_file():
        raise FileNotFoundError(f"Add-on archive not found: {addon_zip}")
    modules_path = bpy.utils.user_resource("SCRIPTS", path="modules", create=True)
    if not modules_path:
        raise RuntimeError("Cannot create Blender's user Python modules directory")
    modules = Path(modules_path)
    subprocess.run([
        python_executable, "-m", "pip", "install", "--upgrade", "--no-deps", "--no-compile",
        "--target", str(modules), str(sensor_library),
    ], check=True)
    if ADDON in bpy.context.preferences.addons:
        _finished("Disabling existing add-on", bpy.ops.preferences.addon_disable(module=ADDON))
    _forget_addon_modules()
    bpy.utils.refresh_script_paths()
    if str(modules) not in sys.path:
        sys.path.insert(0, str(modules))
    _finished("Installing add-on", bpy.ops.preferences.addon_install(overwrite=True, filepath=str(addon_zip)))
    _finished("Enabling add-on", bpy.ops.preferences.addon_enable(module=ADDON))
    if not all(addon_utils.check(ADDON)):
        raise RuntimeError("Blender did not successfully load and enable the add-on; preferences were not saved")
    _finished("Saving preferences", bpy.ops.wm.save_userpref())
    print(f"Installed separate sensor library in {modules / SENSOR_PACKAGE}")
    print(f"Installed and enabled {ADDON}; restart any already-open Blender windows")
    return modules


def main() -> None:
    import addon_utils
    import bpy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", required=True, help="External Python with pip available")
    parser.add_argument("--sensor-library", required=True, type=Path)
    parser.add_argument("--addon-zip", required=True, type=Path)
    arguments = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    options = parser.parse_args(arguments)
    install_addon(
        bpy, addon_utils, python_executable=options.python,
        sensor_library=options.sensor_library.resolve(), addon_zip=options.addon_zip.resolve(),
    )


if __name__ == "__main__":
    main()
