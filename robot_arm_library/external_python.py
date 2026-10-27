"""Expose pure-Python add-on dependencies to external simulation workers."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import tempfile

from .sensor_spec import _sensors


def worker_environment(directory: Path) -> dict[str, str]:
    """Create an isolated import directory owned by the worker's session."""
    imports = Path(tempfile.mkdtemp(prefix="python-", dir=directory))
    source = Path(_sensors.__file__).resolve().parent
    target = imports / "mujoco_sensor_library"
    # Share only this pure-Python package, not Blender's native site-packages.
    try:
        target.symlink_to(source, target_is_directory=True)
    except OSError:
        shutil.copytree(source, target, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    environment = os.environ.copy()
    addon_parent = Path(__file__).resolve().parent.parent
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, (
        str(addon_parent), str(imports), environment.get("PYTHONPATH", ""),
    )))
    environment.pop("PYTHONHOME", None)
    return environment
