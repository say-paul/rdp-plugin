"""Play and export must use external Python when Blender cannot import MuJoCo."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import bpy
from mathutils import Vector

repository_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(repository_root))

import robot_arm_library
from robot_arm_library import blender_addon as addon, mounting


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    with tempfile.TemporaryDirectory() as directory:
        root_dir = Path(directory)
        model_path = root_dir / "arm.xml"
        model_path.write_text(
            '<mujoco><worldbody><body name="base"><geom type="box" size=".1 .1 .02"/>'
            '<body name="wrist" pos="0 0 .2"><joint name="hinge"/><geom type="sphere" size=".03"/>'
            '</body></body></worldbody></mujoco>', encoding="utf-8",
        )
        scene = bpy.context.scene
        item = scene.robot_library.add()
        item.robot_id = "arm"
        item.name = "Arm"
        item.model_path = str(model_path)
        item.source_path = str(model_path)
        robot = addon._place_robot(item, Vector((0, 0, .5)))
        assert robot["import_mode"] == "mujoco_compiled"
        bpy.ops.mesh.primitive_cube_add(size=1)
        table = bpy.context.object
        mounting.mount_robot(robot, table)
        bpy.context.view_layer.update()
        venv = repository_root / ".venv" / "bin" / "python"
        executable = os.environ.get("CARYAM_TEST_MUJOCO_PYTHON") or (str(venv) if venv.exists() else shutil.which("python"))
        assert executable, "Set CARYAM_TEST_MUJOCO_PYTHON to a Python with MuJoCo"
        scene.robot_mujoco_python = executable
        bpy.ops.wm.save_as_mainfile(filepath=str(root_dir / "test.blend"))
        launched = []
        real_popen = subprocess.Popen

        def launch(command, *args, **kwargs):
            if len(command) > 1 and command[1] == "-m":
                return real_popen(command, *args, **kwargs)
            launched.append(command)
            return SimpleNamespace(pid=123)

        with patch.dict(sys.modules, {"mujoco": None}), patch.object(addon.subprocess, "Popen", side_effect=launch):
            assert bpy.ops.robot_library.launch_mujoco() == {"FINISHED"}
            assert launched[-1] == [executable, str(root_dir / "render_world.py")]
            manifest = json.loads((root_dir / "caryam_world.json").read_text())
            assert manifest["robots"][0]["mount"]["target_instance_id"] == table.name
            assert (root_dir / "caryam_world.usd").is_file()
            assert bpy.ops.robot_library.export_world(filepath=str(root_dir / "export.xml")) == {"FINISHED"}
            assert (root_dir / "export.xml").is_file()
            assert bpy.ops.robot_library.export_and_render(filepath=str(root_dir / "render.xml")) == {"FINISHED"}
            assert launched[-1][0] == executable
            scene.robot_mujoco_python = str(root_dir / "missing-python")
            before = len(launched)
            try:
                bpy.ops.robot_library.launch_mujoco()
            except RuntimeError as error:
                assert "missing-python" in str(error) and "Python" in str(error)
            else:
                raise AssertionError("Missing external Python was not reported")
            assert len(launched) == before
    print("External MuJoCo Play, Export, and Export and Render regression passed")
finally:
    robot_arm_library.unregister()
