"""Shared initialization, scene options and standalone renderer regressions."""

from dataclasses import asdict
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from robot_arm_library.physics.runtime import ResetState, initialize
from robot_arm_library.world_exporter import RobotInstance, RobotMount, SceneMeshInstance, WorldOptions, write_world_export
from tests.test_world_physics import _cube_obj


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
class SimulationRuntimeTests(unittest.TestCase):
    def test_options_local_external_and_mounted(self):
        import mujoco
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "arm.xml"
            source.write_text('<mujoco><worldbody><body name="base"><geom size=".1"/></body></worldbody></mujoco>')
            mesh_path = root / "cube.obj"
            _cube_obj(mesh_path)
            mesh = SceneMeshInstance("table", str(mesh_path), (1, 1, 1, 1))
            robot = RobotInstance("arm", "arm", str(source), (0, 0, 0), (1, 0, 0, 0), {},
                                  mount=RobotMount("scene", "table"))
            options = WorldOptions((1, 2, -3), .004)
            for external in (False, True):
                context = patch.dict(sys.modules, {"mujoco": None}) if external else patch.dict(os.environ, {})
                with context:
                    output, manifest, _ = write_world_export(root / f"world_{external}.xml", [robot], [mesh],
                                                            python_executable=sys.executable, world_options=options)
                model = mujoco.MjModel.from_xml_path(str(output))
                self.assertAlmostEqual(model.opt.timestep, .004)
                self.assertEqual(tuple(model.opt.gravity), (1, 2, -3))
                self.assertEqual(json.loads(manifest.read_text())["world_options"], {"gravity": [1, 2, -3], "timestep": .004})

    def test_reset_full_integration_state_and_standalone(self):
        import mujoco
        import numpy as np
        with tempfile.TemporaryDirectory(prefix="standalone runtime ") as directory:
            root = Path(directory)
            source = root / "free.xml"
            source.write_text('<mujoco><worldbody><body name="base" pos="0 0 .1"><freejoint name="base_free"/>'
                              '<geom size=".1"/><body name="link"><joint name="hinge"/><geom size=".05"/></body>'
                              '</body></worldbody><actuator><motor joint="hinge"/></actuator>'
                              '<keyframe><key name="home" qpos="0 0 .3 1 0 0 0 .4"/></keyframe></mujoco>')
            robot = RobotInstance("Robot.001", "robot", str(source), (1, 2, 3), (1, 0, 0, 0), {"hinge": .7})
            output, manifest_path, renderer = write_world_export(root / "world.xml", [robot])
            manifest = json.loads(manifest_path.read_text())
            model = mujoco.MjModel.from_xml_path(str(output))
            data = initialize(mujoco, model, manifest)
            self.assertAlmostEqual(data.qpos[-1], .7)
            np.testing.assert_allclose(data.qpos[:3], [1, 2, 3.3])
            reset = ResetState(mujoco, model, data)
            for _ in range(5):
                data.ctrl[:] = .2
                mujoco.mj_step(model, data)
            reset.restore(mujoco, model, data)
            actual = np.empty_like(reset.value)
            mujoco.mj_getState(model, data, actual, reset.spec)
            np.testing.assert_allclose(actual, reset.value, atol=1e-10)
            env = dict(os.environ)
            env.pop("PYTHONPATH", None)
            result = subprocess.run([sys.executable, str(renderer), "--sensors-only"], cwd="/tmp", env=env,
                                    capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((root / "caryam_physics_runtime.py").is_file())

    def test_invalid_options(self):
        for options in ({"timestep": 0}, {"timestep": float("nan")}, {"gravity": (0, 0, float("inf"))}):
            with self.assertRaises(ValueError):
                WorldOptions(**options)
        self.assertEqual(asdict(WorldOptions())["timestep"], .002)
