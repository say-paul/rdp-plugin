"""External export fallback when Blender's Python cannot import MuJoCo."""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from robot_arm_library import world_exporter as exporter
from robot_arm_library.sensor_spec import CameraSensorSpec, LidarSensorSpec, SensorDefinition
from robot_arm_library.world_export_worker import export_request


HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None


class ExternalExportTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix="caryam external export ")
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.source = self.root / "arm.xml"
        self.source.write_text(
            '<mujoco><worldbody><body name="base"><geom type="box" size=".1 .1 .02"/>'
            '<body name="wrist" pos="0 0 .2"><joint name="hinge"/><geom type="sphere" size=".03"/>'
            '</body></body></worldbody></mujoco>', encoding="utf-8",
        )
        mesh = self.root / "table.obj"
        mesh.write_text(
            'v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n', encoding="utf-8",
        )
        self.platform = exporter.SceneMeshInstance(
            "table", str(mesh), (1, 0, 0, 1), position=(0, 0, 2),
            dynamic=True, mass=3, collision_shape="BOX",
        )
        self.robot = exporter.RobotInstance(
            "arm", "arm", str(self.source), (0, 0, 2.5), (1, 0, 0, 0), {"hinge": .3},
            mount=exporter.RobotMount("scene", "table", position=(0, 0, .5)),
        )
        self.output = self.root / "output" / "world.xml"

    def export(self, robot=None, executable=None):
        return exporter.write_world_export(
            self.output, [self.robot if robot is None else robot], [self.platform],
            python_executable=executable or sys.executable,
        )

    @unittest.skipUnless(HAS_MUJOCO, "MuJoCo is not installed in the external test interpreter")
    def test_missing_local_mujoco_exports_using_selected_python(self):
        import mujoco

        robot = replace(self.robot, source_path=os.path.relpath(self.source))
        with patch.dict(sys.modules, {"mujoco": None}):
            world, manifest, renderer = self.export(robot)
        model = mujoco.MjModel.from_xml_path(str(world))
        self.assertEqual(model.nq, 8)
        value = json.loads(manifest.read_text())
        self.assertEqual(value["robots"][0]["mount"]["target_instance_id"], "table")
        self.assertEqual(value["robots"][0]["qpos"]["hinge"], .3)
        self.assertTrue(renderer.is_file())

    @unittest.skipUnless(HAS_MUJOCO, "MuJoCo is not installed in the external test interpreter")
    def test_incompatible_local_mujoco_uses_external_backend(self):
        import mujoco

        with patch.dict(sys.modules, {"mujoco": SimpleNamespace()}):
            world, _, _ = self.export()
        self.assertEqual(mujoco.MjModel.from_xml_path(str(world)).nq, 8)

    @unittest.skipUnless(HAS_MUJOCO, "MuJoCo is not installed in the external test interpreter")
    def test_external_worker_preserves_included_sensors_and_floating_home(self):
        import mujoco

        fragment = self.root / "bodies.xml"
        fragment.write_text(
            '<mujoco><worldbody><body name="base"><freejoint name="free"/>'
            '<geom type="box" size=".1 .1 .02"/><body name="wrist" pos="0 0 .2">'
            '<joint name="hinge"/><geom type="sphere" size=".03"/></body></body></worldbody></mujoco>',
            encoding="utf-8",
        )
        self.source.write_text(
            '<mujoco><include file="bodies.xml"/><keyframe>'
            '<key name="home" qpos="0 0 .2 1 0 0 0 .1"/></keyframe></mujoco>', encoding="utf-8",
        )
        original = self.source.read_bytes()
        lidar = SensorDefinition("lidar", "Lidar", "lidar", "registry.json", lidar=LidarSensorSpec(horizontal_samples=4))
        depth = SensorDefinition("depth", "Depth", "depth_camera", "registry.json", camera=CameraSensorSpec())
        robot = replace(self.robot, sensors=(
            exporter.SensorMount(lidar, "lidar", (0, 0, .1), (1, 0, 0, 0), "wrist"),
            exporter.SensorMount(depth, "depth", (0, 0, .2), (1, 0, 0, 0)),
        ))
        with patch.dict(sys.modules, {"mujoco": None}):
            world, manifest, _ = self.export(robot)
        model = mujoco.MjModel.from_xml_path(str(world))
        self.assertEqual((model.nq, model.nsensor, model.ncam), (8, 4, 1))
        self.assertFalse(json.loads(manifest.read_text())["robots"][0]["free_base"])
        self.assertEqual(self.source.read_bytes(), original)

    @unittest.skipUnless(HAS_MUJOCO, "MuJoCo is not installed in the external test interpreter")
    def test_external_worker_preserves_mesh_mounts(self):
        import mujoco

        child = replace(
            self.platform, instance_id="part", mass=.8, scale=(.5, .6, .7),
            mount=exporter.RobotMount("scene", "table", position=(0, 0, .5)),
        )
        with patch.dict(sys.modules, {"mujoco": None}):
            world, manifest, _ = exporter.write_world_export(
                self.output, [], [child, self.platform], python_executable=sys.executable,
            )
        model = mujoco.MjModel.from_xml_path(str(world))
        part_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "scene_part")
        table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "scene_table")
        self.assertEqual(int(model.body_parentid[part_id]), table_id)
        self.assertAlmostEqual(float(model.body_mass[part_id]), .8)
        self.assertEqual(model.nq, 7)
        value = json.loads(manifest.read_text())
        self.assertTrue(any(mesh["mount"] is not None for mesh in value["scene_meshes"]))

    def test_user_site_mujoco_is_loaded_before_needing_external_python(self):
        backend = SimpleNamespace()
        with patch.object(exporter.importlib, "import_module", side_effect=[ModuleNotFoundError("mujoco"), backend]) as load:
            with patch.object(exporter.site, "getusersitepackages", return_value="/test/user-site"):
                with patch.object(sys, "path", list(sys.path)):
                    self.assertIs(exporter._load_export_mujoco(), backend)
                    self.assertEqual(sys.path[0], "/test/user-site")
        self.assertEqual(load.call_count, 2)

    def test_plain_world_does_not_require_external_python_or_mujoco(self):
        with patch.dict(sys.modules, {"mujoco": None}), patch.object(exporter.subprocess, "run") as run:
            paths = exporter.write_world_export(
                self.output, [replace(self.robot, mount=None)], python_executable="missing-python",
            )
        self.assertTrue(all(path.is_file() for path in paths))
        run.assert_not_called()

    def test_failed_worker_reports_selected_interpreter_and_cleans_request(self):
        requests = []

        def run(command, **kwargs):
            request = Path(command[-1])
            self.assertTrue(request.is_file())
            requests.append(request)
            return subprocess.CompletedProcess(command, 1, "", "No module named 'mujoco'")

        with patch.object(exporter, "_load_export_mujoco", side_effect=exporter.MuJoCoBackendUnavailable("missing")):
            with patch.object(exporter.subprocess, "run", side_effect=run):
                with self.assertRaisesRegex(RuntimeError, "selected.*Python.*custom-python") as raised:
                    self.export(executable="custom-python")
        self.assertIn("No module named 'mujoco'", str(raised.exception))
        self.assertTrue(all(not request.exists() for request in requests))

    def test_worker_timeout_is_reported_and_request_is_removed(self):
        requests = []

        def run(command, **kwargs):
            requests.append(Path(command[-1]))
            raise subprocess.TimeoutExpired(command, kwargs["timeout"])

        with patch.object(exporter, "_load_export_mujoco", side_effect=exporter.MuJoCoBackendUnavailable("missing")):
            with patch.object(exporter.subprocess, "run", side_effect=run):
                with self.assertRaisesRegex(RuntimeError, "timed out.*120 seconds"):
                    self.export()
        self.assertTrue(all(not request.exists() for request in requests))

    def test_success_without_exported_files_is_not_reported_as_success(self):
        with patch.object(exporter, "_load_export_mujoco", side_effect=exporter.MuJoCoBackendUnavailable("missing")):
            with patch.object(exporter.subprocess, "run", return_value=SimpleNamespace(returncode=0)):
                with self.assertRaisesRegex(RuntimeError, "did not produce all expected files"):
                    self.export()

    @unittest.skipUnless(HAS_MUJOCO, "MuJoCo is not installed")
    def test_invalid_model_does_not_trigger_backend_fallback(self):
        robot = replace(self.robot, mount=exporter.RobotMount("scene", "deleted"))
        with patch.object(exporter.subprocess, "run") as run:
            with self.assertRaisesRegex(ValueError, "Missing scene mount target"):
                self.export(robot)
        run.assert_not_called()

    def test_worker_rejects_unknown_request_versions(self):
        with self.assertRaisesRegex(ValueError, "Unsupported MuJoCo export request version"):
            export_request({"version": 999})


if __name__ == "__main__":
    unittest.main()
