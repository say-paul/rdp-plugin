"""Rigid mount export validation and actual MuJoCo dynamics."""

from __future__ import annotations

import importlib.util
import json
import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from xml.etree import ElementTree

from robot_arm_library.sensor_spec import LidarSensorSpec, SensorDefinition
from robot_arm_library.world_exporter import RobotInstance, RobotMount, SceneMeshInstance, SensorMount, write_world_export


def _cube(path: Path) -> None:
    path.write_text(
        "v -.5 -.5 -.5\nv .5 -.5 -.5\nv .5 .5 -.5\nv -.5 .5 -.5\n"
        "v -.5 -.5 .5\nv .5 -.5 .5\nv .5 .5 .5\nv -.5 .5 .5\n"
        "f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\n"
        "f 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n",
        encoding="utf-8",
    )


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
class RobotMountExportTests(unittest.TestCase):
    def setUp(self):
        import mujoco

        self.mujoco = mujoco
        if not hasattr(mujoco.MjSpec, "attach"):
            self.skipTest("MuJoCo MjSpec.attach is unavailable")
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.source = self.root / "arm.xml"
        self.source.write_text(
            '<mujoco><worldbody><body name="base" pos="0 0 .15">'
            '<geom type="box" size=".08 .08 .03" contype="0" conaffinity="0"/>'
            '<body name="wrist" pos="0 0 .2"><joint name="hinge" type="hinge"/>'
            '<geom type="sphere" size=".04" contype="0" conaffinity="0"/></body>'
            '</body></worldbody></mujoco>', encoding="utf-8",
        )
        self.mesh_path = self.root / "cube.obj"
        _cube(self.mesh_path)
        self.platform = SceneMeshInstance(
            "platform", str(self.mesh_path), (1, 0, 0, 1), position=(0, 0, 2),
            dynamic=True, mass=4, collision_shape="BOX",
        )
        self.robot = RobotInstance(
            "arm", "arm", str(self.source), (0, 0, 2.35), (1, 0, 0, 0), {"hinge": .25},
            mount=RobotMount("scene", "platform", position=(0, 0, .35)),
        )

    def export(self, robots=None, meshes=None):
        world, manifest, renderer = write_world_export(
            self.root / "world.xml", [self.robot] if robots is None else robots,
            [self.platform] if meshes is None else meshes,
        )
        return self.mujoco.MjModel.from_xml_path(str(world)), json.loads(manifest.read_text()), renderer

    def body_id(self, model, name):
        index = self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_BODY, name)
        self.assertGreater(index, 0)
        return index

    def test_dynamic_platform_carries_robot_mass_and_preserves_articulation(self):
        import numpy as np

        model, manifest, _ = self.export()
        data = self.mujoco.MjData(model)
        self.mujoco.mj_forward(model, data)
        platform_id = self.body_id(model, "scene_platform")
        base_id = self.body_id(model, "arm_base")
        self.assertEqual(int(model.body_parentid[base_id]), platform_id)
        np.testing.assert_allclose(data.xpos[base_id] - data.xpos[platform_id], [0, 0, .5], atol=1e-9)
        self.assertEqual(model.nq, 8)
        self.assertGreater(model.body_subtreemass[platform_id], self.platform.mass)
        hinge = self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_JOINT, "arm_hinge")
        data.qvel[int(model.jnt_dofadr[hinge])] = .4
        initial_height = data.xpos[base_id, 2]
        for _ in range(30):
            self.mujoco.mj_step(model, data)
        self.mujoco.mj_forward(model, data)
        self.assertLess(data.xpos[base_id, 2], initial_height)
        np.testing.assert_allclose(data.xpos[base_id] - data.xpos[platform_id], [0, 0, .5], atol=1e-9)
        self.assertNotEqual(float(data.qpos[int(model.jnt_qposadr[hinge])]), 0)
        self.assertEqual(manifest["robots"][0]["mount"]["target_instance_id"], "platform")
        self.assertFalse(manifest["robots"][0]["free_base"])

    def test_static_mount_and_rotated_target_preserve_local_frame(self):
        import numpy as np

        half = math.sqrt(.5)
        platform = replace(self.platform, dynamic=False, quaternion=(half, 0, half, 0))
        model, _, _ = self.export(meshes=[platform])
        data = self.mujoco.MjData(model)
        for _ in range(10):
            self.mujoco.mj_step(model, data)
        self.mujoco.mj_forward(model, data)
        base_id = self.body_id(model, "arm_base")
        np.testing.assert_allclose(data.xpos[base_id], [.5, 0, 2], atol=1e-9)
        self.assertEqual(model.nq, 1)

    def test_robot_link_mounts_are_topologically_ordered(self):
        import numpy as np

        child = replace(
            self.robot, instance_id="child", mount=RobotMount("robot", "arm", "wrist", position=(.1, 0, .2)),
        )
        model, _, _ = self.export([child, self.robot])
        wrist_id = self.body_id(model, "arm_wrist")
        child_id = self.body_id(model, "child_base")
        self.assertEqual(int(model.body_parentid[child_id]), wrist_id)
        data = self.mujoco.MjData(model)
        hinge_id = self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_JOINT, "arm_hinge")
        data.qpos[int(model.jnt_qposadr[hinge_id])] = .7
        self.mujoco.mj_forward(model, data)
        relative = data.xmat[wrist_id].reshape(3, 3).T @ (data.xpos[child_id] - data.xpos[wrist_id])
        np.testing.assert_allclose(relative, [.1, 0, .35], atol=1e-9)
        self.assertEqual(model.nq, 9)

    def test_floating_source_home_key_and_sensors_survive_mounting(self):
        source = self.root / "floating.xml"
        source.write_text(
            '<mujoco><worldbody><body name="base"><freejoint name="root"/>'
            '<geom type="sphere" size=".1"/><body name="wrist" pos="0 0 .2">'
            '<joint name="hinge"/><geom type="sphere" size=".04"/></body></body></worldbody>'
            '<keyframe><key name="home" qpos="0 0 .3 1 0 0 0 .4" qvel="0 0 0 0 0 0 0"/></keyframe></mujoco>',
            encoding="utf-8",
        )
        original = source.read_bytes()
        lidar = SensorDefinition("lidar", "Lidar", "lidar", "registry.json", lidar=LidarSensorSpec(horizontal_samples=3))
        robot = replace(self.robot, source_path=str(source), sensors=(
            SensorMount(lidar, "front", (0, 0, .1), (1, 0, 0, 0), "wrist"),
            SensorMount(lidar, "base_lidar", (0, 0, .2), (1, 0, 0, 0)),
        ))
        model, manifest, renderer = self.export([robot])
        self.assertEqual(model.nq, 8)
        self.assertEqual(model.nsensor, 6)
        self.assertEqual(model.nkey, 1)
        self.assertEqual(source.read_bytes(), original)
        self.assertFalse(manifest["robots"][0]["free_base"])
        # Execute initialization and the sensors-only path without opening a viewer.
        import runpy
        from unittest.mock import patch

        with patch("sys.argv", [str(renderer), "--sensors-only"]):
            with self.assertRaises(SystemExit) as stopped:
                runpy.run_path(str(renderer))
        self.assertEqual(stopped.exception.code, 0)

    def test_mounted_child_keeps_rotated_floating_parent_placement(self):
        import numpy as np
        from unittest.mock import patch

        source = self.root / "mobile.xml"
        source.write_text(
            '<mujoco><worldbody><body name="base"><freejoint/>'
            '<geom type="sphere" size=".1"/></body></worldbody>'
            '<keyframe><key name="home" qpos="1 0 0 1 0 0 0"/></keyframe></mujoco>', encoding="utf-8",
        )
        half = math.sqrt(.5)
        mobile = RobotInstance("mobile", "mobile", str(source), (2, 3, 2), (half, 0, 0, half), {})
        child = replace(self.robot, mount=RobotMount("robot", "mobile", "base", position=(0, 0, .35)))
        _, _, renderer = self.export([child, mobile], meshes=[])
        namespace = {"__file__": str(renderer)}
        with patch("sys.argv", [str(renderer), "--sensors-only"]):
            with self.assertRaises(SystemExit):
                exec(compile(renderer.read_text(), str(renderer), "exec"), namespace)
        model, data = namespace["model"], namespace["data"]
        np.testing.assert_allclose(data.xpos[self.body_id(model, "mobile_base")], [2, 4, 2], atol=1e-9)
        np.testing.assert_allclose(data.xpos[self.body_id(model, "arm_base")], [2, 4, 2.5], atol=1e-9)

    def test_external_mesh_paths_remain_resolvable(self):
        assets = self.root / "source_assets"
        assets.mkdir()
        mesh = assets / "base.obj"
        _cube(mesh)
        self.source.write_text(
            '<mujoco><compiler meshdir="source_assets"/><asset><mesh name="base_mesh" file="base.obj"/></asset>'
            '<worldbody><body name="base"><geom type="mesh" mesh="base_mesh"/></body></worldbody></mujoco>',
            encoding="utf-8",
        )
        model, _, _ = self.export()
        self.assertGreaterEqual(model.nmesh, 2)

    def test_missing_target_and_cycles_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "Missing scene mount target"):
            self.export(meshes=[])
        with self.assertRaisesRegex(ValueError, "cycle"):
            self.export([replace(self.robot, mount=RobotMount("robot", "arm", "base"))])
        child = replace(self.robot, instance_id="child", mount=RobotMount("robot", "arm", "base"))
        with self.assertRaisesRegex(ValueError, "cycle"):
            self.export([replace(self.robot, mount=RobotMount("robot", "child", "base")), child])
        with self.assertRaisesRegex(ValueError, "Missing MuJoCo mount body"):
            self.export([self.robot, replace(child, mount=RobotMount("robot", "arm", "deleted"))])

    def test_ambiguous_base_and_invalid_poses_are_rejected(self):
        self.source.write_text(
            '<mujoco><worldbody><body name="one"/><body name="two"/></worldbody></mujoco>', encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "exactly one base body"):
            self.export()
        with self.assertRaisesRegex(ValueError, "finite"):
            self.export([replace(self.robot, mount=RobotMount("scene", "platform", position=(float("nan"), 0, 0)))])

    def test_ordinary_world_keeps_original_attach_export(self):
        write_world_export(self.root / "plain.xml", [replace(self.robot, mount=None)])
        tree = ElementTree.parse(self.root / "plain.xml")
        self.assertIsNotNone(tree.find("asset/model"))
        self.assertIsNotNone(tree.find("worldbody/body/attach"))


if __name__ == "__main__":
    unittest.main()
