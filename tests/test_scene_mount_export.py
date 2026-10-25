"""Generic rigid scene mounts, including compiled MuJoCo dynamics."""

from __future__ import annotations

import importlib.util
import json
import math
import tempfile
import unittest
from dataclasses import fields, replace
from pathlib import Path
from xml.etree import ElementTree

from robot_arm_library.world_exporter import (
    RobotInstance,
    RobotMount,
    SceneMeshInstance,
    build_world_xml,
    write_world_export,
)


def _cube(path: Path) -> None:
    path.write_text(
        "v -.5 -.5 -.5\nv .5 -.5 -.5\nv .5 .5 -.5\nv -.5 .5 -.5\n"
        "v -.5 -.5 .5\nv .5 -.5 .5\nv .5 .5 .5\nv -.5 .5 .5\n"
        "f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\n"
        "f 2 3 7\nf 2 7 6\nf 3 4 8\nf 3 8 7\nf 4 1 5\nf 4 5 8\n",
        encoding="utf-8",
    )


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
class SceneMountExportTests(unittest.TestCase):
    def setUp(self):
        import mujoco

        self.mujoco = mujoco
        if not hasattr(mujoco.MjSpec, "attach"):
            self.skipTest("MuJoCo MjSpec.attach is unavailable")
        self.directory = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
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
        self.mesh = SceneMeshInstance("object", str(self.mesh_path), (1, 0, 0, 1))
        self.robot = RobotInstance("arm", "arm", str(self.source), (1, 2, 3), (1, 0, 0, 0), {})

    def export(self, robots=(), meshes=()):
        world, manifest, _ = write_world_export(self.root / "world.xml", robots, meshes)
        return self.mujoco.MjModel.from_xml_path(str(world)), json.loads(manifest.read_text())

    def body_id(self, model, name):
        index = self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_BODY, name)
        self.assertGreater(index, 0)
        return index

    def test_no_mount_keeps_original_xml_and_positional_constructor(self):
        mesh = SceneMeshInstance(
            "plain", str(self.mesh_path), (1, 0, 0, 1), (0, 0, 2), (1, 0, 0, 0),
            (.2, .3, .4), True, 2.5, "BOX", True, .7, .1,
        )
        self.assertEqual(fields(SceneMeshInstance)[-1].name, "mount")
        self.assertIsNone(mesh.mount)
        tree = build_world_xml([self.robot], self.root / "world.xml", [mesh])
        self.assertIsNotNone(tree.find("asset/model"))
        self.assertIsNotNone(tree.find("worldbody/body[@name='arm']/attach"))
        body = tree.find("worldbody/body[@name='scene_plain']")
        self.assertEqual(body.get("pos"), "0 0 2")
        self.assertIsNotNone(body.find("freejoint"))
        self.assertEqual(body.find("geom[@name='scene_plain']").get("mass"), "2.5")
        model, manifest = self.export([self.robot], [mesh])
        self.assertEqual(model.nq, 8)
        self.assertIsNone(manifest["scene_meshes"][0]["mount"])

    def test_object_to_object_normalizes_root_pose_and_preserves_asset_scale(self):
        import numpy as np

        half = math.sqrt(.5)
        parent = replace(self.mesh, instance_id="parent", position=(2, 3, 4),
                         quaternion=(half, 0, 0, half), scale=(2, 3, 4))
        mount = RobotMount("scene", "parent", position=(.1, .2, .3), quaternion=(half, half, 0, 0))
        child = replace(self.mesh, instance_id="child", position=(99, 98, 97),
                        quaternion=(0, 0, 1, 0), scale=(.2, .4, .6), dynamic=True, mass=2.5, mount=mount)
        model, manifest = self.export(meshes=[child, parent])
        data = self.mujoco.MjData(model)
        self.mujoco.mj_forward(model, data)
        parent_id = self.body_id(model, "scene_parent")
        child_id = self.body_id(model, "scene_child")
        self.assertEqual(int(model.body_parentid[child_id]), parent_id)
        self.assertEqual(model.nq, 0)
        np.testing.assert_allclose(model.body_pos[child_id], mount.position, atol=1e-9)
        np.testing.assert_allclose(model.body_quat[child_id], mount.quaternion, atol=1e-9)
        np.testing.assert_allclose(data.xpos[child_id], [1.8, 3.1, 4.3], atol=1e-9)
        np.testing.assert_allclose(data.xquat[child_id], [.5, .5, .5, .5], atol=1e-9)
        self.assertAlmostEqual(float(model.body_mass[child_id]), 2.5)
        exported = ElementTree.parse(self.root / "world.xml")
        mesh = exported.find("asset/mesh[@name='scene_child']")
        np.testing.assert_allclose([float(value) for value in mesh.get("scale").split()], child.scale)
        np.testing.assert_allclose([float(value) for value in exported.find(
            "asset/mesh[@name='scene_parent']").get("scale").split()], parent.scale)
        self.assertTrue(Path(mesh.get("file")).is_absolute())
        self.assertEqual(len(exported.findall("worldbody/geom[@name='ground']")), 1)
        self.assertEqual(len(exported.findall("worldbody/light")), 1)
        self.assertEqual(len(exported.findall("asset/material[@name='floor_checker']")), 1)
        self.assertEqual(manifest["scene_meshes"][0]["mount"]["target_instance_id"], "parent")
        self.assertEqual(manifest["scene_meshes"][0]["mount"]["quaternion"], list(mount.quaternion))

    def test_object_to_robot_link_follows_articulation(self):
        import numpy as np

        mesh = replace(self.mesh, mount=RobotMount("robot", "arm", "wrist", position=(.2, 0, .1)))
        model, _ = self.export([self.robot], [mesh])
        data = self.mujoco.MjData(model)
        wrist_id = self.body_id(model, "arm_wrist")
        mesh_id = self.body_id(model, "scene_object")
        self.assertEqual(int(model.body_parentid[mesh_id]), wrist_id)
        self.assertEqual(model.nq, 1)
        for angle in (0, .7):
            data.qpos[0] = angle
            self.mujoco.mj_forward(model, data)
            relative = data.xmat[wrist_id].reshape(3, 3).T @ (data.xpos[mesh_id] - data.xpos[wrist_id])
            np.testing.assert_allclose(relative, [.2, 0, .1], atol=1e-9)
        self.assertGreater(float(model.body_mass[mesh_id]), 0)

    def test_robot_scene_robot_chain_keeps_kind_specific_ids_and_keys(self):
        self.source.write_text(self.source.read_text().replace(
            "</mujoco>", '<keyframe><key name="home" qpos=".4" qvel=".1"/></keyframe></mujoco>',
        ), encoding="utf-8")
        original = self.source.read_bytes()
        parent = replace(self.robot, instance_id="carrier")
        mesh = replace(self.mesh, instance_id="shared", mount=RobotMount("robot", "carrier", "wrist"))
        child = replace(self.robot, instance_id="shared", mount=RobotMount("scene", "shared", position=(0, 0, .2)))
        model, _ = self.export([child, parent], [mesh])
        mesh_id = self.body_id(model, "scene_shared")
        self.assertEqual(int(model.body_parentid[mesh_id]), self.body_id(model, "carrier_wrist"))
        self.assertEqual(int(model.body_parentid[self.body_id(model, "shared_base")]), mesh_id)
        self.assertEqual(model.nq, 2)
        self.assertEqual(model.nkey, 2)
        for prefix in ("shared_", "carrier_"):
            key = self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_KEY, prefix + "home")
            joint = self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_JOINT, prefix + "hinge")
            self.assertGreaterEqual(key, 0)
            self.assertGreaterEqual(joint, 0)
            self.assertAlmostEqual(float(model.key_qpos[key, int(model.jnt_qposadr[joint])]), .4)
            self.assertAlmostEqual(float(model.key_qvel[key, int(model.jnt_dofadr[joint])]), .1)
        self.assertEqual(self.source.read_bytes(), original)

    def test_moving_platform_carries_passive_and_dynamic_scene_mass(self):
        import numpy as np

        parent = replace(self.mesh, instance_id="platform", position=(0, 0, 3),
                         dynamic=True, mass=4, collision_shape="BOX")
        passive = replace(self.mesh, instance_id="passive", scale=(.1, .1, .1), collision_enabled=False,
                          mount=RobotMount("scene", "platform", position=(.2, 0, .8)))
        active = replace(self.mesh, instance_id="active", scale=(.1, .1, .1), dynamic=True, mass=2,
                         mount=RobotMount("scene", "passive", position=(0, .3, .4)))
        model, _ = self.export(meshes=[active, passive, parent])
        data = self.mujoco.MjData(model)
        parent_id = self.body_id(model, "scene_platform")
        passive_id = self.body_id(model, "scene_passive")
        active_id = self.body_id(model, "scene_active")
        self.assertEqual((model.nq, model.nv), (7, 6))
        self.assertEqual(int(model.body_parentid[active_id]), passive_id)
        self.assertEqual(int(model.body_jntnum[passive_id]), 0)
        self.assertEqual(int(model.body_jntnum[active_id]), 0)
        self.assertGreater(float(model.body_mass[passive_id]), 0)
        self.assertAlmostEqual(float(model.body_mass[active_id]), 2)
        self.assertAlmostEqual(float(model.body_subtreemass[parent_id]), 6 + float(model.body_mass[passive_id]))
        data.qvel[:] = [.6, .2, 0, .1, .2, .3]
        self.mujoco.mj_forward(model, data)
        initial_height = float(data.xpos[parent_id, 2])
        for _ in range(30):
            self.mujoco.mj_step(model, data)
        self.mujoco.mj_forward(model, data)
        self.assertLess(float(data.xpos[parent_id, 2]), initial_height)
        for child_id, expected in ((passive_id, [.2, 0, .8]), (active_id, [.2, .3, 1.2])):
            relative = data.xmat[parent_id].reshape(3, 3).T @ (data.xpos[child_id] - data.xpos[parent_id])
            np.testing.assert_allclose(relative, expected, atol=1e-9)

    def test_passive_mounted_mesh_uses_explicit_mass_not_density(self):
        parent = replace(self.mesh, instance_id="platform", position=(0, 0, 3),
                         dynamic=True, mass=4, collision_shape="BOX")
        for shape, collision_enabled in (("MESH", True), ("BOX", True), ("MESH", False)):
            with self.subTest(shape=shape, collision_enabled=collision_enabled):
                passive = replace(
                    self.mesh, instance_id="passive", dynamic=False, mass=2.75, scale=(.2, .3, .4),
                    collision_shape=shape, collision_enabled=collision_enabled,
                    mount=RobotMount("scene", "platform", position=(0, 0, 1)),
                )
                model, manifest = self.export(meshes=[passive, parent])
                child_id = self.body_id(model, "scene_passive")
                parent_id = self.body_id(model, "scene_platform")
                self.assertEqual((model.nq, model.nv), (7, 6))
                self.assertEqual(int(model.body_parentid[child_id]), parent_id)
                self.assertEqual(int(model.body_jntnum[child_id]), 0)
                self.assertAlmostEqual(float(model.body_mass[child_id]), passive.mass)
                self.assertAlmostEqual(float(model.body_subtreemass[parent_id]), parent.mass + passive.mass)
                self.assertFalse(manifest["scene_meshes"][0]["dynamic"])

    def test_missing_targets_and_invalid_scene_mounts_are_rejected(self):
        cases = (
            (RobotMount("scene", "missing"), "Missing scene mount target"),
            (RobotMount("robot", "missing", "base"), "Missing robot mount target"),
            (RobotMount("robot", "arm", "missing"), "Missing MuJoCo mount body"),
            (RobotMount("robot", "arm"), "target MJCF body name"),
            (RobotMount("other", "arm"), "Unsupported mount target kind"),
            (RobotMount("robot", "arm", "base", position=(float("nan"), 0, 0)), "finite"),
            (RobotMount("robot", "arm", "base", quaternion=(0, 0, 0, 0)), "nonzero quaternion"),
            (RobotMount("robot", "arm", "base", position=(0, 0)), "position"),
        )
        for mount, message in cases:
            with self.subTest(mount=mount), self.assertRaisesRegex(ValueError, message):
                self.export([self.robot], [replace(self.mesh, mount=mount)])

    def test_scene_and_cross_kind_cycles_are_rejected(self):
        a = replace(self.mesh, instance_id="a", mount=RobotMount("scene", "b"))
        b = replace(self.mesh, instance_id="b", mount=RobotMount("scene", "a"))
        robot = replace(self.robot, mount=RobotMount("scene", "a"))
        cases = (
            ([], [replace(a, mount=RobotMount("scene", "a"))]),
            ([], [a, b]),
            ([robot], [replace(a, mount=RobotMount("robot", "arm", "base"))]),
            ([robot], [a, replace(b, mount=RobotMount("robot", "arm", "wrist"))]),
        )
        for robots, meshes in cases:
            with self.subTest(robots=robots, meshes=meshes), self.assertRaisesRegex(ValueError, "cycle"):
                self.export(robots, meshes)

    def test_duplicate_ids_and_sanitized_names_are_rejected(self):
        mounted = replace(self.mesh, mount=RobotMount("robot", "arm", "base"))
        cases = (
            ([self.robot], [mounted, mounted], "Duplicate scene mesh id"),
            ([self.robot], [replace(mounted, instance_id="one-two"), replace(mounted, instance_id="one two")],
             "duplicate MuJoCo"),
            ([self.robot, self.robot], [mounted], "Duplicate robot instance id"),
            ([replace(self.robot, instance_id="one-two"), replace(self.robot, instance_id="one two")],
             [replace(mounted, mount=RobotMount("robot", "one-two", "base"))], "duplicate MuJoCo"),
        )
        for robots, meshes, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                self.export(robots, meshes)


if __name__ == "__main__":
    unittest.main()
