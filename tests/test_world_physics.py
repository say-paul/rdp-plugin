"""Compile an exported Blender-style scene and check its actual MuJoCo motion."""

from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path

from robot_arm_library.world_exporter import SceneMeshInstance, write_world_export


def _cube_obj(path: Path) -> None:
    path.write_text(
        """v -0.5 -0.5 -0.5
v 0.5 -0.5 -0.5
v 0.5 0.5 -0.5
v -0.5 0.5 -0.5
v -0.5 -0.5 0.5
v 0.5 -0.5 0.5
v 0.5 0.5 0.5
v -0.5 0.5 0.5
f 1 4 3
f 1 3 2
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 2 3 7
f 2 7 6
f 3 4 8
f 3 8 7
f 4 1 5
f 4 5 8
""",
        encoding="utf-8",
    )


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
class WorldPhysicsTests(unittest.TestCase):
    def test_active_box_has_mass_and_lands_on_ground(self):
        import mujoco

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cube_path = root / "cube.obj"
            _cube_obj(cube_path)
            world_path, _manifest_path, _render_path = write_world_export(
                root / "world.xml",
                [],
                [
                    SceneMeshInstance(
                        "Falling Cube",
                        str(cube_path),
                        (0.8, 0.2, 0.1, 1),
                        position=(0, 0, 2),
                        dynamic=True,
                        mass=2.5,
                        collision_shape="BOX",
                        friction=0.7,
                    )
                ],
            )
            model = mujoco.MjModel.from_xml_path(str(world_path))
            data = mujoco.MjData(model)
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "scene_Falling_Cube")
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "scene_Falling_Cube")
            visual_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "scene_Falling_Cube_visual")
            self.assertEqual(int(model.geom_type[geom_id]), int(mujoco.mjtGeom.mjGEOM_BOX))
            self.assertEqual(float(model.body_mass[body_id]), 2.5)
            self.assertAlmostEqual(float(model.geom_friction[geom_id, 0]), 0.7)
            self.assertEqual(int(model.geom_contype[visual_id]), 0)
            for _ in range(1000):
                mujoco.mj_step(model, data)
            self.assertAlmostEqual(float(data.xpos[body_id, 2]), 0.5, delta=0.08)
            self.assertGreater(data.ncon, 0)


if __name__ == "__main__":
    unittest.main()
