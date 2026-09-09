import json
import tempfile
import unittest
from pathlib import Path

from robot_arm_library.setup_parser import discover_robot_definitions


class SetupParserTests(unittest.TestCase):
    def test_discovers_multiple_json_robot_arms(self):
        setup_path = Path(__file__).parents[1] / "examples" / "robot_setup.json"

        robots = discover_robot_definitions(setup_path)

        self.assertEqual([robot.robot_id for robot in robots], ["ur5e", "franka_panda", "xarm7"])
        self.assertEqual(robots[1].model_path, "assets/franka_panda.xml")

    def test_discovers_named_python_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            setup_path = Path(directory) / "robot_setup.py"
            setup_path.write_text(
                "ROBOTS = {'my_arm': {'model_path': 'meshes/my_arm.obj', 'description': 'test'}}\n",
                encoding="utf-8",
            )

            robots = discover_robot_definitions(setup_path)

        self.assertEqual(len(robots), 1)
        self.assertEqual(robots[0].name, "my_arm")
        self.assertEqual(robots[0].model_path, "meshes/my_arm.obj")

    def test_discovers_simple_name_to_model_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            setup_path = Path(directory) / "robot_setup.json"
            setup_path.write_text(json.dumps({"ur5e": "assets/ur5e.xml"}), encoding="utf-8")

            robots = discover_robot_definitions(setup_path)

        self.assertEqual(len(robots), 1)
        self.assertEqual(robots[0].name, "ur5e")
        self.assertEqual(robots[0].model_path, "assets/ur5e.xml")

    def test_scans_a_setup_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "a.json").write_text(json.dumps({"name": "A", "model_path": "a.obj"}), encoding="utf-8")
            (root / "nested").mkdir()
            (root / "nested" / "b.json").write_text(json.dumps({"name": "B", "model_path": "b.obj"}), encoding="utf-8")

            robots = discover_robot_definitions(root)

        self.assertEqual({robot.name for robot in robots}, {"A", "B"})

    def test_discovers_curated_menagerie_registry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "mujoco_menagerie"
            registry_path = root / "python" / "src" / "mujoco_menagerie" / "registry.json"
            registry_path.parent.mkdir(parents=True)
            registry_path.write_text(
                json.dumps(
                    {
                        "schema": 1,
                        "robots": {
                            "franka_emika_panda": {
                                "display_name": "Franka Emika Panda",
                                "category": "arm",
                                "default_model": "panda",
                                "entry_points": {"panda": {"kind": "robot", "file": "panda.xml"}, "scene": {"kind": "scene", "file": "scene.xml"}},
                            },
                            "unitree_go2": {
                                "display_name": "Unitree Go2",
                                "category": "quadruped",
                                "default_model": "go2",
                                "entry_points": {"go2": {"kind": "robot", "file": "go2.xml"}},
                            },
                        },
                    }
                ),
                encoding="utf-8",
            )

            robots = discover_robot_definitions(root)

        self.assertEqual([robot.robot_id for robot in robots], ["franka_emika_panda", "unitree_go2"])
        self.assertEqual(robots[0].category, "arm")
        self.assertTrue(robots[0].model_path.endswith("franka_emika_panda/panda.xml"))

    def test_scans_menagerie_model_directories_without_registry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "mujoco_menagerie"
            root.mkdir()
            (root / "catalog.py").write_text("", encoding="utf-8")
            model_dir = root / "universal_robots_ur5e"
            model_dir.mkdir()
            (model_dir / "README.md").write_text("# Universal Robots UR5e Description (MJCF)\n", encoding="utf-8")
            (model_dir / "ur5e.xml").write_text("<mujoco />", encoding="utf-8")
            (model_dir / "scene.xml").write_text("<mujoco />", encoding="utf-8")
            (model_dir / "scene_mjx.xml").write_text("<mujoco />", encoding="utf-8")

            robots = discover_robot_definitions(root)

        self.assertEqual(len(robots), 1)
        self.assertEqual(robots[0].name, "Universal Robots UR5e")

    def test_selected_menagerie_model_still_discovers_full_repository(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "mujoco_menagerie"
            root.mkdir()
            (root / "catalog.py").write_text("", encoding="utf-8")
            for directory_name, model_name in (("agilex_piper", "piper"), ("unitree_z1", "z1")):
                model_dir = root / directory_name
                model_dir.mkdir()
                (model_dir / f"{model_name}.xml").write_text("<mujoco />", encoding="utf-8")
                (model_dir / "scene.xml").write_text("<mujoco />", encoding="utf-8")

            robots = discover_robot_definitions(root / "agilex_piper" / "piper.xml")

        self.assertEqual({robot.robot_id for robot in robots}, {"agilex_piper/piper", "unitree_z1/z1"})


if __name__ == "__main__":
    unittest.main()