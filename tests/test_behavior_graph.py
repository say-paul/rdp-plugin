import tempfile
import unittest
from pathlib import Path

from robot_arm_library.behavior_graph import BehaviorGraph, BehaviorLink, BehaviorNode, default_behavior_graph
from robot_arm_library.world_exporter import RobotInstance, write_world_export


class BehaviorGraphTests(unittest.TestCase):
    def test_default_graph_runs_sensor_ai_and_action(self):
        graph = default_behavior_graph()
        actions = []

        values = graph.run(
            lambda data: 0.25,
            lambda data, context: {"target": context["input"] + 0.5},
            lambda data, value: actions.append(value),
        )

        self.assertEqual(values["sensor"], 0.25)
        self.assertEqual(actions, [{"target": 0.75}])

    def test_cycles_are_rejected(self):
        graph = BehaviorGraph(
            nodes=[BehaviorNode("a", "sensor"), BehaviorNode("b", "action")],
            links=[BehaviorLink("a", "b"), BehaviorLink("b", "a")],
        )

        self.assertIn("Behavior graph contains a cycle", graph.validate())

    def test_world_export_writes_render_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "arm.xml"
            source.write_text("<mujoco><worldbody><body name='arm'/></worldbody></mujoco>", encoding="utf-8")
            files = write_world_export(
                root / "world.xml",
                [RobotInstance("arm_1", "arm", str(source), (0, 0, 0), (1, 0, 0, 0), {"joint": 0.5})],
            )

            self.assertTrue(all(path.exists() for path in files))
            self.assertIn("launch_passive", files[2].read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()