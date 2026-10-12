import importlib.util
import tempfile
import unittest
from pathlib import Path

from robot_arm_library.behavior_graph import (
    BehaviorGraph,
    BehaviorLink,
    BehaviorNode,
    PortSpec,
    default_behavior_graph,
)
from robot_arm_library.world_exporter import RobotInstance, SceneMeshInstance, write_world_export


class BehaviorGraphTests(unittest.TestCase):
    def test_application_node_round_trips_ports_and_handler(self):
        graph = BehaviorGraph(
            nodes=[
                BehaviorNode(
                    "camera",
                    "application",
                    "Camera",
                    handler="python scripts/applications/camera.py",
                    outputs=(PortSpec("oak1", (1, 3, 480, 640)),),
                ),
            ],
            links=[],
        )

        restored = BehaviorGraph.from_dict(graph.to_dict())

        camera = restored.node_map()["camera"]
        self.assertEqual(camera.handler, "python scripts/applications/camera.py")
        self.assertEqual(camera.outputs, (PortSpec("oak1", (1, 3, 480, 640)),))
        self.assertEqual(camera.output_names(), ("oak1",))

    def test_application_node_without_ports_is_invalid(self):
        graph = BehaviorGraph(nodes=[BehaviorNode("empty_app", "application")], links=[])

        self.assertIn("Application node has no ports: empty_app", graph.validate())

    def test_virtual_lidar_is_a_typed_point_cloud_component(self):
        graph = BehaviorGraph(
            nodes=[
                BehaviorNode(
                    "lidar",
                    "virtual_lidar",
                    outputs=(PortSpec("point_cloud", data_type="point_cloud2"),),
                ),
            ],
            links=[],
        )

        self.assertEqual(graph.validate(), [])
        self.assertEqual(graph.to_dict()["nodes"][0]["kind"], "virtual_lidar")

    def test_run_executes_application_node_with_named_ports(self):
        graph = BehaviorGraph(
            nodes=[
                BehaviorNode("camera", "application", outputs=(PortSpec("oak1"),)),
                BehaviorNode(
                    "image_transform",
                    "application",
                    inputs=(PortSpec("oak1"),),
                    outputs=(PortSpec("smolvla"),),
                ),
            ],
            links=[BehaviorLink("camera", "image_transform", "oak1", "oak1")],
        )

        def application_runner(node, inputs):
            if node.node_id == "camera":
                return {"oak1": "frame"}
            return {"smolvla": inputs["oak1"] + "-transformed"}

        values = graph.run(lambda data: None, application_runner=application_runner)

        self.assertEqual(values["image_transform"], {"smolvla": "frame-transformed"})

    def test_run_without_application_runner_raises_clear_error(self):
        graph = BehaviorGraph(
            nodes=[BehaviorNode("app", "application", outputs=(PortSpec("x"),))],
            links=[],
        )

        with self.assertRaisesRegex(ValueError, "No application runner configured for node: app"):
            graph.run(lambda data: None)

    def test_validate_rejects_unknown_port_on_link(self):
        graph_dict = {
            "nodes": [
                BehaviorNode("app", "application", outputs=(PortSpec("known"),)).to_dict(),
                BehaviorNode("sink", "application", inputs=(PortSpec("known"),)).to_dict(),
            ],
            "links": [BehaviorLink("app", "sink", "missing", "known").to_dict()],
        }
        graph = BehaviorGraph(
            nodes=[BehaviorNode.from_dict(node) for node in graph_dict["nodes"]],
            links=[
                BehaviorLink(link["source"], link["target"], link["source_port"], link["target_port"])
                for link in graph_dict["links"]
            ],
        )

        self.assertIn("Unknown output port 'missing' on app", graph.validate())

    def test_validate_rejects_incompatible_application_port_dimensions(self):
        graph = BehaviorGraph(
            nodes=[
                BehaviorNode("camera", "application", outputs=(PortSpec("image", (1, 3, 480, 640)),)),
                BehaviorNode("model", "application", inputs=(PortSpec("image", (1, 3, 224, 224)),)),
            ],
            links=[BehaviorLink("camera", "model", "image", "image")],
        )

        self.assertIn(
            "Incompatible port shapes: camera.image (1, 3, 480, 640) -> model.image (1, 3, 224, 224)",
            graph.validate(),
        )

    def test_to_flow_dict_emits_application_entries_and_dataflow_chain(self):
        graph = BehaviorGraph(
            nodes=[
                BehaviorNode(
                    "camera",
                    "application",
                    handler="python scripts/camera.py",
                    outputs=(PortSpec("oak1", (1, 3, 480, 640)),),
                ),
                BehaviorNode(
                    "image_transform",
                    "application",
                    handler="python scripts/transform.py",
                    inputs=(PortSpec("oak1", (1, 3, 480, 640)),),
                    outputs=(PortSpec("smolvla", (2, 3, 512, 512)),),
                ),
            ],
            links=[BehaviorLink("camera", "image_transform", "oak1", "oak1")],
        )

        flow = graph.to_flow_dict()

        camera_entry = next(entry for entry in flow["application"] if entry["name"] == "camera")
        self.assertEqual(camera_entry["outputs"], {"oak1": [1, 3, 480, 640]})
        self.assertIn("oak1 → image_transform → smolvla", flow["dataflow"])

    def test_from_flow_dict_infers_links_from_matching_port_names(self):
        flow = {
            "application": [
                {"name": "camera", "type": "python", "outputs": {"oak1": [1, 3, 480, 640]}},
                {
                    "name": "image_transform",
                    "type": "python",
                    "inputs": {"oak1": [1, 3, 480, 640]},
                    "outputs": {"smolvla": [2, 3, 512, 512]},
                },
            ],
        }

        graph = BehaviorGraph.from_flow_dict(flow)

        self.assertEqual(len(graph.links), 1)
        link = graph.links[0]
        self.assertEqual(
            (link.source, link.source_port, link.target, link.target_port),
            ("camera", "oak1", "image_transform", "oak1"),
        )

    @unittest.skipUnless(importlib.util.find_spec("yaml"), "PyYAML not installed")
    def test_save_and_load_flow_yaml_round_trip(self):
        graph = BehaviorGraph.from_flow_dict(
            {
                "application": [
                    {"name": "camera", "type": "python", "outputs": {"oak1": [1, 3, 480, 640]}},
                    {
                        "name": "image_transform",
                        "type": "python",
                        "inputs": {"oak1": [1, 3, 480, 640]},
                        "outputs": {"smolvla": []},
                    },
                ],
            }
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flow.yaml"
            graph.save_flow_yaml(path)

            restored = BehaviorGraph.load_flow_yaml(path)

        self.assertEqual({node.node_id for node in restored.nodes}, {"camera", "image_transform"})
        self.assertEqual(len(restored.links), 1)

    def test_default_graph_is_the_core_sensor_to_command_pipeline(self):
        graph = default_behavior_graph()

        self.assertEqual(
            [node.kind for node in graph.nodes],
            ["sensor", "application", "joint_controller", "joint_target"],
        )
        self.assertEqual(graph.validate(), [])

    def test_cycles_are_rejected(self):
        graph = BehaviorGraph(
            nodes=[BehaviorNode("a", "sensor"), BehaviorNode("b", "action")],
            links=[BehaviorLink("a", "b"), BehaviorLink("b", "a")],
        )

        self.assertIn("Data graph contains a cycle", graph.validate())

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
            self.assertIn('prefix + "home"', files[2].read_text(encoding="utf-8"))
            self.assertIn("time.perf_counter", files[2].read_text(encoding="utf-8"))

    def test_world_export_allows_an_empty_physics_scene(self):
        with tempfile.TemporaryDirectory() as directory:
            world_path, manifest_path, render_path = write_world_export(Path(directory) / "world.xml", [])

            self.assertTrue(world_path.exists())
            self.assertIn('gravity="0 0 -9.81"', world_path.read_text(encoding="utf-8"))
            self.assertIn('texture name="floor_checker"', world_path.read_text(encoding="utf-8"))
            self.assertIn('material="floor_checker"', world_path.read_text(encoding="utf-8"))
            self.assertIn('contype="1" conaffinity="1"', world_path.read_text(encoding="utf-8"))
            self.assertIn('"robots": []', manifest_path.read_text(encoding="utf-8"))
            self.assertIn("launch_passive", render_path.read_text(encoding="utf-8"))

    def test_world_export_includes_static_scene_meshes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mesh_path = root / "staircase.obj"
            mesh_path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")

            world_path, manifest_path, _render_path = write_world_export(
                root / "world.xml",
                [],
                [
                    SceneMeshInstance(
                        "Staircase",
                        str(mesh_path),
                        (0.4, 0.3, 0.2, 1.0),
                        position=(0.0, 2.0, 0.0),
                    )
                ],
            )

            world_xml = world_path.read_text(encoding="utf-8")
            self.assertIn('mesh name="scene_Staircase"', world_xml)
            self.assertIn('file="staircase.obj"', world_xml)
            self.assertIn('<body name="scene_Staircase" pos="0 2 0"', world_xml)
            self.assertIn('geom name="scene_Staircase"', world_xml)
            self.assertIn('"scene_meshes":', manifest_path.read_text(encoding="utf-8"))

    def test_world_export_includes_dynamic_scene_meshes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mesh_path = root / "ball.obj"
            mesh_path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 4 2\nf 1 3 4\nf 2 4 3\n", encoding="utf-8")

            world_path, _manifest_path, _render_path = write_world_export(
                root / "world.xml",
                [],
                [
                    SceneMeshInstance(
                        "Plastic Ball",
                        str(mesh_path),
                        (0.4, 0.3, 0.2, 1.0),
                        position=(0.0, 2.5, 1.8),
                        dynamic=True,
                        mass=0.5,
                    )
                ],
            )

            world_xml = world_path.read_text(encoding="utf-8")
            self.assertIn('<body name="scene_Plastic_Ball" pos="0 2.5 1.8"', world_xml)
            self.assertIn('<freejoint name="scene_Plastic_Ball_free"', world_xml)
            self.assertIn('mass="0.5"', world_xml)

    def test_world_export_attaches_free_joint_model_to_worldbody(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "free_robot.xml"
            source.write_text(
                "<mujoco><worldbody><body name='base'><freejoint/><geom type='sphere' size='0.1'/></body></worldbody></mujoco>",
                encoding="utf-8",
            )

            world_path, manifest_path, render_path = write_world_export(
                root / "world.xml",
                [RobotInstance("free_1", "free", str(source), (1, 2, 3), (1, 0, 0, 0), {})],
            )

            world_xml = world_path.read_text(encoding="utf-8")
            self.assertIn('<attach model="model_free_1" prefix="free_1_"', world_xml)
            self.assertNotIn('<body name="free_1"', world_xml)
            self.assertIn('"free_base": true', manifest_path.read_text(encoding="utf-8"))
            render_script = render_path.read_text(encoding="utf-8")
            self.assertIn('robot.get("free_base")', render_script)
            self.assertIn('robot["position"]', render_script)


if __name__ == "__main__":
    unittest.main()