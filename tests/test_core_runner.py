import json
import threading
import unittest
from urllib.request import Request, urlopen

from robot_arm_library.core_bridge import CoreBridge, create_server
from robot_arm_library.core_runner import CoreFlowRunner, FlowPacket
from robot_arm_library.data_graph import DataGraph, DataLink, DataNode, DataPort


def _joint_target_graph() -> DataGraph:
    return DataGraph(
        name="Simulation Joint Target",
        nodes=[
            DataNode(
                "sensor",
                "sensor",
                outputs=(DataPort("target", data_type="joint_positions"),),
            ),
            DataNode(
                "controller",
                "joint_controller",
                data={"joint_limits": [[-1.0, 1.0], [-2.0, 2.0]]},
                inputs=(DataPort("target", data_type="joint_positions"),),
                outputs=(DataPort("joint_target", data_type="joint_target", delivery="command"),),
            ),
            DataNode(
                "target",
                "joint_target",
                inputs=(DataPort("joint_target", data_type="joint_target", delivery="command"),),
                outputs=(DataPort("joint_target", data_type="joint_target", delivery="command"),),
            ),
        ],
        links=[
            DataLink("sensor", "controller", "target", "target"),
            DataLink("controller", "target", "joint_target", "joint_target"),
        ],
    )


class CoreFlowRunnerTests(unittest.TestCase):
    def test_fresh_valid_target_is_published_to_compact_blackboard(self):
        runner = CoreFlowRunner(_joint_target_graph(), maximum_age_seconds=0.5)
        runner.start()

        result = runner.tick({"sensor": FlowPacket([0.2, -0.4], 10.0, data_type="joint_positions")}, now=10.1)

        self.assertEqual(result.state, "running")
        self.assertTrue(result.blackboard["caryam.control.command_valid"])
        self.assertEqual(result.blackboard["caryam.control.joint_target"]["positions"], [0.2, -0.4])

    def test_stale_or_invalid_input_never_publishes_a_valid_command(self):
        runner = CoreFlowRunner(_joint_target_graph(), maximum_age_seconds=0.5)
        runner.start()

        stale = runner.tick({"sensor": FlowPacket([0.2, -0.4], 0.0, data_type="joint_positions")}, now=1.0)
        self.assertFalse(stale.blackboard["caryam.control.command_valid"])

        invalid = runner.tick({"sensor": FlowPacket([2.0, 0.0], 2.0, data_type="joint_positions")}, now=2.1)
        self.assertFalse(invalid.blackboard["caryam.control.command_valid"])
        self.assertIn("outside", invalid.blackboard["caryam.control.joint_target"]["reason"])

    def test_virtual_sensor_nodes_accept_external_sensor_packets(self):
        graph = DataGraph(
            name="Virtual Sensors",
            nodes=[
                DataNode(
                    "camera",
                    "virtual_rgb_camera",
                    outputs=(DataPort("rgb_image", data_type="image_rgb8"),),
                ),
                DataNode(
                    "imu",
                    "virtual_imu",
                    outputs=(DataPort("imu", data_type="imu"),),
                ),
            ],
        )
        runner = CoreFlowRunner(graph)
        runner.start()

        result = runner.tick(
            {
                "camera": FlowPacket("frame", 10.0, data_type="image_rgb8"),
                "imu": FlowPacket({"angular_velocity": [0, 0, 0]}, 10.0, data_type="imu"),
            },
            now=10.1,
        )

        self.assertEqual(result.outputs["camera"]["rgb_image"].value, "frame")
        self.assertEqual(result.outputs["imu"]["imu"].data_type, "imu")

    def test_bridge_runs_and_stops_a_simulation_only_flow(self):
        bridge = CoreBridge()
        status = bridge.handle(
            "POST",
            "/run",
            {
                "core_host": "core.example.test",
                "core_port": 17080,
                "graph": _joint_target_graph().to_dict(),
                "sensors": {"sensor": FlowPacket([0.0, 0.0], 5.0).to_dict()},
                "now": 5.1,
            },
        )

        self.assertEqual(status["state"], "running")
        self.assertTrue(status["simulation_only"])
        self.assertTrue(status["blackboard"]["caryam.control.command_valid"])
        self.assertEqual(bridge.handle("POST", "/stop")["state"], "stopped")

    def test_http_bridge_serves_simulation_status_and_outputs(self):
        server = create_server("127.0.0.1", 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base_url = f"http://127.0.0.1:{server.server_port}"
        try:
            payload = {
                "graph": _joint_target_graph().to_dict(),
                "sensors": {"sensor": FlowPacket([0.0, 0.0], 8.0).to_dict()},
                "now": 8.1,
            }
            request = Request(
                f"{base_url}/run",
                data=json.dumps(payload).encode("utf-8"),
                method="POST",
                headers={"Content-Type": "application/json"},
            )
            with urlopen(request) as response:
                status = json.loads(response.read().decode("utf-8"))
            self.assertEqual(status["state"], "running")
            self.assertTrue(status["blackboard"]["caryam.control.command_valid"])

            with urlopen(f"{base_url}/status") as response:
                health = json.loads(response.read().decode("utf-8"))
            self.assertTrue(health["simulation_only"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == "__main__":
    unittest.main()