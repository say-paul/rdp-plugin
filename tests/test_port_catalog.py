import unittest

from robot_arm_library.behavior_graph import BehaviorGraph, BehaviorLink, BehaviorNode, PortSpec
from robot_arm_library.port_catalog import (
    catalog_node_template, robot_joint_command_ports, robot_joint_state_ports, sensor_port_templates,
)


class PortCatalogTests(unittest.TestCase):
    def test_robot_contracts_follow_dof_and_preserve_semantics(self):
        for dof in (6, 7):
            state = robot_joint_state_ports(dof)
            command = robot_joint_command_ports(dof)
            self.assertEqual([port.name for port in state], ["joint_positions", "joint_velocities", "joint_efforts"])
            self.assertEqual(command[0].shape, (1, dof))
            self.assertEqual(state[0].shape, (1, dof))
            self.assertEqual((state[0].dtype, state[0].unit), ("float32", "rad"))
            self.assertEqual(command[0].semantic, "joint_position_command")
        with self.assertRaises(ValueError):
            robot_joint_state_ports(0)

    def test_sensor_categories_resolve_concrete_ports(self):
        entries = {
            "depth_camera": {"camera": {"resolution_width": 640, "resolution_height": 480}},
            "lidar": {"lidar": {"horizontal_samples": 180}},
            "imu": {},
            "force_torque": {},
            "tactile": {"cells": 8},
            "radar": {"bins": 16},
        }
        for category, details in entries.items():
            with self.subTest(category=category):
                ports = sensor_port_templates({"category": category, **details})
                self.assertTrue(ports)
                self.assertTrue(all(port.shape for port in ports))
        self.assertEqual(sensor_port_templates({"category": "depth_camera", **entries["depth_camera"]})[0].shape, (1, 1, 480, 640))
        self.assertEqual(sensor_port_templates({"category": "lidar", "lidar": {"channels": 16, "horizontal_samples": 180}})[0].shape, (1, 16, 180))
        with self.assertRaisesRegex(ValueError, "Missing required sensor dimension"):
            sensor_port_templates({"category": "lidar"})

    def test_catalog_template_keeps_backend_capabilities(self):
        entry = {"category": "lidar", "lidar": {"horizontal_samples": 32}, "backend_capabilities": {"mujoco": "rangefinder_fan"}}
        node = catalog_node_template("lidar", entry)
        self.assertEqual(node.outputs[0].shape, (1, 32))
        self.assertEqual(node.data["backend_capabilities"], {"mujoco": "rangefinder_fan"})

    def test_graph_reports_semantic_and_unit_mismatches(self):
        source = BehaviorNode("state", "application", outputs=(PortSpec("joint", (1, 6), dtype="float32", semantic="joint_state", unit="rad"),))
        target = BehaviorNode("command", "application", inputs=(PortSpec("joint", (1, 6), dtype="float32", semantic="joint_position_command", unit="rad"),))
        graph = BehaviorGraph(nodes=[source, target], links=[BehaviorLink("state", "command", "joint", "joint")])
        self.assertIn("Incompatible port semantic", " ".join(graph.validate()))
        self.assertEqual(PortSpec("legacy").compatibility_error(PortSpec("legacy", dtype="float32")), None)
        self.assertEqual(PortSpec.from_dict({"name": "old", "shape": [1, 2]}), PortSpec("old", (1, 2)))
        self.assertEqual(PortSpec("x", unit="m").to_dict()["unit"], "m")


if __name__ == "__main__":
    unittest.main()
