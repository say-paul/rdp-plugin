"""Exercise multiple asset sources and catalog node creation in Blender."""

import json
import sys
import tempfile
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as addon

robot_arm_library.register()
try:
    with tempfile.TemporaryDirectory() as robot_dir, tempfile.TemporaryDirectory() as sensor_dir:
        (Path(robot_dir) / "robot.json").write_text(
            json.dumps({"name": "Test Arm", "model_path": "arm.xml"}), encoding="utf-8",
        )
        (Path(sensor_dir) / "registry.json").write_text(
            json.dumps({"sensors": {"lidar-1": {
                "category": "lidar", "name": "Front Lidar",
                "lidar": {"horizontal_samples": 16},
            }}}), encoding="utf-8",
        )
        scene = bpy.context.scene
        scene.robot_setup_path = robot_dir
        source = scene.robot_asset_sources.add()
        source.path = sensor_dir
        source.kind = "sensor"
        assert bpy.ops.robot_library.refresh() == {"FINISHED"}
        assert len(scene.robot_library) == 1
        assert len(scene.sensor_library) == 1
        assert bpy.ops.robot_library.add_catalog_node(template="selected_sensor") == {"FINISHED"}
        node = scene.robot_behavior_tree.nodes[-1]
        assert node.catalog_locked
        assert node.output_ports[0].port_name == "ranges"
        assert node.output_ports[0].shape_tuple() == (1, 16)
        graph = addon._graph_from_tree(scene.robot_behavior_tree)
        assert graph.node_map()[node.node_id].data["backend_capabilities"]["mujoco"] == "rangefinder_fan"
        assert bpy.ops.robot_library.add_catalog_node(template="robot_state") == {"FINISHED"}
        assert len(scene.robot_behavior_tree.nodes[-1].output_ports) == 3
    print("Multi-source asset discovery test passed")
finally:
    robot_arm_library.unregister()
