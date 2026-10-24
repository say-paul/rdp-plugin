"""Exercise flow.yaml through the Blender save/load operators."""

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
    tree = bpy.data.node_groups.new("YAML IO", "CARYAM_BEHAVIOR_TREE")
    bpy.context.scene.robot_behavior_tree = tree
    node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
    node.kind = "application"
    node.node_id = "camera"
    node.output_ports.add().port_name = "oak1"
    node.sync_sockets()
    with tempfile.TemporaryDirectory() as directory:
        yaml_path = Path(directory) / "flow.yaml"
        assert bpy.ops.robot_library.save_behavior(filepath=str(yaml_path)) == {"FINISHED"}
        assert "application:" in yaml_path.read_text(encoding="utf-8")
        reloaded = bpy.data.node_groups.new("YAML Reloaded", "CARYAM_BEHAVIOR_TREE")
        bpy.context.scene.robot_behavior_tree = reloaded
        assert bpy.ops.robot_library.load_behavior(filepath=str(yaml_path)) == {"FINISHED"}
        assert addon._graph_from_tree(reloaded).node_map()["camera"].output_names() == ("oak1",)
        json_path = Path(directory) / "flow.json"
        assert bpy.ops.robot_library.save_behavior(filepath=str(json_path)) == {"FINISHED"}
        assert '"nodes"' in json_path.read_text(encoding="utf-8")
    print("flow.yaml save/load operator test passed")
finally:
    robot_arm_library.unregister()
