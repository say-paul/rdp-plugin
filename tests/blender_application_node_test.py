"""Validate dynamic application-node sockets in Blender."""

import sys
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon
from robot_arm_library.behavior_graph import PortSpec


robot_arm_library.register()
try:
    scene = bpy.context.scene
    result = bpy.ops.robot_library.new_behavior()
    assert result == {"FINISHED"}, result
    assert len(scene.robot_behavior_tree.nodes) == 0
    canvas = scene.robot_behavior_tree

    for kind in ("application", "sensor", "ai_model", "action", "condition", "start", "end"):
        result = bpy.ops.robot_library.add_behavior_node(kind=kind)
        assert result == {"FINISHED"}, (kind, result)
    assert {node.kind for node in scene.robot_behavior_tree.nodes} == {
        "application",
        "sensor",
        "ai_model",
        "action",
        "condition",
        "start",
        "end",
    }
    result = bpy.ops.robot_library.new_behavior()
    assert result == {"FINISHED"}, result
    assert scene.robot_behavior_tree == canvas
    assert len(canvas.nodes) == 0

    result = bpy.ops.robot_library.add_behavior_node(preset="virtual_camera")
    assert result == {"FINISHED"}, result
    camera_preset = canvas.nodes[-1]
    assert camera_preset.kind == "application"
    assert camera_preset.output_ports[0].port_name == "rgb"
    assert camera_preset.output_ports[0].shape_tuple() == (1, 3, 480, 640)

    result = bpy.ops.robot_library.add_behavior_node(preset="ai_model")
    assert result == {"FINISHED"}, result
    ai_preset = canvas.nodes[-1]
    assert ai_preset.input_ports[0].shape_tuple() == (1, 3, 480, 640)

    result = bpy.ops.robot_library.add_behavior_node(preset="actuator_system")
    assert result == {"FINISHED"}, result
    actuator_preset = canvas.nodes[-1]
    assert actuator_preset.input_ports[0].port_name == "joint_positions"
    assert actuator_preset.input_ports[0].shape_tuple() == (1, 6)

    area = next(area for area in bpy.context.screen.areas if area.type == "PROPERTIES")
    with bpy.context.temp_override(area=area):
        result = bpy.ops.robot_library.open_behavior_editor()
    assert result == {"FINISHED"}, result
    space = area.spaces.active
    assert space.type == "NODE_EDITOR"
    assert space.edit_tree == canvas

    tree = bpy.data.node_groups.new("Port Sync Test", "CARYAM_BEHAVIOR_TREE")
    scene.robot_behavior_tree = tree
    node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
    node.kind = "application"

    port = node.input_ports.add()
    port.port_name = "oak1"
    port.shape_csv = "1,3,480,640"
    node.sync_sockets()

    assert list(node.inputs.keys()) == ["oak1"], list(node.inputs.keys())
    assert list(node.outputs.keys()) == [], list(node.outputs.keys())
    assert port.shape_tuple() == (1, 3, 480, 640)

    node.kind = "sensor"
    assert list(node.inputs.keys()) == ["value"], list(node.inputs.keys())
    assert list(node.outputs.keys()) == ["value"], list(node.outputs.keys())

    node.kind = "application"
    output_port = node.output_ports.add()
    output_port.port_name = "decision"
    node.sync_sockets()
    assert list(node.outputs.keys()) == ["decision"], list(node.outputs.keys())
    node.output_ports.remove(0)
    node.sync_sockets()
    assert list(node.outputs.keys()) == [], list(node.outputs.keys())

    node.node_id = "camera"
    node.output_ports.add().port_name = "oak1"
    node.output_ports[0].shape_csv = "1,3,480,640"
    node.sync_sockets()
    transform = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
    transform.node_id = "image_transform"
    transform.kind = "application"
    transform.input_ports.add().port_name = "oak1"
    transform.output_ports.add().port_name = "smolvla"
    transform.sync_sockets()
    tree.links.new(node.outputs["oak1"], transform.inputs["oak1"])

    graph = blender_addon._graph_from_tree(tree)
    assert graph.validate() == [], graph.validate()
    assert graph.node_map()["camera"].outputs == (PortSpec("oak1", (1, 3, 480, 640)),)
    link = graph.links[0]
    assert (link.source, link.source_port, link.target, link.target_port) == (
        "camera",
        "oak1",
        "image_transform",
        "oak1",
    )

    incompatible = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
    incompatible.node_id = "incompatible_model"
    incompatible.kind = "application"
    incompatible.input_ports.add().port_name = "oak1"
    incompatible.input_ports[0].shape_csv = "1,3,224,224"
    incompatible.sync_sockets()
    tree.links.new(node.outputs["oak1"], incompatible.inputs["oak1"])
    tree.update()
    assert all(link.to_node != incompatible for link in tree.links)

    values = blender_addon._run_behavior_graph(graph)
    assert values["image_transform"] == {"smolvla": None}
    print("Application node socket sync test passed")
finally:
    robot_arm_library.unregister()