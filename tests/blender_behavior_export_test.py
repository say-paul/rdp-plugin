"""Validate the behavior graph and MuJoCo world export in Blender."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from xml.etree import ElementTree

import bpy
from mathutils import Vector


repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon
from robot_arm_library.behavior_graph import default_behavior_graph


robot_arm_library.register()
scene = bpy.context.scene
scene.robot_setup_path = str(Path.home() / "git_repos" / "mujoco_menagerie")
bpy.ops.robot_library.refresh()
item = next(item for item in scene.robot_library if item.robot_id == "robotstudio_so101/so101")
root = blender_addon._place_robot(item, Vector((1.0, 2.0, 0.0)))
controls = blender_addon._joint_controls(root)
if not controls:
    joint = bpy.data.objects.new("Joint: shoulder_pan", None)
    joint["qpos"] = 0.0
    joint["home_qpos"] = 0.0
    joint["mujoco_joint_id"] = 0
    joint.parent = root
    bpy.data.collections["Robot Library"].objects.link(joint)

tree = bpy.data.node_groups.new("Behavior Export Test", "CARYAM_BEHAVIOR_TREE")
scene.robot_behavior_tree = tree
blender_addon._populate_behavior_tree(tree, default_behavior_graph())
action = next(node for node in tree.nodes if node.kind == "action")
action.robot_id = root["robot_id"]
action.joint_name = "shoulder_pan"
graph = blender_addon._graph_from_tree(tree)
assert graph.validate() == []

joint = blender_addon._joint_controls(root)[0]
old_qpos = float(joint["qpos"])
blender_addon._run_behavior_graph(graph)
assert abs(float(joint["qpos"]) - old_qpos) < 1e-12

robot_collection = bpy.data.collections["Robot Library"]
scene_collection = bpy.data.collections.new("World Export Test")
scene.collection.children.link(scene_collection)
robot_collection.objects.unlink(root)
scene_collection.objects.link(root)

bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, location=(0.0, 2.5, 1.8))
ball = bpy.context.active_object
ball.name = "Plastic Ball"
bpy.ops.mesh.primitive_cube_add(location=(1.0, 0.0, 0.5))
static_block = bpy.context.active_object
static_block.name = "Static Block"

with tempfile.TemporaryDirectory() as directory:
    world_path, manifest_path, render_path = blender_addon._export_world_file(str(Path(directory) / "world.xml"))
    world_root = ElementTree.parse(world_path).getroot()
    assert world_root.find("asset/model") is not None
    assert world_root.find("worldbody/body/attach") is not None
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["robots"][0]["qpos"]["shoulder_pan"] == old_qpos
    assert manifest["scene_meshes"]
    assert Path(manifest["scene_meshes"][0]["source_path"]).is_file()
    ball_mesh = next(mesh for mesh in manifest["scene_meshes"] if mesh["instance_id"] == "Plastic Ball")
    block_mesh = next(mesh for mesh in manifest["scene_meshes"] if mesh["instance_id"] == "Static Block")
    assert ball_mesh["dynamic"]
    assert not block_mesh["dynamic"]
    assert Path(block_mesh["source_path"]).is_file()
    assert world_root.find("worldbody/body/freejoint[@name='scene_Plastic_Ball_free']") is not None
    assert world_path.with_suffix(".usd").is_file()
    assert "mujoco.viewer" in render_path.read_text(encoding="utf-8")

print("Behavior graph and MuJoCo world export test passed")
robot_arm_library.unregister()