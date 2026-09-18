"""Headless Blender smoke test for add-on registration and demo discovery."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import bpy


repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon
from robot_arm_library.setup_parser import discover_robot_definitions


robot_arm_library.register()
robot_arm_library.register()

scene = bpy.context.scene
assert hasattr(scene, "robot_library")
menagerie_path = Path.home() / "git_repos" / "mujoco_menagerie"
if menagerie_path.is_dir():
    scene.robot_setup_path = str(menagerie_path)
bpy.ops.robot_library.refresh()
expected_count = len(discover_robot_definitions(menagerie_path)) if menagerie_path.is_dir() else 3
print(f"Catalog check: expected {expected_count}, discovered {len(scene.robot_library)} from {scene.robot_setup_path}")
assert len(scene.robot_library) == expected_count, f"Expected {expected_count} robot definitions"
robot_count = len(scene.robot_library)

scene.robot_library_filter = ""
filter_list = SimpleNamespace(bitflag_filter_item=1 << 30)
filter_flags, _ = blender_addon.ROBOT_UL_library.filter_items(filter_list, bpy.context, scene, "robot_library")
assert all(filter_flags), "An empty catalog search must show every robot"
scene.robot_library_filter = "panda"
filter_flags, _ = blender_addon.ROBOT_UL_library.filter_items(filter_list, bpy.context, scene, "robot_library")
expected_flags = [filter_list.bitflag_filter_item if "panda" in item.name.lower() else 0 for item in scene.robot_library]
assert filter_flags == expected_flags, "Catalog search must show only matching robots"
scene.robot_library_filter = ""
assert blender_addon.ROBOT_PT_joint_controls.bl_category == "Robot Control"
assert not getattr(blender_addon.ROBOT_PT_joint_controls, "bl_parent_id", "")
assert blender_addon.ROBOT_PT_behavior_graph.bl_label == "Robot Flow"
assert blender_addon.ROBOT_PT_behavior_graph.bl_category == "Robot Control"
assert not getattr(blender_addon.ROBOT_PT_behavior_graph, "bl_parent_id", "")
assert blender_addon.ROBOT_PT_mujoco_world.bl_category == "MuJoCo World Simulation"
assert blender_addon.ROBOT_PT_mujoco_world.bl_label == "MuJoCo Sim"
assert blender_addon.ROBOT_PT_mujoco_world.bl_parent_id == "ROBOT_PT_world_simulation"
assert blender_addon.ROBOT_PT_rcs_control.bl_category == "Robot Control"
assert not getattr(blender_addon.ROBOT_PT_rcs_control, "bl_parent_id", "")

robot_arm_library.unregister()
print(f"MuJoCo Robot Arm Library smoke test passed: {robot_count} robots discovered")