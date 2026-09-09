"""Headless Blender smoke test for add-on registration and demo discovery."""

from __future__ import annotations

import sys
from pathlib import Path

import bpy


repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library


robot_arm_library.register()
robot_arm_library.register()

scene = bpy.context.scene
assert hasattr(scene, "robot_library")
menagerie_path = Path.home() / "git_repos" / "mujoco_menagerie"
if menagerie_path.is_dir():
    scene.robot_setup_path = str(menagerie_path)
bpy.ops.robot_library.refresh()
expected_count = 111 if menagerie_path.is_dir() else 3
assert len(scene.robot_library) == expected_count, f"Expected {expected_count} robot definitions"
robot_count = len(scene.robot_library)

robot_arm_library.unregister()
print(f"MuJoCo Robot Arm Library smoke test passed: {robot_count} robots discovered")