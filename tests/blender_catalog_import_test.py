"""Compare real Menagerie geometry import across representative models."""

from __future__ import annotations

import sys
from pathlib import Path

import bpy
from mathutils import Vector


repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon


robot_arm_library.register()
scene = bpy.context.scene
scene.robot_setup_path = str(Path.home() / "git_repos" / "mujoco_menagerie")
bpy.ops.robot_library.refresh()

model_ids = (
    "agilex_piper/piper",
    "universal_robots_ur5e/ur5e",
    "franka_emika_panda/panda",
    "ufactory_xarm7/xarm7",
    "trs_so_arm100/so_arm100",
)
items = {item.robot_id: item for item in scene.robot_library}
for model_id in model_ids:
    item = items[model_id]
    objects_before = set(bpy.data.objects)
    root = blender_addon._place_robot(item, Vector((0.0, 0.0, 0.0)))
    created_objects = [obj for obj in bpy.data.objects if obj not in objects_before]
    mesh_count = sum(obj.type == "MESH" and "mujoco_geom_id" in obj for obj in created_objects)
    joint_count = sum("mujoco_joint_id" in obj for obj in created_objects)
    assert root["import_mode"] == "mujoco_compiled", f"{model_id} used {root['import_mode']}"
    assert mesh_count > 0, f"{model_id} created no compiled visual meshes"
    print(f"{model_id}: {mesh_count} visual meshes, {joint_count} joints")
    for obj in reversed(created_objects):
        if obj.name in bpy.data.objects:
            bpy.data.objects.remove(obj, do_unlink=True)

robot_arm_library.unregister()