"""Validate SO-101 qpos controls against MuJoCo forward kinematics."""

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
item = next(item for item in scene.robot_library if item.robot_id == "robotstudio_so101/so101")
scene.robot_library_include_collisions = True
root = blender_addon._place_robot(item, Vector((0.0, 0.0, 0.0)))
controls = blender_addon._joint_controls(root)
assert len(controls) == 6, f"Expected 6 SO-101 controls, got {len(controls)}"
collision_objects = [obj for obj in bpy.data.objects if obj.get("is_collision")]
assert root["collision_count"] == 30
assert len(collision_objects) == 30
assert all(obj.display_type == "WIRE" and obj.hide_render for obj in collision_objects)

import mujoco

model = mujoco.MjModel.from_xml_path(item.model_path)
data = mujoco.MjData(model)
home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
if home_key >= 0:
    mujoco.mj_resetDataKeyframe(model, data, home_key)
else:
    mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)

joint = bpy.data.objects["Joint: shoulder_pan"]
joint_id = joint["mujoco_joint_id"]
qpos_address = int(model.jnt_qposadr[joint_id])
new_qpos = float(data.qpos[qpos_address]) + 0.2
joint["qpos"] = new_qpos
joint.update_tag(refresh={"OBJECT"})
scene.frame_set(scene.frame_current)
bpy.context.view_layer.update()
data.qpos[qpos_address] = new_qpos
mujoco.mj_forward(model, data)

geom_id = next(
    geom_id
    for geom_id in range(model.ngeom)
    if int(model.geom_bodyid[geom_id]) > int(model.jnt_bodyid[joint_id])
    and int(model.geom_group[geom_id]) == 2
)
geom = next(obj for obj in bpy.data.objects if obj.get("mujoco_geom_id") == geom_id)
expected_position = Vector(tuple(float(value) for value in data.geom_xpos[geom_id]))
error = (geom.matrix_world.translation - expected_position).length
assert error < 1e-5, f"SO-101 qpos error is {error}"

print(f"SO-101 test passed: {len(controls)} controls, {len(collision_objects)} collision geoms, position error {error}")
robot_arm_library.unregister()