"""Headless Blender smoke test for importing a real Menagerie model."""

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
menagerie_path = Path.home() / "git_repos" / "mujoco_menagerie"
scene.robot_setup_path = str(menagerie_path)
bpy.ops.robot_library.refresh()

piper = next(item for item in scene.robot_library if item.robot_id == "agilex_piper/piper")
root = blender_addon._place_robot(piper, Vector((0.0, 0.0, 0.0)))
mesh_objects = [obj for obj in bpy.data.objects if obj.type == "MESH" and obj.name.startswith("Geom:")]
assert len(mesh_objects) > 10, f"Expected detailed Piper geometry, got {len(mesh_objects)} mesh objects"
assert root["import_mode"] == "mujoco_compiled"
assert root["joint_count"] == 8

import mujoco

model = mujoco.MjModel.from_xml_path(piper.model_path)
data = mujoco.MjData(model)
home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
mujoco.mj_resetDataKeyframe(model, data, home_key)
mujoco.mj_forward(model, data)

sample_geom = mesh_objects[0]
geom_id = sample_geom["mujoco_geom_id"]
expected_position = Vector(tuple(float(value) for value in data.geom_xpos[geom_id]))
assert (sample_geom.matrix_world.translation - expected_position).length < 1e-6
mesh_id = int(model.geom_dataid[geom_id])
vertex_address = int(model.mesh_vertadr[mesh_id])
expected_vertex = Vector(tuple(float(value) for value in model.mesh_vert[vertex_address]))
assert (sample_geom.data.vertices[0].co - expected_vertex).length < 1e-6

sample_joint = next(obj for obj in bpy.data.objects if obj.name.startswith("Joint:"))
joint_id = sample_joint["mujoco_joint_id"]
expected_anchor = Vector(tuple(float(value) for value in data.xanchor[joint_id]))
assert (sample_joint.matrix_world.translation - expected_anchor).length < 1e-6

hinge_joint = bpy.data.objects["Joint: joint1"]
hinge_id = hinge_joint["mujoco_joint_id"]
qpos_address = int(model.jnt_qposadr[hinge_id])
new_qpos = float(data.qpos[qpos_address]) + 0.25
hinge_joint["qpos"] = new_qpos
hinge_joint.update_tag(refresh={"OBJECT"})
bpy.context.scene.frame_set(bpy.context.scene.frame_current)
bpy.context.view_layer.update()
data.qpos[qpos_address] = new_qpos
mujoco.mj_forward(model, data)

downstream_geom_id = next(
    geom_id
    for geom_id in range(model.ngeom)
    if int(model.geom_bodyid[geom_id]) >= 2
    and int(model.geom_group[geom_id]) != 3
    and int(model.geom_type[geom_id]) == int(mujoco.mjtGeom.mjGEOM_MESH)
)
downstream_geom = next(obj for obj in mesh_objects if obj["mujoco_geom_id"] == downstream_geom_id)
expected_downstream_position = Vector(tuple(float(value) for value in data.geom_xpos[downstream_geom_id]))
print(
    "Joint motion comparison:",
    tuple(round(value, 7) for value in downstream_geom.matrix_world.translation),
    tuple(round(value, 7) for value in expected_downstream_position),
    (downstream_geom.matrix_world.translation - expected_downstream_position).length,
)
assert (downstream_geom.matrix_world.translation - expected_downstream_position).length < 1e-5

assert bpy.ops.robot_library.keyframe_pose(root_name=root.name) == {"FINISHED"}
assert hinge_joint.animation_data is not None
assert bpy.ops.robot_library.reset_pose(root_name=root.name) == {"FINISHED"}
assert abs(float(hinge_joint["qpos"]) - float(hinge_joint["home_qpos"])) < 1e-12

print(f"MuJoCo Piper compiled import passed: {len(mesh_objects)} meshes, {root['joint_count']} joints")
robot_arm_library.unregister()