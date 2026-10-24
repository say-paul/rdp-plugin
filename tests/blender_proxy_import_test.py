"""Check that fast robot previews stay small and retain MuJoCo joint placement."""

from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import patch

import bpy
from mathutils import Vector


repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon


robot_arm_library.register()
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)

scene = bpy.context.scene
scene.robot_setup_path = str(Path.home() / "git_repos" / "mujoco_menagerie")
assert scene.robot_library_preview_mode == "PROXY"
bpy.ops.robot_library.refresh()

piper = next(item for item in scene.robot_library if item.robot_id == "agilex_piper/piper")
root = blender_addon._place_robot(piper, Vector((0.0, 0.0, 0.0)))
geoms = [obj for obj in bpy.data.objects if obj.type == "MESH" and "mujoco_geom_id" in obj]
assert root["import_mode"] == "mujoco_compiled"
assert root["preview_mode"] == "PROXY"
assert root["joint_count"] == 8
assert len(geoms) > 10
assert all(len(obj.data.vertices) == 8 and len(obj.data.polygons) == 6 for obj in geoms)
assert root["source_triangle_count"] > root["triangle_count"] * 50

instances = blender_addon._world_robot_instances()
assert len(instances) == 1
assert Path(instances[0].source_path) == Path(piper.model_path)

import mujoco

model = mujoco.MjModel.from_xml_path(piper.model_path)
data = mujoco.MjData(model)
home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
mujoco.mj_resetDataKeyframe(model, data, home_key)
mujoco.mj_forward(model, data)

sample = next(obj for obj in geoms if int(model.geom_bodyid[obj["mujoco_geom_id"]]) >= 2)
geom_id = int(sample["mujoco_geom_id"])
bounds = model.geom_aabb[geom_id]
for axis in range(3):
    coordinates = [float(vertex.co[axis]) for vertex in sample.data.vertices]
    half_size = max(0.0005, float(bounds[axis + 3]))
    assert abs(min(coordinates) - (bounds[axis] - half_size)) < 1e-6
    assert abs(max(coordinates) - (bounds[axis] + half_size)) < 1e-6
expected = Vector(tuple(float(value) for value in data.geom_xpos[geom_id]))
assert (sample.matrix_world.translation - expected).length < 1e-6

joint = bpy.data.objects["Joint: joint1"]
joint_id = int(joint["mujoco_joint_id"])
qpos_address = int(model.jnt_qposadr[joint_id])
new_qpos = float(data.qpos[qpos_address]) + 0.2
joint["qpos"] = new_qpos
joint.update_tag(refresh={"OBJECT"})
scene.frame_set(scene.frame_current)
bpy.context.view_layer.update()
data.qpos[qpos_address] = new_qpos
mujoco.mj_forward(model, data)
expected = Vector(tuple(float(value) for value in data.geom_xpos[geom_id]))
assert (sample.matrix_world.translation - expected).length < 1e-5

with patch("robot_arm_library.compiled_importer.import_compiled_mjcf", side_effect=ModuleNotFoundError("mujoco")):
    with patch.object(blender_addon, "_import_mjcf_meshes", side_effect=AssertionError("dense fallback used")):
        fallback = blender_addon._place_robot(piper, Vector((2.0, 0.0, 0.0)))
assert fallback["import_mode"] == "proxy_unavailable"
assert len([obj for obj in fallback.children if obj.type == "MESH"]) == 4

print(
    f"Fast assembly preview passed: {root['source_triangle_count']:,} source triangles "
    f"to {root['triangle_count']:,} Blender triangles"
)
robot_arm_library.unregister()
