"""Check that Blender rigid-body controls reach the exported MuJoCo world."""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path
from xml.etree import ElementTree

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
from robot_arm_library.blender_addon import _world_scene_meshes

robot_arm_library.register()
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)

bpy.ops.mesh.primitive_cube_add(location=(0, 0, 2))
cube = bpy.context.active_object
cube.name = "Falling Cube"
assert bpy.ops.rigidbody.object_add() == {"FINISHED"}
cube.rigid_body.mass = 2.5
cube.rigid_body.friction = 0.7
cube.rigid_body.restitution = 0.4
cube.rigid_body.collision_shape = "BOX"

bpy.ops.mesh.primitive_uv_sphere_add(location=(2, 0, 1))
sphere = bpy.context.active_object
sphere.name = "Passive Sphere"
assert bpy.ops.rigidbody.object_add() == {"FINISHED"}
sphere.rigid_body.type = "PASSIVE"
sphere.rigid_body.collision_shape = "SPHERE"

bpy.ops.mesh.primitive_cube_add(location=(4, 0, 1))
disabled = bpy.context.active_object
disabled.name = "Disabled Collider"
assert bpy.ops.rigidbody.object_add() == {"FINISHED"}
disabled.rigid_body.enabled = False

bpy.ops.mesh.primitive_uv_sphere_add(location=(6, 0, 1))
unconfigured = bpy.context.active_object
unconfigured.name = "Ball By Name"

with tempfile.TemporaryDirectory() as directory:
    world_path = Path(directory) / "world.xml"
    meshes = _world_scene_meshes(world_path)
    by_name = {mesh.instance_id: mesh for mesh in meshes}
    assert by_name["Falling Cube"].dynamic
    assert by_name["Falling Cube"].collision_shape == "BOX"
    assert by_name["Falling Cube"].mass == 2.5
    assert abs(by_name["Falling Cube"].friction - 0.7) < 1e-6
    assert by_name["Passive Sphere"].collision_shape == "SPHERE"
    assert not by_name["Passive Sphere"].dynamic
    assert not by_name["Disabled Collider"].collision_enabled
    assert not by_name["Ball By Name"].dynamic
    assert bpy.ops.robot_library.export_world(filepath=str(world_path)) == {"FINISHED"}
    assert world_path.is_file()
    root = ElementTree.parse(world_path).getroot()
    falling = root.find("worldbody/body[@name='scene_Falling_Cube']")
    assert falling.find("freejoint") is not None
    visual = falling.find("geom[@name='scene_Falling_Cube_visual']")
    collider = falling.find("geom[@name='scene_Falling_Cube']")
    assert visual.get("contype") == "0"
    assert collider.get("type") == "box"
    assert collider.get("mass") == "2.5"
    assert abs(float(collider.get("friction").split()[0]) - 0.7) < 1e-6
    assert collider.get("solref") is not None
    passive = root.find("worldbody/body[@name='scene_Passive_Sphere']")
    assert passive.find("freejoint") is None
    assert passive.find("geom[@name='scene_Passive_Sphere']").get("type") == "sphere"
    disabled_body = root.find("worldbody/body[@name='scene_Disabled_Collider']")
    assert disabled_body.find("freejoint") is None
    assert disabled_body.find("geom[@name='scene_Disabled_Collider']").get("contype") == "0"

robot_arm_library.unregister()
print("Blender scene physics export test passed")
