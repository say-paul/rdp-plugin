"""Build Blender geometry and kinematic markers from a compiled MuJoCo model."""

from __future__ import annotations

import importlib
import site
import sys
from pathlib import Path
from types import ModuleType

import bpy
from mathutils import Matrix, Vector


def _load_mujoco() -> ModuleType:
    try:
        return importlib.import_module("mujoco")
    except ModuleNotFoundError:
        user_site = site.getusersitepackages()
        if user_site not in sys.path:
            sys.path.insert(0, user_site)
        return importlib.import_module("mujoco")


def _pose_matrix(position, rotation) -> Matrix:
    matrix = Matrix.Identity(4)
    rotation_values = rotation.reshape((3, 3))
    for row in range(3):
        for column in range(3):
            matrix[row][column] = float(rotation_values[row][column])
        matrix[row][3] = float(position[row])
    return matrix


def _material(model, geom_id: int, cache: dict[tuple[float, ...], bpy.types.Material]) -> bpy.types.Material:
    material_id = int(model.geom_matid[geom_id])
    rgba = model.mat_rgba[material_id] if material_id >= 0 else model.geom_rgba[geom_id]
    key = tuple(round(float(value), 5) for value in rgba)
    material = cache.get(key)
    if material is None:
        material = bpy.data.materials.new(name=f"MuJoCo Material {len(cache) + 1}")
        material.diffuse_color = key
        cache[key] = material
    return material


def _mesh_data(model, mesh_id: int, cache: dict[int, bpy.types.Mesh]) -> bpy.types.Mesh:
    cached = cache.get(mesh_id)
    if cached is not None:
        return cached
    vertex_address = int(model.mesh_vertadr[mesh_id])
    vertex_count = int(model.mesh_vertnum[mesh_id])
    face_address = int(model.mesh_faceadr[mesh_id])
    face_count = int(model.mesh_facenum[mesh_id])
    vertices = [tuple(float(value) for value in vertex) for vertex in model.mesh_vert[vertex_address:vertex_address + vertex_count]]
    faces = [tuple(int(value) for value in face) for face in model.mesh_face[face_address:face_address + face_count]]
    mesh = bpy.data.meshes.new(name=f"MuJoCo Mesh {mesh_id}")
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    cache[mesh_id] = mesh
    return mesh


def _primitive_object(model, geom_id: int, name: str) -> bpy.types.Object | None:
    geom_type = int(model.geom_type[geom_id])
    size = tuple(float(value) for value in model.geom_size[geom_id])
    if geom_type == 2:
        bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=16, radius=1.0)
        obj = bpy.context.object
        obj.scale = (size[0], size[0], size[0])
    elif geom_type == 3:
        import bmesh

        mesh = bpy.data.meshes.new(name=f"{name} Mesh")
        geometry = bmesh.new()
        radius, half_length = size[0], size[1]
        bmesh.ops.create_uvsphere(
            geometry,
            u_segments=24,
            v_segments=12,
            radius=radius,
            matrix=Matrix.Translation((0.0, 0.0, half_length)),
        )
        bmesh.ops.create_uvsphere(
            geometry,
            u_segments=24,
            v_segments=12,
            radius=radius,
            matrix=Matrix.Translation((0.0, 0.0, -half_length)),
        )
        if half_length > 0:
            bmesh.ops.create_cone(
                geometry,
                cap_ends=False,
                cap_tris=False,
                segments=24,
                radius1=radius,
                radius2=radius,
                depth=2.0 * half_length,
            )
        geometry.to_mesh(mesh)
        geometry.free()
        obj = bpy.data.objects.new(name, mesh)
    elif geom_type == 4:
        bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=16, radius=1.0)
        obj = bpy.context.object
        obj.scale = size
    elif geom_type == 5:
        bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=1.0, depth=2.0)
        obj = bpy.context.object
        obj.scale = (size[0], size[0], size[1])
    elif geom_type == 6:
        bpy.ops.mesh.primitive_cube_add(size=2.0)
        obj = bpy.context.object
        obj.scale = size
    else:
        return None
    obj.name = name
    return obj


def _move_to_collection(obj: bpy.types.Object, collection: bpy.types.Collection) -> None:
    for current_collection in list(obj.users_collection):
        current_collection.objects.unlink(obj)
    if obj.name not in collection.objects:
        collection.objects.link(obj)


def _joint_pose(position, axis) -> Matrix:
    axis_vector = Vector(tuple(float(value) for value in axis))
    if axis_vector.length_squared == 0:
        axis_vector = Vector((0.0, 0.0, 1.0))
    rotation = Vector((0.0, 0.0, 1.0)).rotation_difference(axis_vector.normalized())
    return Matrix.Translation(Vector(tuple(float(value) for value in position))) @ rotation.to_matrix().to_4x4()


def _add_qpos_driver(motion_object: bpy.types.Object, joint_object: bpy.types.Object, channel: str, index: int, home_value: float) -> None:
    driver_curve = motion_object.driver_add(channel, index)
    driver = driver_curve.driver
    driver.type = "SCRIPTED"
    variable = driver.variables.new()
    variable.name = "qpos"
    variable.type = "SINGLE_PROP"
    target = variable.targets[0]
    target.id = joint_object
    target.data_path = '["qpos"]'
    driver.expression = f"qpos - ({home_value:.17g})"


def import_compiled_mjcf(
    path: Path,
    root: bpy.types.Object,
    collection: bpy.types.Collection,
    include_collisions: bool = False,
) -> list[bpy.types.Object]:
    mujoco = _load_mujoco()
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
    else:
        mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    imported: list[bpy.types.Object] = []
    body_objects: dict[int, bpy.types.Object] = {0: root}
    joint_types = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}
    for body_id in range(1, model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or f"body_{body_id}"
        parent = body_objects.get(int(model.body_parentid[body_id]), root)
        joint_address = int(model.body_jntadr[body_id])
        joint_count = int(model.body_jntnum[body_id])
        for joint_id in range(joint_address, joint_address + joint_count):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or f"joint_{joint_id}"
            joint_type = int(model.jnt_type[joint_id])
            joint_object = bpy.data.objects.new(f"Joint: {joint_name}", None)
            joint_object.empty_display_type = "ARROWS"
            joint_object.empty_display_size = 0.06
            joint_object.show_in_front = True
            joint_object.hide_render = True
            joint_object["mujoco_joint_id"] = joint_id
            joint_object["joint_type"] = joint_types.get(joint_type, "unknown")
            joint_object["range"] = tuple(float(value) for value in model.jnt_range[joint_id])
            collection.objects.link(joint_object)
            joint_object.parent = parent
            joint_object.matrix_world = root.matrix_world @ _joint_pose(data.xanchor[joint_id], data.xaxis[joint_id])
            imported.append(joint_object)

            motion_object = bpy.data.objects.new(f"Motion: {joint_name}", None)
            motion_object.empty_display_type = "PLAIN_AXES"
            motion_object.empty_display_size = 0.02
            collection.objects.link(motion_object)
            motion_object.parent = joint_object
            motion_object.matrix_world = joint_object.matrix_world.copy()
            imported.append(motion_object)

            if joint_type in {int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)}:
                qpos_address = int(model.jnt_qposadr[joint_id])
                home_value = float(data.qpos[qpos_address])
                joint_object["qpos"] = home_value
                joint_object["home_qpos"] = home_value
                if bool(model.jnt_limited[joint_id]):
                    minimum, maximum = (float(value) for value in model.jnt_range[joint_id])
                elif joint_type == int(mujoco.mjtJoint.mjJNT_HINGE):
                    minimum, maximum = -3.141592653589793, 3.141592653589793
                else:
                    minimum, maximum = -1.0, 1.0
                joint_object.id_properties_ui("qpos").update(
                    min=minimum,
                    max=maximum,
                    soft_min=minimum,
                    soft_max=maximum,
                    description=f"MuJoCo {joint_types[joint_type]} joint position",
                )
                if joint_type == int(mujoco.mjtJoint.mjJNT_HINGE):
                    motion_object.rotation_mode = "XYZ"
                    _add_qpos_driver(motion_object, joint_object, "rotation_euler", 2, home_value)
                else:
                    _add_qpos_driver(motion_object, joint_object, "location", 2, home_value)
            parent = motion_object

        body_object = bpy.data.objects.new(f"Body: {body_name}", None)
        body_object.empty_display_type = "PLAIN_AXES"
        body_object.empty_display_size = 0.04
        body_object["mujoco_body_id"] = body_id
        collection.objects.link(body_object)
        body_object.parent = parent
        body_object.matrix_world = root.matrix_world @ _pose_matrix(data.xpos[body_id], data.xmat[body_id])
        body_objects[body_id] = body_object
        imported.append(body_object)

    mesh_cache: dict[int, bpy.types.Mesh] = {}
    material_cache: dict[tuple[float, ...], bpy.types.Material] = {}
    collision_count = 0
    for geom_id in range(model.ngeom):
        geom_group = int(model.geom_group[geom_id])
        is_collision = geom_group in {3, 4}
        if is_collision and not include_collisions:
            continue
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
        geom_type = int(model.geom_type[geom_id])
        if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
            mesh_id = int(model.geom_dataid[geom_id])
            if mesh_id < 0:
                continue
            obj = bpy.data.objects.new(f"Geom: {geom_name}", _mesh_data(model, mesh_id, mesh_cache))
            collection.objects.link(obj)
        else:
            obj = _primitive_object(model, geom_id, f"Geom: {geom_name}")
            if obj is None:
                continue
            primitive_scale = obj.scale.copy()
            _move_to_collection(obj, collection)
        obj.data.materials.append(_material(model, geom_id, material_cache))
        body_id = int(model.geom_bodyid[geom_id])
        obj.parent = body_objects.get(body_id, root)
        obj.matrix_world = root.matrix_world @ _pose_matrix(data.geom_xpos[geom_id], data.geom_xmat[geom_id])
        if geom_type != int(mujoco.mjtGeom.mjGEOM_MESH):
            obj.scale = primitive_scale
        obj["mujoco_geom_id"] = geom_id
        obj["is_collision"] = is_collision
        if is_collision:
            obj.display_type = "WIRE"
            obj.hide_render = True
            obj.color = (1.0, 0.05, 0.05, 0.35)
            collision_count += 1
        imported.append(obj)

    root["import_mode"] = "mujoco_compiled"
    root["mujoco_version"] = mujoco.__version__
    root["body_count"] = model.nbody - 1
    root["joint_count"] = model.njnt
    root["collision_count"] = collision_count
    return imported