"""Reduce imported Blender mesh data while preserving the source asset files."""

from __future__ import annotations

from collections.abc import Iterable

import bpy


SOURCE_TRIANGLE_COUNT_PROPERTY = "robot_source_triangle_count"


def mesh_triangle_count(mesh: bpy.types.Mesh) -> int:
    mesh.calc_loop_triangles()
    return len(mesh.loop_triangles)


def simplify_mesh_data(
    mesh: bpy.types.Mesh,
    detail: float,
    collection: bpy.types.Collection,
) -> bpy.types.Mesh:
    """Return mesh data reduced to the requested fraction of source triangles."""
    source_triangle_count = mesh_triangle_count(mesh)
    detail = max(0.01, min(1.0, detail))
    if detail >= 1.0 or source_triangle_count == 0:
        mesh[SOURCE_TRIANGLE_COUNT_PROPERTY] = source_triangle_count
        return mesh

    temporary = bpy.data.objects.new("Robot Mesh Simplifier", mesh)
    collection.objects.link(temporary)
    simplified: bpy.types.Mesh | None = None
    try:
        modifier = temporary.modifiers.new("Mesh Detail", "DECIMATE")
        modifier.decimate_type = "COLLAPSE"
        modifier.ratio = detail
        bpy.context.view_layer.update()
        dependencies = bpy.context.evaluated_depsgraph_get()
        simplified = bpy.data.meshes.new_from_object(
            temporary.evaluated_get(dependencies),
            depsgraph=dependencies,
        )
    except RuntimeError:
        simplified = None
    finally:
        bpy.data.objects.remove(temporary, do_unlink=True)

    if simplified is None:
        mesh[SOURCE_TRIANGLE_COUNT_PROPERTY] = source_triangle_count
        return mesh
    if mesh.users == 0:
        bpy.data.meshes.remove(mesh)
    simplified[SOURCE_TRIANGLE_COUNT_PROPERTY] = source_triangle_count
    return simplified


def simplify_mesh_objects(
    objects: Iterable[bpy.types.Object],
    detail: float,
    collection: bpy.types.Collection,
) -> tuple[int, int]:
    """Simplify each distinct mesh data block and return source and result totals."""
    simplified_by_source: dict[int, bpy.types.Mesh] = {}
    source_triangle_counts: dict[int, int] = {}
    source_meshes: dict[int, bpy.types.Mesh] = {}
    source_total = 0
    result_total = 0
    for obj in objects:
        if obj.type != "MESH":
            continue
        mesh = obj.data
        source_key = mesh.as_pointer()
        source_meshes[source_key] = mesh
        simplified = simplified_by_source.get(source_key)
        if simplified is None:
            simplified = simplify_mesh_data(mesh, detail, collection)
            simplified_by_source[source_key] = simplified
            source_triangle_counts[source_key] = int(simplified[SOURCE_TRIANGLE_COUNT_PROPERTY])
        obj.data = simplified
        source_total += source_triangle_counts[source_key]
        result_total += mesh_triangle_count(simplified)
    for mesh in source_meshes.values():
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)
    return source_total, result_total