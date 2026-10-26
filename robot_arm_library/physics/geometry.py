"""Detect actual local collision-mesh edits, not dependency-graph refresh flags."""

from __future__ import annotations

from array import array
import hashlib

import bpy


def _mesh_signature(obj, depsgraph):
    if obj.type != "MESH":
        raise RuntimeError(f"Geometry changed: {obj.name}; Start a fresh physics snapshot")
    evaluated = obj.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh()
    try:
        mesh.calc_loop_triangles()
        coordinates = array("f", [0.0]) * (3 * len(mesh.vertices))
        indices = array("i", [0]) * (3 * len(mesh.loop_triangles))
        if coordinates:
            mesh.vertices.foreach_get("co", coordinates)
        if indices:
            mesh.loop_triangles.foreach_get("vertices", indices)
        digest = hashlib.blake2b(digest_size=16)
        digest.update(coordinates)
        digest.update(indices)
        return len(mesh.vertices), len(mesh.loop_triangles), digest.digest()
    finally:
        evaluated.to_mesh_clear()


class GeometrySnapshot:
    def __init__(self, objects, depsgraph):
        self.signatures = {obj: _mesh_signature(obj, depsgraph) for obj in objects if obj.type == "MESH"}
        self.data_users = {}
        for obj in self.signatures:
            self.data_users.setdefault(obj.data, set()).add(obj)
            if obj.data.shape_keys is not None:
                self.data_users.setdefault(obj.data.shape_keys, set()).add(obj)
        self.dirty = set()

    def mark_updates(self, depsgraph):
        # Geometry flags also mean "reevaluated" after custom-property/viewport refreshes.
        for update in depsgraph.updates:
            if not update.is_updated_geometry:
                continue
            original = getattr(update.id, "original", update.id)
            if isinstance(original, bpy.types.Object):
                if original in self.signatures:
                    self.dirty.add(original)
            else:
                self.dirty.update(self.data_users.get(original, ()))

    def validate(self, depsgraph):
        dirty, self.dirty = self.dirty, set()
        for obj in dirty:
            if _mesh_signature(obj, depsgraph) != self.signatures[obj]:
                raise RuntimeError(f"Geometry changed: {obj.name}; Start a fresh physics snapshot")
