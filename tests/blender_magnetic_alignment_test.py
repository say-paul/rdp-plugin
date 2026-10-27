"""Destination-local magnetic axes, explicit overrides and cursor-centered placement."""
# ruff: noqa: E402

import math
import sys
import tempfile
from pathlib import Path
from types import MethodType, SimpleNamespace
from typing import Any, cast

import bpy
from mathutils import Euler, Matrix, Quaternion, Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import mounting


def shortcuts():
    config = bpy.context.window_manager.keyconfigs.addon
    if config is None:
        return []
    return [item for keymap in config.keymaps for item in keymap.keymap_items if item.idname == "robot_library.magnetic_move"]


def close(actual, expected):
    assert max(abs(actual[r][c] - expected[r][c]) for r in range(4) for c in range(4)) < 1e-5


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    bpy.ops.mesh.primitive_cube_add(size=.1, location=(2, 0, 0))
    target_body = bpy.context.object
    target_rotation = Euler((.3, -.4, .7)).to_quaternion()
    target = mounting.create_magnetic_point(
        bpy.context, target_body, Matrix.LocRotScale(Vector((2, 0, .05)), target_rotation, Vector((1, 1, 1))),
    )
    source_rotation = target_rotation @ Quaternion((0, 1, 0), math.radians(80)) @ Quaternion((0, 0, 1), math.radians(17))
    candidate = mounting.nearest_snap(target.matrix_world.translation, source_rotation, .1, [], [target], "AXIS")
    assert candidate is not None
    normal = candidate.rotation @ Vector((0, 0, 1))
    assert normal.dot(target_rotation @ Vector((1, 0, 0))) > .9999, "Automatic snapping did not choose the nearest destination axis"
    assert scene.robot_magnet_alignment == "AXIS"
    for number, mode in ((0, "OPPOSE"), (1, "MATCH"), (2, "POSITION")):
        assert scene.bl_rna.properties["robot_magnet_alignment"].enum_items[mode].value == number
        scene.robot_magnet_alignment = mode
        assert scene.robot_magnet_alignment == mode
    scene.robot_magnet_alignment = "AXIS"
    for name, axis in (("X", (1, 0, 0)), ("NEG_X", (-1, 0, 0)), ("Y", (0, 1, 0)), ("NEG_Y", (0, -1, 0)), ("Z", (0, 0, 1)), ("NEG_Z", (0, 0, -1))):
        target.caryam_magnet_axis = name
        candidate = mounting.nearest_snap(target.matrix_world.translation, source_rotation, .1, [], [target], "AXIS")
        assert candidate is not None
        assert (candidate.rotation @ Vector((0, 0, 1))).dot(target_rotation @ Vector(axis)) > .9999, name
        relative = (target_rotation.inverted() @ candidate.rotation).to_matrix()
        assert all(max(abs(relative[row][column]) for row in range(3)) > .9999 for column in range(3)), name
    target.caryam_magnet_axis = "AUTO"
    first = mounting.nearest_snap(target.matrix_world.translation, source_rotation, .1, [], [target], "AXIS")
    again = mounting.nearest_snap(target.matrix_world.translation, -source_rotation, .1, [], [target], "AXIS")
    assert first is not None and again is not None
    assert abs(first.rotation.dot(again.rotation)) > .9999

    bpy.ops.mesh.primitive_cube_add(size=.1, location=(0, 0, 0))
    moving = bpy.context.object
    source = mounting.create_magnetic_point(bpy.context, moving, Matrix.Translation((0, 0, .05)))
    region = SimpleNamespace(type="WINDOW", x=0, y=0, width=200, height=200)
    context = SimpleNamespace(
        active_object=source, scene=scene, view_layer=bpy.context.view_layer,
        evaluated_depsgraph_get=bpy.context.evaluated_depsgraph_get,
        area=SimpleNamespace(type="VIEW_3D", regions=[region], tag_redraw=lambda: None),
        region=SimpleNamespace(type="UI"), space_data=SimpleNamespace(region_3d=None),
        workspace=SimpleNamespace(status_text_set=lambda _text: None),
        window_manager=SimpleNamespace(modal_handler_add=lambda _state: None),
    )
    state = cast(Any, SimpleNamespace(magnetic_points=True, report=lambda *_args: None))
    for name in ("_prepare", "_restore", "_status"):
        setattr(state, name, MethodType(getattr(mounting.ROBOT_OT_magnetic_move, name), state))
    state._mouse_position = lambda _event: Vector((400, 0, 0))
    original = moving.matrix_world.copy()
    assert mounting.ROBOT_OT_magnetic_move.invoke(state, context, SimpleNamespace(mouse_x=250, mouse_y=50)) == {"RUNNING_MODAL"}
    assert state.mouse_offset.length < 1e-8, "The sidebar injected a cursor offset"
    state._mouse_position = lambda _event: target.matrix_world.translation
    assert mounting.ROBOT_OT_magnetic_move.modal(state, context, SimpleNamespace(type="MOUSEMOVE", mouse_x=250, mouse_y=50)) == {"RUNNING_MODAL"}
    close(moving.matrix_world, original)
    event = SimpleNamespace(type="MOUSEMOVE", mouse_x=50, mouse_y=50)
    assert mounting.ROBOT_OT_magnetic_move.modal(state, context, event) == {"RUNNING_MODAL"}
    assert (source.matrix_world.translation - target.matrix_world.translation).length < 1e-5
    assert mounting.ROBOT_OT_magnetic_move.modal(state, context, SimpleNamespace(type="ESC")) == {"CANCELLED"}
    close(moving.matrix_world, original)

    if bpy.context.window_manager.keyconfigs.addon is not None:
        items = shortcuts()
        assert len(items) == 1
        assert items[0].type == "M" and items[0].ctrl and items[0].shift and not items[0].alt
        robot_arm_library.register()
        assert len(shortcuts()) == 1, "Repeated registration duplicated the shortcut"
    target.caryam_magnet_axis = "NEG_Y"
    scene.robot_magnet_alignment = "MATCH"
    name = target.name
    with tempfile.TemporaryDirectory() as directory:
        blend = str(Path(directory) / "alignment.blend")
        bpy.ops.wm.save_as_mainfile(filepath=blend)
        bpy.ops.wm.open_mainfile(filepath=blend)
        assert bpy.data.objects[name].caryam_magnet_axis == "NEG_Y"
        assert bpy.context.scene.robot_magnet_alignment == "MATCH"
    print("Nearest local magnetic axes, deterministic overrides, sidebar offset, persistence and shortcut lifecycle passed")
finally:
    robot_arm_library.unregister()
    assert not shortcuts()
