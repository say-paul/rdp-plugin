"""Enable through Blender's restricted add-on loader, not a direct register call."""
# ruff: noqa: E402

import json
import sys
from pathlib import Path

import addon_utils
import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import sensor_visuals as visuals
from robot_arm_library.sensor_spec import CameraSensorSpec, SensorDefinition

assert Path(robot_arm_library.__file__).resolve() == ROOT / "robot_arm_library/__init__.py"
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)
definition = SensorDefinition("camera", "Camera", "rgb_camera", "snapshot.json", camera=CameraSensorSpec())
mount = visuals.create_mount(bpy.context.scene.collection, "Sensor: Existing Camera", definition)
mount["sensor_id"] = definition.sensor_id
mount["sensor_category"] = definition.category
mount["sensor_definition_json"] = json.dumps(definition.to_dict())
mount["caryam_sensor_visual_version"] = 1
mount.location = (1, 2, 3)
bpy.context.view_layer.update()
pose = mount.matrix_world.copy()

try:
    errors = []
    enabled = addon_utils.enable("robot_arm_library", default_set=False, handle_error=errors.append)
    assert enabled is robot_arm_library and not errors, errors
    assert addon_utils.check("robot_arm_library")[1]
    assert mount["caryam_sensor_visual_version"] == 1, "Registration must not touch scene data"
    assert bpy.app.timers.is_registered(visuals._upgrade_previews)
    assert bpy.app.handlers.load_post.count(visuals._upgrade_previews) == 1

    visuals._upgrade_previews()
    assert mount["caryam_sensor_visual_version"] == visuals.PREVIEW_VERSION
    assert max(abs(mount.matrix_world[r][c] - pose[r][c]) for r in range(4) for c in range(4)) < 1e-5
    addon_utils.disable("robot_arm_library", default_set=False)
    assert not bpy.app.timers.is_registered(visuals._upgrade_previews)
    assert visuals._upgrade_previews not in bpy.app.handlers.load_post
    assert not hasattr(bpy.types.Scene, "caryam_sensor_axes")

    # Disabling before the timer fires must leave the saved preview untouched.
    mount["caryam_sensor_visual_version"] = 1
    enabled = addon_utils.enable("robot_arm_library", default_set=False, handle_error=errors.append)
    assert enabled is robot_arm_library and not errors, errors
    assert bpy.app.timers.is_registered(visuals._upgrade_previews)
    addon_utils.disable("robot_arm_library", default_set=False)
    assert not bpy.app.timers.is_registered(visuals._upgrade_previews)
    assert mount["caryam_sensor_visual_version"] == 1
    print("Restricted add-on enable, deferred preview migration and disable cleanup passed")
finally:
    if addon_utils.check("robot_arm_library")[1]:
        addon_utils.disable("robot_arm_library", default_set=False)
    else:
        robot_arm_library.unregister()
