"""Generic sensor housing, mounting frames, export and legacy-scene regression."""
# ruff: noqa: E402

import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import robot_arm_library
from robot_arm_library import blender_addon as addon, sensor_visuals as visuals
from robot_arm_library.physics import blender as live
from robot_arm_library.sensor_spec import CameraSensorSpec, LidarSensorSpec, SensorDefinition

assert Path(addon.__file__).resolve() == ROOT / "robot_arm_library/blender_addon.py", "Run against the source checkout"


def matrix_close(left, right):
    assert max(abs(left[r][c] - right[r][c]) for r in range(4) for c in range(4)) < 1e-5


def pump(predicate):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        live._tick()
        live._reap_clients()
        if predicate():
            return
        assert live.active(), live.status()
        time.sleep(0.01)
    raise AssertionError(live.status())


class Layout:
    enabled = True

    def __init__(self):
        self.labels = []
        self.properties = []

    def box(self):
        return self

    def column(self):
        return self

    def label(self, *, text, **kwargs):
        self.labels.append(text)

    def prop(self, obj, name, **kwargs):
        self.properties.append((obj, name))

    def operator(self, *args, **kwargs):
        return type("Operator", (), {})()


robot_arm_library.register()
try:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    with tempfile.TemporaryDirectory(prefix="sensor module test ") as directory:
        path = Path(directory)
        model = path / "arm.xml"
        model.write_text(
            '<mujoco><worldbody><body name="base"><geom size=".03"/>'
            '<body name="wrist" pos="0 0 .2"><geom size=".02"/></body></body></worldbody></mujoco>', encoding="utf-8",
        )
        item = bpy.context.scene.robot_library.add()
        item.name = "Preview Arm"
        item.robot_id = "preview_arm"
        item.model_path = str(model)
        item.source_path = str(model)
        root = addon._place_robot(item, Vector((1, 2, 3)))
        assert root.get("import_mode") == "mujoco_compiled", dict(root.items())
        wrist = next(obj for obj in root.children_recursive if obj.get("mujoco_body_name") == "wrist")
        camera = SensorDefinition("camera", "Camera", "rgbd_camera", "snapshot.json", camera=CameraSensorSpec())
        lidar = SensorDefinition("lidar", "Lidar", "lidar", "snapshot.json", lidar=LidarSensorSpec(horizontal_fov=120, horizontal_samples=3))

        # The camera frustum uses the actual fovy and rendering aspect, not horizontal aperture.
        lines = visuals.guide_lines(camera)
        corner = lines[0][-1]
        assert corner[2] == -visuals.GUIDE_LENGTH
        assert abs(corner[0] / corner[1] - 640 / 480) < 1e-8
        expected_height = visuals.GUIDE_LENGTH * math.tan(math.radians(camera.camera.vertical_fov_degrees()) / 2)
        assert abs(abs(corner[1]) - expected_height) < 1e-8
        fan = visuals.guide_lines(lidar)
        assert len(fan) == 3 and all(abs(point[2]) < 1e-8 for line in fan for point in line)
        assert abs(math.atan2(fan[0][0][1], fan[0][0][0]) + math.pi / 3) < 1e-8
        assert abs(math.atan2(fan[0][-1][1], fan[0][-1][0]) - math.pi / 3) < 1e-8

        mount = visuals.create_mount(addon._robot_collection(), "Sensor: Camera", camera)
        mount["sensor_id"] = camera.sensor_id
        mount["sensor_category"] = camera.category
        mount["sensor_definition_json"] = json.dumps(camera.to_dict())
        mount.parent = wrist
        mount.matrix_parent_inverse = Matrix.Identity(4)
        mount.location = (0.04, 0.02, 0.03)
        mount.rotation_euler = (0.2, -0.4, 0.6)
        bpy.context.view_layer.update()
        saved = mount.matrix_world.copy()
        saved_definition = mount["sensor_definition_json"]
        for _ in range(2):
            visuals.update_preview(mount, camera)
            assert not mount.children
            matrix_close(mount.matrix_world, saved)
        assert mount["sensor_definition_json"] == saved_definition
        assert mount.type == "MESH" and len(mount.data.vertices) > 100
        assert not mount.show_in_front
        # Aim directions are overlay lines, not pickable housing vertices.
        camera_aim = list(visuals.overlay_lines(camera, axes=False, fov=False))[0][1]
        lidar_aim = list(visuals.overlay_lines(lidar, axes=False, fov=False))[0][1]
        assert min(point.z for point in camera_aim) < -0.12
        assert max(point.x for point in lidar_aim) > 0.12
        assert min(point.co.z for point in mount.data.vertices) > -.01
        lidar_mesh = visuals.module_mesh("lidar")
        assert max(point.co.x for point in lidar_mesh.vertices) < .05
        bpy.data.meshes.remove(lidar_mesh)
        exported = addon._sensor_mounts(root)[0]
        assert exported.body_name == "wrist"
        assert max(abs(a - b) for a, b in zip(exported.position, mount.location)) < 1e-5

        # Existing sphere empties retain object identity, children and transforms when upgraded.
        legacy = bpy.data.objects.new("Sensor: Legacy Lidar", None)
        addon._robot_collection().objects.link(legacy)
        legacy.parent = wrist
        legacy.location = (0.1, 0.2, 0.3)
        legacy.rotation_mode = "QUATERNION"
        legacy.rotation_quaternion = (0.9238795, 0, 0, 0.3826834)
        legacy["sensor_id"] = lidar.sensor_id
        legacy["sensor_category"] = lidar.category
        legacy["sensor_definition_json"] = json.dumps(lidar.to_dict())
        child = bpy.data.objects.new("Existing user child", None)
        addon._robot_collection().objects.link(child)
        child.parent = legacy
        bpy.context.view_layer.update()
        before = legacy.matrix_world.copy()
        bpy.context.view_layer.objects.active = legacy
        assert bpy.ops.robot_library.sensor_preview() == {"FINISHED"}
        assert legacy.type == "EMPTY" and child.parent is legacy
        matrix_close(before, legacy.matrix_world)
        assert len([obj for obj in legacy.children if obj.get(visuals.PREVIEW_KEY)]) == 1
        assert bpy.ops.robot_library.sensor_preview() == {"FINISHED"}
        assert len(legacy.children) == 2
        housing = next(obj for obj in legacy.children if obj.type == "MESH")
        assert visuals.selected_mount(housing) is legacy
        assert not housing.show_in_front
        assert bpy.ops.robot_library.select_sensor_mount(mount_name=mount.name) == {"FINISHED"}
        assert bpy.context.active_object is mount
        assert bpy.context.selected_objects == [mount]
        layout = Layout()
        addon._draw_sensor_mount(layout, bpy.context)
        assert "Link: wrist" in layout.labels and "Camera forward: -Z; up: +Y" in layout.labels
        assert (mount, "location") in layout.properties and (mount, "rotation_euler") in layout.properties
        bpy.context.view_layer.objects.active = housing
        layout = Layout()
        addon._draw_sensor_mount(layout, bpy.context)
        assert (legacy, "rotation_quaternion") in layout.properties
        assert (housing, "show_in_front") in layout.properties
        assert "Lidar forward: +X; scan plane: XY" in layout.labels

        # Preview meshes must not become scene collisions, even if render visibility is changed.
        mount.hide_render = False
        housing.hide_render = False
        assert addon._world_scene_meshes(path / "world.xml") == []
        housing.parent = None
        assert addon._world_scene_meshes(path / "world.xml") == []
        housing.parent = legacy
        mount.hide_render = True
        housing.hide_render = True
        output, _, _ = addon._export_world_file(str(path / "world.xml"))
        import mujoco
        compiled = mujoco.MjModel.from_xml_path(str(output))
        assert compiled.ncam == 1 and compiled.nsensor == 3
        assert compiled.ngeom == 3, compiled.ngeom  # Ground plus two original robot geoms only.

        # Display children follow body poses and remain unchanged across live Pause/Stop.
        bpy.context.scene.robot_mujoco_python = os.environ.get("CARYAM_TEST_MUJOCO_PYTHON", str(ROOT / ".venv/bin/python"))
        before_live = mount.matrix_world.copy()
        assert bpy.ops.robot_library.live_physics(action="START") == {"FINISHED"}
        pump(lambda: live.state() == "RUNNING")
        assert bpy.ops.robot_library.live_physics(action="PAUSE") == {"FINISHED"}
        pump(lambda: live.state() == "PAUSED")
        assert not bpy.ops.robot_library.sensor_preview.poll()
        live.stop(wait=True)
        matrix_close(before_live, mount.matrix_world)
        assert not live.active()
        print("Sensor module shapes, directions, mount controls, legacy upgrade, export and live physics passed")
finally:
    live.stop(wait=True)
    robot_arm_library.unregister()
