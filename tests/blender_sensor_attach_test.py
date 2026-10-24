"""Attach a registry sensor to an imported robot and export it."""

import json
import sys
import tempfile
from pathlib import Path
from xml.etree import ElementTree

import bpy
from mathutils import Vector

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as addon

robot_arm_library.register()
try:
    with tempfile.TemporaryDirectory() as directory:
        root_dir = Path(directory)
        model = root_dir / "arm.xml"
        model.write_text(
            "<mujoco><worldbody><body name='arm'><geom type='sphere' size='0.05'/>"
            "<body name='wrist' pos='0 0 0.2'><joint name='hinge' type='hinge'/>"
            "<geom type='sphere' size='0.03'/></body></body></worldbody></mujoco>",
            encoding="utf-8",
        )
        robot_dir = root_dir / "robots"
        robot_dir.mkdir()
        (robot_dir / "robot.json").write_text(json.dumps({"name": "Test Arm", "model_path": str(model)}), encoding="utf-8")
        sensor_dir = root_dir / "sensors"
        sensor_dir.mkdir()
        (sensor_dir / "registry.json").write_text(json.dumps({"sensors": {
            "lidar-1": {"category": "lidar", "name": "Front Lidar", "lidar": {"horizontal_samples": 6}},
            "depth-1": {"category": "depth_camera", "name": "Chest Depth"},
        }}), encoding="utf-8")
        scene = bpy.context.scene
        scene.robot_setup_path = str(robot_dir)
        extra = scene.robot_asset_sources.add()
        extra.path = str(sensor_dir)
        extra.kind = "sensor"
        assert bpy.ops.robot_library.refresh() == {"FINISHED"}
        root = addon._place_robot(scene.robot_library[0], Vector((0, 0, 0)))
        wrist = bpy.data.objects.get("Body: wrist")
        if wrist is None:
            wrist = bpy.data.objects.new("Body: wrist", None)
            scene.collection.objects.link(wrist)
            wrist.parent = root
        lidar_index = next(index for index, item in enumerate(scene.sensor_library) if item.category == "lidar")
        depth_index = next(index for index, item in enumerate(scene.sensor_library) if item.category == "depth_camera")
        bpy.context.view_layer.objects.active = wrist
        assert bpy.ops.robot_library.attach_sensor(sensor_index=lidar_index) == {"FINISHED"}
        bpy.context.view_layer.objects.active = root
        assert bpy.ops.robot_library.attach_sensor(sensor_index=depth_index) == {"FINISHED"}
        mounts = addon._sensor_mounts(root)
        assert {mount.sensor.category for mount in mounts} == {"lidar", "depth_camera"}
        assert next(mount for mount in mounts if mount.sensor.category == "lidar").body_name == "wrist"
        world_path, manifest_path, _ = addon._export_world_file(str(root_dir / "world.xml"))
        world = ElementTree.parse(world_path).getroot()
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        prefix = manifest["robots"][0]["prefix"]
        generated = root_dir / "world_assets" / f"{prefix}sensorized.xml"
        assert generated.is_file(), generated
        sensorized = ElementTree.parse(generated).getroot()
        assert len(sensorized.findall(".//sensor/rangefinder")) == 6
        assert sensorized.find(".//body[@name='wrist']/body/site") is not None
        assert sensorized.find(".//body[@name='arm']/body/camera") is not None
        assert len(manifest["robots"][0]["sensors"]) == 2
        try:
            import mujoco
        except ImportError:
            pass
        else:
            compiled = mujoco.MjModel.from_xml_path(str(world_path))
            wrist_id = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}wrist")
            mount_id = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}front_lidar")
            assert int(compiled.body_parentid[mount_id]) == wrist_id
            base_id = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}arm")
            camera_mount_id = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}chest_depth")
            assert int(compiled.body_parentid[camera_mount_id]) == base_id
    print("Sensor attach and export test passed")
finally:
    robot_arm_library.unregister()
