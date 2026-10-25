"""Render a Caryam world export with MuJoCo."""

from pathlib import Path
import json
import sys
import time

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
manifest = json.loads((ROOT / 'caryam_world.json').read_text(encoding="utf-8"))
model = mujoco.MjModel.from_xml_path(str(ROOT / 'caryam_world.xml'))
data = mujoco.MjData(model)


def robot_joint_ids(prefix):
    joint_ids = set()
    for body_id in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if not body_name or not body_name.startswith(prefix):
            continue
        first_joint = int(model.body_jntadr[body_id])
        joint_count = int(model.body_jntnum[body_id])
        joint_ids.update(range(first_joint, first_joint + joint_count))
    return sorted(joint_ids)


def joint_qpos_width(joint_id):
    start = int(model.jnt_qposadr[joint_id])
    end = int(model.jnt_qposadr[joint_id + 1]) if joint_id + 1 < model.njnt else model.nq
    return start, end


def multiply_quaternions(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


for robot in manifest["robots"]:
    prefix = robot["prefix"]
    joint_ids = robot_joint_ids(prefix)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, prefix + "home")
    if home_key >= 0:
        for joint_id in joint_ids:
            start, end = joint_qpos_width(joint_id)
            data.qpos[start:end] = model.key_qpos[home_key, start:end]
    for joint_name, value in robot["qpos"].items():
        full_name = prefix + joint_name
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, full_name)
        if joint_id < 0:
            continue
        qpos_address = int(model.jnt_qposadr[joint_id])
        if int(model.jnt_type[joint_id]) in {2, 3}:
            data.qpos[qpos_address] = float(value)
    if robot.get("free_base"):
        free_joint = next(
            (joint_id for joint_id in joint_ids if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE)),
            -1,
        )
        if free_joint >= 0:
            qpos_address = int(model.jnt_qposadr[free_joint])
            rotation = robot["quaternion"]
            inverse_rotation = (rotation[0], -rotation[1], -rotation[2], -rotation[3])
            local_position = (0.0, *data.qpos[qpos_address:qpos_address + 3])
            rotated_position = multiply_quaternions(
                multiply_quaternions(rotation, local_position), inverse_rotation,
            )
            for index, value in enumerate(robot["position"]):
                data.qpos[qpos_address + index] = rotated_position[index + 1] + float(value)
            orientation = multiply_quaternions(rotation, data.qpos[qpos_address + 3:qpos_address + 7])
            data.qpos[qpos_address + 3:qpos_address + 7] = orientation

mujoco.mj_forward(model, data)

for robot in manifest["robots"]:
    for mount in robot.get("sensors", []):
        mount_name = robot["prefix"] + mount["mount_name"]
        definition = mount["sensor"]
        if definition["category"] == "lidar":
            spec = definition.get("lidar") or {}
            channels = int(spec.get("channels", 1))
            samples = int(spec.get("horizontal_samples", 360))
            readings = []
            for channel in range(channels):
                row = []
                for index in range(samples):
                    suffix = f"{channel:02d}_{index:02d}" if channels > 1 else f"{index:02d}"
                    name = f"{mount_name}_rf_{suffix}"
                    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
                    if sensor_id < 0:
                        row.append(float("nan"))
                    else:
                        row.append(float(data.sensordata[int(model.sensor_adr[sensor_id])]))
                readings.append(row)
            print(f"{mount_name} lidar scan: {readings[0] if channels == 1 else readings}")
        elif definition["category"] == "depth_camera":
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, mount_name)
            if camera_id < 0:
                continue
            spec = definition.get("camera") or {}
            height = int(spec.get("resolution_height", 480))
            width = int(spec.get("resolution_width", 640))
            model.vis.global_.offheight = max(model.vis.global_.offheight, height)
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
            renderer = mujoco.Renderer(
                model,
                height=height,
                width=width,
            )
            try:
                renderer.enable_depth_rendering()
                renderer.update_scene(data, camera=camera_id)
                import numpy as np

                depth_path = ROOT / f"{mount_name}_depth.npy"
                np.save(depth_path, renderer.render())
                print(f"{mount_name} depth frame saved to {depth_path}")
            finally:
                renderer.close()

if "--sensors-only" in sys.argv:
    raise SystemExit(0)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = (0.0, 2.0, 1.2)
    viewer.cam.distance = 7.0
    viewer.cam.azimuth = 135.0
    viewer.cam.elevation = -20.0
    next_step_time = time.perf_counter()
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        next_step_time += model.opt.timestep
        remaining = next_step_time - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        else:
            next_step_time = time.perf_counter()
