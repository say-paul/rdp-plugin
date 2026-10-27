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
from runpy import run_path
initialize = run_path(str(ROOT / "caryam_physics_runtime.py"))["initialize"]
data = initialize(mujoco, model, manifest)

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
