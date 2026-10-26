"""Shared standalone initialization and reproducible MuJoCo integration state."""

from __future__ import annotations


def multiply_quaternions(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw)


def initialize(mujoco, model, manifest):
    data = mujoco.MjData(model)
    for robot in manifest["robots"]:
        prefix = robot["prefix"]
        joint_ids = []
        for body_id in range(1, model.nbody):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if name and name.startswith(prefix):
                start = int(model.body_jntadr[body_id])
                joint_ids.extend(range(start, start + int(model.body_jntnum[body_id])))
        home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, prefix + "home")
        if home >= 0:
            for joint_id in joint_ids:
                start = int(model.jnt_qposadr[joint_id])
                end = int(model.jnt_qposadr[joint_id + 1]) if joint_id + 1 < model.njnt else model.nq
                data.qpos[start:end] = model.key_qpos[home, start:end]
        for name, value in robot["qpos"].items():
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, prefix + name)
            if joint_id >= 0 and int(model.jnt_type[joint_id]) in {2, 3}:
                data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)
        if robot.get("free_base"):
            free = next((j for j in joint_ids if int(model.jnt_type[j]) == 0), -1)
            if free >= 0:
                start = int(model.jnt_qposadr[free])
                rotation = robot["quaternion"]
                inverse = (rotation[0], -rotation[1], -rotation[2], -rotation[3])
                position = multiply_quaternions(multiply_quaternions(rotation, (0.0, *data.qpos[start:start + 3])), inverse)
                data.qpos[start:start + 3] = [position[i + 1] + float(robot["position"][i]) for i in range(3)]
                data.qpos[start + 3:start + 7] = multiply_quaternions(rotation, data.qpos[start + 3:start + 7])
    mujoco.mj_forward(model, data)
    return data


class ResetState:
    def __init__(self, mujoco, model, data):
        import numpy as np
        self.spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self.value = np.empty(mujoco.mj_stateSize(model, self.spec))
        mujoco.mj_getState(model, data, self.value, self.spec)

    def restore(self, mujoco, model, data):
        mujoco.mj_resetData(model, data)
        mujoco.mj_setState(model, data, self.value, self.spec)
        mujoco.mj_forward(model, data)
