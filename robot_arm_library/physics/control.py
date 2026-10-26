"""Temporary effort-limited joint servos; never overwrite simulated qpos."""

from __future__ import annotations

import math


def normalize_joint_target(value, limits, name):
    if not math.isfinite(value):
        raise ValueError(f"Joint target must be finite: {name}")
    if limits is not None:
        minimum, maximum = limits
        boundary = minimum if value < minimum else (maximum if value > maximum else None)
        if boundary is not None:
            # Blender's numeric widgets can round a valid endpoint to float32.
            if not math.isclose(value, boundary, rel_tol=1e-7, abs_tol=1e-9):
                raise ValueError(f"Joint target is outside limits: {name}: {value!r} not in [{minimum!r}, {maximum!r}]")
            value = boundary
    return float(value)


class JointServos:
    def __init__(self, mujoco, model, data, manifest, *, kp=40.0, kd=4.0, max_effort=20.0):
        if not all(math.isfinite(v) for v in (kp, kd, max_effort)) or kp <= 0 or kd < 0 or max_effort <= 0:
            raise ValueError("Servo stiffness/effort must be positive and damping nonnegative")
        import numpy as np
        self.np = np
        self.mujoco = mujoco
        self._target_order = ()
        self.kp, self.kd = kp, kd
        self.model = model
        self.data = data
        self.targets = {}
        self.joints = {}
        self.metadata = {}
        self.original_gains = model.actuator_gainprm.copy()
        self.original_biases = model.actuator_biasprm.copy()
        self.original_forces = data.qfrc_applied.copy()
        self.direct = {}
        blocked = set()
        requested = {robot["prefix"] + name: robot["prefix"] for robot in manifest["robots"] for name in robot["qpos"]}
        for actuator in range(model.nu):
            transmission = int(model.actuator_trntype[actuator])
            if transmission in {0, 1}:
                joint = int(model.actuator_trnid[actuator, 0])
                self.direct.setdefault(joint, []).append(actuator)
                if int(model.actuator_gaintype[actuator]) != 0 or int(model.actuator_biastype[actuator]) not in {0, 1}:
                    blocked.add(joint)
                if hasattr(model, "actuator_plugin") and model.actuator_plugin[actuator] >= 0:
                    blocked.add(joint)
                continue
            if transmission == int(mujoco.mjtTrn.mjTRN_TENDON):
                tendon = int(model.actuator_trnid[actuator, 0])
                start = int(model.tendon_adr[tendon])
                wraps = range(start, start + int(model.tendon_num[tendon]))
                if all(int(model.wrap_type[wrap]) == int(mujoco.mjtWrap.mjWRAP_JOINT) for wrap in wraps):
                    blocked.update(int(model.wrap_objid[wrap]) for wrap in wraps)
                    continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator) or ""
            owners = [prefix for prefix in set(requested.values()) if name.startswith(prefix)]
            owner = max(owners, key=len) if owners else ""
            blocked.update(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                           for joint_name, prefix in requested.items() if not owner or prefix == owner)
        for name in requested:
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint < 0 or int(model.jnt_type[joint]) not in {2, 3}:
                self.metadata[name] = {"qpos": 0.0, "range": None, "available": False,
                                       "effort_range": [0.0, 0.0], "reason": "No named scalar MJCF joint; reimport with named joints"}
                continue
            address = int(model.jnt_qposadr[joint])
            dof = int(model.jnt_dofadr[joint])
            limits = list(map(float, model.jnt_range[joint])) if model.jnt_limited[joint] else None
            low, high = -max_effort, max_effort
            if hasattr(model, "jnt_actfrclimited") and model.jnt_actfrclimited[joint]:
                low = max(low, float(model.jnt_actfrcrange[joint, 0]))
                high = min(high, float(model.jnt_actfrcrange[joint, 1]))
            actuators = self.direct.get(joint, [])
            if actuators and all(model.actuator_forcelimited[a] for a in actuators):
                ranges = [sorted(float(value) * float(model.actuator_gear[a, 0])
                                 for value in model.actuator_forcerange[a]) for a in actuators]
                low = max(low, sum(values[0] for values in ranges))
                high = min(high, sum(values[1] for values in ranges))
            available = joint not in blocked and low <= high and (low != 0 or high != 0)
            self.joints[name] = (joint, address, dof, low, high)
            self.metadata[name] = {"qpos": float(data.qpos[address]), "range": limits, "available": available,
                                   "effort_range": [low, high],
                                   "reason": "" if available else "Coupled/unsupported actuator or no available joint effort"}

    def set_targets(self, targets):
        if not targets:
            raise ValueError("Expected at least one joint target")
        normalized = {}
        for name, value in targets.items():
            info = self.metadata.get(name)
            if info is None or not info["available"]:
                raise ValueError(f"Joint cannot be servo-controlled: {name}")
            normalized[name] = normalize_joint_target(value, info["range"], name)
        for name, value in normalized.items():
            joint = self.joints[name][0]
            for actuator in self.direct.get(joint, []):
                # Replace only this joint's direct actuators while the temporary servo owns it.
                self.model.actuator_gainprm[actuator] = 0
                self.model.actuator_biasprm[actuator] = 0
            self.targets[name] = float(value)

    def step(self):
        order = tuple(self.targets)
        if not order:
            return
        if order != self._target_order:
            self._target_order = order
            self._rhs = self.np.zeros((len(order), self.model.nv))
            self._inverse = self.np.empty_like(self._rhs)
            for row, name in enumerate(order):
                self._rhs[row, self.joints[name][2]] = 1
        self.mujoco.mj_solveM(self.model, self.data, self._inverse, self._rhs)
        dt = float(self.model.opt.timestep)
        for row, name in enumerate(order):
            _, address, dof, low, high = self.joints[name]
            velocity = float(self.data.qvel[dof])
            # Implicit PD scaling avoids explicit damping instability on tiny imported links.
            effort = (self.kp * (self.targets[name] - float(self.data.qpos[address]) - dt * velocity) - self.kd * velocity)
            effort /= 1 + (self.kd * dt + self.kp * dt * dt) * float(self._inverse[row, dof])
            self.data.qfrc_applied[dof] = self.original_forces[dof] + max(low, min(high, effort))

    def reset(self):
        self.targets.clear()
        self.model.actuator_gainprm[:] = self.original_gains
        self.model.actuator_biasprm[:] = self.original_biases
        self.data.qfrc_applied[:] = self.original_forces

    def positions(self):
        return {name: float(self.data.qpos[value[1]]) for name, value in self.joints.items()}
