"""Versioned bounded JSON-lines messages for the local simulation pipes."""

from __future__ import annotations

import json
import math

VERSION = 1
MAX_MESSAGE = 8 * 1024 * 1024
MAX_BODIES = 20000
COMMANDS = {"start", "pause", "resume", "reset", "stop", "targets"}


def _joint_values(values):
    if not isinstance(values, dict) or len(values) > MAX_BODIES or any(
        not isinstance(name, str) or not name or not isinstance(value, (int, float)) or not math.isfinite(value)
        for name, value in values.items()
    ):
        raise ValueError("Invalid joint target/state values")


def encode(message):
    raw = json.dumps(message, allow_nan=False, separators=(",", ":")).encode("utf-8") + b"\n"
    if len(raw) > MAX_MESSAGE:
        raise ValueError("Physics message exceeds size limit")
    return raw


def decode(raw, session):
    if len(raw) > MAX_MESSAGE:
        raise ValueError("Physics message exceeds size limit")
    message = json.loads(raw)
    if not isinstance(message, dict) or message.get("version") != VERSION or message.get("session") != session:
        raise ValueError("Physics protocol/session mismatch")
    kind = message.get("type")
    if kind == "frame":
        if type(message.get("sequence")) is not int or message["sequence"] < 0:
            raise ValueError("Invalid physics frame sequence")
        for key in ("time", "lag"):
            if not isinstance(message.get(key), (int, float)) or not math.isfinite(message[key]) or message[key] < 0:
                raise ValueError(f"Invalid physics {key}")
        positions, rotations = message.get("positions"), message.get("quaternions")
        if not isinstance(positions, list) or not isinstance(rotations, list) or len(positions) != len(rotations) or len(positions) > MAX_BODIES:
            raise ValueError("Invalid body frame dimensions")
        for values, width in ((positions, 3), (rotations, 4)):
            if any(not isinstance(row, list) or len(row) != width or
                   any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in row) for row in values):
                raise ValueError("Invalid body pose")
        if any(abs(sum(v * v for v in q) - 1) > .001 for q in rotations):
            raise ValueError("Invalid body quaternion")
        _joint_values(message.get("joints", {}))
    elif kind == "ready":
        bodies = message.get("bodies")
        if not isinstance(bodies, list) or len(bodies) > MAX_BODIES or any(not isinstance(n, str) or not n for n in bodies) or len(set(bodies)) != len(bodies):
            raise ValueError("Invalid physics body bindings")
        joints = message.get("joints", {})
        if not isinstance(joints, dict) or len(joints) > MAX_BODIES:
            raise ValueError("Invalid physics joint bindings")
        for name, info in joints.items():
            if not isinstance(info, dict) or type(info.get("available")) is not bool or not isinstance(info.get("reason"), str):
                raise ValueError("Invalid physics joint metadata")
            _joint_values({name: info.get("qpos")})
            for key in ("range", "effort_range"):
                limits = info.get(key)
                if limits is None and key == "range":
                    continue
                if not isinstance(limits, list) or len(limits) != 2 or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in limits):
                    raise ValueError("Invalid physics joint limits")
    elif kind in {"command", "ack"}:
        if message.get("command") not in COMMANDS or type(message.get("id")) is not int or message["id"] < 1:
            raise ValueError("Invalid physics command")
        if kind == "command" and message["command"] == "targets":
            _joint_values(message.get("targets"))
            if not message["targets"]:
                raise ValueError("Empty physics joint targets")
    elif kind == "error":
        if not isinstance(message.get("message"), str):
            raise ValueError("Invalid physics error")
    else:
        raise ValueError("Unknown physics message type")
    return message
