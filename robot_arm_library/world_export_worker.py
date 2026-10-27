"""Compile an export request in the user's external MuJoCo Python environment."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from .sensor_spec import CameraSensorSpec, LidarSensorSpec, SensorDefinition
from .world_exporter import RobotInstance, RobotMount, SceneMeshInstance, SensorMount, WorldOptions, write_world_export


def _vectors(value: dict[str, Any], *keys: str) -> dict[str, Any]:
    result = dict(value)
    for key in keys:
        result[key] = tuple(result[key])
    return result


def _mount(value: dict[str, Any] | None) -> RobotMount | None:
    return RobotMount(**_vectors(value, "position", "quaternion")) if value is not None else None


def _sensor_mount(value: dict[str, Any]) -> SensorMount:
    sensor = dict(value["sensor"])
    if sensor["lidar"] is not None:
        sensor["lidar"] = LidarSensorSpec(**sensor["lidar"])
    if sensor["camera"] is not None:
        sensor["camera"] = CameraSensorSpec(**sensor["camera"])
    mount = _vectors(value, "position", "quaternion")
    mount["sensor"] = SensorDefinition(**sensor)
    return SensorMount(**mount)


def export_request(payload: dict[str, Any]) -> tuple[Path, Path, Path]:
    if payload.get("version") != 1:
        raise ValueError("Unsupported MuJoCo export request version")
    robots = []
    for value in payload["robots"]:
        robot = _vectors(value, "position", "quaternion")
        robot["mount"] = _mount(value["mount"])
        robot["sensors"] = tuple(_sensor_mount(mount) for mount in value["sensors"])
        robots.append(RobotInstance(**robot))
    meshes = []
    for value in payload["scene_meshes"]:
        mesh = _vectors(value, "position", "quaternion", "scale", "rgba")
        mesh["mount"] = _mount(value["mount"])
        meshes.append(SceneMeshInstance(**mesh))
    sensors = tuple(_sensor_mount(mount) for mount in payload.get("scene_sensors", []))
    return write_world_export(
        payload["output_path"], robots, meshes, world_options=WorldOptions(**payload.get("world_options", {})),
        scene_sensors=sensors,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Compile a Caryam world using external MuJoCo")
    parser.add_argument("request", type=Path)
    options = parser.parse_args()
    try:
        export_request(json.loads(options.request.read_text(encoding="utf-8")))
    except Exception as error:
        print(f"{type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
