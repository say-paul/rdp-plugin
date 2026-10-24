"""Discover robots and virtual sensors across registered repositories."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .sensor_spec import SensorDefinition
from .setup_parser import RobotDefinition, discover_robot_definitions, discover_sensor_definitions

SOURCE_KINDS = {"auto", "robot", "sensor"}


@dataclass(frozen=True)
class AssetSource:
    path: str
    kind: str = "auto"
    label: str = ""
    enabled: bool = True


def discover_from_sources(sources: Iterable[AssetSource]) -> tuple[list[RobotDefinition], list[SensorDefinition]]:
    robots: list[RobotDefinition] = []
    sensors: list[SensorDefinition] = []
    seen_robots: set[tuple[str, str]] = set()
    seen_sensors: set[tuple[str, str]] = set()
    for source in sources:
        if source.kind not in SOURCE_KINDS:
            raise ValueError(f"Unsupported asset source kind: {source.kind}")
        if not source.enabled or not source.path:
            continue
        if source.kind in {"auto", "robot"}:
            for robot in discover_robot_definitions(source.path):
                key = (robot.robot_id, robot.model_path)
                if key not in seen_robots:
                    seen_robots.add(key)
                    robots.append(robot)
        if source.kind in {"auto", "sensor"}:
            for sensor in discover_sensor_definitions(source.path):
                key = (sensor.sensor_id, sensor.source_path)
                if key not in seen_sensors:
                    seen_sensors.add(key)
                    sensors.append(sensor)
    return robots, sensors
