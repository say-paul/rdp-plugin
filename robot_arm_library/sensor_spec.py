"""Simulator-independent lidar and depth-camera definitions."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from .behavior_graph import PortSpec

SENSOR_CATEGORIES = {"lidar", "depth_camera"}


@dataclass(frozen=True)
class LidarSensorSpec:
    channels: int = 1
    horizontal_fov: float = 360.0
    vertical_fov: float = 0.0
    horizontal_samples: int = 360
    range_min: float = 0.05
    range_max: float = 30.0
    scan_rate_hz: float = 10.0

    def __post_init__(self) -> None:
        if self.channels < 1 or self.horizontal_samples < 1:
            raise ValueError("Lidar channels and samples must be positive")
        if not 0 < self.horizontal_fov <= 360 or self.vertical_fov < 0:
            raise ValueError("Lidar field of view is invalid")
        if self.range_min < 0 or self.range_max <= self.range_min or self.scan_rate_hz <= 0:
            raise ValueError("Lidar range and scan rate must be positive")

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "LidarSensorSpec":
        return cls(
            channels=int(value.get("channels", 1)),
            horizontal_fov=float(value.get("horizontal_fov", 360.0)),
            vertical_fov=float(value.get("vertical_fov", 0.0)),
            horizontal_samples=int(value.get("horizontal_samples", 360)),
            range_min=float(value.get("range_min", 0.05)),
            range_max=float(value.get("range_max", 30.0)),
            scan_rate_hz=float(value.get("scan_rate_hz", 10.0)),
        )


@dataclass(frozen=True)
class CameraSensorSpec:
    focal_length: float = 1.93
    horizontal_aperture: float = 3.896
    vertical_aperture: float = 2.453
    clipping_near: float = 0.01
    clipping_far: float = 1000.0
    resolution_width: int = 640
    resolution_height: int = 480
    depth_enabled: bool = True

    def __post_init__(self) -> None:
        if min(self.focal_length, self.horizontal_aperture, self.vertical_aperture) <= 0:
            raise ValueError("Camera focal length and apertures must be positive")
        if self.clipping_near <= 0 or self.clipping_far <= self.clipping_near:
            raise ValueError("Camera clipping range is invalid")
        if self.resolution_width < 1 or self.resolution_height < 1:
            raise ValueError("Camera resolution must be positive")

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "CameraSensorSpec":
        return cls(
            focal_length=float(value.get("focal_length", 1.93)),
            horizontal_aperture=float(value.get("horizontal_aperture", 3.896)),
            vertical_aperture=float(value.get("vertical_aperture", 2.453)),
            clipping_near=float(value.get("clipping_near", 0.01)),
            clipping_far=float(value.get("clipping_far", 1000.0)),
            resolution_width=int(value.get("resolution_width", 640)),
            resolution_height=int(value.get("resolution_height", 480)),
            depth_enabled=bool(value.get("depth_enabled", True)),
        )

    def vertical_fov_degrees(self) -> float:
        return math.degrees(2.0 * math.atan(self.vertical_aperture / (2.0 * self.focal_length)))


@dataclass(frozen=True)
class SensorDefinition:
    sensor_id: str
    name: str
    category: str
    source_path: str
    description: str = ""
    lidar: LidarSensorSpec | None = None
    camera: CameraSensorSpec | None = None
    backend_capabilities: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.category not in SENSOR_CATEGORIES:
            raise ValueError(f"Unsupported sensor category: {self.category}")
        if not self.sensor_id:
            raise ValueError("Sensor id cannot be empty")

    def to_dict(self) -> dict[str, Any]:
        value: dict[str, Any] = {
            "id": self.sensor_id,
            "name": self.name,
            "category": self.category,
            "description": self.description,
        }
        if self.lidar is not None:
            value["lidar"] = self.lidar.to_dict()
        if self.camera is not None:
            value["camera"] = self.camera.to_dict()
        if self.backend_capabilities:
            value["backend_capabilities"] = dict(self.backend_capabilities)
        return value

    def output_ports(self) -> tuple[PortSpec, ...]:
        from .port_catalog import sensor_port_templates

        value = self.to_dict()
        if self.category == "lidar" and self.lidar is None:
            value["lidar"] = LidarSensorSpec().to_dict()
        if self.category == "depth_camera" and self.camera is None:
            value["camera"] = CameraSensorSpec().to_dict()
        return sensor_port_templates(value)

    @classmethod
    def from_dict(cls, sensor_id: str, value: dict[str, Any], source_path: str) -> "SensorDefinition":
        category = str(value.get("category", ""))
        default_capability = {"mujoco": "rangefinder_fan" if category == "lidar" else "camera_depth"} if category in SENSOR_CATEGORIES else {}
        capabilities = value.get("backend_capabilities", default_capability)
        if not isinstance(capabilities, dict):
            raise ValueError("backend_capabilities must be a mapping")
        return cls(
            sensor_id=sensor_id,
            name=str(value.get("name", sensor_id)),
            category=category,
            source_path=source_path,
            description=str(value.get("description", "")),
            lidar=LidarSensorSpec.from_dict(value.get("lidar", {})) if category == "lidar" else None,
            camera=CameraSensorSpec.from_dict(value.get("camera", {})) if category == "depth_camera" else None,
            backend_capabilities={str(key): str(item) for key, item in capabilities.items()},
        )
