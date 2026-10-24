"""Compile lidar and depth-camera definitions into MJCF elements."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ElementTree

from ..sensor_spec import CameraSensorSpec, LidarSensorSpec


def compile_lidar(
    spec: LidarSensorSpec, mount_name: str,
) -> tuple[list[ElementTree.Element], list[ElementTree.Element]]:
    """Return body sites and top-level rangefinder sensors for a ray fan."""
    if not mount_name:
        raise ValueError("Lidar mount name is required")
    sites: list[ElementTree.Element] = []
    rangefinders: list[ElementTree.Element] = []
    for channel in range(spec.channels):
        pitch = (
            -spec.vertical_fov / 2 + spec.vertical_fov * channel / (spec.channels - 1)
            if spec.channels > 1 else 0.0
        )
        for index in range(spec.horizontal_samples):
            yaw = (
                -spec.horizontal_fov / 2 + spec.horizontal_fov * index / (spec.horizontal_samples - 1)
                if spec.horizontal_samples > 1 else 0.0
            )
            suffix = f"{channel:02d}_{index:02d}" if spec.channels > 1 else f"{index:02d}"
            name = f"{mount_name}_rf_{suffix}"
            # Rotate the site's +Z ray toward +X, then apply pitch and yaw.
            # A quaternion avoids dependence on the source model's eulerseq.
            yaw_half = math.radians(yaw) / 2
            tilt_half = math.radians(90 - pitch) / 2
            cy, sy = math.cos(yaw_half), math.sin(yaw_half)
            ct, st = math.cos(tilt_half), math.sin(tilt_half)
            quat = (cy * ct, -sy * st, cy * st, sy * ct)
            sites.append(ElementTree.Element(
                "site", {"name": name, "quat": " ".join(f"{value:.12g}" for value in quat), "size": "0.002"},
            ))
            rangefinders.append(ElementTree.Element(
                "rangefinder", {"name": name, "site": name, "cutoff": f"{spec.range_max:.6g}"},
            ))
    return sites, rangefinders


def compile_depth_camera(spec: CameraSensorSpec, mount_name: str) -> ElementTree.Element:
    if not mount_name:
        raise ValueError("Camera mount name is required")
    fovy = spec.vertical_fov_degrees()
    if not 0 < fovy < 180 or not math.isfinite(fovy):
        raise ValueError("Camera vertical FOV must be between 0 and 180 degrees")
    return ElementTree.Element("camera", {"name": mount_name, "mode": "fixed", "fovy": f"{fovy:.6g}"})
