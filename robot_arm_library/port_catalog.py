"""Catalog-backed I/O contracts for data graph nodes.

The catalog is data only. It can describe a sensor before any simulator or
Blender module is available.
"""

from __future__ import annotations

from typing import Any

from .behavior_graph import BehaviorNode, PortSpec


def _dof(dof: int) -> int:
    if isinstance(dof, bool) or int(dof) != dof or dof <= 0:
        raise ValueError("Robot DOF must be a positive integer")
    return int(dof)


def robot_joint_state_ports(dof: int) -> tuple[PortSpec, ...]:
    shape = (1, _dof(dof))
    return (
        PortSpec("joint_positions", shape, dtype="float32", semantic="joint_position", unit="rad"),
        PortSpec("joint_velocities", shape, dtype="float32", semantic="joint_velocity", unit="rad/s"),
        PortSpec("joint_efforts", shape, dtype="float32", semantic="joint_effort", unit="Nm"),
    )


def robot_joint_command_ports(dof: int) -> tuple[PortSpec, ...]:
    return (
        PortSpec(
            "joint_position_command", (1, _dof(dof)), delivery="command",
            dtype="float32", semantic="joint_position_command", unit="rad",
        ),
    )


def _positive_dimension(entry: dict[str, Any], *keys: str) -> int:
    for key in keys:
        value = entry.get(key)
        if value is not None:
            try:
                dimension = int(value)
            except (TypeError, ValueError) as error:
                raise ValueError(f"Invalid sensor dimension {key}: {value}") from error
            if dimension <= 0:
                raise ValueError(f"Sensor dimension {key} must be positive")
            return dimension
    raise ValueError(f"Missing required sensor dimension: {keys[0]}")


def sensor_port_templates(entry: dict[str, Any]) -> tuple[PortSpec, ...]:
    """Resolve output contracts from a normalized catalog entry."""
    category = str(entry.get("category", "")).lower().replace("-", "_")
    details = entry.get("camera") if category in {"camera", "rgb_camera", "depth_camera", "rgbd_camera"} else entry.get("lidar")
    if not isinstance(details, dict):
        details = entry
    rate = entry.get("rate_hz", details.get("scan_rate_hz"))
    rate_hz = float(rate) if rate is not None else None

    def port(name: str, shape: tuple[int, ...], semantic: str, unit: str = "") -> PortSpec:
        return PortSpec(name, shape, dtype="float32" if semantic in {"depth", "ranges", "angles", "imu", "force", "torque", "tactile", "radar"} else "uint8", semantic=semantic, unit=unit, rate_hz=rate_hz)

    if category in {"camera", "rgb_camera", "depth_camera", "rgbd_camera"}:
        width = _positive_dimension(details, "resolution_width", "width")
        height = _positive_dimension(details, "resolution_height", "height")
        outputs: list[PortSpec] = []
        if category in {"camera", "rgb_camera", "rgbd_camera"}:
            outputs.append(port("rgb", (1, 3, height, width), "rgb_image"))
        if category in {"depth_camera", "rgbd_camera"}:
            outputs.append(port("depth", (1, 1, height, width), "depth", "m"))
        return tuple(outputs)
    if category == "lidar":
        samples = _positive_dimension(details, "horizontal_samples", "samples")
        channels = int(details.get("channels", 1))
        if channels < 1:
            raise ValueError("Sensor dimension channels must be positive")
        outputs = [port("ranges", (1, channels, samples) if channels > 1 else (1, samples), "ranges", "m")]
        if bool(entry.get("include_angles", details.get("include_angles", False))):
            outputs.append(port("angles", (1, samples), "angles", "rad"))
        return tuple(outputs)
    if category == "imu":
        return (
            port("linear_acceleration", (1, 3), "imu", "m/s^2"),
            port("angular_velocity", (1, 3), "imu", "rad/s"),
        )
    if category in {"force_torque", "contact"}:
        return (port("force", (1, 3), "force", "N"), port("torque", (1, 3), "torque", "Nm"))
    if category == "tactile":
        cells = _positive_dimension(entry, "cells", "taxels")
        return (port("tactile", (1, cells), "tactile", "N"),)
    if category == "radar":
        bins = _positive_dimension(entry, "bins", "range_bins")
        return (port("radar", (1, bins), "radar"),)
    raise ValueError(f"Unsupported sensor category: {category}")


def catalog_node_template(node_id: str, entry: dict[str, Any], direction: str = "output") -> BehaviorNode:
    """Create an application node with catalog ports and backend metadata."""
    if direction not in {"input", "output"}:
        raise ValueError(f"Unsupported catalog node direction: {direction}")
    category = str(entry.get("category", ""))
    if category == "robot_state":
        ports = robot_joint_state_ports(_positive_dimension(entry, "dof"))
    elif category == "robot_command":
        ports = robot_joint_command_ports(_positive_dimension(entry, "dof"))
    else:
        ports = sensor_port_templates(entry)
    capabilities = entry.get("backend_capabilities", {})
    if not isinstance(capabilities, dict):
        raise ValueError("backend_capabilities must be a mapping")
    return BehaviorNode(
        node_id=node_id,
        kind="application",
        label=str(entry.get("name", node_id)),
        data={"catalog_category": category, "backend_capabilities": dict(capabilities)},
        inputs=ports if direction == "input" or category == "robot_command" else (),
        outputs=ports if direction == "output" and category != "robot_command" else (),
    )
