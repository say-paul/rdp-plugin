"""Simulation-safe execution for Caryam data graphs on an Intrinsic Core host.

The runner deliberately has no robot transport. It computes compact data-flow
outputs and only exposes a command when it is valid, fresh, and inside declared
joint limits. The bridge owns deployment and any optional Intrinsic SDK adapter.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from time import time
from typing import Any, Callable, Mapping

from .data_graph import DataGraph, DataPort, SENSOR_NODE_KINDS


@dataclass(frozen=True)
class FlowPacket:
    value: Any
    timestamp: float
    valid: bool = True
    data_type: str = "any"
    delivery: str = "stream"
    reason: str = ""

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "FlowPacket":
        return cls(
            value=value.get("value"),
            timestamp=float(value.get("timestamp", 0.0)),
            valid=bool(value.get("valid", True)),
            data_type=str(value.get("type", "any")),
            delivery=str(value.get("delivery", "stream")),
            reason=str(value.get("reason", "")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "value": self.value,
            "timestamp": self.timestamp,
            "valid": self.valid,
            "type": self.data_type,
            "delivery": self.delivery,
            "reason": self.reason,
        }

    def is_fresh(self, now: float, maximum_age_seconds: float) -> bool:
        return self.valid and 0.0 <= now - self.timestamp <= maximum_age_seconds

    def with_contract(self, port: DataPort) -> "FlowPacket":
        return replace(
            self,
            data_type=port.data_type if port.data_type != "any" else self.data_type,
            delivery=port.delivery,
        )


@dataclass(frozen=True)
class JointTarget:
    positions: tuple[float, ...]
    timestamp: float
    valid: bool
    velocity_limits: tuple[float, ...] = ()
    acceleration_limits: tuple[float, ...] = ()
    reason: str = ""

    @classmethod
    def from_value(cls, value: Any, timestamp: float, valid: bool, reason: str = "") -> "JointTarget":
        if isinstance(value, Mapping):
            positions = value.get("positions", ())
            velocity_limits = value.get("velocity_limits", ())
            acceleration_limits = value.get("acceleration_limits", ())
        else:
            positions = value
            velocity_limits = ()
            acceleration_limits = ()
        try:
            parsed_positions = tuple(float(position) for position in positions)
            parsed_velocity_limits = tuple(float(limit) for limit in velocity_limits)
            parsed_acceleration_limits = tuple(float(limit) for limit in acceleration_limits)
        except (TypeError, ValueError):
            return cls((), timestamp, False, reason="Joint target positions must be numeric")
        if not parsed_positions:
            return cls((), timestamp, False, reason="Joint target has no positions")
        return cls(
            parsed_positions,
            timestamp,
            valid,
            parsed_velocity_limits,
            parsed_acceleration_limits,
            reason,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "positions": list(self.positions),
            "velocity_limits": list(self.velocity_limits),
            "acceleration_limits": list(self.acceleration_limits),
            "timestamp": self.timestamp,
            "valid": self.valid,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class FlowRunResult:
    state: str
    outputs: dict[str, dict[str, FlowPacket]]
    blackboard: dict[str, Any]
    joint_target: JointTarget | None
    logs: tuple[str, ...]


PacketProcessor = Callable[[dict[str, FlowPacket]], Mapping[str, FlowPacket | Any]]


class CoreFlowRunner:
    """Run a serializable data graph in simulation without commanding hardware."""

    def __init__(
        self,
        graph: DataGraph,
        maximum_age_seconds: float = 0.25,
        processors: Mapping[str, PacketProcessor] | None = None,
    ) -> None:
        errors = graph.validate()
        if errors:
            raise ValueError("Invalid data graph: " + "; ".join(errors))
        if maximum_age_seconds <= 0:
            raise ValueError("maximum_age_seconds must be positive")
        self.graph = graph
        self.maximum_age_seconds = maximum_age_seconds
        self.processors = dict(processors or {})
        self.running = False
        self.logs: list[str] = []
        self.last_result: FlowRunResult | None = None

    def start(self) -> None:
        self.running = True
        self.logs.append("Flow runner started in simulation mode")

    def stop(self) -> None:
        self.running = False
        self.logs.append("Flow runner stopped")

    def tick(
        self,
        sensor_inputs: Mapping[str, FlowPacket | Mapping[str, Any] | Any],
        *,
        now: float | None = None,
    ) -> FlowRunResult:
        if not self.running:
            raise RuntimeError("Flow runner is not running")
        timestamp = time() if now is None else now
        output_packets: dict[str, dict[str, FlowPacket]] = {}
        links_by_target: dict[str, list[object]] = {node.node_id: [] for node in self.graph.nodes}
        for link in self.graph.links:
            links_by_target.setdefault(link.target, []).append(link)

        for node_id in self.graph.execution_order():
            node = self.graph.node_map()[node_id]
            inputs = self._node_inputs(node_id, links_by_target[node_id], output_packets, timestamp)
            if node.kind in SENSOR_NODE_KINDS:
                packet = self._packet(sensor_inputs.get(node_id), timestamp)
                output_packets[node_id] = self._emit(node.outputs, packet)
            elif node.kind == "joint_controller":
                packet = self._joint_controller(inputs, node.data, timestamp)
                output_packets[node_id] = self._emit(node.outputs, packet, fallback_name="joint_target")
            elif node.kind == "joint_target":
                packet = self._joint_target(inputs, timestamp)
                output_packets[node_id] = self._emit(node.outputs, packet, fallback_name="joint_target")
            elif node.kind in {"stop", "failure"}:
                self.stop()
                packet = FlowPacket(None, timestamp, False, reason=f"Control node {node.kind}")
                output_packets[node_id] = self._emit(node.outputs, packet)
            else:
                output_packets[node_id] = self._application_outputs(node_id, node.outputs, inputs, timestamp)

        result = self._result(output_packets)
        self.last_result = result
        return result

    def _packet(self, value: FlowPacket | Mapping[str, Any] | Any, timestamp: float) -> FlowPacket:
        if isinstance(value, FlowPacket):
            return value
        if isinstance(value, Mapping) and {"value", "timestamp"}.intersection(value):
            return FlowPacket.from_dict(value)
        if value is None:
            return FlowPacket(None, timestamp, False, reason="No sensor value received")
        return FlowPacket(value, timestamp)

    def _node_inputs(
        self,
        node_id: str,
        links: list[object],
        outputs: Mapping[str, Mapping[str, FlowPacket]],
        timestamp: float,
    ) -> dict[str, FlowPacket]:
        inputs: dict[str, FlowPacket] = {}
        for link in links:
            source_outputs = outputs.get(link.source, {})
            packet = source_outputs.get(link.source_port)
            if packet is None:
                packet = FlowPacket(None, timestamp, False, reason=f"Missing source {link.source}.{link.source_port}")
            inputs[link.target_port] = self._fresh_packet(packet, timestamp)
        return inputs

    def _fresh_packet(self, packet: FlowPacket, timestamp: float) -> FlowPacket:
        if packet.is_fresh(timestamp, self.maximum_age_seconds):
            return packet
        if not packet.valid:
            return packet
        return replace(packet, valid=False, reason="Input is stale")

    def _emit(
        self,
        ports: tuple[DataPort, ...],
        packet: FlowPacket,
        fallback_name: str = "value",
    ) -> dict[str, FlowPacket]:
        selected_ports = ports or (DataPort(fallback_name),)
        return {port.name: packet.with_contract(port) for port in selected_ports}

    def _application_outputs(
        self,
        node_id: str,
        ports: tuple[DataPort, ...],
        inputs: dict[str, FlowPacket],
        timestamp: float,
    ) -> dict[str, FlowPacket]:
        processor = self.processors.get(node_id)
        if processor is None:
            packet = next(iter(inputs.values()), FlowPacket(None, timestamp, False, reason="No input received"))
            return self._emit(ports, packet)
        produced = processor(inputs)
        result: dict[str, FlowPacket] = {}
        for port in ports or (DataPort("value"),):
            packet = self._packet(produced.get(port.name), timestamp)
            result[port.name] = packet.with_contract(port)
        return result

    def _joint_controller(
        self,
        inputs: Mapping[str, FlowPacket],
        config: Mapping[str, Any],
        timestamp: float,
    ) -> FlowPacket:
        requested = inputs.get("target") or inputs.get("requested_target") or next(iter(inputs.values()), None)
        if requested is None:
            return FlowPacket(None, timestamp, False, "joint_target", "command", "No requested target")
        if not requested.valid:
            return FlowPacket(None, requested.timestamp, False, "joint_target", "command", requested.reason)
        target = JointTarget.from_value(requested.value, requested.timestamp, True)
        if not target.valid:
            return FlowPacket(target.to_dict(), target.timestamp, False, "joint_target", "command", target.reason)
        limits = config.get("joint_limits", [])
        for index, position in enumerate(target.positions):
            if index >= len(limits):
                continue
            try:
                lower, upper = (float(value) for value in limits[index])
            except (TypeError, ValueError):
                return FlowPacket(None, target.timestamp, False, "joint_target", "command", "Invalid joint limits")
            if position < lower or position > upper:
                reason = f"Joint {index} target {position:g} is outside [{lower:g}, {upper:g}]"
                invalid_target = JointTarget(target.positions, target.timestamp, False, reason=reason)
                return FlowPacket(invalid_target.to_dict(), target.timestamp, False, "joint_target", "command", reason)
        return FlowPacket(target.to_dict(), target.timestamp, True, "joint_target", "command")

    def _joint_target(self, inputs: Mapping[str, FlowPacket], timestamp: float) -> FlowPacket:
        packet = inputs.get("joint_target") or next(iter(inputs.values()), None)
        if packet is None:
            return FlowPacket(None, timestamp, False, "joint_target", "command", "No controller output")
        target = JointTarget.from_value(packet.value, packet.timestamp, packet.valid, packet.reason)
        return FlowPacket(target.to_dict(), target.timestamp, target.valid, "joint_target", "command", target.reason)

    def _result(self, outputs: dict[str, dict[str, FlowPacket]]) -> FlowRunResult:
        blackboard: dict[str, Any] = {}
        selected_target: JointTarget | None = None
        for node_outputs in outputs.values():
            for port_name, packet in node_outputs.items():
                if port_name == "part_pose":
                    blackboard["caryam.perception.part_pose"] = packet.value
                elif port_name == "tilt_exceeded":
                    blackboard["caryam.imu.tilt_exceeded"] = packet.value
                if packet.data_type == "joint_target":
                    selected_target = JointTarget.from_value(packet.value, packet.timestamp, packet.valid, packet.reason)
        if selected_target is not None:
            blackboard["caryam.control.joint_target"] = selected_target.to_dict()
            blackboard["caryam.control.command_valid"] = selected_target.valid
        state = "running" if self.running else "stopped"
        return FlowRunResult(state, outputs, blackboard, selected_target, tuple(self.logs[-20:]))