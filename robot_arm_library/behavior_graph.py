"""Serializable behavior graphs for robot sensors, AI decisions, and actions."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable


GRAPH_VERSION = 1
NODE_KINDS = {"start", "sensor", "ai_model", "condition", "action", "end", "application"}


@dataclass(frozen=True)
class PortSpec:
    name: str
    shape: tuple[int, ...] = ()

    def compatibility_error(self, target: "PortSpec") -> str | None:
        if self.shape and target.shape and self.shape != target.shape:
            return f"Incompatible port shapes: {self.shape} -> {target.shape}"
        return None

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "shape": list(self.shape)}

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "PortSpec":
        shape = value.get("shape", [])
        parsed_shape = tuple(int(part) for part in shape) if isinstance(shape, (list, tuple)) else ()
        return cls(name=str(value.get("name", "")), shape=parsed_shape)


@dataclass
class BehaviorNode:
    node_id: str
    kind: str
    label: str = ""
    data: dict[str, Any] = field(default_factory=dict)
    position: tuple[float, float] = (0.0, 0.0)
    handler: str = ""
    inputs: tuple[PortSpec, ...] = ()
    outputs: tuple[PortSpec, ...] = ()

    def input_names(self) -> tuple[str, ...]:
        return tuple(port.name for port in self.inputs)

    def output_names(self) -> tuple[str, ...]:
        return tuple(port.name for port in self.outputs)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.node_id,
            "kind": self.kind,
            "label": self.label or self.node_id,
            "data": self.data,
            "position": list(self.position),
            "handler": self.handler,
            "inputs": [port.to_dict() for port in self.inputs],
            "outputs": [port.to_dict() for port in self.outputs],
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "BehaviorNode":
        position = value.get("position", (0.0, 0.0))
        if not isinstance(position, (list, tuple)) or len(position) != 2:
            position = (0.0, 0.0)
        data = value.get("data", {})
        inputs = tuple(
            PortSpec.from_dict(port) for port in value.get("inputs", []) if isinstance(port, dict)
        )
        outputs = tuple(
            PortSpec.from_dict(port) for port in value.get("outputs", []) if isinstance(port, dict)
        )
        return cls(
            node_id=str(value.get("id", "")),
            kind=str(value.get("kind", "")),
            label=str(value.get("label", "")),
            data=dict(data) if isinstance(data, dict) else {},
            position=(float(position[0]), float(position[1])),
            handler=str(value.get("handler", "")),
            inputs=inputs,
            outputs=outputs,
        )


@dataclass(frozen=True)
class BehaviorLink:
    source: str
    target: str
    source_port: str = "value"
    target_port: str = "value"

    def to_dict(self) -> dict[str, str]:
        return {
            "source": self.source,
            "target": self.target,
            "source_port": self.source_port,
            "target_port": self.target_port,
        }


SensorReader = Callable[[dict[str, Any]], Any]
AIRunner = Callable[[dict[str, Any], dict[str, Any]], Any]
ActionSink = Callable[[dict[str, Any], Any], None]
ApplicationRunner = Callable[[BehaviorNode, dict[str, Any]], dict[str, Any]]


@dataclass
class BehaviorGraph:
    name: str = "Robot Behavior"
    nodes: list[BehaviorNode] = field(default_factory=list)
    links: list[BehaviorLink] = field(default_factory=list)

    def node_map(self) -> dict[str, BehaviorNode]:
        return {node.node_id: node for node in self.nodes}

    def validate(self) -> list[str]:
        errors: list[str] = []
        node_ids = [node.node_id for node in self.nodes]
        if any(not node_id for node_id in node_ids):
            errors.append("Every node needs an id")
        duplicates = sorted({node_id for node_id in node_ids if node_ids.count(node_id) > 1})
        errors.extend(f"Duplicate node id: {node_id}" for node_id in duplicates)
        errors.extend(
            f"Unsupported node kind: {node.kind}"
            for node in self.nodes
            if node.kind not in NODE_KINDS
        )
        errors.extend(
            f"Application node has no ports: {node.node_id}"
            for node in self.nodes
            if node.kind == "application" and not node.inputs and not node.outputs
        )
        node_map = self.node_map()
        known_ids = set(node_ids)
        for link in self.links:
            source_node = node_map.get(link.source)
            target_node = node_map.get(link.target)
            if (
                source_node is not None
                and source_node.kind == "application"
                and link.source_port not in source_node.output_names()
            ):
                errors.append(f"Unknown output port '{link.source_port}' on {link.source}")
            if (
                target_node is not None
                and target_node.kind == "application"
                and link.target_port not in target_node.input_names()
            ):
                errors.append(f"Unknown input port '{link.target_port}' on {link.target}")
            if source_node is not None and target_node is not None:
                source_port = next(
                    (port for port in source_node.outputs if port.name == link.source_port), None
                )
                target_port = next(
                    (port for port in target_node.inputs if port.name == link.target_port), None
                )
                if source_port is not None and target_port is not None:
                    compatibility_error = source_port.compatibility_error(target_port)
                    if compatibility_error is not None:
                        errors.append(
                            "Incompatible port shapes: "
                            f"{link.source}.{link.source_port} {source_port.shape} -> "
                            f"{link.target}.{link.target_port} {target_port.shape}"
                        )
            if link.source not in known_ids:
                errors.append(f"Unknown link source: {link.source}")
            if link.target not in known_ids:
                errors.append(f"Unknown link target: {link.target}")
        try:
            self.execution_order()
        except ValueError as error:
            errors.append(str(error))
        return errors

    def execution_order(self) -> list[str]:
        """Return a deterministic topological order for a behavior graph."""
        node_map = self.node_map()
        incoming = {node_id: 0 for node_id in node_map}
        outgoing: dict[str, list[str]] = {node_id: [] for node_id in node_map}
        for link in self.links:
            if link.source not in node_map or link.target not in node_map:
                continue
            incoming[link.target] += 1
            outgoing[link.source].append(link.target)
        ready = sorted(node_id for node_id, count in incoming.items() if count == 0)
        order: list[str] = []
        while ready:
            node_id = ready.pop(0)
            order.append(node_id)
            for target in sorted(outgoing[node_id]):
                incoming[target] -= 1
                if incoming[target] == 0:
                    ready.append(target)
                    ready.sort()
        if len(order) != len(node_map):
            raise ValueError("Behavior graph contains a cycle")
        return order

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": GRAPH_VERSION,
            "name": self.name,
            "nodes": [node.to_dict() for node in self.nodes],
            "links": [link.to_dict() for link in self.links],
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "BehaviorGraph":
        nodes = value.get("nodes", [])
        links = value.get("links", [])
        graph = cls(
            name=str(value.get("name", "Robot Behavior")),
            nodes=[BehaviorNode.from_dict(node) for node in nodes if isinstance(node, dict)],
            links=[
                BehaviorLink(
                    str(link.get("source", "")),
                    str(link.get("target", "")),
                    str(link.get("source_port", "value")),
                    str(link.get("target_port", "value")),
                )
                for link in links
                if isinstance(link, dict)
            ],
        )
        errors = graph.validate()
        if errors:
            raise ValueError("Invalid behavior graph: " + "; ".join(errors))
        return graph

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "BehaviorGraph":
        value = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("Behavior graph JSON must contain an object")
        return cls.from_dict(value)

    def to_flow_dict(self) -> dict[str, Any]:
        """Convert application nodes into a flow.yaml-shaped mapping."""
        applications: list[dict[str, Any]] = []
        for node in self.nodes:
            if node.kind != "application":
                continue
            entry: dict[str, Any] = {"name": node.node_id, "type": "python"}
            if node.handler:
                entry["handler"] = node.handler
            if node.inputs:
                entry["inputs"] = {port.name: list(port.shape) for port in node.inputs}
            if node.outputs:
                entry["outputs"] = {port.name: list(port.shape) for port in node.outputs}
            applications.append(entry)

        dataflow: list[str] = []
        for node in self.nodes:
            if node.kind != "application":
                continue
            incoming_ports = sorted({link.target_port for link in self.links if link.target == node.node_id})
            outgoing_ports = sorted({link.source_port for link in self.links if link.source == node.node_id})
            if not incoming_ports:
                continue
            if not outgoing_ports:
                outgoing_ports = list(node.output_names())
            left = ", ".join(incoming_ports)
            line = f"{left} → {node.node_id}"
            if outgoing_ports:
                line += f" → {', '.join(outgoing_ports)}"
            dataflow.append(line)
        return {"application": applications, "dataflow": dataflow}

    @classmethod
    def from_flow_dict(cls, value: dict[str, Any]) -> "BehaviorGraph":
        """Build a graph from a flow.yaml-shaped mapping.

        Links are inferred from matching port names across application entries.
        The optional ``dataflow`` list is descriptive because it can contain
        external routes that are not represented by application nodes.
        """
        applications = value.get("application", [])
        nodes: list[BehaviorNode] = []
        producers: dict[str, tuple[str, str]] = {}
        consumers: list[tuple[str, str, str]] = []
        for index, entry in enumerate(applications):
            if not isinstance(entry, dict):
                continue
            name = str(entry.get("name", f"application_{index}"))
            inputs = tuple(
                PortSpec(str(port_name), tuple(int(part) for part in shape))
                for port_name, shape in (entry.get("inputs") or {}).items()
                if isinstance(shape, (list, tuple))
            )
            outputs = tuple(
                PortSpec(str(port_name), tuple(int(part) for part in shape))
                for port_name, shape in (entry.get("outputs") or {}).items()
                if isinstance(shape, (list, tuple))
            )
            nodes.append(
                BehaviorNode(
                    node_id=name,
                    kind="application",
                    label=name,
                    handler=str(entry.get("handler", "")),
                    inputs=inputs,
                    outputs=outputs,
                    position=(float(index) * 220.0, 0.0),
                )
            )
            for port in outputs:
                producers[port.name] = (name, port.name)
            for port in inputs:
                consumers.append((name, port.name, port.name))
        links = [
            BehaviorLink(producers[signal][0], target_node, producers[signal][1], target_port)
            for target_node, target_port, signal in consumers
            if signal in producers
        ]
        graph = cls(name=str(value.get("name", "Robot Flow")), nodes=nodes, links=links)
        errors = graph.validate()
        if errors:
            raise ValueError("Invalid flow definition: " + "; ".join(errors))
        return graph

    def save_flow_yaml(self, path: str | Path) -> None:
        try:
            import yaml
        except ImportError as error:
            raise RuntimeError("PyYAML is required to save a flow.yaml file") from error
        Path(path).write_text(
            yaml.safe_dump(self.to_flow_dict(), sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )

    @classmethod
    def load_flow_yaml(cls, path: str | Path) -> "BehaviorGraph":
        try:
            import yaml
        except ImportError as error:
            raise RuntimeError("PyYAML is required to load a flow.yaml file") from error
        value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("flow.yaml must contain a mapping")
        return cls.from_flow_dict(value)

    def run(
        self,
        sensor_reader: SensorReader,
        ai_runner: AIRunner | None = None,
        action_sink: ActionSink | None = None,
        application_runner: ApplicationRunner | None = None,
    ) -> dict[str, Any]:
        """Run a linear graph and return each node's produced value.

        Branching is represented by condition values for now; callers can use the
        returned values to decide which graph to run next while the Blender editor
        remains deliberately deterministic and serializable.
        """
        errors = self.validate()
        if errors:
            raise ValueError("Invalid behavior graph: " + "; ".join(errors))
        node_map = self.node_map()
        predecessors: dict[str, list[str]] = {node_id: [] for node_id in node_map}
        port_links: dict[str, list[BehaviorLink]] = {node_id: [] for node_id in node_map}
        for link in self.links:
            if link.source in node_map and link.target in node_map:
                predecessors[link.target].append(link.source)
            port_links[link.target].append(link)
        values: dict[str, Any] = {}
        port_outputs: dict[str, dict[str, Any]] = {}
        for node_id in self.execution_order():
            node = node_map[node_id]
            inputs = [values[source] for source in predecessors[node_id] if source in values]
            if node.kind in {"start", "end"}:
                values[node_id] = inputs[0] if inputs else None
            elif node.kind == "sensor":
                values[node_id] = sensor_reader(node.data)
            elif node.kind == "ai_model":
                if ai_runner is None:
                    raise ValueError(f"No AI runner configured for node: {node_id}")
                node_input = inputs[-1] if len(inputs) == 1 else inputs
                values[node_id] = ai_runner(node.data, {"input": node_input, "values": values.copy()})
            elif node.kind == "condition":
                values[node_id] = bool(inputs[-1]) if inputs else bool(node.data.get("value", False))
            elif node.kind == "action":
                action_input = inputs[-1] if len(inputs) == 1 else inputs
                if action_sink is not None:
                    action_sink(node.data, action_input)
                values[node_id] = action_input
            elif node.kind == "application":
                if application_runner is None:
                    raise ValueError(f"No application runner configured for node: {node_id}")
                named_inputs = {
                    link.target_port: port_outputs.get(link.source, {}).get(
                        link.source_port, values.get(link.source)
                    )
                    for link in port_links[node_id]
                }
                outputs = application_runner(node, named_inputs)
                port_outputs[node_id] = dict(outputs)
                values[node_id] = outputs
        return values


def default_behavior_graph() -> BehaviorGraph:
    """Create a useful starter graph for the Blender node editor."""
    return BehaviorGraph(
        nodes=[
            BehaviorNode("start", "start", "Start", position=(-420.0, 0.0)),
            BehaviorNode("sensor", "sensor", "Virtual Sensor", {"sensor": "joint.qpos"}, (-180.0, 0.0)),
            BehaviorNode("ai", "ai_model", "AI Model", {"model": "rule_based"}, (60.0, 0.0)),
            BehaviorNode("action", "action", "Robot Action", {"robot_id": "", "joint": "", "target": 0.0}, (300.0, 0.0)),
            BehaviorNode("end", "end", "End", position=(540.0, 0.0)),
        ],
        links=[
            BehaviorLink("start", "sensor"),
            BehaviorLink("sensor", "ai"),
            BehaviorLink("ai", "action"),
            BehaviorLink("action", "end"),
        ],
    )