"""Serializable behavior graphs for robot sensors, AI decisions, and actions."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable


GRAPH_VERSION = 1
NODE_KINDS = {"start", "sensor", "ai_model", "condition", "action", "end"}


@dataclass
class BehaviorNode:
    node_id: str
    kind: str
    label: str = ""
    data: dict[str, Any] = field(default_factory=dict)
    position: tuple[float, float] = (0.0, 0.0)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.node_id,
            "kind": self.kind,
            "label": self.label or self.node_id,
            "data": self.data,
            "position": list(self.position),
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "BehaviorNode":
        position = value.get("position", (0.0, 0.0))
        if not isinstance(position, (list, tuple)) or len(position) != 2:
            position = (0.0, 0.0)
        data = value.get("data", {})
        return cls(
            node_id=str(value.get("id", "")),
            kind=str(value.get("kind", "")),
            label=str(value.get("label", "")),
            data=dict(data) if isinstance(data, dict) else {},
            position=(float(position[0]), float(position[1])),
        )


@dataclass(frozen=True)
class BehaviorLink:
    source: str
    target: str

    def to_dict(self) -> dict[str, str]:
        return {"source": self.source, "target": self.target}


SensorReader = Callable[[dict[str, Any]], Any]
AIRunner = Callable[[dict[str, Any], dict[str, Any]], Any]
ActionSink = Callable[[dict[str, Any], Any], None]


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
        known_ids = set(node_ids)
        for link in self.links:
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
                BehaviorLink(str(link.get("source", "")), str(link.get("target", "")))
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

    def run(
        self,
        sensor_reader: SensorReader,
        ai_runner: AIRunner | None = None,
        action_sink: ActionSink | None = None,
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
        for link in self.links:
            if link.source in node_map and link.target in node_map:
                predecessors[link.target].append(link.source)
        values: dict[str, Any] = {}
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