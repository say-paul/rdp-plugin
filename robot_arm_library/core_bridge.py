"""Local JSON bridge for running Caryam data graphs in Core simulation.

An Intrinsic SDK adapter can be installed in the bridge process later. Blender
only speaks this local HTTP protocol and never imports the SDK itself.
"""

from __future__ import annotations

import argparse
import json
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Mapping

from .core_runner import CoreFlowRunner, FlowPacket
from .data_graph import DataGraph


class CoreBridge:
    def __init__(self) -> None:
        self.runner: CoreFlowRunner | None = None
        self.core_host = "localhost"
        self.core_port = 17080
        self.logs: list[str] = []

    def status(self) -> dict[str, Any]:
        result = self.runner.last_result if self.runner is not None else None
        return {
            "state": "running" if self.runner is not None and self.runner.running else "stopped",
            "simulation_only": True,
            "core_endpoint": f"{self.core_host}:{self.core_port}",
            "blackboard": result.blackboard if result is not None else {},
            "logs": (self.logs + (list(result.logs) if result is not None else []))[-20:],
        }

    def run(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        graph_data = payload.get("graph")
        if not isinstance(graph_data, dict):
            raise ValueError("run requires a graph object")
        self.core_host = str(payload.get("core_host", self.core_host))
        self.core_port = int(payload.get("core_port", self.core_port))
        self.runner = CoreFlowRunner(
            DataGraph.from_dict(graph_data),
            maximum_age_seconds=float(payload.get("maximum_age_seconds", 0.25)),
        )
        self.runner.start()
        self.logs.append(f"Prepared Core simulation flow for {self.core_host}:{self.core_port}")
        sensors = payload.get("sensors")
        if isinstance(sensors, Mapping):
            self.runner.tick(sensors, now=payload.get("now"))
        return self.status()

    def tick(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        if self.runner is None:
            raise RuntimeError("No flow is running")
        sensors = payload.get("sensors", {})
        if not isinstance(sensors, Mapping):
            raise ValueError("tick sensors must be an object")
        result = self.runner.tick(sensors, now=payload.get("now"))
        return {**self.status(), "blackboard": result.blackboard}

    def stop(self) -> dict[str, Any]:
        if self.runner is not None:
            self.runner.stop()
        self.logs.append("Core simulation flow stopped")
        return self.status()

    def handle(self, method: str, path: str, payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
        if method == "GET" and path in {"/status", "/health", "/logs"}:
            return self.status()
        if method == "POST" and path == "/run":
            return self.run(payload or {})
        if method == "POST" and path == "/tick":
            return self.tick(payload or {})
        if method == "POST" and path == "/stop":
            return self.stop()
        raise ValueError(f"Unknown bridge endpoint: {method} {path}")


class _BridgeRequestHandler(BaseHTTPRequestHandler):
    server: ThreadingHTTPServer

    def do_GET(self) -> None:
        self._respond("GET", None)

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        try:
            value = json.loads(self.rfile.read(length) or b"{}")
            payload = value if isinstance(value, dict) else {}
        except json.JSONDecodeError as error:
            self._write(HTTPStatus.BAD_REQUEST, {"error": str(error)})
            return
        self._respond("POST", payload)

    def _respond(self, method: str, payload: Mapping[str, Any] | None) -> None:
        bridge: CoreBridge = self.server.core_bridge  # type: ignore[attr-defined]
        try:
            self._write(HTTPStatus.OK, bridge.handle(method, self.path, payload))
        except (RuntimeError, ValueError) as error:
            self._write(HTTPStatus.BAD_REQUEST, {"error": str(error)})

    def _write(self, status: HTTPStatus, value: Mapping[str, Any]) -> None:
        data = json.dumps(value).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, format: str, *args: object) -> None:
        return None


def create_server(host: str = "127.0.0.1", port: int = 8765) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), _BridgeRequestHandler)
    server.core_bridge = CoreBridge()  # type: ignore[attr-defined]
    return server


def serve(host: str = "127.0.0.1", port: int = 8765) -> None:
    server = create_server(host, port)
    print(f"Caryam Core bridge listening at http://{host}:{port}")
    server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the Caryam Core simulation bridge")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    options = parser.parse_args()
    serve(options.host, options.port)


if __name__ == "__main__":
    main()