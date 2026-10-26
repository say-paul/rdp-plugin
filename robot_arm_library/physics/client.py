"""Nonblocking pipe client; no Blender or native MuJoCo dependencies."""

from __future__ import annotations

from collections import deque
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import tempfile
import threading
import time
import uuid

from .protocol import COMMANDS, MAX_MESSAGE, VERSION, decode, encode


class SimulationClient:
    def __init__(self, executable, *, rate_hz=30, startup_timeout=120, command_timeout=5):
        self.executable = executable
        self.rate_hz = rate_hz
        self.startup_timeout = startup_timeout
        self.command_timeout = command_timeout
        self.directory = Path(tempfile.mkdtemp(prefix="caryam-physics-"))
        self.session = uuid.uuid4().hex
        self.process = None
        self._events = queue.Queue(maxsize=128)
        self._outgoing = queue.Queue(maxsize=64)
        self._stderr = deque(maxlen=32)
        self._lock = threading.Lock()
        self._frame = None
        self._sequence = -1
        self._pending = {}
        self._command_id = 0
        self._ready = False
        self._closing = False
        self._failed = False
        self._threads = []
        self._started = 0.0
        self._last_frame_at = 0.0
        self._running = False
        self._closed = threading.Event()

    def start(self, request, body_names):
        if self.process is not None or self._closing:
            raise RuntimeError("Physics client has already been started or closed")
        request = dict(request, output_path=str(self.directory / "world.xml"))
        payload = {"session": self.session, "export": request, "bodies": list(body_names), "rate_hz": self.rate_hz,
                   "servo_options": request.pop("servo_options", {})}
        request_path = self.directory / "request.json"
        request_path.write_text(json.dumps(payload, allow_nan=False), encoding="utf-8")
        env = os.environ.copy()
        parent = str(Path(__file__).resolve().parents[2])
        env["PYTHONPATH"] = os.pathsep.join(filter(None, (parent, env.get("PYTHONPATH", ""))))
        env.pop("PYTHONHOME", None)
        self._started = time.monotonic()
        self.process = subprocess.Popen([self.executable, "-u", "-m", "robot_arm_library.physics.worker", str(request_path)],
                                        cwd=str(self.directory), env=env, stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        for target in (self._read, self._write, self._drain_stderr):
            thread = threading.Thread(target=target, name=f"physics-{target.__name__}", daemon=True)
            self._threads.append(thread)
            thread.start()

    def _event(self, value):
        try:
            self._events.put_nowait(value)
        except queue.Full:
            self._failed = True

    def _read(self):
        try:
            while True:
                raw = self.process.stdout.readline(MAX_MESSAGE + 1)
                if not raw:
                    return
                message = decode(raw, self.session)
                if message["type"] == "frame":
                    with self._lock:
                        if message["sequence"] <= self._sequence:
                            raise ValueError("Stale physics frame sequence")
                        self._sequence = message["sequence"]
                        self._frame = message
                        self._last_frame_at = time.monotonic()
                elif message["type"] in {"ready", "ack", "error"}:
                    self._event(message)
                else:
                    raise ValueError("Unexpected worker message")
        except Exception as error:
            if not self._closing:
                self._event({"type": "error", "message": f"Physics protocol: {error}"})

    def _write(self):
        try:
            while True:
                raw = self._outgoing.get()
                if raw is None:
                    return
                self.process.stdin.write(raw)
                self.process.stdin.flush()
        except (OSError, ValueError) as error:
            if not self._closing:
                self._event({"type": "error", "message": f"Physics command pipe: {error}"})

    def _drain_stderr(self):
        while True:
            raw = self.process.stderr.readline(2048)
            if not raw:
                return
            self._stderr.append(raw.decode("utf-8", errors="replace"))

    def poll(self):
        result = []
        while True:
            try:
                message = self._events.get_nowait()
            except queue.Empty:
                break
            if message["type"] == "ready":
                if self._ready:
                    message = {"type": "error", "message": "Duplicate physics handshake"}
                self._ready = True
            elif message["type"] == "ack":
                pending = self._pending.pop(message["id"], None)
                if pending is None or pending[0] != message["command"]:
                    message = {"type": "error", "message": "Unexpected physics acknowledgement"}
                elif message["command"] != "targets":
                    self._running = message["command"] in {"start", "resume"}
            result.append(message)
        if not self._closing and self.process is not None:
            now = time.monotonic()
            error = None
            if self._failed:
                error = "Physics event buffer overflow"
            elif self.process.poll() is not None:
                error = f"Physics worker exited ({self.process.returncode}): {''.join(self._stderr)[-4000:]}"
            elif not self._ready and now - self._started > self.startup_timeout:
                error = "Physics worker startup timed out; check the selected Python and source model"
            elif any(now - timestamp > self.command_timeout for _, timestamp in self._pending.values()):
                error = "Physics command timed out"
            elif self._running and now - self._last_frame_at > max(5, self.command_timeout):
                error = "Physics worker stopped publishing frames"
            if error:
                result.append({"type": "error", "message": error})
        return result

    def latest_frame(self):
        with self._lock:
            frame, self._frame = self._frame, None
            return frame

    def command(self, command, **values):
        if command not in COMMANDS or not self._ready or self._closing:
            raise RuntimeError("Physics worker is not ready for this command")
        if len(self._pending) >= 32:
            raise RuntimeError("Too many pending physics commands")
        self._command_id += 1
        message = {**values, "version": VERSION, "session": self.session, "type": "command", "command": command, "id": self._command_id}
        raw = encode(message)
        decode(raw, self.session)
        try:
            self._outgoing.put_nowait(raw)
        except queue.Full as error:
            raise RuntimeError("Physics command buffer is full") from error
        self._pending[self._command_id] = (command, time.monotonic())
        return self._command_id

    @property
    def closed(self):
        return self._closed.is_set()

    def close(self, *, wait=True):
        if not self._closing:
            if self._ready and self.process is not None and self.process.poll() is None:
                try:
                    self.command("stop")
                except RuntimeError:
                    pass
            self._closing = True
            thread = threading.Thread(target=self._cleanup, name="physics-cleanup", daemon=True)
            thread.start()
        if wait:
            self._closed.wait(2)

    def _cleanup(self):
        try:
            process = self.process
            if process is not None:
                try:
                    process.wait(timeout=.3)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=.3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=.3)
                try:
                    self._outgoing.put_nowait(None)
                except queue.Full:
                    pass
                for thread in self._threads:
                    thread.join(timeout=.2)
                for pipe in (process.stdin, process.stdout, process.stderr):
                    pipe.close()
            shutil.rmtree(self.directory, ignore_errors=True)
        finally:
            self._closed.set()
