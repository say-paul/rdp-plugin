"""Headless MuJoCo child process. Only stdout contains protocol messages."""

from __future__ import annotations

from contextlib import redirect_stdout
import json
from pathlib import Path
import queue
import sys
import threading
import time

from .protocol import MAX_MESSAGE, VERSION, decode, encode
from .runtime import ResetState, initialize


def run(request_path):
    payload = json.loads(Path(request_path).read_text(encoding="utf-8"))
    session = payload["session"]

    def send(kind, **values):
        sys.stdout.buffer.write(encode({"version": VERSION, "session": session, "type": kind, **values}))
        sys.stdout.buffer.flush()

    try:
        with redirect_stdout(sys.stderr):
            import mujoco
            from ..world_export_worker import export_request
            output, manifest_path, _ = export_request(payload["export"])
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            model = mujoco.MjModel.from_xml_path(str(output))
            data = initialize(mujoco, model, manifest)
        names = payload["bodies"]
        if not names or len(set(names)) != len(names):
            raise ValueError("Body bindings must be nonempty and unique")
        ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in names]
        if any(body < 1 for body in ids):
            raise ValueError("Missing MuJoCo body mapping: " + ", ".join(name for name, body in zip(names, ids) if body < 1))
        for robot in manifest["robots"]:
            sensors = {robot["prefix"] + mount["mount_name"] for mount in robot.get("sensors", [])}
            for body in range(1, model.nbody):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
                if name and name.startswith(robot["prefix"]) and name not in sensors and name not in names:
                    raise ValueError(f"Unmapped MuJoCo robot body: {name}; reimport the compiled robot")
        reset = ResetState(mujoco, model, data)
        commands = queue.Queue(maxsize=64)
        reader_error = []

        def receive():
            try:
                while True:
                    raw = sys.stdin.buffer.readline(MAX_MESSAGE + 1)
                    if not raw:
                        commands.put_nowait(None)
                        return
                    message = decode(raw, session)
                    if message["type"] != "command":
                        raise ValueError("Expected physics command")
                    commands.put_nowait(message)
            except Exception as error:
                reader_error.append(error)

        threading.Thread(target=receive, name="physics-commands", daemon=True).start()
        sequence = 0
        running = False
        timestep = float(model.opt.timestep)
        rate = float(payload["rate_hz"])
        if not 1 <= rate <= 120:
            raise ValueError("Viewport update rate must be between 1 and 120 Hz")
        next_step = next_frame = time.monotonic()
        lag = 0.0

        def frame():
            nonlocal sequence
            mujoco.mj_forward(model, data)
            send("frame", sequence=sequence, time=float(data.time), lag=lag,
                 positions=data.xpos[ids].tolist(), quaternions=data.xquat[ids].tolist())
            sequence += 1

        frame()
        send("ready", bodies=names, body_ids=ids)
        while True:
            if reader_error:
                raise RuntimeError(f"Physics command stream failed: {reader_error[0]}")
            now = time.monotonic()
            timeout = max(0, min(.02, next_step - now)) if running else .02
            try:
                message = commands.get(timeout=timeout)
            except queue.Empty:
                message = False
            if message is None:
                return 0
            if isinstance(message, Exception):
                raise message
            if message:
                command = message["command"]
                if command == "stop":
                    send("ack", command=command, id=message["id"])
                    return 0
                if command == "reset":
                    reset.restore(mujoco, model, data)
                    running = False
                    lag = 0.0
                elif command == "pause":
                    running = False
                elif command in {"start", "resume"}:
                    running = True
                next_step = next_frame = time.monotonic()
                frame()
                send("ack", command=command, id=message["id"])
            if not running:
                continue
            steps = 0
            now = time.monotonic()
            while now >= next_step and steps < 20:
                mujoco.mj_step(model, data)
                next_step += timestep
                steps += 1
            lag = max(0, now - next_step)
            if lag > .25:
                next_step = now
            if now >= next_frame:
                frame()
                next_frame = now + 1.0 / rate
    except Exception as error:
        send("error", message=f"{type(error).__name__}: {error}. Check the selected external Python and MuJoCo installation.")
        return 1


if __name__ == "__main__":
    raise SystemExit(run(sys.argv[1]))
