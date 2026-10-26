"""Bounded protocol and real headless worker lifecycle tests."""

from dataclasses import asdict
import importlib.util
import io
import json
import sys
import time
from types import SimpleNamespace
import unittest

from robot_arm_library.physics.client import SimulationClient
from robot_arm_library.physics.protocol import MAX_MESSAGE, decode, encode
from robot_arm_library.world_exporter import RobotInstance, SceneMeshInstance
from tests.test_world_physics import _cube_obj


def await_event(client, predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        events = client.poll()
        for event in events:
            if predicate(event):
                return event
            if event["type"] == "error":
                raise AssertionError(event["message"])
        time.sleep(.01)
    raise AssertionError("Worker event timed out")


class ProtocolTests(unittest.TestCase):
    def test_rejects_bad_frames_and_sessions(self):
        frame = {"type": "frame", "version": 1, "session": "test", "sequence": 0, "time": 0, "lag": 0,
                 "positions": [[0, 0, 1]], "quaternions": [[1, 0, 0, 0]]}
        self.assertEqual(decode(encode(frame), "test"), frame)
        for change in ({"session": "wrong"}, {"version": 2}, {"sequence": -1}, {"time": float("nan")},
                       {"positions": [[0, 1]]}, {"quaternions": [[0, 0, 0, 0]]}, {"type": "unknown"}):
            with self.assertRaises(ValueError):
                decode(json.dumps(dict(frame, **change)), "test")
        with self.assertRaises(ValueError):
            decode(b"x" * (MAX_MESSAGE + 1), "test")
        with self.assertRaises(ValueError):
            encode({"value": float("inf")})

    def test_reader_retains_latest_and_rejects_stale(self):
        client = SimulationClient(sys.executable)
        try:
            frame = {"type": "frame", "version": 1, "session": client.session, "sequence": 0, "time": 0, "lag": 0,
                     "positions": [[0, 0, 1]], "quaternions": [[1, 0, 0, 0]]}
            raw = b"".join(encode(dict(frame, sequence=i)) for i in range(100))
            client.process = SimpleNamespace(stdout=io.BytesIO(raw))
            client._read()
            self.assertEqual(client.latest_frame()["sequence"], 99)
            self.assertIsNone(client.latest_frame())
            client.process.stdout = io.BytesIO(encode(frame))
            client._read()
            self.assertIn("Stale", client._events.get_nowait()["message"])
        finally:
            client.process = None
            client.close()


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "MuJoCo is not installed")
class WorkerTests(unittest.TestCase):
    def start_worker(self, **kwargs):
        client = SimulationClient(sys.executable, **kwargs)
        source = client.directory / "cube with spaces.obj"
        _cube_obj(source)
        mesh = SceneMeshInstance("Falling Cube", str(source), (1, 0, 0, 1), position=(0, 0, 2), dynamic=True)
        request = {"version": 1, "robots": [], "scene_meshes": [asdict(mesh)],
                   "world_options": {"gravity": [0, 0, -5], "timestep": .004}}
        client.start(request, [mesh.asset_name])
        return client

    def test_lifecycle_and_cleanup(self):
        for _ in range(2):
            client = self.start_worker()
            directory = client.directory
            try:
                ready = await_event(client, lambda event: event["type"] == "ready")
                self.assertEqual(ready["bodies"], ["scene_Falling_Cube"])
                initial = client.latest_frame()
                self.assertEqual(initial["time"], 0)
                client.command("start")
                await_event(client, lambda event: event["type"] == "ack")
                time.sleep(.15)
                frame = client.latest_frame()
                self.assertGreater(frame["time"], 0)
                self.assertLess(frame["positions"][0][2], 2)
                client.command("pause")
                await_event(client, lambda event: event["type"] == "ack")
                frame = client.latest_frame()
                time.sleep(.08)
                self.assertIsNone(client.latest_frame())
                client.command("resume")
                await_event(client, lambda event: event["type"] == "ack")
                client.command("reset")
                await_event(client, lambda event: event["type"] == "ack" and event["command"] == "reset")
                self.assertEqual(client.latest_frame()["time"], 0)
            finally:
                client.close()
            self.assertFalse(directory.exists())
            self.assertIsNotNone(client.process.poll())
            self.assertTrue(all(not thread.is_alive() for thread in client._threads))
            client.close()

    def test_bad_mapping_model_missing_python_and_crash(self):
        for mode in ("mapping", "model", "missing-python", "crash"):
            client = SimulationClient("/does/not/exist/python" if mode == "missing-python" else sys.executable)
            try:
                source = client.directory / "cube.obj"
                _cube_obj(source)
                mesh = SceneMeshInstance("Cube", str(source), (1, 1, 1, 1))
                if mode == "model":
                    source.write_text("not an obj")
                request = {"version": 1, "robots": [], "scene_meshes": [asdict(mesh)]}
                if mode == "missing-python":
                    with self.assertRaises(OSError):
                        client.start(request, [mesh.asset_name])
                    continue
                client.start(request, ["missing"] if mode == "mapping" else [mesh.asset_name])
                if mode == "crash":
                    await_event(client, lambda event: event["type"] == "ready")
                    client.process.kill()
                error = await_event(client, lambda event: event["type"] == "error")
                self.assertTrue(error["message"])
            finally:
                client.close()
            self.assertFalse(client.directory.exists())

    def test_joint_targets_pause_resume_and_reset(self):
        client = SimulationClient(sys.executable)
        try:
            source = client.directory / "arm.xml"
            source.write_text('<mujoco><compiler angle="radian"/><option gravity="0 0 0"/>'
                              '<worldbody><body name="base"><joint name="hinge" range="-1.69 1.69"/>'
                              '<geom pos=".1 0 0" size=".03"/></body></worldbody></mujoco>')
            robot = RobotInstance("arm", "arm", str(source), (0, 0, 1), (1, 0, 0, 0), {"hinge": 0})
            request = {"version": 1, "robots": [asdict(robot)], "scene_meshes": [],
                       "world_options": {"gravity": [0, 0, 0], "timestep": .002}}
            client.start(request, ["arm_base"])
            ready = await_event(client, lambda event: event["type"] == "ready")
            self.assertTrue(ready["joints"]["arm_hinge"]["available"])
            initial = client.latest_frame()
            for target in (-1.690000057220459, 1.690000057220459):
                client.command("targets", targets={"arm_hinge": target})
                await_event(client, lambda event: event["type"] == "ack")
                self.assertIsNone(client.latest_frame())
            client.command("targets", targets={"arm_hinge": .6})
            await_event(client, lambda event: event["type"] == "ack")
            self.assertIsNone(client.latest_frame())
            client.command("resume")
            await_event(client, lambda event: event["type"] == "ack")
            time.sleep(.3)
            frame = client.latest_frame()
            self.assertGreater(frame["joints"]["arm_hinge"], .3)
            client.command("targets", targets={"arm_hinge": -.5})
            await_event(client, lambda event: event["type"] == "ack")
            self.assertTrue(client._running)
            client.command("reset")
            await_event(client, lambda event: event["type"] == "ack" and event["command"] == "reset")
            frame = client.latest_frame()
            self.assertEqual(frame["time"], 0)
            self.assertEqual(frame["joints"], initial["joints"])
            self.assertFalse(client._running)
            client.command("targets", targets={"arm_hinge": 2})
            error = await_event(client, lambda event: event["type"] == "error")
            self.assertIn("2 not in [-1.69, 1.69]", error["message"])
            self.assertNotIn("installation", error["message"])
        finally:
            client.close()

    def test_startup_and_command_timeout(self):
        client = self.start_worker(startup_timeout=-1)
        try:
            error = next(event for event in client.poll() if event["type"] == "error")
            self.assertIn("startup timed out", error["message"])
        finally:
            client.close()
        client = self.start_worker(command_timeout=-1)
        try:
            await_event(client, lambda event: event["type"] == "ready")
            client.command("pause")
            error = next(event for event in client.poll() if event["type"] == "error")
            self.assertIn("command timed out", error["message"])
        finally:
            client.close()
