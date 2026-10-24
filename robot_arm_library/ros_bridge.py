"""ROS 1/2 topic and RViz operations through installed command-line tools."""

from __future__ import annotations

import subprocess
from typing import Callable


class RosCommandError(RuntimeError):
    """A ROS CLI invocation failed."""


def _command(distro: str, ros1: list[str], ros2: list[str]) -> list[str]:
    if distro == "ros1":
        return ros1
    if distro == "ros2":
        return ros2
    raise ValueError(f"Unsupported ROS distro: {distro}")


def _run(command: list[str], runner: Callable, timeout: float) -> str:
    try:
        result = runner(command, capture_output=True, text=True, timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RosCommandError(str(error)) from error
    if result.returncode != 0:
        raise RosCommandError((result.stderr or "").strip() or f"ROS command failed: {' '.join(command)}")
    return (result.stdout or "").strip()


def list_topics(distro: str, runner=subprocess.run) -> list[str]:
    output = _run(_command(distro, ["rostopic", "list"], ["ros2", "topic", "list"]), runner, 5.0)
    return [line.strip() for line in output.splitlines() if line.strip()]


def echo_topic(distro: str, topic: str, runner=subprocess.run, timeout: float = 3.0) -> str:
    if not topic.strip():
        raise ValueError("Topic name is required")
    return _run(
        _command(distro, ["rostopic", "echo", "-n", "1", topic], ["ros2", "topic", "echo", "--once", topic]),
        runner, timeout,
    )


def publish_topic(distro: str, topic: str, message_type: str, payload_yaml: str, runner=subprocess.run) -> None:
    if not topic.strip() or not message_type.strip() or not payload_yaml.strip():
        raise ValueError("Topic, message type, and payload are required")
    _run(
        _command(
            distro,
            ["rostopic", "pub", "-1", topic, message_type, payload_yaml],
            ["ros2", "topic", "pub", "--once", topic, message_type, payload_yaml],
        ), runner, 5.0,
    )


def launch_rviz(distro: str, popen=subprocess.Popen) -> subprocess.Popen:
    command = _command(distro, ["rviz"], ["rviz2"])
    try:
        return popen(command, start_new_session=True)
    except OSError as error:
        raise RosCommandError(str(error)) from error
