"""Prepare bootc Containerfiles and podman/docker command arguments."""

from __future__ import annotations

import json
import re
from pathlib import Path

DEFAULT_BASE_IMAGE = "quay.io/centos-bootc/centos-bootc:stream9"
_PACKAGE = re.compile(r"^[A-Za-z0-9_.+:-]+$")


def image_tag(name: str) -> str:
    value = name.strip()
    if not value or any(character.isspace() for character in value) or "@" in value:
        raise ValueError("Provide a valid OCI image name")
    return value if ":" in value.rsplit("/", 1)[-1] else f"{value}:latest"


def render_containerfile(base_image: str, packages: list[str], entrypoint: str) -> str:
    if not base_image.strip() or "\n" in base_image:
        raise ValueError("Base image must be a single nonempty line")
    if any(not _PACKAGE.fullmatch(package) for package in packages):
        raise ValueError("Package names must contain only package characters")
    if not entrypoint.startswith("/") or "\n" in entrypoint:
        raise ValueError("Entrypoint must be an absolute path")
    lines = [f"FROM {base_image}"]
    if packages:
        lines.append(f"RUN dnf install -y {' '.join(packages)} && dnf clean all")
    lines.extend(("COPY caryam_payload/ /opt/caryam/", f"ENTRYPOINT {json.dumps([entrypoint])}"))
    return "\n".join(lines) + "\n"


def write_containerfile(directory: Path, content: str) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "Containerfile"
    path.write_text(content, encoding="utf-8")
    return path


def build_image_command(tool: str, tag: str, containerfile: Path, context_dir: Path) -> list[str]:
    if tool not in {"podman", "docker"} or not tag.strip():
        raise ValueError("Choose podman or docker and provide an image name")
    return [tool, "build", "-t", tag, "-f", str(containerfile), str(context_dir)]


def bootc_install_command(tool: str, image: str, target_disk: str) -> list[str]:
    if tool not in {"podman", "docker"} or not image.strip():
        raise ValueError("Choose podman or docker and provide an image name")
    if not target_disk.startswith("/dev/") or target_disk == "/dev/":
        raise ValueError("Target disk must be an explicit /dev/ device path")
    return [
        tool, "run", "--rm", "--privileged", "--pid=host", "--ipc=host", "-v", "/dev:/dev",
        "-v", "/var/lib/containers:/var/lib/containers", "--security-opt", "label=type:unconfined_t",
        "--entrypoint", "bootc", image, "install", "to-disk", "--wipe", target_disk,
    ]
