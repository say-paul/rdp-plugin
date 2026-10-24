"""Exercise Robot Build without running podman or touching a disk."""

import subprocess
import sys
import tempfile
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as addon

robot_arm_library.register()
try:
    with tempfile.TemporaryDirectory() as directory:
        original_directory = addon._blend_directory
        addon._blend_directory = lambda: Path(directory)
        try:
            scene = bpy.context.scene
            scene.robot_build_oci_name = "caryam-robot"
            assert bpy.ops.robot_library.generate_containerfile() == {"FINISHED"}
            containerfile = Path(directory) / "build" / "Containerfile"
            assert containerfile.read_text(encoding="utf-8").startswith("FROM ")
            assert (Path(directory) / "build" / "caryam_payload" / "entrypoint.sh").exists()
            commands = []
            original_popen = subprocess.Popen
            subprocess.Popen = lambda command, **kwargs: commands.append(command)
            try:
                assert bpy.ops.robot_library.build_container_image() == {"FINISHED"}
                assert commands[-1][:2] == ["podman", "build"]
                scene.robot_build_target_disk = "/dev/sda"
                assert bpy.ops.robot_library.flash_bootc_image() == {"CANCELLED"}
                assert len(commands) == 1
                assert bpy.ops.robot_library.flash_bootc_image(confirmed=True) == {"FINISHED"}
                assert commands[-1][-1] == "/dev/sda"
            finally:
                subprocess.Popen = original_popen
        finally:
            addon._blend_directory = original_directory
    print("Robot Build container operator test passed")
finally:
    robot_arm_library.unregister()
