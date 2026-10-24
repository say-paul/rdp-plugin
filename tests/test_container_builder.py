import tempfile
import unittest
from pathlib import Path

from robot_arm_library.container_builder import (
    DEFAULT_BASE_IMAGE, bootc_install_command, build_image_command, image_tag, render_containerfile, write_containerfile,
)


class ContainerBuilderTests(unittest.TestCase):
    def test_render_and_write_valid_containerfile(self):
        content = render_containerfile(DEFAULT_BASE_IMAGE, ["ros-humble-desktop"], "/opt/caryam/entrypoint.sh")
        self.assertIn(f"FROM {DEFAULT_BASE_IMAGE}", content)
        self.assertIn("RUN dnf install -y ros-humble-desktop", content)
        self.assertIn('ENTRYPOINT ["/opt/caryam/entrypoint.sh"]', content)
        with tempfile.TemporaryDirectory() as directory:
            path = write_containerfile(Path(directory), content)
            self.assertEqual(path.read_text(encoding="utf-8"), content)
        with self.assertRaises(ValueError):
            render_containerfile(DEFAULT_BASE_IMAGE, ["bad\nRUN false"], "/entrypoint")

    def test_build_and_flash_commands(self):
        self.assertEqual(image_tag("localhost:5000/caryam"), "localhost:5000/caryam:latest")
        self.assertEqual(image_tag("caryam:v1"), "caryam:v1")
        self.assertEqual(
            build_image_command("podman", "caryam:latest", Path("/tmp/build/Containerfile"), Path("/tmp/build")),
            ["podman", "build", "-t", "caryam:latest", "-f", "/tmp/build/Containerfile", "/tmp/build"],
        )
        command = bootc_install_command("podman", "caryam:latest", "/dev/sda")
        self.assertIn("--entrypoint", command)
        self.assertIn("bootc", command)
        self.assertEqual(command[-1], "/dev/sda")
        with self.assertRaises(ValueError):
            bootc_install_command("podman", "caryam:latest", "/tmp/file")


if __name__ == "__main__":
    unittest.main()
