import unittest

from robot_arm_library.sensor_compilers.mujoco import compile_depth_camera, compile_lidar
from robot_arm_library.sensor_spec import CameraSensorSpec, LidarSensorSpec


class MujocoSensorCompilerTests(unittest.TestCase):
    def test_lidar_fan_includes_all_channels_and_samples(self):
        sites, rangefinders = compile_lidar(LidarSensorSpec(channels=2, horizontal_samples=4, horizontal_fov=90, vertical_fov=20, range_max=12), "front")
        self.assertEqual(len(sites), 8)
        self.assertEqual(len(rangefinders), 8)
        self.assertEqual(rangefinders[0].get("site"), sites[0].get("name"))
        self.assertEqual(rangefinders[0].get("cutoff"), "12")
        self.assertNotEqual(sites[0].get("quat"), sites[4].get("quat"))

    def test_depth_camera_uses_physical_fov(self):
        spec = CameraSensorSpec(focal_length=1.93, vertical_aperture=2.453)
        camera = compile_depth_camera(spec, "chest")
        self.assertEqual(camera.tag, "camera")
        self.assertAlmostEqual(float(camera.get("fovy")), spec.vertical_fov_degrees(), places=4)


if __name__ == "__main__":
    unittest.main()
