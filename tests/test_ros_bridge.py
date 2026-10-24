import subprocess
import unittest
from unittest.mock import MagicMock

from robot_arm_library.ros_bridge import RosCommandError, echo_topic, launch_rviz, list_topics, publish_topic


class RosBridgeTests(unittest.TestCase):
    def test_topic_commands_and_errors(self):
        runner = MagicMock(return_value=subprocess.CompletedProcess([], 0, stdout="/joint_states\n/tf\n", stderr=""))
        self.assertEqual(list_topics("ros2", runner=runner), ["/joint_states", "/tf"])
        runner.assert_called_once_with(["ros2", "topic", "list"], capture_output=True, text=True, timeout=5.0, check=False)
        runner.return_value = subprocess.CompletedProcess([], 0, stdout="data: 1\n", stderr="")
        self.assertEqual(echo_topic("ros1", "/joint_states", runner=runner), "data: 1")
        publish_topic("ros1", "/cmd", "std_msgs/Float64", "data: 0.5", runner=runner)
        self.assertEqual(runner.call_args.args[0], ["rostopic", "pub", "-1", "/cmd", "std_msgs/Float64", "data: 0.5"])
        runner.return_value = subprocess.CompletedProcess([], 1, stdout="", stderr="no master")
        with self.assertRaisesRegex(RosCommandError, "no master"):
            list_topics("ros1", runner=runner)
        with self.assertRaises(ValueError):
            list_topics("ros3", runner=runner)

    def test_rviz_uses_distro_executable(self):
        popen = MagicMock()
        launch_rviz("ros2", popen=popen)
        popen.assert_called_once_with(["rviz2"], start_new_session=True)


if __name__ == "__main__":
    unittest.main()
