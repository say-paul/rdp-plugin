"""Exercise ROS controls with a stubbed CLI bridge."""

import sys
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.ros_bridge as ros_bridge

robot_arm_library.register()
try:
    scene = bpy.context.scene
    scene.robot_ros_distro = "ros2"
    original = (ros_bridge.list_topics, ros_bridge.echo_topic, ros_bridge.publish_topic, ros_bridge.launch_rviz)
    calls = []
    ros_bridge.list_topics = lambda distro: ["/joint_states", "/tf"]
    ros_bridge.echo_topic = lambda distro, topic: "data: 1.0"
    ros_bridge.publish_topic = lambda *args: calls.append(args)
    ros_bridge.launch_rviz = lambda distro: calls.append(("rviz", distro))
    try:
        assert bpy.ops.robot_library.ros_list_topics() == {"FINISHED"}
        assert scene.robot_ros_status == "/joint_states; /tf"
        assert bpy.ops.robot_library.ros_echo_topic() == {"FINISHED"}
        assert scene.robot_ros_status == "data: 1.0"
        scene.robot_ros_topic = "/cmd"
        scene.robot_ros_message_type = "std_msgs/Float64"
        scene.robot_ros_payload = "data: 0.5"
        assert bpy.ops.robot_library.ros_publish_topic() == {"FINISHED"}
        assert calls[-1] == ("ros2", "/cmd", "std_msgs/Float64", "data: 0.5")
        assert bpy.ops.robot_library.ros_launch_rviz() == {"FINISHED"}
        assert calls[-1] == ("rviz", "ros2")
    finally:
        ros_bridge.list_topics, ros_bridge.echo_topic, ros_bridge.publish_topic, ros_bridge.launch_rviz = original
    print("ROS control operator test passed")
finally:
    robot_arm_library.unregister()
