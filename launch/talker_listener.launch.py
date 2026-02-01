from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    talker = Node(
            package="ros2_sample_pkg",
        executable="talker",
        name="talker",
        output="screen",
    )
    listener = Node(
            package="ros2_sample_pkg",
        executable="listener",
        name="listener",
        output="screen",
    )
    ld.add_action(talker)
    ld.add_action(listener)

    return ld
