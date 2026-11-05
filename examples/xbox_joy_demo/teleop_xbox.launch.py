import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("teleop_twist_joy"),
        "config",
        "xbox.config.yaml",
    )

    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{
                "device_id": 0,
                "deadzone": 0.05,
                "autorepeat_rate": 100.0,
                "coalesce_interval_ms": 1,
            }],
            output="screen",
        ),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            parameters=[config, {"publish_stamped_twist": False}],
            output="screen",
        ),
    ])
