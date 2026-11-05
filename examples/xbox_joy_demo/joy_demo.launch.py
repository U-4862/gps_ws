from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
    ])
