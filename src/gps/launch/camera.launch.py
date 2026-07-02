from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='cam1',
            parameters=[{
                'camera_name': 'cam1',
                'enable_color': True,
                'enable_depth': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_accel': False,
                'enable_gyro': False
            }],
            output='screen'
        )
    ])
