import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    livox_pkg      = get_package_share_directory('livox_ros_driver2')
    fastlio_pkg    = get_package_share_directory('fast_lio')
    slam_bridge_pkg = get_package_share_directory('slam_bridge')

    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': os.path.join(livox_pkg, 'config', 'MID360_config.json'),
        }],
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items(),
    )

    slam_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_bridge_pkg, 'launch', 'slam_bringup.launch.py')
        ),
    )

    gps_node = Node(
        package='gps',
        executable='gps_node',
        name='gps_node',
        output='screen',
        parameters=[{
            'motion_port': '/dev/ttyUSB0',
            'imu_topic': '/livox/imu',
            'tick_period_ms': 50,
        }],
    )

    return LaunchDescription([
        # 1. livox 驱动立即启动
        livox_node,
        # 2. fast_lio: 等 livox 就绪
        TimerAction(period=2.0, actions=[fast_lio_launch]),
        # 3. slam_bridge: 等 fast_lio 开始发布点云
        TimerAction(period=5.0, actions=[slam_bridge_launch]),
        # 4. gps_node: 等 /odom_corrected 可用
        TimerAction(period=8.0, actions=[gps_node]),
    ])
