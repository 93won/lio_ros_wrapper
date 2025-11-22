"""
LIO ROS2 Launch File (Avia LiDAR)

Author: Seungwon Choi
Email: csw3575@snu.ac.kr
Date: 2025-11-22
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('lio_ros_wrapper')
    # Use config from lidar_inertial_odometry subdirectory
    default_config = os.path.join(pkg_share, 'config', 'lidar_inertial_odometry', 'avia.yaml')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/livox/avia/imu',
            description='IMU data topic'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/livox/avia/lidar',
            description='LiDAR point cloud topic'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='LIO configuration file path'
        ),
        DeclareLaunchArgument(
            'init_imu_samples',
            default_value='100',
            description='Number of IMU samples for gravity initialization'
        ),

        # LIO Node
        Node(
            package='lio_ros_wrapper',
            executable='lio_node',
            name='lio_node',
            output='screen',
            parameters=[{
                'imu_topic': LaunchConfiguration('imu_topic'),
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'config_file': LaunchConfiguration('config_file'),
                'init_imu_samples': LaunchConfiguration('init_imu_samples'),
            }]
        ),
    ])
