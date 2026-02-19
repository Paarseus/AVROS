"""Launch sensors + EKF + navsat_transform for localized operation.

Launches:
  - Everything from sensors.launch.py
  - robot_localization EKF (fuses IMU + GPS)
  - navsat_transform_node (GPS -> map frame)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_dir, 'config', 'navsat.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'enable_ntrip', default_value='true',
            description='Enable NTRIP client for RTK corrections'
        ),

        # Include sensors launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_ntrip': LaunchConfiguration('enable_ntrip'),
            }.items(),
        ),

        # robot_localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                ekf_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),

        # navsat_transform_node: GPS -> map frame
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[
                navsat_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gnss'),
                ('odometry/filtered', '/odometry/filtered'),
            ],
            output='screen',
        ),
    ])
