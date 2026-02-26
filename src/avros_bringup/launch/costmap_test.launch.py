"""Launch costmap test: sensors + static TF + Nav2 costmap + RViz.

Tests the costmap/occupancy grid without localization or full Nav2.
Uses a static odom -> base_link transform (vehicle stationary).

Usage:
  ros2 launch avros_bringup costmap_test.launch.py

When localization is ready, switch to navigation.launch.py instead.
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
    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'costmap_test.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        # Sensors: robot_state_publisher + velodyne + realsense + xsens
        # Provides: base_link -> {velodyne, camera_link, imu_link} static TFs
        # Provides: /velodyne_points, /camera/camera/..., /imu/data
        # Also sets RMW_IMPLEMENTATION and CYCLONEDDS_URI env vars
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_ntrip': 'false',
            }.items(),
        ),

        # Static odom -> base_link (identity, stationary vehicle)
        # Replaced by EKF when localization is ready
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_publisher',
            arguments=[
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link',
            ],
        ),

        # Static map -> odom (identity, no GPS)
        # Replaced by navsat_transform when localization is ready
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_publisher',
            arguments=[
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
        ),

        # Nav2 local costmap (standalone, no planner/controller/BT)
        # namespace='local_costmap' + name='local_costmap' matches the
        # YAML structure: local_costmap.local_costmap.ros__parameters
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            namespace='local_costmap',
            parameters=[nav2_config],
            output='screen',
        ),

        # Lifecycle manager to auto-activate the costmap node
        # Without this, the costmap stays in UNCONFIGURED state
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            parameters=[{
                'autostart': True,
                'node_names': ['local_costmap/local_costmap'],
            }],
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
