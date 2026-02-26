"""Launch costmap test: sensors + static TF + Nav2 controller_server.

Tests the local costmap without localization or the full Nav2 stack.
Uses controller_server which creates the local costmap internally.
Static odom/map TFs stand in until EKF and navsat are ready.

Usage:
  ros2 launch avros_bringup costmap_test.launch.py              # with display
  ros2 launch avros_bringup costmap_test.launch.py rviz:=false   # headless
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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

        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz (set false for headless)'
        ),

        # Sensors: robot_state_publisher + velodyne + realsense + xsens
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

        # Static odom -> base_link (replaced by EKF later)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_publisher',
            arguments=[
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link',
            ],
        ),

        # Static map -> odom (replaced by navsat_transform later)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_publisher',
            arguments=[
                '--frame-id', 'map',
                '--child-frame-id', 'odom',
            ],
        ),

        # controller_server creates and manages the local costmap internally
        # Publishes /local_costmap/costmap from sensor data
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_config],
            output='screen',
        ),

        # Lifecycle manager activates controller_server (and its costmap)
        # bond_timeout 0.0: required for standalone use without full Nav2
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            parameters=[{
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['controller_server'],
            }],
            output='screen',
        ),

        # RViz (disable with rviz:=false for headless)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
