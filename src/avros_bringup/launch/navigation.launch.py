"""Launch full autonomous navigation stack.

Launches:
  - Everything from localization.launch.py (sensors + EKF + navsat)
  - actuator_node (cmd_vel -> Teensy UDP)
  - Nav2 (planner, controller, costmaps, behavior tree, lifecycle manager)
  - route_planner_node (OSMnx -> Nav2 waypoints)
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
    actuator_config = os.path.join(pkg_dir, 'config', 'actuator_params.yaml')
    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'enable_ntrip', default_value='true',
            description='Enable NTRIP client for RTK corrections'
        ),

        # Include localization launch (sensors + EKF + navsat)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_ntrip': LaunchConfiguration('enable_ntrip'),
            }.items(),
        ),

        # Actuator bridge
        Node(
            package='avros_control',
            executable='actuator_node',
            name='actuator_node',
            parameters=[
                actuator_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),

        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_config,
                'autostart': 'true',
            }.items(),
        ),

        # Route planner
        Node(
            package='avros_navigation',
            executable='route_planner_node',
            name='route_planner_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'network_type': 'bike',
                'waypoint_spacing': 5.0,
                'destination_lat': 34.059270,
                'destination_lon': -117.820934,
            }],
            output='screen',
        ),
    ])
