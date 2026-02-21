"""Launch full autonomous navigation stack in Webots simulation.

Includes:
  - Base sim.launch.py (Webots + vehicle + robot_state_publisher)
  - EKF (robot_localization) — reuses avros_bringup/config/ekf.yaml
  - navsat_transform — reuses avros_bringup/config/navsat.yaml
  - Nav2 (planner, controller, costmaps) — reuses avros_bringup/config/nav2_params.yaml
  - route_planner_node (OSMnx → Nav2 waypoints)

Does NOT start: actuator_node, velodyne, realsense, xsens, ntrip
(Webots provides all sensor data; Car physics handles actuation)
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('avros_sim')
    bringup_pkg = get_package_share_directory('avros_bringup')

    ekf_config = os.path.join(bringup_pkg, 'config', 'ekf.yaml')
    navsat_config = os.path.join(bringup_pkg, 'config', 'navsat.yaml')
    nav2_config = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Base simulation (Webots + vehicle + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, 'launch', 'sim.launch.py')
            ),
        ),

        # robot_localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                ekf_config,
                {'use_sim_time': True},
            ],
            output='screen',
        ),

        # navsat_transform_node: GPS → map frame
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[
                navsat_config,
                {'use_sim_time': True},
            ],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gnss'),
                ('odometry/filtered', '/odometry/filtered'),
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
                'use_sim_time': 'true',
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
                'use_sim_time': True,
                'network_type': 'bike',
                'waypoint_spacing': 5.0,
                'destination_lat': 34.059270,
                'destination_lon': -117.820934,
            }],
            output='screen',
        ),
    ])
