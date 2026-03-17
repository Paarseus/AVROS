"""Launch full autonomous navigation stack.

Launches:
  - Everything from localization.launch.py (sensors + EKF + navsat)
  - actuator_node (cmd_vel -> Teensy UDP)
  - Nav2 servers (controller, smoother, planner, route_server, behavior,
    velocity_smoother, bt_navigator)
  - Single lifecycle manager (transitions servers in order)
  - foxglove_bridge (WebSocket server for remote visualization)

Nav2 servers are launched directly (not via nav2_bringup) so that
route_server can be included in the lifecycle manager's ordered node list.
The lifecycle manager transitions nodes sequentially, guaranteeing
route_server is active before bt_navigator loads the BT XML.

Supports both ROS2 Humble and Jazzy:
  - BT XML: v3 format on Humble, v4 on Jazzy
  - plugin_lib_names: explicit list on Humble, omitted on Jazzy
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    actuator_config = os.path.join(pkg_dir, 'config', 'actuator_params.yaml')
    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    graph_file = os.path.join(pkg_dir, 'config', 'cpp_campus_graph.geojson')

    # Select BT XML based on ROS distro (v3 for Humble, v4 for Jazzy)
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    if ros_distro == 'humble':
        bt_xml = os.path.join(pkg_dir, 'config', 'navigate_route_graph_humble.xml')
    else:
        bt_xml = os.path.join(pkg_dir, 'config', 'navigate_route_graph.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Rewrite nav2_params.yaml with resolved paths and use_sim_time
    configured_params = RewrittenYaml(
        source_file=nav2_config,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': bt_xml,
            'default_nav_through_poses_bt_xml': bt_xml,
            'graph_filepath': graph_file,
        },
        convert_types=True,
    )

    # Nav2 servers — lifecycle manager transitions these in order,
    # so route_server activates before bt_navigator validates the BT XML
    nav2_servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_route', 'route_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ]

    lifecycle_nodes = [name for _, name in nav2_servers]

    # Humble requires explicit BT plugin list; Jazzy auto-discovers them
    humble_bt_plugins = {
        'plugin_lib_names': [
            'nav2_compute_route_bt_node',
            'nav2_follow_path_action_bt_node',
            'nav2_is_path_valid_condition_bt_node',
            'nav2_wait_action_bt_node',
            'nav2_clear_costmap_service_bt_node',
            'nav2_goal_updated_condition_bt_node',
            'nav2_globally_updated_goal_condition_bt_node',
            'nav2_rate_controller_bt_node',
            'nav2_recovery_node_bt_node',
            'nav2_pipeline_sequence_bt_node',
            'nav2_round_robin_node_bt_node',
            'nav2_goal_reached_condition_bt_node',
        ],
    }

    nav2_nodes = []
    for package, name in nav2_servers:
        if name == 'bt_navigator' and ros_distro == 'humble':
            params = [configured_params, humble_bt_plugins]
        else:
            params = [configured_params]
        nav2_nodes.append(Node(
            package=package,
            executable=name,
            name=name,
            parameters=params,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        DeclareLaunchArgument(
            'enable_ntrip', default_value='true',
            description='Enable NTRIP client for RTK corrections'
        ),

        # Localization (sensors + EKF + navsat)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'localization.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
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
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        # Nav2 servers
        *nav2_nodes,

        # Lifecycle manager — transitions nodes in list order
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
            output='screen',
        ),

        # Foxglove bridge (WebSocket on port 8765 for remote visualization)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ])
