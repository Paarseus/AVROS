"""Launch full autonomous navigation stack with RTAB-Map SLAM localization.

Alternative to navigation.launch.py — uses RTAB-Map SLAM instead of GPS/EKF
for the map->odom transform. The local EKF (odom->base_link) is kept for
smooth odometry. GPS is fed to RTAB-Map as a graph constraint to anchor the
SLAM map to the route graph datum.

Launches:
  - Everything from sensors.launch.py (URDF, velodyne, realsense, xsens, ntrip)
  - Local EKF (odom -> base_link)
  - ZED X cameras (left, right, back — conditional)
  - RGB-D sync nodes (1 per camera)
  - ICP odometry (VLP-16 scan matching)
  - RTAB-Map SLAM (map -> odom via loop closure + GPS prior)
  - actuator_node (cmd_vel -> Teensy UDP)
  - Nav2 servers (same config as navigation.launch.py)
  - foxglove_bridge

TF tree (same shape as GPS mode, different producer for map->odom):
  map -> odom -> base_link
  (RTAB-Map) (local EKF)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    rtabmap_config = os.path.join(pkg_dir, 'config', 'rtabmap.yaml')
    actuator_config = os.path.join(pkg_dir, 'config', 'actuator_params.yaml')
    graph_file = os.path.join(pkg_dir, 'config', 'cpp_campus_graph.geojson')

    zed_launch_dir = PathJoinSubstitution(
        [FindPackageShare('zed_wrapper'), 'launch']
    )

    # Distro-specific Nav2 config (same logic as navigation.launch.py)
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    if ros_distro == 'humble':
        nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params_humble.yaml')
        bt_xml = os.path.join(pkg_dir, 'config', 'navigate_route_graph_humble.xml')
    else:
        nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
        bt_xml = os.path.join(pkg_dir, 'config', 'navigate_route_graph.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    database_path = LaunchConfiguration('database_path')
    localization = LaunchConfiguration('localization')
    use_zed_left = LaunchConfiguration('use_zed_left')
    use_zed_right = LaunchConfiguration('use_zed_right')
    use_zed_back = LaunchConfiguration('use_zed_back')

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

    nav2_nodes = [
        Node(
            package=package,
            executable=name,
            name=name,
            parameters=[configured_params],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        )
        for package, name in nav2_servers
    ]

    return LaunchDescription([
        # ── Launch Arguments ──────────────────────────────────
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_ntrip', default_value='true',
                              description='Enable NTRIP client for RTK corrections'),
        DeclareLaunchArgument('enable_velodyne', default_value='true'),
        DeclareLaunchArgument('enable_realsense', default_value='true'),
        DeclareLaunchArgument('use_zed_left', default_value='true'),
        DeclareLaunchArgument('use_zed_right', default_value='false',
                              description='Disabled by default (hardware fault)'),
        DeclareLaunchArgument('use_zed_back', default_value='true'),
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/rtabmap.db')),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true=localize against saved map, false=mapping mode'),

        # ── 1. Sensors (URDF, velodyne, realsense, xsens, ntrip) ─
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'sensors.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_ntrip': LaunchConfiguration('enable_ntrip'),
                'enable_velodyne': LaunchConfiguration('enable_velodyne'),
                'enable_realsense': LaunchConfiguration('enable_realsense'),
            }.items(),
        ),

        # ── 2. Local EKF: odom -> base_link (smooth odometry) ────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            parameters=[
                ekf_config,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
            output='screen',
        ),

        # ── 3. ZED X Cameras ─────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([zed_launch_dir, 'zed_camera.launch.py'])
            ]),
            launch_arguments={
                'camera_model': 'zedx',
                'camera_name': 'zed_left',
                'node_name': 'zed_node',
                'serial_number': '43779087',
                'publish_tf': 'false',
                'publish_map_tf': 'false',
                'publish_imu_tf': 'false',
                'publish_urdf': 'false',
                'ros_params_override_path': os.path.join(
                    pkg_dir, 'config', 'zed_left.yaml'),
            }.items(),
            condition=IfCondition(use_zed_left),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([zed_launch_dir, 'zed_camera.launch.py'])
            ]),
            launch_arguments={
                'camera_model': 'zedx',
                'camera_name': 'zed_right',
                'node_name': 'zed_node',
                'serial_number': '47753729',
                'publish_tf': 'false',
                'publish_map_tf': 'false',
                'publish_imu_tf': 'false',
                'publish_urdf': 'false',
                'ros_params_override_path': os.path.join(
                    pkg_dir, 'config', 'zed_right.yaml'),
            }.items(),
            condition=IfCondition(use_zed_right),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([zed_launch_dir, 'zed_camera.launch.py'])
            ]),
            launch_arguments={
                'camera_model': 'zedx',
                'camera_name': 'zed_back',
                'node_name': 'zed_node',
                'serial_number': '49910017',
                'publish_tf': 'false',
                'publish_map_tf': 'false',
                'publish_imu_tf': 'false',
                'publish_urdf': 'false',
                'ros_params_override_path': os.path.join(
                    pkg_dir, 'config', 'zed_back.yaml'),
            }.items(),
            condition=IfCondition(use_zed_back),
        ),

        # ── 4. RGB-D Sync Nodes ───────────────────────────────────
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_realsense',
            namespace='realsense_front',
            parameters=[{'approx_sync': True, 'queue_size': 30}],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ],
            output='screen',
        ),

        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_zed_left',
            namespace='zed_left',
            parameters=[{'approx_sync': True, 'queue_size': 30}],
            remappings=[
                ('rgb/image', '/zed_left/zed_node/rgb/color/rect/image'),
                ('depth/image', '/zed_left/zed_node/depth/depth_registered'),
                ('rgb/camera_info', '/zed_left/zed_node/rgb/color/rect/camera_info'),
            ],
            condition=IfCondition(use_zed_left),
            output='screen',
        ),

        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_zed_right',
            namespace='zed_right',
            parameters=[{'approx_sync': True, 'queue_size': 30}],
            remappings=[
                ('rgb/image', '/zed_right/zed_node/rgb/color/rect/image'),
                ('depth/image', '/zed_right/zed_node/depth/depth_registered'),
                ('rgb/camera_info', '/zed_right/zed_node/rgb/color/rect/camera_info'),
            ],
            condition=IfCondition(use_zed_right),
            output='screen',
        ),

        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_zed_back',
            namespace='zed_back',
            parameters=[{'approx_sync': True, 'queue_size': 30}],
            remappings=[
                ('rgb/image', '/zed_back/zed_node/rgb/color/rect/image'),
                ('depth/image', '/zed_back/zed_node/depth/depth_registered'),
                ('rgb/camera_info', '/zed_back/zed_node/rgb/color/rect/camera_info'),
            ],
            condition=IfCondition(use_zed_back),
            output='screen',
        ),

        # ── 5. ICP Odometry (VLP-16 scan matching) ───────────────
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            parameters=[
                rtabmap_config,
                {'Icp/MaxTranslation': '3.0'},
                {'Odom/Strategy': '0'},
                {'wait_imu_to_init': True},
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('scan_cloud', '/velodyne_points'),
                ('imu', '/imu/data'),
            ],
            output='screen',
        ),

        # ── 6. RTAB-Map SLAM (3 cameras: realsense + zed_left + zed_back) ─
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[
                rtabmap_config,
                {
                    'database_path': database_path,
                    'Mem/IncrementalMemory': 'true',
                    'subscribe_rgb': False,
                    'subscribe_depth': False,
                    'subscribe_scan_cloud': True,
                    'subscribe_scan': False,
                    'subscribe_rgbd': True,
                    'subscribe_imu': True,
                    'rgbd_cameras': 3,
                    'approx_sync': True,
                    'approx_sync_max_interval': 0.5,
                    'topic_queue_size': 30,
                    'sync_queue_size': 30,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('scan_cloud', '/velodyne_points'),
                ('imu', '/imu/data'),
                ('gps/fix', '/gnss'),
                ('rgbd_image0', '/realsense_front/rgbd_image'),
                ('rgbd_image1', '/zed_left/rgbd_image'),
                ('rgbd_image2', '/zed_back/rgbd_image'),
            ],
            output='screen',
        ),

        # ── 7. Actuator bridge ────────────────────────────────────
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

        # ── 8. Nav2 servers ───────────────────────────────────────
        *nav2_nodes,

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

        # ── 9. Foxglove bridge ────────────────────────────────────
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
