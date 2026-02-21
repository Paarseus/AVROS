"""Launch Webots simulation with AVROS vehicle.

Base simulation launch — starts Webots with the campus world, connects
the vehicle driver plugin, and publishes the TF tree via robot_state_publisher.

Sensor topics (/velodyne_points, /imu/data, /gnss, /camera/...) are
published by webots_ros2_driver device plugins configured in
resource/avros_webots.urdf.
"""

import os

import launch
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    sim_pkg = get_package_share_directory('avros_sim')
    bringup_pkg = get_package_share_directory('avros_bringup')

    robot_description_path = os.path.join(
        sim_pkg, 'resource', 'avros_webots.urdf')
    urdf_file = os.path.join(bringup_pkg, 'urdf', 'avros.urdf.xacro')

    webots = WebotsLauncher(
        world=os.path.join(sim_pkg, 'worlds', 'cpp_campus.wbt'),
        ros2_supervisor=True,
    )

    avros_driver = WebotsController(
        robot_name='avros',
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True},
        ],
        respawn=True,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]), value_type=str
            ),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        avros_driver,
        robot_state_publisher,

        # Shut down launch when Webots exits
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(
                        event=launch.events.Shutdown()
                    ),
                ],
            )
        ),
    ])
