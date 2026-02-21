"""Launch Webots simulation with keyboard teleop.

Includes the base sim.launch.py (Webots + vehicle + TF) and adds
teleop_twist_keyboard for manual driving.

Keys: i=forward, ,=backward, j=left, l=right, k=stop, q/z=speed up/down
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('avros_sim')

    return LaunchDescription([
        # Base simulation (Webots + vehicle + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg, 'launch', 'sim.launch.py')
            ),
        ),

        # Keyboard teleop — publishes Twist on /cmd_vel
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            parameters=[{
                'speed': 2.0,
                'turn': 0.5,
                'use_sim_time': True,
            }],
            prefix='xterm -e',
            output='screen',
        ),
    ])
