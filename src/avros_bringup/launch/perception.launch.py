"""Launch semantic segmentation perception stack.

Launches:
  - segformer_node (SegFormer-B0 TensorRT inference on D455 color)
  - depth_projection_node (semantic mask + depth -> obstacle PointCloud2)

Requires the TensorRT engine to be pre-built on the Jetson.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('avros_bringup')
    perception_config = os.path.join(pkg_dir, 'config', 'perception_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),

        # SegFormer TensorRT inference
        Node(
            package='avros_perception',
            executable='segformer_node',
            name='segformer_node',
            parameters=[
                perception_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        # Semantic mask + depth -> obstacle point cloud for Nav2
        Node(
            package='avros_perception',
            executable='depth_projection_node',
            name='depth_projection_node',
            parameters=[
                perception_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
    ])
