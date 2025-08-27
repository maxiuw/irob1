# irob_assignment_1/launch/slam.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('irob_assignment_1')

    # Path to the YAML config file
    params_file_path = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file_path]
    )

    return LaunchDescription([
        slam_toolbox_node
    ])

