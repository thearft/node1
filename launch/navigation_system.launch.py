#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Optional: Include Explore Lite if installed.
    try:
        explore_lite_launch_file = os.path.join(
            get_package_share_directory('explore_lite'),
            'launch',
            'explore.launch.py'
        )
        include_explore = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(explore_lite_launch_file)
        )
    except Exception as e:
        include_explore = None

    launch_nodes = []
    if include_explore is not None:
        launch_nodes.append(include_explore)

    launch_nodes.extend([
        Node(
            package='node1',
            executable='state_node_executable',
            name='state_server',
            output='screen'
        ),
        Node(
            package='node1',
            executable='navigation_logic_node',
            name='navigation_node',
            output='screen'
        ),
    ])
    
    return LaunchDescription(launch_nodes)
