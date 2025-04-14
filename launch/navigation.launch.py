#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_speed',
            default_value='0.05',
            description='Forward speed (m/s)'
        ),
        DeclareLaunchArgument(
            'rotate_speed',
            default_value='0.1',
            description='Turning speed (rad/s)'
        ),
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='0.30',
            description='Obstacle detection threshold (m)'
        ),
        DeclareLaunchArgument(
            'wall_follow_distance',
            default_value='1.0',
            description='Desired wall-following distance (m)'
        ),
        DeclareLaunchArgument(
            'kp',
            default_value='1.0',
            description='Proportional gain for wall following'
        ),
        Node(
            package='node1',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{
                'drive_speed': LaunchConfiguration('drive_speed'),
                'rotate_speed': LaunchConfiguration('rotate_speed'),
                'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
                'wall_follow_distance': LaunchConfiguration('wall_follow_distance'),
                'kp': LaunchConfiguration('kp')
            }]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
