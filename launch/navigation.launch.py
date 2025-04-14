#!/usr/bin/env python3
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_speed',
            default_value='0.2',
            description='Forward speed (m/s)'
        ),
        DeclareLaunchArgument(
            'rotate_speed',
            default_value='0.3',
            description='Turning speed (rad/s) for obstacle avoidance'
        ),
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='0.2',
            description='Front obstacle detection threshold (m)'
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
        DeclareLaunchArgument(
            'turning_gain',
            default_value='1.5',
            description='Extra gain for faster turning corrections'
        ),
        DeclareLaunchArgument(
            'sector_width',
            default_value='0.5236',
            description='Half-width of the sector window (radians)'
        ),
        DeclareLaunchArgument(
            'default_front_angle',
            default_value=str(math.pi),
            description='Fallback front angle (radians)'
        ),
        DeclareLaunchArgument(
            'default_right_angle',
            default_value=str(math.pi/2),
            description='Fallback right angle (radians)'
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
                'kp': LaunchConfiguration('kp'),
                'turning_gain': LaunchConfiguration('turning_gain'),
                'sector_width': LaunchConfiguration('sector_width'),
                'default_front_angle': LaunchConfiguration('default_front_angle'),
                'default_right_angle': LaunchConfiguration('default_right_angle')
            }]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
