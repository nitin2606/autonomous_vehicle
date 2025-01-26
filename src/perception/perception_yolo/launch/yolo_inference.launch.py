import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('perception_yolo'),
                'config',
                'frame_subscriber.yaml'
            ]),
            description=PathJoinSubstitution([
                FindPackageShare('perception_yolo'),
                'config',
                'frame_subscriber.yaml'
            ]),
        ),
        Node(
            package='perception_yolo',
            executable='frame_subscriber',
            name='frame_subscriber',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])