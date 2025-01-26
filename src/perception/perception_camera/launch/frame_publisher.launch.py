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
                FindPackageShare('perception_camera'),
                'config',
                'frame_publisher.yaml'
            ]),
            description=PathJoinSubstitution([
                FindPackageShare('perception_camera'),
                'config',
                'frame_publisher.yaml'
            ]),
        ),
        Node(
            package='perception_camera',
            executable='frame_publisher',
            name='frame_publisher',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])