from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_calibration",
            executable="cameracalibrator",
            name="camera_calibration",
            arguments=[
                "--size", "8x6",
                "--square", "0.025",
                "image:=/camera/color/image_raw",
                "camera_info:=/camera/color/camera_info"
            ],
            output="screen",
        ),
    ])
