from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="monocular_camera",
            parameters=[
                {"image_size": [640, 480]},  # Camera resolution
                {"frame_rate": 30.0},        # Frame rate
            ],
            output="screen",
            remappings=[
                ("/image_raw", "/camera/image_raw"),        # Remap raw image topic
                ("/camera_info", "/camera/camera_info"),    # Remap camera info topic
            ],
        ),

        # ORB-SLAM3 Node (Monocular)
        Node(
            package="orb_slam3_ros2",  # ROS2 package for ORB-SLAM3
            executable="mono",          # Executable for monocular SLAM
            name="orb_slam3",
            parameters=[
                {"voc_file": "config/ORBvoc.txt"},  # Path to the vocabulary file
                {"camera_file": "config/camera_calibration.yaml"},  # Camera calibration file
                {"camera_topic": "/camera/image_raw"},  # Topic for camera input
            ],
            output="screen",
        ),
    ])
