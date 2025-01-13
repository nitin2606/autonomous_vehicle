cmake_minimum_required(VERSION 3.5)
project(monocular_vslam_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
