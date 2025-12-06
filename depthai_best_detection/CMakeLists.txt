cmake_minimum_required(VERSION 3.5)
project(depthai_best_detection)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(best_detection_msgs REQUIRED)

install(
  DIRECTORY depthai_best_detection
  DESTINATION lib/python3.10/site-packages
)

ament_package()
