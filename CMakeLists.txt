cmake_minimum_required(VERSION 3.8)

set(ProvizioCodingStandardsVersion "v2025.02.10")

project(provizio_radar_api_ros2)

# Standard Config
file(DOWNLOAD
    "https://raw.githubusercontent.com/provizio/coding_standards/${ProvizioCodingStandardsVersion}/cpp/cmake/StandardConfig.cmake"
    "${CMAKE_BINARY_DIR}/StandardConfig.cmake" TLS_VERIFY ON)
include(${CMAKE_BINARY_DIR}/StandardConfig.cmake)
StandardConfig(SAFETY_CRITICAL)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Executable
add_executable(${PROJECT_NAME} src/provizio_radar_api_ros2_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)

ament_export_dependencies(rclcpp sensor_msgs)

ament_package()
