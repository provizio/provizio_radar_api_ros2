cmake_minimum_required(VERSION 3.8)

# Configuration
set(PROVIZIO_CODING_STANDARDS_VERSION "v2025.02.10" CACHE STRING "provizio/coding_standards version")
set(BUILD_SIMPLE_NODE ON CACHE BOOL "Enable building of provizio_radar_api_ros2_node")
set(BUILD_LIFECYCLE_NODE ON CACHE BOOL "Enable building of provizio_radar_api_ros2_lifecycle_node")

project(provizio_radar_api_ros2)

# Standard Config
file(DOWNLOAD
    "https://raw.githubusercontent.com/provizio/coding_standards/${PROVIZIO_CODING_STANDARDS_VERSION}/cpp/cmake/StandardConfig.cmake"
    "${CMAKE_BINARY_DIR}/StandardConfig.cmake" TLS_VERIFY ON)
include(${CMAKE_BINARY_DIR}/StandardConfig.cmake)
StandardConfig(SAFETY_CRITICAL)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
if(BUILD_LIFECYCLE_NODE)
    find_package(rclcpp_lifecycle REQUIRED)
endif(BUILD_LIFECYCLE_NODE)

# provizio_radar_api_ros2_node
if(BUILD_SIMPLE_NODE)
    add_executable(provizio_radar_api_ros2_node src/provizio_radar_api_ros2_node.cpp)
    ament_target_dependencies(provizio_radar_api_ros2_node rclcpp sensor_msgs)
endif(BUILD_SIMPLE_NODE)

# provizio_radar_api_ros2_lifecycle_node
if(BUILD_LIFECYCLE_NODE)
    add_executable(provizio_radar_api_ros2_lifecycle_node src/provizio_radar_api_ros2_lifecycle_node.cpp)
    ament_target_dependencies(provizio_radar_api_ros2_lifecycle_node rclcpp rclcpp_lifecycle sensor_msgs)
endif(BUILD_LIFECYCLE_NODE)

ament_export_dependencies(rclcpp sensor_msgs)

ament_package()
