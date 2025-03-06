cmake_minimum_required(VERSION 3.8)

# Configuration
set(DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS OFF CACHE BOOL "Disable all provizio/coding_standards checks")
set(PROVIZIO_CODING_STANDARDS_VERSION "v2025.02.10" CACHE STRING "provizio/coding_standards version")
set(BUILD_SIMPLE_NODE ON CACHE BOOL "Enable building of provizio_radar_api_ros2_node")
set(BUILD_LIFECYCLE_NODE ON CACHE BOOL "Enable building of provizio_radar_api_ros2_lifecycle_node")
set(PROVIZIO_RADAR_API "dds" CACHE STRING "Underlying Provizio radar API: dds / udp")
set(PROVIZIO_DDS_VERSION "v2025.02.25" CACHE STRING "provizio_dds version (for dds API)")
set(PROVIZIO_API_CORE_VERSION "v2024.04.16" CACHE STRING "provizio_radar_api_core version (for udp API)")

project(provizio_radar_api_ros2)

# Standard Config (see https://github.com/provizio/coding_standards)
if(NOT DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS)
    file(DOWNLOAD
        "https://raw.githubusercontent.com/provizio/coding_standards/${PROVIZIO_CODING_STANDARDS_VERSION}/cpp/cmake/StandardConfig.cmake"
        "${CMAKE_BINARY_DIR}/StandardConfig.cmake" TLS_VERIFY ON)
    include(${CMAKE_BINARY_DIR}/StandardConfig.cmake)
    StandardConfig(SAFETY_CRITICAL)
else(NOT DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS)
    set(CMAKE_CXX_STANDARD 17)
endif(NOT DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
if(BUILD_LIFECYCLE_NODE)
    find_package(rclcpp_lifecycle REQUIRED)
endif(BUILD_LIFECYCLE_NODE)

# Common include directories
include_directories(include)

# Appropriate radar_api_ros2_wrapper
add_subdirectory(src/provizio_radar_api_ros2_${PROVIZIO_RADAR_API})

# provizio_radar_api_ros2_common
add_library(provizio_radar_api_ros2_common STATIC src/common.cpp)

# provizio_radar_api_ros2_node
if(BUILD_SIMPLE_NODE)
    add_executable(provizio_radar_api_ros2_node src/provizio_radar_api_ros2_node.cpp)
    ament_target_dependencies(provizio_radar_api_ros2_node rclcpp sensor_msgs)
    target_link_libraries(provizio_radar_api_ros2_node provizio_radar_api_ros2_common provizio_radar_api_ros2_${PROVIZIO_RADAR_API})
endif(BUILD_SIMPLE_NODE)

# provizio_radar_api_ros2_lifecycle_node
if(BUILD_LIFECYCLE_NODE)
    add_executable(provizio_radar_api_ros2_lifecycle_node src/provizio_radar_api_ros2_lifecycle_node.cpp)
    ament_target_dependencies(provizio_radar_api_ros2_lifecycle_node rclcpp rclcpp_lifecycle sensor_msgs)
    target_link_libraries(provizio_radar_api_ros2_lifecycle_node provizio_radar_api_ros2_common provizio_radar_api_ros2_${PROVIZIO_RADAR_API})
endif(BUILD_LIFECYCLE_NODE)

ament_export_dependencies(rclcpp sensor_msgs)

ament_package()
