cmake_minimum_required(VERSION 3.13)

# Configuration
set(DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS OFF CACHE BOOL "Disable all provizio/coding_standards checks")
set(PROVIZIO_CODING_STANDARDS_VERSION "1.10.0" CACHE STRING "provizio/coding_standards version")
set(BUILD_SIMPLE_NODE ON CACHE BOOL "Enable building of provizio_radar_api_ros2_node")
set(BUILD_LIFECYCLE_NODE ON CACHE BOOL "Enable building of provizio_radar_api_ros2_lifecycle_node")
set(PROVIZIO_RADAR_API "dds" CACHE STRING "Underlying Provizio radar API: dds / udp")
set(PROVIZIO_DDS_VERSION "1.8.0" CACHE STRING "provizio_dds version (for dds API)")
set(PROVIZIO_API_CORE_VERSION "v2024.04.16" CACHE STRING "provizio_radar_api_core version (for udp API)")

set(PROJECT "provizio_radar_api_ros2")
project(${PROJECT})

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
if(BUILD_LIFECYCLE_NODE)
    find_package(rclcpp_lifecycle REQUIRED)
endif(BUILD_LIFECYCLE_NODE)

# Message/service types
rosidl_generate_interfaces(
    ${PROJECT}
        msg/RadarInfo.msg
        srv/SetRadarRange.srv
    DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(TYPESUPPORT ${PROJECT} rosidl_typesupport_cpp)

# Appropriate radar_api_ros2_wrapper
add_subdirectory(${PROJECT}_${PROVIZIO_RADAR_API})

# provizio_radar_api_ros2_node
if(BUILD_SIMPLE_NODE)
    add_executable(${PROJECT}_node src/provizio_radar_api_ros2_node.cpp src/constants.cpp)
    ament_target_dependencies(${PROJECT}_node rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs)
    target_include_directories(${PROJECT}_node PUBLIC include)
    target_link_libraries(${PROJECT}_node ${PROJECT}_${PROVIZIO_RADAR_API} ${TYPESUPPORT})
endif(BUILD_SIMPLE_NODE)

# provizio_radar_api_ros2_lifecycle_node
if(BUILD_LIFECYCLE_NODE)
    add_executable(${PROJECT}_lifecycle_node src/provizio_radar_api_ros2_lifecycle_node.cpp src/constants.cpp)
    ament_target_dependencies(${PROJECT}_lifecycle_node rclcpp rclcpp_lifecycle sensor_msgs nav_msgs geometry_msgs std_msgs)
    target_include_directories(${PROJECT}_lifecycle_node PUBLIC include)
    target_link_libraries(${PROJECT}_lifecycle_node ${PROJECT}_${PROVIZIO_RADAR_API} ${TYPESUPPORT})
endif(BUILD_LIFECYCLE_NODE)

ament_export_dependencies(rclcpp rclcpp rclcpp_lifecycle sensor_msgs nav_msgs geometry_msgs std_msgs)

ament_package()
