cmake_minimum_required(VERSION 3.14)

# Configuration
set(DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS OFF CACHE BOOL "Disable all provizio/coding_standards checks")
set(PROVIZIO_CODING_STANDARDS_VERSION "1.10.1" CACHE STRING "provizio/coding_standards version")
set(BUILD_SIMPLE_NODE ON CACHE BOOL "Enable building of provizio_radar_api_ros2_node")
set(BUILD_LIFECYCLE_NODE ON CACHE BOOL "Enable building of provizio_radar_api_ros2_lifecycle_node")
set(PROVIZIO_RADAR_API "dds" CACHE STRING "Underlying Provizio radar API: dds / udp")
set(PROVIZIO_DDS_VERSION "1.8.4" CACHE STRING "provizio_dds version (for dds API)")
set(PROVIZIO_API_CORE_VERSION "v2024.04.16" CACHE STRING "provizio_radar_api_core version (for udp API)")

set(PROJECT "provizio_radar_api_ros2")
set(NODES_BASE_NAME "provizio_radar")
project(${PROJECT})

# Standard Config (see https://github.com/provizio/coding_standards)
if(NOT DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS)
    file(DOWNLOAD
        "https://raw.githubusercontent.com/provizio/coding_standards/${PROVIZIO_CODING_STANDARDS_VERSION}/cpp/cmake/StandardConfig.cmake"
        "${CMAKE_BINARY_DIR}/StandardConfig.cmake" TLS_VERIFY ON)
    include(${CMAKE_BINARY_DIR}/StandardConfig.cmake)
    StandardConfig(SAFETY_CRITICAL)

    # Disable runtime analysis tools in case of DDS API due to known issues in Fast-DDS
    if(CMAKE_CXX_FLAGS)
        message(WARNING "Runtime analysis tools will be disabled due to known issues in Fast-DDS and ROS 2")
        set(ENABLE_ASAN FALSE)
        set(ENABLE_TSAN FALSE)
        set(ENABLE_MSAN FALSE)
        set(CMAKE_CXX_FLAGS "")
        set(CMAKE_C_FLAGS "")
    endif(CMAKE_CXX_FLAGS)
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

# Declares all target's include directories "system" to avoid warning in them
function(declare_system_library target)
  get_target_property(target_aliased_name ${target} ALIASED_TARGET)
  if(target_aliased_name)
    set(target ${target_aliased_name})
  endif(target_aliased_name)
  set_target_properties(${target} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES $<TARGET_PROPERTY:${target},INTERFACE_INCLUDE_DIRECTORIES>)
endfunction()

# Message/service types
rosidl_generate_interfaces(
    ${PROJECT}
        msg/RadarInfo.msg
        srv/SetRadarRange.srv
    DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(TYPESUPPORT ${PROJECT} rosidl_typesupport_cpp)
declare_system_library(${TYPESUPPORT}) # Disable warnings in any of generated headers
# Disable any clang-tidy checks in all rosidl generated targets
get_property(ALL_TARGETS DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY BUILDSYSTEM_TARGETS)
foreach(target ${ALL_TARGETS})
    string(REGEX MATCH ".*_rosidl_.*" is_rosidl_target ${target})
    if(is_rosidl_target)
        set_property(TARGET ${target} PROPERTY CXX_CLANG_TIDY "")
        set_property(TARGET ${target} PROPERTY C_CLANG_TIDY "")
    endif(is_rosidl_target)
endforeach()

# Appropriate radar_api_ros2_wrapper
add_subdirectory(${PROJECT}_${PROVIZIO_RADAR_API})

# provizio_radar_api_ros2_node
if(BUILD_SIMPLE_NODE)
    add_executable(${NODES_BASE_NAME}_node src/provizio_radar_api_ros2_node.cpp src/constants.cpp)
    ament_target_dependencies(${NODES_BASE_NAME}_node SYSTEM PUBLIC rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs)
    target_include_directories(${NODES_BASE_NAME}_node PUBLIC include)
    target_link_libraries(${NODES_BASE_NAME}_node PUBLIC ${PROJECT}_${PROVIZIO_RADAR_API} ${TYPESUPPORT})
    install(TARGETS ${NODES_BASE_NAME}_node RUNTIME DESTINATION lib/${PROJECT})
endif(BUILD_SIMPLE_NODE)

# provizio_radar_api_ros2_lifecycle_node
if(BUILD_LIFECYCLE_NODE)
    add_executable(${NODES_BASE_NAME}_lifecycle_node src/provizio_radar_api_ros2_lifecycle_node.cpp src/constants.cpp)
    ament_target_dependencies(${NODES_BASE_NAME}_lifecycle_node SYSTEM PUBLIC rclcpp rclcpp_lifecycle sensor_msgs nav_msgs geometry_msgs std_msgs)
    target_include_directories(${NODES_BASE_NAME}_lifecycle_node PUBLIC include)
    target_link_libraries(${NODES_BASE_NAME}_lifecycle_node PUBLIC ${PROJECT}_${PROVIZIO_RADAR_API} ${TYPESUPPORT})
    install(TARGETS ${NODES_BASE_NAME}_lifecycle_node RUNTIME DESTINATION lib/${PROJECT})
endif(BUILD_LIFECYCLE_NODE)

ament_export_dependencies(rclcpp rclcpp rclcpp_lifecycle sensor_msgs nav_msgs geometry_msgs std_msgs)

ament_package()
