add_subdirectory(${PROJECT}_provizio_dds_contained)

add_library(${PROJECT}_dds STATIC
    src/provizio_dds_container.cpp
    src/constants_dds.cpp
    src/provizio_dds_contained_types_ros2_conversion.cpp
)
target_include_directories(${PROJECT}_dds PUBLIC include ${PROJECT}_provizio_dds_contained/include)
add_dependencies(${PROJECT}_dds ${PROJECT}_provizio_dds_contained)
ament_target_dependencies(${PROJECT}_dds rclcpp sensor_msgs nav_msgs)
target_link_libraries(${PROJECT}_dds ${TYPESUPPORT})
