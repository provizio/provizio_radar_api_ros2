add_subdirectory(${PROJECT}_provizio_dds_contained)

add_library(${PROJECT}_dds STATIC
    src/provizio_dds_container.cpp
    src/constants_dds.cpp
    src/provizio_dds_contained_types_ros2_conversion.cpp
)
target_include_directories(${PROJECT}_dds PUBLIC include ${PROJECT}_provizio_dds_contained/include "${CMAKE_SOURCE_DIR}/include")
add_dependencies(${PROJECT}_dds ${PROJECT}_provizio_dds_contained)
ament_target_dependencies(${PROJECT}_dds SYSTEM PUBLIC rclcpp sensor_msgs nav_msgs)
target_link_libraries(${PROJECT}_dds PUBLIC ${TYPESUPPORT})

if(BUILD_TESTING)
    file(GLOB TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/test/*")
    foreach(FILE ${TEST_FILES})
        if(NOT IS_DIRECTORY ${FILE})
            install(FILES ${FILE} DESTINATION lib)
        endif(NOT IS_DIRECTORY ${FILE})
    endforeach()
endif(BUILD_TESTING)
