include(ExternalProject)

add_library(${PROJECT}_udp STATIC src/radar_api_ros2_wrapper_udp.cpp src/constants_udp.cpp)
target_include_directories(${PROJECT}_udp PUBLIC include "${CMAKE_SOURCE_DIR}/include")
ament_target_dependencies(${PROJECT}_udp SYSTEM PUBLIC rclcpp sensor_msgs nav_msgs)
target_link_libraries(${PROJECT}_udp PUBLIC ${TYPESUPPORT})

# Resolve provizio_radar_api_core (https://github.com/provizio/provizio_radar_api_core)
set(PROVIZIO_RADAR_CORE_API_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_radar_api_core_build")
set(PROVIZIO_RADAR_CORE_API_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_radar_api_core")
set(PROVIZIO_RADAR_CORE_API_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
set(PROVIZIO_RADAR_CORE_API_GITHUB_PROJECT "provizio/provizio_radar_api_core")
set(PROVIZIO_RADAR_CORE_API_GITHUB_BRANCH "${PROVIZIO_API_CORE_VERSION}")
set(PROVIZIO_RADAR_CORE_API_INSTALL_DIR "${PROVIZIO_RADAR_CORE_API_BINARY_DIR}/install")
ExternalProject_Add(libprovizio_radar_api_core
    GIT_REPOSITORY "https://github.com/${PROVIZIO_RADAR_CORE_API_GITHUB_PROJECT}.git"
    GIT_TAG "${PROVIZIO_RADAR_CORE_API_GITHUB_BRANCH}"
    UPDATE_COMMAND ""
    PREFIX "${PROVIZIO_RADAR_CORE_API_PREFIX}"
    SOURCE_DIR "${PROVIZIO_RADAR_CORE_API_SOURCE_DIR}"
    BINARY_DIR "${PROVIZIO_RADAR_CORE_API_BINARY_DIR}"
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}" "-DENABLE_CHECK_FORMAT=OFF" "-DBUILD_TESTING=OFF" "-DLOOK_FOR_FAST_DDS=FALSE"
)
add_dependencies(${PROJECT}_udp libprovizio_radar_api_core)
target_include_directories(${PROJECT}_udp PUBLIC "${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}/include")
target_link_directories(${PROJECT}_udp PUBLIC "${PROVIZIO_RADAR_CORE_API_INSTALL_DIR}/lib")
target_link_libraries(${PROJECT}_udp PUBLIC provizio_radar_api_core)

if(BUILD_TESTING)
    file(GLOB TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/test/*")
    foreach(FILE ${TEST_FILES})
        if(NOT IS_DIRECTORY ${FILE})
            install(FILES ${FILE} DESTINATION lib)
        endif(NOT IS_DIRECTORY ${FILE})
    endforeach()
endif(BUILD_TESTING)
