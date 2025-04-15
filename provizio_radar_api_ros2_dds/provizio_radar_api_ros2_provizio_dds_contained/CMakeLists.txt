include(ExternalProject)

add_library(${PROJECT}_provizio_dds_contained SHARED
    src/provizio_dds_contained.cpp
    src/provizio_dds_contained_types_dds_conversion.cpp
)
target_include_directories(${PROJECT}_provizio_dds_contained PUBLIC include)
target_include_directories(${PROJECT}_provizio_dds_contained PRIVATE src)

# Resolve provizio_dds (https://github.com/provizio/provizio_dds)
set(PROVIZIO_DDS_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds_build")
set(PROVIZIO_DDS_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/provizio_dds")
set(PROVIZIO_DDS_PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
set(PROVIZIO_DDS_GITHUB_PROJECT "provizio/provizio_dds")
set(PROVIZIO_DDS_GITHUB_BRANCH "${PROVIZIO_DDS_VERSION}")
set(PROVIZIO_DDS_INSTALL_DIR "${PROVIZIO_DDS_BINARY_DIR}/install")
ExternalProject_Add(libprovizio_dds_externalproject
    GIT_REPOSITORY "https://github.com/${PROVIZIO_DDS_GITHUB_PROJECT}.git"
    GIT_TAG "${PROVIZIO_DDS_GITHUB_BRANCH}"
    UPDATE_COMMAND ""
    PREFIX "${PROVIZIO_DDS_PREFIX}"
    SOURCE_DIR "${PROVIZIO_DDS_SOURCE_DIR}"
    BINARY_DIR "${PROVIZIO_DDS_BINARY_DIR}"
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${PROVIZIO_DDS_INSTALL_DIR}" "-DENABLE_CHECK_FORMAT=OFF" "-DENABLE_TESTS=OFF" "-DLOOK_FOR_FAST_DDS=FALSE"
    LOG_MERGED_STDOUTERR ON
)

add_custom_target(libprovizio_dds DEPENDS libprovizio_dds_externalproject)
add_dependencies(${PROJECT}_provizio_dds_contained libprovizio_dds)
target_include_directories(${PROJECT}_provizio_dds_contained SYSTEM PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/include")
target_link_directories(${PROJECT}_provizio_dds_contained PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/lib")
target_link_libraries(${PROJECT}_provizio_dds_contained PUBLIC provizio_dds provizio_dds_types)
install(TARGETS ${PROJECT}_provizio_dds_contained LIBRARY DESTINATION lib)
install(DIRECTORY "${PROVIZIO_DDS_INSTALL_DIR}/lib/" DESTINATION lib)
