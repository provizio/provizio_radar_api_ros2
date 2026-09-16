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

# provizio_dds configures itself via provizio/coding_standards' StandardConfig, which enables ASan, LSan
# and UBSan for any "Debug" build. That would instrument the bundled provizio_dds / Fast-DDS shared
# libraries this contained library links against, while the node executable that loads them is NOT
# instrumented (the top-level CMakeLists deliberately strips sanitizer flags for the DDS API, due to known
# Fast-DDS / ROS 2 issues). The mismatch aborts every node at start-up with "ASan runtime does not come
# first in initial library list", which in turn hangs the test harness.
#
# DISABLE_PROVIZIO_CODING_STANDARDS_CHECKS is what provizio_dds' README prescribes for exactly this - a
# library built to be consumed rather than worked on - and it addresses the sanitizers directly, so the
# dependency can still be built as Debug when this project is. Forcing it to RelWithDebInfo instead would
# also avoid the sanitizers, but only as a side effect of changing the build type, and it would leave a
# Debug build of this project linking a non-Debug dependency.

find_package(Git REQUIRED)
ExternalProject_Add(libprovizio_dds_externalproject
    # Cloned by hand rather than through GIT_REPOSITORY, as provizio_dds' own README documents.
    # GIT_SHALLOW is not the answer either: CMake implements it as `--depth 1 --no-single-branch`,
    # which fetches the tip of every branch and every tag, and each ref in that repository carries
    # its own committed binary cache. provizio_dds measured 112 MB / 5 s for the clone below against
    # 1006 MB / 124 s for GIT_SHALLOW - and the full clone this replaces fetched all of that plus the
    # entire history. Nothing here reads that history.
    # The remove_directory keeps the step re-runnable: git refuses a non-empty destination, which
    # changing PROVIZIO_DDS_VERSION and rebuilding in the same tree would otherwise hit.
    DOWNLOAD_COMMAND "${CMAKE_COMMAND}" -E remove_directory "${PROVIZIO_DDS_SOURCE_DIR}"
             COMMAND "${GIT_EXECUTABLE}" clone --depth 1 --single-branch
                     --branch "${PROVIZIO_DDS_GITHUB_BRANCH}"
                     "https://github.com/${PROVIZIO_DDS_GITHUB_PROJECT}.git"
                     "${PROVIZIO_DDS_SOURCE_DIR}"
    UPDATE_COMMAND ""
    PREFIX "${PROVIZIO_DDS_PREFIX}"
    SOURCE_DIR "${PROVIZIO_DDS_SOURCE_DIR}"
    BINARY_DIR "${PROVIZIO_DDS_BINARY_DIR}"
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${PROVIZIO_DDS_INSTALL_DIR}" "-DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON" "-DENABLE_CHECK_FORMAT=OFF" "-DENABLE_TESTS=OFF" "-DSTATIC_ANALYSIS=OFF" "-DLOOK_FOR_FAST_DDS=FALSE" "-DINSTALL_ONLY_FULLY_QUALIFIED_FAST_DDS_LIBS=ON"
    LOG_MERGED_STDOUTERR ON
)

add_custom_target(libprovizio_dds DEPENDS libprovizio_dds_externalproject)
add_dependencies(${PROJECT}_provizio_dds_contained libprovizio_dds)
target_include_directories(${PROJECT}_provizio_dds_contained SYSTEM PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/include")
target_link_directories(${PROJECT}_provizio_dds_contained PUBLIC "${PROVIZIO_DDS_INSTALL_DIR}/lib")
target_link_libraries(${PROJECT}_provizio_dds_contained PUBLIC provizio_dds provizio_dds_types)
install(TARGETS ${PROJECT}_provizio_dds_contained LIBRARY DESTINATION lib)
install(DIRECTORY "${PROVIZIO_DDS_INSTALL_DIR}/lib/" DESTINATION lib)
