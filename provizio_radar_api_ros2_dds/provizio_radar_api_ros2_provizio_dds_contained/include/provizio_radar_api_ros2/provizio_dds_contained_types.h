// Copyright 2025 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES
#define PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace provizio
{
    struct contained_time
    {
        std::int32_t sec;
        std::uint32_t nanosec;
    };

    struct contained_header
    {
        std::string frame_id;
        contained_time stamp;
    };

    struct contained_point_field
    {
        std::string name;
        std::uint32_t offset;
        std::uint32_t count;
        std::uint8_t datatype;
    };

    struct contained_pointcloud2
    {
        contained_header header;
        std::uint32_t width;
        std::uint32_t height;
        std::vector<contained_point_field> fields;
        std::uint32_t point_step;
        std::uint32_t row_step;
        std::vector<std::uint8_t> data;
        bool is_bigendian;
        bool is_dense;
    };

    struct contained_vector3
    {
        double x;
        double y;
        double z;
    };

    struct contained_quaternion
    {
        double x;
        double y;
        double z;
        double w;
    };

    struct contained_pose
    {
        contained_vector3 position;
        contained_quaternion orientation;
    };

    struct contained_pose_with_covariance
    {
        contained_pose pose;
        std::array<double, 36> covariance;
    };

    struct contained_twist
    {
        contained_vector3 linear;
        contained_vector3 angular;
    };

    struct contained_twist_with_covariance
    {
        contained_twist twist;
        std::array<double, 36> covariance;
    };

    struct contained_odometry
    {
        contained_header header;
        std::string child_frame_id;
        contained_pose_with_covariance pose;
        contained_twist_with_covariance twist;
    };

    struct contained_image
    {
        contained_header header;
        std::uint32_t height;
        std::uint32_t width;
        std::string encoding;
        std::uint8_t is_bigendian;
        std::uint32_t step;
        std::vector<std::uint8_t> data;
    };

    struct contained_point32
    {
        float x;
        float y;
        float z;
    };

    struct contained_polygon
    {
        std::vector<contained_point32> points;
    };

    struct contained_polygon_instance
    {
        contained_polygon polygon;
        std::int64_t id;
    };

    struct contained_polygon_instance_stamped
    {
        contained_header header;
        contained_polygon_instance polygon;
    };

    struct contained_radar_info
    {
        contained_header header;
        std::string serial_number;
        std::int8_t current_range;
        std::vector<std::int8_t> supported_ranges;
        std::int8_t current_multiplexing_mode;
    };

    struct contained_set_radar_range
    {
        contained_header header;
        std::string serial_number;
        std::int8_t target_range;
    };

    // Only the fields the C ABI actually marshals back (see
    // provizio_dds_contained_request_set_radar_range). The radar's response also carries a header and a
    // serial_number, but neither crosses the boundary, so they are deliberately absent rather than
    // present and always default-constructed on the caller's side.
    struct contained_set_radar_range_response
    {
        bool success{false};
        std::string error_message;
        std::int8_t current_range{-1}; // provizio_radar_api_ros2::msg::RadarInfo::UNKNOWN_RANGE
        std::vector<std::int8_t> supported_ranges;
    };

    // Outcome of a set_radar_range request/response round-trip. Note that "ok" only means a response
    // was received; check contained_set_radar_range_response::success for whether the range was set.
    enum class contained_set_radar_range_status : std::int32_t
    {
        ok = 0,         // A response was received from the radar
        timed_out = 1,  // No response received within the timeout
        error = 2,      // Failed to issue the request
        interrupted = 3 // The wait was interrupted (the node is being deactivated / shut down)
    };

    // The set_radar_range request/response crosses the extern "C" boundary of a library dlmopen'd into a
    // separate C++ runtime / heap namespace (see provizio_dds_container.cpp). std::string / std::vector
    // objects must NOT cross that boundary (allocating them on one side and reading/freeing them on the
    // other corrupts the heap). Request/response payloads are therefore marshalled as C strings and POD
    // arrays with these fixed capacities.
    constexpr std::size_t contained_set_radar_range_error_message_capacity = 256;
    constexpr std::size_t contained_set_radar_range_max_supported_ranges = 16;
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES
