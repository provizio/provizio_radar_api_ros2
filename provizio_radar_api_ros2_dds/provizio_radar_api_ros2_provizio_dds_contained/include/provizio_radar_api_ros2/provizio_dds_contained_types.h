#ifndef PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES
#define PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES

#include <cstdint>
#include <string>
#include <vector>

namespace provizio
{
    struct contained_time
    {
        std::int32_t sec;
        std::int32_t nanosec;
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
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES
