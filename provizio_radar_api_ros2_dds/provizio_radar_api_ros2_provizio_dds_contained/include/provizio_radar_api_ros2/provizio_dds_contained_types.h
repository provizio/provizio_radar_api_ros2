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
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES
