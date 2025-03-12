#include <algorithm>
#include <iterator>

#include "provizio_radar_api_ros2/provizio_dds_contained_types_ros2_conversion.h"

namespace provizio
{
    builtin_interfaces::msg::Time to_ros2_time(const provizio::contained_time &stamp)
    {
        builtin_interfaces::msg::Time result;
        result.sec = stamp.sec;
        result.nanosec = stamp.nanosec;
        return result;
    }

    std_msgs::msg::Header to_ros2_header(provizio::contained_header header)
    {
        std_msgs::msg::Header result;
        result.frame_id = std::move(header.frame_id);
        result.stamp = to_ros2_time(header.stamp);
        return result;
    }

    sensor_msgs::msg::PointField to_ros2_point_field(provizio::contained_point_field &&field)
    {
        sensor_msgs::msg::PointField result;
        result.name = std::move(field.name);
        result.offset = field.offset;
        result.datatype = field.datatype;
        result.count = field.count;
        return result;
    }

    std::vector<sensor_msgs::msg::PointField> to_ros2_point_fields(std::vector<provizio::contained_point_field> fields)
    {
        std::vector<sensor_msgs::msg::PointField> result;
        std::transform(std::make_move_iterator(fields.begin()), std::make_move_iterator(fields.end()),
                       std::back_inserter(result), to_ros2_point_field);
        return result;
    }

    sensor_msgs::msg::PointCloud2 to_ros2_pointcloud2(provizio::contained_pointcloud2 message)
    {
        sensor_msgs::msg::PointCloud2 result;
        result.header = to_ros2_header(std::move(message.header));
        result.width = message.width;
        result.height = message.height;
        result.fields = to_ros2_point_fields(std::move(message.fields));
        result.is_bigendian = message.is_bigendian;
        result.point_step = message.point_step;
        result.row_step = message.row_step;
        result.data = std::move(message.data);
        result.is_dense = message.is_dense;
        return result;
    }
} // namespace provizio
