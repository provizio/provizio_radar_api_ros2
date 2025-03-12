#ifndef SRC_PROVIZIO_DDS_CONTAINED_TYPES_ROS2_CONVERSION
#define SRC_PROVIZIO_DDS_CONTAINED_TYPES_ROS2_CONVERSION

#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "provizio_radar_api_ros2/provizio_dds_contained_types.h"

namespace provizio
{
    builtin_interfaces::msg::Time to_ros2_time(const provizio::contained_time &stamp);
    std_msgs::msg::Header to_ros2_header(provizio::contained_header header);
    std::vector<sensor_msgs::msg::PointField> to_ros2_point_fields(std::vector<provizio::contained_point_field> fields);
    sensor_msgs::msg::PointCloud2 to_ros2_pointcloud2(provizio::contained_pointcloud2 message);
} // namespace provizio

#endif // SRC_PROVIZIO_DDS_CONTAINED_TYPES_ROS2_CONVERSION
