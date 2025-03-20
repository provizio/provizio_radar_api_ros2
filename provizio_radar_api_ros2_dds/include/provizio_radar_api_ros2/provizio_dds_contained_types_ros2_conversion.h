#ifndef SRC_PROVIZIO_DDS_CONTAINED_TYPES_ROS2_CONVERSION
#define SRC_PROVIZIO_DDS_CONTAINED_TYPES_ROS2_CONVERSION

#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "provizio_radar_api_ros2/provizio_dds_contained_types.h"

namespace provizio
{
    sensor_msgs::msg::PointCloud2 to_ros2_pointcloud2(provizio::contained_pointcloud2 message);
    nav_msgs::msg::Odometry to_ros2_odometry(provizio::contained_odometry message);
} // namespace provizio

#endif // SRC_PROVIZIO_DDS_CONTAINED_TYPES_ROS2_CONVERSION
