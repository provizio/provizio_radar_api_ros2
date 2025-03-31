#ifndef PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES_ROS2_CONVERSION
#define PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES_ROS2_CONVERSION

#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <provizio_radar_api_ros2/msg/radar_info.hpp>
#include <provizio_radar_api_ros2/srv/set_radar_range.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "provizio_radar_api_ros2/features.h"
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#endif
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include "provizio_radar_api_ros2/provizio_dds_contained_types.h"

namespace provizio
{
    sensor_msgs::msg::PointCloud2 to_ros2_pointcloud2(provizio::contained_pointcloud2 message, float snr_threshold = 0);
    nav_msgs::msg::Odometry to_ros2_odometry(provizio::contained_odometry message);
    sensor_msgs::msg::Image to_ros2_image(provizio::contained_image message);
    geometry_msgs::msg::PolygonStamped to_ros2_polygon_stamped(
        const provizio::contained_polygon_instance_stamped &message);
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
    geometry_msgs::msg::PolygonInstanceStamped to_ros2_polygon_instance_stamped(
        const provizio::contained_polygon_instance_stamped &message);
#endif
    provizio_radar_api_ros2::msg::RadarInfo to_ros2_radar_info(provizio::contained_radar_info message);
    provizio::contained_set_radar_range to_contained_set_radar_range(
        const provizio_radar_api_ros2::srv::SetRadarRange::Request &request);
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED_TYPES_ROS2_CONVERSION
