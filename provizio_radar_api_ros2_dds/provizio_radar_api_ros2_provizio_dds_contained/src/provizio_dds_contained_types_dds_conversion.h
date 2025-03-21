#ifndef SRC_PROVIZIO_DDS_CONTAINED_TYPES_DDS_CONVERSION
#define SRC_PROVIZIO_DDS_CONTAINED_TYPES_DDS_CONVERSION

#include <vector>

#include "nav_msgs/msg/Odometry.h"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/PointCloud2.h"

#include "provizio_radar_api_ros2/provizio_dds_contained_types.h"

namespace provizio
{
    provizio::contained_pointcloud2 to_contained_pointcloud2(const sensor_msgs::msg::PointCloud2 &message);
    provizio::contained_odometry to_contained_odometry(const nav_msgs::msg::Odometry &message);
    provizio::contained_image to_contained_image(const sensor_msgs::msg::Image &message);
} // namespace provizio

#endif // SRC_PROVIZIO_DDS_CONTAINED_TYPES_DDS_CONVERSION
