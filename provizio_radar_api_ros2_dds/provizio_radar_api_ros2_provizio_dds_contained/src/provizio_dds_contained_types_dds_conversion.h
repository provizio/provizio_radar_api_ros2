#ifndef SRC_PROVIZIO_DDS_CONTAINED_TYPES_DDS_CONVERSION
#define SRC_PROVIZIO_DDS_CONTAINED_TYPES_DDS_CONVERSION

#include <vector>

#include "geometry_msgs/msg/PolygonInstanceStamped.h"
#include "nav_msgs/msg/Odometry.h"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/PointCloud2.h"

#include "provizio_radar_api_ros2/provizio_dds_contained_types.h"

namespace provizio
{
    provizio::contained_pointcloud2 to_contained_pointcloud2(const sensor_msgs::msg::PointCloud2 &message);
    provizio::contained_odometry to_contained_odometry(const nav_msgs::msg::Odometry &message);
    provizio::contained_image to_contained_image(const sensor_msgs::msg::Image &message);
    provizio::contained_polygon_instance_stamped to_contained_polygon_instance_stamped(
        const geometry_msgs::msg::PolygonInstanceStamped &message);
} // namespace provizio

#endif // SRC_PROVIZIO_DDS_CONTAINED_TYPES_DDS_CONVERSION
