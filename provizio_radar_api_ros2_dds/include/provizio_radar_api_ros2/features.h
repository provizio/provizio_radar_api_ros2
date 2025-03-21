#ifndef PROVIZIO_RADAR_API_ROS2_FEATURES
#define PROVIZIO_RADAR_API_ROS2_FEATURES

#include <rclcpp/version.h>

// ROS2 versions prior to jazzy don't have geometry_msgs/msg/polygon_instance or geometry_msgs/msg/polygon_instance_stamped
#define PROVIZIO_POLYGON_INSTANCE_AVAILABLE RCLCPP_VERSION_GTE(28, 0, 0)

#endif // PROVIZIO_RADAR_API_ROS2_FEATURES
