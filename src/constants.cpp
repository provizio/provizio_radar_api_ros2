#include "provizio_radar_api_ros2/constants.h"

namespace provizio
{
    const std::string radar_pc_ros2_topic_name = "/provizio/radar_point_cloud";       // NOLINT: Never throws
    const std::string radar_pc_sr_ros2_topic_name = "/provizio/radar_point_cloud_sr"; // NOLINT: Never throws
    const std::string entities_radar_ros2_topic_name = "/provizio/entities_radar";    // NOLINT: Never throws
    const std::string entities_camera_ros2_topic_name = "/provizio/entities_camera";  // NOLINT: Never throws
    const std::string entities_fusion_ros2_topic_name = "/provizio/entities_fusion";  // NOLINT: Never throws
    const std::string radar_odometry_ros2_topic_name = "/provizio/radar_odometry";    // NOLINT: Never throws
} // namespace provizio
