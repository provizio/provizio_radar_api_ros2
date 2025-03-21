#include "provizio_radar_api_ros2/constants.h"

namespace provizio
{
    const std::string radar_pc_ros2_topic_name = "/provizio/radar_point_cloud";              // NOLINT: Never throws
    const std::string radar_pc_sr_ros2_topic_name = "/provizio/radar_point_cloud_sr";        // NOLINT: Never throws
    const std::string entities_radar_ros2_topic_name = "/provizio/entities/radar";           // NOLINT: Never throws
    const std::string entities_camera_ros2_topic_name = "/provizio/entities/camera";         // NOLINT: Never throws
    const std::string entities_fusion_ros2_topic_name = "/provizio/entities/fusion";         // NOLINT: Never throws
    const std::string radar_odometry_ros2_topic_name = "/provizio/odometry/radar";           // NOLINT: Never throws
    const std::string camera_ros2_topic_name = "/provizio/camera_raw";                       // NOLINT: Never throws
    const std::string radar_freespace_ros2_topic_name = "/provizio/radar_freespace/stamped"; // NOLINT: Never throws
    const std::string radar_freespace_ros2_instance_topic_name =
        "/provizio/radar_freespace/instance_stamped"; // NOLINT: Never throws
} // namespace provizio
