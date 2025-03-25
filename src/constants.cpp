#include "provizio_radar_api_ros2/constants.h"

namespace provizio
{
    const std::string default_radar_pc_ros2_topic_name = "/provizio/radar_point_cloud";       // NOLINT: Never throws
    const std::string default_radar_pc_sr_ros2_topic_name = "/provizio/radar_point_cloud_sr"; // NOLINT: Never throws
    const std::string default_entities_radar_ros2_topic_name = "/provizio/entities/radar";    // NOLINT: Never throws
    const std::string default_entities_camera_ros2_topic_name = "/provizio/entities/camera";  // NOLINT: Never throws
    const std::string default_entities_fusion_ros2_topic_name = "/provizio/entities/fusion";  // NOLINT: Never throws
    const std::string default_radar_odometry_ros2_topic_name = "/provizio/odometry/radar";    // NOLINT: Never throws
    const std::string default_camera_ros2_topic_name = "/provizio/camera_raw";                // NOLINT: Never throws
    const std::string default_radar_freespace_ros2_topic_name =
        "/provizio/radar_freespace/stamped"; // NOLINT: Never throws
    const std::string default_radar_freespace_ros2_instance_topic_name =
        "/provizio/radar_freespace/instance_stamped";                                          // NOLINT: Never throws
    const std::string default_radar_info_ros2_topic_name = "/provizio/radar_info";             // NOLINT: Never throws
    const std::string default_set_radar_range_ros2_service_name = "/provizio/set_radar_range"; // NOLINT: Never throws
} // namespace provizio
