#include "provizio_radar_api_ros2/constants_dds.h"

namespace provizio
{
    const std::string radar_pc_dds_topic_name = "rt/provizio_radar_point_cloud";       // NOLINT: Never throws
    const std::string radar_pc_sr_dds_topic_name = "rt/provizio_radar_point_cloud_sr"; // NOLINT: Never throws
    const std::string entities_radar_dds_topic_name = "rt/provizio_entities";          // NOLINT: Never throws
    const std::string entities_camera_dds_topic_name = "rt/provizio_entities_camera";  // NOLINT: Never throws
    const std::string entities_fusion_dds_topic_name = "rt/provizio_entities_fusion";  // NOLINT: Never throws
    const std::string radar_odometry_dds_topic_name = "rt/provizio_radar_odometry";    // NOLINT: Never throws
} // namespace provizio
