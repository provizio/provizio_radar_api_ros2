#include "provizio_radar_api_ros2/constants_dds.h"

namespace provizio
{
    const std::string radar_pc_dds_topic_name = "rt/provizio_radar_point_cloud";       // NOLINT: Never throws
    const std::string radar_pc_sr_dds_topic_name = "rt/provizio_radar_point_cloud_sr"; // NOLINT: Never throws
    const std::string entities_radar_dds_topic_name = "rt/provizio_entities";          // NOLINT: Never throws
    const std::string entities_camera_dds_topic_name = "rt/provizio_entities_camera";  // NOLINT: Never throws
    const std::string entities_fusion_dds_topic_name = "rt/provizio_entities_fusion";  // NOLINT: Never throws
    const std::string radar_odometry_dds_topic_name = "rt/provizio_radar_odometry";    // NOLINT: Never throws
    const std::string camera_dds_topic_name = "rt/provizio_camera";                    // NOLINT: Never throws
    const std::string radar_freespace_dds_topic_name = "rt/provizio_freespace_poly";   // NOLINT: Never throws
    const std::string radar_info_dds_topic_name = "rt/provizio_radar_info";            // NOLINT: Never throws
    const std::string set_radar_range_dds_topic_name = "rt/provizio_set_radar_range";  // NOLINT: Never throws
} // namespace provizio
