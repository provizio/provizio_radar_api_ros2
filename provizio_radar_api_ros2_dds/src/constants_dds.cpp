#include "provizio_radar_api_ros2/constants_dds.h"

namespace provizio
{
    const std::string dds_domain_id_param = "provizio_dds_domain_id";              // NOLINT: Never throws
    const std::string publish_radar_pc_sr_param = "publish_radar_pc_sr";           // NOLINT: Never throws
    const std::string publish_entities_radar_param = "publish_entities_radar";     // NOLINT: Never throws
    const std::string publish_entities_camera_param = "publish_entities_camera";   // NOLINT: Never throws
    const std::string publish_entities_fusion_param = "publish_entities_fusion";   // NOLINT: Never throws
    const std::string publish_radar_odometry_param = "publish_radar_odometry";     // NOLINT: Never throws
    const std::string publish_camera_param = "publish_camera";                     // NOLINT: Never throws
    const std::string publish_radar_freespace_param = "publish_radar_freespace";   // NOLINT: Never throws
    const std::string publish_camera_freespace_param = "publish_camera_freespace"; // NOLINT: Never throws
    const std::string publish_radar_freespace_instance_param =                     // NOLINT: Never throws
        "publish_radar_freespace_instance";
    const std::string publish_camera_freespace_instance_param = // NOLINT: Never throws
        "publish_camera_freespace_instance";
    const std::string radar_pc_sr_ros2_topic_name_param = "radar_pc_sr_topic";         // NOLINT: Never throws
    const std::string entities_radar_ros2_topic_name_param = "entities_radar_topic";   // NOLINT: Never throws
    const std::string entities_camera_ros2_topic_name_param = "entities_camera_topic"; // NOLINT: Never throws
    const std::string entities_fusion_ros2_topic_name_param = "entities_fusion_topic"; // NOLINT: Never throws
    const std::string radar_odometry_ros2_topic_name_param = "radar_odometry_topic";   // NOLINT: Never throws
    const std::string camera_ros2_topic_name_param = "camera_topic";                   // NOLINT: Never throws
    const std::string radar_freespace_ros2_topic_name_param = "radar_freespace_topic"; // NOLINT: Never throws
    const std::string radar_freespace_ros2_instance_topic_name_param =                 // NOLINT: Never throws
        "radar_freespace_instance_topic";
    const std::string camera_freespace_ros2_topic_name_param = "camera_freespace_topic"; // NOLINT: Never throws
    const std::string camera_freespace_ros2_instance_topic_name_param =                  // NOLINT: Never throws
        "camera_freespace_instance_topic";
    const std::string radar_pc_dds_topic_name = "rt/provizio_radar_point_cloud";             // NOLINT: Never throws
    const std::string radar_pc_sr_dds_topic_name = "rt/provizio_radar_point_cloud_sr";       // NOLINT: Never throws
    const std::string entities_radar_dds_topic_name = "rt/provizio_entities";                // NOLINT: Never throws
    const std::string entities_camera_dds_topic_name = "rt/provizio_entities_camera";        // NOLINT: Never throws
    const std::string entities_fusion_dds_topic_name = "rt/provizio_entities_fusion";        // NOLINT: Never throws
    const std::string radar_odometry_dds_topic_name = "rt/provizio_radar_odometry";          // NOLINT: Never throws
    const std::string camera_dds_topic_name = "rt/provizio_camera";                          // NOLINT: Never throws
    const std::string radar_freespace_dds_topic_name = "rt/provizio_freespace_poly";         // NOLINT: Never throws
    const std::string camera_freespace_dds_topic_name = "rt/provizio_freespace_camera_poly"; // NOLINT: Never throws
    const std::string radar_info_dds_topic_name = "rt/provizio_radar_info";                  // NOLINT: Never throws
    const std::string set_radar_range_dds_topic_name = "rt/provizio_set_radar_range";        // NOLINT: Never throws
} // namespace provizio
