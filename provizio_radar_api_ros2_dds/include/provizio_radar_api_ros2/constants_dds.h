#ifndef PROVIZIO_RADAR_API_ROS2_CONSTANTS_DDS
#define PROVIZIO_RADAR_API_ROS2_CONSTANTS_DDS

#include <chrono>
#include <cstdint>
#include <string>

namespace provizio
{
    constexpr std::chrono::seconds max_time_to_set_radar_range{30};
    constexpr std::uint32_t default_dds_domain_id = 0;
    extern const std::string dds_domain_id_param;
    extern const std::string publish_radar_pc_param;
    extern const std::string publish_radar_pc_sr_param;
    extern const std::string publish_entities_radar_param;
    extern const std::string publish_entities_camera_param;
    extern const std::string publish_entities_fusion_param;
    extern const std::string publish_radar_odometry_param;
    extern const std::string publish_camera_param;
    extern const std::string publish_radar_freespace_param;
    extern const std::string publish_radar_freespace_instance_param;
    extern const std::string publish_radar_info_param;
    extern const std::string serve_set_radar_range_param;
    extern const std::string radar_pc_sr_ros2_topic_name_param;
    extern const std::string entities_radar_ros2_topic_name_param;
    extern const std::string entities_camera_ros2_topic_name_param;
    extern const std::string entities_fusion_ros2_topic_name_param;
    extern const std::string radar_odometry_ros2_topic_name_param;
    extern const std::string camera_ros2_topic_name_param;
    extern const std::string radar_freespace_ros2_topic_name_param;
    extern const std::string radar_freespace_ros2_instance_topic_name_param;
    extern const std::string radar_info_ros2_topic_name_param;
    extern const std::string set_radar_range_ros2_service_name_param;
    extern const std::string radar_pc_dds_topic_name;
    extern const std::string radar_pc_sr_dds_topic_name;
    extern const std::string entities_radar_dds_topic_name;
    extern const std::string entities_camera_dds_topic_name;
    extern const std::string entities_fusion_dds_topic_name;
    extern const std::string radar_odometry_dds_topic_name;
    extern const std::string camera_dds_topic_name;
    extern const std::string radar_freespace_dds_topic_name;
    extern const std::string radar_info_dds_topic_name;
    extern const std::string set_radar_range_dds_topic_name;
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_CONSTANTS_DDS
