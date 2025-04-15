#include "provizio_radar_api_ros2/constants_dds.h"

namespace provizio
{
    // NOLINTBEGIN: These global constants don't throw on construction
    const std::string dds_domain_id_param = "provizio_dds_domain_id";
    const std::string publish_radar_pc_sr_param = "publish_radar_pc_sr";
    const std::string publish_entities_radar_param = "publish_entities_radar";
    const std::string publish_entities_camera_param = "publish_entities_camera";
    const std::string publish_entities_fusion_param = "publish_entities_fusion";
    const std::string publish_radar_odometry_param = "publish_radar_odometry";
    const std::string publish_camera_param = "publish_camera";
    const std::string publish_radar_freespace_param = "publish_radar_freespace";
    const std::string publish_camera_freespace_param = "publish_camera_freespace";
    const std::string publish_radar_freespace_instance_param = "publish_radar_freespace_instance";
    const std::string publish_camera_freespace_instance_param = "publish_camera_freespace_instance";
    const std::string radar_pc_sr_ros2_topic_name_param = "radar_pc_sr_topic";
    const std::string entities_radar_ros2_topic_name_param = "entities_radar_topic";
    const std::string entities_camera_ros2_topic_name_param = "entities_camera_topic";
    const std::string entities_fusion_ros2_topic_name_param = "entities_fusion_topic";
    const std::string radar_odometry_ros2_topic_name_param = "radar_odometry_topic";
    const std::string camera_ros2_topic_name_param = "camera_topic";
    const std::string radar_freespace_ros2_topic_name_param = "radar_freespace_topic";
    const std::string radar_freespace_ros2_instance_topic_name_param = "radar_freespace_instance_topic";
    const std::string camera_freespace_ros2_topic_name_param = "camera_freespace_topic";
    const std::string camera_freespace_ros2_instance_topic_name_param = "camera_freespace_instance_topic";
    const std::string radar_pc_dds_topic_name = "rt/provizio_radar_point_cloud";
    const std::string radar_pc_sr_dds_topic_name = "rt/provizio_radar_point_cloud_sr";
    const std::string entities_radar_dds_topic_name = "rt/provizio_entities";
    const std::string entities_camera_dds_topic_name = "rt/provizio_entities_camera";
    const std::string entities_fusion_dds_topic_name = "rt/provizio_entities_fusion";
    const std::string radar_odometry_dds_topic_name = "rt/provizio_radar_odometry";
    const std::string camera_dds_topic_name = "rt/provizio_camera";
    const std::string radar_freespace_dds_topic_name = "rt/provizio_freespace_poly";
    const std::string camera_freespace_dds_topic_name = "rt/provizio_freespace_camera_poly";
    const std::string radar_info_dds_topic_name = "rt/provizio_radar_info";
    const std::string set_radar_range_dds_topic_name = "rt/provizio_set_radar_range";
    // NOLINTEND
} // namespace provizio
