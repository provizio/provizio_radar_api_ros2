#ifndef PROVIZIO_RADAR_API_ROS2_CONSTANTS
#define PROVIZIO_RADAR_API_ROS2_CONSTANTS

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace provizio
{
    constexpr float default_snr_threshold = 0.0F;
    extern const std::string default_radar_pc_ros2_topic_name;
    extern const std::string default_radar_pc_sr_ros2_topic_name;
    extern const std::string default_entities_radar_ros2_topic_name;
    extern const std::string default_entities_camera_ros2_topic_name;
    extern const std::string default_entities_fusion_ros2_topic_name;
    extern const std::string default_radar_odometry_ros2_topic_name;
    extern const std::string default_camera_ros2_topic_name;
    extern const std::string default_radar_freespace_ros2_topic_name;
    extern const std::string default_radar_freespace_ros2_instance_topic_name;
    extern const std::string default_radar_info_ros2_topic_name;
    extern const std::string default_set_radar_range_ros2_service_name;
    extern const std::string radar_pc_ros2_topic_name_param;
    extern const rclcpp::QoS default_ros2_qos;
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_CONSTANTS
