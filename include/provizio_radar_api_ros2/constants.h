#ifndef PROVIZIO_RADAR_API_ROS2_CONSTANTS
#define PROVIZIO_RADAR_API_ROS2_CONSTANTS

#include <string>

namespace provizio
{
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
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_CONSTANTS
