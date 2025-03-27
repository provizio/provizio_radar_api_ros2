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
    const std::string publish_radar_pc_param = "publish_radar_pc";                             // NOLINT: Never throws
    const std::string publish_radar_info_param = "publish_radar_info";                         // NOLINT: Never throws
    const std::string serve_set_radar_range_param = "serve_set_radar_range";                   // NOLINT: Never throws
    const std::string radar_pc_ros2_topic_name_param = "radar_pc_topic";                       // NOLINT: Never throws
    const std::string radar_info_ros2_topic_name_param = "radar_info_topic";                   // NOLINT: Never throws
    const std::string set_radar_range_ros2_service_name_param = "set_radar_range_service";     // NOLINT: Never throws
    const std::string snr_threshold_param = "snr_threshold";                                   // NOLINT: Never throws
    const rclcpp::QoS default_ros2_qos{10};
} // namespace provizio
