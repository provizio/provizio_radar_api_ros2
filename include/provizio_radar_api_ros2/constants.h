// Copyright 2025 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PROVIZIO_RADAR_API_ROS2_CONSTANTS
#define PROVIZIO_RADAR_API_ROS2_CONSTANTS

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace provizio
{
    constexpr float default_snr_threshold = 0.0F;
    constexpr bool feature_enabled_by_default = true;
    extern const std::string default_radar_pc_ros2_topic_name;
    extern const std::string default_radar_pc_sr_ros2_topic_name;
    extern const std::string default_entities_radar_ros2_topic_name;
    extern const std::string default_entities_camera_ros2_topic_name;
    extern const std::string default_entities_fusion_ros2_topic_name;
    extern const std::string default_radar_odometry_ros2_topic_name;
    extern const std::string default_camera_ros2_topic_name;
    extern const std::string default_radar_freespace_ros2_topic_name;
    extern const std::string default_radar_freespace_ros2_instance_topic_name;
    extern const std::string default_camera_freespace_ros2_topic_name;
    extern const std::string default_camera_freespace_ros2_instance_topic_name;
    extern const std::string default_radar_info_ros2_topic_name;
    extern const std::string default_set_radar_range_ros2_service_name;
    extern const std::string publish_radar_pc_param;
    extern const std::string publish_radar_info_param;
    extern const std::string serve_set_radar_range_param;
    extern const std::string radar_pc_ros2_topic_name_param;
    extern const std::string radar_info_ros2_topic_name_param;
    extern const std::string set_radar_range_ros2_service_name_param;
    extern const std::string snr_threshold_param;
    extern const rclcpp::QoS default_ros2_qos;
    extern const bool is_host_big_endian;
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_CONSTANTS
