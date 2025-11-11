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

#ifndef PROVIZIO_RADAR_API_ROS2_PARAMETERS
#define PROVIZIO_RADAR_API_ROS2_PARAMETERS

#include "provizio_radar_api_ros2/constants.h"

namespace provizio
{
    template <typename node_type> void declare_common_parameters(node_type &node)
    {
        node.declare_parameter(publish_radar_pc_param, feature_enabled_by_default);
        node.declare_parameter(publish_radar_info_param, feature_enabled_by_default);
        node.declare_parameter(serve_set_radar_range_param, feature_enabled_by_default);
        node.declare_parameter(radar_pc_ros2_topic_name_param, default_radar_pc_ros2_topic_name);
        node.declare_parameter(frame_id_param, default_frame_id);
        node.declare_parameter(snr_threshold_param, static_cast<double>(default_snr_threshold));
        node.declare_parameter(radar_info_ros2_topic_name_param, default_radar_info_ros2_topic_name);
        node.declare_parameter(set_radar_range_ros2_service_name_param, default_set_radar_range_ros2_service_name);
    }
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_PARAMETERS
