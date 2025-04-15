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

#ifndef PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER
#define PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER

#include "provizio_radar_api_ros2/radar_api_ros2_wrapper_udp.h"

namespace provizio
{
    template <typename node_t> using radar_api_ros2_wrapper = radar_api_ros2_wrapper_udp<node_t>;
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER
