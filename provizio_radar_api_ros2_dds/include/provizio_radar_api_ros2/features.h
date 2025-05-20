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

#ifndef PROVIZIO_RADAR_API_ROS2_FEATURES
#define PROVIZIO_RADAR_API_ROS2_FEATURES

#include <rclcpp/version.h>

// ROS2 versions prior to jazzy don't have geometry_msgs/msg/polygon_instance or
// geometry_msgs/msg/polygon_instance_stamped
#define PROVIZIO_POLYGON_INSTANCE_AVAILABLE RCLCPP_VERSION_GTE(28, 0, 0)

#endif // PROVIZIO_RADAR_API_ROS2_FEATURES
