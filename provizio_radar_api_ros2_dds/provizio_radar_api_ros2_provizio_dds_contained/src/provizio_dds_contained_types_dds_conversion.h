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

#ifndef SRC_PROVIZIO_DDS_CONTAINED_TYPES_DDS_CONVERSION
#define SRC_PROVIZIO_DDS_CONTAINED_TYPES_DDS_CONVERSION

#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/PolygonInstanceStamped.h"
#include "nav_msgs/msg/Odometry.h"
#include "provizio/msg/radar_info.h"
#include "provizio/srv/set_radar_range.h"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/PointCloud2.h"

#include "provizio_radar_api_ros2/provizio_dds_contained_types.h"

namespace provizio
{
    provizio::contained_pointcloud2 to_contained_pointcloud2(const sensor_msgs::msg::PointCloud2 &message);
    provizio::contained_odometry to_contained_odometry(const nav_msgs::msg::Odometry &message);
    provizio::contained_image to_contained_image(const sensor_msgs::msg::Image &message);
    provizio::contained_polygon_instance_stamped to_contained_polygon_instance_stamped(
        const geometry_msgs::msg::PolygonInstanceStamped &message);
    provizio::contained_radar_info to_contained_radar_info(const provizio::msg::radar_info &message);
    // Builds the DDS request from POD / C-string inputs so no caller-side std::string crosses the
    // contained library's C++ runtime boundary. A null frame_id / serial_number is treated as empty.
    provizio::srv::set_radar_range_Request to_dds_set_radar_range_request(const char *frame_id,
                                                                          const char *serial_number,
                                                                          std::int8_t target_range,
                                                                          std::int32_t header_stamp_sec,
                                                                          std::uint32_t header_stamp_nanosec);
    provizio::contained_set_radar_range_response to_contained_set_radar_range_response(
        const provizio::srv::set_radar_range_Response &message);
} // namespace provizio

#endif // SRC_PROVIZIO_DDS_CONTAINED_TYPES_DDS_CONVERSION
