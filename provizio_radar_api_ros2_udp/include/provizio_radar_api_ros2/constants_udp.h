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

#ifndef PROVIZIO_RADAR_API_ROS2_CONSTANTS_UDP
#define PROVIZIO_RADAR_API_ROS2_CONSTANTS_UDP

#include <cstdint>
#include <string>

namespace provizio
{
    constexpr std::uint64_t ns_in_s = 1000000000;
    constexpr std::uint64_t receive_timeout_ns = ns_in_s / 10;
    constexpr std::size_t default_max_radars = 6;
    extern const std::string field_x_name;
    extern const std::string field_y_name;
    extern const std::string field_z_name;
    extern const std::string field_radar_relative_radial_velocity_name;
    extern const std::string field_signal_to_noise_ratio_name;
    extern const std::string field_ground_relative_radial_velocity_name;
    extern const std::string max_radars_param;
    extern const std::string pc_udp_port_param;
    extern const std::string set_range_udp_port_param;
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_CONSTANTS_UDP
