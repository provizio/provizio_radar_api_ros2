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

#include "provizio_radar_api_ros2/constants_udp.h"

namespace provizio
{
    // NOLINTBEGIN: These global constants don't throw on construction
    const std::string field_x_name = "x";
    const std::string field_y_name = "y";
    const std::string field_z_name = "z";
    const std::string field_radar_relative_radial_velocity_name = "radar_relative_radial_velocity";
    const std::string field_signal_to_noise_ratio_name = "signal_to_noise_ratio";
    const std::string field_ground_relative_radial_velocity_name = "ground_relative_radial_velocity";
    const std::string max_radars_param = "max_radars";
    const std::string pc_udp_port_param = "point_clouds_udp_port";
    const std::string set_range_udp_port_param = "set_range_udp_port";
    const std::string set_range_ip_address_param = "set_range_ip_address";
    // NOLINTEND
} // namespace provizio
