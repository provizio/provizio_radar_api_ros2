#include "provizio_radar_api_ros2/constants_udp.h"

namespace provizio
{
    const std::string field_x_name = "x"; // NOLINT: Never throws
    const std::string field_y_name = "y"; // NOLINT: Never throws
    const std::string field_z_name = "z"; // NOLINT: Never throws
    const std::string field_radar_relative_radial_velocity_name =
        "radar_relative_radial_velocity";                                         // NOLINT: Never throws
    const std::string field_signal_to_noise_ratio_name = "signal_to_noise_ratio"; // NOLINT: Never throws
    const std::string field_ground_relative_radial_velocity_name =
        "ground_relative_radial_velocity";                             // NOLINT: Never throws
    const std::string max_radars_param = "max_radars";                 // NOLINT: Never throws
    const std::string pc_udp_port_param = "point_clouds_udp_port";     // NOLINT: Never throws
    const std::string set_range_udp_port_param = "set_range_udp_port"; // NOLINT: Never throws
} // namespace provizio