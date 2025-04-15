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
    // NOLINTEND
} // namespace provizio
