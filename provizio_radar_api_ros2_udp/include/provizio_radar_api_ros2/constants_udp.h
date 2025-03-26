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
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_CONSTANTS_UDP
