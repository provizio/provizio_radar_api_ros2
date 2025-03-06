#ifndef PROVIZIO_RADAR_API_ROS2_COMMON
#define PROVIZIO_RADAR_API_ROS2_COMMON

#include <string>

namespace provizio
{
    extern const std::string param_enable_radar_point_clouds;

    template <typename node_t> void declare_common_parameters(node_t &node)
    {
        node.declare_parameter(param_enable_radar_point_clouds, true);
    }
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_COMMON
