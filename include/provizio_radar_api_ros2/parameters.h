#ifndef PROVIZIO_RADAR_API_ROS2_PARAMETERS
#define PROVIZIO_RADAR_API_ROS2_PARAMETERS

#include "provizio_radar_api_ros2/constants.h"

namespace provizio
{
    template <typename node_type> void declare_common_parameters(node_type &node)
    {
        node.declare_parameter(radar_pc_ros2_topic_name_param, default_radar_pc_ros2_topic_name);
    }
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_PARAMETERS
