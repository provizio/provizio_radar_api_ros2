#include "provizio_radar_api_ros2/radar_api_ros2_wrapper_udp.h"

namespace provizio
{
    builtin_interfaces::msg::Time ns_to_ros2_time(const std::uint64_t timestamp)
    {
        builtin_interfaces::msg::Time result;
        result.sec = static_cast<std::int32_t>(timestamp / ns_in_s);
        result.nanosec = static_cast<std::int32_t>(timestamp % ns_in_s);
        return result;
    }

    std::string radar_position_id_to_frame_id(const std::uint16_t position_id)
    {
        return todo(position_id);
    }
} // namespace provizio
