#include "provizio_radar_api_ros2/radar_api_ros2_wrapper_udp.h"

namespace provizio
{
    namespace
    {
        const std::string frame_id_prefix = "provizio_radar_"; // NOLINT: Can't throw
    } // namespace

    void make_sure_sockets_initialized()
    {
        static struct sockets_initialization_guard
        {
            sockets_initialization_guard()
            {
                provizio_sockets_initialize();
            }

            ~sockets_initialization_guard()
            {
                provizio_sockets_deinitialize();
            }
        } guard;
        (void)guard;
    }

    builtin_interfaces::msg::Time ns_to_ros2_time(const std::uint64_t timestamp)
    {
        builtin_interfaces::msg::Time result;
        result.sec = static_cast<std::int32_t>(timestamp / ns_in_s);
        result.nanosec = static_cast<std::int32_t>(timestamp % ns_in_s);
        return result;
    }

    std::string radar_position_id_to_frame_id(const std::uint16_t position_id)
    {
        switch (position_id)
        {
        case provizio_radar_position_unknown:
            return {};

        case provizio_radar_position_front_center:
            return frame_id_prefix + "front_center";

        case provizio_radar_position_front_left:
            return frame_id_prefix + "front_left";

        case provizio_radar_position_front_right:
            return frame_id_prefix + "front_right";

        case provizio_radar_position_rear_left:
            return frame_id_prefix + "rear_left";

        case provizio_radar_position_rear_right:
            return frame_id_prefix + "rear_right";

        case provizio_radar_position_rear_center:
            return frame_id_prefix + "rear_center";

        default:
            // position number instead
            return frame_id_prefix + std::to_string(position_id);
        }
    }

    std::int8_t udp_api_radar_range_to_ros2_range(const std::uint16_t udp_api_radar_range)
    {
        switch (udp_api_radar_range)
        {
        case provizio_radar_range_short:
            return provizio_radar_api_ros2::msg::RadarInfo::SHORT_RANGE;

        case provizio_radar_range_medium:
            return provizio_radar_api_ros2::msg::RadarInfo::MEDIUM_RANGE;

        case provizio_radar_range_long:
            return provizio_radar_api_ros2::msg::RadarInfo::LONG_RANGE;

        case provizio_radar_range_ultra_long:
            return provizio_radar_api_ros2::msg::RadarInfo::ULTRA_LONG_RANGE;

        case provizio_radar_range_hyper_long:
            return provizio_radar_api_ros2::msg::RadarInfo::HYPER_LONG_RANGE;

        default:
            return provizio_radar_api_ros2::msg::RadarInfo::UNKNOWN_RANGE;
        }
    }
} // namespace provizio
