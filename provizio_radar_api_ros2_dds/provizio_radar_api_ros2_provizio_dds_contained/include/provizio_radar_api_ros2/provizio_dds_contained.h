#ifndef PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED
#define PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED

#include "provizio_radar_api_ros2/provizio_dds_contained_types.h"

#include <memory>
#include <string>

namespace provizio
{
    using on_message_context = void *;
    template <typename contained_data_type>
    using on_message_function = void (*)(on_message_context context, contained_data_type message);
} // namespace provizio

extern "C"
{
    std::shared_ptr<void> provizio_dds_contained_make_domain_participant(std::uint32_t domain_id);
    std::shared_ptr<void> provizio_dds_contained_make_subscriber_pointcloud2(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_pointcloud2> on_message,
        provizio::on_message_context context);
    std::shared_ptr<void> provizio_dds_contained_make_subscriber_odometry(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_odometry> on_message, provizio::on_message_context context);
    std::shared_ptr<void> provizio_dds_contained_make_subscriber_image(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_image> on_message, provizio::on_message_context context);
    std::shared_ptr<void> provizio_dds_contained_make_subscriber_polygon_instance_stamped(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_polygon_instance_stamped> on_message,
        provizio::on_message_context context);
}

#endif // PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED
