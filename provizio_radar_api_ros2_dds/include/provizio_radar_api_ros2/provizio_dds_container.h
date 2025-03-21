#ifndef PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINER
#define PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINER

#include "provizio_radar_api_ros2/provizio_dds_contained.h"

namespace provizio
{
    std::shared_ptr<void> make_dds_domain_participant(uint32_t domain_id = 0);
    std::shared_ptr<void> make_dds_subscriber_pointcloud2(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        on_message_function<provizio::contained_pointcloud2> on_message, on_message_context context);
    std::shared_ptr<void> make_dds_subscriber_odometry(const std::shared_ptr<void> &domain_participant,
                                                       const std::string &topic_name,
                                                       on_message_function<provizio::contained_odometry> on_message,
                                                       on_message_context context);
    std::shared_ptr<void> make_dds_subscriber_image(const std::shared_ptr<void> &domain_participant,
                                                    const std::string &topic_name,
                                                    on_message_function<provizio::contained_image> on_message,
                                                    on_message_context context);
    std::shared_ptr<void> make_dds_subscriber_polygon_instance_stamped(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        on_message_function<provizio::contained_polygon_instance_stamped> on_message, on_message_context context);
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINER
