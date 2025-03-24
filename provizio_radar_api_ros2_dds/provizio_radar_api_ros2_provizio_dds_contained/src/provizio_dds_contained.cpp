#include "provizio_radar_api_ros2/provizio_dds_contained.h"

#include "geometry_msgs/msg/PolygonInstanceStampedPubSubTypes.h"
#include "nav_msgs/msg/OdometryPubSubTypes.h"
#include "provizio/msg/radar_infoPubSubTypes.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"
#include "sensor_msgs/msg/PointCloud2PubSubTypes.h"

#include "provizio/dds/subscriber.h"

#include "provizio_dds_contained_types_dds_conversion.h"

extern "C"
{
    std::shared_ptr<void> provizio_dds_contained_make_domain_participant(const std::uint32_t domain_id)
    {
        return provizio::dds::make_domain_participant(domain_id);
    }

    std::shared_ptr<void> provizio_dds_contained_make_subscriber_pointcloud2(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_pointcloud2> on_message, provizio::on_message_context context)
    {
        return provizio::dds::make_subscriber<sensor_msgs::msg::PointCloud2PubSubType>(
            std::static_pointer_cast<provizio::dds::DomainParticipant>(domain_participant), topic_name,
            [context, on_message](const sensor_msgs::msg::PointCloud2 &message) {
                return on_message(context, provizio::to_contained_pointcloud2(message));
            });
    }

    std::shared_ptr<void> provizio_dds_contained_make_subscriber_odometry(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_odometry> on_message, provizio::on_message_context context)
    {
        return provizio::dds::make_subscriber<nav_msgs::msg::OdometryPubSubType>(
            std::static_pointer_cast<provizio::dds::DomainParticipant>(domain_participant), topic_name,
            [context, on_message](const nav_msgs::msg::Odometry &message) {
                return on_message(context, provizio::to_contained_odometry(message));
            });
    }

    std::shared_ptr<void> provizio_dds_contained_make_subscriber_image(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_image> on_message, provizio::on_message_context context)
    {
        return provizio::dds::make_subscriber<sensor_msgs::msg::ImagePubSubType>(
            std::static_pointer_cast<provizio::dds::DomainParticipant>(domain_participant), topic_name,
            [context, on_message](const sensor_msgs::msg::Image &message) {
                return on_message(context, provizio::to_contained_image(message));
            });
    }

    std::shared_ptr<void> provizio_dds_contained_make_subscriber_polygon_instance_stamped(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_polygon_instance_stamped> on_message,
        provizio::on_message_context context)
    {
        return provizio::dds::make_subscriber<geometry_msgs::msg::PolygonInstanceStampedPubSubType>(
            std::static_pointer_cast<provizio::dds::DomainParticipant>(domain_participant), topic_name,
            [context, on_message](const geometry_msgs::msg::PolygonInstanceStamped &message) {
                return on_message(context, provizio::to_contained_polygon_instance_stamped(message));
            });
    }

    std::shared_ptr<void> provizio_dds_contained_make_subscriber_radar_info(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_radar_info> on_message, provizio::on_message_context context)
    {
        return provizio::dds::make_subscriber<provizio::msg::radar_infoPubSubType>(
            std::static_pointer_cast<provizio::dds::DomainParticipant>(domain_participant), topic_name,
            [context, on_message](const provizio::msg::radar_info &message) {
                return on_message(context, provizio::to_contained_radar_info(message));
            });
    }
}
