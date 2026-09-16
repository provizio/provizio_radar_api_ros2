// Copyright 2025 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "provizio_radar_api_ros2/provizio_dds_contained.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <future>
#include <memory>

#include "geometry_msgs/msg/PolygonInstanceStampedPubSubTypes.h"
#include "nav_msgs/msg/OdometryPubSubTypes.h"
#include "provizio/msg/radar_infoPubSubTypes.h"
#include "provizio/srv/set_radar_rangePubSubTypes.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"
#include "sensor_msgs/msg/PointCloud2PubSubTypes.h"

#include "provizio/dds/request_response.h"
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

    std::shared_ptr<void> provizio_dds_contained_make_service_client_set_radar_range(
        const std::shared_ptr<void> &domain_participant, const std::string &service_name)
    {
        return provizio::dds::make_service_client<provizio::srv::set_radar_range_RequestPubSubType,
                                                  provizio::srv::set_radar_range_ResponsePubSubType>(
            std::static_pointer_cast<provizio::dds::DomainParticipant>(domain_participant), service_name);
    }

    provizio::contained_set_radar_range_status provizio_dds_contained_request_set_radar_range(
        const std::shared_ptr<void> &service_client, const char *const frame_id, const char *const serial_number,
        const std::int8_t target_range, const std::int32_t header_stamp_sec, const std::uint32_t header_stamp_nanosec,
        const std::uint64_t timeout_ns, std::atomic<bool> *const should_stop, bool *const out_success,
        std::int8_t *const out_current_range, char *const out_error_message, std::int8_t *const out_supported_ranges,
        std::size_t *const out_num_supported_ranges)
    {
        // This function is the C-ABI boundary of a library loaded via dlmopen into a separate linker
        // namespace, so: (1) no exception may escape it (that would cross the boundary and std::terminate),
        // and (2) no std::string / std::vector may cross it - the request arrives as C strings / POD and the
        // response leaves through the caller-owned POD out_* buffers.
        try
        {
            using client_type = provizio::dds::service_client<provizio::srv::set_radar_range_RequestPubSubType,
                                                              provizio::srv::set_radar_range_ResponsePubSubType>;

            auto client = std::static_pointer_cast<client_type>(service_client);
            if (!client)
            {
                return provizio::contained_set_radar_range_status::error;
            }

            auto dds_request = provizio::to_dds_set_radar_range_request(frame_id, serial_number, target_range,
                                                                        header_stamp_sec, header_stamp_nanosec);
            auto future = client->request(dds_request);

            constexpr auto poll_interval = std::chrono::milliseconds{100};
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::nanoseconds{static_cast<std::chrono::nanoseconds::rep>(timeout_ns)};
            while (true)
            {
                if (should_stop != nullptr && should_stop->load())
                {
                    return provizio::contained_set_radar_range_status::timed_out;
                }

                if (future.wait_for(poll_interval) == std::future_status::ready)
                {
                    // Build the response inside this library, then copy it out through POD buffers so no
                    // std::string / std::vector crosses the boundary.
                    const auto response = provizio::to_contained_set_radar_range_response(future.get());
                    *out_success = response.success;
                    *out_current_range = response.current_range;

                    const std::size_t message_length =
                        std::min(response.error_message.size(),
                                 provizio::contained_set_radar_range_error_message_capacity - 1);
                    std::memcpy(out_error_message, response.error_message.data(), message_length);
                    out_error_message[message_length] = '\0';

                    const std::size_t num_ranges = std::min(
                        response.supported_ranges.size(), provizio::contained_set_radar_range_max_supported_ranges);
                    std::copy_n(response.supported_ranges.begin(), num_ranges, out_supported_ranges);
                    *out_num_supported_ranges = num_ranges;

                    return provizio::contained_set_radar_range_status::ok;
                }

                if (std::chrono::steady_clock::now() >= deadline)
                {
                    return provizio::contained_set_radar_range_status::timed_out;
                }
            }
        }
        catch (...)
        {
            return provizio::contained_set_radar_range_status::error;
        }
    }
}
