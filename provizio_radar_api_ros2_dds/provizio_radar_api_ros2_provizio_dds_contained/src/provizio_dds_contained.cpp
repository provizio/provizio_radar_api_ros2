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
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <future>
#include <iostream>
#include <memory>
#include <string>

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
        const std::shared_ptr<void> &domain_participant, const char *const service_name)
    {
        // service_name arrives as a C string (not std::string) for the same reason the request/response
        // payloads do: this is the extern "C" boundary of a dlmopen'd library, and the std::string is
        // constructed here, inside the contained runtime that will also destroy it.
        return provizio::dds::make_service_client<provizio::srv::set_radar_range_RequestPubSubType,
                                                  provizio::srv::set_radar_range_ResponsePubSubType>(
            std::static_pointer_cast<provizio::dds::DomainParticipant>(domain_participant),
            std::string{service_name != nullptr ? service_name : ""});
    }

    provizio::contained_set_radar_range_status provizio_dds_contained_request_set_radar_range(
        const std::shared_ptr<void> &service_client, const char *const frame_id, const char *const serial_number,
        const std::int8_t target_range, const std::int32_t header_stamp_sec, const std::uint32_t header_stamp_nanosec,
        const std::uint64_t timeout_ns, std::atomic<bool> *const should_stop, bool *const out_success,
        std::int8_t *const out_current_range, char *const out_error_message,
        const std::size_t out_error_message_capacity, std::int8_t *const out_supported_ranges,
        const std::size_t out_supported_ranges_capacity, std::size_t *const out_num_supported_ranges)
    {
        // std::atomic<bool> is passed by pointer across the boundary, which is only sound because it's
        // always lock-free: a lock-based implementation would use a different lock table in each of the
        // two linker namespaces, so the two sides wouldn't actually synchronize with each other.
        static_assert(std::atomic<bool>::is_always_lock_free,
                      "std::atomic<bool> must be lock-free to be shared across the dlmopen boundary");

        // Mandatory out_ parameters (should_stop is optional). Checked rather than assumed: this is a C
        // ABI, and a null dereference here would fault in the caller's namespace.
        if (out_success == nullptr || out_current_range == nullptr || out_error_message == nullptr ||
            out_supported_ranges == nullptr || out_num_supported_ranges == nullptr || out_error_message_capacity == 0)
        {
            return provizio::contained_set_radar_range_status::error;
        }

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

            // Poll should_stop periodically, but never wait past the deadline: waiting a full poll
            // interval unconditionally would overshoot the caller's timeout by up to that interval and
            // ignore timeouts shorter than it.
            //
            // timeout_ns arrives as an unsigned value over the C ABI, so it has to be bounded before it
            // becomes a signed duration. Clamping it to nanoseconds::max() is not enough: the clamped
            // value is then *added* to steady_clock::now(), and that addition is what overflows int64 -
            // producing exactly the deadline in the past the clamp is there to prevent. Bound it to what
            // can still be added to now() instead.
            constexpr auto poll_interval = std::chrono::milliseconds{100};
            const auto start = std::chrono::steady_clock::now();
            const auto max_timeout =
                static_cast<std::uint64_t>((std::chrono::nanoseconds::max() - start.time_since_epoch()).count());
            const auto deadline =
                start +
                std::chrono::nanoseconds{static_cast<std::chrono::nanoseconds::rep>(std::min(timeout_ns, max_timeout))};
            while (true)
            {
                if (should_stop != nullptr && should_stop->load())
                {
                    return provizio::contained_set_radar_range_status::interrupted;
                }

                const auto now = std::chrono::steady_clock::now();
                const auto expired = now >= deadline;
                // Past the deadline the wait is zero rather than skipped: a response that arrived during
                // the last poll interval is already in the future, and returning timed_out without
                // looking would fail a request that actually succeeded.
                const auto wait_for =
                    expired ? std::chrono::nanoseconds::zero()
                            : std::min(std::chrono::duration_cast<std::chrono::nanoseconds>(poll_interval),
                                       std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now));

                if (future.wait_for(wait_for) == std::future_status::ready)
                {
                    // Build the response inside this library, then copy it out through POD buffers so no
                    // std::string / std::vector crosses the boundary.
                    const auto response = provizio::to_contained_set_radar_range_response(future.get());
                    *out_success = response.success;
                    *out_current_range = response.current_range;

                    const std::size_t message_length =
                        std::min(response.error_message.size(), out_error_message_capacity - 1);
                    std::memcpy(out_error_message, response.error_message.data(), message_length);
                    out_error_message[message_length] = '\0';

                    const std::size_t num_ranges =
                        std::min(response.supported_ranges.size(), out_supported_ranges_capacity);
                    std::copy_n(response.supported_ranges.begin(), num_ranges, out_supported_ranges);
                    *out_num_supported_ranges = num_ranges;

                    // Truncation would otherwise be indistinguishable from a complete response to the
                    // ROS 2 client, so make it visible at least in the node's output.
                    if (message_length < response.error_message.size())
                    {
                        std::cerr << "[provizio_dds_contained] set_radar_range error_message truncated from "
                                  << response.error_message.size() << " to " << message_length << " bytes" << std::endl;
                    }
                    if (num_ranges < response.supported_ranges.size())
                    {
                        std::cerr << "[provizio_dds_contained] set_radar_range supported_ranges truncated from "
                                  << response.supported_ranges.size() << " to " << num_ranges << " entries"
                                  << std::endl;
                    }

                    return provizio::contained_set_radar_range_status::ok;
                }

                if (expired)
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
