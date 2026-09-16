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

#ifndef PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED
#define PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED

#include "provizio_radar_api_ros2/provizio_dds_contained_types.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
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
    std::shared_ptr<void> provizio_dds_contained_make_subscriber_radar_info(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        provizio::on_message_function<provizio::contained_radar_info> on_message, provizio::on_message_context context);
    // Creates a persistent request/response client for the "provizio_set_radar_range" service. The
    // request/response DDS topics are inferred from service_name (rq/<service_name>Request,
    // rr/<service_name>Reply).
    std::shared_ptr<void> provizio_dds_contained_make_service_client_set_radar_range(
        const std::shared_ptr<void> &domain_participant, const char *service_name);
    // Issues a single set_radar_range request and waits (up to timeout_ns, polling should_stop so the
    // caller can interrupt on shutdown) for the radar's response. On contained_set_radar_range_status::ok
    // the out_* parameters are populated with the radar's response.
    //
    // The request/response is marshalled as C strings and POD arrays (never std::string / std::vector),
    // because this function is the extern "C" boundary of a library dlmopen'd into a separate C++ runtime
    // / heap namespace. The capacities of the caller-owned out_ buffers are passed explicitly rather than
    // taken from contained_set_radar_range_error_message_capacity /
    // contained_set_radar_range_max_supported_ranges: each side of the boundary compiles its own copy of
    // those constants, so relying on them here would silently turn any version skew between the node and
    // the contained .so into a heap overflow in the *caller's* namespace. All pointer parameters except
    // should_stop are mandatory; the function returns "error" if any of them is null.
    provizio::contained_set_radar_range_status provizio_dds_contained_request_set_radar_range(
        const std::shared_ptr<void> &service_client, const char *frame_id, const char *serial_number,
        std::int8_t target_range, std::int32_t header_stamp_sec, std::uint32_t header_stamp_nanosec,
        std::uint64_t timeout_ns, std::atomic<bool> *should_stop, bool *out_success, std::int8_t *out_current_range,
        char *out_error_message, std::size_t out_error_message_capacity, std::int8_t *out_supported_ranges,
        std::size_t out_supported_ranges_capacity, std::size_t *out_num_supported_ranges);
}

#endif // PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED
