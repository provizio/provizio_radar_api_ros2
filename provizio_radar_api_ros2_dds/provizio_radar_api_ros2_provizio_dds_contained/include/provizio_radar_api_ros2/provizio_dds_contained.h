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
    std::shared_ptr<void> provizio_dds_contained_make_publisher_set_radar_range(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name);
    bool provizio_dds_contained_publish_set_radar_range(const std::shared_ptr<void> &publisher,
                                                        provizio::contained_set_radar_range message);
}

#endif // PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINED
