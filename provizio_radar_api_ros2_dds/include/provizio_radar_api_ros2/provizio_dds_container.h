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
    std::shared_ptr<void> make_dds_subscriber_radar_info(const std::shared_ptr<void> &domain_participant,
                                                         const std::string &topic_name,
                                                         on_message_function<provizio::contained_radar_info> on_message,
                                                         on_message_context context);
    std::shared_ptr<void> make_dds_publisher_set_radar_range(const std::shared_ptr<void> &domain_participant,
                                                             const std::string &topic_name);
    bool dds_publish_set_radar_range(const std::shared_ptr<void> &publisher,
                                     provizio::contained_set_radar_range message);
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_PROVIZIO_DDS_CONTAINER
