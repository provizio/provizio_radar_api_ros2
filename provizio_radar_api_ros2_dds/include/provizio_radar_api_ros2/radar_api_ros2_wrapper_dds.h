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

#ifndef PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_DDS
#define PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_DDS

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "provizio_radar_api_ros2/constants.h"
#include "provizio_radar_api_ros2/constants_dds.h"
#include "provizio_radar_api_ros2/provizio_dds_contained_types_ros2_conversion.h"
#include "provizio_radar_api_ros2/provizio_dds_container.h"

namespace provizio
{
    template <typename node_t> class radar_api_ros2_wrapper_dds
    {
      public:
        radar_api_ros2_wrapper_dds(node_t &node) : node(node)
        {
        }

        bool activate();
        bool deactivate();

      private:
        static void on_radar_point_cloud(void *context, contained_pointcloud2 message);

        const rclcpp::QoS default_ros2_qos{10};
        const std::uint32_t dds_domain_id = 0; // TODO: Read from config, if specified

        node_t &node;
        std::shared_ptr<void> dds_domain_participant;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_radar_pc_publisher;
        std::shared_ptr<void> dds_radar_pc_subscriber;
    };

    template <typename node_t> bool radar_api_ros2_wrapper_dds<node_t>::activate()
    {
        if (dds_domain_participant != nullptr)
        {
            // Already active
            return false;
        }

        // dds_domain_participant
        dds_domain_participant = make_dds_domain_participant(dds_domain_id);

        // Create publishers first, so any subscriber can re-publish straight away
        // TODO: Read topic names from config, if specified
        ros2_radar_pc_publisher =
            node.template create_publisher<sensor_msgs::msg::PointCloud2>(radar_pc_ros2_topic_name, default_ros2_qos);

        // Create subscribers
        // TODO: Read topic names from config, if specified
        dds_radar_pc_subscriber = make_dds_subscriber_pointcloud2(dds_domain_participant, radar_pc_dds_topic_name,
                                                                  &on_radar_point_cloud, this);

        return true;
    }

    template <typename node_t> bool radar_api_ros2_wrapper_dds<node_t>::deactivate()
    {
        if (dds_domain_participant == nullptr)
        {
            // Already deactivated
            return false;
        }

        // Destroy the subscribers first so none of them tries to publish with destroyed publishers
        dds_radar_pc_subscriber.reset();

        // Destroy the publishers
        ros2_radar_pc_publisher.reset();

        // dds_domain_participant
        dds_domain_participant.reset();

        return true;
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_point_cloud(void *context, contained_pointcloud2 message)
    {
        auto publisher =
            static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                ->ros2_radar_pc_publisher; // To make sure it can't be destroyed by another thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_DDS
