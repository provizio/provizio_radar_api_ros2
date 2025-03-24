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

#include <nav_msgs/msg/odometry.hpp>
#include <provizio_radar_api_ros2/msg/radar_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "provizio_radar_api_ros2/features.h"
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#endif
#include <geometry_msgs/msg/polygon_stamped.hpp>

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
        static void on_radar_point_cloud_sr(void *context, contained_pointcloud2 message);
        static void on_entities_radar(void *context, contained_pointcloud2 message);
        static void on_entities_camera(void *context, contained_pointcloud2 message);
        static void on_entities_fusion(void *context, contained_pointcloud2 message);
        static void on_radar_odometry(void *context, contained_odometry message);
        static void on_camera(void *context, contained_image message);
        static void on_radar_freespace(void *context, contained_polygon_instance_stamped message);
        static void on_radar_info(void *context, contained_radar_info message);

        const rclcpp::QoS default_ros2_qos{2};
        const std::uint32_t dds_domain_id = 0; // TODO: Read from config, if specified

        node_t &node;
        std::shared_ptr<void> dds_domain_participant;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_radar_pc_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_radar_pc_sr_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_entities_radar_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_entities_camera_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_entities_fusion_publisher;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> ros2_radar_odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> ros2_camera_publisher;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>> ros2_radar_freespace_publisher;
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PolygonInstanceStamped>>
            ros2_radar_freespace_instance_publisher;
#endif
        std::shared_ptr<rclcpp::Publisher<provizio_radar_api_ros2::msg::RadarInfo>> ros2_radar_info_publisher;

        std::shared_ptr<void> dds_radar_pc_subscriber;
        std::shared_ptr<void> dds_radar_pc_sr_subscriber;
        std::shared_ptr<void> dds_entities_radar_subscriber;
        std::shared_ptr<void> dds_entities_camera_subscriber;
        std::shared_ptr<void> dds_entities_fusion_subscriber;
        std::shared_ptr<void> dds_radar_odometry_subscriber;
        std::shared_ptr<void> dds_camera_subscriber;
        std::shared_ptr<void> dds_radar_freespace_subscriber;
        std::shared_ptr<void> dds_radar_info_subscriber;
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
        ros2_radar_pc_sr_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
            radar_pc_sr_ros2_topic_name, default_ros2_qos);
        ros2_entities_radar_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
            entities_radar_ros2_topic_name, default_ros2_qos);
        ros2_entities_camera_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
            entities_camera_ros2_topic_name, default_ros2_qos);
        ros2_entities_fusion_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
            entities_fusion_ros2_topic_name, default_ros2_qos);
        ros2_radar_odometry_publisher =
            node.template create_publisher<nav_msgs::msg::Odometry>(radar_odometry_ros2_topic_name, default_ros2_qos);
        ros2_camera_publisher =
            node.template create_publisher<sensor_msgs::msg::Image>(camera_ros2_topic_name, default_ros2_qos);
        ros2_radar_freespace_publisher = node.template create_publisher<geometry_msgs::msg::PolygonStamped>(
            radar_freespace_ros2_topic_name, default_ros2_qos);
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        ros2_radar_freespace_instance_publisher =
            node.template create_publisher<geometry_msgs::msg::PolygonInstanceStamped>(
                radar_freespace_instance_ros2_topic_name, default_ros2_qos);
#endif
        ros2_radar_info_publisher = node.template create_publisher<provizio_radar_api_ros2::msg::RadarInfo>(
            radar_info_ros2_topic_name, default_ros2_qos);

        // Create subscribers
        // TODO: Read topic names from config, if specified
        dds_radar_pc_subscriber = make_dds_subscriber_pointcloud2(dds_domain_participant, radar_pc_dds_topic_name,
                                                                  &on_radar_point_cloud, this);
        dds_radar_pc_sr_subscriber = make_dds_subscriber_pointcloud2(dds_domain_participant, radar_pc_sr_dds_topic_name,
                                                                     &on_radar_point_cloud_sr, this);
        dds_entities_radar_subscriber = make_dds_subscriber_pointcloud2(
            dds_domain_participant, entities_radar_dds_topic_name, &on_entities_radar, this);
        dds_entities_camera_subscriber = make_dds_subscriber_pointcloud2(
            dds_domain_participant, entities_camera_dds_topic_name, &on_entities_camera, this);
        dds_entities_fusion_subscriber = make_dds_subscriber_pointcloud2(
            dds_domain_participant, entities_fusion_dds_topic_name, &on_entities_fusion, this);
        dds_radar_odometry_subscriber = make_dds_subscriber_odometry(
            dds_domain_participant, radar_odometry_dds_topic_name, &on_radar_odometry, this);
        dds_camera_subscriber =
            make_dds_subscriber_image(dds_domain_participant, camera_dds_topic_name, &on_camera, this);
        dds_radar_freespace_subscriber = make_dds_subscriber_polygon_instance_stamped(
            dds_domain_participant, radar_freespace_dds_topic_name, &on_radar_freespace, this);
        dds_radar_info_subscriber =
            make_dds_subscriber_radar_info(dds_domain_participant, radar_info_dds_topic_name, &on_radar_info, this);

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
        dds_radar_pc_sr_subscriber.reset();
        dds_entities_radar_subscriber.reset();
        dds_entities_camera_subscriber.reset();
        dds_entities_fusion_subscriber.reset();
        dds_radar_odometry_subscriber.reset();
        dds_camera_subscriber.reset();
        dds_radar_freespace_subscriber.reset();

        // Destroy the publishers
        ros2_radar_pc_publisher.reset();
        ros2_radar_pc_sr_publisher.reset();
        ros2_entities_radar_publisher.reset();
        ros2_entities_camera_publisher.reset();
        ros2_entities_fusion_publisher.reset();
        ros2_radar_odometry_publisher.reset();
        ros2_camera_publisher.reset();
        ros2_radar_freespace_publisher.reset();
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        ros2_radar_freespace_instance_publisher.reset();
#endif

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

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_point_cloud_sr(void *context, contained_pointcloud2 message)
    {
        auto publisher =
            static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                ->ros2_radar_pc_sr_publisher; // To make sure it can't be destroyed by another thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_entities_radar(void *context, contained_pointcloud2 message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_entities_radar_publisher; // To make sure it can't be destroyed by another thread
                                                              // during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_entities_camera(void *context, contained_pointcloud2 message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_entities_camera_publisher; // To make sure it can't be destroyed by another thread
                                                               // during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_entities_fusion(void *context, contained_pointcloud2 message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_entities_fusion_publisher; // To make sure it can't be destroyed by another thread
                                                               // during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_odometry(void *context, contained_odometry message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_radar_odometry_publisher; // To make sure it can't be destroyed by another thread
                                                              // during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_odometry(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_camera(void *context, contained_image message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_camera_publisher; // To make sure it can't be destroyed by another thread
                                                      // during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_image(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_freespace(void *context,
                                                                contained_polygon_instance_stamped message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_radar_freespace_publisher; // To make sure it can't be destroyed by another thread
                                                               // during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_polygon_stamped(message));
        }

#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        auto instance_publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                                      ->ros2_radar_freespace_instance_publisher; // Same logic as above
        if (instance_publisher != nullptr)
        {
            instance_publisher->publish(to_ros2_polygon_instance_stamped(message));
        }
#endif
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_info(void *context, contained_radar_info message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_radar_info_publisher; // To make sure it can't be destroyed by another thread
                                                          // during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_radar_info(std::move(message)));
        }
    }
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_DDS
