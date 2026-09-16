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

#include <atomic>
#include <chrono>
#include <exception>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>

#include <nav_msgs/msg/odometry.hpp>
#include <provizio_radar_api_ros2/msg/radar_info.hpp>
#include <provizio_radar_api_ros2/srv/set_radar_range.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "provizio_radar_api_ros2/features.h"
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
#include <geometry_msgs/msg/polygon_instance_stamped.hpp>
#endif
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include "provizio_radar_api_ros2/constants.h"
#include "provizio_radar_api_ros2/constants_dds.h"
#include "provizio_radar_api_ros2/parameters.h"
#include "provizio_radar_api_ros2/provizio_dds_contained_types_ros2_conversion.h"
#include "provizio_radar_api_ros2/provizio_dds_container.h"

namespace provizio
{
    template <typename node_t> class radar_api_ros2_wrapper_dds
    {
      public:
        radar_api_ros2_wrapper_dds(node_t &node, rclcpp::Executor & /*unused*/) : node(node)
        {
            // Declare all of the Node parameters
            declare_common_parameters(node);
            node.declare_parameter(dds_domain_id_param, static_cast<int>(default_dds_domain_id));
            node.declare_parameter(publish_radar_pc_sr_param, feature_enabled_by_default);
            node.declare_parameter(publish_entities_radar_param, feature_enabled_by_default);
            node.declare_parameter(publish_entities_camera_param, feature_enabled_by_default);
            node.declare_parameter(publish_entities_fusion_param, feature_enabled_by_default);
            node.declare_parameter(publish_radar_odometry_param, feature_enabled_by_default);
            node.declare_parameter(publish_camera_param, feature_enabled_by_default);
            node.declare_parameter(publish_radar_freespace_param, feature_enabled_by_default);
            node.declare_parameter(publish_camera_freespace_param, feature_enabled_by_default);
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
            node.declare_parameter(publish_radar_freespace_instance_param, feature_enabled_by_default);
            node.declare_parameter(publish_camera_freespace_instance_param, feature_enabled_by_default);
#endif
            node.declare_parameter(radar_pc_sr_ros2_topic_name_param, default_radar_pc_sr_ros2_topic_name);
            node.declare_parameter(entities_radar_ros2_topic_name_param, default_entities_radar_ros2_topic_name);
            node.declare_parameter(entities_camera_ros2_topic_name_param, default_entities_camera_ros2_topic_name);
            node.declare_parameter(entities_fusion_ros2_topic_name_param, default_entities_fusion_ros2_topic_name);
            node.declare_parameter(radar_odometry_ros2_topic_name_param, default_radar_odometry_ros2_topic_name);
            node.declare_parameter(camera_ros2_topic_name_param, default_camera_ros2_topic_name);
            node.declare_parameter(radar_freespace_ros2_topic_name_param, default_radar_freespace_ros2_topic_name);
            node.declare_parameter(camera_freespace_ros2_topic_name_param, default_camera_freespace_ros2_topic_name);
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
            node.declare_parameter(radar_freespace_ros2_instance_topic_name_param,
                                   default_radar_freespace_ros2_instance_topic_name);
            node.declare_parameter(camera_freespace_ros2_instance_topic_name_param,
                                   default_camera_freespace_ros2_instance_topic_name);
#endif
        }

        ~radar_api_ros2_wrapper_dds()
        {
            if (dds_domain_participant != nullptr)
            {
                deactivate();
            }
        }

        bool activate();
        bool deactivate();

      private:
        // Functions
        static void on_radar_point_cloud(void *context, contained_pointcloud2 message);
        static void on_radar_point_cloud_sr(void *context, contained_pointcloud2 message);
        static void on_entities_radar(void *context, contained_pointcloud2 message);
        static void on_entities_camera(void *context, contained_pointcloud2 message);
        static void on_entities_fusion(void *context, contained_pointcloud2 message);
        static void on_radar_odometry(void *context, contained_odometry message);
        static void on_camera(void *context, contained_image message);
        static void on_radar_freespace(void *context, contained_polygon_instance_stamped message);
        static void on_camera_freespace(void *context, contained_polygon_instance_stamped message);
        static void on_radar_info(void *context, contained_radar_info message);
        void on_set_radar_range_request(
            const std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Request> request,
            std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Response> response);
        bool filter_by_frame_id(const std::string &message_frame_id);

        // Variables
        node_t &node;
        std::string frame_id{default_frame_id};
        float snr_threshold{default_snr_threshold};
        std::shared_ptr<void> dds_domain_participant;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_radar_pc_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_radar_pc_sr_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_entities_radar_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_entities_camera_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_entities_fusion_publisher;
        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> ros2_radar_odometry_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> ros2_camera_publisher;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>> ros2_radar_freespace_publisher;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>> ros2_camera_freespace_publisher;
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PolygonInstanceStamped>>
            ros2_radar_freespace_instance_publisher;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PolygonInstanceStamped>>
            ros2_camera_freespace_instance_publisher;
#endif
        std::shared_ptr<rclcpp::Publisher<provizio_radar_api_ros2::msg::RadarInfo>> ros2_radar_info_publisher;
        std::shared_ptr<void> dds_set_radar_range_client;

        std::shared_ptr<void> dds_radar_pc_subscriber;
        std::shared_ptr<void> dds_radar_pc_sr_subscriber;
        std::shared_ptr<void> dds_entities_radar_subscriber;
        std::shared_ptr<void> dds_entities_camera_subscriber;
        std::shared_ptr<void> dds_entities_fusion_subscriber;
        std::shared_ptr<void> dds_radar_odometry_subscriber;
        std::shared_ptr<void> dds_camera_subscriber;
        std::shared_ptr<void> dds_radar_freespace_subscriber;
        std::shared_ptr<void> dds_camera_freespace_subscriber;
        std::shared_ptr<void> dds_radar_info_subscriber;

        std::shared_ptr<rclcpp::Service<provizio_radar_api_ros2::srv::SetRadarRange>> ros2_set_radar_range_service;
        // Set on deactivate() so an in-flight set_radar_range request stops waiting rather than blocking
        // up to max_time_to_set_radar_range.
        //
        // No current path actually reaches an in-flight request, because the service callback runs in the
        // node's default (mutually exclusive) callback group: an explicit deactivate transition is
        // serialised behind the very callback it would interrupt, and the shutdown path is too, since
        // MultiThreadedExecutor::spin() joins its worker threads before returning to main(). The flag is
        // kept because it is free, correct, and the half of the mechanism that isn't the callback group -
        // once a group whose lifetime is decoupled from this wrapper lands (see the revert of the
        // dedicated group, which segfaulted on humble when destroyed from on_cleanup), this starts
        // working with no further change here.
        std::atomic<bool> stop_set_radar_range{false};

        // Guards dds_set_radar_range_client against a concurrent reset() in deactivate(). Copying a
        // shared_ptr is not atomic with respect to a concurrent reset of the same object, so the copy in
        // on_set_radar_range_request must be made under this lock rather than bare.
        std::mutex dds_set_radar_range_client_mutex;

        std::mutex current_radar_ranges_mutex;
        std::unordered_map<std::string, std::int8_t> current_radar_ranges_by_frame_id;
        std::unordered_map<std::string, std::int8_t> current_radar_ranges_by_serial_number;
    };

    template <typename node_t> bool radar_api_ros2_wrapper_dds<node_t>::activate()
    {
        if (dds_domain_participant != nullptr)
        {
            // Already active
            return false;
        }

        frame_id = node.get_parameter(frame_id_param).as_string();
        snr_threshold = static_cast<float>(node.get_parameter(snr_threshold_param).as_double());
        RCLCPP_DEBUG(node.get_logger(), "Running with frame_id %s, snr_threshold=%f",
                     (frame_id.empty() ? "unrestricted" : frame_id.c_str()), snr_threshold);

        // dds_domain_participant
        dds_domain_participant =
            make_dds_domain_participant(static_cast<std::uint32_t>(node.get_parameter(dds_domain_id_param).as_int()));

        // Create publishers first and then subscribers so they can re-publish straight away

        if (node.get_parameter(publish_radar_pc_param).as_bool())
        {
            ros2_radar_pc_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
                node.get_parameter(radar_pc_ros2_topic_name_param).as_string(), default_ros2_qos);
            dds_radar_pc_subscriber = make_dds_subscriber_pointcloud2(dds_domain_participant, radar_pc_dds_topic_name,
                                                                      &on_radar_point_cloud, this);
        }

        if (node.get_parameter(publish_radar_pc_sr_param).as_bool())
        {
            ros2_radar_pc_sr_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
                node.get_parameter(radar_pc_sr_ros2_topic_name_param).as_string(), default_ros2_qos);
            dds_radar_pc_sr_subscriber = make_dds_subscriber_pointcloud2(
                dds_domain_participant, radar_pc_sr_dds_topic_name, &on_radar_point_cloud_sr, this);
        }

        if (node.get_parameter(publish_entities_radar_param).as_bool())
        {
            ros2_entities_radar_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
                node.get_parameter(entities_radar_ros2_topic_name_param).as_string(), default_ros2_qos);
            dds_entities_radar_subscriber = make_dds_subscriber_pointcloud2(
                dds_domain_participant, entities_radar_dds_topic_name, &on_entities_radar, this);
        }

        if (node.get_parameter(publish_entities_camera_param).as_bool())
        {
            ros2_entities_camera_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
                node.get_parameter(entities_camera_ros2_topic_name_param).as_string(), default_ros2_qos);
            dds_entities_camera_subscriber = make_dds_subscriber_pointcloud2(
                dds_domain_participant, entities_camera_dds_topic_name, &on_entities_camera, this);
        }

        if (node.get_parameter(publish_entities_fusion_param).as_bool())
        {
            ros2_entities_fusion_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
                node.get_parameter(entities_fusion_ros2_topic_name_param).as_string(), default_ros2_qos);
            dds_entities_fusion_subscriber = make_dds_subscriber_pointcloud2(
                dds_domain_participant, entities_fusion_dds_topic_name, &on_entities_fusion, this);
        }

        if (node.get_parameter(publish_radar_odometry_param).as_bool())
        {
            ros2_radar_odometry_publisher = node.template create_publisher<nav_msgs::msg::Odometry>(
                node.get_parameter(radar_odometry_ros2_topic_name_param).as_string(), default_ros2_qos);
            dds_radar_odometry_subscriber = make_dds_subscriber_odometry(
                dds_domain_participant, radar_odometry_dds_topic_name, &on_radar_odometry, this);
        }

        if (node.get_parameter(publish_camera_param).as_bool())
        {
            ros2_camera_publisher = node.template create_publisher<sensor_msgs::msg::Image>(
                node.get_parameter(camera_ros2_topic_name_param).as_string(), default_ros2_qos);
            dds_camera_subscriber =
                make_dds_subscriber_image(dds_domain_participant, camera_dds_topic_name, &on_camera, this);
        }

        if (node.get_parameter(publish_radar_freespace_param).as_bool())
        {
            ros2_radar_freespace_publisher = node.template create_publisher<geometry_msgs::msg::PolygonStamped>(
                node.get_parameter(radar_freespace_ros2_topic_name_param).as_string(), default_ros2_qos);
        }
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        if (node.get_parameter(publish_radar_freespace_instance_param).as_bool())
        {
            ros2_radar_freespace_instance_publisher =
                node.template create_publisher<geometry_msgs::msg::PolygonInstanceStamped>(
                    node.get_parameter(radar_freespace_ros2_instance_topic_name_param).as_string(), default_ros2_qos);
        }
#endif
        if (node.get_parameter(publish_radar_freespace_param).as_bool()
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
            || node.get_parameter(publish_radar_freespace_instance_param).as_bool()
#endif
        )
        {
            // dds_radar_freespace_subscriber provides data to both ros2_radar_freespace_publisher and
            // ros2_radar_freespace_instance_publisher
            dds_radar_freespace_subscriber = make_dds_subscriber_polygon_instance_stamped(
                dds_domain_participant, radar_freespace_dds_topic_name, &on_radar_freespace, this);
        }

        if (node.get_parameter(publish_camera_freespace_param).as_bool())
        {
            ros2_camera_freespace_publisher = node.template create_publisher<geometry_msgs::msg::PolygonStamped>(
                node.get_parameter(camera_freespace_ros2_topic_name_param).as_string(), default_ros2_qos);
        }
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        if (node.get_parameter(publish_camera_freespace_instance_param).as_bool())
        {
            ros2_camera_freespace_instance_publisher =
                node.template create_publisher<geometry_msgs::msg::PolygonInstanceStamped>(
                    node.get_parameter(camera_freespace_ros2_instance_topic_name_param).as_string(), default_ros2_qos);
        }
#endif
        if (node.get_parameter(publish_camera_freespace_param).as_bool()
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
            || node.get_parameter(publish_camera_freespace_instance_param).as_bool()
#endif
        )
        {
            // dds_camera_freespace_subscriber provides data to both ros2_camera_freespace_publisher and
            // ros2_camera_freespace_instance_publisher
            dds_camera_freespace_subscriber = make_dds_subscriber_polygon_instance_stamped(
                dds_domain_participant, camera_freespace_dds_topic_name, &on_camera_freespace, this);
        }

        if (node.get_parameter(publish_radar_info_param).as_bool())
        {
            ros2_radar_info_publisher = node.template create_publisher<provizio_radar_api_ros2::msg::RadarInfo>(
                node.get_parameter(radar_info_ros2_topic_name_param).as_string(), default_ros2_qos);
        }

        if (node.get_parameter(publish_radar_info_param).as_bool() ||
            node.get_parameter(serve_set_radar_range_param).as_bool())
        {
            // dds_radar_info_subscriber is used by both radar_info re-publishing and set_radar_range service
            dds_radar_info_subscriber =
                make_dds_subscriber_radar_info(dds_domain_participant, radar_info_dds_topic_name, &on_radar_info, this);
        }

        // Eventually, create services

        if (node.get_parameter(serve_set_radar_range_param).as_bool())
        {
            {
                std::lock_guard<std::mutex> lock{dds_set_radar_range_client_mutex};
                dds_set_radar_range_client =
                    make_dds_service_client_set_radar_range(dds_domain_participant, set_radar_range_dds_service_name);
            }
            stop_set_radar_range = false;
            ros2_set_radar_range_service = node.template create_service<provizio_radar_api_ros2::srv::SetRadarRange>(
                node.get_parameter(set_radar_range_ros2_service_name_param).as_string(),
                std::bind(&radar_api_ros2_wrapper_dds::on_set_radar_range_request, this, std::placeholders::_1,
                          std::placeholders::_2));
        }

        return true;
    }

    template <typename node_t> bool radar_api_ros2_wrapper_dds<node_t>::deactivate()
    {
        if (dds_domain_participant == nullptr)
        {
            // Already deactivated
            return false;
        }

        // Destroy the services. Signal any in-flight set_radar_range request to stop waiting first.
        stop_set_radar_range = true;
        ros2_set_radar_range_service.reset();

        // Destroy the subscribers first so none of them tries to publish with destroyed publishers
        dds_radar_pc_subscriber.reset();
        dds_radar_pc_sr_subscriber.reset();
        dds_entities_radar_subscriber.reset();
        dds_entities_camera_subscriber.reset();
        dds_entities_fusion_subscriber.reset();
        dds_radar_odometry_subscriber.reset();
        dds_camera_subscriber.reset();
        dds_radar_freespace_subscriber.reset();
        dds_camera_freespace_subscriber.reset();
        dds_radar_info_subscriber.reset();

        // Destroy the publishers
        ros2_radar_pc_publisher.reset();
        ros2_radar_pc_sr_publisher.reset();
        ros2_entities_radar_publisher.reset();
        ros2_entities_camera_publisher.reset();
        ros2_entities_fusion_publisher.reset();
        ros2_radar_odometry_publisher.reset();
        ros2_camera_publisher.reset();
        ros2_radar_freespace_publisher.reset();
        ros2_camera_freespace_publisher.reset();
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        ros2_radar_freespace_instance_publisher.reset();
        ros2_camera_freespace_instance_publisher.reset();
#endif
        ros2_radar_info_publisher.reset();
        {
            std::lock_guard<std::mutex> lock{dds_set_radar_range_client_mutex};
            dds_set_radar_range_client.reset();
        }

        // dds_domain_participant
        dds_domain_participant.reset();

        return true;
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_point_cloud(void *context, contained_pointcloud2 message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        auto publisher =
            self.ros2_radar_pc_publisher; // To make sure it can't be destroyed by another thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message), self.snr_threshold));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_point_cloud_sr(void *context, contained_pointcloud2 message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        auto publisher = self.ros2_radar_pc_sr_publisher; // To make sure it can't be destroyed by another thread
                                                          // during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message), self.snr_threshold));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_entities_radar(void *context, contained_pointcloud2 message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        auto publisher = self.ros2_entities_radar_publisher; // To make sure it can't be destroyed by another
                                                             // thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_entities_camera(void *context, contained_pointcloud2 message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        auto publisher = self.ros2_entities_camera_publisher; // To make sure it can't be destroyed by another
                                                              // thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_entities_fusion(void *context, contained_pointcloud2 message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        auto publisher = self.ros2_entities_fusion_publisher; // To make sure it can't be destroyed by another
                                                              // thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_odometry(void *context, contained_odometry message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        auto publisher = self.ros2_radar_odometry_publisher; // To make sure it can't be destroyed by another
                                                             // thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_odometry(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_camera(void *context, contained_image message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        auto publisher = self.ros2_camera_publisher; // To make sure it can't be destroyed by another thread
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
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        auto publisher = self.ros2_radar_freespace_publisher; // To make sure it can't be destroyed by another
                                                              // thread during this call
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
    void radar_api_ros2_wrapper_dds<node_t>::on_camera_freespace(void *context,
                                                                 contained_polygon_instance_stamped message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        auto publisher = self.ros2_camera_freespace_publisher; // To make sure it can't be destroyed by another
                                                               // thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_polygon_stamped(message));
        }

#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        auto instance_publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                                      ->ros2_camera_freespace_instance_publisher; // Same logic as above
        if (instance_publisher != nullptr)
        {
            instance_publisher->publish(to_ros2_polygon_instance_stamped(message));
        }
#endif
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_info(void *context, contained_radar_info message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        if (!self.filter_by_frame_id(message.header.frame_id))
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock{self.current_radar_ranges_mutex};
            self.current_radar_ranges_by_frame_id[message.header.frame_id] = message.current_range;
            if (!message.serial_number.empty())
            {
                self.current_radar_ranges_by_serial_number[message.serial_number] = message.current_range;
            }
        }

        auto publisher = self.ros2_radar_info_publisher; // To make sure it can't be destroyed by another thread
                                                         // during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_radar_info(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_set_radar_range_request(
        const std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Request> request,
        std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Response> response)
    {
        const auto get_current_radar_range = [this](const std::string &frame_id, const std::string &serial_number) {
            std::lock_guard<std::mutex> lock{current_radar_ranges_mutex};

            if (!serial_number.empty())
            {
                auto it = current_radar_ranges_by_serial_number.find(serial_number);
                if (it != current_radar_ranges_by_serial_number.end())
                {
                    // The range for this specific radar by serial_number is known
                    return it->second;
                }
            }

            auto it = current_radar_ranges_by_frame_id.find(frame_id);
            if (it == current_radar_ranges_by_frame_id.end())
            {
                return static_cast<std::int8_t>(provizio_radar_api_ros2::msg::RadarInfo::UNKNOWN_RANGE);
            }

            return it->second;
        };

        if (!this->frame_id.empty() && !request->header.frame_id.empty() && request->header.frame_id != this->frame_id)
        {
            RCLCPP_WARN(node.get_logger(),
                        "The node's frame_id %s doesn't match the request's (%s)! Assuming the node's one correct",
                        frame_id.c_str(), request->header.frame_id.c_str());
        }
        const auto &frame_id = !this->frame_id.empty() ? this->frame_id : request->header.frame_id;
        const auto &serial_number = request->serial_number;

        // Quick-set: if the radar already reports the requested range (via radar_info), skip the round-trip.
        if (get_current_radar_range(frame_id, serial_number) == request->target_range)
        {
            response->actual_range = request->target_range;
            response->success = true;
            return;
        }

        // Issue a request/response call to the radar's set_radar_range service and wait for its response.
        // Take a copy of the client under the lock: deactivate() may reset the member from another thread,
        // and copying a shared_ptr concurrently with a reset of the same object is a data race. The local
        // copy then keeps the client alive for the duration of the call.
        std::shared_ptr<void> client;
        {
            std::lock_guard<std::mutex> lock{dds_set_radar_range_client_mutex};
            client = dds_set_radar_range_client;
        }
        if (client == nullptr)
        {
            // Deactivated between the service callback starting and this point
            response->success = false;
            response->actual_range = get_current_radar_range(frame_id, serial_number);
            response->error_message = "The node is not active";
            return;
        }

        provizio::contained_set_radar_range_response contained_response;
        // Defaulted to error, not to a value-initialised ok: every path below assigns it, and if a
        // future edit introduces one that doesn't, failing closed beats reporting a response that
        // was never received.
        auto status = provizio::contained_set_radar_range_status::error;
        try
        {
            status = dds_request_set_radar_range(
                client, to_contained_set_radar_range(*request, frame_id),
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(max_time_to_set_radar_range).count()),
                &stop_set_radar_range, contained_response);
        }
        catch (const std::exception &exception)
        {
            // Resolving the contained symbol throws when the loaded library doesn't export it - the
            // version-skew case the dlmopen bridge exists to contain. Everything on the far side of the
            // boundary is already exception-proofed; this is the near side, and it is the first such call
            // on a callback path rather than inside activate(). rclcpp doesn't catch exceptions thrown by
            // a service callback, so letting one escape here would std::terminate the node.
            RCLCPP_ERROR(node.get_logger(), "Failed to issue a set_radar_range request: %s", exception.what());
            response->success = false;
            response->actual_range = get_current_radar_range(frame_id, serial_number);
            response->error_message = std::string{"Failed to issue the request: "} + exception.what();
            return;
        }

        if (status == provizio::contained_set_radar_range_status::ok)
        {
            *response = to_ros2_set_radar_range_response(std::move(contained_response));
            if (!response->success && response->error_message.empty())
            {
                // The radar reported a failure without a reason; keep the srv contract's error_message populated
                response->error_message = "The radar could not set the requested range";
            }
        }
        else
        {
            // No response from the radar: report the last known range, and say which of the three reasons
            // it was - a timeout, a deactivation part-way through, or a local failure to issue the request.
            response->success = false;
            response->actual_range = get_current_radar_range(frame_id, serial_number);
            switch (status)
            {
            case provizio::contained_set_radar_range_status::timed_out:
                response->error_message = "Timed out waiting for the radar to respond to the set range request";
                break;

            case provizio::contained_set_radar_range_status::interrupted:
                response->error_message =
                    "Interrupted waiting for the radar to respond to the set range request (the node is "
                    "being deactivated)";
                break;

            default:
                response->error_message = "Failed to issue the set range request";
                break;
            }
        }

        if (!response->success)
        {
            RCLCPP_WARN(node.get_logger(), "Failed to change the radar range of %s to %d. The radar range stays %d.",
                        frame_id.c_str(), static_cast<int>(request->target_range),
                        static_cast<int>(response->actual_range));
        }
    }

    template <typename node_t>
    bool radar_api_ros2_wrapper_dds<node_t>::filter_by_frame_id(const std::string &message_frame_id)
    {
        return this->frame_id.empty() || this->frame_id == message_frame_id;
    }
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_DDS
