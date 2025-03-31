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

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
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
            node.declare_parameter(publish_radar_pc_sr_param, function_enabled_by_default);
            node.declare_parameter(publish_entities_radar_param, function_enabled_by_default);
            node.declare_parameter(publish_entities_camera_param, function_enabled_by_default);
            node.declare_parameter(publish_entities_fusion_param, function_enabled_by_default);
            node.declare_parameter(publish_radar_odometry_param, function_enabled_by_default);
            node.declare_parameter(publish_camera_param, function_enabled_by_default);
            node.declare_parameter(publish_radar_freespace_param, function_enabled_by_default);
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
            node.declare_parameter(publish_radar_freespace_instance_param, function_enabled_by_default);
#endif
            node.declare_parameter(radar_pc_sr_ros2_topic_name_param, default_radar_pc_sr_ros2_topic_name);
            node.declare_parameter(entities_radar_ros2_topic_name_param, default_entities_radar_ros2_topic_name);
            node.declare_parameter(entities_camera_ros2_topic_name_param, default_entities_camera_ros2_topic_name);
            node.declare_parameter(entities_fusion_ros2_topic_name_param, default_entities_fusion_ros2_topic_name);
            node.declare_parameter(radar_odometry_ros2_topic_name_param, default_radar_odometry_ros2_topic_name);
            node.declare_parameter(camera_ros2_topic_name_param, default_camera_ros2_topic_name);
            node.declare_parameter(radar_freespace_ros2_topic_name_param, default_radar_freespace_ros2_topic_name);
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
            node.declare_parameter(radar_freespace_ros2_instance_topic_name_param,
                                   default_radar_freespace_ros2_instance_topic_name);
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
        static void on_radar_info(void *context, contained_radar_info message);
        void on_set_radar_range_request(
            const std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Request> request,
            std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Response> response);

        // Variables
        node_t &node;
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
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PolygonInstanceStamped>>
            ros2_radar_freespace_instance_publisher;
#endif
        std::shared_ptr<rclcpp::Publisher<provizio_radar_api_ros2::msg::RadarInfo>> ros2_radar_info_publisher;
        std::shared_ptr<void> dds_set_radar_range_publisher;

        std::shared_ptr<void> dds_radar_pc_subscriber;
        std::shared_ptr<void> dds_radar_pc_sr_subscriber;
        std::shared_ptr<void> dds_entities_radar_subscriber;
        std::shared_ptr<void> dds_entities_camera_subscriber;
        std::shared_ptr<void> dds_entities_fusion_subscriber;
        std::shared_ptr<void> dds_radar_odometry_subscriber;
        std::shared_ptr<void> dds_camera_subscriber;
        std::shared_ptr<void> dds_radar_freespace_subscriber;
        std::shared_ptr<void> dds_radar_info_subscriber;

        std::shared_ptr<rclcpp::Service<provizio_radar_api_ros2::srv::SetRadarRange>> ros2_set_radar_range_service;
        bool stop_ros2_set_radar_range_service{false};

        std::condition_variable current_radar_ranges_cv;
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

        snr_threshold = static_cast<float>(node.get_parameter(snr_threshold_param).as_double());

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
            dds_set_radar_range_publisher = make_dds_publisher_set_radar_range(
                dds_domain_participant, node.get_parameter(set_radar_range_ros2_service_name_param).as_string());
            stop_ros2_set_radar_range_service = false;
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

        // Destroy the services
        {
            std::lock_guard<std::mutex> lock{current_radar_ranges_mutex};
            stop_ros2_set_radar_range_service = true;
        }
        current_radar_ranges_cv.notify_all(); // To stop waiting for radar range setting, if waiting
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
#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
        ros2_radar_freespace_instance_publisher.reset();
#endif
        ros2_radar_info_publisher.reset();
        dds_set_radar_range_publisher.reset();

        // dds_domain_participant
        dds_domain_participant.reset();

        return true;
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_point_cloud(void *context, contained_pointcloud2 message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
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
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_entities_radar_publisher; // To make sure it can't be destroyed by another
                                                              // thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_entities_camera(void *context, contained_pointcloud2 message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_entities_camera_publisher; // To make sure it can't be destroyed by another
                                                               // thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_entities_fusion(void *context, contained_pointcloud2 message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_entities_fusion_publisher; // To make sure it can't be destroyed by another
                                                               // thread during this call
        if (publisher != nullptr)
        {
            publisher->publish(to_ros2_pointcloud2(std::move(message)));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_odometry(void *context, contained_odometry message)
    {
        auto publisher = static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context)
                             ->ros2_radar_odometry_publisher; // To make sure it can't be destroyed by another
                                                              // thread during this call
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
                             ->ros2_radar_freespace_publisher; // To make sure it can't be destroyed by another
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
    void radar_api_ros2_wrapper_dds<node_t>::on_radar_info(void *context, contained_radar_info message)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_dds<node_t> *>(context);
        {
            std::lock_guard<std::mutex> lock{self.current_radar_ranges_mutex};
            self.current_radar_ranges_by_frame_id[message.header.frame_id] = message.current_range;
            if (!message.serial_number.empty())
            {
                self.current_radar_ranges_by_serial_number[message.serial_number] = message.current_range;
            }
        }
        self.current_radar_ranges_cv.notify_all();

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
        std::unique_lock<std::mutex> lock{current_radar_ranges_mutex};

        auto get_current_radar_range = [this](const std::string &frame_id, const std::string &serial_number) {
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
                return provizio_radar_api_ros2::msg::RadarInfo::UNKNOWN_RANGE;
            }

            return it->second;
        };

        const auto &frame_id = request->header.frame_id;
        const auto &serial_number = request->serial_number;

        if (get_current_radar_range(frame_id, serial_number) == request->target_range)
        {
            // Already set
            response->actual_range = request->target_range;
            return;
        }

        if (dds_publish_set_radar_range(dds_set_radar_range_publisher, to_contained_set_radar_range(*request)))
        {
            current_radar_ranges_cv.wait_for(lock, max_time_to_set_radar_range, [&]() {
                return stop_ros2_set_radar_range_service ||
                       get_current_radar_range(frame_id, serial_number) == request->target_range;
            });
        }
        else
        {
            RCLCPP_ERROR(node.get_logger(), "Failed to publish set_radar_range message!");
        }

        response->actual_range = get_current_radar_range(frame_id, serial_number);
    }
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_DDS
