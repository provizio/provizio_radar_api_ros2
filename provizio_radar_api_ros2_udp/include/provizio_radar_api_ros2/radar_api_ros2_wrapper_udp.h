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

#ifndef PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_UDP
#define PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_UDP

#include <atomic>
#include <cstring>
#include <functional>
#include <string>
#include <thread>

#include "provizio/radar_api/core.h"
#include "provizio/radar_api/entities.h"

#include <provizio_radar_api_ros2/msg/radar_info.hpp>
#include <provizio_radar_api_ros2/srv/set_radar_range.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "provizio_radar_api_ros2/constants.h"
#include "provizio_radar_api_ros2/constants_udp.h"
#include "provizio_radar_api_ros2/parameters.h"

namespace provizio
{
    template <typename node_t> class radar_api_ros2_wrapper_udp
    {
      public:
        radar_api_ros2_wrapper_udp(node_t &node, rclcpp::Executor &executor) : node(node), executor(executor)
        {
            std::memset(&radar_api_connection, 0, sizeof(radar_api_connection));

            declare_common_parameters(node);
            node.declare_parameter(max_radars_param, static_cast<int>(default_max_radars));
            node.declare_parameter(pc_udp_port_param, static_cast<int>(PROVIZIO__RADAR_API_DEFAULT_PORT));
            node.declare_parameter(set_range_udp_port_param,
                                   static_cast<int>(PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_PORT));
            node.declare_parameter(set_range_ip_address_param, "");
            node.declare_parameter(publish_entities_radar_param, feature_enabled_by_default);
            node.declare_parameter(entities_radar_ros2_topic_name_param, default_entities_radar_ros2_topic_name);
        }

        ~radar_api_ros2_wrapper_udp()
        {
            if (receive_thread != nullptr)
            {
                deactivate();
            }
        }

        bool activate();
        bool deactivate();

      private:
        // Functions
        static void on_radar_point_cloud(const provizio_radar_point_cloud *point_cloud,
                                         struct provizio_radar_point_cloud_api_context *context);
        static void on_radar_entities(const provizio_radar_entities_frame *entities_frame,
                                      struct provizio_radar_entities_api_context *context);
        void receive_loop();
        void on_set_radar_range_request(
            const std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Request> request,
            std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Response> response);
        bool filter_by_frame_id(const std::string &message_frame_id);

        // Variables
        node_t &node;
        rclcpp::Executor &executor;
        std::string frame_id{default_frame_id};
        float snr_threshold{default_snr_threshold};
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_radar_pc_publisher;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_entities_radar_publisher;
        std::shared_ptr<rclcpp::Publisher<provizio_radar_api_ros2::msg::RadarInfo>> ros2_radar_info_publisher;
        std::shared_ptr<rclcpp::Service<provizio_radar_api_ros2::srv::SetRadarRange>> ros2_set_radar_range_service;
        std::vector<provizio_radar_point_cloud_api_context> contexts;
        std::vector<provizio_radar_entities_api_context> entities_contexts;
        provizio_radar_api_connection radar_api_connection;
        std::atomic<bool> stop_receiving{false};
        std::unique_ptr<std::thread> receive_thread;
        std::mutex current_radar_ranges_mutex;
        std::unordered_map<std::uint16_t, std::int8_t> current_radar_ranges_by_frame_id;
    };

    void make_sure_sockets_initialized();
    builtin_interfaces::msg::Time ns_to_ros2_time(std::uint64_t timestamp);
    std::string radar_position_id_to_frame_id(provizio_radar_position position_id);
    provizio_radar_position radar_frame_id_to_position_id(const std::string &frame_id);
    std::int8_t udp_api_radar_range_to_ros2_range(provizio_radar_range udp_api_radar_range);
    provizio_radar_range ros2_range_to_udp_api_radar_range(std::int8_t ros2_radar_range);
    sensor_msgs::msg::PointCloud2 to_ros2_radar_entities(const std_msgs::msg::Header &header,
                                                         const provizio_radar_entities_frame &entities_frame);

    template <typename node_t> bool radar_api_ros2_wrapper_udp<node_t>::activate()
    {
        if (receive_thread != nullptr)
        {
            // Already active
            return false;
        }

        make_sure_sockets_initialized();
        frame_id = node.get_parameter(frame_id_param).as_string();
        snr_threshold = static_cast<float>(node.get_parameter(snr_threshold_param).as_double());
        RCLCPP_DEBUG(node.get_logger(), "Running with frame_id %s, snr_threshold=%f",
                     (frame_id.empty() ? "unrestricted" : frame_id.c_str()), snr_threshold);

        // Create ROS2 publishers
        if (node.get_parameter(publish_radar_pc_param).as_bool())
        {
            ros2_radar_pc_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
                node.get_parameter(radar_pc_ros2_topic_name_param).as_string(), default_ros2_qos);
        }

        if (node.get_parameter(publish_radar_info_param).as_bool())
        {
            ros2_radar_info_publisher = node.template create_publisher<provizio_radar_api_ros2::msg::RadarInfo>(
                node.get_parameter(radar_info_ros2_topic_name_param).as_string(), default_ros2_qos);
        }

        if (node.get_parameter(publish_entities_radar_param).as_bool())
        {
            ros2_entities_radar_publisher = node.template create_publisher<sensor_msgs::msg::PointCloud2>(
                node.get_parameter(entities_radar_ros2_topic_name_param).as_string(), default_ros2_qos);
        }

        // Create services
        if (node.get_parameter(serve_set_radar_range_param).as_bool())
        {
            ros2_set_radar_range_service = node.template create_service<provizio_radar_api_ros2::srv::SetRadarRange>(
                node.get_parameter(set_radar_range_ros2_service_name_param).as_string(),
                std::bind(&radar_api_ros2_wrapper_udp::on_set_radar_range_request, this, std::placeholders::_1,
                          std::placeholders::_2));
        }

        // Initialize the Provizio Radar API contexts (needed in any case). max_radars is a ROS 2 parameter,
        // i.e. settable over the DDS graph, and each context is large (an entities context embeds
        // PROVIZIO__MAX_RADAR_ENTITIES_PER_FRAME entities), so validate it rather than resizing to whatever
        // arrives: as_int() is signed, and a negative value would wrap into a nonsensical allocation.
        const auto max_radars_param_value = node.get_parameter(max_radars_param).as_int();
        if (max_radars_param_value < 1 || static_cast<std::uint64_t>(max_radars_param_value) > max_supported_radars)
        {
            RCLCPP_ERROR(node.get_logger(), "%s must be in [1; %ld], got %ld", max_radars_param.c_str(),
                         static_cast<std::int64_t>(max_supported_radars),
                         static_cast<std::int64_t>(max_radars_param_value));
            return false;
        }
        const auto max_radars = static_cast<std::size_t>(max_radars_param_value);
        contexts.resize(max_radars);
        provizio_radar_point_cloud_api_contexts_init(&radar_api_ros2_wrapper_udp<node_t>::on_radar_point_cloud, this,
                                                     contexts.data(), contexts.size());

        // Initialize the entities contexts only when radar entities are to be re-published
        if (ros2_entities_radar_publisher != nullptr)
        {
            entities_contexts.resize(max_radars);
            provizio_radar_entities_api_contexts_init(&radar_api_ros2_wrapper_udp<node_t>::on_radar_entities, this,
                                                      entities_contexts.data(), entities_contexts.size());
        }

        // Open a live Provizio radars connection
        auto status = provizio_open_radars_connection(
            static_cast<uint16_t>(node.get_parameter(pc_udp_port_param).as_int()), receive_timeout_ns, 0,
            contexts.data(), contexts.size(),
            entities_contexts.empty() ? nullptr : entities_contexts.data(), entities_contexts.size(),
            &radar_api_connection);
        if (status != 0)
        {
            RCLCPP_ERROR(node.get_logger(), "provizio_open_radars_connection failed. Error code: %d",
                         static_cast<int>(status));
            contexts.clear();
            entities_contexts.clear();
            ros2_radar_pc_publisher.reset();
            ros2_radar_info_publisher.reset();
            ros2_entities_radar_publisher.reset();

            return false;
        }

        // Start receiving and handling packets
        stop_receiving = false;
        receive_thread = std::make_unique<std::thread>(&radar_api_ros2_wrapper_udp::receive_loop, this);

        return true;
    }

    template <typename node_t> bool radar_api_ros2_wrapper_udp<node_t>::deactivate()
    {
        if (receive_thread == nullptr)
        {
            // Already deactivated
            return false;
        }

        // Stop receiving and handling packets
        stop_receiving = true;
        receive_thread->join();
        receive_thread.reset();

        // Close the connection
        provizio_close_radar_connection(&radar_api_connection);

        // Clear the API contexts
        contexts.clear();
        entities_contexts.clear();

        // Delete the ROS2 publishers and services. Note this happens *after* the receive thread has been
        // joined above: the publisher members are read only by the API callbacks, which run on that
        // thread, so resetting them here cannot race with a callback reading them. Keep that ordering -
        // resetting a publisher while the receive thread is still running would be a data race on the
        // shared_ptr itself, which taking a local copy inside the callback does not prevent.
        ros2_set_radar_range_service.reset();
        ros2_radar_pc_publisher.reset();
        ros2_radar_info_publisher.reset();
        ros2_entities_radar_publisher.reset();

        return true;
    }

    template <typename node_t> void radar_api_ros2_wrapper_udp<node_t>::receive_loop()
    {
        while (!stop_receiving)
        {
            const auto status = provizio_radar_api_receive_packet(&radar_api_connection);
            if (status != 0 && status != PROVIZIO_E_TIMEOUT && status != PROVIZIO_E_SKIPPED)
            {
                if (status == EINTR)
                {
                    RCLCPP_INFO(node.get_logger(),
                                "provizio_radar_api_receive_packet has been interrupted by user. Done receiving.");
                    executor.cancel();
                    return;
                }

                RCLCPP_ERROR(node.get_logger(), "provizio_radar_api_receive_packet failed. Error code: %d", status);
            }
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_udp<node_t>::on_radar_point_cloud(
        const provizio_radar_point_cloud *point_cloud, struct provizio_radar_point_cloud_api_context *context)
    {
        constexpr std::uint8_t float_field_type = sensor_msgs::msg::PointField::FLOAT32;
        constexpr std::size_t num_fields_per_point = 6;
        constexpr std::size_t field_x = 0;
        constexpr std::size_t field_y = 1;
        constexpr std::size_t field_z = 2;
        constexpr std::size_t field_radar_relative_radial_velocity = 3;
        constexpr std::size_t field_signal_to_noise_ratio = 4;
        constexpr std::size_t field_ground_relative_radial_velocity =
            5; // May be valid or NaN, depending on radar configuration

        auto &self = *static_cast<radar_api_ros2_wrapper_udp<node_t> *>(context->user_data);

        const auto frame_id =
            radar_position_id_to_frame_id(static_cast<provizio_radar_position>(point_cloud->radar_position_id));

        if (!self.filter_by_frame_id(frame_id))
        {
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = ns_to_ros2_time(point_cloud->timestamp);
        header.frame_id = frame_id;

        auto ros2_radar_pc_publisher = self.ros2_radar_pc_publisher;  // Kept alive for the duration of this callback
        if (ros2_radar_pc_publisher != nullptr)
        {
            // Convert provizio_radar_point_cloud to sensor_msgs::PointCloud2
            sensor_msgs::msg::PointCloud2 ros_point_cloud;
            ros_point_cloud.header = header;
            ros_point_cloud.height = 1;
            ros_point_cloud.fields.resize(num_fields_per_point);
            for (auto &it : ros_point_cloud.fields)
            {
                it.count = 1;
                it.datatype = float_field_type;
            }
            ros_point_cloud.fields[field_x].name = field_x_name;
            ros_point_cloud.fields[field_x].offset = offsetof(provizio_radar_point, x_meters);
            ros_point_cloud.fields[field_y].name = field_y_name;
            ros_point_cloud.fields[field_y].offset = offsetof(provizio_radar_point, y_meters);
            ros_point_cloud.fields[field_z].name = field_z_name;
            ros_point_cloud.fields[field_z].offset = offsetof(provizio_radar_point, z_meters);
            ros_point_cloud.fields[field_radar_relative_radial_velocity].name =
                field_radar_relative_radial_velocity_name;
            ros_point_cloud.fields[field_radar_relative_radial_velocity].offset =
                offsetof(provizio_radar_point, radar_relative_radial_velocity_m_s);
            ros_point_cloud.fields[field_signal_to_noise_ratio].name = field_signal_to_noise_ratio_name;
            ros_point_cloud.fields[field_signal_to_noise_ratio].offset =
                offsetof(provizio_radar_point, signal_to_noise_ratio);
            ros_point_cloud.fields[field_ground_relative_radial_velocity].name =
                field_ground_relative_radial_velocity_name;
            ros_point_cloud.fields[field_ground_relative_radial_velocity].offset =
                offsetof(provizio_radar_point, ground_relative_radial_velocity_m_s);
            ros_point_cloud.is_bigendian = is_host_big_endian;
            ros_point_cloud.point_step = sizeof(provizio_radar_point);
            if (self.snr_threshold <= 0.0F)
            {
                // Output all points
                ros_point_cloud.width = point_cloud->num_points_received;
                ros_point_cloud.data.resize(ros_point_cloud.point_step * point_cloud->num_points_received);
                std::memcpy(ros_point_cloud.data.data(), point_cloud->radar_points, ros_point_cloud.data.size());
            }
            else
            {
                // Apply SNR filter
                ros_point_cloud.width = 0;
                ros_point_cloud.data.reserve(ros_point_cloud.point_step * point_cloud->num_points_received);
                for (std::size_t i = 0; i < point_cloud->num_points_received; ++i)
                {
                    if (point_cloud->radar_points[i].signal_to_noise_ratio >= self.snr_threshold)
                    {
                        ++ros_point_cloud.width;
                        ros_point_cloud.data.resize(ros_point_cloud.data.size() + ros_point_cloud.point_step);
                        std::memcpy(ros_point_cloud.data.data() + ros_point_cloud.data.size() -
                                        ros_point_cloud.point_step,
                                    &point_cloud->radar_points[i], ros_point_cloud.point_step);
                    }
                }
            }
            ros_point_cloud.row_step = static_cast<decltype(ros_point_cloud.row_step)>(ros_point_cloud.data.size());
            ros_point_cloud.is_dense = true;

            // Good to publish now
            ros2_radar_pc_publisher->publish(std::move(ros_point_cloud));
        }

        // Update the radar range
        const auto ros2_range =
            udp_api_radar_range_to_ros2_range(static_cast<provizio_radar_range>(point_cloud->radar_range));
        {
            std::lock_guard<std::mutex> lock{self.current_radar_ranges_mutex};
            self.current_radar_ranges_by_frame_id[point_cloud->radar_position_id] = ros2_range;
        }

        auto ros2_radar_info_publisher =
            self.ros2_radar_info_publisher;  // Kept alive for the duration of this callback
        if (ros2_radar_info_publisher != nullptr)
        {
            provizio_radar_api_ros2::msg::RadarInfo radar_info;
            radar_info.header = header;
            radar_info.current_range = ros2_range;
            radar_info.current_multiplexing_mode =
                -1; // TODO(iivanov): Use actual multiplexing mode when it's available
            // TODO(iivanov): serial_number and supported_ranges are currently unavailable in the UDP API

            ros2_radar_info_publisher->publish(std::move(radar_info));
        }
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_udp<node_t>::on_radar_entities(
        const provizio_radar_entities_frame *entities_frame, struct provizio_radar_entities_api_context *context)
    {
        auto &self = *static_cast<radar_api_ros2_wrapper_udp<node_t> *>(context->user_data);

        const auto frame_id =
            radar_position_id_to_frame_id(static_cast<provizio_radar_position>(entities_frame->radar_position_id));

        if (!self.filter_by_frame_id(frame_id))
        {
            return;
        }

        auto ros2_entities_radar_publisher =
            self.ros2_entities_radar_publisher;  // Kept alive for the duration of this callback
        if (ros2_entities_radar_publisher == nullptr)
        {
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = ns_to_ros2_time(entities_frame->timestamp);
        header.frame_id = frame_id;

        ros2_entities_radar_publisher->publish(to_ros2_radar_entities(header, *entities_frame));
    }

    template <typename node_t>
    void radar_api_ros2_wrapper_udp<node_t>::on_set_radar_range_request(
        const std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Request> request,
        std::shared_ptr<provizio_radar_api_ros2::srv::SetRadarRange::Response> response)
    {
        if (!request->serial_number.empty())
        {
            // TODO(iivanov): Change when the UDP API supports setting ranges by serial number
            RCLCPP_ERROR(node.get_logger(), "radar_api_ros2_wrapper_udp doesn't support setting ranges by radar's "
                                            "serial_number. Using header.frame_id instead.");
        }

        const auto get_current_radar_range = [this](const std::uint16_t position_id) {
            std::lock_guard<std::mutex> lock{current_radar_ranges_mutex};
            const auto it = current_radar_ranges_by_frame_id.find(position_id);
            return it != current_radar_ranges_by_frame_id.end()
                       ? it->second
                       : static_cast<std::int8_t>(provizio_radar_api_ros2::msg::RadarInfo::UNKNOWN_RANGE);
        };

        if (!frame_id.empty() && !request->header.frame_id.empty() && request->header.frame_id != frame_id)
        {
            RCLCPP_WARN(node.get_logger(),
                        "The node's frame_id %s doesn't match the request's (%s)! Assuming the node's one correct",
                        frame_id.c_str(), request->header.frame_id.c_str());
        }
        const std::string &the_frame_id = !frame_id.empty() ? frame_id : request->header.frame_id;

        const auto position_id = radar_frame_id_to_position_id(the_frame_id);

        // Quick-set: if the radar already reports the requested range (via received point clouds), skip the
        // round-trip.
        if (get_current_radar_range(position_id) == request->target_range)
        {
            response->actual_range = request->target_range;
            response->success = true;
            return;
        }

        const auto set_range_ip_address = node.get_parameter(set_range_ip_address_param).as_string();

        // The new UDP API blocks until the radar acknowledges the request and reports its resulting range
        // (request -> acknowledgement -> response), or PROVIZIO__RADAR_API_SET_RANGE_DEFAULT_TIMEOUT elapses.
        provizio_radar_range actual_udp_range = provizio_radar_range_unknown;
        const std::int32_t status = provizio_set_radar_range(
            position_id, ros2_range_to_udp_api_radar_range(request->target_range),
            static_cast<std::uint16_t>(node.get_parameter(set_range_udp_port_param).as_int()),
            set_range_ip_address.empty() ? nullptr : set_range_ip_address.c_str(), &actual_udp_range);

        response->success = (status == 0);
        // supported_ranges stays empty: the UDP API doesn't expose it.
        if (response->success)
        {
            response->actual_range = udp_api_radar_range_to_ros2_range(actual_udp_range);
        }
        else
        {
            // Prefer the range the radar reported (if any), otherwise fall back to the last one seen in point
            // clouds.
            const auto reported_range = udp_api_radar_range_to_ros2_range(actual_udp_range);
            response->actual_range = (reported_range != provizio_radar_api_ros2::msg::RadarInfo::UNKNOWN_RANGE)
                                         ? reported_range
                                         : get_current_radar_range(position_id);
            response->error_message = (status == PROVIZIO_E_TIMEOUT)
                                          ? "Timed out waiting for the radar to respond to the set range request"
                                          : "The radar could not set the requested range";
            RCLCPP_WARN(node.get_logger(), "Failed to change the radar range of %s to %d. The radar range stays %d.",
                        the_frame_id.c_str(), static_cast<int>(request->target_range),
                        static_cast<int>(response->actual_range));
        }
    }

    template <typename node_t>
    bool radar_api_ros2_wrapper_udp<node_t>::filter_by_frame_id(const std::string &message_frame_id)
    {
        return this->frame_id.empty() || this->frame_id == message_frame_id;
    }
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_UDP
