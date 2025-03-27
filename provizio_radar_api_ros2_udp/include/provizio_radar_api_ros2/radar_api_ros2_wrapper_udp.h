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
#include <string>
#include <thread>

#include "provizio/radar_api/core.h"

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
            node.declare_parameter(udp_port_param, static_cast<int>(PROVIZIO__RADAR_API_DEFAULT_PORT));
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
        void receive_loop();

        // Variables
        node_t &node;
        rclcpp::Executor &executor;
        float snr_threshold{default_snr_threshold};
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> ros2_radar_pc_publisher;
        std::shared_ptr<rclcpp::Publisher<provizio_radar_api_ros2::msg::RadarInfo>> ros2_radar_info_publisher;
        std::vector<provizio_radar_point_cloud_api_context> contexts;
        provizio_radar_api_connection radar_api_connection;
        std::atomic<bool> stop_receiving{false};
        std::unique_ptr<std::thread> receive_thread;
    };

    constexpr bool is_host_big_endian()
    {
        union {
            uint32_t as_int;
            char as_chars[4];
        } testint = {0x01020304};

        return testint.as_chars[0] == 1;
    }

    void make_sure_sockets_initialized();
    builtin_interfaces::msg::Time ns_to_ros2_time(std::uint64_t timestamp);
    std::string radar_position_id_to_frame_id(std::uint16_t position_id);
    std::int8_t udp_api_radar_range_to_ros2_range(std::uint16_t udp_api_radar_range);

    template <typename node_t> bool radar_api_ros2_wrapper_udp<node_t>::activate()
    {
        if (receive_thread != nullptr)
        {
            // Already active
            return false;
        }

        make_sure_sockets_initialized();
        snr_threshold = static_cast<float>(node.get_parameter(snr_threshold_param).as_double());

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

        // Initialize the Provizio Radar API contexts (needed in any case)
        contexts.resize(static_cast<std::size_t>(node.get_parameter(max_radars_param).as_int()));
        provizio_radar_point_cloud_api_contexts_init(&radar_api_ros2_wrapper_udp<node_t>::on_radar_point_cloud, this,
                                                     contexts.data(), contexts.size());

        // Open a live Provizio radars connection
        auto status = provizio_open_radars_connection(
            static_cast<uint16_t>(node.get_parameter(udp_port_param).as_int()), receive_timeout_ns, 0, contexts.data(),
            contexts.size(), &radar_api_connection);
        if (status != 0)
        {
            RCLCPP_ERROR(node.get_logger(), "provizio_open_radars_connection failed. Error code: %d",
                         static_cast<int>(status));
            contexts.clear();
            ros2_radar_pc_publisher.reset();
            ros2_radar_info_publisher.reset();

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

        // Delete the ROS2 publishers
        ros2_radar_pc_publisher.reset();
        ros2_radar_info_publisher.reset();

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

        std_msgs::msg::Header header;
        header.stamp = ns_to_ros2_time(point_cloud->timestamp);
        header.frame_id = radar_position_id_to_frame_id(point_cloud->radar_position_id);

        auto ros2_radar_pc_publisher =
            self.ros2_radar_pc_publisher; // So we're sure it won't be reset in another thread
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
            ros_point_cloud.is_bigendian = is_host_big_endian();
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
                // Apply SnR filter
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

        auto ros2_radar_info_publisher =
            self.ros2_radar_info_publisher; // So we're sure it won't be reset in another thread
        if (ros2_radar_info_publisher != nullptr)
        {
            provizio_radar_api_ros2::msg::RadarInfo radar_info;
            radar_info.header = header;
            radar_info.current_range = udp_api_radar_range_to_ros2_range(point_cloud->radar_range);
            radar_info.current_multiplexing_mode =
                -1; // TODO(iivanov): Use actual multiplexing mode when it's available
            ros2_radar_info_publisher->publish(std::move(radar_info));
        }
    }
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_UDP
