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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class provizio_radar_api_ros2_lifecycle_node : public rclcpp_lifecycle::LifecycleNode
{
  public:
    provizio_radar_api_ros2_lifecycle_node() : LifecycleNode("provizio_radar_api_ros2_lifecycle_node")
    {
        publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/provizio_radar_point_cloud",
                                                                          rclcpp::SystemDefaultsQoS{});
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node configuring...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node activating...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node deactivating...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node cleanup...");
        publisher.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node shutdown...");
        publisher.reset();
        return CallbackReturn::SUCCESS;
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<provizio_radar_api_ros2_lifecycle_node>();

    RCLCPP_INFO(node->get_logger(), "provizio_radar_api_ros2_lifecycle_node running...");

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
