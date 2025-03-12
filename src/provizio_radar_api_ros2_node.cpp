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

#include "provizio_radar_api_ros2/radar_api_ros2_wrapper.h"

namespace provizio
{
    class provizio_radar_api_ros2_node : public rclcpp::Node
    {
      public:
        provizio_radar_api_ros2_node()
            : Node("provizio_radar_api_ros2_node"),
              api_wrapper(std::make_unique<radar_api_ros2_wrapper<rclcpp::Node>>(*this))
        {
            if (!api_wrapper->activate())
            {
                throw std::runtime_error{"Failed to activate the API wrapper!"};
            }
        }

      private:
        std::unique_ptr<radar_api_ros2_wrapper<rclcpp::Node>> api_wrapper;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> publisher;
    };
} // namespace provizio

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<provizio::provizio_radar_api_ros2_node> node;
    int error_code = 0;
    try
    {
        node = std::make_shared<provizio::provizio_radar_api_ros2_node>();

        RCLCPP_INFO(node->get_logger(), "provizio_radar_api_ros2_node started");

        rclcpp::spin(node);

        RCLCPP_INFO(node->get_logger(), "provizio_radar_api_ros2_node finished");
        node.reset();
    }
    catch (const std::exception &exception)
    {
        if (node)
        {
            RCLCPP_ERROR(node->get_logger(), "provizio_radar_api_ros2_node threw an exception: %s", exception.what());
            node.reset();
        }
        else
        {
            std::cerr << "provizio_radar_api_ros2_node threw an exception: " << exception.what() << std::endl;
        }

        error_code = 1;
    }

    rclcpp::shutdown();
    return error_code;
}
