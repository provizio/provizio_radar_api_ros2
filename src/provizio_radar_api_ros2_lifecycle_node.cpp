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

#include <assert.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "provizio_radar_api_ros2/common.h"
#include "provizio_radar_api_ros2/radar_api_ros2_wrapper.h"

namespace provizio
{
    class provizio_radar_api_ros2_lifecycle_node : public rclcpp_lifecycle::LifecycleNode
    {
      public:
        provizio_radar_api_ros2_lifecycle_node() : LifecycleNode("provizio_radar_api_ros2_lifecycle_node")
        {
            declare_common_parameters(*this);
        }

        CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
        {
            if (api_wrapper)
            {
                RCLCPP_ERROR(get_logger(),
                             "provizio_radar_api_ros2_lifecycle_node can't be configured as it's already configured");
                return CallbackReturn::FAILURE;
            }

            assert(!is_active); // It can't be active if there is no api_wrapper

            api_wrapper = std::make_unique<radar_api_ros2_wrapper<rclcpp_lifecycle::LifecycleNode>>(*this);
            RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node configured");
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
        {
            if (!api_wrapper)
            {
                RCLCPP_ERROR(get_logger(),
                             "provizio_radar_api_ros2_lifecycle_node can't be activated as it's not yet configured");
                return CallbackReturn::FAILURE;
            }

            if (is_active)
            {
                RCLCPP_ERROR(get_logger(),
                             "provizio_radar_api_ros2_lifecycle_node can't be activated as it's already active");
                return CallbackReturn::FAILURE;
            }

            if (api_wrapper->activate())
            {
                is_active = true;
                RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node activated");
                return CallbackReturn::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "provizio_radar_api_ros2_lifecycle_node failed to activate");
                return CallbackReturn::FAILURE;
            }
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
        {
            if (!api_wrapper)
            {
                RCLCPP_ERROR(get_logger(),
                             "provizio_radar_api_ros2_lifecycle_node can't be deactivated as it's not yet configured");
                return CallbackReturn::FAILURE;
            }

            if (!is_active)
            {
                RCLCPP_ERROR(get_logger(),
                             "provizio_radar_api_ros2_lifecycle_node can't be deactivated as it's not active");
                return CallbackReturn::FAILURE;
            }

            if (api_wrapper->deactivate())
            {
                is_active = false;
                RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node deactivated");
                return CallbackReturn::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "provizio_radar_api_ros2_lifecycle_node failed to deactivate");
                return CallbackReturn::FAILURE;
            }
        }

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
        {
            cleanup();

            RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node cleaned up");
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
        {
            cleanup();

            RCLCPP_INFO(get_logger(), "provizio_radar_api_ros2_lifecycle_node shut down");
            return CallbackReturn::SUCCESS;
        }

      private:
        void cleanup()
        {
            api_wrapper.reset();
            is_active = false;
        }

        std::unique_ptr<radar_api_ros2_wrapper<rclcpp_lifecycle::LifecycleNode>> api_wrapper;
        bool is_active = false;
    };
} // namespace provizio

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<provizio::provizio_radar_api_ros2_lifecycle_node>();

    RCLCPP_INFO(node->get_logger(), "provizio_radar_api_ros2_lifecycle_node started");

    rclcpp::spin(node->get_node_base_interface());

    RCLCPP_INFO(node->get_logger(), "provizio_radar_api_ros2_lifecycle_node finished");

    rclcpp::shutdown();

    return 0;
}
