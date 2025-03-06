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

namespace provizio
{
    template <typename node_t> class radar_api_ros2_wrapper_dds
    {
      public:
        radar_api_ros2_wrapper_dds(node_t &node) : node(node)
        {
        }

        bool activate()
        {
            return true;
        }

        bool deactivate()
        {
            return true;
        }

      private:
        node_t &node;
    };
} // namespace provizio

#endif // PROVIZIO_RADAR_API_ROS2_RADAR_API_ROS2_WRAPPER_DDS
