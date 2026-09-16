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

#include "provizio_radar_api_ros2/radar_api_ros2_wrapper_udp.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace provizio
{
    namespace
    {
        // NOLINTBEGIN: These global constants don't throw on construction
        const std::string frame_id_prefix = "provizio_radar_";
        const std::string frame_id_front_center = frame_id_prefix + "front_center";
        const std::string frame_id_front_left = frame_id_prefix + "front_left";
        const std::string frame_id_front_right = frame_id_prefix + "front_right";
        const std::string frame_id_rear_left = frame_id_prefix + "rear_left";
        const std::string frame_id_rear_right = frame_id_prefix + "rear_right";
        const std::string frame_id_rear_center = frame_id_prefix + "rear_center";
        // NOLINTEND

        constexpr std::uint32_t entity_orientation_num_components = 4; // x, y, z, w
        constexpr std::uint32_t entity_size_num_components = 3;        // x, y, z

        // ROS 2 PointCloud2 layout of a single radar entity. The exact byte layout is an internal detail
        // (consumers resolve fields via PointCloud2::fields); it carries the same content as the DDS radar
        // entities, with orientation as (x, y, z, w). The layout is naturally aligned - the 4-byte fields sit
        // at 4-byte offsets and the whole point is a multiple of 4 bytes - so consumers that read the floats
        // via aligned loads (e.g. PointCloud2Iterator) stay well-defined on strict-alignment CPUs (ARM). The
        // single-byte fields are grouped at the end for this reason.
        struct ros2_radar_entity
        {
            std::uint32_t entity_id;
            float x;
            float y;
            float z;
            float radar_relative_radial_velocity;
            float ground_relative_radial_velocity;
            std::array<float, entity_orientation_num_components> orientation;
            std::array<float, entity_size_num_components> size;
            std::uint8_t entity_class;
            std::uint8_t entity_confidence;
            std::uint8_t entity_class_confidence;
        };
        static_assert(sizeof(ros2_radar_entity) % sizeof(float) == 0,
                      "ros2_radar_entity point_step must be a multiple of 4 bytes for aligned float access");
    } // namespace

    void make_sure_sockets_initialized()
    {
        static struct sockets_initialization_guard
        {
            sockets_initialization_guard()
            {
                provizio_sockets_initialize();
            }

            ~sockets_initialization_guard()
            {
                provizio_sockets_deinitialize();
            }
        } guard;
        (void)guard;
    }

    builtin_interfaces::msg::Time ns_to_ros2_time(const std::uint64_t timestamp)
    {
        builtin_interfaces::msg::Time result;
        result.sec = static_cast<std::int32_t>(timestamp / ns_in_s);
        result.nanosec = static_cast<std::int32_t>(timestamp % ns_in_s);
        return result;
    }

    std::string radar_position_id_to_frame_id(const provizio_radar_position position_id)
    {
        switch (position_id)
        {
        case provizio_radar_position_unknown:
            return {};

        case provizio_radar_position_front_center:
            return frame_id_front_center;

        case provizio_radar_position_front_left:
            return frame_id_front_left;

        case provizio_radar_position_front_right:
            return frame_id_front_right;

        case provizio_radar_position_rear_left:
            return frame_id_rear_left;

        case provizio_radar_position_rear_right:
            return frame_id_rear_right;

        case provizio_radar_position_rear_center:
            return frame_id_rear_center;

        default:
            // position number instead
            return frame_id_prefix + std::to_string(position_id);
        }
    }

    provizio_radar_position radar_frame_id_to_position_id(const std::string &frame_id)
    {
        if (frame_id == frame_id_front_center)
        {
            return provizio_radar_position_front_center;
        }

        if (frame_id == frame_id_front_left)
        {
            return provizio_radar_position_front_left;
        }

        if (frame_id == frame_id_front_right)
        {
            return provizio_radar_position_front_right;
        }

        if (frame_id == frame_id_rear_left)
        {
            return provizio_radar_position_rear_left;
        }

        if (frame_id == frame_id_rear_right)
        {
            return provizio_radar_position_rear_right;
        }

        if (frame_id == frame_id_rear_center)
        {
            return provizio_radar_position_rear_center;
        }

        try
        {
            return static_cast<provizio_radar_position>(std::stoul(frame_id.substr(frame_id_prefix.length())));
        }
        catch (const std::invalid_argument &)
        {
            return provizio_radar_position_unknown;
        }
    }

    std::int8_t udp_api_radar_range_to_ros2_range(const provizio_radar_range udp_api_radar_range)
    {
        switch (udp_api_radar_range)
        {
        case provizio_radar_range_short:
            return provizio_radar_api_ros2::msg::RadarInfo::SHORT_RANGE;

        case provizio_radar_range_medium:
            return provizio_radar_api_ros2::msg::RadarInfo::MEDIUM_RANGE;

        case provizio_radar_range_long:
            return provizio_radar_api_ros2::msg::RadarInfo::LONG_RANGE;

        case provizio_radar_range_ultra_long:
            return provizio_radar_api_ros2::msg::RadarInfo::ULTRA_LONG_RANGE;

        case provizio_radar_range_hyper_long:
            return provizio_radar_api_ros2::msg::RadarInfo::HYPER_LONG_RANGE;

        default:
            return provizio_radar_api_ros2::msg::RadarInfo::UNKNOWN_RANGE;
        }
    }

    provizio_radar_range ros2_range_to_udp_api_radar_range(std::int8_t ros2_radar_range)
    {
        switch (ros2_radar_range)
        {
        case provizio_radar_api_ros2::msg::RadarInfo::SHORT_RANGE:
            return provizio_radar_range_short;

        case provizio_radar_api_ros2::msg::RadarInfo::MEDIUM_RANGE:
            return provizio_radar_range_medium;

        case provizio_radar_api_ros2::msg::RadarInfo::LONG_RANGE:
            return provizio_radar_range_long;

        case provizio_radar_api_ros2::msg::RadarInfo::ULTRA_LONG_RANGE:
            return provizio_radar_range_ultra_long;

        case provizio_radar_api_ros2::msg::RadarInfo::HYPER_LONG_RANGE:
            return provizio_radar_range_hyper_long;

        default:
            return provizio_radar_range_unknown;
        }
    }

    sensor_msgs::msg::PointCloud2 to_ros2_radar_entities(const std_msgs::msg::Header &header,
                                                         const provizio_radar_entities_frame &entities_frame)
    {
        constexpr std::uint8_t float_type = sensor_msgs::msg::PointField::FLOAT32;
        constexpr std::uint8_t uint32_type = sensor_msgs::msg::PointField::UINT32;
        constexpr std::uint8_t uint8_type = sensor_msgs::msg::PointField::UINT8;

        sensor_msgs::msg::PointCloud2 result;
        result.header = header;
        result.height = 1;
        result.is_bigendian = is_host_big_endian;
        result.point_step = sizeof(ros2_radar_entity);

        const auto add_field = [&result](const std::string &name, const std::size_t offset,
                                         const std::uint8_t datatype, const std::uint32_t count) {
            sensor_msgs::msg::PointField field;
            field.name = name;
            field.offset = static_cast<std::uint32_t>(offset);
            field.datatype = datatype;
            field.count = count;
            result.fields.push_back(field);
        };
        add_field(field_entity_id_name, offsetof(ros2_radar_entity, entity_id), uint32_type, 1);
        add_field(field_entity_class_name, offsetof(ros2_radar_entity, entity_class), uint8_type, 1);
        add_field(field_x_name, offsetof(ros2_radar_entity, x), float_type, 1);
        add_field(field_y_name, offsetof(ros2_radar_entity, y), float_type, 1);
        add_field(field_z_name, offsetof(ros2_radar_entity, z), float_type, 1);
        add_field(field_radar_relative_radial_velocity_name,
                  offsetof(ros2_radar_entity, radar_relative_radial_velocity), float_type, 1);
        add_field(field_ground_relative_radial_velocity_name,
                  offsetof(ros2_radar_entity, ground_relative_radial_velocity), float_type, 1);
        add_field(field_orientation_name, offsetof(ros2_radar_entity, orientation), float_type,
                  entity_orientation_num_components);
        add_field(field_size_name, offsetof(ros2_radar_entity, size), float_type, entity_size_num_components);
        add_field(field_entity_confidence_name, offsetof(ros2_radar_entity, entity_confidence), uint8_type, 1);
        add_field(field_entity_class_confidence_name, offsetof(ros2_radar_entity, entity_class_confidence),
                  uint8_type, 1);

        // Defence-in-depth: the core parser already bounds num_entities_received to radar_entities[]'s size,
        // but never index a network-sourced count without clamping it.
        const std::uint16_t num_entities =
            std::min(entities_frame.num_entities_received,
                     static_cast<std::uint16_t>(PROVIZIO__MAX_RADAR_ENTITIES_PER_FRAME));
        result.width = num_entities;
        result.data.resize(static_cast<std::size_t>(result.point_step) * num_entities);
        for (std::uint16_t i = 0; i < num_entities; ++i)
        {
            const provizio_radar_entity &source = entities_frame.radar_entities[i];
            ros2_radar_entity entity{};
            entity.entity_id = source.entity_id;
            entity.entity_class = source.entity_class;
            entity.x = source.x_meters;
            entity.y = source.y_meters;
            entity.z = source.z_meters;
            entity.radar_relative_radial_velocity = source.radar_relative_radial_velocity_m_s;
            entity.ground_relative_radial_velocity = source.ground_relative_radial_velocity_m_s;
            // The UDP quaternion is stored (w, x, y, z); emit (x, y, z, w) to match the DDS entities.
            entity.orientation = {source.orientation.x, source.orientation.y, source.orientation.z,
                                  source.orientation.w};
            entity.size = {source.size.x_meters, source.size.y_meters, source.size.z_meters};
            entity.entity_confidence = source.entity_confidence;
            entity.entity_class_confidence = source.entity_class_confidence;
            std::memcpy(result.data.data() + static_cast<std::size_t>(i) * result.point_step, &entity,
                        sizeof(entity));
        }
        result.row_step = static_cast<decltype(result.row_step)>(result.data.size());
        result.is_dense = true;

        return result;
    }
} // namespace provizio
