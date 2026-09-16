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

#include <algorithm>
#include <iterator>

#include "builtin_interfaces/msg/Time.h"
#include "std_msgs/msg/Header.h"

#include "provizio_dds_contained_types_dds_conversion.h"

namespace provizio
{
    namespace
    {
        // provizio_dds_idls radar_info carries ranges as uint32 (SHORT=0..HYPER_LONG=4, UNKNOWN=65535),
        // while ROS 2 RadarInfo.msg uses int8 (the same 0..4, but UNKNOWN=-1). Map the known values
        // explicitly rather than narrowing with a static_cast: the cast turns 65535 into -1 only by
        // coincidence of two's-complement truncation, and silently turns every *unknown* value into a
        // plausible-looking range (256 -> SHORT_RANGE, 200 -> -56), which the ROS 2 client cannot tell
        // apart from a genuine answer. Anything unrecognised becomes UNKNOWN_RANGE, matching what the
        // UDP wrapper's udp_api_radar_range_to_ros2_range does.
        constexpr std::int8_t contained_unknown_range = -1;  // RadarInfo::UNKNOWN_RANGE
        constexpr std::uint32_t dds_unknown_range = 65535;   // radar_info_Constants::UNKNOWN_RANGE
        constexpr std::uint32_t dds_max_known_range = 4;     // radar_info_Constants::HYPER_LONG_RANGE

        std::int8_t to_contained_radar_range(const std::uint32_t dds_range)
        {
            if (dds_range > dds_max_known_range)
            {
                // Includes dds_unknown_range itself, and any range this build doesn't know about
                static_assert(dds_unknown_range > dds_max_known_range, "UNKNOWN_RANGE must not be a valid range");
                return contained_unknown_range;
            }

            return static_cast<std::int8_t>(dds_range);
        }

        // The reverse of to_contained_radar_range. Needed for the same reason: a bare cast would turn
        // ROS 2's UNKNOWN_RANGE (-1) into 4294967295 rather than the DDS UNKNOWN_RANGE (65535), putting a
        // value on the wire that means nothing to the radar, and would do the same to any other negative
        // or out-of-range value a service client happens to send.
        std::uint32_t to_dds_radar_range(const std::int8_t contained_range)
        {
            if (contained_range < 0 || static_cast<std::uint32_t>(contained_range) > dds_max_known_range)
            {
                return dds_unknown_range;
            }

            return static_cast<std::uint32_t>(contained_range);
        }

        provizio::contained_time to_contained_time(const builtin_interfaces::msg::Time &stamp)
        {
            return {stamp.sec(), stamp.nanosec()};
        }

        builtin_interfaces::msg::Time to_dds_time(const provizio::contained_time &stamp)
        {
            builtin_interfaces::msg::Time result;
            result.sec(stamp.sec);
            result.nanosec(stamp.nanosec);
            return result;
        }

        provizio::contained_header to_contained_header(const std_msgs::msg::Header &header)
        {
            provizio::contained_header result;
            result.frame_id = header.frame_id();
            result.stamp = to_contained_time(header.stamp());
            return result;
        }

        std_msgs::msg::Header to_dds_header(provizio::contained_header header)
        {
            std_msgs::msg::Header result;
            result.frame_id(std::move(header.frame_id));
            result.stamp(to_dds_time(header.stamp));
            return result;
        }

        provizio::contained_point_field to_contained_point_field(const sensor_msgs::msg::PointField &field)
        {
            provizio::contained_point_field result;
            result.name = field.name();
            result.offset = field.offset();
            result.datatype = field.datatype();
            result.count = field.count();
            return result;
        }

        std::vector<provizio::contained_point_field> to_contained_point_fields(
            const std::vector<sensor_msgs::msg::PointField> &fields)
        {
            std::vector<provizio::contained_point_field> result;
            result.reserve(fields.size());
            std::transform(fields.begin(), fields.end(), std::back_inserter(result), to_contained_point_field);
            return result;
        }

        template <typename dds_vector_type,
                  typename contained_vector_type = contained_vector3> // supports both Point and Vector3 types
        contained_vector_type to_contained_vector3(const dds_vector_type &vector)
        {
            return {vector.x(), vector.y(), vector.z()};
        }

        contained_quaternion to_contained_quaternion(const geometry_msgs::msg::Quaternion &orientation)
        {
            return {orientation.x(), orientation.y(), orientation.z(), orientation.w()};
        }

        contained_pose to_contained_pose(const geometry_msgs::msg::Pose &pose)
        {
            return {to_contained_vector3(pose.position()), to_contained_quaternion(pose.orientation())};
        }

        contained_pose_with_covariance to_contained_pose_with_covariance(
            const geometry_msgs::msg::PoseWithCovariance &pose)
        {
            return {to_contained_pose(pose.pose()), pose.covariance()};
        }

        contained_twist to_contained_twist(const geometry_msgs::msg::Twist &twist)
        {
            return {to_contained_vector3(twist.linear()), to_contained_vector3(twist.angular())};
        }

        contained_twist_with_covariance to_contained_twist_with_covariance(
            const geometry_msgs::msg::TwistWithCovariance &twist)
        {
            return {to_contained_twist(twist.twist()), twist.covariance()};
        }
    } // namespace

    provizio::contained_pointcloud2 to_contained_pointcloud2(const sensor_msgs::msg::PointCloud2 &message)
    {
        provizio::contained_pointcloud2 result;
        result.header = to_contained_header(message.header());
        result.width = message.width();
        result.height = message.height();
        result.fields = to_contained_point_fields(message.fields());
        result.is_bigendian = message.is_bigendian();
        result.point_step = message.point_step();
        result.row_step = message.row_step();
        result.data = message.data();
        result.is_dense = message.is_dense();
        return result;
    }

    provizio::contained_odometry to_contained_odometry(const nav_msgs::msg::Odometry &message)
    {
        provizio::contained_odometry result;
        result.header = to_contained_header(message.header());
        result.child_frame_id = message.child_frame_id();
        result.pose = to_contained_pose_with_covariance(message.pose());
        result.twist = to_contained_twist_with_covariance(message.twist());
        return result;
    }

    provizio::contained_image to_contained_image(const sensor_msgs::msg::Image &message)
    {
        provizio::contained_image result;
        result.header = to_contained_header(message.header());
        result.height = message.height();
        result.width = message.width();
        result.encoding = message.encoding();
        result.is_bigendian = message.is_bigendian();
        result.step = message.step();
        result.data = message.data();
        return result;
    }

    provizio::contained_polygon to_contained_polygon(const geometry_msgs::msg::Polygon &polygon)
    {
        provizio::contained_polygon result;
        const auto &points = polygon.points();
        result.points.reserve(points.size());
        std::transform(points.begin(), points.end(), std::back_inserter(result.points),
                       to_contained_vector3<geometry_msgs::msg::Point32, contained_point32>);
        return result;
    }

    provizio::contained_polygon_instance to_contained_polygon_instance(
        const geometry_msgs::msg::PolygonInstance &message)
    {
        provizio::contained_polygon_instance result;
        result.polygon = to_contained_polygon(message.polygon());
        result.id = message.id();
        return result;
    }

    provizio::contained_polygon_instance_stamped to_contained_polygon_instance_stamped(
        const geometry_msgs::msg::PolygonInstanceStamped &message)
    {
        provizio::contained_polygon_instance_stamped result;
        result.header = to_contained_header(message.header());
        result.polygon = to_contained_polygon_instance(message.polygon());
        return result;
    }

    provizio::contained_radar_info to_contained_radar_info(const provizio::msg::radar_info &message)
    {
        provizio::contained_radar_info result;
        result.header = to_contained_header(message.header());
        result.serial_number = message.serial_number();
        result.current_range = to_contained_radar_range(message.current_range());
        const auto &supported_ranges = message.supported_ranges();
        result.supported_ranges.reserve(supported_ranges.size());
        // As of provizio_dds 2.0 / provizio_dds_idls 2.1 radar_info range fields are plain uint32 (were an
        // enum radar_range before); the on-wire bytes are identical.
        std::transform(supported_ranges.begin(), supported_ranges.end(), std::back_inserter(result.supported_ranges),
                       [](const std::uint32_t range) { return to_contained_radar_range(range); });
        result.current_multiplexing_mode = -1; // TODO(iivanov): Use actual multiplexing mode when it's available
        return result;
    }

    provizio::srv::set_radar_range_Request to_dds_set_radar_range_request(const char *const frame_id,
                                                                          const char *const serial_number,
                                                                          const std::int8_t target_range,
                                                                          const std::int32_t header_stamp_sec,
                                                                          const std::uint32_t header_stamp_nanosec)
    {
        // Build all std::string values here, inside the contained library's own C++ runtime, from the raw
        // C strings that crossed the extern "C" boundary. A null pointer is treated as an empty string.
        contained_header header;
        header.frame_id = frame_id != nullptr ? std::string{frame_id} : std::string{};
        header.stamp.sec = header_stamp_sec;
        header.stamp.nanosec = header_stamp_nanosec;

        provizio::srv::set_radar_range_Request result;
        result.header(to_dds_header(std::move(header)));
        result.serial_number(serial_number != nullptr ? std::string{serial_number} : std::string{});
        result.target_range(to_dds_radar_range(target_range));
        return result;
    }

    provizio::contained_set_radar_range_response to_contained_set_radar_range_response(
        const provizio::srv::set_radar_range_Response &message)
    {
        provizio::contained_set_radar_range_response result;
        result.header = to_contained_header(message.header());
        result.success = message.success();
        result.error_message = message.error_message();
        result.serial_number = message.serial_number();
        result.current_range = to_contained_radar_range(message.current_range());
        const auto &supported_ranges = message.supported_ranges();
        result.supported_ranges.reserve(supported_ranges.size());
        std::transform(supported_ranges.begin(), supported_ranges.end(), std::back_inserter(result.supported_ranges),
                       [](const std::uint32_t range) { return to_contained_radar_range(range); });
        return result;
    }
} // namespace provizio
