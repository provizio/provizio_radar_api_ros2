#include <algorithm>
#include <iterator>

#include "provizio_radar_api_ros2/constants.h"
#include "provizio_radar_api_ros2/provizio_dds_contained_types_ros2_conversion.h"

namespace provizio
{
    namespace
    {
        const std::string snr_field_name = "signal_to_noise_ratio"; // NOLINT: Never throws

        builtin_interfaces::msg::Time to_ros2_time(const provizio::contained_time &stamp)
        {
            builtin_interfaces::msg::Time result;
            result.sec = stamp.sec;
            result.nanosec = stamp.nanosec;
            return result;
        }

        provizio::contained_time to_contained_time(const builtin_interfaces::msg::Time &stamp)
        {
            return {stamp.sec, stamp.nanosec};
        }

        std_msgs::msg::Header to_ros2_header(provizio::contained_header header)
        {
            std_msgs::msg::Header result;
            result.frame_id = std::move(header.frame_id);
            result.stamp = to_ros2_time(header.stamp);
            return result;
        }

        provizio::contained_header to_contained_header(std_msgs::msg::Header header)
        {
            return {std::move(header.frame_id), to_contained_time(header.stamp)};
        }

        sensor_msgs::msg::PointField to_ros2_point_field(provizio::contained_point_field &&field)
        {
            sensor_msgs::msg::PointField result;
            result.name = std::move(field.name);
            result.offset = field.offset;
            result.datatype = field.datatype;
            result.count = field.count;
            return result;
        }

        std::vector<sensor_msgs::msg::PointField> to_ros2_point_fields(
            std::vector<provizio::contained_point_field> fields)
        {
            std::vector<sensor_msgs::msg::PointField> result;
            result.reserve(fields.size());
            std::transform(std::make_move_iterator(fields.begin()), std::make_move_iterator(fields.end()),
                           std::back_inserter(result), to_ros2_point_field);
            return result;
        }

        template <typename ros2_vector3_type, typename contained_vector3_type>
        ros2_vector3_type to_ros2_vector3(const contained_vector3_type &vector3)
        {
            ros2_vector3_type result;
            result.x = vector3.x;
            result.y = vector3.y;
            result.z = vector3.z;
            return result;
        }

        geometry_msgs::msg::Quaternion to_ros2_quaternion(const contained_quaternion &quaternion)
        {
            geometry_msgs::msg::Quaternion result;
            result.x = quaternion.x;
            result.y = quaternion.y;
            result.z = quaternion.z;
            result.w = quaternion.w;
            return result;
        }

        geometry_msgs::msg::Pose to_ros2_pose(const contained_pose &pose)
        {
            geometry_msgs::msg::Pose result;
            result.position = to_ros2_vector3<geometry_msgs::msg::Point>(pose.position);
            result.orientation = to_ros2_quaternion(pose.orientation);
            return result;
        }

        geometry_msgs::msg::PoseWithCovariance to_ros2_pose_with_covariance(const contained_pose_with_covariance &pose)
        {
            geometry_msgs::msg::PoseWithCovariance result;
            result.pose = to_ros2_pose(pose.pose);
            result.covariance = pose.covariance;
            return result;
        }

        geometry_msgs::msg::Twist to_ros2_twist(const provizio::contained_twist &twist)
        {
            geometry_msgs::msg::Twist result;
            result.linear = to_ros2_vector3<geometry_msgs::msg::Vector3>(twist.linear);
            result.angular = to_ros2_vector3<geometry_msgs::msg::Vector3>(twist.angular);
            return result;
        }

        geometry_msgs::msg::TwistWithCovariance to_ros2_twist_with_covariance(
            const provizio::contained_twist_with_covariance &twist)
        {
            geometry_msgs::msg::TwistWithCovariance result;
            result.twist = to_ros2_twist(twist.twist);
            result.covariance = twist.covariance;
            return result;
        }

        float get_float_host_byte_order(float value, const bool value_byte_order_bigendian)
        {
            if (value_byte_order_bigendian != is_host_big_endian)
            {
                // Byte order needs to be reversed
                std::uint32_t value_as_uint32_t; // NOLINT: Initialized right below
                static_assert(sizeof(value_as_uint32_t) == sizeof(value), "uint32_t and float sizes mismatch!");
                memcpy(&value_as_uint32_t, &value, sizeof(value_as_uint32_t));

                // Clang-tidy has a number of complains regarding the expression below, but we really have to do a
                // pretty non-standard thing here
                value_as_uint32_t =
                    ((value_as_uint32_t & 0xff000000) >> 24) | ((value_as_uint32_t & 0x00ff0000) >> 8) | // NOLINT
                    ((value_as_uint32_t & 0x0000ff00) << 8) | ((value_as_uint32_t & 0x000000ff) << 24);  // NOLINT

                memcpy(&value, &value_as_uint32_t, sizeof(value_as_uint32_t));
            }

            return value;
        }

        float get_float_from_buffer(const std::vector<uint8_t> &data, const std::size_t offset,
                                    const bool bigendian_data)
        {
            if (offset + sizeof(float) > data.size())
            {
                throw std::out_of_range{"offset is out of range of data"};
            }

            float value; // NOLINT: Initialized right below
            std::memcpy(&value, &data[offset], sizeof(float));
            return get_float_host_byte_order(value, bigendian_data);
        }
    } // namespace

    sensor_msgs::msg::PointCloud2 to_ros2_pointcloud2(provizio::contained_pointcloud2 message,
                                                      const float snr_threshold)
    {
        sensor_msgs::msg::PointCloud2 result;
        result.header = to_ros2_header(std::move(message.header));
        result.height = message.height;
        result.is_bigendian = message.is_bigendian;
        result.point_step = message.point_step;
        result.row_step = message.row_step;
        result.is_dense = message.is_dense;
        if (snr_threshold <= 0.0F || message.height != 1)
        {
            // Output all points
            result.width = message.width;
            result.data = std::move(message.data);
        }
        else
        {
            // Apply SNR filter
            std::uint32_t snr_offset = message.point_step;
            for (const auto &field : message.fields)
            {
                if (field.name == snr_field_name && field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                    field.count == 1)
                {
                    snr_offset = field.offset;
                    break;
                }
            }

            if (snr_offset < message.point_step)
            {
                result.data.reserve(message.data.size());
                for (std::size_t i = 0; i < message.width; ++i)
                {
                    if (get_float_from_buffer(message.data, i * message.point_step + snr_offset,
                                              message.is_bigendian) >= snr_threshold)
                    {
                        // Add this point
                        ++result.width;
                        result.data.resize(std::size_t{message.point_step} * result.width);
                        std::memcpy(&result.data[std::size_t{result.width - 1} * message.point_step],
                                    &message.data[i * message.point_step], message.point_step);
                    }
                }
            }
            else
            {
                // Can't filter by SNR as SNR field not found
                result.width = message.width;
                result.data = std::move(message.data);
            }
        }
        result.fields = to_ros2_point_fields(std::move(message.fields));
        return result;
    }

    nav_msgs::msg::Odometry to_ros2_odometry(provizio::contained_odometry message)
    {
        nav_msgs::msg::Odometry result;
        result.header = to_ros2_header(std::move(message.header));
        result.child_frame_id = std::move(message.child_frame_id);
        result.pose = to_ros2_pose_with_covariance(message.pose);
        result.twist = to_ros2_twist_with_covariance(message.twist);
        return result;
    }

    sensor_msgs::msg::Image to_ros2_image(provizio::contained_image message)
    {
        sensor_msgs::msg::Image result;
        result.header = to_ros2_header(std::move(message.header));
        result.height = message.height;
        result.width = message.width;
        result.encoding = std::move(message.encoding);
        result.is_bigendian = message.is_bigendian;
        result.step = message.step;
        result.data = std::move(message.data);
        return result;
    }

    geometry_msgs::msg::Polygon to_ros2_polygon(const provizio::contained_polygon &polygon)
    {
        geometry_msgs::msg::Polygon result;
        result.points.reserve(polygon.points.size());
        std::transform(polygon.points.begin(), polygon.points.end(), std::back_inserter(result.points),
                       to_ros2_vector3<geometry_msgs::msg::Point32, provizio::contained_point32>);
        return result;
    }

    geometry_msgs::msg::PolygonStamped to_ros2_polygon_stamped(
        const provizio::contained_polygon_instance_stamped &message)
    {
        geometry_msgs::msg::PolygonStamped result;
        result.header = to_ros2_header(message.header);
        result.polygon = to_ros2_polygon(message.polygon.polygon);
        return result;
    }

#if PROVIZIO_POLYGON_INSTANCE_AVAILABLE
    geometry_msgs::msg::PolygonInstance to_ros2_polygon_instance(const provizio::contained_polygon_instance &message)
    {
        geometry_msgs::msg::PolygonInstance result;
        result.polygon = to_ros2_polygon(message.polygon);
        result.id = message.id;
        return result;
    }

    geometry_msgs::msg::PolygonInstanceStamped to_ros2_polygon_instance_stamped(
        const provizio::contained_polygon_instance_stamped &message)
    {
        geometry_msgs::msg::PolygonInstanceStamped result;
        result.header = to_ros2_header(message.header);
        result.polygon = to_ros2_polygon_instance(message.polygon);
        return result;
    }
#endif

    provizio_radar_api_ros2::msg::RadarInfo to_ros2_radar_info(provizio::contained_radar_info message)
    {
        provizio_radar_api_ros2::msg::RadarInfo result;
        result.header = to_ros2_header(std::move(message.header));
        result.serial_number = std::move(message.serial_number);
        result.current_range = message.current_range;
        result.supported_ranges = std::move(message.supported_ranges);
        result.current_multiplexing_mode = message.current_multiplexing_mode;
        return result;
    }

    provizio::contained_set_radar_range to_contained_set_radar_range(
        const provizio_radar_api_ros2::srv::SetRadarRange::Request &request)
    {
        provizio::contained_set_radar_range result;
        result.header = to_contained_header(request.header);
        result.serial_number = request.serial_number;
        result.target_range = request.target_range;
        return result;
    }
} // namespace provizio
