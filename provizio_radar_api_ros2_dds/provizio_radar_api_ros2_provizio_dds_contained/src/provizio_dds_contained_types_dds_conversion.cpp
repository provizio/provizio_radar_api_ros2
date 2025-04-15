#include <algorithm>
#include <iterator>

#include "builtin_interfaces/msg/Time.h"
#include "std_msgs/msg/Header.h"

#include "provizio_dds_contained_types_dds_conversion.h"

namespace provizio
{
    namespace
    {
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
        result.current_range = static_cast<std::int8_t>(message.current_range());
        const auto &supported_ranges = message.supported_ranges();
        result.supported_ranges.reserve(supported_ranges.size());
        std::transform(supported_ranges.begin(), supported_ranges.end(), std::back_inserter(result.supported_ranges),
                       [](const provizio::msg::radar_range range) { return static_cast<std::int8_t>(range); });
        result.current_multiplexing_mode = -1; // TODO(iivanov): Use actual multiplexing mode when it's available
        return result;
    }

    provizio::msg::set_radar_range to_dds_set_radar_range(contained_set_radar_range message)
    {
        provizio::msg::set_radar_range result;
        result.header(to_dds_header(std::move(message.header)));
        result.serial_number(std::move(message.serial_number));
        result.target_range(static_cast<provizio::msg::radar_range>(message.target_range));
        return result;
    }
} // namespace provizio
