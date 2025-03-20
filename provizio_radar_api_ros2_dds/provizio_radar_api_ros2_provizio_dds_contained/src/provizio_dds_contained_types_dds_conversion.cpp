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
            provizio::contained_time result;
            result.sec = stamp.sec();
            result.nanosec = stamp.nanosec();
            return result;
        }

        provizio::contained_header to_contained_header(const std_msgs::msg::Header &header)
        {
            provizio::contained_header result;
            result.frame_id = header.frame_id();
            result.stamp = to_contained_time(header.stamp());
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
            std::transform(fields.begin(), fields.end(), std::back_inserter(result), to_contained_point_field);
            return result;
        }

        template <typename dds_vector_type> // supports both Point and Vector3 types
        contained_vector3 to_contained_vector3(const dds_vector_type &vector)
        {
            return {vector.x(), vector.y(), vector.z()};
        }

        contained_quaternion to_contained_quaternion(const geometry_msgs::msg::Quaternion &orientation)
        {
            return {orientation.x(), orientation.y(), orientation.z(), orientation.w()};
        }

        contained_pose to_contained_pose(const geometry_msgs::msg::Pose &pose)
        {
            contained_pose result;
            result.position = to_contained_vector3(pose.position());
            result.orientation = to_contained_quaternion(pose.orientation());
            return result;
        }

        contained_pose_with_covariance to_contained_pose_with_covariance(
            const geometry_msgs::msg::PoseWithCovariance &pose)
        {
            contained_pose_with_covariance result;
            result.pose = to_contained_pose(pose.pose());
            result.covariance = pose.covariance();
            return result;
        }

        contained_twist to_contained_twist(const geometry_msgs::msg::Twist &twist)
        {
            contained_twist result;
            result.linear = to_contained_vector3(twist.linear());
            result.angular = to_contained_vector3(twist.angular());
            return result;
        }

        contained_twist_with_covariance to_contained_twist_with_covariance(
            const geometry_msgs::msg::TwistWithCovariance &twist)
        {
            contained_twist_with_covariance result;
            result.twist = to_contained_twist(twist.twist());
            result.covariance = twist.covariance();
            return result;
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
} // namespace provizio
