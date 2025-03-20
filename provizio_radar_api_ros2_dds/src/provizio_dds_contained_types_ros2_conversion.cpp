#include <algorithm>
#include <iterator>

#include "provizio_radar_api_ros2/provizio_dds_contained_types_ros2_conversion.h"

namespace provizio
{
    namespace
    {
        builtin_interfaces::msg::Time to_ros2_time(const provizio::contained_time &stamp)
        {
            builtin_interfaces::msg::Time result;
            result.sec = stamp.sec;
            result.nanosec = stamp.nanosec;
            return result;
        }

        std_msgs::msg::Header to_ros2_header(provizio::contained_header header)
        {
            std_msgs::msg::Header result;
            result.frame_id = std::move(header.frame_id);
            result.stamp = to_ros2_time(header.stamp);
            return result;
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
            std::transform(std::make_move_iterator(fields.begin()), std::make_move_iterator(fields.end()),
                           std::back_inserter(result), to_ros2_point_field);
            return result;
        }

        template <typename ros2_vector3_type> ros2_vector3_type to_ros2_vector3(const contained_vector3 &vector3)
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

        geometry_msgs::msg::PoseWithCovariance to_ros2_pose_with_covariance(contained_pose_with_covariance pose)
        {
            geometry_msgs::msg::PoseWithCovariance result;
            result.pose = to_ros2_pose(pose.pose);
            result.covariance = std::move(pose.covariance);
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
            provizio::contained_twist_with_covariance twist)
        {
            geometry_msgs::msg::TwistWithCovariance result;
            result.twist = to_ros2_twist(twist.twist);
            result.covariance = std::move(twist.covariance);
            return result;
        }
    } // namespace

    sensor_msgs::msg::PointCloud2 to_ros2_pointcloud2(provizio::contained_pointcloud2 message)
    {
        sensor_msgs::msg::PointCloud2 result;
        result.header = to_ros2_header(std::move(message.header));
        result.width = message.width;
        result.height = message.height;
        result.fields = to_ros2_point_fields(std::move(message.fields));
        result.is_bigendian = message.is_bigendian;
        result.point_step = message.point_step;
        result.row_step = message.row_step;
        result.data = std::move(message.data);
        result.is_dense = message.is_dense;
        return result;
    }

    nav_msgs::msg::Odometry to_ros2_odometry(provizio::contained_odometry message)
    {
        nav_msgs::msg::Odometry result;
        result.header = to_ros2_header(std::move(message.header));
        result.child_frame_id = std::move(message.child_frame_id);
        result.pose = to_ros2_pose_with_covariance(std::move(message.pose));
        result.twist = to_ros2_twist_with_covariance(std::move(message.twist));
        return result;
    }
} // namespace provizio
