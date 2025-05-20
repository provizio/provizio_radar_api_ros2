#!/usr/bin/python3

# Copyright 2025 Provizio Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from nav_msgs.msg import Odometry
import test_framework

dds_domain_id = 23
timeout_sec = 8.0
MAX_MESSAGE_AGE = 0.5
test_name = "test_radar_odometry"
frame_id = "test_radar_odometry_frame"
NUM_MESSAGES_NEEDED = 10
expected_child_frame_id = "odometry_child_frame"
expected_pose_covariance = [
    0.0,
    0.1,
    0.2,
    0.3,
    0.4,
    0.5,
    1.0,
    1.1,
    1.2,
    1.3,
    1.4,
    1.5,
    2.0,
    2.1,
    2.2,
    2.3,
    2.4,
    2.5,
    3.0,
    3.1,
    3.2,
    3.3,
    3.4,
    3.5,
    4.0,
    4.1,
    4.2,
    4.3,
    4.4,
    4.5,
    5.0,
    5.1,
    5.2,
    5.3,
    5.4,
    5.5,
]
expected_position = [1.0, 2.0, 3.0]
expected_orientation = [4.0, 5.0, 6.0, 7.0]
expected_twist_covariance = [
    -0.0,
    -0.1,
    -0.2,
    -0.3,
    -0.4,
    -0.5,
    -1.0,
    -1.1,
    -1.2,
    -1.3,
    -1.4,
    -1.5,
    -2.0,
    -2.1,
    -2.2,
    -2.3,
    -2.4,
    -2.5,
    -3.0,
    -3.1,
    -3.2,
    -3.3,
    -3.4,
    -3.5,
    -4.0,
    -4.1,
    -4.2,
    -4.3,
    -4.4,
    -4.5,
    -5.0,
    -5.1,
    -5.2,
    -5.3,
    -5.4,
    -5.5,
]
expected_angular_twist = [-100.0, -200.0, -300.0]
expected_linear_twist = [100.0, 200.0, 300.0]


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(test_name)
        self.subscription = self.create_subscription(
            Odometry,
            "/provizio/odometry/radar",
            self.listener_callback,
            qos_profile=self.qos_profile,
        )

    def listener_callback(self, msg: Odometry):
        if msg.header.frame_id != frame_id:
            # Something else received, we want another frame_id
            print(
                f"{test_name}: Unexpected frame_id message received: {msg.header.frame_id}"
            )
            return

        self.total_messages += 1
        if self.done:
            # Don't overwrite the result
            return

        self.check_age(msg.header, MAX_MESSAGE_AGE)

        self.check_value(
            "msg.child_frame_id", msg.child_frame_id, expected_child_frame_id
        )
        self.check_value(
            "msg.pose.covariance", msg.pose.covariance, expected_pose_covariance
        )
        self.check_value(
            "pose.pose.position",
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            expected_position,
        )
        self.check_value(
            "pose.pose.orientation",
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            expected_orientation,
        )
        self.check_value(
            "msg.twist.covariance", msg.twist.covariance, expected_twist_covariance
        )
        self.check_value(
            "twist.twist.angular",
            [
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ],
            expected_angular_twist,
        )
        self.check_value(
            "twist.twist.linear",
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
            expected_linear_twist,
        )

        self.message_checked(NUM_MESSAGES_NEEDED)


def main(args=None):
    return test_framework.run(
        test_name=test_name,
        synthetic_data_dds_args=[
            "--radar_odometry",
            f"--frame_id={frame_id}",
            f"--dds_domain_id={dds_domain_id}",
        ],
        node_type=TestNode,
        timeout_sec=timeout_sec,
        rclpy_args=args,
        node_args=[["provizio_dds_domain_id", dds_domain_id]],
    )


if __name__ == "__main__":
    main()
