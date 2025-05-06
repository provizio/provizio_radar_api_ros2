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
import sys

dds_domain_id = 23
timeout_sec = 8.0
max_message_age = 0.5
test_name = "test_radar_odometry"
frame_id = "test_radar_odometry_frame"
num_messages_needed = 10
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

        message_age = test_framework.message_age(msg.header)
        print(f"{test_name}: Received message of age = {message_age} sec")
        if message_age > max_message_age:
            print(
                f"{test_name}: Message delivery took too long: {message_age} sec",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.child_frame_id != expected_child_frame_id:
            print(
                f"{test_name}: child_frame_id = {msg.child_frame_id} received while {expected_child_frame_id} was expected",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True

        if (msg.pose.covariance != expected_pose_covariance).any():
            print(
                f"{test_name}: pose.covariance = {msg.pose.covariance} received while {expected_pose_covariance} was expected",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True

        if [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z] != expected_position:
            print(
                f"{test_name}: pose.pose.position = {[msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]} received while {expected_position} was expected",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True

        if [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ] != expected_orientation:
            print(
                f"{test_name}: pose.pose.orientation = {[msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]} received while {expected_orientation} was expected",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True

        if (msg.twist.covariance != expected_twist_covariance).any():
            print(
                f"{test_name}: twist.covariance = {msg.pose.covariance} received while {expected_twist_covariance} was expected",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True

        if [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z] != expected_angular_twist:
            print(
                f"{test_name}: twist.twist.angular = {[msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]} received while {expected_angular_twist} was expected",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True

        if [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ] != expected_linear_twist:
            print(
                f"{test_name}: twist.twist.linear = {[msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]} received while {expected_linear_twist} was expected",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True

        self.successful_messages += 1
        if self.successful_messages >= num_messages_needed:
            self.success = True
            self.done = True


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
