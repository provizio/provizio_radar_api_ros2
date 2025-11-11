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

DDS_DOMAIN_ID = 23
TIMEOUT_SEC = 8.0
MAX_MESSAGE_AGE = 0.5
TEST_NAME = "test_radar_odometry"
FRAME_ID = "test_radar_odometry_frame"
NUM_MESSAGES_NEEDED = 10
EXPECTED_CHILD_FRAME_ID = "odometry_child_frame"
EXPECTED_POSE_COVARIANCE = [
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
EXPECTED_POSITION = [1.0, 2.0, 3.0]
EXPECTED_ORIENTATION = [4.0, 5.0, 6.0, 7.0]
EXPECTED_TWIST_COVARIANCE = [
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
EXPECTED_ANGULAR_TWIST = [-100.0, -200.0, -300.0]
EXPECTED_LINEAR_TWIST = [100.0, 200.0, 300.0]


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(TEST_NAME)
        self.subscription = self.create_subscription(
            Odometry,
            "/provizio/odometry/radar",
            self.listener_callback,
            qos_profile=self.qos_profile,
        )

    def listener_callback(self, msg: Odometry):
        if msg.header.frame_id != FRAME_ID:
            # Something else received, we want another FRAME_ID
            print(
                f"{TEST_NAME}: Unexpected FRAME_ID message received: {msg.header.frame_id}"
            )
            return

        self.total_messages += 1
        if self.done:
            # Don't overwrite the result
            return

        self.check_age(msg.header, MAX_MESSAGE_AGE)

        self.check_value(
            "msg.child_frame_id", msg.child_frame_id, EXPECTED_CHILD_FRAME_ID
        )
        self.check_value(
            "msg.pose.covariance", msg.pose.covariance, EXPECTED_POSE_COVARIANCE
        )
        self.check_value(
            "pose.pose.position",
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            EXPECTED_POSITION,
        )
        self.check_value(
            "pose.pose.orientation",
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            EXPECTED_ORIENTATION,
        )
        self.check_value(
            "msg.twist.covariance", msg.twist.covariance, EXPECTED_TWIST_COVARIANCE
        )
        self.check_value(
            "twist.twist.angular",
            [
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ],
            EXPECTED_ANGULAR_TWIST,
        )
        self.check_value(
            "twist.twist.linear",
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
            EXPECTED_LINEAR_TWIST,
        )

        self.message_checked(NUM_MESSAGES_NEEDED)


def main(args=None):
    return test_framework.run(
        test_name=TEST_NAME,
        synthetic_data_dds_args=[
            "--radar_odometry",
            f"--frame_id={FRAME_ID}",
            f"--dds_domain_id={DDS_DOMAIN_ID}",
        ],
        node_type=TestNode,
        timeout_sec=TIMEOUT_SEC,
        rclpy_args=args,
        node_args=[["provizio_dds_domain_id", DDS_DOMAIN_ID]],
        frame_id_filters_success=[None, FRAME_ID],
        frame_id_filters_failure=[FRAME_ID + "_mismatch"],
    )


if __name__ == "__main__":
    main()
