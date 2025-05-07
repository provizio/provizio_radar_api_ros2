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

from sensor_msgs.msg import Image
import test_framework

dds_domain_id = 24
timeout_sec = 8.0
max_message_age = 0.5
test_name = "test_camera"
frame_id = "test_camera_frame"
num_messages_needed = 10
expected_encoding = "test_encoding"
expected_width = 2000
expected_height = 1000
expected_step = 9
expected_is_bigendian = False
expected_data = bytearray([1, 2, 4, 8, 16, 32, 64, 128])


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(test_name)
        self.subscription = self.create_subscription(
            Image,
            "/provizio/camera_raw",
            self.listener_callback,
            qos_profile=self.qos_profile,
        )

    def listener_callback(self, msg: Image):
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
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.encoding != expected_encoding:
            print(
                f"{test_name}: encoding = {msg.encoding} received while {expected_encoding} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.width != expected_width:
            print(
                f"{test_name}: width = {msg.width} received while {expected_width} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.height != expected_height:
            print(
                f"{test_name}: height = {msg.height} received while {expected_height} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.step != expected_step:
            print(
                f"{test_name}: step = {msg.step} received while {expected_step} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.is_bigendian != expected_is_bigendian:
            print(
                f"{test_name}: is_bigendian = {msg.is_bigendian} received while {expected_is_bigendian} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.data != expected_data:
            print(
                f"{test_name}: data = {msg.data} received while {expected_data} was expected",
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
            "--camera_frames",
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
