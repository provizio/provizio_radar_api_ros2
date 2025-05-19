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

DDS_DOMAIN_ID = 24
TIMEOUT_SEC = 8.0
MAX_MESSAGE_AGE = 0.5
TEST_NAME = "test_camera"
FRAME_ID = "test_camera_frame"
NUM_MESSAGES_NEEDED = 10
EXPECTED_ENCODING = "test_encoding"
EXPECTED_WIDTH = 2000
EXPECTED_HEIGHT = 1000
EXPECTED_STEP = 9
EXPECTED_IS_BIGENDIAN = False
EXPECTED_DATA = bytearray([1, 2, 4, 8, 16, 32, 64, 128])


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(TEST_NAME)
        self.subscription = self.create_subscription(
            Image,
            "/provizio/camera_raw",
            self.listener_callback,
            qos_profile=self.qos_profile,
        )

    def listener_callback(self, msg: Image):
        if msg.header.frame_id != FRAME_ID:
            # Something else received, we want another frame_id
            print(
                f"{TEST_NAME}: Unexpected frame_id message received: {msg.header.frame_id}"
            )
            return

        self.total_messages += 1
        if self.done:
            # Don't overwrite the result
            return

        message_age = test_framework.message_age(msg.header)
        print(f"{TEST_NAME}: Received message of age = {message_age} sec")
        if message_age > MAX_MESSAGE_AGE:
            print(
                f"{TEST_NAME}: Message delivery took too long: {message_age} sec",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.encoding != EXPECTED_ENCODING:
            print(
                f"{TEST_NAME}: encoding = {msg.encoding} received while {EXPECTED_ENCODING} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.width != EXPECTED_WIDTH:
            print(
                f"{TEST_NAME}: width = {msg.width} received while {EXPECTED_WIDTH} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.height != EXPECTED_HEIGHT:
            print(
                f"{TEST_NAME}: height = {msg.height} received while {EXPECTED_HEIGHT} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.step != EXPECTED_STEP:
            print(
                f"{TEST_NAME}: step = {msg.step} received while {EXPECTED_STEP} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.is_bigendian != EXPECTED_IS_BIGENDIAN:
            print(
                f"{TEST_NAME}: is_bigendian = {msg.is_bigendian} received while {EXPECTED_IS_BIGENDIAN} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        if msg.data != EXPECTED_DATA:
            print(
                f"{TEST_NAME}: data = {msg.data} received while {EXPECTED_DATA} was expected",
                flush=True,
            )

            self.success = False
            self.done = True

        self.successful_messages += 1
        if self.successful_messages >= NUM_MESSAGES_NEEDED:
            self.success = True
            self.done = True


def main(args=None):
    return test_framework.run(
        test_name=TEST_NAME,
        synthetic_data_dds_args=[
            "--camera_frames",
            f"--frame_id={FRAME_ID}",
            f"--dds_domain_id={DDS_DOMAIN_ID}",
        ],
        node_type=TestNode,
        timeout_sec=TIMEOUT_SEC,
        rclpy_args=args,
        node_args=[["provizio_dds_domain_id", DDS_DOMAIN_ID]],
    )


if __name__ == "__main__":
    main()
