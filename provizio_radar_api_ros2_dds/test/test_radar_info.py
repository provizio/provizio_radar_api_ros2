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

from provizio_radar_api_ros2.msg import RadarInfo
import test_framework

DDS_DOMAIN_ID = 22
TIMEOUT_SEC = 8.0
MAX_MESSAGE_AGE = 0.5
TEST_NAME = "test_radar_info"
FRAME_ID = "test_radar_info_frame"
NUM_MESSAGES_NEEDED = 10
EXPECTED_SERIAL_NUMBER = "0987654321"
EXPECTED_CURRENT_RANGE = RadarInfo.LONG_RANGE
EXPECTED_SUPPORTED_RANGES = [
    RadarInfo.MEDIUM_RANGE,
    RadarInfo.LONG_RANGE,
    RadarInfo.ULTRA_LONG_RANGE,
]


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(TEST_NAME)
        self.subscription = self.create_subscription(
            RadarInfo,
            "/provizio/radar_info",
            self.listener_callback,
            qos_profile=self.qos_profile,
        )

    def listener_callback(self, msg: RadarInfo):
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

        if msg.serial_number != EXPECTED_SERIAL_NUMBER:
            print(
                f"{TEST_NAME}: serial_number = {msg.serial_number} received, {EXPECTED_SERIAL_NUMBER} was expected",
                flush=True,
            )

            self.success = False
            self.done = True
            return

        if msg.current_range != EXPECTED_CURRENT_RANGE:
            print(
                f"{TEST_NAME}: current_range = {msg.current_range} received, {EXPECTED_CURRENT_RANGE} was expected",
                flush=True,
            )

            self.success = False
            self.done = True
            return

        for i in range(len(EXPECTED_SUPPORTED_RANGES)):
            if msg.supported_ranges[i] != EXPECTED_SUPPORTED_RANGES[i]:
                print(
                    f"{TEST_NAME}: supported_range[{i}] = {msg.supported_ranges[i]} received, {EXPECTED_SUPPORTED_RANGES[i]} was expected",
                    flush=True,
                )

                self.success = False
                self.done = True
                return

        self.successful_messages += 1
        if self.successful_messages >= NUM_MESSAGES_NEEDED:
            self.success = True
            self.done = True


def main(args=None):
    return test_framework.run(
        test_name=TEST_NAME,
        synthetic_data_dds_args=[
            "--radar_info",
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
