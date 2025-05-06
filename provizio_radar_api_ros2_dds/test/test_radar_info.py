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
import sys

dds_domain_id = 22
timeout_sec = 8.0
max_message_age = 0.5
test_name = "test_radar_info"
frame_id = "test_radar_info_frame"
num_messages_needed = 10
expected_serial_number = "0987654321"
expected_current_range = RadarInfo.LONG_RANGE
expected_supported_ranges = [
    RadarInfo.MEDIUM_RANGE,
    RadarInfo.LONG_RANGE,
    RadarInfo.ULTRA_LONG_RANGE,
]


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(test_name)
        self.subscription = self.create_subscription(
            RadarInfo,
            "/provizio/radar_info",
            self.listener_callback,
            qos_profile=self.qos_profile,
        )

    def listener_callback(self, msg: RadarInfo):
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

        if msg.serial_number != expected_serial_number:
            print(
                f"{test_name}: serial_number = {msg.serial_number} received, {expected_serial_number} was expected",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True
            return

        if msg.current_range != expected_current_range:
            print(
                f"{test_name}: current_range = {msg.current_range} received, {expected_current_range} was expected",
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True
            return

        for i in range(len(expected_supported_ranges)):
            if msg.supported_ranges[i] != expected_supported_ranges[i]:
                print(
                    f"{test_name}: supported_range[{i}] = {msg.supported_ranges[i]} received, {expected_supported_ranges[i]} was expected",
                    file=sys.stderr,
                    flush=True,
                )

                self.success = False
                self.done = True
                return

        self.successful_messages += 1
        if self.successful_messages >= num_messages_needed:
            self.success = True
            self.done = True


def main(args=None):
    return test_framework.run(
        test_name=test_name,
        synthetic_data_dds_args=[
            "--radar_info",
            f"--frame_id={frame_id}",
            f"--dds_domain_id={dds_domain_id}",
        ],
        node_type=TestNode,
        timeout_sec=timeout_sec,
        rclpy_args=args,
        node_args=[["provizio_dds_domain_id", dds_domain_id]],
    )
    # TODO: Test with SNR filter too


if __name__ == "__main__":
    main()
