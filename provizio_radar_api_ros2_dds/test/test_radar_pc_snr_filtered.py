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

from sensor_msgs.msg import PointCloud2
import test_framework

DDS_DOMAIN_ID = 27
TIMEOUT_SEC = 8.0
MAX_MESSAGE_AGE = 0.5
TEST_NAME = "test_radar_pc_snr_filtered"
FRAME_ID = "test_radar_pc_snr_filtered_frame"
MAX_SNR = 5.0
EXPECTED_POINTS = "[Point(x=1.0, y=2.0, z=3.0, radar_relative_radial_velocity=4.0, signal_to_noise_ratio=5.0, ground_relative_radial_velocity=nan)]"
EXPECTED_POINTS_NP = "[Point(x=np.float32(1.0), y=np.float32(2.0), z=np.float32(3.0), radar_relative_radial_velocity=np.float32(4.0), signal_to_noise_ratio=np.float32(5.0), ground_relative_radial_velocity=np.float32(nan))]"
NUM_MESSAGES_NEEDED = 10

snr_threshold = 2.0


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(TEST_NAME)
        self.subscription = self.create_subscription(
            PointCloud2,
            "/provizio/radar_point_cloud",
            self.listener_callback,
            qos_profile=self.qos_profile,
        )

    def listener_callback(self, msg):
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

        self.check_age(msg.header, MAX_MESSAGE_AGE)

        points = test_framework.read_points_list(msg)
        self.check_value(
            "points",
            str(points),
            [EXPECTED_POINTS, EXPECTED_POINTS_NP] if snr_threshold <= MAX_SNR else "[]",
            multiple_options=(snr_threshold <= MAX_SNR),
        )

        self.message_checked(NUM_MESSAGES_NEEDED)


def main(high_snr_threshold=False, args=None):
    global snr_threshold
    if high_snr_threshold:
        snr_threshold = 100.0

    return test_framework.run(
        test_name=TEST_NAME,
        synthetic_data_dds_args=[
            "--radar_pc",
            f"--frame_id={FRAME_ID}",
            f"--dds_domain_id={DDS_DOMAIN_ID}",
        ],
        node_type=TestNode,
        timeout_sec=TIMEOUT_SEC,
        rclpy_args=args,
        node_args=[
            ["provizio_dds_domain_id", DDS_DOMAIN_ID],
            ["snr_threshold", snr_threshold],
        ],
    )


if __name__ == "__main__":
    main()
