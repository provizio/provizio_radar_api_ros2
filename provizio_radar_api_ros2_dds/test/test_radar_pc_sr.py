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

dds_domain_id = 18
timeout_sec = 8.0
max_message_age = 0.5
test_name = "test_radar_pc_sr"
frame_id = "test_radar_pc_sr_frame"
expected_points = "[Point(x=-0.1, y=-0.2, z=-0.3, radar_relative_radial_velocity=-0.4, signal_to_noise_ratio=50.0, ground_relative_radial_velocity=-0.6), Point(x=-1.0, y=-2.0, z=-3.0, radar_relative_radial_velocity=-4.0, signal_to_noise_ratio=500.0, ground_relative_radial_velocity=-6.0)]"
expected_points_np = "[Point(x=np.float32(-0.1), y=np.float32(-0.2), z=np.float32(-0.3), radar_relative_radial_velocity=np.float32(-0.4), signal_to_noise_ratio=np.float32(50.0), ground_relative_radial_velocity=np.float32(-0.6)), Point(x=np.float32(-1.0), y=np.float32(-2.0), z=np.float32(-3.0), radar_relative_radial_velocity=np.float32(-4.0), signal_to_noise_ratio=np.float32(500.0), ground_relative_radial_velocity=np.float32(-6.0))]"
num_messages_needed = 10


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(test_name)
        self.subscription = self.create_subscription(
            PointCloud2,
            "/provizio/radar_point_cloud_sr",
            self.listener_callback,
            qos_profile=self.qos_profile,
        )

    def listener_callback(self, msg):
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

        points = test_framework.read_points_list(msg)
        if str(points) != expected_points and str(points) != expected_points_np:
            print(
                f"{test_name}: {points} received, {expected_points} was expected",
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
            "--radar_pc_sr",
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
