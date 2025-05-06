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

import sys
from sensor_msgs.msg import PointCloud2
import test_framework

dds_domain_id = 19
timeout_sec = 8.0
max_message_age = 0.5
test_name = "test_entities_radar"
frame_id = "test_entities_radar_frame"
expected_entities = "[Entity(entity_id=np.uint32(1), entity_class=np.uint8(2), x=np.float32(3.3), y=np.float32(4.4), z=np.float32(5.5), radar_relative_radial_velocity=np.float32(6.6), ground_relative_radial_velocity=np.float32(7.7), orientation_0=np.float32(8.8), orientation_1=np.float32(9.9), orientation_2=np.float32(10.1), orientation_3=np.float32(11.11), size_0=np.float32(12.12), size_1=np.float32(13.13), size_2=np.float32(14.14), entity_confidence=np.uint8(15), entity_class_confidence=np.uint8(16)), Entity(entity_id=np.uint32(101), entity_class=np.uint8(102), x=np.float32(-3.3), y=np.float32(-4.4), z=np.float32(-5.5), radar_relative_radial_velocity=np.float32(-6.6), ground_relative_radial_velocity=np.float32(-7.7), orientation_0=np.float32(-8.8), orientation_1=np.float32(-9.9), orientation_2=np.float32(-10.1), orientation_3=np.float32(-11.11), size_0=np.float32(-12.12), size_1=np.float32(-13.13), size_2=np.float32(-14.14), entity_confidence=np.uint8(115), entity_class_confidence=np.uint8(116))]"
expected_entities_np = "[Entity(entity_id=1, entity_class=2, x=3.3, y=4.4, z=5.5, radar_relative_radial_velocity=6.6, ground_relative_radial_velocity=7.7, orientation_0=8.8, orientation_1=9.9, orientation_2=10.1, orientation_3=11.11, size_0=12.12, size_1=13.13, size_2=14.14, entity_confidence=15, entity_class_confidence=16), Entity(entity_id=101, entity_class=102, x=-3.3, y=-4.4, z=-5.5, radar_relative_radial_velocity=-6.6, ground_relative_radial_velocity=-7.7, orientation_0=-8.8, orientation_1=-9.9, orientation_2=-10.1, orientation_3=-11.11, size_0=-12.12, size_1=-13.13, size_2=-14.14, entity_confidence=115, entity_class_confidence=116)]"
num_messages_needed = 10


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(test_name)
        self.subscription = self.create_subscription(
            PointCloud2,
            "/provizio/entities/radar",
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
                file=sys.stderr,
                flush=True,
            )

            self.success = False
            self.done = True

        entities = test_framework.read_points_list(msg, tuple_name="Entity")
        if str(entities) != expected_entities and str(entities) != expected_entities_np:
            print(
                f"{test_name}: {entities} received, {expected_entities} was expected",
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
            "--radar_entities",
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
