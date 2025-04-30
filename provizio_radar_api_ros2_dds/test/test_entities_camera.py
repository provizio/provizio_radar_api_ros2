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

dds_domain_id = 20
timeout_sec = 8.0
test_name = "test_entities_camera"
frame_id = "test_entities_camera_frame"
expected_entities = "[Entity(camera_entity_id=1, entity_class=2, x=3.0, y=4.0, z=5.0, camera_bbox_0=6.0, camera_bbox_1=7.0, camera_bbox_2=8.0, camera_bbox_3=9.0, entity_confidence=10, entity_class_confidence=11)]"
expected_entities_np = "[Entity(camera_entity_id=np.uint32(1), entity_class=np.uint8(2), x=np.float32(3.0), y=np.float32(4.0), z=np.float32(5.0), camera_bbox_0=np.float32(6.0), camera_bbox_1=np.float32(7.0), camera_bbox_2=np.float32(8.0), camera_bbox_3=np.float32(9.0), entity_confidence=np.uint8(10), entity_class_confidence=np.uint8(11))]"
num_messages_needed = 10


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(test_name)
        self.subscription = self.create_subscription(
            PointCloud2,
            "/provizio/entities/camera",
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

        points = test_framework.read_points_list(msg, tuple_name="Entity")
        if str(points) != expected_entities and str(points) != expected_entities_np:
            print(
                f"{test_name}: {points} received, {expected_entities} was expected",
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
            "--camera_entities",
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
