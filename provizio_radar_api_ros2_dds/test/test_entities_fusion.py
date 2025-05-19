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

DDS_DOMAIN_ID = 21
TIMEOUT_SEC = 8.0
MAX_MESSAGE_AGE = 0.5
TEST_NAME = "test_entities_fusion"
FRAME_ID = "test_entities_fusion_frame"
EXPECTED_ENTITIES = "[Entity(entity_id=1, camera_entity_id=2, entity_class=3, x=4.0, y=5.0, z=6.0, radar_relative_radial_velocity=7.0, ground_relative_radial_velocity=8.0, orientation_0=9.0, orientation_1=10.0, orientation_2=11.0, orientation_3=12.0, size_0=13.0, size_1=14.0, size_2=15.0, camera_bbox_0=16.0, camera_bbox_1=17.0, camera_bbox_2=18.0, camera_bbox_3=19.0, entity_confidence=20, entity_class_confidence=21)]"
EXPECTED_ENTITIES_NP = "[Entity(entity_id=np.uint32(1), camera_entity_id=np.uint32(2), entity_class=np.uint8(3), x=np.float32(4.0), y=np.float32(5.0), z=np.float32(6.0), radar_relative_radial_velocity=np.float32(7.0), ground_relative_radial_velocity=np.float32(8.0), orientation_0=np.float32(9.0), orientation_1=np.float32(10.0), orientation_2=np.float32(11.0), orientation_3=np.float32(12.0), size_0=np.float32(13.0), size_1=np.float32(14.0), size_2=np.float32(15.0), camera_bbox_0=np.float32(16.0), camera_bbox_1=np.float32(17.0), camera_bbox_2=np.float32(18.0), camera_bbox_3=np.float32(19.0), entity_confidence=np.uint8(20), entity_class_confidence=np.uint8(21))]"
NUM_MESSAGES_NEEDED = 10


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(TEST_NAME)
        self.subscription = self.create_subscription(
            PointCloud2,
            "/provizio/entities/fusion",
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

        entities = test_framework.read_points_list(msg, tuple_name="Entity")
        self.check_value(
            "entities",
            str(entities),
            [EXPECTED_ENTITIES, EXPECTED_ENTITIES_NP],
            multiple_options=True,
        )

        self.message_checked(NUM_MESSAGES_NEEDED)


def main(args=None):
    return test_framework.run(
        test_name=TEST_NAME,
        synthetic_data_dds_args=[
            "--fusion_entities",
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
