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

import test_framework
from enum import Enum
from geometry_msgs.msg import PolygonStamped

HAS_POLYGON_INSTANCE_STAMPED = False
try:
    from geometry_msgs.msg import PolygonInstanceStamped

    HAS_POLYGON_INSTANCE_STAMPED = True
except ImportError:
    # No PolygonInstanceStamped
    pass


DDS_DOMAIN_ID = 25
TIMEOUT_SEC = 8.0
MAX_MESSAGE_AGE = 0.5
TEST_NAME = "test_freespace"
FRAME_ID = "test_freespace_frame"
NUM_MESSAGES_NEEDED = 10
EXPECTED_POLYGON_ID = 120
EXPECTED_POINTS = [[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0], [10.0, 200.0, 3000.0]]


class FreespaceSource(Enum):
    RADAR = 0
    CAMERA = 1


_source = FreespaceSource.RADAR


class TestNode(test_framework.Node):

    def __init__(self):
        super().__init__(TEST_NAME)
        self.successful_messages_instance_stamped = 0
        self.successful_messages_stamped = 0
        self.subscription = self.create_subscription(
            PolygonStamped,
            (
                "/provizio/radar_freespace/stamped"
                if _source == FreespaceSource.RADAR
                else "/provizio/camera_freespace/stamped"
            ),
            self.listener_callback,
            qos_profile=self.qos_profile,
        )
        if HAS_POLYGON_INSTANCE_STAMPED:
            self.subscription = self.create_subscription(
                PolygonInstanceStamped,
                (
                    "/provizio/radar_freespace/instance_stamped"
                    if _source == FreespaceSource.RADAR
                    else "/provizio/camera_freespace/instance_stamped"
                ),
                self.listener_callback,
                qos_profile=self.qos_profile,
            )

    def listener_callback(self, msg):
        instance_message = HAS_POLYGON_INSTANCE_STAMPED and not isinstance(
            msg, PolygonStamped
        )

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

        if instance_message:
            self.check_value("msg.polygon.id", msg.polygon.id, EXPECTED_POLYGON_ID)

        polygon_points = [
            [it.x, it.y, it.z]
            for it in (
                msg.polygon.polygon.points if instance_message else msg.polygon.points
            )
        ]
        self.check_value("polygon_points", polygon_points, EXPECTED_POINTS)
        
        if not self.done:
            if instance_message:
                self.successful_messages_instance_stamped += 1
            else:
                self.successful_messages_stamped += 1

            if self.successful_messages_stamped >= NUM_MESSAGES_NEEDED and (
                not HAS_POLYGON_INSTANCE_STAMPED
                or self.successful_messages_instance_stamped >= NUM_MESSAGES_NEEDED
            ):
                self.succeed()


def main(source: FreespaceSource, args=None):
    global _source
    global TEST_NAME

    _source = source
    TEST_NAME = f"test_freespace_{source.name}"
    return test_framework.run(
        test_name=TEST_NAME,
        synthetic_data_dds_args=[
            (
                "--radar_freespace"
                if _source == FreespaceSource.RADAR
                else "--camera_freespace"
            ),
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
    main(FreespaceSource.RADAR)
    main(FreespaceSource.CAMERA)
