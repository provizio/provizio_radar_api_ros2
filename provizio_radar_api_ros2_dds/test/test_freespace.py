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

has_polygon_instance_stamped = False
try:
    from geometry_msgs.msg import PolygonInstanceStamped

    has_polygon_instance_stamped = True
except ImportError:
    # No PolygonInstanceStamped
    pass


dds_domain_id = 25
timeout_sec = 8.0
max_message_age = 0.5
test_name = "test_freespace"
frame_id = "test_freespace_frame"
num_messages_needed = 10
expected_polygon_id = 120
expected_points = [[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0], [10.0, 200.0, 3000.0]]


class FreespaceSource(Enum):
    RADAR = 0
    CAMERA = 1


_source = FreespaceSource.RADAR


class TestNode(test_framework.Node):

    def __init__(self):
        super().__init__(test_name)
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
        if has_polygon_instance_stamped:
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
        instance_message = has_polygon_instance_stamped and not isinstance(
            msg, PolygonStamped
        )

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
        print(
            f"{test_name}: Received message of age = {message_age} sec of type {type(msg).__name__}"
        )
        if message_age > max_message_age:
            print(
                f"{test_name}: Message delivery took too long: {message_age} sec",
                flush=True,
            )

            self.success = False
            self.done = True

        if instance_message and msg.polygon.id != expected_polygon_id:
            print(
                f"{test_name}: polygon id = {msg.polygon.id} received, {expected_polygon_id} was expected",
                flush=True,
            )

            self.success = False
            self.done = True
            return

        polygon_points = [
            [it.x, it.y, it.z]
            for it in (
                msg.polygon.polygon.points if instance_message else msg.polygon.points
            )
        ]
        if polygon_points != expected_points:
            print(
                f"{test_name}: polygon points = {polygon_points} received, {expected_points} was expected",
                flush=True,
            )

            self.success = False
            self.done = True
            return

        if instance_message:
            self.successful_messages_instance_stamped += 1
        else:
            self.successful_messages_stamped += 1

        if self.successful_messages_stamped >= num_messages_needed and (
            not has_polygon_instance_stamped
            or self.successful_messages_instance_stamped >= num_messages_needed
        ):
            self.success = True
            self.done = True


def main(source: FreespaceSource, args=None):
    global _source
    global test_name

    _source = source
    test_name = f"test_freespace_{source.name}"
    return test_framework.run(
        test_name=test_name,
        synthetic_data_dds_args=[
            (
                "--radar_freespace"
                if _source == FreespaceSource.RADAR
                else "--camera_freespace"
            ),
            f"--frame_id={frame_id}",
            f"--dds_domain_id={dds_domain_id}",
        ],
        node_type=TestNode,
        timeout_sec=timeout_sec,
        rclpy_args=args,
        node_args=[["provizio_dds_domain_id", dds_domain_id]],
    )


if __name__ == "__main__":
    main(FreespaceSource.RADAR)
    main(FreespaceSource.CAMERA)
