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

import argparse
import provizio_dds
import signal
import threading
import time

# Constants
default_frame_id = "provizio_radar_front_center"
radar_pc_topic_name = "rt/provizio_radar_point_cloud"
radar_pc_points = [
    [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    [1.0, 2.0, 3.0, 4.0, 5.0, float("nan")],
]
radar_pc_sr_topic_name = "rt/provizio_radar_point_cloud_sr"
radar_pc_sr_points = [
    [-0.1, -0.2, -0.3, -0.4, 50.0, -0.6],
    [-1.0, -2.0, -3.0, -4.0, 500.0, -6.0],
]
radar_entities_topic_name = "rt/provizio_entities"
radar_entities = [
    [
        1,
        2,
        3.3,
        4.4,
        5.5,
        6.6,
        7.7,
        8.8,
        9.9,
        10.10,
        11.11,
        12.12,
        13.13,
        14.14,
        15,
        16,
    ],
    [
        101,
        102,
        -3.3,
        -4.4,
        -5.5,
        -6.6,
        -7.7,
        -8.8,
        -9.9,
        -10.10,
        -11.11,
        -12.12,
        -13.13,
        -14.14,
        115,
        116,
    ],
]
camera_entities_topic_name = "rt/provizio_entities_camera"
camera_entities = [
    [
        1,
        2,
        3.0,
        4.0,
        5.0,
        6.0,
        7.0,
        8.0,
        9.0,
        10,
        11,
    ]
]
fused_entities_topic_name = "rt/provizio_entities_fusion"
fused_entities = [
    [
        1,
        2,
        3,
        4.0,
        5.0,
        6.0,
        7.0,
        8.0,
        9.0,
        10.0,
        11.0,
        12.0,
        13.0,
        14.0,
        15.0,
        16.0,
        17.0,
        18.0,
        19.0,
        20,
        21,
    ]
]
radar_info_topic_name = "rt/provizio_radar_info"
radar_info_serial_number = "0987654321"
radar_info_current_range = provizio_dds.long_range
radar_info_supported_ranges = [
    provizio_dds.medium_range,
    provizio_dds.long_range,
    provizio_dds.ultra_long_range,
]
radar_odometry_topic_name = "rt/provizio_radar_odometry"
odometry_child_frame_id = "odometry_child_frame"
odometry_position = [1.0, 2.0, 3.0]
odometry_orientation = [4.0, 5.0, 6.0, 7.0]
odometry_pose_covariance = [
    0.0,
    0.1,
    0.2,
    0.3,
    0.4,
    0.5,
    1.0,
    1.1,
    1.2,
    1.3,
    1.4,
    1.5,
    2.0,
    2.1,
    2.2,
    2.3,
    2.4,
    2.5,
    3.0,
    3.1,
    3.2,
    3.3,
    3.4,
    3.5,
    4.0,
    4.1,
    4.2,
    4.3,
    4.4,
    4.5,
    5.0,
    5.1,
    5.2,
    5.3,
    5.4,
    5.5,
]
odometry_twist_linear = [100.0, 200.0, 300.0]
odometry_twist_angular = [-100.0, -200.0, -300.0]
odometry_twist_covariance = [
    -0.0,
    -0.1,
    -0.2,
    -0.3,
    -0.4,
    -0.5,
    -1.0,
    -1.1,
    -1.2,
    -1.3,
    -1.4,
    -1.5,
    -2.0,
    -2.1,
    -2.2,
    -2.3,
    -2.4,
    -2.5,
    -3.0,
    -3.1,
    -3.2,
    -3.3,
    -3.4,
    -3.5,
    -4.0,
    -4.1,
    -4.2,
    -4.3,
    -4.4,
    -4.5,
    -5.0,
    -5.1,
    -5.2,
    -5.3,
    -5.4,
    -5.5,
]
camera_frames_topic_name = "rt/provizio_camera"
camera_frames_encoding = "test_encoding"
camera_frames_width = 2000
camera_frames_height = 1000
camera_frames_step = 9
camera_frames_is_bigendian = False
camera_frames_data = [1, 2, 4, 8, 16, 32, 64, 128]


def spin(name, iteration_function, stop_event, period, *args, **kwargs):
    class bg_thread(threading.Thread):
        def __init__(self, iteration_function, stop_event, period, *args, **kwargs):
            super().__init__()
            self.daemon = True
            self.iteration_function = iteration_function
            self.stop_event = stop_event
            self.period = period
            self.successful_iterations = 0
            self.failed_iterations = 0
            self.args = args
            self.kwargs = kwargs

        def run(self):
            while not self.stop_event.is_set():
                if self.iteration_function(*self.args, **self.kwargs):
                    self.successful_iterations += 1
                else:
                    self.failed_iterations += 1
                if self.stop_event.wait(timeout=self.period):
                    break
            print(
                f"{name} thread has finished. OK: {self.successful_iterations}, Failed: {self.failed_iterations}"
            )

    thread = bg_thread(iteration_function, stop_event, period)
    thread.start()
    return thread


def make_header(frame_id=default_frame_id):
    ns_in_sec = 1000000000
    timestamp = time.time_ns()
    return provizio_dds.point_cloud2.make_header(
        int(timestamp / ns_in_sec), timestamp % ns_in_sec, frame_id
    )


def publish_radar_pc(
    participant,
    stop_event,
    name="publish_radar_pc",
    topic_name=radar_pc_topic_name,
    frame_id=default_frame_id,
    points=radar_pc_points,
    publish_period=0.1,
):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.PointCloud2PubSubType
    )

    def publish():
        return publisher.publish(
            provizio_dds.point_cloud2.make_radar_point_cloud(
                make_header(frame_id), points
            )
        )

    return spin(name, publish, stop_event, publish_period)


def publish_entities(
    participant,
    stop_event,
    name="publish_radar_entities",
    topic_name=radar_entities_topic_name,
    frame_id=default_frame_id,
    entities=radar_entities,
    publish_period=0.1,
    make_entities_function=provizio_dds.point_cloud2.make_radar_entities,
):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.PointCloud2PubSubType
    )

    def publish():
        return publisher.publish(
            make_entities_function(make_header(frame_id), entities)
        )

    return spin(name, publish, stop_event, publish_period)


def publish_radar_info(
    participant,
    stop_event,
    name="publish_radar_info",
    topic_name=radar_info_topic_name,
    frame_id=default_frame_id,
    publish_period=0.1,
):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.radar_infoPubSubType
    )

    def publish():
        radar_info = provizio_dds.radar_info()
        radar_info.header(make_header(frame_id))
        radar_info.serial_number(radar_info_serial_number)
        radar_info.current_range(radar_info_current_range)
        supported_ranges = provizio_dds.provizio_msg_radar_range_vector()
        for it in radar_info_supported_ranges:
            supported_ranges.append(it)
        radar_info.supported_ranges(supported_ranges)
        return publisher.publish(radar_info)

    return spin(name, publish, stop_event, publish_period)


def publish_radar_odometry(
    participant,
    stop_event,
    name="publish_radar_odometry",
    topic_name=radar_odometry_topic_name,
    frame_id=default_frame_id,
    publish_period=0.1,
):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.OdometryPubSubType
    )

    def publish():
        odometry = provizio_dds.Odometry()
        odometry.header(make_header(frame_id))
        odometry.child_frame_id(odometry_child_frame_id)
        position = provizio_dds.Point()
        position.x(odometry_position[0])
        position.y(odometry_position[1])
        position.z(odometry_position[2])
        orientation = provizio_dds.Quaternion()
        orientation.x(odometry_orientation[0])
        orientation.y(odometry_orientation[1])
        orientation.z(odometry_orientation[2])
        orientation.w(odometry_orientation[3])
        pose = provizio_dds.Pose()
        pose.position(position)
        pose.orientation(orientation)
        pose_with_covariance = provizio_dds.PoseWithCovariance()
        pose_with_covariance.pose(pose)
        pose_with_covariance.covariance(odometry_pose_covariance)
        odometry.pose(pose_with_covariance)
        linear = provizio_dds.Vector3()
        linear.x(odometry_twist_linear[0])
        linear.y(odometry_twist_linear[1])
        linear.z(odometry_twist_linear[2])
        angular = provizio_dds.Vector3()
        angular.x(odometry_twist_angular[0])
        angular.y(odometry_twist_angular[1])
        angular.z(odometry_twist_angular[2])
        twist = provizio_dds.Twist()
        twist.linear(linear)
        twist.angular(angular)
        twist_with_covariance = provizio_dds.TwistWithCovariance()
        twist_with_covariance.twist(twist)
        twist_with_covariance.covariance(odometry_twist_covariance)
        odometry.twist(twist_with_covariance)
        return publisher.publish(odometry)

    return spin(name, publish, stop_event, publish_period)


def publish_camera_frames(
    participant,
    stop_event,
    name="publish_camera_frames",
    topic_name=camera_frames_topic_name,
    frame_id=default_frame_id,
    publish_period=0.1,
):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.ImagePubSubType
    )

    def publish():
        image = provizio_dds.Image()
        image.header(make_header(frame_id))
        image.encoding(camera_frames_encoding)
        image.width(camera_frames_width)
        image.height(camera_frames_height)
        image.step(camera_frames_step)
        image.is_bigendian(camera_frames_is_bigendian)
        image.data(camera_frames_data)
        return publisher.publish(image)

    return spin(name, publish, stop_event, publish_period)


stop_event = None


def run(arguments=None):
    global stop_event
    stop_event = threading.Event()
    parser = argparse.ArgumentParser(description="Print Command Line Arguments")

    # Define arguments with their default values
    parser.add_argument("--dds_domain_id", type=int, default=0, help="DDS Domain ID")
    parser.add_argument(
        "--frame_id",
        type=str,
        default=default_frame_id,
        help="DDS Frame ID",
    )
    parser.add_argument(
        "--radar_pc",
        action="store_true",
        default=False,
        help="Publish Radar Point Clouds",
    )
    parser.add_argument(
        "--radar_pc_sr",
        action="store_true",
        default=False,
        help="Publish Radar Point Clouds Super Resolution",
    )
    parser.add_argument(
        "--radar_info",
        action="store_true",
        default=False,
        help="Publish Radar Info",
    )
    parser.add_argument(
        "--set_radar_range",
        type=str,
        default=None,
        help="Serve Radar Range Mode: None/OK/Fail",
    )
    parser.add_argument(
        "--radar_odometry",
        action="store_true",
        default=False,
        help="Publish Radar Odometry",
    )
    parser.add_argument(
        "--radar_entities",
        action="store_true",
        default=False,
        help="Publish Radar Entities",
    )
    parser.add_argument(
        "--camera_entities",
        action="store_true",
        default=False,
        help="Publish Camera Entities",
    )
    parser.add_argument(
        "--fusion_entities",
        action="store_true",
        default=False,
        help="Publish Fusion Entities",
    )
    parser.add_argument(
        "--camera_frames",
        action="store_true",
        default=False,
        help="Publish Camera Frames",
    )
    parser.add_argument(
        "--radar_freespace",
        action="store_true",
        default=False,
        help="Publish Radar Free Space",
    )
    parser.add_argument(
        "--camera_freespace",
        action="store_true",
        default=False,
        help="Publish Camera Free Space",
    )

    args = parser.parse_args(arguments)
    participant = provizio_dds.make_domain_participant(args.dds_domain_id)

    signal.signal(signal.SIGINT, lambda sig, frame: stop_event.set())

    threads = []
    if args.radar_pc:
        threads.append(
            publish_radar_pc(
                participant,
                stop_event,
                name="publish_radar_pc",
                topic_name=radar_pc_topic_name,
                frame_id=args.frame_id,
                points=radar_pc_points,
            )
        )
    if args.radar_pc_sr:
        threads.append(
            publish_radar_pc(
                participant,
                stop_event,
                name="publish_radar_pc_sr",
                topic_name=radar_pc_sr_topic_name,
                frame_id=args.frame_id,
                points=radar_pc_sr_points,
            )
        )
    if args.radar_entities:
        threads.append(
            publish_entities(
                participant,
                stop_event,
                name="publish_radar_entities",
                topic_name=radar_entities_topic_name,
                frame_id=args.frame_id,
                entities=radar_entities,
                make_entities_function=provizio_dds.point_cloud2.make_radar_entities,
            )
        )
    if args.camera_entities:
        threads.append(
            publish_entities(
                participant,
                stop_event,
                name="publish_camera_entities",
                topic_name=camera_entities_topic_name,
                frame_id=args.frame_id,
                entities=camera_entities,
                make_entities_function=provizio_dds.point_cloud2.make_camera_entities,
            )
        )
    if args.fusion_entities:
        threads.append(
            publish_entities(
                participant,
                stop_event,
                name="publish_fusion_entities",
                topic_name=fused_entities_topic_name,
                frame_id=args.frame_id,
                entities=fused_entities,
                make_entities_function=provizio_dds.point_cloud2.make_fused_entities,
            )
        )
    if args.radar_info:
        threads.append(
            publish_radar_info(participant, stop_event, frame_id=args.frame_id)
        )
    if args.radar_odometry:
        threads.append(
            publish_radar_odometry(participant, stop_event, frame_id=args.frame_id)
        )
    if args.camera_frames:
        threads.append(
            publish_camera_frames(participant, stop_event, frame_id=args.frame_id)
        )

    return threads


def stop(threads=None):
    if stop_event is not None:
        stop_event.set()
    if threads:
        for thread in threads:
            thread.join()


def running():
    return stop_event is not None and not stop_event.is_set()


def main():
    for thread in run():
        thread.join()
    print("All threads finished")


if __name__ == "__main__":
    main()
