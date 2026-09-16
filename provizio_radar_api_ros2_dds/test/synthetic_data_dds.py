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
import asyncio
import provizio_dds
import signal
import threading
import time

# Radar range constants. As of provizio_dds 2.0 / provizio_dds_idls 2.1 the radar_info and
# set_radar_range range fields are plain uint32 (were a provizio_dds.<name>_range enum before), so the
# module-level enum constants no longer exist. These literal values match the on-wire encoding and
# provizio_radar_api_ros2/msg/RadarInfo.msg.
SHORT_RANGE = 0
MEDIUM_RANGE = 1
LONG_RANGE = 2
ULTRA_LONG_RANGE = 3
HYPER_LONG_RANGE = 4
UNKNOWN_RANGE = 65535


def make_range_vector(values):
    # supported_ranges is a sequence<uint32>. provizio_dds 2.0 wraps it as uint32_t_vector; fall back to
    # the legacy provizio_msg_radar_range_vector for older/interop builds.
    for name in ("uint32_t_vector", "provizio_msg_radar_range_vector"):
        vector_type = getattr(provizio_dds, name, None)
        if vector_type is None:
            continue
        vector = vector_type()
        for value in values:
            vector.append(value)
        return vector
    raise RuntimeError("provizio_dds exposes no uint32 vector type for supported_ranges")


# Constants
DEFAULT_FRAME_ID = "provizio_radar_front_center"
RADAR_PC_TOPIC_NAME = "rt/provizio_radar_point_cloud"
RADAR_PC_POINTS = [
    [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    [1.0, 2.0, 3.0, 4.0, 5.0, float("nan")],
]
RADAR_PC_SR_TOPIC_NAME = "rt/provizio_radar_point_cloud_sr"
RADAR_PC_SR_POINTS = [
    [-0.1, -0.2, -0.3, -0.4, 50.0, -0.6],
    [-1.0, -2.0, -3.0, -4.0, 500.0, -6.0],
]
RADAR_ENTITIES_TOPIC_NAME = "rt/provizio_entities"
RADAR_ENTITIES = [
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
CAMERA_ENTITIES_TOPIC_NAME = "rt/provizio_entities_camera"
CAMERA_ENTITIES = [
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
FUSED_ENTITIES_TOPIC_NAME = "rt/provizio_entities_fusion"
FUSED_ENTITIES = [
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
RADAR_INFO_TOPIC_NAME = "rt/provizio_radar_info"
RADAR_INFO_SERIAL_NUMBER = "0987654321"
RADAR_INFO_CURRENT_RANGE = LONG_RANGE
RADAR_INFO_SUPPORTED_RANGES = [
    MEDIUM_RANGE,
    LONG_RANGE,
    ULTRA_LONG_RANGE,
]
RADAR_ODOMETRY_TOPIC_NAME = "rt/provizio_radar_odometry"
ODOMETRY_CHILD_FRAME_ID = "odometry_child_frame"
ODOMETRY_POSITION = [1.0, 2.0, 3.0]
ODOMETRY_ORIENTATION = [4.0, 5.0, 6.0, 7.0]
ODOMETRY_POSE_COVARIANCE = [
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
ODOMETRY_TWIST_LINEAR = [100.0, 200.0, 300.0]
ODOMETRY_TWIST_ANGULAR = [-100.0, -200.0, -300.0]
ODOMETRY_TWIST_COVARIANCE = [
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
CAMERA_FRAMES_TOPIC_NAME = "rt/provizio_camera"
CAMERA_FRAMES_ENCODING = "test_encoding"
CAMERA_FRAMES_WIDTH = 2000
CAMERA_FRAMES_HEIGHT = 1000
CAMERA_FRAMES_STEP = 9
CAMERA_FRAMES_IS_BIGENDIAN = False
CAMERA_FRAMES_DATA = [1, 2, 4, 8, 16, 32, 64, 128]
RADAR_FREESPACE_TOPIC_NAME = "rt/provizio_freespace_poly"
CAMERA_FREESPACE_TOPIC_NAME = "rt/provizio_freespace_camera_poly"
FREESPACE_POLYGON_ID = 120
FREESPACE_POINTS = [[1.0, 2.0, 3.0], [-1.0, -2.0, -3.0], [10.0, 200.0, 3000.0]]
SET_RADAR_RANGE_SERVICE_NAME = "provizio_set_radar_range"
SET_RADAR_RANGE_START_RANGE = MEDIUM_RANGE
SET_RANGE_OK_FAST = SHORT_RANGE
SET_RANGE_OK_SLOW = LONG_RANGE
SET_RANGE_FAIL = ULTRA_LONG_RANGE
SET_RANGE_DROP = HYPER_LONG_RANGE
SET_RANGE_SLOW_TIME = 15.0
SET_RANGE_SUPPORTED_RANGES = [
    SHORT_RANGE,
    MEDIUM_RANGE,
    LONG_RANGE,
    ULTRA_LONG_RANGE,
    HYPER_LONG_RANGE,
]


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


def make_header(frame_id=DEFAULT_FRAME_ID):
    ns_in_sec = 1000000000
    timestamp = time.time_ns()
    return provizio_dds.point_cloud2.make_header(
        int(timestamp / ns_in_sec), timestamp % ns_in_sec, frame_id
    )


def publish_radar_pc(
    participant,
    stop_event,
    name="publish_radar_pc",
    topic_name=RADAR_PC_TOPIC_NAME,
    frame_id=DEFAULT_FRAME_ID,
    points=RADAR_PC_POINTS,
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
    topic_name=RADAR_ENTITIES_TOPIC_NAME,
    frame_id=DEFAULT_FRAME_ID,
    entities=RADAR_ENTITIES,
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
    topic_name=RADAR_INFO_TOPIC_NAME,
    frame_id=DEFAULT_FRAME_ID,
    publish_period=0.1,
):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.radar_infoPubSubType
    )

    def publish():
        radar_info = provizio_dds.radar_info()
        radar_info.header(make_header(frame_id))
        radar_info.serial_number(RADAR_INFO_SERIAL_NUMBER)
        radar_info.current_range(RADAR_INFO_CURRENT_RANGE)
        radar_info.supported_ranges(make_range_vector(RADAR_INFO_SUPPORTED_RANGES))
        return publisher.publish(radar_info)

    return spin(name, publish, stop_event, publish_period)


def publish_radar_odometry(
    participant,
    stop_event,
    name="publish_radar_odometry",
    topic_name=RADAR_ODOMETRY_TOPIC_NAME,
    frame_id=DEFAULT_FRAME_ID,
    publish_period=0.1,
):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.OdometryPubSubType
    )

    def publish():
        odometry = provizio_dds.Odometry()
        odometry.header(make_header(frame_id))
        odometry.child_frame_id(ODOMETRY_CHILD_FRAME_ID)
        position = provizio_dds.Point()
        position.x(ODOMETRY_POSITION[0])
        position.y(ODOMETRY_POSITION[1])
        position.z(ODOMETRY_POSITION[2])
        orientation = provizio_dds.Quaternion()
        orientation.x(ODOMETRY_ORIENTATION[0])
        orientation.y(ODOMETRY_ORIENTATION[1])
        orientation.z(ODOMETRY_ORIENTATION[2])
        orientation.w(ODOMETRY_ORIENTATION[3])
        pose = provizio_dds.Pose()
        pose.position(position)
        pose.orientation(orientation)
        pose_with_covariance = provizio_dds.PoseWithCovariance()
        pose_with_covariance.pose(pose)
        pose_with_covariance.covariance(ODOMETRY_POSE_COVARIANCE)
        odometry.pose(pose_with_covariance)
        linear = provizio_dds.Vector3()
        linear.x(ODOMETRY_TWIST_LINEAR[0])
        linear.y(ODOMETRY_TWIST_LINEAR[1])
        linear.z(ODOMETRY_TWIST_LINEAR[2])
        angular = provizio_dds.Vector3()
        angular.x(ODOMETRY_TWIST_ANGULAR[0])
        angular.y(ODOMETRY_TWIST_ANGULAR[1])
        angular.z(ODOMETRY_TWIST_ANGULAR[2])
        twist = provizio_dds.Twist()
        twist.linear(linear)
        twist.angular(angular)
        twist_with_covariance = provizio_dds.TwistWithCovariance()
        twist_with_covariance.twist(twist)
        twist_with_covariance.covariance(ODOMETRY_TWIST_COVARIANCE)
        odometry.twist(twist_with_covariance)
        return publisher.publish(odometry)

    return spin(name, publish, stop_event, publish_period)


def publish_camera_frames(
    participant,
    stop_event,
    name="publish_camera_frames",
    topic_name=CAMERA_FRAMES_TOPIC_NAME,
    frame_id=DEFAULT_FRAME_ID,
    publish_period=0.1,
):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.ImagePubSubType
    )

    def publish():
        image = provizio_dds.Image()
        image.header(make_header(frame_id))
        image.encoding(CAMERA_FRAMES_ENCODING)
        image.width(CAMERA_FRAMES_WIDTH)
        image.height(CAMERA_FRAMES_HEIGHT)
        image.step(CAMERA_FRAMES_STEP)
        image.is_bigendian(CAMERA_FRAMES_IS_BIGENDIAN)
        image.data(CAMERA_FRAMES_DATA)
        return publisher.publish(image)

    return spin(name, publish, stop_event, publish_period)


def publish_freespace(
    participant,
    stop_event,
    name="publish_freespace",
    topic_name=RADAR_FREESPACE_TOPIC_NAME,
    frame_id=DEFAULT_FRAME_ID,
    publish_period=0.1,
):
    publisher = provizio_dds.Publisher(
        participant, topic_name, provizio_dds.PolygonInstanceStampedPubSubType
    )

    def make_point(it):
        point = provizio_dds.Point32()
        point.x(it[0])
        point.y(it[1])
        point.z(it[2])
        return point

    def publish():
        freespace = provizio_dds.PolygonInstanceStamped()
        freespace.header(make_header(frame_id))
        polygon_instance = provizio_dds.PolygonInstance()
        polygon_instance.id(FREESPACE_POLYGON_ID)
        polygon = provizio_dds.Polygon()
        polygon.points([make_point(it) for it in FREESPACE_POINTS])
        polygon_instance.polygon(polygon)
        freespace.polygon(polygon_instance)

        return publisher.publish(freespace)

    return spin(name, publish, stop_event, publish_period)


def serve_set_radar_range(
    participant,
    stop_event,
    name="serve_set_radar_range",
    service_name=SET_RADAR_RANGE_SERVICE_NAME,
    info_topic_name=RADAR_INFO_TOPIC_NAME,
    frame_id=DEFAULT_FRAME_ID,
    publish_period=0.1,
):
    # The radar's current range, shared between the request/response Service handler and the radar_info
    # publisher. The ROS 2 wrapper's quick-set short-circuit reads the current range from radar_info.
    state = {"current_range": SET_RADAR_RANGE_START_RANGE}

    radar_info_publisher = provizio_dds.Publisher(
        participant, info_topic_name, provizio_dds.radar_infoPubSubType
    )

    def publish_radar_info():
        radar_info = provizio_dds.radar_info()
        radar_info.header(make_header(frame_id))
        radar_info.serial_number(RADAR_INFO_SERIAL_NUMBER)
        radar_info.current_range(state["current_range"])
        radar_info.supported_ranges(make_range_vector(SET_RANGE_SUPPORTED_RANGES))
        return radar_info_publisher.publish(radar_info)

    def make_response(request, success, error_message):
        response = provizio_dds.set_radar_range_Response()
        response.header(make_header(frame_id))
        response.success(success)
        response.error_message(error_message)
        response.serial_number(request.serial_number())
        response.current_range(state["current_range"])
        response.supported_ranges(make_range_vector(SET_RANGE_SUPPORTED_RANGES))
        return response

    # An async handler so a slow range change doesn't block concurrent requests (the Service runs async
    # handlers as concurrent asyncio tasks).
    async def serve(request):
        target_range = request.target_range()
        match target_range:
            case v if v == SET_RANGE_OK_FAST or v == SET_RADAR_RANGE_START_RANGE:
                print(
                    f"synthetic_data_dds: Setting the radar range (fast) = {target_range}"
                )
                state["current_range"] = target_range
                return make_response(request, True, "")

            case v if v == SET_RANGE_OK_SLOW:
                print(
                    f"synthetic_data_dds: Setting the radar range (slow) = {target_range}..."
                )
                await asyncio.sleep(SET_RANGE_SLOW_TIME)
                state["current_range"] = target_range
                print(
                    f"synthetic_data_dds: Setting the radar range (slow) done = {target_range}"
                )
                return make_response(request, True, "")

            case v if v == SET_RANGE_FAIL:
                # Don't change the range; respond with a failure
                print(
                    f"synthetic_data_dds: Refusing to change the range. current_range = {state['current_range']}"
                )
                return make_response(request, False, "Requested range is not supported")

            case _:  # SET_RANGE_DROP (and any unexpected range): drop the request without a response
                print(
                    f"synthetic_data_dds: Dropping the request without a response. current_range = {state['current_range']}"
                )
                raise provizio_dds.Service.IgnoreRequest()

    service = provizio_dds.Service(
        participant,
        provizio_dds.set_radar_range_RequestPubSubType,
        provizio_dds.set_radar_range_Request,
        provizio_dds.set_radar_range_ResponsePubSubType,
        serve,
        service_name=service_name,
    )

    info_thread = spin(name, publish_radar_info, stop_event, publish_period)

    class thread_wrapper:
        def __init__(self, service, info_thread):
            self.service = service
            self.info_thread = info_thread

        def join(self):
            self.info_thread.join()
            del self.service
            print(f"{name} service has finished")

    return thread_wrapper(service, info_thread)


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
        default=DEFAULT_FRAME_ID,
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
        action="store_true",
        default=False,
        help="Serve Radar Range Mode",
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
                topic_name=RADAR_PC_TOPIC_NAME,
                frame_id=args.frame_id,
                points=RADAR_PC_POINTS,
            )
        )
    if args.radar_pc_sr:
        threads.append(
            publish_radar_pc(
                participant,
                stop_event,
                name="publish_radar_pc_sr",
                topic_name=RADAR_PC_SR_TOPIC_NAME,
                frame_id=args.frame_id,
                points=RADAR_PC_SR_POINTS,
            )
        )
    if args.radar_entities:
        threads.append(
            publish_entities(
                participant,
                stop_event,
                name="publish_radar_entities",
                topic_name=RADAR_ENTITIES_TOPIC_NAME,
                frame_id=args.frame_id,
                entities=RADAR_ENTITIES,
                make_entities_function=provizio_dds.point_cloud2.make_radar_entities,
            )
        )
    if args.camera_entities:
        threads.append(
            publish_entities(
                participant,
                stop_event,
                name="publish_camera_entities",
                topic_name=CAMERA_ENTITIES_TOPIC_NAME,
                frame_id=args.frame_id,
                entities=CAMERA_ENTITIES,
                make_entities_function=provizio_dds.point_cloud2.make_camera_entities,
            )
        )
    if args.fusion_entities:
        threads.append(
            publish_entities(
                participant,
                stop_event,
                name="publish_fusion_entities",
                topic_name=FUSED_ENTITIES_TOPIC_NAME,
                frame_id=args.frame_id,
                entities=FUSED_ENTITIES,
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
    if args.radar_freespace:
        threads.append(
            publish_freespace(
                participant,
                stop_event,
                name="publish_radar_freespace",
                frame_id=args.frame_id,
                topic_name=RADAR_FREESPACE_TOPIC_NAME,
            )
        )
    if args.camera_freespace:
        threads.append(
            publish_freespace(
                participant,
                stop_event,
                name="publish_camera_freespace",
                frame_id=args.frame_id,
                topic_name=CAMERA_FREESPACE_TOPIC_NAME,
            )
        )
    if args.set_radar_range:
        threads.append(
            serve_set_radar_range(participant, stop_event, frame_id=args.frame_id)
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
    print("synthetic_data_dds: All threads finished")


if __name__ == "__main__":
    main()
