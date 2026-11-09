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
from enum import Enum
import signal
import socket
import struct
import time
import threading


PUBLISH_PERIOD = 0.1
RADAR_PC_POINTS = [
    [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    [1.0, 2.0, 3.0, 4.0, 5.0, float("nan")],
]
RADAR_PC_PROTOCOL_VERSION = 2
RADAR_ENTITY_PROTOCOL_VERSION = 1
# Each entity is sent in wire (struct) order: entity_id, x, y, z, radar_relative_radial_velocity,
# ground_relative_radial_velocity, orientation quaternion (w, x, y, z), bounding box size (x, y, z),
# entity_class, entity_confidence, entity_class_confidence, reserved. The driver re-emits the quaternion as
# (x, y, z, w), so the published orientation is (8.8, 9.9, 10.1, 11.11) for the first entity, etc.
RADAR_ENTITIES = [
    (1, 3.3, 4.4, 5.5, 6.6, 7.7, 11.11, 8.8, 9.9, 10.1, 12.12, 13.13, 14.14, 2, 15, 16, 0),
    (101, -3.3, -4.4, -5.5, -6.6, -7.7, -11.11, -8.8, -9.9, -10.1, -12.12, -13.13, -14.14, 102, 115, 116, 0),
]
SET_RADAR_RANGE_START_RANGE = 1  # medium_range
SET_RADAR_RANGE_PROTOCOL_VERSION = 2
SET_RANGE_OK_FAST = 0  # short_range
SET_RANGE_OK_SLOW = 2  # long_range
SET_RANGE_FAIL = 3  # ultra_long_range
SET_RANGE_DROP = 4  # hyper_long_range
SET_RANGE_SLOW_TIME = 15.0
SET_RANGE_ERROR_NOT_SUPPORTED = 95  # PROVIZIO_E_NOT_SUPPORTED (EOPNOTSUPP)


stop_event = None


class RadarPacket(Enum):
    POINT_CLOUD = 1
    SET_MODE = 2
    SET_MODE_ACK = 3  # Sent by the radar when a set-range request is received
    SET_MODE_RESPONSE = 4  # Sent by the radar when the range change completes
    ENTITIES = 5


class Runner:

    def __init__(
        self,
        stop_event,
        args=None,
        radar_pc=RADAR_PC_POINTS,
        publish_period=PUBLISH_PERIOD,
    ):
        self.args = args
        self.stop_event = stop_event
        self.publish_period = publish_period
        self.radar_pc = radar_pc
        self.radar_range = SET_RADAR_RANGE_START_RANGE
        self.error_code = None
        self.threads = []

        if args.radar_pc or args.set_radar_range:
            self.threads.append(threading.Thread(target=self.publish_radar_pc))

        if args.radar_entities:
            self.threads.append(threading.Thread(target=self.publish_radar_entities))

        if args.set_radar_range:
            self.threads.append(threading.Thread(target=self.set_radar_range))

        for it in self.threads:
            it.start()

    def wait(self):
        for it in self.threads:
            it.join()

    def stop(self):
        if self.error_code is None:
            self.error_code = 0
        self.stop_event.set()

    def finished(self):
        return self.stop_event.is_set()

    def publish_radar_pc(self):
        max_point_per_packet = 60

        pc_header_struct = struct.Struct(">HHIQHHHH")
        pc_point_struct = struct.Struct(">ffffff")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.bind(("127.0.0.1", 0))

        frame_index = 0
        while not self.finished():
            frame_index = (frame_index + 1) % 4294967296  # wrap at 2^32 (full uint32 range)
        
            timestamp = time.time_ns()

            # get all detections associated with frame index
            frame = self.radar_pc
            total_points = len(frame)
            points_left = total_points

            for i in range(0, total_points, max_point_per_packet):
                chunk = frame[i : i + min(max_point_per_packet, points_left)]
                points_left -= len(chunk)

                points = bytearray()
                for it in chunk:
                    point = pc_point_struct.pack(
                        it[0], it[1], it[2], it[3], it[4], it[5]
                    )

                    points.extend(point)

                packet = bytearray()
                header = pc_header_struct.pack(
                    RadarPacket.POINT_CLOUD.value,
                    RADAR_PC_PROTOCOL_VERSION,
                    frame_index,
                    timestamp,
                    self.args.radar_position_id,
                    total_points,
                    len(chunk),
                    self.radar_range,
                )
                packet.extend(header)
                packet.extend(points)

                sock.sendto(packet, ("127.0.0.1", self.args.radar_pc_port_number))

            time.sleep(self.publish_period)

        sock.close()

    def publish_radar_entities(self):
        # Entities arrive on the same UDP port as point clouds (a single radars connection handles both).
        header_struct = struct.Struct(">HHIQHHHH")
        entity_struct = struct.Struct(">I5f4f3f4B")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.bind(("127.0.0.1", 0))

        frame_index = 0
        while not self.finished():
            frame_index = (frame_index + 1) % 4294967296  # wrap at 2^32 (full uint32 range)

            timestamp = time.time_ns()
            num_entities = len(RADAR_ENTITIES)

            packet = bytearray()
            packet.extend(
                header_struct.pack(
                    RadarPacket.ENTITIES.value,
                    RADAR_ENTITY_PROTOCOL_VERSION,
                    frame_index,
                    timestamp,
                    self.args.radar_position_id,
                    num_entities,  # total_entities_in_frame
                    num_entities,  # num_entities_in_packet
                    self.radar_range,
                )
            )
            for entity in RADAR_ENTITIES:
                packet.extend(entity_struct.pack(*entity))

            sock.sendto(packet, ("127.0.0.1", self.args.radar_pc_port_number))

            time.sleep(self.publish_period)

        sock.close()

    def set_radar_range(self):
        request_struct = struct.Struct(">HHHH")
        # protocol_header (type, version), radar_position_id, requested_range, current_range, reserved, error_code
        response_struct = struct.Struct(">HHHHHHi")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(self.publish_period)
        sock.bind(("127.0.0.1", self.args.set_radar_range_port_number))

        def send_packet(packet_type, address, requested_range, error_code):
            sock.sendto(
                response_struct.pack(
                    packet_type,
                    SET_RADAR_RANGE_PROTOCOL_VERSION,
                    self.args.radar_position_id,
                    requested_range,
                    self.radar_range,  # current range
                    0,  # reserved
                    error_code,
                ),
                address,
            )

        # Slow range changes in progress: list of (address, target_range, completion_time). Kept as a list
        # (never cleared by a later request) so an already-acknowledged slow change always gets its
        # completion response sent - otherwise the waiting client would hang until timeout.
        pending_completions = []

        while not self.finished():
            now = time.time()
            still_pending = []
            for completion in pending_completions:
                completion_address, completion_range, completion_time = completion
                if now >= completion_time:
                    # The slow range change completes now: apply it and send the completion response
                    self.radar_range = completion_range
                    print(
                        f"synthetic_data_udp: Setting the radar range (slow) done = {completion_range}"
                    )
                    send_packet(
                        RadarPacket.SET_MODE_RESPONSE.value, completion_address, completion_range, 0
                    )
                else:
                    still_pending.append(completion)
            pending_completions = still_pending

            try:
                request, address = sock.recvfrom(1024)
            except socket.timeout:
                # It's fine, go on
                continue

            packet_type, protocol_version, radar_position_id, target_range = (
                request_struct.unpack_from(request)
            )

            if packet_type != RadarPacket.SET_MODE.value:
                print(
                    f"synthetic_data_udp: set_radar_range - unexpected packet type received: {packet_type}"
                )
                self.error_code = 1
                self.stop_event.set()
                break

            if protocol_version != SET_RADAR_RANGE_PROTOCOL_VERSION:
                print(
                    f"synthetic_data_udp: set_radar_range - unexpected protocol version received: {protocol_version}"
                )
                self.error_code = 1
                self.stop_event.set()
                break

            if radar_position_id != self.args.radar_position_id:
                print(
                    f"synthetic_data_udp: set_radar_range - unexpected radar_position_id received: {radar_position_id}"
                )
                self.error_code = 1
                self.stop_event.set()
                break

            error_code = 0
            send_ack = True

            match target_range:
                case v if v == SET_RANGE_OK_FAST or v == SET_RADAR_RANGE_START_RANGE:
                    # Change immediately; the acknowledgement already reports the new range, so the
                    # request completes on the acknowledgement (no separate response needed)
                    print(
                        f"synthetic_data_udp: Setting the radar range (fast) = {target_range}"
                    )
                    self.radar_range = target_range

                case v if v == SET_RANGE_OK_SLOW:
                    # Acknowledge now (range not changed yet) and schedule the change + completion response.
                    # Appended (not replacing) so a concurrent request can't orphan this pending completion.
                    print(
                        f"synthetic_data_udp: Setting the radar range (slow) = {target_range}..."
                    )
                    pending_completions.append(
                        (address, target_range, time.time() + SET_RANGE_SLOW_TIME)
                    )

                case v if v == SET_RANGE_FAIL:
                    # Don't change the range, but acknowledge with a failure error code
                    print(
                        f"synthetic_data_udp: Don't change the range but send a failure acknowledgement. current_range = {self.radar_range}"
                    )
                    error_code = SET_RANGE_ERROR_NOT_SUPPORTED

                case v if v == SET_RANGE_DROP:
                    # Don't change the range and don't acknowledge at all
                    print(
                        f"synthetic_data_udp: Don't change the range and don't send any acknowledgement. current_range = {self.radar_range}"
                    )
                    send_ack = False

            if send_ack:
                send_packet(
                    RadarPacket.SET_MODE_ACK.value, address, target_range, error_code
                )

        sock.close()


def run(arguments=None):
    global stop_event
    stop_event = threading.Event()

    parser = argparse.ArgumentParser(description="Print Command Line Arguments")
    # Define arguments with their default values
    parser.add_argument("--dds_domain_id", type=int, default=0, help="DDS Domain ID")
    parser.add_argument(
        "--radar_position_id",
        type=int,
        default=0,
        help="Radar Position ID",
    )
    parser.add_argument(
        "--radar_pc",
        action="store_true",
        default=False,
        help="Publish Radar Point Clouds",
    )
    parser.add_argument(
        "--radar_pc_port_number",
        type=int,
        default=7769,
        help="Radar Point Clouds UDP Port Number",
    )
    parser.add_argument(
        "--radar_entities",
        action="store_true",
        default=False,
        help="Publish Radar Entities",
    )
    parser.add_argument(
        "--set_radar_range",
        action="store_true",
        default=False,
        help="Serve Radar Range Mode",
    )
    parser.add_argument(
        "--set_radar_range_port_number",
        type=int,
        default=7770,
        help="Set Radar Range UDP Port Number",
    )

    return Runner(stop_event=stop_event, args=parser.parse_args(arguments))


def main(arguments=None):
    runner = run(arguments)
    runner.wait()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda sig, frame: stop_event.set())
    main()
