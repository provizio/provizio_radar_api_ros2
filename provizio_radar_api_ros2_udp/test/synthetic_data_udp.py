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
SET_RADAR_RANGE_START_RANGE = 1  # medium_range
SET_RADAR_RANGE_PROTOCOL_VERSION = 1
SET_RANGE_OK_FAST = 0  # short_range
SET_RANGE_OK_SLOW = 2  # long_range
SET_RANGE_FAIL = 3  # ultra_long_range
SET_RANGE_DROP = 4  # hyper_long_range
SET_RANGE_SLOW_TIME = 15.0


stop_event = None


class RadarPacket(Enum):
    POINT_CLOUD = 1
    SET_MODE = 2
    SET_MODE_ACK = 3


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
            frame_index = (frame_index + 1) % 4294967295
        
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

    def set_radar_range(self):
        request_struct = struct.Struct(">HHHH")
        acknowledgement_struct = struct.Struct(">HHHHI")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(self.publish_period)
        sock.bind(("127.0.0.1", self.args.set_radar_range_port_number))

        slow_target_range = None
        time_to_set_range_slowly = None

        while not self.finished():
            if (
                slow_target_range is not None
                and time.time() >= time_to_set_range_slowly
            ):
                # Simulates changing the range slowly
                self.radar_range = slow_target_range
                slow_target_range = None
                time_to_set_range_slowly = None
                print(
                    f"synthetic_data_udp: Setting the radar range (slow) done = {self.radar_range}"
                )

            try:
                error_code = 1
                request, address = sock.recvfrom(1024)
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

                match target_range:
                    case v if (
                        v == SET_RANGE_OK_FAST or v == SET_RADAR_RANGE_START_RANGE
                    ):
                        print(
                            f"synthetic_data_udp: Setting the radar range (fast) = {target_range}"
                        )
                        self.radar_range = target_range
                        error_code = 0
                        slow_target_range = None
                        time_to_set_range_slowly = None

                    case v if v == SET_RANGE_OK_SLOW:
                        print(
                            f"synthetic_data_udp: Setting the radar range (slow) = {target_range}..."
                        )
                        slow_target_range = target_range
                        time_to_set_range_slowly = time.time() + SET_RANGE_SLOW_TIME
                        error_code = 0

                    case v if v == SET_RANGE_FAIL:
                        # Don't change the range
                        print(
                            f"synthetic_data_udp: Don't change the range but send appropriate acknowledgement. current_range = {self.radar_range}"
                        )
                        error_code = 1
                        slow_target_range = None
                        time_to_set_range_slowly = None

                    case v if v == SET_RANGE_DROP:
                        # Don't change the range and in addition to that stop publishing
                        print(
                            f"synthetic_data_udp: Don't change the range and don't send any acknowledgement. current_range = {self.radar_range}"
                        )
                        error_code = 1
                        slow_target_range = None
                        time_to_set_range_slowly = None

                if target_range != SET_RANGE_DROP:
                    time.sleep(2 * PUBLISH_PERIOD) # Enough time to publish at least one new point cloud
                    
                    # Send appropriate acknowledgement
                    sock.sendto(
                        acknowledgement_struct.pack(
                            RadarPacket.SET_MODE_ACK.value,
                            SET_RADAR_RANGE_PROTOCOL_VERSION,
                            radar_position_id,
                            target_range,
                            error_code,
                        ),
                        address,
                    )

            except socket.timeout:
                # It's fine, go on
                pass

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
