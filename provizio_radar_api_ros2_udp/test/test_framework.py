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

from enum import Enum
import os
import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import signal
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import subprocess
import time
from collections import namedtuple
from typing import Iterable, List, NamedTuple, Optional
import synthetic_data_udp
import traceback


package_name = "provizio_radar_api_ros2"


class Node(rclpy.node.Node):
    def __init__(self, test_name):
        super().__init__(test_name)
        self.test_name = test_name

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.total_messages = 0
        self.successful_messages = 0
        self.success = None
        self.done = False

    def fail(self):
        traceback.print_stack()

        if self.done:
            if self.success:
                raise RuntimeError("Can't fail a test after it succeeded")
            return

        self.success = False
        self.done = True

        return False

    def succeed(self):
        if self.done:
            if not self.success:
                raise RuntimeError("Can't succeed the test after it failed")
            return

        self.success = True
        self.done = True

        return True

    def succeed_unless_already_done(self):
        if not self.done:
            return self.succeed()
        return False

    def succeed_if_enough_and_not_yet_done(self, num_messages_needed):
        if self.successful_messages >= num_messages_needed:
            return self.succeed_unless_already_done()
        return False

    def message_checked(self, num_messages_needed):
        if not self.done:
            self.successful_messages += 1
            return self.succeed_if_enough_and_not_yet_done(num_messages_needed)
        return False

    def check_age(self, header, max_age):
        age = message_age(header)
        print(f"{self.test_name}: Received message of age = {age} sec")
        if age > max_age:
            print(
                f"{self.test_name}: Message delivery took too long: {age} sec",
                flush=True,
            )
            self.fail()
            return False
        return True

    def check_value(self, value_name, actual, expected, multiple_options: bool = False):
        if multiple_options:
            matches = False
            for expected_val in expected:
                if actual == expected_val:
                    matches = True
                    break
        else:
            matches = actual == expected

        if not matches:
            print(
                f"{self.test_name}: {value_name} = {actual} received, while {'one of ' if multiple_options else ''}{expected} was expected",
                flush=True,
            )
            self.fail()
            return False
        return True


class RunNodes(Enum):
    ALL_AVAILABLE = 0
    BOTH = 1
    SIMPLE = 2
    LIFECYCLE = 3


def _do_run(
    test_name,
    synthetic_data_udp_args,
    node_type,
    timeout_sec,
    lifecycle_node=False,
    node_args=None,
    rclpy_args=None,
):
    # Init rclpy
    rclpy.init(args=rclpy_args)

    package_name = "provizio_radar_api_ros2"
    node_name = (
        "provizio_radar_lifecycle_node" if lifecycle_node else "provizio_radar_node"
    )

    print(f"Running test {test_name} with {node_name} (UDP API)...")

    node_cmd = [
        "ros2",
        "run",
        package_name,
        node_name,
    ]
    if node_args:
        node_cmd.append("--ros-args")
        for param in node_args:
            node_cmd.append("-p")
            node_cmd.append(f"{param[0]}:={param[1]}")

    def switch_node_state(action):
        if os.system(f"ros2 lifecycle set /{node_name} {action}") != 0:
            raise RuntimeError(f"Failed to {action} {node_name}")

    driver_process = None
    synthetic_data = None
    try:
        # Start the driver node
        driver_process = subprocess.Popen(
            node_cmd,
            start_new_session=True,
        )

        # Start the synthetic data publishing over UDP
        synthetic_data = synthetic_data_udp.run(synthetic_data_udp_args)

        # Start the receiving ROS 2 node and wait for it to finish or timeout
        test_node = node_type()

        # Configure and activate the node, if needed
        if lifecycle_node:
            # First, wait for the node registration to be propagated in ROS 2
            os.system(
                f'timeout 20 bash -c "until ros2 lifecycle get /{node_name} | grep -q "unconfigured"; do sleep 0.1; done" || echo "Waiting for the lifecycle node timed out!"'
            )

            # Should be good to switch now
            switch_node_state("configure")
            switch_node_state("activate")

        end_time = time.time() + timeout_sec
        try:
            while True:
                if test_node.done:
                    print(
                        f"{test_name}: Test node finished. Success = {test_node.success}"
                    )
                    break
                if time.time() > end_time:
                    print(f"{test_name}: Timeout")
                    break
                if synthetic_data.finished():
                    print(
                        f"{test_name}: Synthetic data process finished with code {synthetic_data.error_code}"
                    )
                    break
                if driver_process.poll() is not None:
                    print(
                        f"{test_name}: {node_name} process finished with code {driver_process.poll()}"
                    )
                    break

                rclpy.spin_once(test_node, timeout_sec=timeout_sec / 20)
        except KeyboardInterrupt:
            print(f"{test_name}: Keyboard Interrupt")
        finally:
            # Deactivate, cleanup and shutdown the node, if needed
            if lifecycle_node:
                switch_node_state("deactivate")
                switch_node_state("cleanup")
                switch_node_state("shutdown")

            test_node.destroy_node()
            rclpy.try_shutdown()

        print(f"{test_name}: Finishing...")

    finally:
        if synthetic_data and not synthetic_data.finished():
            # Stop the synthetic data
            synthetic_data.stop()

        if driver_process and driver_process.poll() is None:
            # Stop the provizio radar node
            os.killpg(os.getpgid(driver_process.pid), signal.SIGINT)

        # Wait till both are finished
        if synthetic_data:
            synthetic_data.wait()
        if driver_process:
            driver_process.wait()

    # Report the results
    if not test_node.success:
        print(f"{test_name}: Failure. {test_node.total_messages} messages received.\n")
        return 1

    print(f"{test_name}: Success!\n")
    return 0


def run(
    test_name,
    synthetic_data_udp_args,
    node_type,
    timeout_sec,
    run_nodes=RunNodes.ALL_AVAILABLE,
    node_args=None,
    rclpy_args=None,
):
    match run_nodes:
        case RunNodes.BOTH:
            for i in [False, True]:
                result = _do_run(
                    test_name=test_name,
                    synthetic_data_udp_args=synthetic_data_udp_args,
                    node_type=node_type,
                    timeout_sec=timeout_sec,
                    lifecycle_node=i,
                    node_args=node_args,
                    rclpy_args=rclpy_args,
                )
                if result != 0:
                    return result
            return 0

        case RunNodes.ALL_AVAILABLE:
            available_nodes = (
                subprocess.check_output(["ros2", "pkg", "executables", package_name])
                .decode("utf-8")
                .splitlines()
            )
            nodes = []
            if f"{package_name} provizio_radar_node" in available_nodes:
                nodes.append(False)
            if f"{package_name} provizio_radar_lifecycle_node" in available_nodes:
                nodes.append(True)
            if len(nodes) == 0:
                print(f"{test_name}: No nodes found!")
                return 1
            for i in nodes:
                result = _do_run(
                    test_name=test_name,
                    synthetic_data_udp_args=synthetic_data_udp_args,
                    node_type=node_type,
                    timeout_sec=timeout_sec,
                    lifecycle_node=i,
                    node_args=node_args,
                    rclpy_args=rclpy_args,
                )
                if result != 0:
                    return result
            return 0

        case RunNodes.SIMPLE:
            return _do_run(
                test_name=test_name,
                synthetic_data_udp_args=synthetic_data_udp_args,
                node_type=node_type,
                timeout_sec=timeout_sec,
                lifecycle_node=False,
                node_args=node_args,
                rclpy_args=rclpy_args,
            )

        case RunNodes.LIFECYCLE:
            return _do_run(
                test_name=test_name,
                synthetic_data_udp_args=synthetic_data_udp_args,
                node_type=node_type,
                timeout_sec=timeout_sec,
                lifecycle_node=True,
                node_args=node_args,
                rclpy_args=rclpy_args,
            )


def read_points_list(
    cloud: PointCloud2,
    field_names: Optional[List[str]] = None,
    skip_nans: bool = False,
    uvs: Optional[Iterable] = None,
    tuple_name: str = "Point",
) -> List[NamedTuple]:
    """
    Read points from a provizio_dds.PointCloud2 message.

    This function returns a list of namedtuples. It operates on top of the
    read_points method. For more efficient access use read_points directly.

    :param cloud: The point cloud to read from. (Type: provizio_dds.PointCloud2)
    :param field_names: The names of fields to read. If None, read all fields.
                        (Type: Iterable, Default: None)
    :param skip_nans: If True, then don't return any point with a NaN value.
                      (Type: Bool, Default: False)
    :param uvs: If specified, then only return the points at the given
                coordinates. (Type: Iterable, Default: None]
    :return: List of namedtuples containing the values for each point
    """
    assert isinstance(cloud, PointCloud2), "cloud is not a provizio_dds.PointCloud2"

    if field_names is None:
        field_names = pc2.dtype_from_fields(
            cloud.fields, point_step=cloud.point_step
        ).names

    Point = namedtuple(tuple_name, field_names)

    return [Point._make(p) for p in pc2.read_points(cloud, field_names, skip_nans, uvs)]


def message_age(header):
    ns_in_sec = 1000000000
    header_timestamp = header.stamp.sec * ns_in_sec + header.stamp.nanosec
    timestamp_now = time.time_ns()
    return float(timestamp_now - header_timestamp) / ns_in_sec
