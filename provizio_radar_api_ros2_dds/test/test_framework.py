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
import pathlib
import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import signal
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import subprocess
import sys
import threading
import time
from collections import namedtuple
from typing import Iterable, List, NamedTuple, Optional
import traceback


PACKAGE_NAME = "provizio_radar_api_ros2"


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
                try:
                    if actual == expected_val:
                        matches = True
                        break
                except ValueError:
                    if (actual == expected_val).all():
                        matches = True
                        break
        else:
            matches = actual == expected

        try:
            # For multi-item comparisons
            matches = matches.all()
        except:
            pass

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




def _bounded_rclpy_teardown(test_name, test_node, processes, timeout_sec=30.0):
    """Tears the rclpy side down, but never waits on it forever.

    destroy_node() and try_shutdown() can wedge inside the RMW. Observed on lyrical +
    rmw_fastrtps_cpp: a test whose messages never arrived timed out, and the teardown that followed
    never returned - two CI jobs sat for hours until the job limit killed them. Every test in this
    suite runs in this one process, so one wedged teardown takes the whole run with it, and the
    subprocess teardown that was already bounded never even got the chance to run.

    A teardown that does not finish is not something to carry on from: rclpy is left half shut down,
    so the next rclpy.init() would likely wedge in the same place and the run would hang anyway,
    just later and with a more confusing log. So say plainly what happened, take the child processes
    down, and exit non-zero - a fast, diagnosable failure instead of a silent multi-hour stall.
    """
    finished = threading.Event()

    def teardown():
        try:
            test_node.destroy_node()
            rclpy.try_shutdown()
        finally:
            finished.set()

    threading.Thread(
        target=teardown, name=f"{test_name}-rclpy-teardown", daemon=True
    ).start()

    if finished.wait(timeout_sec):
        return

    print(
        f"{test_name}: rclpy teardown did not complete within {timeout_sec} sec - the RMW is wedged. "
        "Failing the run rather than waiting: every test shares this process, so nothing after this "
        "would be trustworthy.",
        flush=True,
    )
    for process in processes:
        if process and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)

def _do_run(
    test_name,
    synthetic_data_dds_args,
    node_type,
    timeout_sec,
    lifecycle_node=False,
    node_args=None,
    rclpy_args=None,
    frame_id_filter=None,
    failure_expected=False,
):
    # Init rclpy
    rclpy.init(args=rclpy_args)

    package_name = "provizio_radar_api_ros2"
    node_name = (
        "provizio_radar_lifecycle_node" if lifecycle_node else "provizio_radar_node"
    )

    print(
        f"Running test {test_name} with {node_name} and frame_id_filter={frame_id_filter} (DDS API)..."
    )

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
    if frame_id_filter:
        if not node_args:
            node_cmd.append("--ros-args")
        node_cmd.append("-p")
        node_cmd.append(f"frame_id:={frame_id_filter}")

    synthetic_data_cmd = ["python3", "synthetic_data_dds.py"] + (
        synthetic_data_dds_args if synthetic_data_dds_args is not None else []
    )
    scripts_location = pathlib.Path(__file__).parent.resolve()

    # Lifecycle state each transition can be requested from. `shutdown` is reachable from any of the
    # three non-final states.
    transition_start_states = {
        "configure": ("unconfigured",),
        "activate": ("inactive",),
        "deactivate": ("active",),
        "cleanup": ("inactive",),
        "shutdown": ("unconfigured", "inactive", "active"),
    }

    def wait_for_transition(action, timeout_sec=60):
        # `ros2 lifecycle set` talks to the node's lifecycle services, which only exist once ROS 2
        # discovery has propagated them to the CLI's own (freshly created) node. Until then the request
        # fails with "Node not found" or an empty transition list, which is indistinguishable from the
        # transition genuinely being refused. So first wait until the node reports a state the transition
        # can actually be requested from, and fail with a message saying which of the two happened.
        # `ros2 lifecycle get` is used rather than `list` because `list` silently prints nothing (and still
        # exits 0) when given a fully qualified "/node" name. Also bail out early if the node died, rather
        # than waiting out the whole timeout for services that will never appear.
        expected_states = transition_start_states[action]
        end_time = time.time() + timeout_sec
        last_seen = ""
        while time.time() < end_time:
            if driver_process is not None and driver_process.poll() is not None:
                raise RuntimeError(
                    f"{node_name} exited with code {driver_process.returncode} before it could {action}"
                )
            try:
                state = subprocess.run(
                    ["ros2", "lifecycle", "get", f"/{node_name}"],
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                last_seen = state.stdout.strip()
                # Prints "<label> [<id>]", e.g. "inactive [2]"
                if any(last_seen.startswith(s) for s in expected_states):
                    return
            except subprocess.TimeoutExpired:
                pass
            time.sleep(0.5)

        raise RuntimeError(
            f"{node_name} never reached a state to {action} from "
            f"({' or '.join(expected_states)}) within {timeout_sec} sec. "
            f"Last state seen: {last_seen or '<none>'}"
        )

    def switch_node_state(action):
        # Only request the transition once it's actually offered (see wait_for_transition). Even then the
        # request can still lose a race: `ros2 lifecycle set` spins up its own ROS 2 node and rediscovers
        # the target from scratch, so under slow discovery it can find no services ("Node not found") or an
        # empty transition list ("Unknown transition requested, available ones are:" followed by nothing)
        # even though the node is healthy. Both are "not visible yet", not "the transition failed", so
        # retry only those two signatures - a transition the node genuinely refuses fails immediately.
        # Bound each attempt with `timeout`: if the node dies mid-transition, `ros2 lifecycle set` would
        # otherwise block forever waiting for a service that never returns, hanging the whole test.
        wait_for_transition(action)

        not_yet_visible = ("Unknown transition requested", "Node not found")
        end_time = time.time() + 60
        attempt = 0
        while True:
            attempt += 1
            result = subprocess.run(
                ["timeout", "30", "ros2", "lifecycle", "set", f"/{node_name}", action],
                capture_output=True,
                text=True,
            )
            output = (result.stdout or "") + (result.stderr or "")
            print(output, end="", flush=True)
            if result.returncode == 0:
                return

            if not any(s in output for s in not_yet_visible) or time.time() >= end_time:
                raise RuntimeError(
                    f"Failed to {action} {node_name} (attempt {attempt}): {output.strip() or 'no output'}"
                )

            print(
                f"{node_name} not visible to `ros2 lifecycle set` yet, retrying {action}...",
                flush=True,
            )
            time.sleep(1.0)

    driver_process = None
    synthetic_data_process = None
    try:
        # Start the driver node
        driver_process = subprocess.Popen(
            node_cmd,
            start_new_session=True,
        )

        # Start the synthetic data publishing over DDS in a separate process (to avoid confusing its Fast-DDS with ROS 2 one)
        synthetic_data_process = subprocess.Popen(
            synthetic_data_cmd,
            cwd=scripts_location,
            start_new_session=True,
        )

        # Start the receiving ROS 2 node and wait for it to finish or timeout
        test_node = node_type()

        # Configure and activate the node, if needed
        if lifecycle_node:
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
                if synthetic_data_process.poll() is not None:
                    print(
                        f"{test_name}: Synthetic data process finished with code {synthetic_data_process.poll()}"
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

            _bounded_rclpy_teardown(
                test_name, test_node, (synthetic_data_process, driver_process)
            )

        print(f"{test_name}: Finishing...")

    finally:
        if synthetic_data_process and synthetic_data_process.poll() is None:
            # Stop the synthetic data publishing
            os.killpg(os.getpgid(synthetic_data_process.pid), signal.SIGINT)

        if driver_process and driver_process.poll() is None:
            # Stop the provizio radar node
            os.killpg(os.getpgid(driver_process.pid), signal.SIGINT)

        # Wait till both are stopped. Bound each wait and escalate to SIGKILL if a process ignores SIGINT,
        # so a wedged node/synthetic-data process can never hang the test (and CI) indefinitely.
        for process in (synthetic_data_process, driver_process):
            if not process:
                continue
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                print(
                    f"{test_name}: process {process.pid} didn't stop on SIGINT, sending SIGKILL",
                    flush=True,
                )
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()

    # Report the results
    if test_node.success == failure_expected:
        print(
            f"{test_name}: Failure with frame_id_filter={frame_id_filter}. {test_node.total_messages} messages received.\n"
        )
        return 1

    print(f"{test_name}: Success with frame_id_filter={frame_id_filter}!\n")
    return 0


def _run_with_frame_id_filter(
    test_name,
    synthetic_data_dds_args,
    node_type,
    timeout_sec,
    run_nodes=RunNodes.ALL_AVAILABLE,
    node_args=None,
    rclpy_args=None,
    frame_id_filter=None,
    failure_expected=False,
):
    match run_nodes:
        case RunNodes.BOTH:
            for i in [False, True]:
                result = _do_run(
                    test_name=test_name,
                    synthetic_data_dds_args=synthetic_data_dds_args,
                    node_type=node_type,
                    timeout_sec=timeout_sec,
                    lifecycle_node=i,
                    node_args=node_args,
                    rclpy_args=rclpy_args,
                    frame_id_filter=frame_id_filter,
                    failure_expected=failure_expected,
                )
                if result != 0:
                    return result
            return 0

        case RunNodes.ALL_AVAILABLE:
            available_nodes = (
                subprocess.check_output(["ros2", "pkg", "executables", PACKAGE_NAME])
                .decode("utf-8")
                .splitlines()
            )
            nodes = []
            if f"{PACKAGE_NAME} provizio_radar_node" in available_nodes:
                nodes.append(False)
            if f"{PACKAGE_NAME} provizio_radar_lifecycle_node" in available_nodes:
                nodes.append(True)
            if len(nodes) == 0:
                print(f"{test_name}: No nodes found!")
                return 1
            for node in nodes:
                result = _do_run(
                    test_name=test_name,
                    synthetic_data_dds_args=synthetic_data_dds_args,
                    node_type=node_type,
                    timeout_sec=timeout_sec,
                    lifecycle_node=node,
                    node_args=node_args,
                    rclpy_args=rclpy_args,
                    frame_id_filter=frame_id_filter,
                    failure_expected=failure_expected,
                )
                if result != 0:
                    return result
            return 0

        case RunNodes.SIMPLE:
            return _do_run(
                test_name=test_name,
                synthetic_data_dds_args=synthetic_data_dds_args,
                node_type=node_type,
                timeout_sec=timeout_sec,
                lifecycle_node=False,
                node_args=node_args,
                rclpy_args=rclpy_args,
                frame_id_filter=frame_id_filter,
                failure_expected=failure_expected,
            )

        case RunNodes.LIFECYCLE:
            return _do_run(
                test_name=test_name,
                synthetic_data_dds_args=synthetic_data_dds_args,
                node_type=node_type,
                timeout_sec=timeout_sec,
                lifecycle_node=True,
                node_args=node_args,
                rclpy_args=rclpy_args,
                frame_id_filter=frame_id_filter,
                failure_expected=failure_expected,
            )


def run(
    test_name,
    synthetic_data_dds_args,
    node_type,
    timeout_sec,
    run_nodes=RunNodes.ALL_AVAILABLE,
    node_args=None,
    rclpy_args=None,
    frame_id_filters_success=[],
    frame_id_filters_failure=[],
):
    if len(frame_id_filters_success) + len(frame_id_filters_failure) > 0:
        for frame_id_filters, failure_expected in zip(
            [frame_id_filters_success, frame_id_filters_failure], [False, True]
        ):
            for frame_id_filter in frame_id_filters:
                return_code = _run_with_frame_id_filter(
                    test_name=test_name,
                    synthetic_data_dds_args=synthetic_data_dds_args,
                    node_type=node_type,
                    timeout_sec=timeout_sec,
                    run_nodes=run_nodes,
                    node_args=node_args,
                    rclpy_args=rclpy_args,
                    frame_id_filter=frame_id_filter,
                    failure_expected=failure_expected,
                )
                if return_code != 0:
                    return return_code
        return 0  # All good!
    else:
        return _run_with_frame_id_filter(
            test_name=test_name,
            synthetic_data_dds_args=synthetic_data_dds_args,
            node_type=node_type,
            timeout_sec=timeout_sec,
            run_nodes=run_nodes,
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
