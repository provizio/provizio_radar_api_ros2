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
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
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





# Lifecycle transition ids by the action name this harness uses. "shutdown" is deliberately absent:
# which shutdown transition is valid depends on where the node currently is, see _SHUTDOWN_TRANSITIONS.
_LIFECYCLE_TRANSITIONS = {
    "configure": Transition.TRANSITION_CONFIGURE,
    "cleanup": Transition.TRANSITION_CLEANUP,
    "activate": Transition.TRANSITION_ACTIVATE,
    "deactivate": Transition.TRANSITION_DEACTIVATE,
}

_SHUTDOWN_TRANSITIONS = {
    State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
    State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_INACTIVE_SHUTDOWN,
    State.PRIMARY_STATE_ACTIVE: Transition.TRANSITION_ACTIVE_SHUTDOWN,
}

# Where each action is meant to leave the node. Reaching it is the definition of success, so an
# action whose node is already there is a no-op rather than an invalid request.
_STATE_AFTER = {
    "configure": State.PRIMARY_STATE_INACTIVE,
    "cleanup": State.PRIMARY_STATE_UNCONFIGURED,
    "activate": State.PRIMARY_STATE_ACTIVE,
    "deactivate": State.PRIMARY_STATE_INACTIVE,
    "shutdown": State.PRIMARY_STATE_FINALIZED,
}


class _LifecycleController:
    """Drives a managed node's transitions over its own lifecycle services.

    This replaces shelling out to `ros2 lifecycle set`, which is not reliable under load. That CLI
    creates a ROS 2 node and rediscovers the target from scratch on every single call, and when the
    rediscovery loses a race it reports "Transitioning failed" for a transition the node's own log
    shows completing. The harness would then retry, find the node somewhere other than where it
    thought, and report "Unknown transition requested, available ones are:" with an empty list -
    which reads like the node refusing a transition when nothing of the sort happened.

    Two changes fix that. One client is held for the whole run, so discovery happens once instead of
    per transition. And the node's own reported state is the authority on whether a transition is
    needed and whether it worked, rather than the exit status of a CLI that may have simply missed
    the answer.
    """

    def __init__(self, node_name, timeout_sec=60.0):
        self._node_name = node_name
        self._timeout_sec = timeout_sec
        self._node = rclpy.create_node(f"{node_name}_lifecycle_controller")
        self._get_state = self._node.create_client(GetState, f"/{node_name}/get_state")
        self._change_state = self._node.create_client(
            ChangeState, f"/{node_name}/change_state"
        )

    def destroy(self):
        self._node.destroy_node()

    def _call(self, client, request, timeout_sec):
        """Calls one service, returning None rather than raising if it does not answer in time."""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if not future.done():
            future.cancel()
            return None

        return future.result()

    def state(self, timeout_sec=15.0):
        """The node's current primary state id, or None if it did not answer."""
        response = self._call(self._get_state, GetState.Request(), timeout_sec)
        return response.current_state.id if response is not None else None

    def switch(self, action, driver_process=None):
        """Brings the node to the state `action` denotes, or raises if it cannot within the timeout."""
        target = _STATE_AFTER[action]
        end_time = time.time() + self._timeout_sec
        last_state = None

        while True:
            if driver_process is not None and driver_process.poll() is not None:
                raise RuntimeError(
                    f"{self._node_name} exited with code {driver_process.returncode} "
                    f"before it could {action}"
                )

            last_state = self.state()
            if last_state == target:
                return

            if last_state is not None:
                transition = (
                    _SHUTDOWN_TRANSITIONS.get(last_state)
                    if action == "shutdown"
                    else _LIFECYCLE_TRANSITIONS.get(action)
                )
                if transition is not None:
                    request = ChangeState.Request()
                    request.transition.id = transition
                    self._call(self._change_state, request, timeout_sec=30.0)
                    # The response is deliberately not trusted on its own: the loop re-reads the
                    # state, which is the only thing that actually says whether the node moved.

            if time.time() >= end_time:
                raise RuntimeError(
                    f"{self._node_name} did not reach the state to {action} within "
                    f"{self._timeout_sec} sec (last state id seen: {last_state})"
                )

            time.sleep(0.25)

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
    driver_process = None
    lifecycle = None
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
            lifecycle = _LifecycleController(node_name)
            lifecycle.switch("configure", driver_process)
            lifecycle.switch("activate", driver_process)

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
            # Deactivate, cleanup and shutdown the node, if needed. Best effort: the run is
            # already finishing, so a transition that cannot be made now - because the node has
            # died, say - must be reported rather than raised, or it would replace whatever
            # actually went wrong with a complaint about the tidying up afterwards.
            if lifecycle_node and lifecycle is not None:
                for action in ("deactivate", "cleanup", "shutdown"):
                    try:
                        lifecycle.switch(action, driver_process)
                    except RuntimeError as error:
                        print(
                            f"{test_name}: could not {action} while finishing: {error}",
                            flush=True,
                        )
                        break

            if lifecycle is not None:
                lifecycle.destroy()
                lifecycle = None

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
