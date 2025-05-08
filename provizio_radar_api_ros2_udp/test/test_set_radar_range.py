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

from provizio_radar_api_ros2.srv import SetRadarRange
from provizio_radar_api_ros2.msg import RadarInfo
import test_framework
import threading
import time

dds_domain_id = 26
timeout_sec = 2.0
timeout_sec_long = 38.0  # As failure to set a range takes 30 seconds
total_test_timeout = (
    timeout_sec_long * 5
)  # As the setting is done a few times in this test
wait_for_service_timeout = 5.0
wait_for_service_retries = 8
test_name = "test_set_radar_range"
radar_position_id = 15
radar_pc_port_number = 17703
set_range_port_number = 17704
set_radar_range_start_range = RadarInfo.MEDIUM_RANGE
set_range_ok_fast = RadarInfo.SHORT_RANGE
set_range_ok_slow = RadarInfo.LONG_RANGE
set_range_fail = RadarInfo.ULTRA_LONG_RANGE
set_range_drop = RadarInfo.HYPER_LONG_RANGE
set_range_ip_address = "127.0.0.1"


def report(message):
    print(f"{test_name}: {message}")


def result_with_timeout(future, timeout=timeout_sec):
    time_was = time.time()
    while not future.done():
        if time.time() - time_was >= timeout:
            future.cancel()
            raise TimeoutError()
        time.sleep(0.05)
    return future.result()


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(test_name)
        self.client = self.create_client(SetRadarRange, "/provizio/set_radar_range")
        self.start_thread = threading.Thread(target=self.start_tests)
        self.start_thread.start()

    def __del__(self):
        try:
            self.start_thread.join()
        except:
            print(
                f"{test_name}: Failed to join the tests thread",
                flush=True,
            )

    def fail(self, error_message):
        print(
            f"{test_name}: {error_message}",
            flush=True,
        )

        self.success = False
        self.done = True

        return False

    def requesting(self, message):
        self.request_time = time.time()
        report(message)

    def got_response(self):
        report(f"Response received in {time.time() - self.request_time} sec")

    def start_tests(self):
        for i in range(wait_for_service_retries):
            if self.client.wait_for_service(wait_for_service_timeout):
                report("Service available, starting tests...")
                try:
                    if (
                        self.test_same_range()
                        and self.test_same_range()  # Yep, the test is done twice to make sure the quick-set logic is applied
                        and self.test_ok_slow()
                        and self.test_ok_fast()
                        and self.test_fail()
                        and self.test_drop()
                        and self.test_concurrent_requests()
                    ):
                        report("All good")
                        self.success = True
                        self.done = True
                except Exception as e:
                    self.fail(f"Exception {type(e).__name__}: {e}")
                break
            else:
                if i == wait_for_service_retries - 1:
                    self.fail("Timeout waiting for the service")

    def test_same_range(self):
        self.requesting("Setting same range...")

        request = SetRadarRange.Request()
        request.header.frame_id = f"provizio_radar_{radar_position_id}"
        request.target_range = set_radar_range_start_range

        response = result_with_timeout(self.client.call_async(request))
        self.got_response()

        if response.actual_range != request.target_range:
            return self.fail(
                f"response.actual_range = {response.actual_range} while {request.target_range} was expected"
            )

        self.successful_messages += 1
        return True

    def test_ok_fast(self):
        self.requesting("Setting range fast...")

        request = SetRadarRange.Request()
        request.header.frame_id = f"provizio_radar_{radar_position_id}"
        request.target_range = set_range_ok_fast

        response = result_with_timeout(self.client.call_async(request))
        self.got_response()

        if response.actual_range != request.target_range:
            return self.fail(
                f"response.actual_range = {response.actual_range} while {request.target_range} was expected"
            )

        self.successful_messages += 1
        return True

    def test_ok_slow(self):
        self.requesting("Setting range slow...")

        request = SetRadarRange.Request()
        request.header.frame_id = f"provizio_radar_{radar_position_id}"
        request.target_range = set_range_ok_slow

        response = result_with_timeout(
            self.client.call_async(request), timeout=timeout_sec_long
        )
        self.got_response()

        if response.actual_range != request.target_range:
            return self.fail(
                f"response.actual_range = {response.actual_range} while {request.target_range} was expected"
            )

        self.successful_messages += 1
        return True

    def test_fail(self):
        self.requesting("Setting range to fail...")

        previous_test_range = set_range_ok_fast

        request = SetRadarRange.Request()
        request.header.frame_id = f"provizio_radar_{radar_position_id}"
        request.target_range = set_range_fail

        response = result_with_timeout(
            self.client.call_async(request), timeout=timeout_sec_long
        )
        self.got_response()

        if response.actual_range != previous_test_range:
            return self.fail(
                f"response.actual_range = {response.actual_range} while {previous_test_range} was expected"
            )

        self.successful_messages += 1
        return True

    def test_drop(self):
        self.requesting("Setting range to drop...")

        previous_test_range = set_range_ok_fast

        request = SetRadarRange.Request()
        request.header.frame_id = f"provizio_radar_{radar_position_id}"
        request.target_range = set_range_drop

        response = result_with_timeout(
            self.client.call_async(request), timeout=timeout_sec_long
        )
        self.got_response()

        if response.actual_range != previous_test_range:
            return self.fail(
                f"response.actual_range = {response.actual_range} while {previous_test_range} was expected"
            )

        self.successful_messages += 1
        return True

    def test_concurrent_requests(self):
        self.requesting("Setting ranges concurrently...")

        request1 = SetRadarRange.Request()
        request1.header.frame_id = f"provizio_radar_{radar_position_id}"
        request1.target_range = set_range_ok_slow

        request2 = SetRadarRange.Request()
        request2.header.frame_id = f"provizio_radar_{radar_position_id}"
        request2.target_range = set_range_ok_fast

        future1 = self.client.call_async(request1)
        future2 = self.client.call_async(request2)

        response1 = result_with_timeout(future1, timeout=timeout_sec_long)
        self.got_response()

        response2 = result_with_timeout(future2)
        self.got_response()

        if response1.actual_range != request1.target_range:
            return self.fail(
                f"response1.actual_range = {response1.actual_range} while {request1.target_range} was expected"
            )
        if response2.actual_range != request2.target_range:
            return self.fail(
                f"response2.actual_range = {response2.actual_range} while {request2.target_range} was expected"
            )

        self.successful_messages += 1
        return True


def main(args=None):
    return test_framework.run(
        test_name=test_name,
        synthetic_data_udp_args=[
            "--set_radar_range",
            f"--radar_position_id={radar_position_id}",
            f"--radar_pc_port_number={radar_pc_port_number}",
            f"--set_radar_range_port_number={set_range_port_number}",
        ],
        node_type=TestNode,
        timeout_sec=total_test_timeout,
        rclpy_args=args,
        node_args=[
            ["point_clouds_udp_port", radar_pc_port_number],
            ["set_range_udp_port", set_range_port_number],
            ["set_range_ip_address", set_range_ip_address],
        ],
    )


if __name__ == "__main__":
    main()
