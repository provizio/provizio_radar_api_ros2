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

DDS_DOMAIN_ID = 26
# Generous enough for a fast set-range round-trip: the "quick-set" short-circuit relies on the radar_info
# cache being warm, which can lag node startup by a second or two on slower RMWs/runners; until then the
# request does a full (still quick) request/response round-trip, which must not spuriously time out.
TIMEOUT_SEC = 10.0
TIMEOUT_SEC_LONG = 38.0  # As failure to set a range takes 30 seconds
TOTAL_TEST_TIMEOUT = (
    TIMEOUT_SEC_LONG * 5
)  # As the setting is done a few times in this test
WAIT_FOR_SERVICE_TIMEOUT = 5.0
WAIT_FOR_SERVICE_RETRIES = 8
TEST_NAME = "test_set_radar_range"
FRAME_ID = "test_set_radar_range_frame"
SET_RADAR_RANGE_START_RANGE = RadarInfo.MEDIUM_RANGE
SET_RANGE_OK_FAST = RadarInfo.SHORT_RANGE
SET_RANGE_OK_SLOW = RadarInfo.LONG_RANGE
SET_RANGE_FAIL = RadarInfo.ULTRA_LONG_RANGE
SET_RANGE_DROP = RadarInfo.HYPER_LONG_RANGE
# Ranges the synthetic radar reports as supported in its set_radar_range response
EXPECTED_SUPPORTED_RANGES = [
    RadarInfo.SHORT_RANGE,
    RadarInfo.MEDIUM_RANGE,
    RadarInfo.LONG_RANGE,
    RadarInfo.ULTRA_LONG_RANGE,
    RadarInfo.HYPER_LONG_RANGE,
]


def report(message):
    print(f"{TEST_NAME}: {message}")


def result_with_timeout(future, timeout=TIMEOUT_SEC):
    time_was = time.time()
    while not future.done():
        if time.time() - time_was >= timeout:
            future.cancel()
            raise TimeoutError()
        time.sleep(0.05)
    return future.result()


class TestNode(test_framework.Node):
    def __init__(self):
        super().__init__(TEST_NAME)
        self.client = self.create_client(SetRadarRange, "/provizio/set_radar_range")
        self.start_thread = threading.Thread(target=self.start_tests)
        self.start_thread.start()

    def __del__(self):
        try:
            self.start_thread.join()
        except:
            print(
                f"{TEST_NAME}: Failed to join the tests thread",
                flush=True,
            )

    def fail_with_message(self, error_message):
        print(
            f"{TEST_NAME}: {error_message}",
            flush=True,
        )
        return self.fail()

    def requesting(self, message):
        self.request_time = time.time()
        report(message)

    def got_response(self):
        report(f"Response received in {time.time() - self.request_time} sec")

    def check_response(
        self, response, expected_range, expected_success, expected_supported_ranges=None
    ):
        # Returns whether the response matched, so callers can short-circuit instead of counting a
        # failed check as a successful message (matching the UDP twin of this test).
        if not self.check_value(
            "response.actual_range", response.actual_range, expected_range
        ):
            return False
        if not self.check_value("response.success", response.success, expected_success):
            return False
        if expected_success:
            if not self.check_value(
                "response.error_message (empty on success)", response.error_message, ""
            ):
                return False
        elif not response.error_message:
            self.fail_with_message("Expected a non-empty error_message on failure")
            return False
        if expected_supported_ranges is not None:
            if not self.check_value(
                "response.supported_ranges",
                list(response.supported_ranges),
                expected_supported_ranges,
            ):
                return False
        return True

    def start_tests(self):
        for i in range(WAIT_FOR_SERVICE_RETRIES):
            if self.client.wait_for_service(WAIT_FOR_SERVICE_TIMEOUT):
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
                        self.succeed()
                except Exception as e:
                    self.fail_with_message(f"Exception {type(e).__name__}: {e}")
                break
            else:
                if i == WAIT_FOR_SERVICE_RETRIES - 1:
                    self.fail_with_message("Timeout waiting for the service")

    def test_same_range(self):
        self.requesting("Setting same range...")

        request = SetRadarRange.Request()
        request.header.frame_id = FRAME_ID
        request.target_range = SET_RADAR_RANGE_START_RANGE

        response = result_with_timeout(self.client.call_async(request))
        self.got_response()

        # This may be served via the wrapper's quick-set short-circuit (no round-trip), which doesn't
        # populate supported_ranges, so only actual_range/success are checked here.
        if not self.check_response(
            response, request.target_range, expected_success=True
        ):
            return False

        self.successful_messages += 1
        return True

    def test_ok_fast(self):
        self.requesting("Setting range fast...")

        request = SetRadarRange.Request()
        request.header.frame_id = FRAME_ID
        request.target_range = SET_RANGE_OK_FAST

        response = result_with_timeout(self.client.call_async(request))
        self.got_response()

        if not self.check_response(
            response,
            request.target_range,
            expected_success=True,
            expected_supported_ranges=EXPECTED_SUPPORTED_RANGES,
        ):
            return False

        self.successful_messages += 1
        return True

    def test_ok_slow(self):
        self.requesting("Setting range slow...")

        request = SetRadarRange.Request()
        request.header.frame_id = FRAME_ID
        request.target_range = SET_RANGE_OK_SLOW

        response = result_with_timeout(
            self.client.call_async(request), timeout=TIMEOUT_SEC_LONG
        )
        self.got_response()

        if not self.check_response(
            response,
            request.target_range,
            expected_success=True,
            expected_supported_ranges=EXPECTED_SUPPORTED_RANGES,
        ):
            return False

        self.successful_messages += 1
        return True

    def test_fail(self):
        self.requesting("Setting range to fail...")

        previous_test_range = SET_RANGE_OK_FAST

        request = SetRadarRange.Request()
        request.header.frame_id = FRAME_ID
        request.target_range = SET_RANGE_FAIL

        response = result_with_timeout(
            self.client.call_async(request), timeout=TIMEOUT_SEC_LONG
        )
        self.got_response()

        # The radar received the request and refused it, so it still reports its supported ranges.
        if not self.check_response(
            response,
            previous_test_range,
            expected_success=False,
            expected_supported_ranges=EXPECTED_SUPPORTED_RANGES,
        ):
            return False

        self.successful_messages += 1
        return True

    def test_drop(self):
        self.requesting("Setting range to drop...")

        previous_test_range = SET_RANGE_OK_FAST

        request = SetRadarRange.Request()
        request.header.frame_id = FRAME_ID
        request.target_range = SET_RANGE_DROP

        response = result_with_timeout(
            self.client.call_async(request), timeout=TIMEOUT_SEC_LONG
        )
        self.got_response()

        # No response came back (dropped), so the wrapper reports the last known range with success=false
        # and an error_message; supported_ranges stays empty (nothing was received to populate it).
        if not self.check_response(
            response, previous_test_range, expected_success=False
        ):
            return False

        self.successful_messages += 1
        return True

    def test_concurrent_requests(self):
        self.requesting("Setting ranges concurrently...")

        request1 = SetRadarRange.Request()
        request1.header.frame_id = FRAME_ID
        request1.target_range = SET_RANGE_OK_SLOW

        request2 = SetRadarRange.Request()
        request2.header.frame_id = FRAME_ID
        request2.target_range = SET_RANGE_OK_FAST

        future1 = self.client.call_async(request1)
        future2 = self.client.call_async(request2)

        response1 = result_with_timeout(future1, timeout=TIMEOUT_SEC_LONG)
        self.got_response()

        response2 = result_with_timeout(future2)
        self.got_response()

        if not self.check_response(
            response1, request1.target_range, expected_success=True
        ):
            return False
        if not self.check_response(
            response2, request2.target_range, expected_success=True
        ):
            return False

        self.successful_messages += 1
        return True


def main(args=None):
    return test_framework.run(
        test_name=TEST_NAME,
        synthetic_data_dds_args=[
            "--set_radar_range",
            f"--frame_id={FRAME_ID}",
            f"--dds_domain_id={DDS_DOMAIN_ID}",
        ],
        node_type=TestNode,
        timeout_sec=TOTAL_TEST_TIMEOUT,
        rclpy_args=args,
        node_args=[["provizio_dds_domain_id", DDS_DOMAIN_ID]],
        frame_id_filters_success=[None, FRAME_ID],
        # Pointless testing for a failure on mismatched frame_id, as the service doesn't filter out received requests by design
    )


if __name__ == "__main__":
    main()
