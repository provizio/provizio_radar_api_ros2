#!/bin/bash

# Copyright 2026 Provizio Ltd.
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

# Runs the test suite under a watchdog that captures stacks if it wedges.
#
# A hung run is otherwise the least informative failure there is: the job sits until the runner's own
# limit kills it - six hours, at GitHub's default - and the log ends mid-test with nothing to say why.
# That happened to two ROS 2 Lyrical / rmw_fastrtps_cpp jobs, and the absence of evidence was what made
# the cause impossible to pin down afterwards. So when the suite overruns, dump what every process is
# doing - Python frames and native frames both - and only then kill it. The job still fails, but it
# fails in minutes with the stacks that explain it.

set -eu -o pipefail

WORKSPACE_DIR=${WORKSPACE_DIR:-/opt/test_workspace}
# Comfortably longer than the suite takes anywhere, including the slower self-hosted runner, and well
# inside the job's own timeout-minutes so this fires first and leaves evidence.
TEST_WATCHDOG_SEC=${TEST_WATCHDOG_SEC:-2700}

cd "${WORKSPACE_DIR}"

# The ROS 2 / colcon setup scripts read variables they do not set (COLCON_TRACE,
# AMENT_TRACE_SETUP_FILES), so -u has to come off while they are sourced or they abort the run.
set +u
# shellcheck disable=SC1091
source install/setup.bash
# shellcheck disable=SC1091
source test_env/bin/activate
set -u

dump_stacks() {
    echo "==============================================================================="
    echo "The test suite has not finished within ${TEST_WATCHDOG_SEC} sec, so it is wedged."
    echo "Dumping every process's stacks before killing it, so this failure leaves evidence."
    echo "==============================================================================="

    echo "----- process tree -----"
    ps -eo pid,ppid,stat,etimes,wchan:24,args --sort=pid 2>/dev/null | cut -c1-200 || true

    echo "----- /dev/shm (Fast-DDS segments outlive processes that died uncleanly) -----"
    ls -la /dev/shm 2>/dev/null | head -40 || true

    for pid in $(pgrep -f "test_all\.py|test_[a-z_0-9]*\.py|synthetic_data|provizio_radar|lifecycle set" 2>/dev/null || true); do
        [ "${pid}" = "$$" ] && continue
        [ -d "/proc/${pid}" ] || continue
        echo "===== pid ${pid}: $(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null | cut -c1-140) ====="

        echo "--- Python frames (py-spy) ---"
        timeout 60 py-spy dump --pid "${pid}" 2>&1 | head -50 || true

        echo "--- native frames, all threads (gdb) ---"
        timeout 120 gdb -p "${pid}" -batch -ex "set pagination off" -ex "thread apply all bt 25" 2>&1 \
            | grep -vE "^\[New LWP|^Reading symbols|^warning:|^\[Thread debugging" \
            | cut -c1-200 | head -250 || true
    done

    echo "===== end of stack dump ====="
}

python3 install/provizio_radar_api_ros2/lib/test_all.py &
tests=$!

(
    sleep "${TEST_WATCHDOG_SEC}"
    if kill -0 "${tests}" 2>/dev/null; then
        dump_stacks
        kill -9 "${tests}" 2>/dev/null || true
        pkill -9 -f "provizio_radar|synthetic_data" 2>/dev/null || true
    fi
) &
watchdog=$!

set +e
wait "${tests}"
status=$?
set -e

kill "${watchdog}" 2>/dev/null || true
wait "${watchdog}" 2>/dev/null || true

exit "${status}"
