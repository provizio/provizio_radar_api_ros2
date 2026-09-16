#!/bin/bash

# Copyright 2022 Provizio Ltd.
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

# Usage test.sh[ ros_version[ c_compiler[ provizio_api[ build_type[ ros_rmw[ static_analysis]]]]]]

set -eu

ROS_DISTRO=${1:-"jazzy"}
CC=${2:-"gcc"}
PROVIZIO_RADAR_API=${3:-"dds"}
CMAKE_BUILD_TYPE=${4:-"Release"}
ROS_RMW=${5:-"rmw_fastrtps_cpp"}
STATIC_ANALYSIS=${6:-"OFF"}
CONTAINER_TAG="provizio_radar_api_ros2_test:${ROS_DISTRO}"

cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)/../..

# shellcheck disable=SC2086
docker build \
    --build-arg ROS_DISTRO=${ROS_DISTRO} \
    --build-arg CC=${CC} \
    --build-arg PROVIZIO_RADAR_API=${PROVIZIO_RADAR_API} \
    --build-arg CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
    --build-arg STATIC_ANALYSIS=${STATIC_ANALYSIS} \
    --tag ${CONTAINER_TAG} .

# shellcheck disable=SC2086
(docker rm -f provizio_radar_api_ros2_test_${ROS_DISTRO} || true) > /dev/null 2>&1
# shellcheck disable=SC2086
# --shm-size=512m: the DDS request/response tests run several Fast-DDS participants (radar, node's
# contained DDS, ROS 2 RMW) at once, and provizio's large-sample QoS sizes each participant's shared-memory
# segment at ~34 MB. Docker's default 64 MB /dev/shm can't hold more than one, so the extra participants
# fail to register the SHM transport ("Failed to create segment ... / SHM Transport is not supported") and
# fall back to UDP; on Fast-DDS 3.x (kilted+) that breaks discovery and the tests never connect. A 512 MB
# /dev/shm lets the SHM transport work as intended. Mirrors provizio_dds' own ROS 2 CI.
docker run --shm-size=512m --name provizio_radar_api_ros2_test_${ROS_DISTRO} --entrypoint "/bin/bash" ${CONTAINER_TAG} -c "export RMW_IMPLEMENTATION=${ROS_RMW} && echo \"Testing via \${RMW_IMPLEMENTATION}...\" && source install/setup.bash && source test_env/bin/activate && python3 install/provizio_radar_api_ros2/lib/test_all.py"
