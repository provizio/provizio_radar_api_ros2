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

# Usage test.sh[ ros_version[ c_compiler[ provizio_api[ build_type[ static_analysis]]]]]

set -eu

ROS_DISTRO=${1:-"jazzy"}
CC=${2:-"gcc"}
PROVIZIO_RADAR_API=${3:-"dds"}
CMAKE_BUILD_TYPE=${4:-"Release"}
STATIC_ANALYSIS=${5:-"OFF"}
CONTAINER_TAG="provizio_radar_api_ros2_test:${ROS_DISTRO}"

cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)/../..

# shellcheck disable=SC2086
docker build \
    --build-arg ROS_DISTRO=${ROS_DISTRO} \
    --build-arg CC=${CC} \
    --build-arg PROVIZIO_RADAR_API=${PROVIZIO_RADAR_API} \
    --build-arg CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
    --build-arg STATIC_ANALYSIS=${STATIC_ANALYSIS} \
    --progress=plain \
    --tag ${CONTAINER_TAG} .

# shellcheck disable=SC2086
docker rm -f provizio_radar_api_ros2_test_${ROS_DISTRO}
# shellcheck disable=SC2086
docker run --name provizio_radar_api_ros2_test_${ROS_DISTRO} --entrypoint "/bin/bash" ${CONTAINER_TAG} -c "source install/setup.bash && source test_env/bin/activate && python3 install/provizio_radar_api_ros2/lib/test_all.py"
