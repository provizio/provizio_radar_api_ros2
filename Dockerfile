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

# This Dockerfile is used to test the project, particularly in CI

ARG ROS_DISTRO=jazzy
ARG CC=gcc
ARG STATIC_ANALYSIS=OFF

FROM ros:${ROS_DISTRO}

ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE_DIR=/opt/test_workspace
ENV PROJECT_DIR=${WORKSPACE_DIR}/src/provizio_radar_api_ros2

## Install dependencies (before copying the rest source for optimization)
COPY .github/workflows/install_dependencies.sh /tmp
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && /tmp/install_dependencies.sh "${STATIC_ANALYSIS}" "${WORKSPACE_DIR}/test_env" && rm -rf /var/lib/apt/lists/*'

## Installing missing item in Humble (it's present in other ROS distros)
RUN if [ "${ROS_DISTRO}" = "humble" ]; then apt update && apt install -y ros-humble-sensor-msgs-py && rm -rf /var/lib/apt/lists/*; fi

## Disable "detected dubious ownership in repository" git issue due to running git commands in Docker
RUN git config --global --add safe.directory '*'

## Copy the entire project source
COPY . ${PROJECT_DIR}

## In the workspace directory
WORKDIR ${WORKSPACE_DIR}

## Build
ARG CMAKE_BUILD_TYPE=Release
ARG PROVIZIO_RADAR_API=dds
RUN /bin/bash -c "source ${PROJECT_DIR}/.github/workflows/export_cxx.sh \
                && source /opt/ros/${ROS_DISTRO}/setup.bash \
                && colcon build \
                    --event-handlers console_direct+ \
                    --symlink-install \
                    --cmake-args \
                        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
                        -DPROVIZIO_RADAR_API=${PROVIZIO_RADAR_API} \
                        -DSTATIC_ANALYSIS=${STATIC_ANALYSIS} \
                        -DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=OFF \
                        -DBUILD_SIMPLE_NODE=ON \
                        -DBUILD_LIFECYCLE_NODE=ON \
                        -DBUILD_TESTING=ON"

## Default entry point: run the unmanaged node with default configuration
ENTRYPOINT [ "/bin/bash", "-c", "source install/setup.bash && ros2 run provizio_radar_api_ros2 provizio_radar_node" ]
