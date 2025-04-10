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

set -eu

CC=${CC:-"gcc"}
STATIC_ANALYSIS=${1:-"OFF"}
TEST_ENVIRONMENT=${2:-""} # Don't install/configure test dependencies and environment by default
INSTALL_ROS=${3:-""} # Don't install ROS by default
TESTS_PROVIZIO_DDS_VERSION="feature/APT-7834-PIP-Custom-CMake-Arguments"

# Update apt cache
apt update

apt install -y --no-install-recommends ca-certificates build-essential cmake git libssl-dev unzip

# Install GCC/clang
if [[ "${CC}" == "gcc" ]]; then
    apt install -y --no-install-recommends gcc g++
else 
    apt install -y --no-install-recommends clang
fi

if [[ "${STATIC_ANALYSIS}" != "OFF" ]]; then
    # Install cppcheck, clang-format and clang-tidy (and clang for proper clang-tidy checks)
    apt install -y --no-install-recommends clang clang-format clang-tidy cppcheck
fi

if [[ "${TEST_ENVIRONMENT}" != "" ]]; then
    echo "Configuring test environment at: ${TEST_ENVIRONMENT}..."
    apt install -y --no-install-recommends python3 python3-pip python3-venv
    python3 -m venv --system-site-packages "${TEST_ENVIRONMENT}"
    touch "${TEST_ENVIRONMENT}/COLCON_IGNORE"
    source "${TEST_ENVIRONMENT}/bin/activate"

    # provizio_dds
    apt install -y --no-install-recommends curl
    curl -s https://raw.githubusercontent.com/provizio/provizio_dds/${TESTS_PROVIZIO_DDS_VERSION}/install_dependencies.sh | bash -s ON
    python3 -m pip install -v git+https://github.com/provizio/provizio_dds.git@${TESTS_PROVIZIO_DDS_VERSION}
fi

if [[ "${INSTALL_ROS}" != "" ]]; then
    echo "Installing ROS2: ${INSTALL_ROS}..."

    apt install -y --no-install-recommends locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    apt install -y --no-install-recommends software-properties-common
    add-apt-repository universe
    apt install -y --no-install-recommends curl
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    apt update
    apt install -y --no-install-recommends "ros-${INSTALL_ROS}-desktop"
fi

echo "Done installing build dependencies!"
