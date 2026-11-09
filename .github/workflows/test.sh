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
# Where the ros:<distro> base image comes from. Defaults to Docker Hub, so a local build needs no
# credentials. CI overrides it with this org's GHCR mirror and sets a fallback, because Docker Hub
# limits anonymous pulls per source IP and this matrix cannot stay under that limit.
BASE_IMAGE_REPO=${BASE_IMAGE_REPO:-"ros"}
# Optional second registry, tried only if the first fails for a registry reason. Deliberately empty
# by default: a fallback nobody asked for would hide a broken primary.
BASE_IMAGE_FALLBACK_REPO=${BASE_IMAGE_FALLBACK_REPO:-""}

cd $(cd -P -- "$(dirname -- "$0")" && pwd -P)/../..

# Pulling the ros:<distro> base image can fail before any of this project's code is touched, at
# "load metadata for docker.io/library/ros:<distro>". Two different causes look similar and must NOT
# be treated the same:
#
#   - Docker Hub throttling. Anonymous pulls are limited per source IP, and this matrix fires well
#     over a hundred jobs at once - from GitHub-hosted runners whose egress IPs are shared with the
#     rest of the world, and from one self-hosted runner with a fixed IP. Retrying that is worse than
#     useless: the window is hours long, so every retry only adds load. Those are failed immediately
#     with a message saying what actually needs fixing.
#   - A genuine transient network fault. Worth exactly one more try.
#
# A build failure that is neither is reported straight away - retrying it would burn CI time across
# the whole matrix for no reason.
BUILD_LOG=$(mktemp)
trap 'rm -f "${BUILD_LOG}"' EXIT
BUILD_ATTEMPTS=2

# Builds against one base image registry. Returns:
#   0 - built
#   1 - could not get the base image from this registry (another registry is worth trying)
#   2 - the build itself failed (trying anywhere else would fail identically)
build_with_base_repo() {
    local base_repo="$1"
    local attempt
    local build_status

    for attempt in $(seq 1 ${BUILD_ATTEMPTS}); do
        set +e
        # shellcheck disable=SC2086
        docker build \
            --build-arg ROS_DISTRO=${ROS_DISTRO} \
            --build-arg CC=${CC} \
            --build-arg PROVIZIO_RADAR_API=${PROVIZIO_RADAR_API} \
            --build-arg CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
            --build-arg STATIC_ANALYSIS=${STATIC_ANALYSIS} \
            --build-arg BASE_IMAGE_REPO="${base_repo}" \
            --tag ${CONTAINER_TAG} . 2>&1 | tee "${BUILD_LOG}"
        build_status=${PIPESTATUS[0]}
        set -e

        if [ "${build_status}" -eq 0 ]; then
            return 0
        fi

        # Throttling is not a transient fault. The window is hours long, so retrying only adds load;
        # say what actually needs fixing instead. Worth distinguishing even though a fallback follows,
        # because being rate limited on the primary is itself the thing worth knowing.
        if grep -qiE "toomanyrequests|too many requests|429|pull rate limit|rate limit exceeded" "${BUILD_LOG}"; then
            echo "${base_repo}: the registry is rate limiting this runner. Anonymous pulls are counted"
            echo "per source IP, which on a hosted runner is shared with everyone else, and the limit"
            echo "is applied over hours - so this will not clear by trying again."
            return 1
        fi

        # Deliberately specific. A bare "not found" or "denied" would also match a genuine build
        # failure - CMake's "Could NOT find", apt's "Unable to locate", a shell "command not found" -
        # and send every such job through a pointless fallback and a second full build. A missing or
        # unreadable image already reports "failed to resolve source metadata" or "failed to
        # authorize", so the broad words buy nothing and cost a rebuild across the whole matrix.
        if ! grep -qE "failed to authorize|failed to fetch oauth token|failed to resolve source metadata|connection reset by peer|TLS handshake timeout|i/o timeout|net/http: request canceled|502 Bad Gateway|503 Service Unavailable" "${BUILD_LOG}"; then
            echo "${base_repo}: the build failed for a reason that is not the registry - not retrying,"
            echo "and not trying another registry, as it would fail in exactly the same way."
            return 2
        fi

        if [ "${attempt}" -eq "${BUILD_ATTEMPTS}" ]; then
            echo "${base_repo}: could not get the base image after ${BUILD_ATTEMPTS} attempts."
            return 1
        fi

        echo "${base_repo}: registry failure fetching the base image (attempt ${attempt}/${BUILD_ATTEMPTS}); retrying once in 60s..."
        sleep 60
    done

    return 1
}

# record_base_image_source prints the line that proves which registry was actually used, and repeats
# it in the CI step summary. A fallback that quietly papered over a misconfigured primary would be
# worse than having no fallback at all: every job would go on passing while the mirror it is meant to
# exercise sat broken or empty, and nobody would find out until Docker Hub throttled the matrix again.
record_base_image_source() {
    # Printed, not written to the CI step summary: one summary entry per job is 160 of them per run,
    # which drowns the summary view for something that is only interesting when it says FALLBACK -
    # and that case already raises a workflow warning below, which is visible on the job itself.
    echo "PROVIZIO_BASE_IMAGE_SOURCE=$1 $2"
}

if build_with_base_repo "${BASE_IMAGE_REPO}"; then
    record_base_image_source "${BASE_IMAGE_REPO}" "PRIMARY"
else
    build_result=$?

    if [ "${build_result}" -eq 2 ]; then
        exit 1
    fi

    if [ -z "${BASE_IMAGE_FALLBACK_REPO}" ] || [ "${BASE_IMAGE_FALLBACK_REPO}" = "${BASE_IMAGE_REPO}" ]; then
        record_base_image_source "none" "NO_FALLBACK_CONFIGURED"
        exit 1
    fi

    echo "==============================================================================="
    echo "WARNING: could not pull the base image from ${BASE_IMAGE_REPO}."
    echo "Falling back to ${BASE_IMAGE_FALLBACK_REPO}. The build may still pass, but the"
    echo "primary registry is not working and that is the thing to fix."
    echo "==============================================================================="
    if [ -n "${GITHUB_ACTIONS:-}" ]; then
        echo "::warning title=Base image fallback used::Could not pull from ${BASE_IMAGE_REPO}; fell back to ${BASE_IMAGE_FALLBACK_REPO}"
    fi

    if build_with_base_repo "${BASE_IMAGE_FALLBACK_REPO}"; then
        record_base_image_source "${BASE_IMAGE_FALLBACK_REPO}" "FALLBACK"
    else
        record_base_image_source "none" "BOTH_REGISTRIES_FAILED"
        exit 1
    fi
fi

# shellcheck disable=SC2086
(docker rm -f provizio_radar_api_ros2_test_${ROS_DISTRO} || true) > /dev/null 2>&1
# shellcheck disable=SC2086
# --shm-size=512m: the DDS request/response tests run several Fast-DDS participants (radar, node's
# contained DDS, ROS 2 RMW) at once, and provizio's large-sample QoS sizes each participant's shared-memory
# segment at ~34 MB. Docker's default 64 MB /dev/shm can't hold more than one, so the extra participants
# fail to register the SHM transport ("Failed to create segment ... / SHM Transport is not supported") and
# fall back to UDP; on Fast-DDS 3.x (kilted+) that breaks discovery and the tests never connect. A 512 MB
# /dev/shm lets the SHM transport work as intended. Mirrors provizio_dds' own ROS 2 CI.
docker run --shm-size=512m --name provizio_radar_api_ros2_test_${ROS_DISTRO} --entrypoint "/bin/bash" ${CONTAINER_TAG} -c "export RMW_IMPLEMENTATION=${ROS_RMW} && echo \"Testing via \${RMW_IMPLEMENTATION}...\" && src/provizio_radar_api_ros2/.github/workflows/run_tests.sh"
