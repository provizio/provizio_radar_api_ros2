#!/bin/bash

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

set -e

# List of all excluded files
EXCLUDED=("./python/gps_utils.py")

cd "$(cd "$(dirname "$0")" && pwd -P)"/../..

# Use as check_license_header file_to_check comment_mark
check_license_header() {
    OUTPUT_PREFIX="Checking license header:"
    FAILURE_PREFIX="License header is missing or invalid in"
    FILE=$1
    COMMENT_MARK=$2

    for EX in "${EXCLUDED[@]}"; do
        if [[ "${EX}" == "${FILE}" ]]; then
            # Ignore the check for this file
            return
        fi
    done

    echo "${OUTPUT_PREFIX} ${FILE}"
    grep -q "${COMMENT_MARK} Copyright 2[0-9][0-9][0-9] Provizio Ltd." "${FILE}" || (echo "${FAILURE_PREFIX} ${FILE}"; exit 1)
    grep -q "${COMMENT_MARK} Licensed under the Apache License, Version 2.0 (the \"License\");" "${FILE}" || (echo "${FAILURE_PREFIX} ${FILE}"; exit 1)
}

# .c/.cpp/.h/.hpp
for FILE in $(find . -not \( -path ./build -prune \) -name '*.c' -or -name '*.cpp' -or -name '*.h' -or -name '*.hpp'); do
    check_license_header "$FILE" "//"
done

# .sh/.py
for FILE in $(find . -not \( -path ./build -prune \) -name '*.sh' -or -name '*.py'); do
    check_license_header "$FILE" "#"
done

# .bat
for FILE in $(find . -not \( -path ./build -prune \) -name '*.bat'); do
    check_license_header "$FILE" "::"
done

echo "Licence headers OK"
