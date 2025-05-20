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

import sys
import test_radar_pc
import test_radar_pc_snr_filtered
import test_set_radar_range


def main(args=None):
    if (
        True
        and test_radar_pc.main(args) == 0
        and test_radar_pc_snr_filtered.main(False, args) == 0
        and test_radar_pc_snr_filtered.main(True, args) == 0
        and test_set_radar_range.main(args) == 0
    ):
        print("All tests pass!")
        return 0

    return 1


if __name__ == "__main__":
    sys.exit(main())
