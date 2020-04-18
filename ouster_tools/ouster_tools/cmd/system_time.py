# Copyright 2020 Box Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""Script that lets us compare our local system time to the LiDAR"""

import argparse
import json
import sys
import time
import urllib.request

def get_args():
    parser = argparse.ArgumentParser(
        description="Compare the local system time to the LiDAR system time",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--ip', required=False,  default="192.168.0.254",
                        type=str, help="LiDAR IP address")
    parser.add_argument('--hz', required=False, default=1,
                        type=int, help="Frequency to poll timestamps")
    parser.add_argument("-1", "--one", required=False,
                        action='store_true', default=False,
                        help="Poll once and exit")


    args = parser.parse_args(sys.argv[1:])
    return args

def main() -> int:
    args = get_args()
    sleep_time = 1.
    if args.hz > 1:
        sleep_time = 1./args.hz

    try:
        req = urllib.request.Request(
          url="http://%s/api/v1/system/time/system" % args.ip,
          method="GET")

        count = 0

        while True:
            with urllib.request.urlopen(req) as R:
                sys_time = time.time()
                j_resp = json.loads(str(R.read().decode("utf-8")))
                print(json.dumps(
                        {"system_time": "{:.9f}".format(sys_time),
                         "lidar_time": "{:.9f}".format(j_resp["realtime"])},
                      separators=(',', ':')),
                      flush=True)
                count += 1

            if args.one and count > 0:
                break

            time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    finally:
        return 0
