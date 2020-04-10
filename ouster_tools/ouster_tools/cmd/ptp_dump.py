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
"""Driver for for the PTPDump node"""

import argparse
import json
import sys

import rclpy
from ouster_tools.ros_node.ptp import PTPDump

def get_args():
    parser = argparse.ArgumentParser(
        description="Dump the JSON being published by the pmc_node",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("-t", "--topic", required=False,
                        default="/ouster_tools/ptp", type=str,
                        help="Topic to subscribe to")
    parser.add_argument("-p", "--pretty", required=False,
                        action='store_true', default=False,
                        help="Pretty print the json output")
    parser.add_argument("-1", "--one", required=False,
                        action='store_true', default=False,
                        help="Dump one message and exit")

    args, _ = parser.parse_known_args(sys.argv[1:])
    return args

def main() -> int:
    args = get_args()
    count = 0

    def cb(json_str):
        nonlocal count
        count += 1
        if args.pretty:
            print(json.dumps(json.loads(json_str),
                        sort_keys=True, indent=2, separators=(',', ': ')),
                  flush=True)
        else:
            print(json_str, flush=True)

    try:
        rclpy.init(args=sys.argv[1:])
        node = PTPDump(topic=args.topic, cb=cb)
        while rclpy.ok():
            if args.one and count > 0:
                break
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        pass

    except:
        print(sys.exc_info()[0], ": ", sys.exc_info()[1])
        raise

    finally:
        node.destroy_node()
        rclpy.shutdown()
        return 0
