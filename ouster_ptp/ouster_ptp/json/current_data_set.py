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

import json
import pandas as pd

def df_from_file(fname):
    """
    Reads a log file assumed to be generated from `ptp-dump` where each line
    contains a single JSON return from "GET CURRENT_DATA_SET". It returns a
    pandas dataframe encoding of those data.
    """
    dset = {
             "send_stamp": [],
             "recv_stamp": [],
             "id": [],
             "mean_path_delay": [],
             "offset_from_master": [],
             "steps_removed": []
           }

    with open(fname) as f:
        line = f.readline()
        while line:
            obj_list = json.loads(line)
            for obj in obj_list:
                current_dset = obj["GET CURRENT_DATA_SET"]
                send_stamp = current_dset["send_stamp_ns"]
                for msg in current_dset["ptp_msgs"]:
                    dset["send_stamp"].append(int(send_stamp))
                    dset["recv_stamp"].append(int(msg["recv_stamp_ns"]))
                    dset["id"].append(msg["ptp_header"]["sourcePortIdentity"])
                    dset["mean_path_delay"].append(
                        int(float(msg["current_data_set"]["meanPathDelay"])))
                    dset["offset_from_master"].append(
                        int(float(msg["current_data_set"]["offsetFromMaster"])))
                    dset["steps_removed"].append(
                        int(msg["current_data_set"]["stepsRemoved"]))

            line = f.readline()

    return pd.DataFrame.from_dict(dset)

# Quick little test if we execute the module directly with:
# $ python3 ./current_data_set.py
# (assumes we are in this directory)
if __name__ == '__main__':
    df = df_from_file("../../doc/notebooks/ptp_json_log-00.txt")
    print(df)
