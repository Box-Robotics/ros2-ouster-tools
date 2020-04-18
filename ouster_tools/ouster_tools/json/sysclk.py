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
import numpy as np

def df_from_file(fname):
    """
    Reads a log file assumed to be generated from `sys-time` where each line
    contains a single JSON return from logging the local and LiDAR system clock
    times. It returns a pandas dataframe encoding of those data.
    """
    dset = {
             "system_time": [],
             "lidar_time": []
           }

    with open(fname) as f:
        line = f.readline()
        while line:
            d = json.loads(line)
            dset["system_time"].append(np.float64(d["system_time"]))
            dset["lidar_time"].append(np.float64(d["lidar_time"]))

            line = f.readline()

    return pd.DataFrame.from_dict(dset)

# test
if __name__ == '__main__':
    df = df_from_file("../../doc/notebooks/sysclk-00.txt")
    print(df)
