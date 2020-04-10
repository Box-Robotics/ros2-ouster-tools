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

"""ROS nodes related to PTP"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String

class PTPDump(Node):
    """Subscriber to the /ouster_tools/ptp PMC topic"""

    def __init__(self, topic="/ouster_tools/ptp", cb=None):
        """
        ctor creates the node and subscribes to the passed in topic

        Parameters
        ----------
        topic : string
            The topic to subscribe to

        cb : callable
            Callback function to call with the json string as its only
            argument. The json is extracted from the latest ROS message
            recieved on the `topic`.

        """
        super().__init__("ptp_dump")
        self.topic_ = topic
        self.cb_ = cb

        if self.cb_ is None:
            self.cb_ = lambda json_str: print(json_str)

        self.sub_ = self.create_subscription(
            String, self.topic_,
            lambda msg: self.cb_(msg.data), qos_profile_system_default)
