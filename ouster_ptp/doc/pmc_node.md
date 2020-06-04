pmc_node
========

The `pmc_node` provides a ROS interface equivalent to the `linuxptp` management
client `pmc`. It is designed to serve several purposes. First, it can act as a
real-time monitor on your robot to detect if the LiDAR clock and the local
system clock have skewed too much, which could result in potentially dangerous
situations (i.e., if your algorithms are working on out-of-sync data, and
cannot detect that, bad things can happen). Second, it can be used to collect
data for off-line batch processing for tuning the PTP syncing parameters.

### Parameters
<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Default Value</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>~/boundary_hops</td>
    <td>int</td>
    <td>1</td>
    <td>
    The boundary hops value to be specified in the sent PMC messages. This
    should be 1 for Ouster LiDARs that are directly connected to this
    host. You may need to increase it if there are switches between your PTP
    master host (this machine) and the LiDARs.
    </td>
  </tr>
  <tr>
    <td>~/uds_prefix</td>
    <td>string</td>
    <td>/var/tmp/ouster_pmc.sock.</td>
    <td>
    File path prefix for our (client side) Unix Domain Socket
    for talking with the ptp4l server. Make sure you have permissions to
    write a file (UDS) to this directory. The software will append the
    process id (PID) to the end of this string.
    </td>
  </tr>
  <tr>
    <td>~/domain_number</td>
    <td>int</td>
    <td>0</td>
    <td>
    PTP domain number (should be the same as on the LiDARs)
    </td>
  </tr>
  <tr>
    <td>~/poll_timeout</td>
    <td>int</td>
    <td>100</td>
    <td>
    Timeout in millis to wait for ptp4l server when reading or writing data
    </td>
  </tr>
  <tr>
    <td>~/poll_period</td>
    <td>float</td>
    <td>1.0</td>
    <td>
    How often to poll the ptp4l server for data in seconds
    </td>
  </tr>
  <tr>
    <td>~/pmc_commands</td>
    <td>vector of strings</td>
    <td>["GET CURRENT_DATA_SET", "GET TIME_STATUS_NP"]</td>
    <td>
    The list of commands to send to the ptp4l server whose responses will be
    published on the ~ptp topic.
    </td>
  </tr>
  <tr>
    <td>~/pmc_config_hook</td>
    <td>string</td>
    <td>""</td>
    <td>
    A pmc command that can be sent to the PTP network at config time of the
    node lifecycle FSM.
    </td>
  </tr>
</table>


### Published Topics
<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Quality of Service</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>ptp</td>
    <td>std_msgs/msg/String</td>
    <td>System default QoS</td>
    <td>A JSON serialization of the ptp4l response to the ~/pmc_commands</td>
  </tr>
</table>


### Subscribed Topics

None.

### Advertised Services

None.

### Launch

To launch the node:

```
$ ros2 launch ouster_ptp pmc_managed.launch.py
```

### Other tools

#### ptp-dump

A command-line tool, `ptp-dump` is provided as a driver for accessing the PMC
data being published by the `pmc_node`. The help for the command is shown
below:

```
$ ros2 run ouster_ptp ptp-dump --help
usage: ptp-dump [-h] [-t TOPIC] [-p] [-1]

Dump the JSON being published by the pmc_node

optional arguments:
  -h, --help            show this help message and exit
  -t TOPIC, --topic TOPIC
                        Topic to subscribe to (default: /ouster_tools/ptp)
  -p, --pretty          Pretty print the json output (default: False)
  -1, --one             Dump one message and exit (default: False)
```

Assuming the `pmc_node` is running (see `launch` command above), exemplary
usage of the `ptp-dump` tool would be to get a single snapshot of the PTP
information being published by the `pmc_node`:

(NOTE: The output shown was acquired with an Ouster OS1-16 connected)

```
$ ros2 run ouster_ptp ptp-dump --pretty -1 2>/dev/null
[
  {
    "GET CURRENT_DATA_SET": {
      "ptp_msgs": [
        {
          "current_data_set": {
            "meanPathDelay": "0.0",
            "offsetFromMaster": "0.0",
            "stepsRemoved": "0"
          },
          "ptp_header": {
            "action": "RESPONSE",
            "sequenceId": "0",
            "sourcePortIdentity": "e86a64.fffe.f43c5b-0"
          },
          "recv_stamp_ns": "1586609496950442776"
        },
        {
          "current_data_set": {
            "meanPathDelay": "-8364626.0",
            "offsetFromMaster": "-34846416896.0",
            "stepsRemoved": "1"
          },
          "ptp_header": {
            "action": "RESPONSE",
            "sequenceId": "0",
            "sourcePortIdentity": "bc0fa7.fffe.000792-1"
          },
          "recv_stamp_ns": "1586609496950738649"
        }
      ],
      "send_stamp_ns": "1586609496949940625"
    }
  },
  {
    "GET TIME_STATUS_NP": {
      "ptp_msgs": [
        {
          "ptp_header": {
            "action": "RESPONSE",
            "sequenceId": "1",
            "sourcePortIdentity": "e86a64.fffe.f43c5b-0"
          },
          "recv_stamp_ns": "1586609497051603061",
          "time_status_np": {
            "cumulativeScaledRateOffset": "+0.000000000",
            "gmIdentity": "e86a64.fffe.f43c5b",
            "gmPresent": "false",
            "gmTimeBaseIndicator": "0",
            "ingress_time": "0",
            "lastGmPhaseChange": "0x0000'0000000000000000.0000",
            "master_offset": "0",
            "scaledLastGmPhaseChange": "0"
          }
        },
        {
          "ptp_header": {
            "action": "RESPONSE",
            "sequenceId": "1",
            "sourcePortIdentity": "bc0fa7.fffe.000792-1"
          },
          "recv_stamp_ns": "1586609497051905096",
          "time_status_np": {
            "cumulativeScaledRateOffset": "+0.000000000",
            "gmIdentity": "e86a64.fffe.f43c5b",
            "gmPresent": "true",
            "gmTimeBaseIndicator": "0",
            "ingress_time": "1586609497317820538",
            "lastGmPhaseChange": "0x0000'0000000000000000.0000",
            "master_offset": "-34846418270",
            "scaledLastGmPhaseChange": "0"
          }
        }
      ],
      "send_stamp_ns": "1586609497051215293"
    }
  }
]
```
