Performance Analysis
====================

This document attempts to help quantify the performance of using Ouster LiDARs
within ROS2. We have several objectives in developing this framework and set of
tools. They are:

1. To quantitatively understand the *current* performance characteristics that
   can be expected when using Ouster LiDARs in ROS2
2. To shine a light on any performance bottlenecks with the intention of giving
   a directed way to address the issues
3. To work toward an *optimized* ROS2 Ouster LiDAR technology stack

# Introduction

At [Box Robotics](http://boxrobotics.ai), we are rebuilding the AGV perception
stack in HD (see our website to understand what that means). To do this, we
believe long-range 3D LiDAR, like those produced by Ouster, are a critical
enabling technology. The application of this technology extends beyond what we
are doing at Box Robotics. 3D LiDAR generalizes to almost all autonomous mobile
robotics systems. The performance analysis work presented herein will be of
particular interest to those working on *higher-speed* mobile robots.

This is a living document. It is expected to be periodically updated as we
address any performance issues that this analsyis uncovers. The ROS2 stack for
interfacing with Ouster LiDARs is *deep*. To that end, for our purposes we will
create a simplified model of the various layers in this stack. Our model is
shown in the block diagram below.

<div style="text-align:center">

![data_flow_model](figures/perf-dataflow-model.png)

</div>


In order to tune performance, we need to understand how data flow through this
set of *modules* and what free variables we have to play with at each layer. As
we *put our probes* on this system we need to understand where in the stack the
measurement is being sampled. This allows us to focus our tuning efforts. In
this document, we will do our best to articulate our process.

# Preliminaries

To establish some context, my test setup and assumtions are as follows.

I'm using a Thinkpad T480 running Ubuntu 18.04 LTS with the 5.3.x low latency
Linux kernel. The hardware is 64-bit Quad core i7 hyperthreaded @ 1.9 GHz (8
virtual cores), 32 GB of RAM, NVIDIA GeForce MX150/PCIe/SSE2 discrete GPU, 1 TB
SSD.  The LiDAR under test is an Ouster OS1-16. The computer and LiDAR are
hard-wired directly over Gig Ethernet (no wifi, no switches, etc.) The specs on
my laptop are outlined in the figure below. In my setup, my computer is at
`192.168.0.92` and the LiDAR is at `192.168.0.254`.

<div style="text-align:center">

![laptop_arch](figures/laptop-lidar-arch.png)

</div>

The LiDAR is running firmware version `v1.13.0`:

```
$ curl -s http://192.168.0.254/api/v1/system/firmware | jq
{
  "fw": "ousteros-image-prod-aries-v1.13.0-20191105025459"
}
```

I'm using ROS2 Eloquent and unless explicitly specified below, the RMW
implementation in usage is
[Eclipse Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds)
(`rmw_cyclonedds_cpp`). Also, unless explicitly specified below, the Eclipse
Cyclone DDS configuration I am using is the stock default shipped with the
Eloquent ROS2 distribution. The ROS2 LiDAR driver in use is the one provided in
the [ros2_ouster_drivers](https://github.com/SteveMacenski/ros2_ouster_drivers)
package. For purposes of reproducibility, I've created a branch in my fork of
the driver to establish a baseline. That branch is
[here](https://github.com/tpanzarella/ros2_ouster_drivers/tree/tp-perf-analysis-baseline)
-- it is current to the upstream `eloquent-devel` branch as of April 14, 2020.

Since much of the analysis shown below relies upon consistent timing, my
computer and the LiDAR are time-synchronized using PTP as described
[here](./ptp_tuning.md).

```
$ curl -s http://192.168.0.254/api/v1/system/time/ | jq .sensor.timestamp.mode
"TIME_FROM_PTP_1588"
```

As of this writing, I am getting (roughly) +/- 200 usec of accuracy between my
laptop and the LiDAR over the PTP link. A representative sampling of the
`offsetFromMaster` over 5 minutes at 1 Hz is shown in the figure below.

<div style="text-align:center">

![ptp_plot](figures/offset_from_master-01.png)

</div>

**NOTE:** I have an active request in with Ouster to expose some of the PTP
servo tuning parameters so we can eliminate this oscillation. Bottom-line, we
can assume sub-millisecond accuracy between the computer and LiDAR clocks for
the data reported herein.

Additionally, I have `phc2sys` running on my laptop keeping my system clock
synchronized to the PTP master clock on my NIC. Per the last statistic in the
system log, the RMS between my system clock and the PTP time (on the NIC) is
207 nanoseconds. We will consider them synchronized. This is important to note
as the ROS time (in some of our data collection code) will come from the local
system clock.

Importantly related to `phc2sys`, I have the `currentUtcOffset` correctly set
to `37` seconds:

```
$ pmc -u -i /var/tmp/pmc.sock -b 1 "GET GRANDMASTER_SETTINGS_NP"
sending: GET GRANDMASTER_SETTINGS_NP
	e86a64.fffe.f43c5b-0 seq 0 RESPONSE MANAGEMENT GRANDMASTER_SETTINGS_NP
		clockClass              128
		clockAccuracy           0xfe
		offsetScaledLogVariance 0xffff
		currentUtcOffset        37
		leap61                  0
		leap59                  0
		currentUtcOffsetValid   1
		ptpTimescale            1
		timeTraceable           1
		frequencyTraceable      0
		timeSource              0xa0
	bc0fa7.fffe.000792-1 seq 0 RESPONSE MANAGEMENT GRANDMASTER_SETTINGS_NP
		clockClass              255
		clockAccuracy           0xfe
		offsetScaledLogVariance 0xffff
		currentUtcOffset        37
		leap61                  0
		leap59                  0
		currentUtcOffsetValid   0
		ptpTimescale            1
		timeTraceable           0
		frequencyTraceable      0
		timeSource              0xa0
```

This is important information we will use later in our analysis as the Ouster
stamps the data with the PTP time (TAI), so, we need to translate it on our end
to compare to our local system time (UTC). I will likely add an option to the
ROS2 driver that will perform a conversion to UTC when in PTP timestamp mode
(if you want it to).

Finally, since data from the Ouster are sent via UDP unicast, our socket
receive buffer sizes are relevant. On my machine they are set to 25 MB
(`26214400. / 2**20 -> 25.0`):

```
$ sysctl net.core.rmem_max
net.core.rmem_max = 26214400

$ sysctl net.core.rmem_default
net.core.rmem_default = 26214400
```

# Analysis

To collect data for our analysis, we will use the
[perf_node](./perf_node.md). Additionally, we need to run the ROS2 driver. My
driver configuration for this set of tests is as follows:

```
ouster_driver:
  ros__parameters:
    lidar_ip: 192.168.0.254
    computer_ip: 192.168.0.92
    lidar_mode: "2048x10"
    imu_port: 7503
    lidar_port: 7502
    sensor_frame: laser_sensor_frame
    laser_frame: laser_data_frame
    imu_frame: imu_data_frame
    use_system_default_qos: False
    timestamp_mode: TIME_FROM_PTP_1588
```

To start the ROS driver we run:

```
$ ros2 launch ros2_ouster os1_launch.py params_file:=${HOME}/.params2/os1.yaml
```



- [Jitter Analysis](#jitter-analysis)

## Jitter Analysis
