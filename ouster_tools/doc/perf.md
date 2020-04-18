Performance Analysis
====================

This document attempts to help quantify the performance of using Ouster LiDARs
within ROS2. We have several objectives in developing this framework and set of
tools. They are:

1. To quantitatively understand the performance characteristics that can be
   expected when using Ouster LiDARs in ROS2
2. To identify any performance bottlenecks with the intention of giving a
   directed way to address the issues
3. To work toward an *optimized* ROS2 Ouster LiDAR technology stack

# Introduction

At [Box Robotics](http://boxrobotics.ai), we are rebuilding the AGV perception
stack in HD (see our website to understand what that means). To do this, we
believe long-range 3D LiDAR, like those produced by Ouster, are a critical
enabling technology. The application of this technology extends beyond what we
are doing at Box Robotics. 3D LiDAR generalizes to almost all autonomous mobile
robotics systems. The performance analysis work presented herein will be of
particular interest to those working on *higher-speed* mobile robots.

The ROS2 stack for interfacing with Ouster LiDARs is *deep*. To that end, for
our purposes we will create a simplified model of the various layers in this
stack. Our model is shown in the block diagram below.

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
Linux kernel.

```
$ uname  -a
Linux jelly 5.3.0-46-lowlatency #38~18.04.1-Ubuntu SMP PREEMPT Tue Mar 31 04:59:24 UTC 2020 x86_64 x86_64 x86_64 GNU/Linux
```

The hardware is 64-bit Quad core i7 hyperthreaded @ 1.9 GHz (8
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
package.

**TODO:** Push a branch to baseline the code I used for measuring performance.

Since much of the analysis shown below relies upon consistent timing, my
computer and the LiDAR are time-synchronized using PTP as described
[here](./ptp_tuning.md). You can see the type of clock sync performance I am
getting by looking at a set of representative notebooks. My PTP performance is
shown [here](./notebooks/offset_from_master.ipynb) and the system clock sync
performance is shown [here](./notebooks/sysclk.ipynb). I'll assert that the
system clock between the LiDAR and my computer are sub-millisecond and at worst
single-digit millisecond synchronized.

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
[perf_node](./perf_node.md). The `perf_node` acts as our ROS2
application. Additionally, we need to run the ROS2 driver as the data source
for `perf_node`. The configuration of the driver may change from run-to-run in
this analysis. Before each run, we will always show the parameterization of the
ROS driver for the given data collection run.

## Test Case 1

In this first test case, I am going to look at both latency and jitter in
receiving data from the LiDAR into my ROS2 application (in this case,
`perf_node`). The measurement points in our dataflow model are shown below:

<div style="text-align:center">

![data_flow_model_test_cast_1](figures/perf-dataflow-model_test-case-1.png)

</div>

Referring to the figure above, the data log we generate with `perf_node` will
reflect (1) `msg_stamp` which is taken at the LiDAR using the LiDAR system time
and (2) `recv_stamp` which is taken in the `perf_node` using the ROS2 system
time on the local computer. Recall, the clocks are synced to sub-millisecond
accuracy.

The ROS2 driver parameterization for this run looks like:

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
    timestamp_mode: TIME_FROM_SYS_CLK
```
