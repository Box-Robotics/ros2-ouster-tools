Ouster LiDAR Performance in ROS2
================================

This package provides an evolving set of tools to help quantify the performance
of using Ouster LiDARs within ROS2. We have several objectives in developing
this framework and set of tools. They are:

1. To quantitatively understand the performance characteristics that can be
   expected when using Ouster LiDARs in ROS2
2. To identify any performance bottlenecks with the intention of giving a
   directed way to address the issues
3. To work toward an *optimized* ROS2 Ouster LiDAR technology stack


What has been done so far?
--------------------------

1. [Eloquent Performance Analysis](./doc/eloquent_perf.md): Significant work
   was conducted to get a baseline sense of the jitter and latency we can
   expect when using Ouster LiDARs with ROS2. This was done in ROS2 eloquent.
2. [Foxy Performance Analysis](): A new baseline against ROS2 Foxy needs to be
   conducted. This will pick up where the [Eloquent
   analysis](./doc/eloquent_perf.md) left off. (In progress)
3. [Data organization](./notebooks/Ouster_Data_Layout.ipynb): In
   [this](https://github.com/SteveMacenski/ros2_ouster_drivers/pull/46) PR
   conversation, it was alluded to that some performance hit is being taken
   by how the LiDAR data are organized in the memory buffers of the
   driver. This notebook articulates exactly how those data are laid out in
   memory and proposes a (simple) reshaping filter that could be applied in
   the driver to save all consuming applications from having to do this. The
   hope is that it will lead to better overall performance and provide a more
   inuitive interface to the data. This notebook serves as the motivation for
   some work that will be conducted in the driver code itself.
