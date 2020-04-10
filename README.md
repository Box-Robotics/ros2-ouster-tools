ros2-ouster-tools
=================
Tools and utilities for working with Ouster LiDARs. This package does *not*
provide a driver but rather a set of tools and utilities to complement the
[ROS2 Ouster Driver](https://github.com/SteveMacenski/ros2_ouster_drivers).

**NOTE:** This README is very preliminary and will be updated shortly (don't
believe a word it says ... you have been warned)


Tested Configurations
=====================
<table>
  <tr>
    <th>ros2-ouster-tools version</th>
    <th>Linux distribution</th>
    <th>ROS distribution</th>
    <th>Ouster LiDAR</th>
    <th>Ouster Firmware</th>
  </tr>
  <tr>
    <td>0.0.0</td>
    <td>Ubuntu 18.04</td>
    <td>Eloquent</td>
    <td>OS1-16</td>
    <td>1.13.0</td>
  </tr>
</table>

NOTE: The above chart represents what [I](https://github.com/tpanzarella) have
been able to test and validate. It is likely that this package supports a
broader mix of hardware and software than is listed above.

Building and Installing the Software
====================================

### Bootstrap dependencies

This package contains some system dependencies and uses external tools that are
not (yet?) supported by `rosdep`. To that end, some system setup is required
prior to building.

*Call for help:* Any ROS packaging/devops specialist out there interested in
contributing to this project, we would love your help in getting the proper
system deps in `rosdep` and even better getting this project added to the ROS2
distro.

#### Required system deps (not in rosdep)


#### linuxptp sources

TODO: XXX: Moving this to be a git submodule, so this will all change

This package requires the source for `linuxptp` to be available. The build
system makes an assumption *a priori* about where they are
installed. Specifically, in a subdirectory of the this project's root folder
called `third_party/linuxptp`. To that end, from the root directory of this
project:

```
$ mkdir third_party
$ cd third_party
$ git clone git://git.code.sf.net/p/linuxptp/code linuxptp
```

You do not need to build any of the code contained in that project.

#### Optional tools

TODO: (In here list things like `jq` and `linuxptp` etc....

#### rosdep dependencies

TODO: XXX: Add the proper rosdep command here

### Building with colcon

This package should be built and installed using `colcon`.

To build:

```
$ colcon build --event-handlers console_cohesion+
$ colcon test
$ colcon test-result --all
```

To run, be sure to source in the setup file:

TODO: XXX: Add instructions on why we need the postinst script.
(no longer applies ... explain how to set up the ptp4l daemon)

```
$ . install/setup.bash
$ ros2 run ouster_tools postinst.sh
<SUDO HERE>
```

Usage
=====
This package provides the following utilities:
- [dhcp-server](ouster_tools/doc/dhcp_server.md): A wrapper around `dnsmasq`
  for giving the Ouster its IP address.

LICENSE
=======
Please see the file called [LICENSE](LICENSE)

<p align="center">
  <br/>
  <img src="ouster_tools/doc/figures/box-logo.png"/>
  <br/>
  Copyright &copy; 2020 Box Robotics, Inc.
</p>
