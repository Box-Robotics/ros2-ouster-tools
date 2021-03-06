ros2-ouster-tools
=================
Tools and utilities for working with Ouster LiDARs. This set of packages does
*not* provide a driver but rather tools and utilities to complement
the [ROS2 Ouster Driver](https://github.com/SteveMacenski/ros2_ouster_drivers).

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
    <td>0.1.0</td>
    <td>Ubuntu 18.04</td>
    <td>Eloquent</td>
    <td>OS1-16</td>
    <td>1.13.0</td>
  </tr>
  <tr>
    <td>0.2.0</td>
    <td>Ubuntu 20.04</td>
    <td>Foxy</td>
    <td>OS1-16</td>
    <td>1.13.0</td>
  </tr>
</table>

What's Included?
================

### Packages

- [ouster_dhcp](ouster_dhcp/): Manage IP address allocation to Ouster LiDARs on
  Linux.
- [ouster_h5](ouster_h5/): Record Ouster LiDAR data to HDF5. The intention is
  to support analysis. It is not intended to be a replacement for `rosbag`.
- [ouster_perf](ouster_perf/): Performance analysis of using Ouster LiDARs
  within ROS2.
- [ouster_ptp](ouster_ptp/): PTP clock synchronization tools between ROS2 Linux
  hosts and Ouster LiDARs.

This is (currently) no inter-dependencies between the above packages. If there
is a piece of functionality you do not want, you can skip building a particular
package by placing a `COLCON_IGNORE` file in the top-level directory of that
package prior to building (see build instructions below).

Building and Installing the Software
====================================

### Preliminaries

#### ouster_dhcp

If you want to use our provided DHCP server wrapper, you will need `dnsmasq` on
your system:

```
$ sudo apt install dnsmasq-base
```

#### ouster_h5

If you wish to use the HDF5 recording capabilities of this toolbox, you will
need to first install [h5_bridge](https://github.com/Box-Robotics/ros2-h5_bridge).


#### ouster_ptp

If you want to use our PTP time syncing tools (or follow along with some of our
docs on syncing the LiDAR to your system time) you will need the following:

```
$ sudo apt install linuxptp chrony ethtool
```

### Cloning the sources

To clone this repository, please use the following command -- there is a
dependency on the `linuxptp` source as a submodule:

```
$ git clone --recurse-submodules git@github.com:Box-Robotics/ros2-ouster-tools.git
```

Optional, but **highly** recommended, we patch the `linuxptp` sources prior to
building. Assuming you have just cloned this repo:

```
$ cd ros2-ouster-tools/ouster_ptp/
$ patch third_party/linuxptp/pmc_common.c pmc_common.c.patch
patching file third_party/linuxptp/pmc_common.c
Hunk #1 succeeded at 682 (offset 29 lines).
```

NOTE: This must be done in addition to installing the `linuxptp` binaries as
we did in the above section. We compile the `linuxptp` sources so we can build
the `pmc` client as a shared object file that we then leverage via our
`pmc_node` ROS node.


### Building with colcon

We use the following shell script to build and install the software via
`colcon`.

```
#!/bin/bash

colcon build \
  --install-base "${BOX_ROS2}/ouster_tools" \
  --cmake-args " -DCMAKE_BUILD_TYPE=Release" \
  --event-handlers console_cohesion+
```

**NOTE:** The `${BOX_ROS2}` environment variable is set prior to running this
script and points to the root directory of where we install ROS2 packages on
our system outside of a workspace. Modify as you see fit.


### Post-installation setup

**NOTE:** This only applies if you have built the `ouster_ptp` package.

The `pmc_node` provided with this package (for monitoring PTP time sync)
assumes it is running on a machine that has a `ptp4l` daemon running
locally. It communicates with `ptp4l` via a Unix Domain Socket (UDS). So, there
are some permissions related items we need to take care of to ensure this will
all work properly without having to run the ROS node as `root` or (even more of
a pain) as setuid root.

Stop the `ptp4l` daemon:

```
$ sudo systemctl stop ptp4l.service
```

Create a new Linux group called `ptp`:

```
$ sudo groupadd -r ptp
```

Now edit `/etc/systemd/system/ptp4l.server.d/override.conf` so that `ptp4l`
will run as this new group. Mine looks like:

```
[Service]
ExecStart=
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf
Group=ptp
```

Have `systemd` reload the configuration and restart `ptp4l`:

```
$ sudo systemctl daemon-reload
$ sudo systemctl start ptp4l.service
```

We can now check the `ptp4l` UDS file permissions. Group ownership should
belong to the `ptp` group we just created:

```
$ ls -l /var/run/ptp4l
srw-rw---- 1 root ptp 0 Apr  9 12:25 /var/run/ptp4l
```

Now, you need to add yourself (or whoever the ROS node will run as) to the
`ptp` group as well. For exemplary purposes, we will use my username, `tpanzarella`.

```
$ sudo usermod -a -G ptp tpanzarella
```

You will now need to log out and log back in for this to take effect. Once you
log back in, you can validate with:

```
$ groups
tpanzarella adm dialout cdrom sudo dip plugdev lpadmin sambashare wireshark ptp
```

Finally, you can check that the permissions are all setup properly by using the
`linuxptp` supplied `pmc` commandline tool. It does not matter if the LiDAR is
plugged in or not. You should should see output similar to what I show below:

```
$ pmc -u -i /var/tmp/pmc.sock -b 1 "GET TIME_STATUS_NP"
sending: GET TIME_STATUS_NP
	e86a64.fffe.f43c5b-0 seq 0 RESPONSE MANAGEMENT TIME_STATUS_NP
		master_offset              0
		ingress_time               0
		cumulativeScaledRateOffset +0.000000000
		scaledLastGmPhaseChange    0
		gmTimeBaseIndicator        0
		lastGmPhaseChange          0x0000'0000000000000000.0000
		gmPresent                  false
		gmIdentity                 e86a64.fffe.f43c5b

```

The point here is that you can run `pmc` **without** having to `sudo`.


LICENSE
=======
Please see the file called [LICENSE](LICENSE)

<p align="center">
  <br/>
  <img src="ouster_perf/doc/figures/box-logo.png"/>
  <br/>
  Copyright &copy; 2020 Box Robotics, Inc.
</p>
