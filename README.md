ros2-ouster-tools
=================
Tools and utilities for working with Ouster LiDARs. This package does *not*
provide a driver but rather a set of tools and utilities to complement the
[ROS2 Ouster Driver](https://github.com/SteveMacenski/ros2_ouster_drivers).

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


What's Included?
================
This package provides the following utilities:

- [DHCP Server](ouster_tools/doc/dhcp_server.md): A wrapper around `dnsmasq`
  for giving the Ouster its IP address.
- [PTP tuning tools](ouster_tools/doc/ptp_tuning.md): Tools for
  synchronizing and monitoring the synchronization of the LiDAR clock and your
  local system clock.
- [ROS node](ouster_tools/doc/pmc_node.md): A ROS2 node for monitoring the PTP
  time sync between the Linux host and the LiDAR.


Building and Installing the Software
====================================

### Preliminaries

If you want to use our provided DHCP server wrapper, you will need `dnsmasq` on
your system:

```
$ sudo apt install dnsmasq-base
```

If you want to use our PTP time syncing tools (or follow along with some of our
docs on syncing the LiDAR to your system time) you will need the following:

```
$ sudo apt install linuxptp chrony ethtool
```

#### Cloning the sources

To clone this repository, please use the following command -- there is a
dependency on the `linuxptp` source as a submodule:

```
$ git clone --recurse-submodules git@github.com:Box-Robotics/ros2-ouster-tools.git
```

Optional, but **highly** recommended, we patch the `linuxptp` sources prior to
building. Assuming you have just cloned this repo:

```
$ cd ros2-ouster-tools/ouster_tools/
$ patch third_party/linuxptp/pmc_common.c pmc_common.c.patch
patching file third_party/linuxptp/pmc_common.c
Hunk #1 succeeded at 682 (offset 29 lines).
```

NOTE: This must be done in addition to installing the `linuxptp` binaries as
we did in the above section. We compile the `linuxptp` sources so we can build
the `pmc` client as a shared object file that we then leverage via our
`pmc_node` ROS node.


### Building with colcon

This package should be built and installed using `colcon`.

To build, assuming you have a colcon workspace set up:

```
$ colcon build --event-handlers console_cohesion+
```

### Post-installation setup

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
  <img src="ouster_tools/doc/figures/box-logo.png"/>
  <br/>
  Copyright &copy; 2020 Box Robotics, Inc.
</p>
