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

"""Wrapper around the dnsmasq dhcp server address."""

import argparse
import ipaddress as ipa
import os
import shutil
import sys

from ouster_tools.net import get_network_interfaces

DHCPD = 'dnsmasq'


def get_args():
    parser = argparse.ArgumentParser(
        description="Runs a local dhcp server using %s" % DHCPD,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    #
    # I want to try to make sensible default argumenets, somewhat biased to my
    # primary set-up. Others can always override these defaults by explicitly
    # passing parameters to the script.
    #
    # So we need three args:
    #
    # --iface: The primary network interface to bind to
    # --min-ip: The lowest address in the range we will serve up
    # --max-ip: The highest address in the range we will serve up
    #
    # On my laptop (where I am doing most of my Ouster development), I have
    # three interfaces: enp0s31f6, lo, wlp3s0 which are my hardwired, local
    # loopback and wireless interfaces respectively. In my dev setup, I plug
    # the Ouster directly into my hardwired interface (enp0s31f6). If I
    # lexographically sort these three interfaces, the hardwired one will be
    # the first element of that list, so, my hueristic for the default
    # interface will be the first one in that sorted list (and there will
    # always be at least one since we have the local loopback).
    #
    # I'll then take the IP that is bound to that interface and make the
    # assumption it is a Class C IPv4 address so a 24-bit subnet mask. I'll
    # then look at that network and serve up the highest valid IP address on
    # that network (I assume I do not have that address).
    #
    # This lets me serve up an IP to my Ouster in a way that almost makes it
    # act like the Ouster has a static IP address and I can set the parameters
    # on the ROS2 driver making this assumption.
    #
    iface_addrs = get_network_interfaces()
    ifaces = [iface[0] for iface in iface_addrs]
    iface_idxs_sorted = sorted(range(len(ifaces)), key=ifaces.__getitem__)
    default_idx = iface_idxs_sorted.index(0)

    iface, ipstr = iface_addrs[default_idx]
    net4 = (ipa.ip_interface("%s/24" % ipstr)).network
    min_max_ip = None
    for min_max_ip in net4.hosts():
        # This is how we get the last value from the generator
        # ... sucks (slow), but it works.
        pass

    parser.add_argument('-i', '--iface', required=False,
                        default=iface, type=str,
                        help="Listen on this network interface")
    parser.add_argument('--min-ip', required=False, default=min_max_ip,
                        type=str, help="Low end of the IP allocation range")
    parser.add_argument('--max-ip', required=False, default=min_max_ip,
                        type=str, help="High end of the IP allocation range")

    args = parser.parse_args(sys.argv[1:])
    return args


def main() -> int:
    args = get_args()

    exe = shutil.which(DHCPD)
    if exe is None:
        sys.stderr.write("Could not find '%s' on your ${PATH}" % DHCPD)
        return -1

    try:
        cmd = \
          "sudo {0} -C /dev/null -kd -F{1},{2} -i {3} --bind-dynamic".format(
            exe, args.min_ip, args.max_ip, args.iface)
        print(cmd)
        return os.system(cmd)

    except KeyboardInterrupt:
        pass

    finally:
        return 0
