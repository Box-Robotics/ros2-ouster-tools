dhcp-server
===========
This package provides a wrapper around `dnsmasq` for allocating an IP address to
the Ouster LiDAR. To run the dhcp server:

```
[ ~/colcon/ros2_ouster_tools ]
tpanzarella@jelly $ ros2 run ouster_tools dhcp-server
sudo /usr/sbin/dnsmasq -C /dev/null -kd -F192.168.0.254,192.168.0.254 -i enp0s31f6 --bind-dynamic
dnsmasq: started, version 2.79 cachesize 150
dnsmasq: compile time options: IPv6 GNU-getopt DBus i18n IDN DHCP DHCPv6 no-Lua TFTP conntrack ipset auth DNSSEC loop-detect inotify
dnsmasq-dhcp: DHCP, IP range 192.168.0.254 -- 192.168.0.254, lease time 1h
dnsmasq-dhcp: DHCP, sockets bound exclusively to interface enp0s31f6
dnsmasq: reading /etc/resolv.conf
dnsmasq: using nameserver 127.0.0.53#53
dnsmasq: read /etc/hosts - 7 addresses
dnsmasq-dhcp: DHCPDISCOVER(enp0s31f6) bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPOFFER(enp0s31f6) 192.168.0.254 bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPDISCOVER(enp0s31f6) bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPOFFER(enp0s31f6) 192.168.0.254 bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPREQUEST(enp0s31f6) 192.168.0.254 bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPACK(enp0s31f6) 192.168.0.254 bc:0f:a7:00:07:92 os1-991937000055

```

The command above is run using the default arguments and expected outputs are
shown. The help message for the program showing its arguments looks like:

```
[ ~/colcon/ros2_ouster_tools ]
tpanzarella@jelly $ ros2 run ouster_tools dhcp-server --help
usage: dhcp-server [-h] [-i IFACE] [--min-ip MIN_IP] [--max-ip MAX_IP]

Runs a local dhcp server using dnsmasq

optional arguments:
  -h, --help            show this help message and exit
  -i IFACE, --iface IFACE
                        Listen on this network interface (default: enp0s31f6)
  --min-ip MIN_IP       Low end of the IP allocation range (default:
                        192.168.0.254)
  --max-ip MAX_IP       High end of the IP allocation range (default:
                        192.168.0.254)
```

Effort has been made to try to make sensible default arguments, somewhat biased
to my primary development set-up. Others can always override these defaults by
explicitly passing parameters to the script.

On my laptop (where I am doing most of my Ouster development), I have three
interfaces: `enp0s31f6`, `lo`, `wlp3s0` which are my hardwired, local loopback,
and wireless interfaces respectively. In my dev setup, I plug the Ouster
directly into my hardwired interface (`enp0s31f6`). If I probe the system to
list those interfaces dynamically and lexographically sort them, the hardwired
one will be the first element of that list. So, my hueristic for the default
interface will be the first one in that sorted list (and there will always be
at least one since we have the local loopback). I then take the IP that is
bound to that interface and make the assumption it is a Class C IPv4 address
(so a 24-bit subnet mask). I then look at that network and serve up the highest
valid IP address on that network (I assume I do not have that address). This
lets me serve up an IP to my Ouster in a way that almost makes it act like the
Ouster has a static IP address and I can set the parameters on the ROS2 driver
making this assumption.

So, for clarity, my typical setup is to bind the static IP address of
`192.168.0.92` (I use this for legacy reasons going back roughly almost 2
decades) to my hardwired interface, which on my current laptop is
`enp0s31f6`. I then serve up an IP address range over dhcp that consists of a
single address, specfically `192.168.0.254`. Since I wire the Ouster directly
to my machine I know it is the only device looking for an address and it will
always be the same. So, I can treat it as *static*. Of course, we always have
the explicit command line options that can override these assumptions.

Tip for when using together with Network Manager
------------------------------------------------

If you are using Network Manager to manage your network connections (it is
convenient on a laptop -- I am using it), you should take care to ensure your
wired connection is activated prior to starting the DHCP server. In my case, I
have named my wired configuration that I use with the Ouster `wired-0`. So, to
first bring it up:

```
$ nmcli connection up wired-0
Connection successfully activated (D-Bus active path: /org/freedesktop/NetworkManager/ActiveConnection/5)
```

You can see that my IP is now bound to the interface:

```
$ ip addr show dev enp0s31f6
2: enp0s31f6: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc fq_codel state DOWN group default qlen 1000
    link/ether e8:6a:64:f4:3c:5b brd ff:ff:ff:ff:ff:ff
    inet 192.168.0.92/24 brd 192.168.0.255 scope global noprefixroute enp0s31f6
       valid_lft forever preferred_lft forever
    inet6 fe80::fe1f:f914:cc92:db5b/64 scope link tentative
       valid_lft forever preferred_lft forever
```

or

```
$ ifconfig
enp0s31f6: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        inet 192.168.0.92  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::fe1f:f914:cc92:db5b  prefixlen 64  scopeid 0x20<link>
        ether e8:6a:64:f4:3c:5b  txqueuelen 1000  (Ethernet)
        RX packets 565  bytes 59890 (59.8 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 1793  bytes 178974 (178.9 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
        device interrupt 16  memory 0xe9200000-e9220000

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 2400705  bytes 40176131048 (40.1 GB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 2400705  bytes 40176131048 (40.1 GB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wlp3s0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.86.136  netmask 255.255.255.0  broadcast 192.168.86.255
        inet6 fe80::9447:d27:b47f:de36  prefixlen 64  scopeid 0x20<link>
        ether a4:c3:f0:93:bd:d1  txqueuelen 1000  (Ethernet)
        RX packets 22319401  bytes 26854286293 (26.8 GB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 10557951  bytes 7569423946 (7.5 GB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

Now I can start the dhcp server without having the Ouster powered on and serve
addresses correctly:

```
$ ros2 run ouster_tools dhcp-server
sudo /usr/sbin/dnsmasq -C /dev/null -kd -F192.168.0.254,192.168.0.254 -i enp0s31f6 --bind-dynamic
[sudo] password for tpanzarella:
dnsmasq: started, version 2.79 cachesize 150
dnsmasq: compile time options: IPv6 GNU-getopt DBus i18n IDN DHCP DHCPv6 no-Lua TFTP conntrack ipset auth DNSSEC loop-detect inotify
dnsmasq-dhcp: DHCP, IP range 192.168.0.254 -- 192.168.0.254, lease time 1h
dnsmasq-dhcp: DHCP, sockets bound exclusively to interface enp0s31f6
dnsmasq: reading /etc/resolv.conf
dnsmasq: using nameserver 127.0.0.53#53
dnsmasq: read /etc/hosts - 7 addresses
```

And once I power on the Ouster we see it gets the `192.168.0.254` address that
we expect

```
dnsmasq-dhcp: DHCPDISCOVER(enp0s31f6) bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPOFFER(enp0s31f6) 192.168.0.254 bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPDISCOVER(enp0s31f6) bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPOFFER(enp0s31f6) 192.168.0.254 bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPREQUEST(enp0s31f6) 192.168.0.254 bc:0f:a7:00:07:92
dnsmasq-dhcp: DHCPACK(enp0s31f6) 192.168.0.254 bc:0f:a7:00:07:92 os1-991937000055
```
