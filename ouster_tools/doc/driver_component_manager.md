driver_component_manager
========================

The [ROS2 ouster driver](https://github.com/SteveMacenski/ros2_ouster_drivers)
is implemented as a managed node. Additionally, it has a component interface
and can be loaded into a component container for running intraprocess pipelines
with the data. It is well known that (as of this writing) ROS2 has no way of
driving the FSM for a *managed component* via `launch`, the typical way is it
done for non-component managed nodes. To plug that hole in the ROS2 tooling, we
provide an out-of-process lifecycle manager for the LiDAR driver when run in a
component container.

The `driver_component_manager` is designed as a *high availability*
manager. That is, it tries to keep the driver up and `active` at all times. It
also tries to immediately, at startup, transition to `active` immediately.

### Parameters
<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Default Value</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>~/driver_node</td>
    <td>string</td>
    <td>/ouster_driver</td>
    <td>
    The fully-qualified name to the ROS node (in the ROS graph) that implements
    the Ouster driver as a component. This is the node that this manager is
    responsible for driving the FSM.
    </td>
  </tr>
  <tr>
  <td>~/timeout_millis</td>
    <td>int</td>
    <td>1000</td>
    <td>
    Timeout in milliseconds to wait when calling the driver's FSM services
    (e.g., to get and change its state).
    </td>
  </tr>
</table>

These parameters can be set in the [configuration file](../etc/driver_component_manager.yaml).

### Running

We provide a [launch file](../launch/driver_component_manager.launch.py) for
bringing up the manager. However, in typical usage, our supplied launch file
would be called by another launch file that brings up the driver (and component
container) as well. For a concrete example of that, please see [this launch
file](../launch/perf_component.launch.py) which is used to run our `perf_node`
in a component container.
