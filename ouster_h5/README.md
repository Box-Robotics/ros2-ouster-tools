ouster_h5
=========
This package provides the `h5_node`. It is used to record data published from
the Ouster driver to an HDF5 file. It is still under active development in
terms of its public interface.

### Parameters

<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Default Value</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>points_topic</td>
    <td>string</td>
    <td>/points</td>
    <td>The name of the pointcloud topic to subscribe to</td>
  </tr>
  <tr>
    <td>range_topic</td>
    <td>string</td>
    <td>/range_image</td>
    <td>The name of the range image topic to subscribe to</td>
  </tr>
  <tr>
    <td>distance_topic</td>
    <td>string</td>
    <td>/distance_image</td>
    <td>
      The name of the distance image topic to subscribe to. (As of this
      writing, the distance topic is not available in the mainline
      driver. This is currently only available on an internal development
      branch).
    </td>
  </tr>
  <tr>
    <td>h5_outfile</td>
    <td>string</td>
    <td>/tmp/h5_node.h5</td>
    <td>The full path to the HDF5 output file to write.</td>
  </tr>
  <tr>
    <td>h5_root_group</td>
    <td>string</td>
    <td>/ouster</td>
    <td>The top-level H5 group to write data underneath.</td>
  </tr>
</table>

### Published topics

None.

### Subscribed topics

This node subscribes to the topic names in the paraemeters listed above.

### Advertised services

None.
