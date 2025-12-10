# Publish a subsection/chunk of the map

Run this service command to extract and publish a specific region (chunk) of the map on topic `/radix_chunk` (need to source the package before calling with `source install/setup.zsh`):

```bash
ros2 service call /radix_server/chunk radix_msgs/srv/Chunk "{x_min: <min-x>, x_max: <max-x>, y_min: <min-y>, y_max: <max-y>, z_min: <min-z>, z_max: <max-z>, free: <true|false>, delete_rest: <true|false>, publish: <true|false>, occupancy_threshold: <threshold>, level: <'cell'|'leaf'|'inner'>}"
```

The `*_min` and `*_max` specify the minimum and maximum coordinates along each axis (X, Y, Z) for the chunk region. Setting `free` to true will include free voxels in the chunk (default is false). The `delete_rest` option, if set to true, will delete all voxels outside the specified chunk. If `publish` is set to true, the chunk will be published as a ROS topic.`occupancy_threshold` is the threshold for considering a voxel occupied, and `level` is the map resolution level (`'cell'`, `'leaf'`, or `'inner'`).


![chunk](./assets/imgs/chunk.png)


The service returns:
a chunk as a `sensor_msgs/PointCloud2` message.
a bool `success` that is true if the operation succeeded, if it is false, check the `reason` field for an error message.

Chunk with free voxels in green (with opacity 0.005):
![chunk](./assets/imgs/chunk_free.png)
