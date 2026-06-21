# Publish a Subsection / Chunk of the Map

This service extracts a specific bounding box region of the map (need to source the workspace first: `source install/setup.bash`):

```bash
ros2 service call /radix_server/chunk radix_msgs/srv/Chunk "{x_min: <min-x>, x_max: <max-x>, y_min: <min-y>, y_max: <max-y>, z_min: <min-z>, z_max: <max-z>, free: <true|false>, delete_rest: <true|false>, publish: <true|false>, occupancy_threshold: <threshold>, level: <'cell'|'leaf'|'inner'>}"
```

`*_min` / `*_max` define the bounding box in world coordinates. Setting `free: true` includes free-space voxels in the result (default: `false`). Setting `delete_rest: true` removes all voxels outside the bounding box from the map. Setting `publish: true` publishes the result on the `/radix_chunk` topic. `occupancy_threshold` sets the minimum probability to consider a voxel occupied. `level` selects the grid resolution (`'cell'`, `'leaf'`, or `'inner'`).

The service returns:

- `chunk` — the extracted region as a `sensor_msgs/PointCloud2` message
- `success` — `true` if the operation succeeded; if `false`, check `reason` for an error message
