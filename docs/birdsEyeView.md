# Bird's Eye View / 2D Map

This service generates a 2D orthographic top-down image of a specified area of the map.

```bash
ros2 service call /radix_server/birds_eye_view radix_msgs/srv/BirdsEyeView "{x_min: <x_min>, x_max: <x_max>, y_min: <y_min>, y_max: <y_max>, min_height: <min_height>, max_height: <max_height>, occupancy_threshold: <threshold>, level: <'cell'|'leaf'|'inner'>}"
```

`x_min`/`x_max` and `y_min`/`y_max` define the 2D region to render. `min_height`/`max_height` filter the Z range included. `occupancy_threshold` sets the minimum probability to consider a voxel occupied. `level` selects the grid resolution (`'cell'`, `'leaf'`, or `'inner'`).

The service returns:

- `semantic_img` — semantic label image (`sensor_msgs/Image`)
- `semantic_prob_img` — label confidence image (`sensor_msgs/Image`)
- `depth_img` — depth image (`sensor_msgs/Image`)
- `depth_prob_img` — depth probability image (`sensor_msgs/Image`)
- `success` — `true` if the operation succeeded; if `false`, check `reason` for an error message
