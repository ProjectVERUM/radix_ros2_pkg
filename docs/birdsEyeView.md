# Bird's eye view/2D map

This service generates a 2D image (bird's eye view) of a specified area of the map.

```bash
ros2 service call /radix_server/birds_eye_view radix_msgs/srv/BirdsEyeView "{x_min: <x_min>, x_max: <x_max>, y_min: <y_min>, y_max: <y_max>, min_height: <min_height>, max_height: <max_height>, occupancy_threshold: <threshold>, level: <'cell'|'leaf'|'inner'>}"
```
Where *_min and *_max represent the minimum and maximum spatial bounds along each axis (X, Y, Z), `occupancy_threshold` is the threshold for considering a voxel occupied, and `level` is the map resolution level (`'cell'`, `'leaf'`, or `'inner'`).

The service returns:
an image, which is the resulting 2D map as a `sensor_msgs/Image`.
`success` flag that is true if the operation succeeded, if it is false check `reason` field for an error message.
`width` and `height`, which specify the dimensions of the returned image.
