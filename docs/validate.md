# Validating the map

Run this service to check and validate a subsection/chunk of the map.

```bash
ros2 service call /radix_server/check 'radix_msgs/srv/Check' "{x: 0, y: 20, z: 0, range: 10.0, free: 1}"
```
This will return the percentage of valid voxels out of total occupied voxels.

In addition a boolean value `success` is returned, which is False when the map is empty.
