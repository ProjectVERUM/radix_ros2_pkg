# Rendering

Radix provides a rendering service that generates images of the map from a specified viewpoint. Results are published on `/radix_image`.

## Taking an image of the map

Two options are available:

### Using the latest TF pose

Renders from the current sensor pose as reported by TF (source the workspace first: `source install/setup.bash`):

```bash
ros2 service call /radix_server/render 'radix_msgs/srv/Render' "{live: true}"
```

### Custom pose

```bash
ros2 service call /radix_server/render radix_msgs/srv/Render "{x: <x>, y: <y>, z: <z>, roll: <roll>, pitch: <pitch>, yaw: <yaw>, max_range: <max_range>, width: <width>, height: <height>, yaw_angle: <yaw_angle>, pitch_angle: <pitch_angle>, level: <'cell'|'leaf'|'inner'>, live: <true|false>, publish: <true|false>, occupancy_threshold: <threshold>, use_rgb: <true|false>}"
```

Setting `publish: true` also publishes the images as ROS topics. `occupancy_threshold` sets the minimum probability to count a voxel as occupied. `use_rgb` includes RGB color in the output images. All fields have defaults defined in `srv/Render.srv`.

The service returns:

- `semantic_img` — semantic label image (`sensor_msgs/Image`)
- `semantic_prob_img` — label confidence image (`sensor_msgs/Image`)
- `depth_img` — depth image (`sensor_msgs/Image`)
- `depth_prob_img` — depth probability image (`sensor_msgs/Image`)
- `success` — `true` if the operation succeeded; if `false`, check `reason` for an error message
