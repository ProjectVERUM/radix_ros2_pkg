# Rendering

Bonxai provides a rendering service to generate images of the map from a specified viewpoint.

## Taking an image of the map

Rendering of an image from a certain perspective. The image will be published on the topic `/radix_image`.

Two options are available:

#### Latest tf data

Run this service to take an image of the map with the latest tf data as the camera pose(need to source the package before calling with `source install/setup.zsh`):

```bash
ros2 service call /radix_server/render 'radix_msgs/srv/Render' "{live: true}"
```

#### Custom Pose

Run this service to take an image of the map with a custom pose:

```bash
ros2 service call /radix_server/render radix_msgs/srv/Render "{x: <x-coordinate>, y: <y-coordinate>, z: <z-coordinate>, roll: <roll>, pitch: <pitch>, yaw: <yaw>, max_range: <max_range>, width: <width>, height: <height>, yaw_angle: <yaw_angle>, pitch_angle: <pitch_angle>, level: <'cell'|'leaf'|'inner'>, live: <true|false>, publish: <true|false>, occupancy_threshold: <threshold>, use_rgb: <true|false>}"
```

If `publish` is set to true, the images will also be published as ROS topics. The `occupancy_threshold` parameter defines the threshold for considering a voxel occupied, and if `use_rgb` is set to true, the rendering will include RGB color information in the output image.


The service returns:
a `sensor_msgs/Image` for the Semantic Image.
a sensor_msgs/Image for the Semantic Probability Image.
a sensor_msgs/Image for the Depth Image.
a sensor_msgs/Image for the Depth Probability Image.
a bool `success` that is true if the operation succeeded; if it is false, check the `reason` field for an error message.

All the values have reasonable defaults, so you can leave them empty if you want to use the defaults, which are defined in `srv/Render.srv`

![rendering](./assets/imgs/rendering.png)
