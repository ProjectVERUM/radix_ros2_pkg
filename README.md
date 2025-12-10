# Radix

For more information about the original Bonxai library have a look at [this](README_bonxai.md) Readme.

<!--
<p align="center">
  <img src="./docs/assets/radix_square.png" width="60%" style="max-width:1000px; min-width:120px;">
</p>

## Examples

The following examples shows a Radix map (top) built from a semantic point cloud (bottom) over time.

![](./docs/assets/gifs/vis_radix.gif)

![](./docs/assets/gifs/vis_fused_point_cloud.gif)
-->



# Installation

## [Option 1] Local Build
Clone the required repositories:
```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/ProjectVERUM/radix_ros2_pkg
git clone https://github.com/ProjectVERUM/radix_msgs_ros2_pkg
```

Then build the packages:
```bash
cd ros2_ws
colcon build --packages-select radix_ros radix_msgs --symlink-install
source install/setup.bash
```

## [Option 2] Docker
The image can be built locally. The resulting image will contain a fully configured ROS 2 Humble environment with Radix (`radix_ros2_pkg` and `radix_msgs_ros2_pkg`) and all dependencies pre-installed and ready to use.

To build the image manually, ensure you cloned the required repositories as described above and run:
```bash
cd ros2_ws/src/radix_ros2_pkg
docker build -f src/radix_ros2_pkg/Dockerfile -t radix_ros2_pkg:humble --load .
```

Alternatively, pre-built Docker images can be pulled directly from the repository's [package registry](https://github.com/orgs/ProjectVERUM/packages?repo_name=radix_ros2_pkg), e.g.:
```bash
# linux/amd64
docker pull ghcr.io/projectverum/radix_ros2_pkg:humble-2025_12_15
```


# Usage

TODO introduction to usage

## [Option 1] Local Build
TODO

## [Option 2] Docker

Simply **run** the container (will run `ros2 launch radix_ros radix_semantic.launch.py`, i.e. using the `semantic` behavior):

```bash
docker run radix_ros2_pkg:humble
```

The launch file to be used can be specified:
```bash
docker run radix_ros2_pkg:humble ros2 launch radix_ros radix_semantic.launch.py
```

TODO parameter mounting

<!--

Starting the container will automatically run `ros2 launch radix_ros radix_semantic.launch.py`.

To use custom parameters (see [label_params.yaml](radix_ros/params/label_params.yaml), [main_params.yaml](radix_ros/params/main_params.yaml), and [rules.yaml](radix_ros/params/rules.yaml)), modify these files as needed and mount them into the container to overwrite the defaults.

For example, from `ros2_ws/src/radix_ros2_pkg`, after editing the parameter files, start the container with:

```bash
docker run -t --rm \
  --name radix_ros2_pkg \
  -v ./radix_ros/params/:/ros2_ws/src/radix_ros2_pkg/radix_ros/params/ \
  radix_ros2_pkg:humble
```


-->



<!--
## Usage

You can set the parameters for Radix in the following files:
`radix_ros/params/main_params.yaml` , `label_params.yaml` and `rules.yaml`.

### Custom Sensor Data

If you want to use your own sensor data, set the `cloud_in_topic` parameter to the topic where your sensor publishes point clouds (with cleaning ray)


### Run Radix

Before running Radix, you need to source the ROS 2 environment so ros2 knows that it exists. These files normally are in the `install` folder of the workspace.

```bash
source install/setup.bash
```

Then launch the server node with the desired behavior with the following command:

```bash
ros2 launch radix_ros radix_basic.launch.py
ros2 launch radix_ros radix_gaussian.launch.py
ros2 launch radix_ros radix_semantic.launch.py
ros2 launch radix_ros radix_icp.launch.py
```

### Run Rviz

```bash
rivz2

# with settings for real data
rviz2 -d src/radixrviz/realdata.rviz

# with settings for simulation
rviz2 -d src/radix/rviz/simdata.rviz
```

Radix expects a pointcloud with label_id's (with colors defined in class_config) in the label field of the pc2.

Sometimes it is necessary to change some settings in Rviz to visualize the map correctly (but with rviz config above it should be fine).

Add the topic `/radix_point_cloud_centers`, use flat squares or boxes and increase the size to 0.5. This should make the map clearly visible.

### Run a rosbag

```bash
# Run outside of the folder where the ros_bag_name.db3 and metadata.yaml file is located
ros2 bag play <folder_name>
```

![Radix](./docs/assets/imgs/map.png)

## Behaviors

Radix supports different behaviors that can be configured to modify how the map is built or interpreted (e.g., Voxeltype, incoming pointcloud type, insertion behavior).

For details on available behaviors and how to use them, see the [Behaviors](./docs/behaviors.md).

## Services

### [Saving and Loading Maps](./docs/loading.md)

### [Publishing Chunks](./docs/chunking.md)

### [Bird’s Eye View](./docs/birdsEyeView.md)

### [Rendering Options](./docs/rendering.md)

### [Validating Map](./docs/validate.md)

### [Debugging](./docs/debugging.md)

### [Publishing](./docs/publishing.md)

### [Rules](./docs/rules.md)

### (X) Different Resolutions

For the **chunk**, **render** and **birds_eye_view** services, you can specify the level it should be ran at.
The default level is "cell", the other levels are "cell" < "leaf" < "inner".
**WARNING** Some services are not working correctly yet with "leaf" and "inner" levels.

For publishing chunks, the size of the points in rviz will need to be adjusted depending on level/resolution to make it look nice.

# Setting Parameters at Runtime

You can modify certain Radix parameters at runtime using the following command:

```bash
ros2 param set /radix_server <parameter_name> <new_value>
```

Replace <parameter_name>, and <new_value> with your specific values.

**Note**: Some parameters may not make sense to change at runtime.
-->




# Development

This repository uses [`pre-commit`](https://pre-commit.com/) to run automated

### Setup

Install `pre-commit`:

```bash
pip install pre-commit
```

Install the Git hooks for this repository:
```bash
pre-commit install
```

Once installed, the checks will run automatically on every git commit. Checks can also be run with `pre-commit run -a`.


# Docs

More detailed explanations about how the application is structured can be found in `./docs`.

You can may serve those with mkdocs (`pip install mkdocs==1.6.0 mkdocs-material==9.5.21 jinja2==3.1.4`):

```bash
mkdocs serve  # serves on http://127.0.0.1:8000/
```
