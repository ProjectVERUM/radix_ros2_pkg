# Radix

<p align="center">
  <a href="https://docs.ros.org/en/humble/"><img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" alt="ROS 2 Humble"></a>
  <a href="https://en.cppreference.com/w/cpp/17"><img src="https://img.shields.io/badge/C%2B%2B-17-blue?logo=cplusplus" alt="C++17"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-MPL_2.0-brightgreen" alt="License: MPL 2.0"></a>
  <a href="https://github.com/orgs/ProjectVERUM/packages?repo_name=radix_ros2_pkg"><img src="https://img.shields.io/badge/Docker-GHCR-blue?logo=docker" alt="GHCR"></a>
</p>

<p align="center"><img src="./docs/assets/radix_square.png" width="40%"></p>

Radix is a C++ library for **semantic occupancy grid mapping**, integrated as a ROS 2 node. It continuously fuses incoming point clouds from rosbag replay or manual insertion into a sparse 3D voxel grid and exposes the live map to downstream applications via ROS 2 services.

Radix is built on top of [Bonxai](https://github.com/facontidavide/Bonxai) by Davide Faconti, a high-performance sparse voxel grid library that serves as the underlying data structure.

<img src="./docs/assets/radix.svg" width="100%">

## Ecosystem
Radix is built from the following modules, listed bottom-up from the core data structure to downstream usage:

- **radix_ros2_pkg/bonxai_core** — Header-only sparse hierarchical voxel grid [Bonxai](https://github.com/facontidavide/Bonxai), the foundational data structure Radix is built on.
- **radix_ros2_pkg/radix_map** — Core mapping library. Fuses incoming point clouds into the Bonxai grid using Bayesian log-odds occupancy updates and ray-casting for free-space clearing. A key feature of Radix is its **behavior-based design**: custom mapping logic can be introduced through minimal, well-scoped modifications. Each behavior defines the voxel data layout and insertion logic (e.g. occupancy probability only, geometry statistics, or per-class semantic label distributions).
- **radix_ros2_pkg/radix_ros** — ROS 2 server node wrapping the above. Subscribes to `/semantic_cloud` (sensor data or rosbag replay) per default, continuously builds the occupancy grid, and exposes it via ROS 2 services.
- **[radix_msgs_ros2_pkg](https://github.com/ProjectVERUM/radix_msgs_ros2_pkg)** — Custom ROS 2 message and service definitions (`Chunk`, `Render`, `BirdsEyeView`, `Save`, `Load`, `RayHit`, …). These are the communication protocol between the Radix server and any downstream client.
- **[radix_clients](https://github.com/ProjectVERUM/radix_clients)** — Python client library. Wraps the ROS 2 service calls into simple functions that return numpy arrays for downstream applications.
- **[radix_examples](https://github.com/ProjectVERUM/radix_examples)** — Runnable Python scripts demonstrating the full workflow against a live Radix server using a bundled KITTI point cloud snippet (publish cloud, retrieve chunk, render, ray cast).


# Installation

**Requirements:** Ubuntu 22.04, [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html), C++17, `colcon`. For the Docker option, only Docker is required.

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

To build the image manually, ensure you cloned the required repositories as described above and run from the workspace root:
```bash
cd ros2_ws
docker build -f src/radix_ros2_pkg/Dockerfile -t radix_ros2_pkg:latest .
```

Alternatively, pre-built Docker images can be pulled directly from the repository's [package registry](https://github.com/orgs/ProjectVERUM/packages?repo_name=radix_ros2_pkg), e.g.:
```bash
docker pull ghcr.io/projectverum/radix_ros2_pkg:latest
```


# Usage

## [Option 1] Local Build

Source the workspace and launch the server with the desired behavior:

```bash
source install/setup.bash

ros2 launch radix_ros radix_basic.launch.py
ros2 launch radix_ros radix_gaussian.launch.py
ros2 launch radix_ros radix_semantic.launch.py
ros2 launch radix_ros radix_icp.launch.py
```

**Configuration files** for each behavior live at:
```
radix_ros/params/<behavior>/
├── main_params.yaml   # core settings: topic names, resolution, occupancy probabilities, …
├── label_params.yaml  # semantic label definitions and colors
└── rules.yaml         # optional rule-based voxel filtering
```

Edit `main_params.yaml` before launching to change, e.g., the input topic or voxel resolution:
```yaml
cloud_in_topic: /your/sensor/topic   # input PointCloud2 topic
resolution: 0.5                      # voxel edge length in meters
```

Alternatively, override individual parameters at launch time without editing the file:
```bash
ros2 launch radix_ros radix_semantic.launch.py cloud_in_topic:=/your/sensor/topic
```

## [Option 2] Docker

Simply **run** the container (defaults to `ros2 launch radix_ros radix_semantic.launch.py`):

```bash
docker run radix_ros2_pkg:latest
```

To specify a different launch file:
```bash
docker run radix_ros2_pkg:latest ros2 launch radix_ros radix_semantic.launch.py
```

To use custom parameters, modify the parameter files under `radix_ros/params/` and mount the directory into the container:

```bash
docker run -t --rm \
  --name radix_ros2_pkg \
  -v ./radix_ros/params/:/ros2_ws/src/radix_ros2_pkg/radix_ros/params/ \
  radix_ros2_pkg:latest
```

## During Runtime

### Replay a rosbag
Make sure the point cloud topic published by the rosbag matches `cloud_in_topic` in `main_params.yaml` (default: `/semantic_cloud`). If Radix is running and receiving messages, you should see log output confirming that points are being inserted into the occupancy grid.
```bash
ros2 bag play <folder_name>
```

### Visualize in Rviz
Radix re-publishes the voxel grid after every point cloud insertion (`publish_on_insert: true` by default), which allows live inspection of the map as it is built. Note that publishing on every insertion adds runtime overhead, for large clouds or high-frequency sensors, consider setting `publish_on_insert: false` instead.

To visualize the voxel grid, open Rviz2:
```bash
rviz2
```
Add the topic `/radix_point_cloud_centers`, set the display type to *PointCloud2*, style to e.g. *Boxes*, and set the size to match the voxel resolution (default `0.5` m).


# Behaviors

Radix supports four interchangeable behaviors out of the box that determine the voxel type, expected input point cloud format, and map update logic. These behaviors are not fixed, if you need to store different or additional information per voxel (e.g. intensity, timestamp, custom uncertainty), a new behavior can be added with minimal, well-scoped changes: define the voxel data struct, the corresponding point type, and the insertion logic. The existing behaviors serve as ready-to-use templates for this.

| Behavior | Voxel stores | Input point cloud | Use case |
|----------|-------------|-------------------|----------|
| `basic` | Occupancy probability | XYZ only | Lightweight obstacle mapping |
| `gaussian` | Occupancy + geometry statistics (μ, Σ) | XYZ | Geometry-aware mapping, planarity estimation |
| `semantic` | Occupancy + per-class label distribution + dominant label | XYZ + semantic `label` field | Semantic mapping from labeled sensor data |
| `icp` | Occupancy + geometry statistics + semantic labels | XYZ + semantic `label` field | Semantics-assisted odometry (SOCC-ICP) |

Select the behavior via the launch file or by setting the `behavior` parameter.

For details on available behaviors and how to use them, see [Behaviors](./docs/behaviors.md).


# Services

The Radix server exposes the following ROS 2 services (all under the `/radix_server` namespace by default). Service request/response fields are defined in [radix_msgs_ros2_pkg](https://github.com/ProjectVERUM/radix_msgs_ros2_pkg).

| Service | Type | Description |
|---------|------|-------------|
| `~/chunk` | `radix_msgs/Chunk` | Extract occupied voxels from a bounding box as PointCloud2 |
| `~/render` | `radix_msgs/Render` | Ray-cast render from a camera pose → 4 images |
| `~/birds_eye_view` | `radix_msgs/BirdsEyeView` | Orthographic top-down render → 4 images |
| `~/save` | `radix_msgs/Save` | Save map to file |
| `~/load` | `radix_msgs/Load` | Load map from file |
| `~/check` | `radix_msgs/Check` | Validate voxel integrity around a query point |
| `~/check_ray_hit` | `radix_msgs/RayHit` | Test line-of-sight; returns first hit voxel coordinates |
| `~/config` | `radix_msgs/Config` | Query current server configuration |
| `~/rules` | `radix_msgs/Rules` | Apply rule-based filtering to a bounding box |
| `~/reset` | `std_srvs/Empty` | Clear the entire map |
| `~/publish` | `std_srvs/Empty` | Trigger immediate map publication |

More detailed explanations for each service can be found in `./docs`. They can be served locally with mkdocs:

```bash
pip install mkdocs==1.6.0 mkdocs-material==9.5.21 jinja2==3.1.4
mkdocs serve  # serves on http://127.0.0.1:8000/
```

### Service level parameter

For the `chunk`, `render`, and `birds_eye_view` services, the `level` field controls the grid hierarchy at which the operation runs:

| Level | Voxel size | Note |
|-------|-----------|------|
| `"cell"` | `resolution` (default) | Full resolution |
| `"leaf"` | coarser | Faster, lower detail |
| `"inner"` | coarsest | Fastest, lowest detail |

**Warning**: `"leaf"` and `"inner"` levels are not fully supported for all services yet.


# Development

This repository uses [`pre-commit`](https://pre-commit.com/) to run automated checks.

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


# Citation

Radix is a general-purpose semantic occupancy grid mapping library. The work below demonstrates one concrete application, semantics-assisted LiDAR odometry, for which Radix was adapted and used. It also contains a brief introduction to the library. If you find this repository useful for your research, please consider citing it.

> J. Scherer, S. Hirt, H. Meeß, *"SOCC-ICP: Semantics-Assisted Odometry based on Occupancy Grids and ICP"*
> [arXiv:2605.15074](https://arxiv.org/abs/2605.15074) | [IEEE](https://ieeexplore.ieee.org/document/11543211)
