# Behaviors

Behaviors are the main way to configure the map in more detail. They use a strategy pattern with templates, so switching behaviors requires only changing the `behavior` parameter in `radix_ros/params/<behavior>/main_params.yaml`.

The available behavior names are defined in `radix_map/include/radix_map/map_factory.hpp`, which maps each name string to its implementation class. The behavior implementations live in `radix_map/include/radix_map/behavior/`.

Adding a new behavior requires defining three things: the voxel data struct, the corresponding point type, and the insertion logic. The existing behaviors serve as ready-to-use templates for this.

## Basic

The simplest behavior. Stores only an occupancy probability per voxel. Input point clouds require XYZ fields only.

## Semantic

Maintains a per-voxel histogram of label probabilities over all observed labels. As new points arrive, the histogram is updated via an exponential moving average, capturing the weighted label distribution within the voxel. For publishing, the label with the highest probability and its associated confidence are selected.

Input point clouds require an additional semantic `label` field.

## Gaussian

Extends Semantic by adding per-voxel geometric statistics. In addition to the label probability histogram, it incrementally computes and stores the mean (μ) and covariance matrix (Σ) of all points observed within the voxel, modeling the local 3D spatial distribution. For publishing, the behavior outputs the maximum-probability label with its confidence alongside the Gaussian statistics.

Input point clouds require an additional semantic `label` field.

## ICP

Combines the Gaussian geometry statistics with semantic labels, specifically to support ICP-based scan registration and odometry. Used in the SOCC-ICP pipeline.

Input point clouds require an additional semantic `label` field.
