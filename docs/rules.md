# Rules

Rules apply denoising filters to a specified region of the map to remove noise and correct voxel labels.

```bash
ros2 service call /radix_server/rules 'radix_msgs/srv/Rules' "{x_min: -200, y_min: -200, z_min: -200, x_max: 200, y_max: 200, z_max: 200}"
```

This first collects all voxels within the bounding box that need to be updated, then applies the updates in a single pass.

## Majority Label / Label Dilation Rule

The main implemented rule is label dilation: each voxel's label is updated based on the majority label among its neighbors. The process is configurable per class.

- **Class-based configuration** — each label can define an optional `dilation_update_thres` in `label_params.yaml`:
  - *Not defined*: uses the default `update_thres` from `rules.yaml`
  - *Set to `-1`*: label is excluded from updates (e.g. persons)

- **Kernel-based processing** — one or more kernels can be applied sequentially (`kernel_size: [5, 5, 5]` in `rules.yaml`). The dilation operation runs after each kernel in order.

- **Update logic** — for each voxel in the region: collect neighbor labels within the kernel, determine the majority label, and update the voxel only if the majority count exceeds the threshold for that class.

This allows fine-grained control over which labels are updated and how strongly neighboring voxels influence the result.

To test class-specific thresholds, add a `dilation_update_thres` value for a class in `label_params.yaml`. This overrides the default threshold for that class during label dilation.
