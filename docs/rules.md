# Rules

You can apply filters (denoising kernels) to a specified chunk to remove noise and correct the map.

To apply filters, run:

```bash
ros2 service call /radix_server/rules 'radix_msgs/srv/Rules' "{x_min: -200, y_min: -200, z_min: -200, x_max: 200, y_max: -200, z_max: 200}"
```

This command first gathers all voxels that need to be updated and then updates the map in a single step.

## Majority Label / Label Dilation Rule

The main rule implemented is label dilation, which updates voxel labels based on the majority of neighboring labels. The process is configurable per class and uses a neighborhood kernel.

- Class-based configuration:

    Each label can have an optional parameter dilation_update_thres:

    - **Not defined**: The label uses the default update_thres from the rules configuration.
    - **Set to -1**: The label is ignored during the update (e.g., persons are not updated).

- Kernel-based processing:

    One or more kernels can be applied sequentially, as specified in the configuration (kernel_size: [5, 5, 5]).
    The dilation operation is applied after each kernel in order.

- Updating voxels:

    For each voxel in the selected region:

    - Collects labels from the neighborhood defined by the current kernel.

    - Determines the majority label among these neighbors.

    - Updates the voxel only if the majority label count exceeds the threshold for that class (or default
      threshold if not specified)

This approach allows control over which labels are updated and how strongly neighboring voxels influence the update.

## Example

Below is an example of a map chunk before and after applying the rules.


before aplying rules :

![before](./assets/imgs/before.png)



after applying rules :

![corrected](./assets/imgs/corrected.png)

---


To test class-specific thresholds, add a `dilation_update_thres` value for an existing class in `label_params.yaml`. This will override the default threshold for that class during label dilation.


After applying the rules while excluding the building class:


![corrected_skip_building](./assets/imgs/corrected_skip_building.png)
