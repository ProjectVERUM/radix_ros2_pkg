# Map Loading and Saving

## Loading

Loading a map deserializes it in the following steps:

1. Read the map header
2. Determine the behavior type stored in the map
3. Set the server configuration to the matching behavior
4. Create a new map with that behavior (`createMapFromConfig`)
5. Deserialize the voxel grid
6. Replace the current map grid with the deserialized one

Two ways to trigger loading:

- **At startup** — set `load_map` to the map name in `radix_ros/params/<behavior>/main_params.yaml`
- **At runtime** — call the load service:

```bash
ros2 service call /radix_server/load 'radix_msgs/srv/Load' "{filename: <string>}"
```

## Saving

```bash
ros2 service call /radix_server/save 'radix_msgs/srv/Save' "{filename: <string>}"
```

`filename` is the map name only (relative to `map_dir`), without a file extension. Radix appends `.radix` automatically.

## Known Limitations

- Voxels containing heap-allocated objects (e.g. `unordered_map`) require custom serialization: keys and values must be written/read explicitly rather than using a flat memory copy.
