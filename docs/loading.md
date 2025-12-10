# Map Loading and Saving

## Loading

Basic loading:

- deserialize header of map
- find the type of behavior of the map
- set the current config to the correct behavior
- create a new map with the correct behavior (createMapFromConfig)
- deserialize the grid
- set the maps grid to the deserialized grid


These are the ways to load a map:
- set map name in `params/label_params.yaml`
    - set `load_map` to the name of the map
    - Basic loading
- call load service with map name
    - set `load_map` to the name of the map in the service request
    - destruct the current map
    - Basic loading


```bash
ros2 service call /radix_server/load 'radix_msgs/srv/Load' "{filename: <string>}"
```

## Saving

Saving only works by calling the save service with a map name.
```bash
ros2 service call /radix_server/save 'radix_msgs/srv/Save' "{filename: <string>}"
```

filename is only the name of the map (in the map dir), not the full path, and without extension


Currently Bonxai has a 10s timer that resets when a pointcloud is inserted. If the timer runs out, it will try to save to the specified directory with the specified name.

# Issues/Weirdness

- Need custom serialization for heap allocated objects (e.g. unordered_map) in voxels
    - so need to loop over map and write/read keys and values instead
    - would be nice to find an alternative
