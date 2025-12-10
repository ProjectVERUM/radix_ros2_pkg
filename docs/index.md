---
title: Overview
---

# Overview

This describes the structure built upon the Bonxai library (bonxai.hpp and related header files).

![Overview](./assets/gifs/vis_radix.gif)


# Main Classes

## Map

The map class contains the VoxelGrid and holds methods like ray casting and adding points to the map.
It then uses the VoxelGrid API in these methods to access the actual map.

It holds most things defined in the behavior classes.

## Server

The server class mainly handles the connection to the ROS2 ecosystem.
This includes reading the launch files, and various other configs.
It also handles all the ROS2 topics (publishers and subscribers) and services.

The methods on this class should always just handle the communication with ROS2.
For everything else, it should call the map methods that interface with the map.

For receiving point clouds, it is necessary to use the map methods to decode as well, because the behavior point types can't be used from the server class, because it is not templated.


## Behavior

The behavior class that needs to be implemented for each custom behavior.
It modifies different parts of the application, mainly in the map class.
You can create a custom Voxel type and a Point type that a pointcloud might have.
Then you can define how that Voxel (or the fields on it) change when a point of that type gets inserted.

More info found in [Behaviors](./behaviors.md)
