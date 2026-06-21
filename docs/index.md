---
title: Overview
---

# Overview

This describes the internal structure of Radix, which is built on top of the Bonxai library (`bonxai.hpp` and related header files).

# Main Classes

## Map

The `Map` class holds the `VoxelGrid` and provides methods for ray-casting and inserting points into the map. It uses the VoxelGrid API internally to access and modify the grid.

Most of the map's behavior-specific logic is delegated to the behavior classes.

## Server

The `Server` class handles all ROS 2 integration: reading launch parameters, managing topics (publishers and subscribers), and exposing services.

Methods on this class should only handle ROS 2 communication. All map logic is delegated to `Map` methods. Point cloud decoding also goes through the map methods, because the behavior-specific point types are not accessible from the non-templated server class.

## Behavior

The `Behavior` class defines the interface that each custom behavior must implement. It controls voxel data layout and insertion logic in the `Map` class. Custom behaviors define a `Voxel` type, a `Point` type matching the incoming point cloud format, and how voxel fields are updated when a point is inserted.

See [Behaviors](./behaviors.md) for details.
