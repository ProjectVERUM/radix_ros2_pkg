#pragma once
#include "radix_map/map.hpp"

template <typename B>
void MapImpl<B>::traverseMap() {
  std::shared_ptr<MapImpl<B>> map_copy = this->copy();

  for (int x = 0; x < _size.x; x++) {
    for (int y = 0; y < _size.y; y++) {
      for (int z = 0; z < _size.z; z++) {
        CoordT current_coord = {x, y, z};

        int new_label = getMaxLabel(current_coord, *map_copy);

        typename B::Voxel& orig_voxel = _accessor.value(current_coord, false);

        applyMaxFilter(orig_voxel, new_label);
      }
    }
  }
}

template <typename B>
int MapImpl<B>::getMaxLabel(const CoordT& coord, const MapImpl<B>& map_copy) {

  std::unordered_map<int, int> labelCounts;
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      for (int dz = -1; dz <= 1; dz++) {

        CoordT neighbor_coord = {coord.x + dx, coord.y + dy, coord.z + dz};

        const typename B::Voxel& neighbor_voxel =
            map_copy._accessor.value(neighbor_coord, false);

        if (!neighbor.label_history.empty()) {
          int label_id = neighbor.label_history.back();
          labelCounts[label_id]++;
        }
      }
    }
  }

  int maxCount = 0;
  int maxLabel = -1;
  for (const auto& l : labelCounts) {
    if (l.second > maxCount) {
      maxCount = l.second;
      maxLabel = l.first;
    }
  }

  return maxLabel;
}

template <typename B>
void MapImpl<B>::applyMaxFilter(typename B::Voxel& cell, int maxLabel) {
  if (maxLabel != -1) {
    if (!cell.label_history.empty()) {
      cell.label_history.back() = maxLabel;
    } else {
      cell.label_history.push(maxLabel);
    }
  }
}
