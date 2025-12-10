#pragma once
#include "radix_map/map.hpp"
#include "radix_map/utils/queue.hpp"
#include "radix_utils/chunk_config.hpp"
#include "radix_utils/config.hpp"
#include "radix_utils/helpers.hpp"
#include "radix_utils/rules_config.hpp"

template <typename B>
void MapImpl<B>::getMajorityLabel(
    const int& ksize, const int& update_thres, const CoordT& coord,
    std::unordered_map<CoordT, int>& voxels_to_update) {

  int thres = update_thres;
  std::unordered_map<int, int> labelCounts;

  auto accessor = grid.createAccessor();

  typename B::Voxel* center_cell = accessor.value(coord, false);
  if (!center_cell || center_cell->label == 0) {
    return;
  }

  const int center_label = center_cell->label;
  const LabelEntry& entry = config.getLabelEntry(center_label);

  if (entry.dilation_update_thres.has_value()) {
    thres = entry.dilation_update_thres.value();
    if (thres == -1) {
      return;
    }
  }

  const int half_ksize = ksize / 2;

  for (int dx = -half_ksize; dx <= half_ksize; dx++) {
    for (int dy = -half_ksize; dy <= half_ksize; dy++) {
      for (int dz = -half_ksize; dz <= half_ksize; dz++) {

        CoordT current_coord = {coord.x + dx, coord.y + dy, coord.z + dz};
        typename B::Voxel* current_cell = accessor.value(current_coord, false);

        if (current_cell && current_cell->label != 0) {
          labelCounts[current_cell->label]++;
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

  if (maxLabel != -1 && maxLabel != center_label &&
      (maxCount - labelCounts[center_label]) >= thres) {

    voxels_to_update.insert({coord, maxLabel});
  }
}

template <typename B>
void MapImpl<B>::applyDilation(
    std::unordered_map<CoordT, int>& voxels_to_update) {

  auto accessor = grid.createAccessor();
  for (auto& vox : voxels_to_update) {
    const CoordT& c = vox.first;
    const int label = vox.second;

    typename B::Voxel* cell = accessor.value(c, false);
    behavior.correctVoxelLabel(cell, label);
  }
}

template <typename B>
void MapImpl<B>::traverseChunk(const int& ksize, const int& update_thres,
                               const Region& region) {

  std::unordered_map<CoordT, int> voxels_to_update;

  const float step = getChunkEdgeLength();
  const float res_half = grid.getResolution() / 2.0;

  for (float x = region.x_min; x < region.x_max + step; x += step) {
    for (float y = region.y_min; y < region.y_max + step; y += step) {
      for (float z = region.z_min; z < region.z_max + step; z += step) {

        grid.forEachCellInInner(
            [&](typename B::Voxel&, const CoordT& coord) {
              Point3D p = grid.coordToPos(coord);
              Point3D center = {p.x + res_half, p.y + res_half, p.z + res_half};

              if (center.x < region.x_min || center.x > region.x_max ||
                  center.y < region.y_min || center.y > region.y_max ||
                  center.z < region.z_min || center.z > region.z_max) {
                return;
              }

              getMajorityLabel(ksize, update_thres, coord, voxels_to_update);
            },
            x, y, z);
      }
    }

    if (!voxels_to_update.empty()) {
      applyDilation(voxels_to_update);
      voxels_to_update.clear();
    }
  }
}
