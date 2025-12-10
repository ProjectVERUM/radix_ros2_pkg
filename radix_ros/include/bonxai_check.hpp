#include <unordered_set>
#include <vector>
#include "bonxai/bonxai.hpp"
#include "bonxai_map/probabilistic_map.hpp"

// loop through all map voxels in the map
template <typename PointT>
std::tuple<bool, double> checkMap(const std::vector<PointT> map) {
  int validVoxels = 0;
  int totalVoxels = map.size();

  if (totalVoxels == 0) {
    std::clog << "Warning! The chunk is empty." << std::endl;
    return std::make_tuple(false, 0);
  }

  for (const auto& voxel : map) {
    if (voxel.class_history.valid()) {
      validVoxels++;
    }
  }

#ifdef CLASSHISTORY
  for (const auto& voxel : map) {
    if (checkHistory(voxel.class_history)) {
      validVoxels++;
    }
  }
#endif
  std::clog << "Valid Voxels: " << validVoxels << std::endl;
  std::clog << "Total Voxels: " << totalVoxels << std::endl;
  double validPercent = (static_cast<double>(validVoxels) / totalVoxels) * 100;
  return std::make_tuple(true, validPercent);
}
