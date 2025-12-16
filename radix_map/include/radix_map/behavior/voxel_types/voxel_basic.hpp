#pragma once
#include <iostream>
#include "radix_map/behavior/voxel_types/voxel_base.hpp"
#include "radix_map/serialize.hpp"
#include "radix_map/utils/queue.hpp"

class VoxelBasic : public VoxelBase {
 public:
  // bitfields: cannot be default-initialized before C++20
  int32_t update_id : 4;
  int32_t probability_log : 28;

  int instanceID;
  int32_t label;
  float semantic_prob = 0.5f;  // unsupported: will remain unchanged

  Queue<int, 5> label_history;
  Queue<float, 5> uncertainty_history;
  Queue<float, 5> semantic_conf_history;
  Queue<float, 5> topological_conf_history;

  VoxelBasic() : update_id(0), probability_log(0), instanceID(0), label(0) {}

  size_t customMemUsage() const { return sizeof(VoxelBasic); }
};

template <>
inline void Write(std::ostream& out, const VoxelBasic& voxel) {
  Write(out, voxel.update_id);
  Write(out, voxel.probability_log);
  Write(out, voxel.instanceID);
  Write(out, voxel.label_history);
  Write(out, voxel.uncertainty_history);
  Write(out, voxel.semantic_conf_history);
  Write(out, voxel.topological_conf_history);
}

template <>
inline VoxelBasic Read(std::istream& input) {
  VoxelBasic voxel;
  voxel.update_id = Read<int32_t>(input);
  voxel.probability_log = Read<int32_t>(input);
  voxel.instanceID = Read<int>(input);

  voxel.label_history = Read<Queue<int, 5>>(input);
  voxel.uncertainty_history = Read<Queue<float, 5>>(input);
  voxel.semantic_conf_history = Read<Queue<float, 5>>(input);
  voxel.topological_conf_history = Read<Queue<float, 5>>(input);

  return voxel;
}
