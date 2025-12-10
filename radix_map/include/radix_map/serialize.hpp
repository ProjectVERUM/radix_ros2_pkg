#pragma once
#include <filesystem>
#include <regex>
#include "radix_map/map.hpp"
#include "radix_map/utils/queue.hpp"
#include "radix_utils/logger.hpp"

#ifdef __GNUG__
inline std::string demangle(const char* name) {
  int status = -4;  // some arbitrary value to eliminate the compiler warning

  std::unique_ptr<char, void (*)(void*)> res{
      abi::__cxa_demangle(name, NULL, NULL, &status), std::free};
  return (status == 0) ? res.get() : name;
}

#else

// does nothing if not g++
inline std::string demangle(const char* name) {
  return name;
}
#endif

struct HeaderInfo {
  std::string type_name;
  std::string behavior_name;
  int inner_bits = 0;
  int leaf_bits = 0;
  double resolution = 0;
  std::string timestamp;
};

template <typename T>
inline void Write(std::ostream& out, const T& val) {
  static_assert(std::is_trivially_copyable_v<T>, "Must be trivially copyable");
  out.write(reinterpret_cast<const char*>(&val), sizeof(T));
}

template <typename T>
inline T Read(std::istream& input) {
  T out;
  static_assert(std::is_trivially_copyable_v<T>, "Must be trivially copyable");
  input.read(reinterpret_cast<char*>(&out), sizeof(T));
  return out;
}

template <>
inline void Write(std::ostream& out, const MetaData& val) {
  Write(out, val.last_class);
  Write(out, val.max_class);
  Write(out, val.max_log_prob);
  size_t size = val.class_count.size();
  Write(out, size);
  for (const auto& [key, value] : val.class_count) {
    Write(out, key);
    Write(out, value);
  }
}

template <>
inline MetaData Read(std::istream& input) {
  MetaData meta;
  meta.last_class = Read<int32_t>(input);
  meta.max_class = Read<int32_t>(input);
  meta.max_log_prob = Read<int32_t>(input);

  size_t size = Read<size_t>(input);
  for (size_t i = 0; i < size; i++) {
    int32_t key = Read<int32_t>(input);
    int32_t value = Read<int32_t>(input);
    meta.class_count[key] = value;
  }
  return meta;
}

template <>
inline void Write(std::ostream& out, const Queue<int32_t, 5>& queue) {
  Write(out, queue.size());
  for (size_t i = 0; i < queue.size(); i++) {
    Write(out, queue[i]);
  }
}

template <>
inline Queue<int32_t, 5> Read(std::istream& input) {
  size_t size = Read<size_t>(input);
  Queue<int32_t, 5> queue;
  for (size_t i = 0; i < size; i++) {
    int32_t key = Read<int32_t>(input);
    queue.push(key);
  }
  return queue;
}

inline uint32_t read_header_length(std::istream& in) {
  uint32_t header_len = 0;
  in.read(reinterpret_cast<char*>(&header_len), sizeof(header_len));
  if (!in)
    throw std::runtime_error("Could not read header length");
  return header_len;
}

inline std::string get_header(const std::string& path) {
  std::ifstream file_in(path, std::ios::binary);
  if (!file_in.is_open()) {
    RCLCPP_WARN(Logging::logger, "Could not open map file %s", path.c_str());
    return "";
  }

  uint32_t header_len = read_header_length(file_in);

  std::string header(header_len,
                     '\0');  // pre-allocate string of header_len size
  file_in.read(&header[0], header_len);
  if (!file_in) {
    RCLCPP_WARN(Logging::logger, "Could not read header string");
    return "";
  }

  RCLCPP_DEBUG(Logging::logger, "Found header: %s at path %s", header.c_str(),
               path.c_str());

  return header;
}

template <typename B>
void serialize(std::ostream& out, const VoxelGrid<typename B::Voxel>& grid,
               std::string behavior_name) {
  // static_assert(std::is_trivially_copyable_v<typename B::Voxel>, "B::Voxel must be trivially copyable");

  std::string type_name = demangle(typeid(typename B::Voxel).name());

  std::stringstream header_ss;

  header_ss << "VoxelGrid"
            << " type_name=" << type_name << " behavior_name=" << behavior_name
            << " inner_bits=" << grid.innerBits()
            << " leaf_bits=" << grid.leafBits()
            << " resolution=" << grid.getResolution() << " timestamp="
            << std::to_string(
                   std::chrono::system_clock::now().time_since_epoch().count())
            << "\n";

  std::string header = header_ss.str();

  uint32_t header_len = header.size();
  out.write(reinterpret_cast<const char*>(&header_len), sizeof(header_len));

  out.write(header.data(), header.size());

  //------------
  Write(out, uint32_t(grid.rootMap().size()));

  for (const auto& it : grid.rootMap()) {
    const CoordT& root_coord = it.first;
    Write(out, root_coord.x);
    Write(out, root_coord.y);
    Write(out, root_coord.z);

    const auto& inner_grid = it.second;
    for (size_t w = 0; w < inner_grid.mask().wordCount(); w++) {
      Write(out, inner_grid.mask().getWord(w));
    }
    for (auto inner = inner_grid.mask().beginOn(); inner; ++inner) {
      const uint32_t inner_index = *inner;
      const auto& leaf_grid = *(inner_grid.cell(inner_index));

      for (size_t w = 0; w < leaf_grid.mask().wordCount(); w++) {
        Write(out, leaf_grid.mask().getWord(w));
      }
      for (auto leaf = leaf_grid.mask().beginOn(); leaf; ++leaf) {
        const uint32_t leaf_index = *leaf;
        Write(out, leaf_grid.cell(leaf_index));
      }

      const MetaData* leaf_meta = leaf_grid.meta_;
      Write(out, *leaf_meta);
    }
    const MetaData* inner_meta = inner_grid.meta_;
    Write(out, *inner_meta);
  }
}

template <typename B>
void save_to_file(const std::string& full_filename, const std::string& map_dir_,
                  Bonxai::VoxelGrid<typename B::Voxel>& grid,
                  std::string behavior_name) {
  if (!std::filesystem::exists(map_dir_)) {
    std::filesystem::create_directories(map_dir_);
  }

  auto path = std::filesystem::path(map_dir_) / full_filename;
  std::ostringstream ofile(path, std::ios::binary);
  serialize<B>(ofile, grid, behavior_name);

  std::ofstream file_out(path, std::ios::binary);
  if (file_out.is_open()) {
    file_out << ofile.str();
    file_out.close();
  } else {
    RCLCPP_WARN(Logging::logger, "Could not open file");
    return;
  }
}

template <typename B>
void MapImpl<B>::save_map(std::string filename) {
  if (filename != "") {
    auto path = std::filesystem::path(config.map_dir) /
                (filename + config.save_extension);
    RCLCPP_WARN(Logging::logger, "Saving map to %s ...",
                std::string(path).c_str());
    save_to_file<B>(filename + config.save_extension, config.map_dir, grid,
                    config.behavior);
    RCLCPP_WARN(Logging::logger, "Saved map to %s", std::string(path).c_str());
  } else {
    RCLCPP_WARN(Logging::logger, "Map not saved, no name provided");
  }
}

inline HeaderInfo get_header_info(const std::string path) {

  std::string header = get_header(path);

  std::regex re(
      R"(VoxelGrid type_name=([^\s]+)\s+behavior_name=([^\s]+)\s+inner_bits=(\d+)\s+leaf_bits=(\d+)\s+resolution=([-+eE0-9\.]+)\s+timestamp=([^\s]+))");
  std::smatch match;
  if (!std::regex_search(header, match, re)) {
    throw std::runtime_error("Header wasn't recognized. (REGEX)");
  }

  HeaderInfo info;
  info.type_name = match[1];
  info.behavior_name = match[2];
  info.inner_bits = std::stoi(match[3]);
  info.leaf_bits = std::stoi(match[4]);
  info.resolution = std::stod(match[5]);
  info.timestamp = match[6];
  return info;

  RCLCPP_INFO(Logging::logger,
              "Type_name: %s, Behavior_name: %s, Inner_bits: %d, Leaf_bits: "
              "%d, Resolution: %f"
              "Timestamp: %s",
              info.type_name.c_str(), info.behavior_name.c_str(),
              info.inner_bits, info.leaf_bits, info.resolution,
              info.timestamp.c_str());
  return info;
}

template <typename V>
inline VoxelGrid<V> Deserialize(std::string path) {

  std::ifstream file_in(path, std::ios::binary);
  if (!file_in.is_open()) {
    RCLCPP_WARN(Logging::logger,
                "Could not open map file %s | initializing empty grid (0.5 "
                "res, 2 inner, 3 leaf)",
                path.c_str());
    return VoxelGrid<V>(0.5, 2, 3);
  }

  std::ostringstream file_contents;
  file_contents << file_in.rdbuf();
  std::istringstream input(file_contents.str(), std::ios::binary);

  HeaderInfo info = get_header_info(path);
  std::string type_name = demangle(typeid(V).name());
  if (type_name != info.type_name) {
    throw std::runtime_error("Voxel type does not match");
  }

  // Skip encoded header length + header
  uint32_t header_len = read_header_length(input);
  input.seekg(header_len, std::ios::cur);

  VoxelGrid<V> grid(info.resolution, info.inner_bits, info.leaf_bits);

  uint32_t root_count = Read<uint32_t>(input);

  for (size_t root_index = 0; root_index < root_count; root_index++) {
    CoordT root_coord;
    root_coord.x = Read<int32_t>(input);
    root_coord.y = Read<int32_t>(input);
    root_coord.z = Read<int32_t>(input);

    auto inner_it = grid.rootMap().find(root_coord);
    if (inner_it == grid.rootMap().end()) {
      inner_it =
          grid.rootMap()
              .insert({root_coord,
                       typename VoxelGrid<V>::InnerGrid(info.inner_bits)})
              .first;
    }

    auto& inner_grid = inner_it->second;

    for (size_t w = 0; w < inner_grid.mask().wordCount(); w++) {
      uint64_t word = Read<uint64_t>(input);
      inner_grid.mask().setWord(w, word);
    }
    for (auto inner = inner_grid.mask().beginOn(); inner; ++inner) {
      auto& leaf_grid = inner_grid.cell(*inner);
      leaf_grid = grid.allocateLeafGrid();

      for (size_t w = 0; w < leaf_grid->mask().wordCount(); w++) {
        uint64_t word = Read<uint64_t>(input);
        leaf_grid->mask().setWord(w, word);
      }
      for (auto leaf = leaf_grid->mask().beginOn(); leaf; ++leaf) {
        const uint32_t leaf_index = *leaf;
        leaf_grid->cell(leaf_index) = Read<V>(input);
      }
      MetaData* leaf_meta_data = new MetaData(Read<MetaData>(input));
      leaf_grid->meta_ = leaf_meta_data;
    }

    MetaData* inner_meta_data = new MetaData(Read<MetaData>(input));
    inner_grid.meta_ = inner_meta_data;
  }

  return grid;
}

template <typename B>
void MapImpl<B>::load_grid(std::string path) {
  VoxelGrid<typename B::Voxel> loaded_grid =
      Deserialize<typename B::Voxel>(path);
  grid = std::move(loaded_grid);
}

template <>
inline void Write(std::ostream& out, const std::vector<float>& vec) {
  size_t size = vec.size();
  Write(out, size);
  for (size_t i = 0; i < size; ++i) {
    Write(out, vec[i]);
  }
}

template <>
inline std::vector<float> Read(std::istream& input) {
  size_t size = Read<size_t>(input);
  std::vector<float> vec(size);
  for (size_t i = 0; i < size; ++i) {
    vec[i] = Read<float>(input);
  }
  return vec;
}
