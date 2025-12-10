#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <ostream>
using namespace std;

template <typename T, size_t N>
struct Queue {
  std::array<T, N> data;

  Queue() {
    data.fill(
        T{});  // Value initialization; for fundamental types, this means zero initialization.
  }

  // Overloaded constructor to initialize all elements with a specified default value.
  Queue(const T& defaultValue) { data.fill(defaultValue); }

  // passthrough access
  T& operator[](size_t index) { return data[index]; }
  const T& operator[](size_t index) const { return data[index]; }

  size_t size() const { return data.size(); }

  size_t mem_usage() const { return sizeof(data); }

  // Returns true if all elements are equal to the default-constructed value T{}
  // This matches the queue's initial state after default construction
  bool empty() const {
    const T defaultValue{};
    for (const auto& v : data) {
      if (v != defaultValue) {
        return false;
      }
    }
    return true;
  }

  void push(T label) {
    for (size_t i = 0; i < data.size() - 1; i++) {
      data[i] = data[i + 1];
    }
    data[data.size() - 1] = label;
  }

  T back() const { return data[data.size() - 1]; }

  bool valid(uint8_t x = 3) const {
    uint8_t currentCount = 0;
    uint8_t maxCount = 0;
    T currentClass = data[0];

    if (data.size() != 0) {
      for (size_t i = 0; i < data.size(); i++) {
        if (data[i] == currentClass) {
          currentCount++;
          maxCount = std::max(maxCount, currentCount);
        } else {
          currentCount = 1;
          currentClass = data[i];
        }
      }
    } else {
      return false;
    }
    if (maxCount >= x) {
      return true;
    } else {
      return false;
    }
  }
};
