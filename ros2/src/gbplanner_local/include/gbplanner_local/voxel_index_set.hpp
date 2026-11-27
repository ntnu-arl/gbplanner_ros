#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>

namespace gbplanner2 {

// Exact membership for one snapshot, with no per-voxel allocation. There is
// no erase/iteration/rehash: the validated record count bounds unique keys.
class VoxelIndexSet {
 public:
  using Key = std::array<int, 3>;

  explicit VoxelIndexSet(std::size_t max_records)
      : max_records_(max_records), capacity_(capacityFor(max_records)),
        keys_(capacity_ ? new Key[capacity_] : nullptr),
        occupied_(capacity_ ? new std::uint8_t[capacity_]() : nullptr) {}

  VoxelIndexSet(const VoxelIndexSet&) = delete;
  VoxelIndexSet& operator=(const VoxelIndexSet&) = delete;
  VoxelIndexSet(VoxelIndexSet&&) = delete;
  VoxelIndexSet& operator=(VoxelIndexSet&&) = delete;

  // true: inserted; false: the complete coordinate was already present.
  // Exceeding the declared bound is an error, including for an empty set.
  // Even at the bound, another occurrence of an existing key returns false.
  bool insert(const Key& key) {
    if (!capacity_) throw std::length_error("Voxel index set record limit exceeded");
    std::size_t slot = hash(key) & (capacity_ - 1);
    for (std::size_t probes = 0; probes < capacity_; ++probes) {
      if (!occupied_[slot]) {
        if (size_ == max_records_)
          throw std::length_error("Voxel index set record limit exceeded");
        keys_[slot] = key;
        occupied_[slot] = 1;
        ++size_;
        return true;
      }
      if (keys_[slot] == key) return false;
      slot = (slot + 1) & (capacity_ - 1);
    }
    throw std::length_error("Voxel index set capacity exhausted");
  }

  bool contains(const Key& key) const noexcept {
    if (!capacity_) return false;
    std::size_t slot = hash(key) & (capacity_ - 1);
    for (std::size_t probes = 0; probes < capacity_; ++probes) {
      if (!occupied_[slot]) return false;
      if (keys_[slot] == key) return true;
      slot = (slot + 1) & (capacity_ - 1);
    }
    return false;
  }

  std::size_t size() const noexcept { return size_; }
  std::size_t capacity() const noexcept { return capacity_; }
  std::size_t storageBytes() const noexcept {
    return capacity_ * (sizeof(Key) + sizeof(std::uint8_t));
  }

  static std::size_t hash(const Key& key) noexcept {
    // Preserve the snapshot importer's 21-bit packing + SplitMix finalizer.
    // Unsigned addition is defined for every int, including INT_MIN/MAX.
    // Outside the sparse lattice different keys may hash alike; equality
    // always checks all three full integers, never only this packed value.
    constexpr std::uint64_t mask = (UINT64_C(1) << 21) - 1;
    const auto axis = [](int value) {
      return (static_cast<std::uint64_t>(value) + UINT64_C(1000000)) & mask;
    };
    std::uint64_t value = axis(key[0]) | (axis(key[1]) << 21) | (axis(key[2]) << 42);
    value = (value ^ (value >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)) * UINT64_C(0x94d049bb133111eb);
    return static_cast<std::size_t>(value ^ (value >> 31));
  }

 private:
  static std::size_t capacityFor(std::size_t max_records) {
    if (!max_records) return 0;
    // Separate key and occupancy arrays avoid per-slot padding
    // and do not initialize keys that will never be read. A 75% load ceiling
    // keeps linear probing bounded without a second allocation during import.
    constexpr auto max_slots = std::numeric_limits<std::size_t>::max() /
        (sizeof(Key) + sizeof(std::uint8_t));
    std::size_t capacity = 4;
    while (max_records > capacity - capacity / 4) {
      if (capacity > max_slots / 2)
        throw std::length_error("Voxel index set capacity overflow");
      capacity *= 2;
    }
    if (capacity > max_slots)
      throw std::length_error("Voxel index set allocation overflow");
    return capacity;
  }

  std::size_t max_records_;
  std::size_t capacity_;
  std::size_t size_{0};
  std::unique_ptr<Key[]> keys_;
  std::unique_ptr<std::uint8_t[]> occupied_;
};

}  // namespace gbplanner2
