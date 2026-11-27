#pragma once

#include "gbplanner_local/voxel_index_set.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace gbplanner2 {

// Compact exact-key storage: entries are contiguous, and each occupied hash
// slot contains entry_index + 1. The zero slot value denotes an empty bucket.
// Insertion/reserve can invalidate value pointers; erase can move the last
// entry into the erased entry's position. Const queries allocate and modify
// nothing, so concurrent read-only queries on an unchanged table are safe.
template <class Value>
class VoxelIndexTable {
  static_assert(std::is_trivially_copyable<Value>::value,
                "VoxelIndexTable values must be trivially copyable");
  static_assert(std::is_nothrow_copy_constructible<Value>::value &&
                std::is_nothrow_copy_assignable<Value>::value,
                "VoxelIndexTable values require nonthrowing copying");

 public:
  using Key = std::array<int, 3>;
  struct Entry { Key key; Value value; };

  VoxelIndexTable() = default;
  VoxelIndexTable(const VoxelIndexTable&) = default;
  // Copy assignment must commit both vectors together if allocation fails.
  VoxelIndexTable& operator=(const VoxelIndexTable& other) {
    if (this != &other) {
      VoxelIndexTable copy(other);
      swap(copy);
    }
    return *this;
  }
  // Explicitly leave moved-from tables empty and reusable, rather than
  // depending on two independent vectors' unspecified moved-from contents.
  VoxelIndexTable(VoxelIndexTable&& other) noexcept { swap(other); }
  VoxelIndexTable& operator=(VoxelIndexTable&& other) noexcept {
    if (this != &other) {
      VoxelIndexTable moved(std::move(other));
      swap(moved);
    }
    return *this;
  }

  Value* find(const Key& key) noexcept {
    return const_cast<Value*>(std::as_const(*this).find(key));
  }
  const Value* find(const Key& key) const noexcept {
    if (slots_.empty()) return nullptr;
    const auto index = slots_[locate(key)];
    return index ? &entries_[index - 1].value : nullptr;
  }

  Value& operator[](const Key& key) {
    std::size_t slot = 0;
    if (!slots_.empty()) {
      slot = locate(key);
      if (slots_[slot]) return entries_[slots_[slot] - 1].value;
    }
    if (entries_.size() == maxRecordCount())
      throw std::length_error("Voxel index table entry count overflow");
    // Construct before allocating or rehashing, so a throwing default
    // constructor leaves the entire table untouched. Do not inspect default
    // constructibility at class scope: a nested Value with member initializers
    // may be instantiated before its enclosing class has finished definition.
    const Entry entry{key, Value{}};
    if (entries_.size() == entries_.capacity() ||
        entries_.size() >= slots_.size() - slots_.size() / 4) {
      reserve(entries_.size() + 1);
      slot = locate(entry.key);
    }
    // Capacity is already available, and copying cannot throw.
    // Commit the slot only after its referenced entry exists.
    entries_.push_back(entry);
    slots_[slot] = static_cast<Slot>(entries_.size());
    return entries_.back().value;
  }

  bool erase(const Key& key) noexcept {
    if (slots_.empty()) return false;
    std::size_t hole = locate(key);
    if (!slots_[hole]) return false;
    const std::size_t removed = slots_[hole] - 1;
    const std::size_t last = entries_.size() - 1;
    if (removed != last) {
      // Locate the moved entry before changing either entry's key. Its hash
      // slot must refer to the new dense index before repairing the cluster.
      const std::size_t moved_slot = locate(entries_.back().key);
      entries_[removed] = entries_.back();
      slots_[moved_slot] = static_cast<Slot>(removed + 1);
    }
    entries_.pop_back();
    const std::size_t mask = slots_.size() - 1;
    for (std::size_t next = (hole + 1) & mask; slots_[next]; next = (next + 1) & mask) {
      const std::size_t home = VoxelIndexSet::hash(entries_[slots_[next] - 1].key) & mask;
      // Move a displaced record only when its probe path crosses this hole.
      // Modular distances handle a cluster spanning the end of the array.
      if (((next - home) & mask) >= ((next - hole) & mask)) {
        slots_[hole] = slots_[next];
        hole = next;
      }
    }
    slots_[hole] = 0;
    return true;
  }

  void reserve(std::size_t count) {
    if (count > maxRecordCount())
      throw std::length_error("Voxel index table entry count overflow");
    if (count <= entries_.capacity() && count <= slots_.size() - slots_.size() / 4)
      return;
    const std::size_t capacity = std::max(slots_.size(), capacityFor(count));
    std::vector<Slot> replacement;
    if (capacity > slots_.size()) {
      replacement.assign(capacity, 0);
      const std::size_t mask = capacity - 1;
      for (std::size_t index = 0; index < entries_.size(); ++index) {
        std::size_t slot = VoxelIndexSet::hash(entries_[index].key) & mask;
        while (replacement[slot]) slot = (slot + 1) & mask;
        replacement[slot] = static_cast<Slot>(index + 1);
      }
    }
    // Geometric entry growth follows the 75% slot ceiling, avoiding a new
    // entry allocation per operator[]. Build new slots first; if either
    // allocation fails, all existing contents and associations remain intact.
    entries_.reserve(std::min(capacity - capacity / 4, maxRecordCount()));
    if (!replacement.empty()) slots_.swap(replacement);
  }

  std::size_t size() const noexcept { return entries_.size(); }

 private:
  using Slot = std::uint32_t;

  std::size_t maxRecordCount() const noexcept {
    return std::min(entries_.max_size(),
                    static_cast<std::size_t>(std::numeric_limits<Slot>::max()));
  }
  std::size_t capacityFor(std::size_t count) const {
    if (!count) return 0;
    const std::size_t limit = slots_.max_size();
    std::size_t capacity = 4;
    while (count > capacity - capacity / 4) {
      if (capacity > limit / 2)
        throw std::length_error("Voxel index table slot capacity overflow");
      capacity *= 2;
    }
    if (capacity > limit)
      throw std::length_error("Voxel index table slot allocation overflow");
    return capacity;
  }
  std::size_t locate(const Key& key) const noexcept {
    const std::size_t mask = slots_.size() - 1;
    std::size_t slot = VoxelIndexSet::hash(key) & mask;
    // At most 75% of slots are occupied: every missing-key search terminates
    // at an empty slot. Hash equality alone never implies key equality.
    while (slots_[slot] && entries_[slots_[slot] - 1].key != key)
      slot = (slot + 1) & mask;
    return slot;
  }
  void swap(VoxelIndexTable& other) noexcept {
    entries_.swap(other.entries_);
    slots_.swap(other.slots_);
  }

  std::vector<Entry> entries_;
  std::vector<Slot> slots_;
};

}  // namespace gbplanner2
