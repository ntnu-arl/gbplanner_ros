#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace gbplanner2 {

// Construct fixed-size PointCloud2 records in batches. Only the current cloud
// owns storage: the previous cloud contributes a count, never a retained copy.
// All bytes of every appended record are overwritten, including field padding.
template<std::size_t Stride>
class PointcloudBuffer {
 public:
  static_assert(Stride > 0, "A point record cannot be empty");
  explicit PointcloudBuffer(std::vector<std::uint8_t>& data,
                            std::size_t previous_points = 0) : data_(data) {
    data_.clear();
    const auto limit = data_.max_size() / Stride;
    if (previous_points > limit) throw std::length_error("Point cloud size overflow");
    if (previous_points) {
      // Modest headroom avoids copying a large growing map on each snapshot.
      // A reset/load discards the hint; finish() records the actual new count.
      const auto extra = std::min(previous_points / 16, limit - previous_points);
      data_.resize((previous_points + extra) * Stride);
    }
  }

  std::uint8_t* append(const void* record) {
    if (used_ == data_.size()) grow();
    auto* target = data_.data() + used_;
    std::memcpy(target, record, Stride);
    used_ += Stride;
    return target;
  }

  std::size_t finish() {
    // Expose only initialized records to ROS serialization; capacity is not
    // part of the message and remains owned by this single outgoing cloud.
    data_.resize(used_);
    return used_ / Stride;
  }

 private:
  void grow() {
    const auto points = data_.size() / Stride;
    const auto limit = data_.max_size() / Stride;
    if (points == limit) throw std::length_error("Point cloud size overflow");
    const auto extra = std::min(std::max<std::size_t>(4096, points / 2), limit - points);
    const auto bytes = (points + extra) * Stride;
    // reserve() controls the bound instead of vector::resize's usual doubling.
    // At most 50% spare records (or 4096 for small clouds), not a second map.
    data_.reserve(bytes);
    data_.resize(bytes);
  }

  std::vector<std::uint8_t>& data_;
  std::size_t used_{0};
};

}  // namespace gbplanner2
