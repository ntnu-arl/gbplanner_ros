// Copyright (c) 2016, ETHZ ASL. All rights reserved.
// Adapted from voxblox_ros/src/transformer.cc, fixed Voxblox commit
// 0ee88c26c203a3fe88e348dbbb552ab431a35352. BSD-3-Clause; LICENSE.voxblox.
#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include <voxblox/core/common.h>

namespace gbplanner2 {

struct TimedTransform {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t stamp_ns{0};
  voxblox::Transformation T_G_D;
};

class TransformQueue {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit TransformQueue(int64_t tolerance_ns = 1000000)
      : tolerance_ns_(tolerance_ns) {
    if (tolerance_ns < 0)
      throw std::invalid_argument("Transform timestamp tolerance must be nonnegative");
  }

  // Keep arrival order, as in the original transform callback. The caller is
  // responsible for ordered source stamps and for converting/validating poses.
  // The upstream ROS subscriber's depth of 40 does not bound this internal deque.
  void push(const TimedTransform& sample) { samples_.push_back(sample); }
  void clear() { samples_.clear(); }
  std::size_t size() const { return samples_.size(); }

  // On success, retain the matched sample or interpolation's old endpoint and
  // every later sample. On failure, leave both the queue and output unchanged.
  bool lookup(int64_t stamp_ns, voxblox::Transformation* T_G_D) {
    if (!T_G_D) throw std::invalid_argument("Missing transform lookup output");
    if (samples_.empty()) return false;

    bool matched = false;
    auto it = samples_.begin();
    for (; it != samples_.end(); ++it) {
      if (it->stamp_ns > stamp_ns) {
        matched = elapsed(it->stamp_ns, stamp_ns) < static_cast<uint64_t>(tolerance_ns_);
        break;
      }
      if (elapsed(stamp_ns, it->stamp_ns) < static_cast<uint64_t>(tolerance_ns_)) {
        matched = true;
        break;
      }
    }

    voxblox::Transformation result;
    if (matched) {
      // This is the first qualifying sample in queue order, not nearest time.
      result = it->T_G_D;
    } else {
      if (it == samples_.begin() || it == samples_.end()) return false;
      const auto newer = it;
      --it;
      const voxblox::FloatingPoint alpha =
          static_cast<voxblox::FloatingPoint>(elapsed(stamp_ns, it->stamp_ns)) /
          static_cast<voxblox::FloatingPoint>(elapsed(newer->stamp_ns, it->stamp_ns));
      // Use the pinned kindr implementation directly. Its Transformation log
      // contains translation followed by SO(3) log; replacing this with a full
      // SE(3) screw interpolation would change the original trajectory.
      const voxblox::Transformation::Vector6 difference =
          (it->T_G_D.inverse() * newer->T_G_D).log();
      result = it->T_G_D * voxblox::Transformation::exp(alpha * difference);
    }

    *T_G_D = result;
    samples_.erase(samples_.begin(), it);
    return true;
  }

 private:
  // The caller supplies high >= low at each use. Unsigned subtraction avoids
  // overflowing signed nanoseconds while preserving the original differences.
  static uint64_t elapsed(int64_t high, int64_t low) {
    return static_cast<uint64_t>(high) - static_cast<uint64_t>(low);
  }

  int64_t tolerance_ns_;
  voxblox::AlignedDeque<TimedTransform> samples_;
};

// T_B_D and T_B_C are already oriented according to their names; optional
// invert_T_B_D/invert_T_B_C parameter handling belongs to the caller.
inline voxblox::Transformation sensorTransform(
    const voxblox::Transformation& T_G_D,
    const voxblox::Transformation& T_B_D,
    const voxblox::Transformation& T_B_C) {
  return T_G_D * T_B_D.inverse() * T_B_C;
}

}  // namespace gbplanner2
