#pragma once

#include <cstdint>
#include <limits>

namespace gbplanner2 {

// Upstream augment_free_voxels_time is a graph-build count, not a duration.
// Keep this object for the node lifetime: ordinary graph/task resets do not
// restart the odometry window. The separate non-free root-box augmentation
// in Rrg::reset is not subject to this window.
class UnknownAugmentationSchedule {
 public:
  explicit UnknownAugmentationSchedule(std::uint64_t build_limit=5)
      : build_limit_(build_limit) {}

  bool odometryWindowOpen() const { return completed_builds_<build_limit_; }
  std::uint64_t completedBuilds() const { return completed_builds_; }
  std::uint64_t buildLimit() const { return build_limit_; }

  // Pass PlanResult::graph_build_completed, not its path-success flag.
  void recordGraphBuild(bool completed) {
    if(completed&&completed_builds_<std::numeric_limits<std::uint64_t>::max())
      ++completed_builds_;
  }

 private:
  std::uint64_t build_limit_;
  std::uint64_t completed_builds_{0};
};

}  // namespace gbplanner2
