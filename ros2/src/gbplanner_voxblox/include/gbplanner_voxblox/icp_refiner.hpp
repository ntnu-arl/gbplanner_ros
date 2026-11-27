// Copyright (c) 2016, ETHZ ASL. All rights reserved.
// Adapts TsdfServer::processPointCloudMessageAndInsert from fixed Voxblox
// commit 0ee88c26c203a3fe88e348dbbb552ab431a35352. BSD-3-Clause;
// see LICENSE.voxblox. The ICP algorithm remains in the original core library.
#pragma once

#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>

#include <voxblox/alignment/icp.h>

namespace gbplanner2 {

// Serial mapper state: one correction is carried between incoming clouds.
// The caller retains the raw pose for upstream callbacks and uses refined for
// integration; publishing the correction and its acquisition stamp is external.
class IcpRefiner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Result {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    voxblox::Transformation refined;
    voxblox::Transformation correction;
    std::size_t successful_batches{0};
  };

  explicit IcpRefiner(const voxblox::ICP::Config& config, bool accumulate = true)
      : icp_(validatedConfig(config)), accumulate_(accumulate) {}

  Result refine(const voxblox::Layer<voxblox::TsdfVoxel>& layer,
                 const voxblox::Pointcloud& points,
                 const voxblox::Transformation& raw,
                 std::optional<unsigned> seed = std::nullopt) {
    if (!accumulate_) correction_.setIdentity();
    Result result;
    // With zero matches the original ICP still returns its supplied initial
    // transform. Preserve that result, including any accumulated correction.
    if (seed) {
      result.successful_batches = icp_.runICP(
          layer, points, correction_ * raw, &result.refined, *seed);
    } else {
      result.successful_batches = icp_.runICP(
          layer, points, correction_ * raw, &result.refined);
    }
    correction_ = result.refined * raw.inverse();
    if (!icp_.refiningRollPitch()) {
      // Keep the original order: sanitize the correction carried forward and
      // published to TF, while leaving this scan's refined pose as ICP returned.
      auto tangent = correction_.log();
      tangent[3] = 0.f;
      tangent[4] = 0.f;
      correction_ = voxblox::Transformation::exp(tangent);
    }
    result.correction = correction_;
    return result;
  }

  void clear() { correction_.setIdentity(); }

 private:
  static voxblox::ICP::Config validatedConfig(const voxblox::ICP::Config& config) {
    const auto valid_ratio = [](float value) {
      return std::isfinite(value) && value > 0.f && value <= 1.f;
    };
    const auto valid_weight = [](float value) {
      return std::isfinite(value) && value > 0.f;
    };
    if (config.num_threads == 0 || config.mini_batch_size < 3 ||
        !valid_ratio(config.min_match_ratio) || !valid_ratio(config.subsample_keep_ratio) ||
        !valid_weight(config.inital_translation_weighting) ||
        !valid_weight(config.inital_rotation_weighting))
      throw std::invalid_argument("Invalid ICP thread count, batch size, ratio or initial weight");
    return config;
  }

  voxblox::ICP icp_;
  bool accumulate_;
  voxblox::Transformation correction_;
};

}  // namespace gbplanner2
