#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>

#include "gbplanner_voxblox/tsdf_backend.hpp"

namespace {
gbplanner2::TsdfConfig config() {
  gbplanner2::TsdfConfig value;
  value.resolution = .25;
  value.voxels_per_side = 8;
  value.storage = "sparse";
  value.method = "simple";
  value.threads = 1;
  value.clearing_weight = .1;
  return value;
}
}  // namespace

TEST(TsdfConfiguration, WeightingPoliciesKeepSymmetricThenConstantThenDropoffPriority) {
  struct Policy { bool symmetric, constant; };
  for (const auto& policy : std::array<Policy, 3>{{{true, true}, {false, true}, {false, false}}}) {
    auto settings = config();
    settings.truncation_distance = .5 * std::sqrt(3.);
    settings.use_symmetric_weight_dropoff = policy.symmetric;
    settings.use_const_weight = policy.constant;
    settings.weight_dropoff = true;
    gbplanner2::TsdfBackend map(settings);
    map.integrate(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), {{2., 2., 2.}});

    const auto* front = map.voxel({1.875, 1.875, 1.875});
    const auto* back = map.voxel({2.375, 2.375, 2.375});
    ASSERT_NE(front, nullptr);
    ASSERT_NE(back, nullptr);
    // The two centers have sdf=+.125*sqrt(3), -.375*sqrt(3).
    // Relative to tau=.5*sqrt(3), symmetric weights are .775 and .325.
    EXPECT_NEAR(front->weight, policy.symmetric ? .775 : 1., 1e-5);
    const double back_weight = policy.symmetric ? .325 : policy.constant ? 1. :
        (.125 * std::sqrt(3.)) / (settings.truncation_distance - settings.resolution);
    EXPECT_NEAR(back->weight, back_weight, 1e-5);
    EXPECT_NEAR(front->distance, .125 * std::sqrt(3.), 1e-5);
    EXPECT_NEAR(back->distance, -.375 * std::sqrt(3.), 1e-5);
  }
}

TEST(TsdfConfiguration, RangeWeightUsesSensorDepthAndSkipsUndefinedWeights) {
  for (const bool by_range : {false, true}) {
    auto settings = config();
    settings.use_const_weight = true;
    settings.weight_ray_by_range = by_range;
    gbplanner2::TsdfBackend map(settings);
    const auto result = map.integrate(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                                     {{2., 2., 2.}, {2., 2., 0.}});
    const auto* voxel = map.voxel({1.875, 1.875, 1.875});
    ASSERT_NE(voxel, nullptr);
    // z=2 gives 1/4. Using Euclidean range instead would give 1/12.
    EXPECT_NEAR(voxel->weight, by_range ? .25 : 1., 1e-5);
    EXPECT_EQ(result.accepted_points, by_range ? 1u : 2u);
    EXPECT_EQ(result.skipped_points, by_range ? 1u : 0u);
  }
}

TEST(TsdfConfiguration, AllowClearControlsOverRangeReturnsButNotExplicitFreeSpace) {
  for (const bool allow_clear : {false, true}) {
    auto settings = config();
    settings.max_range = 3.;
    settings.allow_clear = allow_clear;
    gbplanner2::TsdfBackend map(settings);
    map.integrate(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), {{4., 4., 4.}});
    EXPECT_EQ(map.status({.875, .875, .875}),
              allow_clear ? gbplanner2::VoxelStatus::Free : gbplanner2::VoxelStatus::Unknown);
    EXPECT_EQ(map.status({2.125, 2.125, 2.125}), gbplanner2::VoxelStatus::Unknown);
    if (!allow_clear) EXPECT_EQ(map.allocatedBlocks(), 0u);

    // Explicit free-space clouds retain the original exemption from allow_clear.
    map.integrateFreeSpace(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), {{4., 4., 4.}});
    EXPECT_EQ(map.status({.875, .875, .875}), gbplanner2::VoxelStatus::Free);
  }
  auto settings = config();
  settings.max_range = 3.;
  settings.allow_clear = true;
  settings.carving = false;
  gbplanner2::TsdfBackend no_carving(settings);
  EXPECT_FALSE(no_carving.config().allow_clear);
  no_carving.integrate(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), {{4., 4., 4.}});
  EXPECT_EQ(no_carving.allocatedBlocks(), 0u);
}

TEST(TsdfConfiguration, MixedAndSortedOrdersProduceDifferentSaturatedFusion) {
  for (const std::string order : {"mixed", "sorted"}) {
    auto settings = config();
    settings.integration_order_mode = order;
    settings.truncation_distance = 1.;
    settings.max_weight = 1.;
    settings.use_const_weight = true;
    gbplanner2::TsdfBackend map(settings);
    map.integrate({.125, .125, .125}, Eigen::Quaterniond::Identity(),
                  {{1.075, 1.075, 1.075}, {.975, .975, .975}, {.875, .875, .875}});
    const auto* voxel = map.voxel({.875, .875, .875});
    ASSERT_NE(voxel, nullptr);
    EXPECT_FLOAT_EQ(voxel->weight, 1.f);
    // The three sdf values are (.325,.225,.125)*sqrt(3). With capped
    // weight 1, the last return contributes half of the final value.
    // Mixed keeps input order for this short cloud; sorted reverses it.
    EXPECT_NEAR(voxel->distance, (order == "mixed" ? .2 : .25) * std::sqrt(3.), 1e-5);
  }
}

TEST(TsdfConfiguration, FastZeroBudgetSkipsFusionAndSubsamplingChangesAcceptedRays) {
  auto disabled_settings = config();
  disabled_settings.method = "fast";
  disabled_settings.max_integration_time_s = 0.;
  gbplanner2::TsdfBackend disabled(disabled_settings);
  disabled.integrate(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), {{1.1, 1.1, 1.1}});
  EXPECT_EQ(disabled.allocatedBlocks(), 0u);

  for (const double factor : {1., 2.}) {
    auto settings = config();
    settings.method = "fast";
    settings.use_const_weight = true;
    settings.start_voxel_subsampling_factor = factor;
    gbplanner2::TsdfBackend map(settings);
    map.integrate(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                  {{1.1, 1.1, 1.1}, {1.2, 1.2, 1.2}});
    const auto* voxel = map.voxel({1.125, 1.125, 1.125});
    ASSERT_NE(voxel, nullptr);
    // Both endpoints share a .25 cell, but occupy distinct .125 cells.
    // The query lies in both truncation bands, so collision pruning cannot
    // suppress its update. All instances keep the default per-frame reset.
    EXPECT_NEAR(voxel->weight, factor == 1. ? 1. : 2., 1e-5);
    EXPECT_NEAR(voxel->distance, (factor == 1. ? -.025 : .025) * std::sqrt(3.), 1e-5);
  }
}

TEST(TsdfConfiguration, MergedAntiGrazingPreservesAnotherReturnsEndpoint) {
  for (const bool anti_grazing : {false, true}) {
    auto settings = config();
    settings.method = "merged";
    settings.use_const_weight = true;
    settings.anti_grazing = anti_grazing;
    gbplanner2::TsdfBackend map(settings);
    map.integrate(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                  {{1.2, 1.2, 1.2}, {2.2, 2.2, 2.2}});
    const auto* voxel = map.voxel({1.125, 1.125, 1.125});
    ASSERT_NE(voxel, nullptr);
    const double measured_distance = .075 * std::sqrt(3.);
    EXPECT_NEAR(voxel->weight, anti_grazing ? 1. : 1.1, 1e-5);
    if (anti_grazing) EXPECT_NEAR(voxel->distance, measured_distance, 1e-5);
    else EXPECT_GT(voxel->distance, measured_distance + .01);
  }
}

TEST(TsdfConfiguration, UnspecifiedConstantPolicyResolvesOnceAndExplicitFalseIsRetained) {
  auto settings = config();
  settings.weight_dropoff = false;
  gbplanner2::TsdfBackend compatible(settings);
  ASSERT_TRUE(compatible.config().use_const_weight.has_value());
  EXPECT_TRUE(*compatible.config().use_const_weight);
  compatible.reset();
  EXPECT_TRUE(*compatible.config().use_const_weight);

  settings.use_const_weight = false;
  EXPECT_THROW(gbplanner2::TsdfBackend invalid(settings), std::invalid_argument);
  settings.use_symmetric_weight_dropoff = true;
  gbplanner2::TsdfBackend symmetric(settings);
  ASSERT_TRUE(symmetric.config().use_const_weight.has_value());
  EXPECT_FALSE(*symmetric.config().use_const_weight);
  symmetric.integrate(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), {{2., 2., 2.}});
  const auto* voxel = symmetric.voxel({1.875, 1.875, 1.875});
  ASSERT_NE(voxel, nullptr);
  EXPECT_NEAR(voxel->weight, .1 + .9 * (1. - .125 * std::sqrt(3.) / .75), 1e-5);
}
