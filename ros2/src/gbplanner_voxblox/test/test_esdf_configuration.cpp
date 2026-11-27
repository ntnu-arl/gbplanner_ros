#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "gbplanner_voxblox/tsdf_backend.hpp"

namespace {
gbplanner2::TsdfConfig config() {
  gbplanner2::TsdfConfig value;
  value.resolution = .25;
  value.voxels_per_side = 2;
  value.storage = "sparse";
  value.method = "simple";
  value.threads = 1;
  value.esdf_enabled = true;
  return value;
}

std::unique_ptr<voxblox::Layer<voxblox::TsdfVoxel>> uniformObserved(
    const gbplanner2::TsdfBackend& map, float distance = 1.5f) {
  auto layer = map.copyTsdfLayer();
  auto block = layer->allocateBlockPtrByIndex({0, 0, 0});
  block->has_data() = true;
  block->updated().set();
  for (std::size_t i = 0; i < block->num_voxels(); ++i) {
    auto& voxel = block->getVoxelByLinearIndex(i);
    voxel.distance = distance;
    voxel.weight = 1.f;
  }
  return layer;
}

double totalObservedWeight(const gbplanner2::TsdfBackend& map) {
  voxblox::BlockIndexList indices;
  map.tsdfLayer().getAllAllocatedBlocks(&indices);
  double result = 0.;
  for (const auto& index : indices) {
    const auto& block = map.tsdfLayer().getBlockByIndex(index);
    for (std::size_t i = 0; i < block.num_voxels(); ++i)
      result += block.getVoxelByLinearIndex(i).weight;
  }
  return result;
}

void expectSynchronizedEsdf(const gbplanner2::TsdfBackend& map) {
  ASSERT_NE(map.esdfLayer(), nullptr);
  voxblox::BlockIndexList indices;
  map.tsdfLayer().getAllAllocatedBlocks(&indices);
  std::size_t observed = 0, fixed = 0;
  for (const auto& index : indices) {
    const auto& tsdf = map.tsdfLayer().getBlockByIndex(index);
    EXPECT_FALSE(tsdf.updated().test(voxblox::Update::kEsdf));
    const auto esdf = map.esdfLayer()->getBlockPtrByIndex(index);
    ASSERT_NE(esdf, nullptr);
    for (std::size_t i = 0; i < tsdf.num_voxels(); ++i) {
      if (tsdf.getVoxelByLinearIndex(i).weight < 1e-6f) continue;
      const auto& voxel = esdf->getVoxelByLinearIndex(i);
      ++observed;
      EXPECT_TRUE(voxel.observed);
      EXPECT_TRUE(std::isfinite(voxel.distance));
      if (voxel.fixed) ++fixed;
    }
  }
  EXPECT_GT(observed, 0u);
  EXPECT_GT(fixed, 0u);
}

void updatePreservingOtherConsumers(gbplanner2::TsdfBackend& map) {
  const auto before = map.copyTsdfLayer();
  voxblox::BlockIndexList pending;
  before->getAllUpdatedBlocks(voxblox::Update::kEsdf, &pending);
  ASSERT_FALSE(pending.empty());
  map.updateEsdf();
  voxblox::BlockIndexList indices;
  before->getAllAllocatedBlocks(&indices);
  for (const auto& index : indices) {
    const auto& old_flags = before->getBlockByIndex(index).updated();
    const auto& flags = map.tsdfLayer().getBlockByIndex(index).updated();
    EXPECT_EQ(flags.test(voxblox::Update::kMesh), old_flags.test(voxblox::Update::kMesh));
    EXPECT_EQ(flags.test(voxblox::Update::kMap), old_flags.test(voxblox::Update::kMap));
  }
  expectSynchronizedEsdf(map);
}

void expectUnchangedEsdf(const voxblox::Layer<voxblox::EsdfVoxel>& before,
                         const voxblox::Layer<voxblox::EsdfVoxel>& after) {
  ASSERT_EQ(before.getNumberOfAllocatedBlocks(), after.getNumberOfAllocatedBlocks());
  voxblox::BlockIndexList indices;
  before.getAllAllocatedBlocks(&indices);
  for (const auto& index : indices) {
    const auto& previous = before.getBlockByIndex(index);
    const auto current = after.getBlockPtrByIndex(index);
    ASSERT_NE(current, nullptr);
    EXPECT_EQ(previous.updated(), current->updated());
    EXPECT_EQ(previous.has_data(), current->has_data());
    for (std::size_t i = 0; i < previous.num_voxels(); ++i) {
      const auto& old_voxel = previous.getVoxelByLinearIndex(i);
      const auto& voxel = current->getVoxelByLinearIndex(i);
      EXPECT_FLOAT_EQ(old_voxel.distance, voxel.distance);
      EXPECT_EQ(old_voxel.observed, voxel.observed);
      EXPECT_EQ(old_voxel.fixed, voxel.fixed);
      EXPECT_EQ(old_voxel.hallucinated, voxel.hallucinated);
      EXPECT_EQ(old_voxel.in_queue, voxel.in_queue);
      EXPECT_TRUE((old_voxel.parent.array() == voxel.parent.array()).all());
    }
  }
}

void expectCompletedTiming(const gbplanner2::TsdfIntegrationTiming& timing, bool esdf_updated) {
  EXPECT_TRUE(timing.completed);
  EXPECT_EQ(timing.esdf_updated, esdf_updated);
  for (const double duration : {timing.filter_wall_s, timing.tsdf_wall_s,
                                timing.esdf_wall_s, timing.total_wall_s}) {
    EXPECT_TRUE(std::isfinite(duration));
    EXPECT_GE(duration, 0.);
  }
  EXPECT_GE(timing.total_wall_s + 1e-12,
            timing.filter_wall_s + timing.tsdf_wall_s + timing.esdf_wall_s);
  if (!esdf_updated) EXPECT_DOUBLE_EQ(timing.esdf_wall_s, 0.);
}
}  // namespace

TEST(EsdfConfiguration, DeferredScansAccumulateUntilExplicitUpdate) {
  auto settings = config();
  settings.esdf_update_on_scan = false;
  gbplanner2::TsdfBackend map(settings);
  const std::array<Eigen::Vector3d, 3> first_batch{{
      {1.5, .15, .15}, {.15, 1.5, .15}, {-.15, -1.5, .15}}};
  double weight = 0.;
  for (const auto& endpoint : first_batch) {
    const auto scan = map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), {endpoint});
    EXPECT_EQ(scan.accepted_points, 1u);
    const double updated_weight = totalObservedWeight(map);
    EXPECT_GT(updated_weight, weight);
    weight = updated_weight;
    EXPECT_EQ(map.esdfLayer()->getNumberOfAllocatedBlocks(), 0u);
    expectCompletedTiming(map.lastIntegrationTiming(), false);
  }
  const auto last_scan = map.lastIntegrationTiming();
  updatePreservingOtherConsumers(map);
  EXPECT_DOUBLE_EQ(map.lastIntegrationTiming().total_wall_s, last_scan.total_wall_s);
  EXPECT_FALSE(map.lastIntegrationTiming().esdf_updated);

  // Existing ESDF blocks, not only the initially empty layer, remain frozen
  // while a second batch adds observations above and below the first rays.
  const auto first_esdf = map.copyEsdfLayer();
  for (const double height : {1.75, -1.75}) {
    map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), {{.15, .15, height}});
    const double updated_weight = totalObservedWeight(map);
    EXPECT_GT(updated_weight, weight);
    weight = updated_weight;
    expectUnchangedEsdf(*first_esdf, *map.esdfLayer());
  }
  updatePreservingOtherConsumers(map);
  EXPECT_GT(map.esdfLayer()->getNumberOfAllocatedBlocks(), first_esdf->getNumberOfAllocatedBlocks());
}

TEST(EsdfConfiguration, DefaultScanUpdatesEsdfAndSeparatesTiming) {
  gbplanner2::TsdfBackend map(config());
  EXPECT_TRUE(map.config().esdf_update_on_scan);
  EXPECT_FALSE(map.lastIntegrationTiming().completed);
  map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), {{1.5, .15, .15}});
  expectSynchronizedEsdf(map);
  expectCompletedTiming(map.lastIntegrationTiming(), true);

  // A rejected scan must not leave the previous successful measurement visible.
  EXPECT_THROW(map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), {}),
               std::invalid_argument);
  EXPECT_FALSE(map.lastIntegrationTiming().completed);
  EXPECT_FALSE(map.lastIntegrationTiming().esdf_updated);
  EXPECT_DOUBLE_EQ(map.lastIntegrationTiming().total_wall_s, 0.);

  map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), {{1.5, .15, .15}});
  map.reset();
  EXPECT_FALSE(map.lastIntegrationTiming().completed);
  EXPECT_DOUBLE_EQ(map.lastIntegrationTiming().total_wall_s, 0.);
}

TEST(EsdfConfiguration, EsdfPlanningRejectsDeferredScansEvenWhenImplicitlyEnabled) {
  for (const bool enabled : {false, true}) {
    auto settings = config();
    settings.esdf_enabled = enabled;
    settings.distance_field = "esdf";
    settings.esdf_update_on_scan = false;
    EXPECT_THROW({ gbplanner2::TsdfBackend map(settings); }, std::invalid_argument);
    settings.esdf_update_on_scan = true;
    gbplanner2::TsdfBackend map(settings);
    EXPECT_TRUE(map.config().esdf_enabled);
  }
}

TEST(EsdfConfiguration, DeferredModeStillAppliesRobotSpheresImmediately) {
  auto settings = config();
  settings.esdf_update_on_scan = false;
  settings.esdf_clear_sphere_for_planning = true;
  settings.esdf_clear_sphere_radius = .4;
  settings.esdf_occupied_sphere_radius = 1.;
  gbplanner2::TsdfBackend map(settings);
  map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), {{1.5, .15, .15}});
  const auto timing = map.lastIntegrationTiming();
  EXPECT_EQ(map.esdfLayer()->getNumberOfAllocatedBlocks(), 0u);
  map.addNewRobotPosition({0., 0., 0.});
  expectSynchronizedEsdf(map);
  std::size_t inner_free = 0, outer_occupied = 0;
  map.forEachObservedVoxel([&](const Eigen::Vector3i&, gbplanner2::VoxelStatus,
      const voxblox::TsdfVoxel*, const voxblox::EsdfVoxel* esdf) {
    if (!esdf || !esdf->hallucinated) return;
    if (esdf->distance > 0.f) ++inner_free;
    if (esdf->distance < 0.f) ++outer_occupied;
  });
  EXPECT_GT(inner_free, 0u);
  EXPECT_GT(outer_occupied, 0u);
  EXPECT_DOUBLE_EQ(map.lastIntegrationTiming().total_wall_s, timing.total_wall_s);
  EXPECT_FALSE(map.lastIntegrationTiming().esdf_updated);
}

TEST(EsdfConfiguration, DisabledEsdfUpdateIsNoOp) {
  for (const bool update_on_scan : {false, true}) {
    auto settings = config();
    settings.esdf_enabled = false;
    settings.esdf_update_on_scan = update_on_scan;
    gbplanner2::TsdfBackend map(settings);
    map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), {{1.5, .15, .15}});
    map.updateEsdf();
    EXPECT_EQ(map.esdfLayer(), nullptr);
    expectCompletedTiming(map.lastIntegrationTiming(), false);
    voxblox::BlockIndexList pending;
    map.tsdfLayer().getAllUpdatedBlocks(voxblox::Update::kEsdf, &pending);
    EXPECT_FALSE(pending.empty());
  }
}

TEST(EsdfConfiguration, SingleSeedSeparatesEuclideanAndQuasiEuclideanDistances) {
  for (const bool full_euclidean : {false, true}) {
    auto settings = config();
    settings.voxels_per_side = 8;
    settings.esdf_min_distance = .2;
    settings.esdf_full_euclidean_distance = full_euclidean;
    gbplanner2::TsdfBackend map(settings);
    auto layer = uniformObserved(map);
    layer->getBlockByIndex({0, 0, 0}).getVoxelByVoxelIndex({2, 2, 2}).distance = .125f;
    map.replaceTsdfLayer(std::move(layer));

    const auto* seed = map.esdfVoxel({.625, .625, .625});
    const auto* target = map.esdfVoxel({1.125, .875, .625});
    ASSERT_NE(seed, nullptr);
    ASSERT_NE(target, nullptr);
    ASSERT_TRUE(seed->fixed);
    EXPECT_FLOAT_EQ(seed->distance, .125f);
    EXPECT_TRUE(target->observed);
    EXPECT_FALSE(target->fixed);
    // Offset (2,1,0): Euclidean length sqrt(5), or one axial plus one
    // diagonal grid step. Add the positive seed's existing surface distance.
    const double grid_distance = full_euclidean ? std::sqrt(5.) : 1. + std::sqrt(2.);
    EXPECT_NEAR(target->distance, .125 + grid_distance * .25, 1e-5)
        << "full_euclidean=" << full_euclidean;
  }
}

TEST(EsdfConfiguration, FixedBandUsesHalfTruncationByDefaultAndStrictBoundary) {
  for (const bool explicit_band : {false, true}) {
    auto settings = config();
    settings.truncation_distance = 1.;
    if (explicit_band) settings.esdf_min_distance = .25;
    gbplanner2::TsdfBackend map(settings);
    auto layer = uniformObserved(map);
    auto& block = layer->getBlockByIndex({0, 0, 0});
    block.getVoxelByLinearIndex(0).distance = .375f;
    block.getVoxelByLinearIndex(1).distance = .5f;
    block.getVoxelByLinearIndex(2).distance = -.375f;
    map.replaceTsdfLayer(std::move(layer));
    const auto& result = map.esdfLayer()->getBlockByIndex({0, 0, 0});
    EXPECT_DOUBLE_EQ(map.config().esdf_min_distance, explicit_band ? .25 : .5);
    EXPECT_EQ(result.getVoxelByLinearIndex(0).fixed, !explicit_band);
    EXPECT_FALSE(result.getVoxelByLinearIndex(1).fixed);  // abs(d)==band is not fixed.
    EXPECT_EQ(result.getVoxelByLinearIndex(2).fixed, !explicit_band);
    if (!explicit_band) {
      EXPECT_FLOAT_EQ(result.getVoxelByLinearIndex(0).distance, .375f);
      EXPECT_FLOAT_EQ(result.getVoxelByLinearIndex(2).distance, -.375f);
    }
  }
}

TEST(EsdfConfiguration, UnseededObservedCellsUseConfiguredDefaultClampedToMaximum) {
  struct Case { double maximum, requested_default, expected; };
  for (const auto& example : std::array<Case, 2>{{{1., 2., 2.}, {3., .5, 3.}}}) {
    auto settings = config();
    settings.esdf_max_distance = example.maximum;
    settings.esdf_default_distance = example.requested_default;
    gbplanner2::TsdfBackend map(settings);
    map.replaceTsdfLayer(uniformObserved(map));
    const auto& block = map.esdfLayer()->getBlockByIndex({0, 0, 0});
    EXPECT_DOUBLE_EQ(map.config().esdf_default_distance, example.expected);
    for (std::size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      EXPECT_TRUE(voxel.observed);
      EXPECT_FALSE(voxel.fixed);
      EXPECT_FLOAT_EQ(voxel.distance, example.expected);
    }
  }
}

TEST(EsdfConfiguration, OccupiedCrustChangesUnknownCellsOnlyDuringBatchRebuild) {
  for (const bool crust : {false, true}) {
    auto settings = config();
    settings.esdf_add_occupied_crust = crust;
    gbplanner2::TsdfBackend map(settings);
    auto layer = map.copyTsdfLayer();
    auto block = layer->allocateBlockPtrByIndex({0, 0, 0});
    block->has_data() = true;
    block->updated().set();
    block->getVoxelByLinearIndex(0).distance = .125f;
    block->getVoxelByLinearIndex(0).weight = 1.f;
    map.replaceTsdfLayer(std::move(layer));
    const auto& result = map.esdfLayer()->getBlockByIndex({0, 0, 0});
    const auto& measured = result.getVoxelByLinearIndex(0);
    EXPECT_TRUE(measured.observed);
    EXPECT_TRUE(measured.fixed);
    EXPECT_FALSE(measured.hallucinated);
    EXPECT_FLOAT_EQ(measured.distance, .125f);
    for (std::size_t i = 1; i < result.num_voxels(); ++i) {
      const auto& voxel = result.getVoxelByLinearIndex(i);
      EXPECT_EQ(voxel.observed, crust);
      EXPECT_EQ(voxel.hallucinated, crust);
      EXPECT_FALSE(voxel.fixed);
      if (crust) EXPECT_LT(voxel.distance, 0.f);
      EXPECT_FLOAT_EQ(map.tsdfLayer().getBlockByIndex({0, 0, 0})
                          .getVoxelByLinearIndex(i).weight, 0.f);
    }
  }

  auto settings = config();
  settings.esdf_add_occupied_crust = true;
  gbplanner2::TsdfBackend incremental(settings);
  incremental.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), {{1.5, .15, .15}});
  voxblox::BlockIndexList indices;
  incremental.tsdfLayer().getAllAllocatedBlocks(&indices);
  std::size_t unknown_count = 0;
  for (const auto& index : indices) {
    const auto& tsdf = incremental.tsdfLayer().getBlockByIndex(index);
    const auto& esdf = incremental.esdfLayer()->getBlockByIndex(index);
    for (std::size_t i = 0; i < tsdf.num_voxels(); ++i) {
      if (tsdf.getVoxelByLinearIndex(i).weight >= 1e-6f) continue;
      ++unknown_count;
      EXPECT_FALSE(esdf.getVoxelByLinearIndex(i).observed);
      EXPECT_FALSE(esdf.getVoxelByLinearIndex(i).hallucinated);
    }
  }
  EXPECT_GT(unknown_count, 0u);  // The incremental call did encounter unknown cells.
}

TEST(EsdfConfiguration, MinimumDifferenceControlsStrictLocalDistanceLowering) {
  for (const double minimum_change : {1.5, 1.625}) {
    auto settings = config();
    settings.esdf_min_diff = minimum_change;
    gbplanner2::TsdfBackend map(settings);
    auto layer = uniformObserved(map);
    layer->getBlockByIndex({0, 0, 0}).getVoxelByLinearIndex(0).distance = .125f;
    map.replaceTsdfLayer(std::move(layer));
    const auto* neighbor = map.esdfVoxel({.375, .125, .125});
    ASSERT_NE(neighbor, nullptr);
    EXPECT_TRUE(neighbor->observed);
    EXPECT_FALSE(neighbor->fixed);
    // One axial step proposes .125+.25=.375 against the initial default 2.
    // An improvement equal to min_diff is retained; only a larger one lowers.
    EXPECT_FLOAT_EQ(neighbor->distance, minimum_change == 1.5 ? .375f : 2.f);
  }
}
