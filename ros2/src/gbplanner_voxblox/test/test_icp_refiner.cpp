#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "gbplanner_voxblox/icp_refiner.hpp"

namespace {
using Layer = voxblox::Layer<voxblox::TsdfVoxel>;

voxblox::ICP::Config configuration() {
  voxblox::ICP::Config config;
  config.num_threads = 1;
  config.subsample_keep_ratio = 1.f;
  return config;
}

std::unique_ptr<Layer> plane() {
  auto layer = std::make_unique<Layer>(.05f, 8);
  // Analytic x=2 m plane. Fully observed neighboring cells provide central
  // finite differences throughout the input cloud and its shifted initial pose.
  for (int x = 3; x <= 6; ++x)
    for (int y = -3; y <= 2; ++y)
      for (int z = -3; z <= 2; ++z) {
        auto block = layer->allocateBlockPtrByIndex({x, y, z});
        for (std::size_t i = 0; i < block->num_voxels(); ++i) {
          auto& voxel = block->getVoxelByLinearIndex(i);
          voxel.distance = block->computeCoordinatesFromLinearIndex(i).x() - 2.f;
          voxel.weight = 1.f;
        }
        block->set_has_data(true);
      }
  return layer;
}

voxblox::Pointcloud planeReturns() {
  voxblox::Pointcloud points;
  for (int y = -8; y <= 8; ++y)
    for (int z = -8; z <= 8; ++z)
      points.emplace_back(2.f, y * .1f, z * .1f);
  return points;
}

float meanPlaneError(const voxblox::Pointcloud& points,
                     const voxblox::Transformation& transform) {
  float error = 0.f;
  for (const auto& point : points) error += std::abs((transform * point).x() - 2.f);
  return error / points.size();
}

void expectSameTransform(const voxblox::Transformation& actual,
                         const voxblox::Transformation& expected) {
  EXPECT_LT((actual.getPosition() - expected.getPosition()).norm(), 1e-5f);
  for (const voxblox::Point& basis : {voxblox::Point(1.f, 0.f, 0.f),
                                    voxblox::Point(0.f, 1.f, 0.f),
                                    voxblox::Point(0.f, 0.f, 1.f)})
    EXPECT_LT((actual.getRotation().rotate(basis) -
               expected.getRotation().rotate(basis)).norm(), 1e-5f);
}
}  // namespace

TEST(IcpRefiner, ReducesMeasuredPlaneErrorAndMatchesTheOriginalCore) {
  const auto map = plane();
  const auto points = planeReturns();
  const auto config = configuration();
  const voxblox::Transformation raw(voxblox::Rotation(), voxblox::Point(.2f, 0.f, 0.f));
  gbplanner2::IcpRefiner refiner(config);
  const auto result = refiner.refine(*map, points, raw, 42u);
  const float before = meanPlaneError(points, raw);
  const float after = meanPlaneError(points, result.refined);
  EXPECT_NEAR(before, .2f, 1e-5f);
  ASSERT_GT(result.successful_batches, 0u);
  EXPECT_LT(after, before * .5f);
  EXPECT_LT(result.correction.getPosition().x(), -.1f);
  EXPECT_NEAR(result.correction.log()[3], 0.f, 1e-7f);
  EXPECT_NEAR(result.correction.log()[4], 0.f, 1e-7f);

  // An independent original ICP instance confirms the wrapper preserves the
  // single-scan algorithm; the measured residual above establishes usefulness.
  voxblox::ICP original(config);
  voxblox::Transformation original_refined;
  const auto batches = original.runICP(*map, points, raw, &original_refined, 42u);
  EXPECT_EQ(result.successful_batches, batches);
  expectSameTransform(result.refined, original_refined);
}

TEST(IcpRefiner, AccumulationAndClearDetermineThePoseWhenTheNextMapHasNoMatches) {
  const auto map = plane();
  const auto points = planeReturns();
  const auto config = configuration();
  const voxblox::Transformation first_raw(voxblox::Rotation(), voxblox::Point(.2f, 0.f, 0.f));
  gbplanner2::IcpRefiner accumulated(config, true), independent(config, false);
  const auto first = accumulated.refine(*map, points, first_raw, 42u);
  const auto independent_first = independent.refine(*map, points, first_raw, 42u);
  ASSERT_GT(first.successful_batches, 0u);
  ASSERT_GT(independent_first.successful_batches, 0u);

  Layer unknown(.05f, 8);
  const Eigen::Quaternionf orientation =
      Eigen::AngleAxisf(.07f, voxblox::Point::UnitZ()) *
      Eigen::AngleAxisf(-.08f, voxblox::Point::UnitY()) *
      Eigen::AngleAxisf(.1f, voxblox::Point::UnitX());
  const voxblox::Transformation next_raw(voxblox::Rotation(orientation),
                                        voxblox::Point(.35f, .1f, 0.f));
  const auto continued = accumulated.refine(unknown, points, next_raw, 42u);
  const auto fresh = independent.refine(unknown, points, next_raw, 42u);
  EXPECT_EQ(continued.successful_batches, 0u);
  EXPECT_EQ(fresh.successful_batches, 0u);
  expectSameTransform(continued.refined, first.correction * next_raw);
  expectSameTransform(fresh.refined, next_raw);
  // Raw roll/pitch belongs to the measurement pose and must survive even when
  // correction roll/pitch refinement is disabled. Zero matches still gives a
  // usable integration pose instead of dropping the pointcloud.
  EXPECT_GT(std::abs(continued.refined.getRotation().log().x()), .05f);
  EXPECT_NEAR(continued.correction.log()[3], 0.f, 1e-7f);
  EXPECT_NEAR(continued.correction.log()[4], 0.f, 1e-7f);

  accumulated.clear();
  const auto cleared = accumulated.refine(unknown, points, next_raw, 42u);
  EXPECT_EQ(cleared.successful_batches, 0u);
  expectSameTransform(cleared.refined, next_raw);
}
