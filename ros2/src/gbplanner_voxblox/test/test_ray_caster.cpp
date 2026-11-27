#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <set>
#include <vector>

#include "gbplanner_voxblox/tsdf_backend.hpp"
#include "voxblox/integrator/integrator_utils.h"

namespace {
using Cell = std::array<int64_t, 3>;
using Cells = std::vector<Cell>;

Cells traverse(const voxblox::Point& start, const voxblox::Point& end) {
  voxblox::RayCaster ray(start, end);
  Cells cells;
  // All fixtures contain fewer than 32 cells. Bound collection so a broken
  // termination rule produces a useful failure instead of hanging the test.
  for (int step = 0; step < 128; ++step) {
    voxblox::GlobalIndex index;
    if (!ray.nextRayIndex(&index)) return cells;
    cells.push_back({index.x(), index.y(), index.z()});
  }
  ADD_FAILURE() << "RayCaster did not terminate within the fixture limit";
  return cells;
}

void expectSequence(const voxblox::Point& start, const voxblox::Point& end,
                    const Cells& expected) {
  const auto actual = traverse(start, end);
  EXPECT_EQ(actual, expected);
  EXPECT_EQ(std::set<Cell>(actual.begin(), actual.end()).size(), actual.size())
      << "A ray must not repeatedly report a stationary-axis voxel";
}
}  // namespace

TEST(RayCaster, TraversesAllSixAxesFromVoxelInteriorsAndBoundaries) {
  for (const auto& start : {voxblox::Point(.5f, .5f, .5f),
                            voxblox::Point(0.f, 0.f, 0.f)}) {
    for (int axis = 0; axis < 3; ++axis) {
      for (int direction : {-1, 1}) {
        SCOPED_TRACE(::testing::Message() << "start=" << start.transpose()
                     << ", axis=" << axis << ", direction=" << direction);
        auto end = start;
        end[axis] += direction * 4.f;
        Cells expected;
        for (int offset = 0; offset <= 4; ++offset) {
          Cell cell{0, 0, 0};
          cell[axis] = direction * offset;
          expected.push_back(cell);
        }
        // At an exact boundary, the starting cell is still returned first.
        // A negative ray then crosses into the preceding cell at t=0.
        expectSequence(start, end, expected);
      }
    }
  }
}

TEST(RayCaster, PreservesNonAxisAlignedTraversalAndSimultaneousCrossingOrder) {
  // All three direction components are nonzero. These explicit sequences
  // preserve the upstream DDA order without reimplementing it in the test.
  expectSequence({.5f, .5f, .5f}, {4.5f, 2.5f, 1.5f},
                 {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {2, 1, 0},
                  {2, 1, 1}, {3, 1, 1}, {3, 2, 1}, {4, 2, 1}});
  // Equal crossing times retain Eigen minCoeff's first-axis tie selection.
  expectSequence({.5f, .5f, .5f}, {2.5f, 2.5f, 2.5f},
                 {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {1, 1, 1},
                  {2, 1, 1}, {2, 2, 1}, {2, 2, 2}});
}

TEST(RayCaster, ZeroLengthScaledRayReturnsItsCellOnce) {
  expectSequence({.5f, -.5f, 1.5f}, {.5f, -.5f, 1.5f}, {{0, -1, 1}});
  expectSequence({0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {{0, 0, 0}});
}

TEST(RayCaster, AxisAlignedFreespaceRayObservesTheCorridorBeyondItsOrigin) {
  gbplanner2::TsdfConfig config;
  config.resolution = .05;
  config.voxels_per_side = 8;
  config.truncation_distance = .1;
  config.min_range = .01;
  config.max_range = 2.;
  config.method = "simple";
  config.storage = "sparse";
  gbplanner2::TsdfBackend backend(config);
  // This strict Y ray reproduced the original failure: only the origin voxel
  // became observed, despite a one-metre free-space measurement.
  const auto result = backend.integrateFreeSpace(
      {.025, .025, .025}, Eigen::Quaterniond::Identity(), {{0., 1., 0.}});
  ASSERT_EQ(result.accepted_points, 1u);
  EXPECT_EQ(result.skipped_points, 0u);
  for (double y : {.225, .525, .825}) {
    SCOPED_TRACE(::testing::Message() << "corridor y=" << y);
    const auto* voxel = backend.voxel({.025, y, .025});
    ASSERT_NE(voxel, nullptr);
    EXPECT_GT(voxel->weight, 0.f);
    EXPECT_GT(voxel->distance, 0.f);
  }
  std::size_t observed = 0;
  backend.forEachObservedVoxel(
      [&](const Eigen::Vector3i&, gbplanner2::VoxelStatus,
          const voxblox::TsdfVoxel*, const voxblox::EsdfVoxel*) { ++observed; });
  EXPECT_GE(observed, 18u);
}
