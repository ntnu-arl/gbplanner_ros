#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <utility>

#include "gbplanner_voxblox/tsdf_backend.hpp"

namespace {
gbplanner2::TsdfConfig config() {
  gbplanner2::TsdfConfig value;
  value.resolution = .25;
  value.voxels_per_side = 2;
  value.storage = "sparse";
  value.method = "fast";
  value.threads = 1;
  return value;
}
}  // namespace

TEST(BlockPruning, UsesFloatPositionAndBlockOriginsWithInclusiveBoundary) {
  gbplanner2::TsdfBackend map(config());
  auto layer = map.copyTsdfLayer();
  const std::array<voxblox::BlockIndex, 6> indices{{
      {0, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {4, 0, 0}}};
  for (const auto& index : indices) {
    auto block = layer->allocateBlockPtrByIndex(index);
    block->has_data() = true;
    block->updated().set();
    block->getVoxelByLinearIndex(0).weight = 2.f;
  }
  map.replaceTsdfLayer(std::move(layer));
  // In float arithmetic +/-0.5 minus 1e-8 still has magnitude 0.5.
  // A double-distance calculation would incorrectly remove the negative block.
  EXPECT_EQ(map.removeDistantTsdfBlocks({1e-8, 0., 0.}, .5), 2u);
  EXPECT_EQ(map.allocatedBlocks(), 4u);
  for (const auto& index : {voxblox::BlockIndex(0, 0, 0),
                            voxblox::BlockIndex(1, 0, 0),
                            voxblox::BlockIndex(-1, 0, 0),
                            voxblox::BlockIndex(0, 1, 0)}) {
    ASSERT_TRUE(map.tsdfLayer().hasBlock(index));
    const auto& block = map.tsdfLayer().getBlockByIndex(index);
    EXPECT_TRUE(block.has_data());
    EXPECT_TRUE(block.updated().all());
    EXPECT_FLOAT_EQ(block.getVoxelByLinearIndex(0).weight, 2.f);
  }
  EXPECT_FALSE(map.tsdfLayer().hasBlock({1, 1, 0}));
  EXPECT_FALSE(map.tsdfLayer().hasBlock({4, 0, 0}));
  // The block at origin (0.5,0,0) survives a zero radius. Its geometric
  // center (0.75,0.25,0.25) would not. Position first rounds to float 0.5.
  EXPECT_EQ(map.removeDistantTsdfBlocks({.5 + 1e-10, 0., 0.}, 0.), 3u);
  ASSERT_EQ(map.allocatedBlocks(), 1u);
  EXPECT_TRUE(map.tsdfLayer().hasBlock({1, 0, 0}));
  EXPECT_EQ(map.removeDistantTsdfBlocks({.5, 0., 0.}, 0.), 0u);
  EXPECT_EQ(map.removeDistantEsdfBlocks({.5, 0., 0.}, 0.), 0u);
}

TEST(BlockPruning, EsdfOnlySpheresArePrunedIndependentlyAfterPropagation) {
  auto settings = config();
  settings.esdf_enabled = true;
  settings.esdf_clear_sphere_for_planning = true;
  settings.esdf_clear_sphere_radius = .4;
  settings.esdf_occupied_sphere_radius = 1.;
  gbplanner2::TsdfBackend map(settings);
  map.addNewRobotPosition({4., 0., 0.});
  ASSERT_TRUE(map.esdfLayer()->hasBlock({8, 0, 0}));
  const auto far_count = map.esdfLayer()->getNumberOfAllocatedBlocks();
  EXPECT_EQ(map.allocatedBlocks(), 0u);
  EXPECT_EQ(map.removeDistantTsdfBlocks({0., 0., 0.}, .75), 0u);
  EXPECT_EQ(map.esdfLayer()->getNumberOfAllocatedBlocks(), far_count);
  // The caller's TSDF -> clear sphere -> ESDF sequence also handles cells
  // introduced solely by planning clearance, without a TSDF counterpart.
  map.addNewRobotPosition({0., 0., 0.});
  ASSERT_TRUE(map.esdfLayer()->hasBlock({0, 0, 0}));
  const auto before = map.esdfLayer()->getNumberOfAllocatedBlocks();
  const auto removed = map.removeDistantEsdfBlocks({0., 0., 0.}, .75);
  EXPECT_GT(removed, 0u);
  EXPECT_EQ(removed, before - map.esdfLayer()->getNumberOfAllocatedBlocks());
  EXPECT_FALSE(map.esdfLayer()->hasBlock({8, 0, 0}));
  EXPECT_TRUE(map.esdfLayer()->hasBlock({0, 0, 0}));
  EXPECT_TRUE(map.esdfLayer()->hasBlock({-1, 0, 0}));
  EXPECT_EQ(map.allocatedBlocks(), 0u);
  const auto* cleared = map.esdfVoxel({.125, .125, .125});
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->observed);
  EXPECT_TRUE(cleared->hallucinated);
  EXPECT_GT(cleared->distance, 0.f);

  // This invokes the retained integrators after the sphere's queues drained
  // and blocks were removed, rather than merely inspecting imported voxels.
  const auto scan = map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(),
                                 {{1.5, .15, .15}});
  EXPECT_EQ(scan.accepted_points, 1u);
  const auto* measured = map.esdfVoxel({1.375, .125, .125});
  ASSERT_NE(measured, nullptr);
  EXPECT_TRUE(measured->observed);
  EXPECT_FALSE(measured->hallucinated);
}

TEST(BlockPruning, FastIntegratorRepopulatesRemovedBlocksOnTheNextRealScan) {
  auto settings = config();
  settings.resolution = .2;
  settings.truncation_distance = .6;
  settings.esdf_enabled = true;
  gbplanner2::TsdfBackend map(settings);
  const std::vector<Eigen::Vector3d> points{{1.5, .15, .15}};
  map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), points);
  const auto* first = map.voxel({1.5, .1, .1});
  ASSERT_NE(first, nullptr);
  ASSERT_GT(first->weight, 0.f);
  const float first_distance = first->distance;
  const float first_weight = first->weight;
  const auto tsdf_count = map.allocatedBlocks();
  const auto esdf_count = map.esdfLayer()->getNumberOfAllocatedBlocks();
  ASSERT_GT(esdf_count, 0u);
  EXPECT_EQ(map.removeDistantTsdfBlocks({20., 0., 0.}, .1), tsdf_count);
  EXPECT_EQ(map.removeDistantEsdfBlocks({20., 0., 0.}, .1), esdf_count);
  EXPECT_EQ(map.allocatedBlocks(), 0u);
  EXPECT_EQ(map.esdfLayer()->getNumberOfAllocatedBlocks(), 0u);

  const auto scan = map.integrate({0., 0., 0.}, Eigen::Quaterniond::Identity(), points);
  EXPECT_EQ(scan.accepted_points, 1u);
  const auto* restored = map.voxel({1.5, .1, .1});
  ASSERT_NE(restored, nullptr);
  EXPECT_FLOAT_EQ(restored->distance, first_distance);
  EXPECT_FLOAT_EQ(restored->weight, first_weight);
  const auto* distance = map.esdfVoxel({1.5, .1, .1});
  ASSERT_NE(distance, nullptr);
  EXPECT_TRUE(distance->observed);
  EXPECT_FALSE(distance->hallucinated);
}

TEST(BlockPruning, RefreshesCachedPredecessorTrianglesAfterDeletingTheirNeighbor) {
  gbplanner2::TsdfBackend map(config());
  auto layer = map.copyTsdfLayer();
  const voxblox::BlockIndex near_index(0, 0, 0), far_index(1, 0, 0);
  for (const auto& index : {near_index, far_index}) {
    auto block = layer->allocateBlockPtrByIndex(index);
    block->has_data() = true;
    block->updated().set();
    for (std::size_t i = 0; i < block->num_voxels(); ++i) {
      auto& voxel = block->getVoxelByLinearIndex(i);
      voxel.weight = 1.f;
      voxel.distance = block->computeCoordinatesFromLinearIndex(i).y() - .25f;
      voxel.color = voxblox::Color(30, 90, 150);
    }
  }
  auto near_tsdf = layer->getBlockPtrByIndex(near_index);
  map.replaceTsdfLayer(std::move(layer));
  voxblox::MeshLayer meshes(.5f);
  map.updateMeshLayer(&meshes, false);
  auto& near_mesh = meshes.getMeshByIndex(near_index);
  auto& far_mesh = meshes.getMeshByIndex(far_index);
  ASSERT_EQ(near_mesh.vertices.size(), 12u);  // Interior plus positive-x border.
  ASSERT_EQ(far_mesh.vertices.size(), 6u);
  float old_max_x = 0.f;
  for (const auto& vertex : near_mesh.vertices) old_max_x = std::max(old_max_x, vertex.x());
  EXPECT_FLOAT_EQ(old_max_x, .625f);
  EXPECT_FALSE(near_tsdf->updated().test(voxblox::Update::kMesh));
  // Mimic consumption of the first published mesh and preserve distinct
  // non-mesh flags, so an accidental all-flags reset/set cannot pass.
  near_mesh.updated = far_mesh.updated = false;
  near_tsdf->updated().set(voxblox::Update::kMap);
  near_tsdf->updated().reset(voxblox::Update::kEsdf);

  voxblox::BlockIndexList removed{voxblox::BlockIndex(99, 99, 99)};
  EXPECT_EQ(map.removeDistantTsdfBlocks({0., 0., 0.}, .25, &removed), 1u);
  ASSERT_EQ(removed.size(), 1u);
  EXPECT_EQ(removed[0], far_index);
  EXPECT_TRUE(near_tsdf->updated().test(voxblox::Update::kMesh));
  EXPECT_TRUE(near_tsdf->updated().test(voxblox::Update::kMap));
  EXPECT_FALSE(near_tsdf->updated().test(voxblox::Update::kEsdf));

  // An undisplayed map must stay undisplayed, with kMesh still pending.
  voxblox::MeshLayer undisplayed(.5f);
  map.updateMeshBlocks(&undisplayed, {near_index});
  EXPECT_EQ(undisplayed.getNumberOfAllocatedMeshes(), 0u);
  EXPECT_TRUE(near_tsdf->updated().test(voxblox::Update::kMesh));

  map.updateMeshBlocks(&meshes, {near_index, near_index, far_index});
  ASSERT_EQ(near_mesh.vertices.size(), 6u);
  float new_max_x = 0.f;
  for (const auto& vertex : near_mesh.vertices) {
    new_max_x = std::max(new_max_x, vertex.x());
    EXPECT_FLOAT_EQ(vertex.y(), .25f);
  }
  EXPECT_FLOAT_EQ(new_max_x, .375f);  // The near block's final voxel center.
  EXPECT_TRUE(near_mesh.updated);
  EXPECT_FALSE(near_tsdf->updated().test(voxblox::Update::kMesh));
  EXPECT_TRUE(near_tsdf->updated().test(voxblox::Update::kMap));
  EXPECT_FALSE(near_tsdf->updated().test(voxblox::Update::kEsdf));
  // The backend repairs the surviving neighbor only. The mapper separately
  // emits the deletion tombstone for the far mesh and then removes its cache.
  EXPECT_EQ(meshes.getNumberOfAllocatedMeshes(), 2u);
  EXPECT_EQ(far_mesh.vertices.size(), 6u);
  EXPECT_FALSE(far_mesh.updated);
}
