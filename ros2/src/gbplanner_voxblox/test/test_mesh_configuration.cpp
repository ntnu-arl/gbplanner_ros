#include <gtest/gtest.h>

#include <memory>

#include "gbplanner_voxblox/tsdf_backend.hpp"

namespace {
const voxblox::BlockIndex kBlock(0, 0, 0);
const voxblox::Color kColor(30, 90, 150);

gbplanner2::TsdfConfig configuration() {
  gbplanner2::TsdfConfig config;
  config.resolution = .25;
  config.voxels_per_side = 4;
  config.truncation_distance = .5;
  config.storage = "sparse";
  config.method = "simple";
  config.mesh_min_weight = .5;
  return config;
}

std::unique_ptr<voxblox::Layer<voxblox::TsdfVoxel>> plane(float weight) {
  auto layer = std::make_unique<voxblox::Layer<voxblox::TsdfVoxel>>(.25f, 4);
  auto block = layer->allocateBlockPtrByIndex(kBlock);
  for (std::size_t i = 0; i < block->num_voxels(); ++i) {
    auto& voxel = block->getVoxelByLinearIndex(i);
    voxel.distance = block->computeCoordinatesFromLinearIndex(i).z() - .5f;
    voxel.weight = weight;
    voxel.color = kColor;
  }
  block->set_has_data(true);
  block->updated().set();
  return layer;
}

void expectPlane(const voxblox::Mesh& mesh, bool colors) {
  ASSERT_EQ(mesh.indices.size(), 54u);  // 3x3 crossing cells, two triangles each.
  ASSERT_FALSE(mesh.vertices.empty());
  EXPECT_EQ(mesh.normals.size(), mesh.vertices.size());
  for (const auto& point : mesh.vertices) {
    EXPECT_FLOAT_EQ(point.z(), .5f);
    EXPECT_GE(point.x(), .125f);
    EXPECT_LE(point.x(), .875f);
    EXPECT_GE(point.y(), .125f);
    EXPECT_LE(point.y(), .875f);
  }
  if (colors) {
    ASSERT_EQ(mesh.colors.size(), mesh.vertices.size());
    for (const auto& color : mesh.colors) {
      EXPECT_EQ(color.r, kColor.r);
      EXPECT_EQ(color.g, kColor.g);
      EXPECT_EQ(color.b, kColor.b);
    }
  } else {
    EXPECT_TRUE(mesh.colors.empty());
  }
}
}  // namespace

TEST(MeshConfiguration, StrictWeightThresholdAppliesToFullIncrementalAndCachedBlockMeshing) {
  gbplanner2::TsdfBackend backend(configuration());
  backend.replaceTsdfLayer(plane(1.f));
  expectPlane(backend.generateMesh(), true);
  voxblox::MeshLayer meshes(backend.tsdfLayer().block_size());
  backend.updateMeshLayer(&meshes, false);
  ASSERT_EQ(meshes.getNumberOfAllocatedMeshes(), 1u);
  expectPlane(meshes.getMeshByIndex(kBlock), true);

  // Equal-to-threshold weights remain observed TSDF, but cannot support any
  // Marching Cubes corner. The incremental path must erase the old surface.
  backend.replaceTsdfLayer(plane(.5f));
  EXPECT_TRUE(backend.generateMesh().indices.empty());
  backend.updateMeshLayer(&meshes, true);
  EXPECT_TRUE(meshes.getMeshByIndex(kBlock).vertices.empty());
  EXPECT_TRUE(meshes.getMeshByIndex(kBlock).updated);
  const auto* retained = backend.voxel({.125, .125, .375});
  ASSERT_NE(retained, nullptr);
  EXPECT_FLOAT_EQ(retained->weight, .5f);

  // The pruning repair uses a separate cached-block entry point. It must use
  // the same configuration in both directions, without allocating new meshes.
  backend.replaceTsdfLayer(plane(1.f));
  backend.updateMeshBlocks(&meshes, {kBlock});
  expectPlane(meshes.getMeshByIndex(kBlock), true);
  backend.replaceTsdfLayer(plane(.5f));
  backend.updateMeshBlocks(&meshes, {kBlock});
  EXPECT_TRUE(meshes.getMeshByIndex(kBlock).vertices.empty());
  EXPECT_EQ(meshes.getNumberOfAllocatedMeshes(), 1u);
  EXPECT_FALSE(backend.tsdfLayer().getBlockByIndex(kBlock).updated().test(voxblox::Update::kMesh));
}

TEST(MeshConfiguration, MeshColorSwitchPreservesTsdfColorAndGeometryOnEveryMeshingPath) {
  for (bool mesh_colors : {false, true}) {
    SCOPED_TRACE(::testing::Message() << "mesh_use_color=" << mesh_colors);
    auto config = configuration();
    config.use_color = true;
    config.mesh_use_color = mesh_colors;
    gbplanner2::TsdfBackend backend(config);
    const Eigen::Vector3d origin(.125, .125, .125), point(1.2, .2, .1);
    const auto integrated = backend.integrate(origin, Eigen::Quaterniond::Identity(), {point}, {kColor});
    ASSERT_EQ(integrated.accepted_points, 1u);
    const auto* measured = backend.voxel(origin + point);
    ASSERT_NE(measured, nullptr);
    ASSERT_GT(measured->weight, 0.f);
    EXPECT_EQ(measured->color.r, kColor.r);
    EXPECT_EQ(measured->color.g, kColor.g);
    EXPECT_EQ(measured->color.b, kColor.b);

    backend.replaceTsdfLayer(plane(1.f));
    expectPlane(backend.generateMesh(), mesh_colors);
    voxblox::MeshLayer meshes(backend.tsdfLayer().block_size());
    backend.updateMeshLayer(&meshes, true);
    expectPlane(meshes.getMeshByIndex(kBlock), mesh_colors);
    backend.replaceTsdfLayer(plane(1.f));
    backend.updateMeshBlocks(&meshes, {kBlock});
    expectPlane(meshes.getMeshByIndex(kBlock), mesh_colors);
    const auto* stored = backend.voxel({.125, .125, .375});
    ASSERT_NE(stored, nullptr);
    EXPECT_EQ(stored->color.r, kColor.r);
    EXPECT_EQ(stored->color.g, kColor.g);
    EXPECT_EQ(stored->color.b, kColor.b);
  }
}
