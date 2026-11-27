#include <gtest/gtest.h>

#include <array>
#include <utility>

#include "gbplanner_voxblox/mesh_messages.hpp"
#include "gbplanner_voxblox/tsdf_backend.hpp"

namespace {
const voxblox::BlockIndex kChanged(0, 0, 0);
const voxblox::BlockIndex kUnchanged(4, 0, 0);

const voxblox_msgs::msg::MeshBlock* findBlock(const voxblox_msgs::msg::Mesh& message,
                                            const voxblox::BlockIndex& index) {
  for (const auto& block : message.mesh_blocks)
    if (block.index[0] == index.x() && block.index[1] == index.y() &&
        block.index[2] == index.z()) return &block;
  return nullptr;
}

void expectOnlyMeshConsumed(const gbplanner2::TsdfBackend& backend,
                            const voxblox::BlockIndex& index) {
  const auto& flags = backend.tsdfLayer().getBlockByIndex(index).updated();
  EXPECT_FALSE(flags.test(voxblox::Update::kMesh));
  EXPECT_TRUE(flags.test(voxblox::Update::kMap));
  EXPECT_TRUE(flags.test(voxblox::Update::kEsdf));
}
}  // namespace

TEST(IncrementalMesh, PersistentPlanesConsumeOnlyMeshFlagsAndDeleteOnlyChangedBlock) {
  gbplanner2::TsdfConfig config;
  config.resolution = .25;
  config.voxels_per_side = 2;
  config.storage = "sparse";
  config.esdf_enabled = false;
  config.threads = 1;
  gbplanner2::TsdfBackend backend(config);
  auto initial = backend.copyTsdfLayer();
  const std::array<voxblox::BlockIndex, 2> indices{{kChanged, kUnchanged}};
  for (const auto& index : indices) {
    auto block = initial->allocateBlockPtrByIndex(index);
    block->has_data() = true;
    const float plane_x = .5f * index.x() + .25f;
    for (std::size_t i = 0; i < block->num_voxels(); ++i) {
      auto& voxel = block->getVoxelByLinearIndex(i);
      voxel.distance = block->computeCoordinatesFromLinearIndex(i).x() - plane_x;
      voxel.weight = 1.f;
      voxel.color = voxblox::Color(30, 90, 150);
    }
  }
  // Retain mutable block handles across ownership transfer, so the fixture
  // controls the flags after replacement without casting away constness.
  auto first = initial->getBlockPtrByIndex(kChanged);
  auto second = initial->getBlockPtrByIndex(kUnchanged);
  backend.replaceTsdfLayer(std::move(initial));
  first->updated().set();
  second->updated().set();
  voxblox::MeshLayer meshes(.5f);
  backend.updateMeshLayer(&meshes, false);
  ASSERT_EQ(meshes.getNumberOfAllocatedMeshes(), 2u);
  for (const auto& index : indices) {
    expectOnlyMeshConsumed(backend, index);
    const auto& mesh = meshes.getMeshByIndex(index);
    ASSERT_EQ(mesh.vertices.size(), 6u);  // One square plane, two triangles.
    EXPECT_TRUE(mesh.updated);
    for (const auto& vertex : mesh.vertices) {
      EXPECT_FLOAT_EQ(vertex.x(), .5f * index.x() + .25f);
      EXPECT_GE(vertex.y(), .125f);
      EXPECT_LE(vertex.y(), .375f);
      EXPECT_GE(vertex.z(), .125f);
      EXPECT_LE(vertex.z(), .375f);
    }
  }
  const auto unchanged_vertices = meshes.getMeshByIndex(kUnchanged).vertices;
  const auto unchanged_colors = meshes.getMeshByIndex(kUnchanged).colors;
  voxblox_msgs::msg::Mesh message;
  gbplanner2::generateVoxbloxMeshMsg(&meshes, gbplanner2::MeshColorMode::Color, &message);
  ASSERT_EQ(message.mesh_blocks.size(), 2u);
  for (const auto& index : indices) {
    const auto* block = findBlock(message, index);
    ASSERT_NE(block, nullptr);
    EXPECT_EQ(block->x.size(), 6u);
  }

  backend.updateMeshLayer(&meshes, true);
  gbplanner2::generateVoxbloxMeshMsg(&meshes, gbplanner2::MeshColorMode::Color, &message);
  EXPECT_TRUE(message.mesh_blocks.empty());
  EXPECT_EQ(meshes.getNumberOfAllocatedMeshes(), 2u);

  auto modified = backend.copyTsdfLayer();
  first = modified->getBlockPtrByIndex(kChanged);
  second = modified->getBlockPtrByIndex(kUnchanged);
  for (std::size_t i = 0; i < first->num_voxels(); ++i)
    first->getVoxelByLinearIndex(i).distance = .25f;
  backend.replaceTsdfLayer(std::move(modified));
  first->updated().set();
  second->updated().reset();
  second->updated().set(voxblox::Update::kMap);
  second->updated().set(voxblox::Update::kEsdf);

  backend.updateMeshLayer(&meshes, true);
  for (const auto& index : indices) expectOnlyMeshConsumed(backend, index);
  EXPECT_TRUE(meshes.getMeshByIndex(kChanged).updated);
  EXPECT_TRUE(meshes.getMeshByIndex(kChanged).vertices.empty());
  EXPECT_FALSE(meshes.getMeshByIndex(kUnchanged).updated);
  gbplanner2::generateVoxbloxMeshMsg(&meshes, gbplanner2::MeshColorMode::Color, &message);
  ASSERT_EQ(message.mesh_blocks.size(), 1u);
  const auto* deleted = findBlock(message, kChanged);
  ASSERT_NE(deleted, nullptr);
  EXPECT_TRUE(deleted->x.empty());
  EXPECT_TRUE(deleted->y.empty());
  EXPECT_TRUE(deleted->z.empty());
  EXPECT_TRUE(deleted->r.empty());
  EXPECT_TRUE(deleted->g.empty());
  EXPECT_TRUE(deleted->b.empty());
  EXPECT_EQ(findBlock(message, kUnchanged), nullptr);
  ASSERT_EQ(meshes.getNumberOfAllocatedMeshes(), 1u);
  const auto& retained = meshes.getMeshByIndex(kUnchanged);
  ASSERT_EQ(retained.vertices.size(), unchanged_vertices.size());
  ASSERT_EQ(retained.colors.size(), unchanged_colors.size());
  for (std::size_t i = 0; i < retained.vertices.size(); ++i) {
    for (int axis = 0; axis < 3; ++axis)
      EXPECT_FLOAT_EQ(retained.vertices[i][axis], unchanged_vertices[i][axis]);
    EXPECT_EQ(retained.colors[i].r, unchanged_colors[i].r);
    EXPECT_EQ(retained.colors[i].g, unchanged_colors[i].g);
    EXPECT_EQ(retained.colors[i].b, unchanged_colors[i].b);
  }
}
