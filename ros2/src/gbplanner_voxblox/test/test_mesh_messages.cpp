#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "gbplanner_voxblox/mesh_messages.hpp"

namespace {
const voxblox::BlockIndex kTriangle(-2, 1, -1);
const voxblox::BlockIndex kUnchanged(4, 0, 0);
const voxblox::BlockIndex kDeleted(-3, 2, 1);

voxblox::Mesh::Ptr triangle(voxblox::MeshLayer* layer,
                            const voxblox::BlockIndex& index, bool updated) {
  auto mesh = layer->allocateMeshPtrByIndex(index);
  const voxblox::Point origin = index.cast<float>() * layer->block_size();
  mesh->vertices = {origin + voxblox::Point(.25f, .5f, .75f),
                    origin + voxblox::Point(1.25f, .5f, .75f),
                    origin + voxblox::Point(.25f, 1.5f, .75f)};
  mesh->indices = {0, 1, 2};
  mesh->normals = {voxblox::Point::UnitZ(), voxblox::Point::UnitZ(), voxblox::Point::UnitZ()};
  mesh->colors = {voxblox::Color(17, 34, 51, 68),
                  voxblox::Color(80, 120, 160, 200),
                  voxblox::Color(250, 100, 20, 10)};
  mesh->updated = updated;
  return mesh;
}

const voxblox_msgs::msg::MeshBlock* findBlock(const voxblox_msgs::msg::Mesh& message,
                                             const voxblox::BlockIndex& index) {
  for (const auto& block : message.mesh_blocks)
    if (block.index[0] == index.x() && block.index[1] == index.y() && block.index[2] == index.z())
      return &block;
  return nullptr;
}
}  // namespace

TEST(MeshMessages, NegativeBlockTrianglesQuantizedDecodeAndEmptyDeletion) {
  voxblox::MeshLayer layer(2.f);
  const auto original = triangle(&layer, kTriangle, true);
  triangle(&layer, kUnchanged, false);
  layer.allocateMeshPtrByIndex(kDeleted)->updated = true;

  voxblox_msgs::msg::Mesh message;
  message.header.frame_id = "map";
  message.header.stamp.sec = 123;
  gbplanner2::generateVoxbloxMeshMsg(&layer, gbplanner2::MeshColorMode::Color, &message);
  ASSERT_EQ(message.mesh_blocks.size(), 2u);
  EXPECT_EQ(message.header.frame_id, "map");
  EXPECT_EQ(message.header.stamp.sec, 123);
  EXPECT_FLOAT_EQ(message.block_edge_length, 2.f);
  const auto* block = findBlock(message, kTriangle);
  ASSERT_NE(block, nullptr);
  ASSERT_EQ(block->x.size(), 3u);
  ASSERT_EQ(block->y.size(), 3u);
  ASSERT_EQ(block->z.size(), 3u);
  // Absolute (-3.75, 2.5, -1.25) belongs to block (-2, 1, -1).
  // Upstream encoding truncates to uint16, rather than rounding.
  EXPECT_EQ(block->x[0], 4095u);
  EXPECT_EQ(block->y[0], 8191u);
  EXPECT_EQ(block->z[0], 12287u);
  EXPECT_EQ(block->x[1], 20479u);
  EXPECT_EQ(block->y[2], 24575u);
  ASSERT_EQ(block->r.size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(block->r[i], original->colors[i].r);
    EXPECT_EQ(block->g[i], original->colors[i].g);
    EXPECT_EQ(block->b[i], original->colors[i].b);
  }

  std::array<voxblox::Point, 3> decoded;
  for (std::size_t i = 0; i < 3; ++i) {
    // Original RViz decoder: uint16 coordinate is normalized by TWO block
    // lengths. This independently checks block translation and vertex order.
    constexpr float factor = 2.f / std::numeric_limits<uint16_t>::max();
    decoded[i] = voxblox::Point(
        (block->x[i] * factor + block->index[0]) * message.block_edge_length,
        (block->y[i] * factor + block->index[1]) * message.block_edge_length,
        (block->z[i] * factor + block->index[2]) * message.block_edge_length);
    for (int axis = 0; axis < 3; ++axis)
      EXPECT_NEAR(decoded[i][axis], original->vertices[i][axis],
                  2.f * layer.block_size() / 65535.f + 1e-6f);
  }
  EXPECT_GT((decoded[1] - decoded[0]).cross(decoded[2] - decoded[0]).z(), 0.f);
  const auto* deleted = findBlock(message, kDeleted);
  ASSERT_NE(deleted, nullptr);
  EXPECT_TRUE(deleted->x.empty());
  EXPECT_TRUE(deleted->r.empty());
  EXPECT_EQ(findBlock(message, kUnchanged), nullptr);
  EXPECT_EQ(layer.getNumberOfAllocatedMeshes(), 2u);
  EXPECT_FALSE(original->updated);

  // The same output object must not retain an old update or deletion.
  gbplanner2::generateVoxbloxMeshMsg(&layer, gbplanner2::MeshColorMode::Color, &message);
  EXPECT_TRUE(message.mesh_blocks.empty());
  // Full snapshots include unchanged meshes; normals are reconstructed at the
  // receiver from vertex triples, so no RGB or sender normal array is needed.
  original->normals.clear();
  gbplanner2::generateVoxbloxMeshMsg(&layer, gbplanner2::MeshColorMode::Normals, &message, false);
  ASSERT_EQ(message.mesh_blocks.size(), 2u);
  for (const auto& normals_block : message.mesh_blocks) {
    EXPECT_TRUE(normals_block.r.empty());
    EXPECT_TRUE(normals_block.g.empty());
    EXPECT_TRUE(normals_block.b.empty());
  }

  EXPECT_EQ(gbplanner2::meshColorModeFromString("none"), gbplanner2::MeshColorMode::Gray);
  original->updated = true;
  gbplanner2::generateVoxbloxMeshMsg(&layer, gbplanner2::meshColorModeFromString("none"), &message);
  ASSERT_EQ(message.mesh_blocks.size(), 1u);
  EXPECT_EQ(message.mesh_blocks[0].r, std::vector<uint8_t>({127, 127, 127}));
  EXPECT_EQ(message.mesh_blocks[0].g, message.mesh_blocks[0].r);
  EXPECT_EQ(message.mesh_blocks[0].b, message.mesh_blocks[0].r);
}
