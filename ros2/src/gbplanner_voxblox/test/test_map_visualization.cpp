#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>

#include "gbplanner_voxblox/map_visualization.hpp"

namespace {
uint32_t wordAt(const sensor_msgs::msg::PointCloud2& cloud,
                std::size_t point, std::size_t offset) {
  const auto* bytes = cloud.data.data() + point * cloud.point_step + offset;
  return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) |
         (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
}

float floatAt(const sensor_msgs::msg::PointCloud2& cloud,
              std::size_t point, std::size_t offset) {
  const uint32_t bits = wordAt(cloud, point, offset);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void expectFarCenter(const sensor_msgs::msg::PointCloud2& cloud,
                     std::size_t point, unsigned linear_index) {
  // The .25 m voxels in block (20,0,0), two voxels per axis, start at x=10.
  EXPECT_FLOAT_EQ(floatAt(cloud, point, 0), 10.125f + .25f * (linear_index % 2));
  EXPECT_FLOAT_EQ(floatAt(cloud, point, 4), .125f + .25f * ((linear_index / 2) % 2));
  EXPECT_FLOAT_EQ(floatAt(cloud, point, 8), .125f + .25f * (linear_index / 4));
}
}  // namespace

TEST(MapVisualization, FullDistanceAndPackedSurfaceCloudsKeepDistinctPredicates) {
  voxblox::Layer<voxblox::TsdfVoxel> tsdf(.25f, 2);
  voxblox::Layer<voxblox::EsdfVoxel> esdf(.25f, 2);
  auto tsdf_block = tsdf.allocateBlockPtrByIndex({20, 0, 0});
  auto esdf_block = esdf.allocateBlockPtrByIndex({20, 0, 0});
  const std::array<float, 8> distances{{0.f, 0.f, 0.f, .1875f, -.1875f, .125f, -.125f, .25f}};
  const std::array<float, 8> weights{{0.f, .0005f, .001f, 1.f, 1.f, 1.f, 1.f, 1.f}};
  for (unsigned i = 0; i < 8; ++i) {
    auto& t = tsdf_block->getVoxelByLinearIndex(i);
    t.distance = distances[i];
    t.weight = weights[i];
    t.color = voxblox::Color(10 + i, 20 + i, 30 + i, 40 + i);
    auto& e = esdf_block->getVoxelByLinearIndex(i);
    e.observed = i != 0;
    e.distance = -.5f + .25f * i;
  }
  const auto tsdf_cloud = gbplanner2::createDistancePointcloudFromTsdfLayer(tsdf);
  ASSERT_EQ(tsdf_cloud.width, 5u);
  ASSERT_EQ(tsdf_cloud.point_step, 16u);
  ASSERT_EQ(tsdf_cloud.fields.size(), 4u);
  EXPECT_EQ(tsdf_cloud.fields[3].name, "intensity");
  for (unsigned i = 0; i < 5; ++i) {
    expectFarCenter(tsdf_cloud, i, i + 3);
    EXPECT_FLOAT_EQ(floatAt(tsdf_cloud, i, 12), distances[i + 3]);
  }
  const auto esdf_cloud = gbplanner2::createDistancePointcloudFromEsdfLayer(esdf);
  ASSERT_EQ(esdf_cloud.width, 7u);
  for (unsigned i = 0; i < 7; ++i) {
    expectFarCenter(esdf_cloud, i, i + 1);
    EXPECT_FLOAT_EQ(floatAt(esdf_cloud, i, 12), -.5f + .25f * (i + 1));
  }

  const auto surface = gbplanner2::createSurfacePointcloudFromTsdfLayer(tsdf);
  ASSERT_EQ(surface.width, 4u);
  EXPECT_EQ(surface.height, 1u);
  ASSERT_EQ(surface.point_step, 32u);
  ASSERT_EQ(surface.row_step, 128u);
  ASSERT_EQ(surface.data.size(), 128u);
  EXPECT_FALSE(surface.is_bigendian);
  EXPECT_TRUE(surface.is_dense);
  ASSERT_EQ(surface.fields.size(), 4u);
  EXPECT_EQ(surface.fields[3].name, "rgb");
  EXPECT_EQ(surface.fields[3].offset, 16u);
  EXPECT_EQ(surface.fields[3].datatype, sensor_msgs::msg::PointField::FLOAT32);
  const std::array<unsigned, 4> surface_indices{{1, 2, 5, 6}};
  for (unsigned point = 0; point < surface_indices.size(); ++point) {
    const unsigned i = surface_indices[point];
    expectFarCenter(surface, point, i);
    EXPECT_FLOAT_EQ(floatAt(surface, point, 12), 1.f);
    EXPECT_EQ(wordAt(surface, point, 16),
              0xFF000000u | ((10u + i) << 16) | ((20u + i) << 8) | (30u + i));
  }
  tsdf.removeAllBlocks();
  esdf.removeAllBlocks();
  const auto empty_surface = gbplanner2::createSurfacePointcloudFromTsdfLayer(tsdf);
  EXPECT_EQ(empty_surface.width, 0u);
  EXPECT_EQ(empty_surface.row_step, 0u);
  EXPECT_TRUE(empty_surface.data.empty());
  EXPECT_EQ(empty_surface.fields.size(), 4u);
  EXPECT_EQ(gbplanner2::createDistancePointcloudFromTsdfLayer(tsdf).width, 0u);
  EXPECT_EQ(gbplanner2::createDistancePointcloudFromEsdfLayer(esdf).width, 0u);
}

TEST(MapVisualization, OccupiedCubeGeometryHeightColorsAndEmptyDeletion) {
  voxblox::Layer<voxblox::TsdfVoxel> tsdf(.5f, 2);
  auto block = tsdf.allocateBlockPtrByIndex({-10, 0, -1});
  block->getVoxelByLinearIndex(0).weight = 1.f;
  block->getVoxelByLinearIndex(0).distance = 0.f;  // Inclusive boundary.
  block->getVoxelByLinearIndex(1).weight = 1.f;
  block->getVoxelByLinearIndex(1).distance = .25f;
  block->getVoxelByLinearIndex(2).weight = 1e-3f;  // Strict weight cutoff.
  block->getVoxelByLinearIndex(2).distance = -.5f;
  block->getVoxelByLinearIndex(3).weight = 0.f;
  block->getVoxelByLinearIndex(3).distance = -.5f;
  block->getVoxelByLinearIndex(4).weight = 1.f;
  block->getVoxelByLinearIndex(4).distance = -.25f;
  const auto occupied = gbplanner2::createOccupancyBlocksFromTsdfLayer(tsdf, 0.f);
  ASSERT_EQ(occupied.markers.size(), 1u);
  const auto& marker = occupied.markers[0];
  EXPECT_EQ(marker.ns, "occupied_voxels");
  EXPECT_EQ(marker.id, 0);
  EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::CUBE_LIST);
  EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(marker.scale.x, .5);
  EXPECT_DOUBLE_EQ(marker.scale.y, .5);
  EXPECT_DOUBLE_EQ(marker.scale.z, .5);
  EXPECT_DOUBLE_EQ(marker.pose.orientation.w, 1.);
  EXPECT_FLOAT_EQ(marker.color.a, 1.f);
  ASSERT_EQ(marker.points.size(), 2u);
  ASSERT_EQ(marker.colors.size(), 2u);
  for (unsigned i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(marker.points[i].x, -9.75);
    EXPECT_DOUBLE_EQ(marker.points[i].y, .25);
    EXPECT_DOUBLE_EQ(marker.points[i].z, -.75 + .5 * i);
    EXPECT_FLOAT_EQ(marker.colors[i].r, 0.f);
    EXPECT_FLOAT_EQ(marker.colors[i].g, 1.f);
    EXPECT_FLOAT_EQ(marker.colors[i].b, (i == 0 ? 25.f : 178.f) / 255.f);
    EXPECT_FLOAT_EQ(marker.colors[i].a, 1.f);
  }
  EXPECT_EQ(gbplanner2::createOccupancyBlocksFromTsdfLayer(tsdf, .25f)
                .markers[0].points.size(), 3u);
  tsdf.removeAllBlocks();
  const auto empty = gbplanner2::createOccupancyBlocksFromTsdfLayer(tsdf);
  ASSERT_EQ(empty.markers.size(), 1u);
  EXPECT_EQ(empty.markers[0].ns, marker.ns);
  EXPECT_EQ(empty.markers[0].id, marker.id);
  EXPECT_EQ(empty.markers[0].action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_TRUE(empty.markers[0].points.empty());
  EXPECT_TRUE(empty.markers[0].colors.empty());
}
