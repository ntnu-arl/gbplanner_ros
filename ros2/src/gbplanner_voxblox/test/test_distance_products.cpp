#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <vector>

#include "gbplanner_voxblox/distance_products.hpp"

namespace {
using PointRecord = std::array<float, 4>;

std::vector<PointRecord> decode(const sensor_msgs::msg::PointCloud2& cloud) {
  EXPECT_EQ(cloud.height, 1u);
  EXPECT_EQ(cloud.point_step, 16u);
  EXPECT_EQ(cloud.row_step, cloud.width * cloud.point_step);
  EXPECT_FALSE(cloud.is_bigendian);
  EXPECT_EQ(cloud.fields.size(), 4u);
  EXPECT_EQ(cloud.fields.back().name, "intensity");
  std::vector<PointRecord> records;
  for (std::size_t i = 0; i < cloud.width; ++i) {
    PointRecord record;
    for (std::size_t field = 0; field < 4; ++field) {
      const auto* bytes = cloud.data.data() + i * cloud.point_step + field * 4;
      const uint32_t bits = uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) |
                            (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
      std::memcpy(&record[field], &bits, sizeof(bits));
    }
    records.push_back(record);
  }
  std::sort(records.begin(), records.end());
  return records;
}

// Independent transcription of the fixed upstream ptcloud_vis.h predicates
// and outer traversal, without the ROS 1/PCL dependencies or new helpers.
template <class Voxel, class Predicate>
std::vector<PointRecord> upstreamPoints(const voxblox::Layer<Voxel>& layer,
                                       Predicate predicate) {
  voxblox::BlockIndexList indices;
  layer.getAllAllocatedBlocks(&indices);
  std::vector<PointRecord> result;
  for (const auto& index : indices) {
    const auto& block = layer.getBlockByIndex(index);
    for (std::size_t i = 0; i < block.num_voxels(); ++i) {
      const auto point = block.computeCoordinatesFromLinearIndex(i);
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (predicate(voxel, point))
        result.push_back({point.x(), point.y(), point.z(), voxel.distance});
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}
}  // namespace

TEST(DistanceProducts, UpstreamSliceBoundariesObservedSetsAndTraversability) {
  voxblox::Layer<voxblox::TsdfVoxel> tsdf(.25f, 2);
  voxblox::Layer<voxblox::EsdfVoxel> esdf(.25f, 2);
  // Below z=0, near the origin, and well outside the ordinary dense ROI.
  const voxblox::BlockIndex indices[] = {{-1, 0, -1}, {0, 0, 0}, {20, 0, 0}};
  for (const auto& index : indices) {
    auto tsdf_block = tsdf.allocateBlockPtrByIndex(index);
    auto esdf_block = esdf.allocateBlockPtrByIndex(index);
    for (std::size_t i = 0; i < 8; ++i) {
      auto& t = tsdf_block->getVoxelByLinearIndex(i);
      auto& e = esdf_block->getVoxelByLinearIndex(i);
      t.distance = e.distance = .125f * static_cast<float>(i);
      t.weight = 1.f;
      e.observed = true;
    }
  }
  auto& t_origin = tsdf.getBlockByIndex({0, 0, 0});
  t_origin.getVoxelByLinearIndex(0).weight = 1e-3f;
  t_origin.getVoxelByLinearIndex(1).weight = std::nextafter(1e-3f, 1.f);
  auto& e_origin = esdf.getBlockByIndex({0, 0, 0});
  e_origin.getVoxelByLinearIndex(0).observed = false;
  e_origin.getVoxelByLinearIndex(0).distance = 10.f;  // Unknown is never free.
  e_origin.getVoxelByLinearIndex(1).distance = -.5f;
  e_origin.getVoxelByLinearIndex(4).distance = std::nextafter(.5f, 0.f);

  for (float slice : {-.375f, -.25f, -.125f, -.1f, 0.f, .125f,
                      .25f, .250001f, .250002f, .375f}) {
    float upstream_plane = slice;
    if (std::remainder(upstream_plane, .25f) < voxblox::kFloatEpsilon)
      upstream_plane += .25f / 2.0;
    const auto expected_tsdf = upstreamPoints(tsdf,
        [&](const voxblox::TsdfVoxel& voxel, const voxblox::Point& point) {
          return std::abs(point.z() - upstream_plane) <=
                     (.25f / 2.0 + voxblox::kFloatEpsilon) && voxel.weight > 1e-3f;
        });
    const auto expected_esdf = upstreamPoints(esdf,
        [&](const voxblox::EsdfVoxel& voxel, const voxblox::Point& point) {
          return std::abs(point.z() - upstream_plane) <=
                     (.25f / 2.0 + voxblox::kFloatEpsilon) && voxel.observed;
        });
    EXPECT_EQ(decode(gbplanner2::createDistancePointcloudFromTsdfLayerSlice(tsdf, 2, slice)),
              expected_tsdf) << "slice=" << slice;
    EXPECT_EQ(decode(gbplanner2::createDistancePointcloudFromEsdfLayerSlice(esdf, 2, slice)),
              expected_esdf) << "slice=" << slice;
  }
  const auto zero = decode(gbplanner2::createDistancePointcloudFromTsdfLayerSlice(tsdf, 2, 0.f));
  ASSERT_EQ(zero.size(), 7u);  // Exact boundary selects the upper slab, not both.
  for (const auto& point : zero) EXPECT_FLOAT_EQ(point[2], .125f);
  EXPECT_EQ(std::count_if(zero.begin(), zero.end(), [](const auto& p) { return p[0] > 10.f; }), 4);
  EXPECT_EQ(decode(gbplanner2::createDistancePointcloudFromTsdfLayerSlice(tsdf, 2, .25f)).size(), 8u);
  EXPECT_EQ(decode(gbplanner2::createDistancePointcloudFromEsdfLayerSlice(esdf, 2, -.25f)).size(), 4u);

  const auto traversable = decode(gbplanner2::createFreePointcloudFromEsdfLayer(esdf, .5f));
  const auto reference = upstreamPoints(esdf, [](const voxblox::EsdfVoxel& voxel, const voxblox::Point&) {
    return voxel.observed && voxel.distance >= .5f;
  });
  EXPECT_EQ(traversable, reference);
  EXPECT_EQ(traversable.size(), 11u);
  EXPECT_EQ(std::count_if(traversable.begin(), traversable.end(), [](const auto& p) { return p[3] == .5f; }), 2);
  for (const auto& point : traversable) {
    EXPECT_GE(point[3], .5f);
    EXPECT_LT(point[3], 10.f);
  }
}
