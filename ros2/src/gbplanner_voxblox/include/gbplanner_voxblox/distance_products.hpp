// Copyright (c) 2016, ETHZ ASL. All rights reserved.
// Adapted from voxblox_ros/ptcloud_vis.h, fixed Voxblox commit
// 0ee88c26c203a3fe88e348dbbb552ab431a35352. BSD-3-Clause; LICENSE.voxblox.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>

#include "gbplanner_voxblox/pointcloud_buffer.hpp"

namespace gbplanner2 {
namespace distance_product_detail {

inline sensor_msgs::msg::PointCloud2 emptyCloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.point_step = 16;
  const char* names[] = {"x", "y", "z", "intensity"};
  for (uint32_t i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = i * 4;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  return cloud;
}

inline void writeLittleEndianWord(uint32_t bits, uint8_t* bytes) {
  for (unsigned byte = 0; byte < 4; ++byte)
    bytes[byte] = static_cast<uint8_t>(bits >> (byte * 8));
}

inline void writeLittleEndianFloat(float value, uint8_t* bytes) {
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  writeLittleEndianWord(bits, bytes);
}

inline void appendPoint(const voxblox::Point& center, float intensity,
                        PointcloudBuffer<16>* buffer, bool* is_dense) {
  // Encode one complete XYZI record before appending it. Explicit packing
  // preserves the little-endian wire format on either host byte order.
  uint8_t record[16];
  const float values[] = {center.x(), center.y(), center.z(), intensity};
  for (unsigned field = 0; field < 4; ++field) {
    writeLittleEndianFloat(values[field], record + field * 4);
    if (!std::isfinite(values[field])) *is_dense = false;
  }
  buffer->append(record);
}

template <typename VoxelType, typename Predicate>
sensor_msgs::msg::PointCloud2 fromLayer(const voxblox::Layer<VoxelType>& layer,
                                       Predicate include) {
  auto cloud = emptyCloud();
  PointcloudBuffer<16> buffer(cloud.data);
  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);
  for (const auto& index : blocks) {
    const auto& block = layer.getBlockByIndex(index);
    for (std::size_t i = 0; i < block.num_voxels(); ++i) {
      const auto center = block.computeCoordinatesFromLinearIndex(i);
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (include(voxel, center))
        appendPoint(center, voxel.distance, &buffer, &cloud.is_dense);
    }
  }
  cloud.width = buffer.finish();
  cloud.row_step = cloud.width * cloud.point_step;
  return cloud;
}

inline voxblox::FloatingPoint adjustedSlice(voxblox::FloatingPoint slice,
                                           voxblox::FloatingPoint voxel_size) {
  // This is the upstream signed remainder comparison, including negative
  // remainders. Using abs(remainder) would change which slice is published.
  if (std::remainder(slice, voxel_size) < voxblox::kFloatEpsilon)
    slice += voxel_size / 2.0;
  return slice;
}

inline void validateSlice(unsigned axis, voxblox::FloatingPoint slice) {
  if (axis > 2 || !std::isfinite(slice))
    throw std::invalid_argument("Slice axis must be 0..2 and position finite");
}
}  // namespace distance_product_detail

// Full allocated layers are traversed; dense planner-map bounds do not clip
// these visualization products. The caller supplies frame_id and timestamp.
inline sensor_msgs::msg::PointCloud2 createDistancePointcloudFromTsdfLayerSlice(
    const voxblox::Layer<voxblox::TsdfVoxel>& layer, unsigned axis,
    voxblox::FloatingPoint slice) {
  distance_product_detail::validateSlice(axis, slice);
  slice = distance_product_detail::adjustedSlice(slice, layer.voxel_size());
  return distance_product_detail::fromLayer(layer,
      [&](const voxblox::TsdfVoxel& voxel, const voxblox::Point& center) {
        return std::abs(center(axis) - slice) <=
                   layer.voxel_size() / 2.0 + voxblox::kFloatEpsilon &&
               voxel.weight > 1e-3f;
      });
}

inline sensor_msgs::msg::PointCloud2 createDistancePointcloudFromEsdfLayerSlice(
    const voxblox::Layer<voxblox::EsdfVoxel>& layer, unsigned axis,
    voxblox::FloatingPoint slice) {
  distance_product_detail::validateSlice(axis, slice);
  slice = distance_product_detail::adjustedSlice(slice, layer.voxel_size());
  return distance_product_detail::fromLayer(layer,
      [&](const voxblox::EsdfVoxel& voxel, const voxblox::Point& center) {
        return std::abs(center(axis) - slice) <=
                   layer.voxel_size() / 2.0 + voxblox::kFloatEpsilon &&
               voxel.observed;
      });
}

inline sensor_msgs::msg::PointCloud2 createFreePointcloudFromEsdfLayer(
    const voxblox::Layer<voxblox::EsdfVoxel>& layer, float min_distance) {
  if (!std::isfinite(min_distance) || min_distance < 0.)
    throw std::invalid_argument("Traversability radius must be finite and nonnegative");
  return distance_product_detail::fromLayer(layer,
      [min_distance](const voxblox::EsdfVoxel& voxel, const voxblox::Point&) {
        return voxel.observed && voxel.distance >= min_distance;
      });
}

}  // namespace gbplanner2
