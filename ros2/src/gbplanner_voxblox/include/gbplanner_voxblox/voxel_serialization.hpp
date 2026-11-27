// Copyright (c) 2016, ETHZ ASL. All rights reserved.
// Adapted from the Voxblox integer codec recorded in layer_SOURCE.json.
// BSD-3-Clause; see LICENSE.voxblox. No ROS message dependency.
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

#include <voxblox/core/block.h>
#include <voxblox/core/voxel.h>

namespace gbplanner2 {
// Keep the existing conversion helper names so message and file paths share
// exactly one codec without changing their callers.
namespace layer_message_detail {
template <typename VoxelType>
constexpr std::size_t wordsPerVoxel() {
  static_assert(std::is_same_v<VoxelType, voxblox::TsdfVoxel> ||
                    std::is_same_v<VoxelType, voxblox::EsdfVoxel>,
                "Layer transport supports TSDF and ESDF voxels");
  return std::is_same_v<VoxelType, voxblox::TsdfVoxel> ? 3u : 2u;
}

template <typename VoxelType>
void serializeBlock(const voxblox::Block<VoxelType>& block,
                    std::vector<uint32_t>* data) {
  if constexpr (std::is_same_v<VoxelType, voxblox::EsdfVoxel>) {
    data->clear();
    data->reserve(block.num_voxels() * wordsPerVoxel<VoxelType>());
    for (std::size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      uint32_t distance_bits;
      static_assert(sizeof(distance_bits) == sizeof(voxel.distance));
      std::memcpy(&distance_bits, &voxel.distance, sizeof(distance_bits));
      data->push_back(distance_bits);
      // Convert each signed parent to an unsigned byte before shifting. The
      // original encoder sign-extends negative Y/Z into preceding bytes.
      const auto parent_byte = [](int value) {
        const int clamped = std::clamp(value, -128, 127);
        return static_cast<uint32_t>(static_cast<uint8_t>(clamped));
      };
      uint32_t parent_flags = (parent_byte(voxel.parent.x()) << 24) |
                              (parent_byte(voxel.parent.y()) << 16) |
                              (parent_byte(voxel.parent.z()) << 8);
      parent_flags |= (voxel.observed ? 1u : 0u) |
                      (voxel.hallucinated ? 2u : 0u) |
                      (voxel.in_queue ? 4u : 0u) |
                      (voxel.fixed ? 8u : 0u);
      data->push_back(parent_flags);
    }
  } else {
    block.serializeToIntegers(data);
  }
}
}  // namespace layer_message_detail
}  // namespace gbplanner2
