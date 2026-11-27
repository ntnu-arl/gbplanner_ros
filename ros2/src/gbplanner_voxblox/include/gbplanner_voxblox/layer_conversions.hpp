// Copyright (c) 2016, ETHZ ASL. All rights reserved.
// Adapted from voxblox_ros/conversions{,_inl}.h at the commit recorded in
// layer_SOURCE.json. Distributed under the BSD-3-Clause license; see
// LICENSE.voxblox. ROS 2 adaptation and validation: local course project.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <voxblox_msgs/msg/layer.hpp>

#include "gbplanner_voxblox/voxel_serialization.hpp"

namespace gbplanner2 {

enum class MapDeserializationAction : uint8_t {
  kUpdate = voxblox_msgs::msg::Layer::ACTION_UPDATE,
  kMerge = voxblox_msgs::msg::Layer::ACTION_MERGE,
  kReset = voxblox_msgs::msg::Layer::ACTION_RESET
};

namespace layer_message_detail {
inline bool validAction(MapDeserializationAction action) {
  return action == MapDeserializationAction::kUpdate ||
         action == MapDeserializationAction::kMerge ||
         action == MapDeserializationAction::kReset;
}

inline bool fail(const char* reason, std::string* error) {
  if (error) *error = reason;
  return false;
}

}  // namespace layer_message_detail

// Uses the upstream word layout, correcting its signed ESDF parent packing.
// Serialization leaves kMap/kMesh/kEsdf flags unchanged, as in ROS 1;
// publication policy owns their lifecycle.
template <typename VoxelType>
void serializeLayerAsMsg(
    const voxblox::Layer<VoxelType>& layer, bool only_updated,
    voxblox_msgs::msg::Layer* msg,
    MapDeserializationAction action = MapDeserializationAction::kUpdate) {
  (void)layer_message_detail::wordsPerVoxel<VoxelType>();
  if (!msg || !layer_message_detail::validAction(action))
    throw std::invalid_argument("Invalid Layer message output or action");
  if (layer.voxels_per_side() > std::numeric_limits<uint32_t>::max())
    throw std::invalid_argument("Layer voxel count does not fit the message");

  msg->voxels_per_side = static_cast<uint32_t>(layer.voxels_per_side());
  msg->voxel_size = layer.voxel_size();
  msg->layer_type = voxblox::getVoxelType<VoxelType>();
  msg->action = static_cast<uint8_t>(action);

  voxblox::BlockIndexList indices;
  if (only_updated)
    layer.getAllUpdatedBlocks(voxblox::Update::kMap, &indices);
  else
    layer.getAllAllocatedBlocks(&indices);

  // A reused output message must represent this call alone. The original
  // converter appended to msg->blocks and assumed callers always used a new msg.
  msg->blocks.clear();
  msg->blocks.reserve(indices.size());
  for (const auto& index : indices) {
    voxblox_msgs::msg::Block block;
    block.x_index = index.x();
    block.y_index = index.y();
    block.z_index = index.z();
    layer_message_detail::serializeBlock(layer.getBlockByIndex(index), &block.data);
    msg->blocks.push_back(std::move(block));
  }
}

// Preflight all blocks before RESET or UPDATE changes any voxel. The upstream
// codec uses fatal CHECKs for invalid lengths, so those checks belong here at
// the transport boundary. Matching voxel size retains its upstream 1e-5 limit.
template <typename VoxelType>
bool validateLayerMsg(const voxblox_msgs::msg::Layer& msg,
                      MapDeserializationAction action,
                      const voxblox::Layer<VoxelType>& layer,
                      std::string* error = nullptr) {
  using namespace layer_message_detail;
  if (error) error->clear();
  if (!validAction(action)) return fail("Unknown Layer action", error);
  if (msg.layer_type != voxblox::getVoxelType<VoxelType>())
    return fail("Layer voxel type does not match", error);
  if (!std::isfinite(msg.voxel_size) || msg.voxel_size <= 0. ||
      msg.voxels_per_side != layer.voxels_per_side() ||
      std::abs(msg.voxel_size - layer.voxel_size()) > 1e-5)
    return fail("Layer voxel size or block shape does not match", error);

  std::size_t expected_words = wordsPerVoxel<VoxelType>();
  for (int axis = 0; axis < 3; ++axis) {
    if (msg.voxels_per_side == 0 ||
        expected_words > std::numeric_limits<std::size_t>::max() /
                             msg.voxels_per_side)
      return fail("Layer block shape overflows its payload size", error);
    expected_words *= msg.voxels_per_side;
  }
  std::set<std::array<int32_t, 3>> indices;
  for (const auto& block : msg.blocks) {
    if (block.data.size() != expected_words)
      return fail("Layer block payload has the wrong number of words", error);
    if (!indices.insert({block.x_index, block.y_index, block.z_index}).second)
      return fail("Layer message contains a duplicate block index", error);
  }
  return true;
}

template <typename VoxelType>
bool validateLayerMsg(const voxblox_msgs::msg::Layer& msg,
                      const voxblox::Layer<VoxelType>& layer,
                      std::string* error = nullptr) {
  return validateLayerMsg(msg, static_cast<MapDeserializationAction>(msg.action),
                          layer, error);
}

// UPDATE replaces only listed blocks; MERGE delegates voxel combination to the
// upstream core; RESET removes unlisted blocks. No frame or timestamp is added
// to Layer.msg, matching the ROS 1 protocol (the endpoints share a fixed frame).
template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::msg::Layer& msg,
                           MapDeserializationAction action,
                           voxblox::Layer<VoxelType>* layer,
                           std::string* error = nullptr) {
  if (!layer) return layer_message_detail::fail("Missing destination layer", error);
  if (!validateLayerMsg(msg, action, *layer, error)) return false;
  if (action == MapDeserializationAction::kReset) layer->removeAllBlocks();

  for (const auto& block_msg : msg.blocks) {
    const voxblox::BlockIndex index(block_msg.x_index, block_msg.y_index,
                                    block_msg.z_index);
    const bool existed = layer->hasBlock(index);
    auto block = layer->allocateBlockPtrByIndex(index);
    if (action == MapDeserializationAction::kMerge && existed) {
      voxblox::Block<VoxelType> incoming(block->voxels_per_side(),
                                         block->voxel_size(), block->origin());
      incoming.deserializeFromIntegers(block_msg.data);
      // The ROS 1 converter omitted this flag, causing mergeBlock() to return
      // without merging. Integer decoding itself does not set block metadata.
      incoming.set_has_data(true);
      block->mergeBlock(incoming);
    } else {
      block->deserializeFromIntegers(block_msg.data);
    }
    // Received data must participate in downstream mesh/ESDF updates and map
    // publication. The original transport omitted these flags for new blocks.
    block->set_has_data(true);
    block->updated().set();
  }
  return true;
}

template <typename VoxelType>
bool deserializeMsgToLayer(const voxblox_msgs::msg::Layer& msg,
                           voxblox::Layer<VoxelType>* layer,
                           std::string* error = nullptr) {
  return deserializeMsgToLayer(
      msg, static_cast<MapDeserializationAction>(msg.action), layer, error);
}

}  // namespace gbplanner2
