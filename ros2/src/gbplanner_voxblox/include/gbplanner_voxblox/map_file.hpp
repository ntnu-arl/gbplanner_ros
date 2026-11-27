// Copyright (c) 2016, ETHZ ASL. All rights reserved.
// Adapted from Voxblox layer_io/layer_inl at the commit in layer_SOURCE.json.
// BSD-3-Clause; see LICENSE.voxblox.
#pragma once

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Block.pb.h>
#include <Layer.pb.h>
#include <voxblox/core/layer.h>
#include <voxblox/utils/protobuf_utils.h>

#include "gbplanner_voxblox/voxel_serialization.hpp"

namespace gbplanner2 {
namespace map_file_detail {
template <typename VoxelType>
bool writeLayer(const voxblox::Layer<VoxelType>& layer, std::fstream* stream) {
  voxblox::BlockIndexList indices;
  layer.getAllAllocatedBlocks(&indices);
  if (indices.size() >= std::numeric_limits<uint32_t>::max()) return false;
  if (!voxblox::utils::writeProtoMsgCountToStream(
          static_cast<uint32_t>(indices.size() + 1), stream)) return false;
  voxblox::LayerProto header;
  layer.getProto(&header);
  if (!voxblox::utils::writeProtoMsgToStream(header, stream)) return false;
  std::vector<uint32_t> data;
  for (const auto& index : indices) {
    const auto& block = layer.getBlockByIndex(index);
    voxblox::BlockProto proto;
    // Do not call ESDF Block::getProto: it invokes the original signed-shift
    // encoder before we could replace its payload. The metadata is unchanged.
    proto.set_voxels_per_side(block.voxels_per_side());
    proto.set_voxel_size(block.voxel_size());
    proto.set_origin_x(block.origin().x());
    proto.set_origin_y(block.origin().y());
    proto.set_origin_z(block.origin().z());
    proto.set_has_data(block.has_data());
    layer_message_detail::serializeBlock(block, &data);
    for (const uint32_t word : data) proto.add_voxel_data(word);
    if (!voxblox::utils::writeProtoMsgToStream(proto, stream) || !stream->good())
      return false;
  }
  return stream->good();
}

inline bool sameVoxelSize(double size, voxblox::FloatingPoint expected) {
  // Retain Layer::isCompatible's absolute float-epsilon tolerance.
  return std::isfinite(size) && size > 0. &&
         std::abs(size - expected) < std::numeric_limits<voxblox::FloatingPoint>::epsilon();
}

inline bool readMessage(std::fstream* stream, google::protobuf::Message* message,
                        uint32_t* offset, uint32_t file_size) {
  // Check the declared length as well as protobuf parsing. EOF inside a
  // length-delimited message must never be accepted as a complete section.
  uint32_t payload_offset = *offset, payload_size = 0;
  if (!voxblox::utils::readProtoMsgCountToStream(stream, &payload_size, &payload_offset) ||
      payload_size == 0 || payload_offset > file_size ||
      payload_size > file_size - payload_offset) return false;
  return voxblox::utils::readProtoMsgFromStream(stream, message, offset) &&
         *offset == payload_offset + payload_size;
}

inline void validateBlock(const voxblox::BlockProto& block,
                          const voxblox::Layer<voxblox::TsdfVoxel>& shape,
                          std::size_t words_per_voxel) {
  if (block.voxels_per_side() <= 0 ||
      static_cast<std::size_t>(block.voxels_per_side()) != shape.voxels_per_side() ||
      !sameVoxelSize(block.voxel_size(), shape.voxel_size()))
    throw std::runtime_error("Map block geometry does not match the configured layer");
  std::size_t expected_words = words_per_voxel;
  for (int axis = 0; axis < 3; ++axis) {
    if (expected_words > std::numeric_limits<std::size_t>::max() / shape.voxels_per_side())
      throw std::runtime_error("Map block payload size overflows");
    expected_words *= shape.voxels_per_side();
  }
  if (static_cast<std::size_t>(block.voxel_data_size()) != expected_words)
    throw std::runtime_error("Map block payload has the wrong number of words");
  // addBlockFromProto rounds float origins into an int block index. Check
  // that conversion before invoking it; leave the original rounding intact.
  const voxblox::FloatingPoint inverse = 1.f / shape.block_size();
  for (const double origin : {block.origin_x(), block.origin_y(), block.origin_z()}) {
    const double index = std::round(static_cast<voxblox::FloatingPoint>(origin) * inverse);
    if (!std::isfinite(origin) || !std::isfinite(index) ||
        index < std::numeric_limits<voxblox::IndexElement>::min() ||
        index > std::numeric_limits<voxblox::IndexElement>::max())
      throw std::runtime_error("Map block origin cannot be represented by a block index");
  }
}
}  // namespace map_file_detail

// Original concatenated protobuf format: TSDF section first, then ESDF when
// enabled. The caller owns the temporary-file/rename transaction.
inline bool writeMapFile(const std::string& file,
                         const voxblox::Layer<voxblox::TsdfVoxel>& tsdf,
                         const voxblox::Layer<voxblox::EsdfVoxel>* esdf) {
  std::fstream stream(file, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!stream.is_open()) return false;
  bool success = map_file_detail::writeLayer(tsdf, &stream);
  if (success && esdf) success = map_file_detail::writeLayer(*esdf, &stream);
  stream.flush();
  success = success && stream.good();
  stream.close();
  return success && !stream.fail();
}

// Read into caller-owned temporary copies using original same-block replacement.
// Return whether an ESDF section exists, including a valid zero-block section.
// Only a fully parsed TSDF-only file may return false; a damaged/incompatible
// ESDF tail must not silently become a TSDF rebuild.
inline bool readMapFile(const std::string& file,
                        voxblox::Layer<voxblox::TsdfVoxel>* tsdf,
                        voxblox::Layer<voxblox::EsdfVoxel>* esdf) {
  if (!tsdf) throw std::runtime_error("Missing TSDF destination layer");
  if (esdf && (esdf->voxels_per_side() != tsdf->voxels_per_side() ||
               !map_file_detail::sameVoxelSize(esdf->voxel_size(), tsdf->voxel_size())))
    throw std::runtime_error("TSDF and ESDF destination geometry differs");
  std::fstream stream(file, std::ios::in | std::ios::binary | std::ios::ate);
  if (!stream.is_open()) throw std::runtime_error("Cannot open map file: " + file);
  const auto end = stream.tellg();
  if (end < std::streampos(0) ||
      static_cast<std::streamoff>(end) > std::numeric_limits<uint32_t>::max())
    throw std::runtime_error("Map file exceeds the original protobuf offset range");
  const auto file_size = static_cast<uint32_t>(static_cast<std::streamoff>(end));
  uint32_t offset = 0;
  bool found_tsdf = false, found_esdf = false;
  while (offset < file_size) {
    uint32_t count = 0;
    voxblox::LayerProto header;
    if (!voxblox::utils::readProtoMsgCountToStream(&stream, &count, &offset) || count == 0 ||
        !map_file_detail::readMessage(&stream, &header, &offset, file_size))
      throw std::runtime_error("Invalid or truncated map layer header");
    const bool is_tsdf = header.type() == voxblox::getVoxelType<voxblox::TsdfVoxel>();
    const bool is_esdf = header.type() == voxblox::getVoxelType<voxblox::EsdfVoxel>();
    if ((is_tsdf || is_esdf) &&
        (header.voxels_per_side() != tsdf->voxels_per_side() ||
         !map_file_detail::sameVoxelSize(header.voxel_size(), tsdf->voxel_size())))
      throw std::runtime_error("Map layer geometry does not match the configured layer");
    // Original LoadBlocksFromFile selects the first matching layer. Continue
    // parsing later sections for file completeness without applying them twice.
    const bool import_tsdf = is_tsdf && !found_tsdf;
    const bool import_esdf = is_esdf && !found_esdf && esdf;
    found_tsdf = found_tsdf || is_tsdf;
    found_esdf = found_esdf || is_esdf;
    for (uint32_t i = 1; i < count; ++i) {
      voxblox::BlockProto block;
      if (!map_file_detail::readMessage(&stream, &block, &offset, file_size))
        throw std::runtime_error("Invalid or truncated map block");
      if (is_tsdf || is_esdf)
        map_file_detail::validateBlock(block, *tsdf, is_tsdf ? 3u : 2u);
      if (import_tsdf && !tsdf->addBlockFromProto(block,
              voxblox::Layer<voxblox::TsdfVoxel>::BlockMergingStrategy::kReplace))
        throw std::runtime_error("Could not load TSDF map block");
      if (import_esdf && !esdf->addBlockFromProto(block,
              voxblox::Layer<voxblox::EsdfVoxel>::BlockMergingStrategy::kReplace))
        throw std::runtime_error("Could not load ESDF map block");
    }
  }
  if (offset != file_size || !found_tsdf)
    throw std::runtime_error("Map file has no complete TSDF layer");
  return found_esdf;
}
}  // namespace gbplanner2
