#include <gtest/gtest.h>

#include <limits>

#include "gbplanner_voxblox/layer_conversions.hpp"

using gbplanner2::MapDeserializationAction;
using gbplanner2::deserializeMsgToLayer;
using gbplanner2::serializeLayerAsMsg;

namespace {
const voxblox::BlockIndex kIncoming(-2, 3, -1);
const voxblox::BlockIndex kUntouched(4, -5, 2);
using TsdfLayer = voxblox::Layer<voxblox::TsdfVoxel>;
using EsdfLayer = voxblox::Layer<voxblox::EsdfVoxel>;

void fillTsdf(TsdfLayer* layer, const voxblox::BlockIndex& index, float distance,
              float weight, const voxblox::Color& color) {
  auto block = layer->allocateBlockPtrByIndex(index);
  for (std::size_t i = 0; i < block->num_voxels(); ++i) {
    auto& voxel = block->getVoxelByLinearIndex(i);
    voxel.distance = distance;
    voxel.weight = weight;
    voxel.color = color;
  }
  block->set_has_data(true);
  block->updated().set();
}

void expectSameColor(const voxblox::Color& actual, const voxblox::Color& expected) {
  EXPECT_EQ(actual.r, expected.r);
  EXPECT_EQ(actual.g, expected.g);
  EXPECT_EQ(actual.b, expected.b);
  EXPECT_EQ(actual.a, expected.a);
}
}  // namespace

TEST(LayerConversions, TsdfWordsVoxelOrderAndIncrementalSelection) {
  TsdfLayer source(.2f, 2);
  fillTsdf(&source, kIncoming, 1.25f, 2.5f, {17, 34, 51, 68});
  fillTsdf(&source, kUntouched, -2.f, 1.f, {5, 6, 7, 8});
  auto& block = source.getBlockByIndex(kIncoming);
  // Eight distinct distances expose voxel order changes within the block.
  for (std::size_t i = 1; i < block.num_voxels(); ++i)
    block.getVoxelByLinearIndex(i).distance = static_cast<float>(i);
  source.getBlockByIndex(kUntouched).updated().reset(voxblox::Update::kMap);

  voxblox_msgs::msg::Layer message;
  serializeLayerAsMsg(source, true, &message);
  ASSERT_EQ(message.blocks.size(), 1u);
  EXPECT_EQ(message.layer_type, "tsdf");
  EXPECT_EQ(message.action, message.ACTION_UPDATE);
  const auto& encoded = message.blocks.front();
  EXPECT_EQ(encoded.x_index, -2);
  EXPECT_EQ(encoded.y_index, 3);
  EXPECT_EQ(encoded.z_index, -1);
  ASSERT_EQ(encoded.data.size(), 24u);
  // IEEE-754 distance, IEEE-754 weight, then packed RGBA in upstream order.
  EXPECT_EQ(encoded.data[0], 0x3fa00000u);
  EXPECT_EQ(encoded.data[1], 0x40200000u);
  EXPECT_EQ(encoded.data[2], 0x11223344u);
  EXPECT_EQ(encoded.data[3], 0x3f800000u);
  EXPECT_EQ(encoded.data[21], 0x40e00000u);
  EXPECT_TRUE(block.updated()[voxblox::Update::kMap]);

  TsdfLayer destination(.2f, 2);
  ASSERT_TRUE(deserializeMsgToLayer(message, &destination));
  const auto& decoded = destination.getBlockByIndex(kIncoming);
  EXPECT_TRUE(decoded.has_data());
  EXPECT_TRUE(decoded.updated().all());
  for (std::size_t i = 0; i < block.num_voxels(); ++i) {
    EXPECT_FLOAT_EQ(decoded.getVoxelByLinearIndex(i).distance,
                    block.getVoxelByLinearIndex(i).distance);
    EXPECT_FLOAT_EQ(decoded.getVoxelByLinearIndex(i).weight, 2.5f);
    expectSameColor(decoded.getVoxelByLinearIndex(i).color, {17, 34, 51, 68});
  }

  serializeLayerAsMsg(source, false, &message);
  EXPECT_EQ(message.blocks.size(), 2u);
  source.getBlockByIndex(kIncoming).updated().reset(voxblox::Update::kMap);
  serializeLayerAsMsg(source, true, &message);
  EXPECT_TRUE(message.blocks.empty());  // No accumulation on output reuse.
}

TEST(LayerConversions, UpdateMergeAndResetAffectTheIntendedBlocks) {
  TsdfLayer source(.2f, 2), destination(.2f, 2);
  fillTsdf(&source, kIncoming, -1.f, 3.f, {100, 80, 60, 200});
  fillTsdf(&destination, kIncoming, 3.f, 1.f, {20, 40, 60, 80});
  fillTsdf(&destination, kUntouched, 7.f, 2.f, {1, 2, 3, 4});
  voxblox_msgs::msg::Layer message;
  serializeLayerAsMsg(source, false, &message, MapDeserializationAction::kMerge);
  ASSERT_TRUE(deserializeMsgToLayer(message, &destination));
  const auto& merged = destination.getBlockByIndex(kIncoming).getVoxelByLinearIndex(0);
  EXPECT_FLOAT_EQ(merged.distance, 0.f);
  EXPECT_FLOAT_EQ(merged.weight, 4.f);
  expectSameColor(merged.color, {80, 70, 60, 170});
  EXPECT_FLOAT_EQ(destination.getBlockByIndex(kUntouched).getVoxelByLinearIndex(0).distance, 7.f);

  message.action = message.ACTION_UPDATE;
  ASSERT_TRUE(deserializeMsgToLayer(message, &destination));
  EXPECT_FLOAT_EQ(destination.getBlockByIndex(kIncoming).getVoxelByLinearIndex(0).distance, -1.f);
  EXPECT_FLOAT_EQ(destination.getBlockByIndex(kIncoming).getVoxelByLinearIndex(0).weight, 3.f);
  EXPECT_TRUE(destination.hasBlock(kUntouched));

  message.action = message.ACTION_RESET;
  ASSERT_TRUE(deserializeMsgToLayer(message, &destination));
  EXPECT_EQ(destination.getNumberOfAllocatedBlocks(), 1u);
  EXPECT_FALSE(destination.hasBlock(kUntouched));
  message.blocks.clear();
  ASSERT_TRUE(deserializeMsgToLayer(message, &destination));
  EXPECT_EQ(destination.getNumberOfAllocatedBlocks(), 0u);
}

TEST(LayerConversions, EsdfPreservesPackedFlagsAndSignedParents) {
  EsdfLayer source(.1f, 2), destination(.1f, 2);
  auto block = source.allocateBlockPtrByIndex(kIncoming);
  auto& voxel = block->getVoxelByLinearIndex(0);
  voxel.distance = -2.f;
  voxel.parent = voxblox::VoxelIndex(-3, 4, -5);
  voxel.observed = true;
  voxel.hallucinated = true;
  voxel.in_queue = true;
  voxel.fixed = true;
  auto& second_voxel = block->getVoxelByLinearIndex(1);
  second_voxel.distance = 1.f;
  second_voxel.parent = voxblox::VoxelIndex(3, -4, 5);
  second_voxel.observed = true;
  voxblox_msgs::msg::Layer message;
  serializeLayerAsMsg(source, false, &message, MapDeserializationAction::kMerge);
  ASSERT_EQ(message.blocks.size(), 1u);
  ASSERT_EQ(message.blocks[0].data.size(), 16u);
  EXPECT_EQ(message.blocks[0].data[0], 0xc0000000u);
  EXPECT_EQ(message.blocks[0].data[1], 0xfd04fb0fu);
  EXPECT_EQ(message.blocks[0].data[2], 0x3f800000u);
  EXPECT_EQ(message.blocks[0].data[3], 0x03fc0501u);
  // MERGE at a new index copies the entire block; merging with zero-initialized
  // voxels instead would lose parent direction and the three metadata flags.
  ASSERT_TRUE(deserializeMsgToLayer(message, &destination));
  auto& decoded = destination.getBlockByIndex(kIncoming).getVoxelByLinearIndex(0);
  EXPECT_FLOAT_EQ(decoded.distance, -2.f);
  EXPECT_EQ(decoded.parent, voxel.parent);
  EXPECT_TRUE(decoded.observed);
  EXPECT_TRUE(decoded.hallucinated);
  EXPECT_TRUE(decoded.in_queue);
  EXPECT_TRUE(decoded.fixed);
  EXPECT_EQ(destination.getBlockByIndex(kIncoming).getVoxelByLinearIndex(1).parent,
            second_voxel.parent);

  decoded.distance = 4.f;
  decoded.parent = voxblox::VoxelIndex(1, 2, 3);
  decoded.hallucinated = false;
  ASSERT_TRUE(deserializeMsgToLayer(message, &destination));
  // Existing ESDF blocks use the upstream merge rule: average observed
  // distances, OR observed, and retain destination parent/other flags.
  EXPECT_FLOAT_EQ(decoded.distance, 1.f);
  EXPECT_EQ(decoded.parent, voxblox::VoxelIndex(1, 2, 3));
  EXPECT_FALSE(decoded.hallucinated);
}

TEST(LayerConversions, InvalidResetDoesNotRemoveOrPartiallyReplaceLiveMap) {
  TsdfLayer source(.2f, 2), destination(.2f, 2);
  fillTsdf(&source, kIncoming, -1.f, 3.f, {100, 80, 60, 200});
  fillTsdf(&destination, kUntouched, 7.f, 2.f, {1, 2, 3, 4});
  voxblox_msgs::msg::Layer valid;
  serializeLayerAsMsg(source, false, &valid, MapDeserializationAction::kReset);
  auto reject = [&](const voxblox_msgs::msg::Layer& candidate) {
    std::string error;
    EXPECT_FALSE(deserializeMsgToLayer(candidate, &destination, &error));
    EXPECT_FALSE(error.empty());
    ASSERT_EQ(destination.getNumberOfAllocatedBlocks(), 1u);
    ASSERT_TRUE(destination.hasBlock(kUntouched));
    EXPECT_FLOAT_EQ(destination.getBlockByIndex(kUntouched).getVoxelByLinearIndex(0).distance, 7.f);
  };
  auto invalid = valid;
  invalid.blocks.push_back(valid.blocks[0]);
  invalid.blocks.back().x_index = 9;
  invalid.blocks.back().data.pop_back();  // Reject before accepting first block.
  reject(invalid);
  invalid = valid;
  invalid.blocks.push_back(valid.blocks[0]);
  reject(invalid);
  invalid = valid;
  invalid.action = 99;
  reject(invalid);
  invalid = valid;
  invalid.layer_type = "esdf";
  reject(invalid);
  invalid = valid;
  invalid.voxel_size = std::numeric_limits<double>::quiet_NaN();
  reject(invalid);
  invalid = valid;
  invalid.voxels_per_side = 4;
  reject(invalid);
}

TEST(LayerConversions, ExplicitActionOverrideMatchesUpstreamApi) {
  TsdfLayer source(.2f, 2), destination(.2f, 2);
  fillTsdf(&source, kIncoming, 1.f, 1.f, {1, 2, 3, 4});
  fillTsdf(&destination, kUntouched, 7.f, 2.f, {1, 2, 3, 4});
  voxblox_msgs::msg::Layer message;
  serializeLayerAsMsg(source, false, &message, MapDeserializationAction::kReset);
  ASSERT_TRUE(deserializeMsgToLayer(message, MapDeserializationAction::kUpdate,
                                   &destination));
  EXPECT_EQ(destination.getNumberOfAllocatedBlocks(), 2u);
}
