#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <voxblox/io/layer_io.h>

#include "gbplanner_voxblox/tsdf_backend.hpp"

namespace {
using TsdfLayer = voxblox::Layer<voxblox::TsdfVoxel>;
using EsdfLayer = voxblox::Layer<voxblox::EsdfVoxel>;
const voxblox::BlockIndex kIncoming(0, 0, 0);
const voxblox::BlockIndex kIncomingEsdfOnly(3, -2, 1);
const voxblox::BlockIndex kRetained(-3, 1, 0);
const voxblox::BlockIndex kRetainedEsdfOnly(4, 4, 4);

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    const auto pattern = (std::filesystem::temp_directory_path() / "gbplanner-map-XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const char* created = mkdtemp(buffer.data());
    if (!created) throw std::runtime_error("Cannot create map-test directory");
    path_ = created;
  }
  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }
  std::string file(const char* name) const { return (path_ / name).string(); }
 private:
  std::filesystem::path path_;
};

gbplanner2::TsdfConfig config() {
  gbplanner2::TsdfConfig settings;
  settings.resolution = .25;
  settings.voxels_per_side = 8;
  settings.storage = "sparse";
  settings.method = "simple";
  settings.threads = 1;
  settings.esdf_enabled = true;
  settings.esdf_min_distance = .2;
  settings.esdf_full_euclidean_distance = true;
  return settings;
}

void fillTsdf(TsdfLayer* layer, const voxblox::BlockIndex& index, float distance,
              float weight, const voxblox::Color& color) {
  auto block = layer->allocateBlockPtrByIndex(index);
  block->set_has_data(true);
  block->updated().set();
  for (std::size_t i = 0; i < block->num_voxels(); ++i) {
    auto& voxel = block->getVoxelByLinearIndex(i);
    voxel.distance = distance;
    voxel.weight = weight;
    voxel.color = color;
  }
}

void fillSource(gbplanner2::TsdfBackend* map) {
  auto tsdf = map->copyTsdfLayer();
  fillTsdf(tsdf.get(), kIncoming, 1.5f, 2.f, {17, 34, 51, 255});
  tsdf->getBlockByIndex(kIncoming).getVoxelByLinearIndex(0).distance = -.125f;
  map->replaceTsdfLayer(std::move(tsdf));

  // Include distance data that a TSDF-only rebuild cannot reproduce.
  auto esdf = std::make_unique<EsdfLayer>(.25f, 8);
  auto real = esdf->allocateBlockPtrByIndex(kIncoming);
  real->set_has_data(true);
  real->updated().set();
  auto& fixed = real->getVoxelByLinearIndex(0);
  fixed.distance = -.125f;
  fixed.observed = true;
  fixed.fixed = true;
  auto& propagated = real->getVoxelByLinearIndex(1);
  propagated.distance = .625f;
  propagated.observed = true;
  propagated.parent = {1, -2, 3};

  auto sphere = esdf->allocateBlockPtrByIndex(kIncomingEsdfOnly);
  sphere->set_has_data(true);
  sphere->updated().set();
  auto& inner = sphere->getVoxelByLinearIndex(0);
  inner.distance = 1.25f;
  inner.observed = true;
  inner.hallucinated = true;
  inner.parent = {-2, 1, -3};
  auto& outer = sphere->getVoxelByLinearIndex(1);
  outer.distance = -.75f;
  outer.observed = true;
  outer.hallucinated = true;
  map->replaceEsdfLayer(std::move(esdf));
}

void fillDestination(gbplanner2::TsdfBackend* map) {
  auto tsdf = map->copyTsdfLayer();
  fillTsdf(tsdf.get(), kIncoming, .5f, .5f, {101, 102, 103, 255});
  fillTsdf(tsdf.get(), kRetained, 1.5f, 3.f, {201, 202, 203, 255});
  tsdf->getBlockByIndex(kRetained).getVoxelByVoxelIndex({2, 2, 2}).distance = -.125f;
  map->replaceTsdfLayer(std::move(tsdf));
  auto esdf = map->copyEsdfLayer();
  auto& stale = esdf->getBlockByIndex(kIncoming).getVoxelByVoxelIndex({4, 3, 2});
  stale.distance = -1.5f;
  stale.observed = true;
  stale.hallucinated = true;
  auto sphere = esdf->allocateBlockPtrByIndex(kRetainedEsdfOnly);
  sphere->set_has_data(true);
  sphere->updated().reset();
  sphere->updated().set(voxblox::Update::kMap);
  auto& voxel = sphere->getVoxelByLinearIndex(0);
  voxel.distance = 1.75f;
  voxel.observed = true;
  voxel.hallucinated = true;
  voxel.parent = {-1, 2, -3};
  map->replaceEsdfLayer(std::move(esdf));
}

void expectSameVoxel(const voxblox::TsdfVoxel& actual, const voxblox::TsdfVoxel& expected) {
  EXPECT_FLOAT_EQ(actual.distance, expected.distance);
  EXPECT_FLOAT_EQ(actual.weight, expected.weight);
  EXPECT_EQ(actual.color.r, expected.color.r);
  EXPECT_EQ(actual.color.g, expected.color.g);
  EXPECT_EQ(actual.color.b, expected.color.b);
  EXPECT_EQ(actual.color.a, expected.color.a);
}

void expectSameVoxel(const voxblox::EsdfVoxel& actual, const voxblox::EsdfVoxel& expected) {
  EXPECT_FLOAT_EQ(actual.distance, expected.distance);
  EXPECT_EQ(actual.observed, expected.observed);
  EXPECT_EQ(actual.fixed, expected.fixed);
  EXPECT_EQ(actual.hallucinated, expected.hallucinated);
  EXPECT_EQ(actual.in_queue, expected.in_queue);
  EXPECT_EQ(actual.parent, expected.parent);
}

template<class Voxel>
void expectSameBlock(const voxblox::Block<Voxel>& actual, const voxblox::Block<Voxel>& expected,
                     bool compare_state = false) {
  ASSERT_EQ(actual.num_voxels(), expected.num_voxels());
  if (compare_state) {
    EXPECT_EQ(actual.has_data(), expected.has_data());
    EXPECT_EQ(actual.updated(), expected.updated());
  }
  for (std::size_t i = 0; i < expected.num_voxels(); ++i) {
    SCOPED_TRACE(i);
    expectSameVoxel(actual.getVoxelByLinearIndex(i), expected.getVoxelByLinearIndex(i));
  }
}

template<class Voxel>
void expectSameLayer(const voxblox::Layer<Voxel>& actual, const voxblox::Layer<Voxel>& expected,
                     bool compare_state = false) {
  ASSERT_EQ(actual.getNumberOfAllocatedBlocks(), expected.getNumberOfAllocatedBlocks());
  voxblox::BlockIndexList indices;
  expected.getAllAllocatedBlocks(&indices);
  for (const auto& index : indices) {
    SCOPED_TRACE(::testing::Message() << index.transpose());
    ASSERT_TRUE(actual.hasBlock(index));
    expectSameBlock(actual.getBlockByIndex(index), expected.getBlockByIndex(index), compare_state);
  }
}
}  // namespace

TEST(MapPersistence, DualLayerRoundTripReplacesMatchingBlocksAndRetainsOthers) {
  TemporaryDirectory directory;
  gbplanner2::TsdfBackend source(config()), destination(config());
  fillSource(&source);
  fillDestination(&destination);
  const auto retained_tsdf = destination.copyTsdfLayer();
  const auto retained_esdf = destination.copyEsdfLayer();
  source.saveMap(directory.file("source.vxblx"));

  // Read the same output through the original Voxblox file/voxel decoder,
  // independently of the adapter reader used by loadMap below.
  TsdfLayer::Ptr upstream_tsdf;
  EsdfLayer::Ptr upstream_esdf;
  ASSERT_TRUE(voxblox::io::LoadLayer<voxblox::TsdfVoxel>(
      directory.file("source.vxblx"), true, &upstream_tsdf));
  // The old LoadLayer outer loop checks fstream.eof() between layers.
  // Protobuf read-ahead can set that bit while the logical byte offset is
  // still before the ESDF section. Keep its original protobuf/block/voxel
  // decoders, but locate that section using their explicit offset instead.
  std::fstream upstream_file(directory.file("source.vxblx"), std::ios::in | std::ios::binary);
  ASSERT_TRUE(upstream_file.is_open());
  uint32_t upstream_offset = 0, tsdf_count = 0;
  ASSERT_TRUE(voxblox::utils::readProtoMsgCountToStream(
      &upstream_file, &tsdf_count, &upstream_offset));
  ASSERT_GT(tsdf_count, 0u);
  voxblox::LayerProto tsdf_header;
  ASSERT_TRUE(voxblox::utils::readProtoMsgFromStream(
      &upstream_file, &tsdf_header, &upstream_offset));
  ASSERT_EQ(tsdf_header.type(), "tsdf");
  for (uint32_t i = 1; i < tsdf_count; ++i) {
    voxblox::BlockProto skipped_tsdf_block;
    ASSERT_TRUE(voxblox::utils::readProtoMsgFromStream(
        &upstream_file, &skipped_tsdf_block, &upstream_offset));
  }
  const bool eof_after_tsdf = upstream_file.eof();
  const auto file_bytes = std::filesystem::file_size(directory.file("source.vxblx"));
  std::cout << "Upstream reader after TSDF: eofbit=" << eof_after_tsdf
            << " logical_offset=" << upstream_offset << " file_size=" << file_bytes << '\n';
  RecordProperty("upstream_eof_after_tsdf", eof_after_tsdf ? 1 : 0);
  RecordProperty("upstream_offset_after_tsdf", std::to_string(upstream_offset));
  RecordProperty("map_file_size", std::to_string(file_bytes));
  ASSERT_LT(upstream_offset, file_bytes);
  uint32_t esdf_count = 0;
  ASSERT_TRUE(voxblox::utils::readProtoMsgCountToStream(
      &upstream_file, &esdf_count, &upstream_offset));
  ASSERT_GT(esdf_count, 0u);
  voxblox::LayerProto esdf_header;
  ASSERT_TRUE(voxblox::utils::readProtoMsgFromStream(
      &upstream_file, &esdf_header, &upstream_offset));
  ASSERT_EQ(esdf_header.type(), "esdf");
  upstream_esdf = voxblox::aligned_shared<EsdfLayer>(esdf_header);
  ASSERT_TRUE(voxblox::io::LoadBlocksFromStream<voxblox::EsdfVoxel>(
      esdf_count - 1, EsdfLayer::BlockMergingStrategy::kProhibit,
      &upstream_file, upstream_esdf.get(), &upstream_offset));
  EXPECT_EQ(upstream_offset, file_bytes);
  ASSERT_TRUE(upstream_tsdf->hasBlock(kIncoming));
  ASSERT_TRUE(upstream_esdf->hasBlock(kIncoming));
  ASSERT_TRUE(upstream_esdf->hasBlock(kIncomingEsdfOnly));
  EXPECT_FALSE(upstream_tsdf->hasBlock(kIncomingEsdfOnly));
  expectSameVoxel(upstream_tsdf->getBlockByIndex(kIncoming).getVoxelByLinearIndex(0),
                  source.tsdfLayer().getBlockByIndex(kIncoming).getVoxelByLinearIndex(0));
  expectSameVoxel(upstream_esdf->getBlockByIndex(kIncoming).getVoxelByLinearIndex(1),
                  source.esdfLayer()->getBlockByIndex(kIncoming).getVoxelByLinearIndex(1));
  expectSameVoxel(upstream_esdf->getBlockByIndex(kIncomingEsdfOnly).getVoxelByLinearIndex(0),
                  source.esdfLayer()->getBlockByIndex(kIncomingEsdfOnly).getVoxelByLinearIndex(0));
  destination.loadMap(directory.file("source.vxblx"));

  ASSERT_EQ(destination.tsdfLayer().getNumberOfAllocatedBlocks(), 2u);
  ASSERT_EQ(destination.esdfLayer()->getNumberOfAllocatedBlocks(), 4u);
  ASSERT_TRUE(destination.esdfLayer()->hasBlock(kIncomingEsdfOnly));
  EXPECT_FALSE(destination.tsdfLayer().hasBlock(kIncomingEsdfOnly));
  expectSameBlock(destination.tsdfLayer().getBlockByIndex(kIncoming),
                  source.tsdfLayer().getBlockByIndex(kIncoming));
  expectSameBlock(destination.esdfLayer()->getBlockByIndex(kIncoming),
                  source.esdfLayer()->getBlockByIndex(kIncoming));
  expectSameBlock(destination.esdfLayer()->getBlockByIndex(kIncomingEsdfOnly),
                  source.esdfLayer()->getBlockByIndex(kIncomingEsdfOnly));
  expectSameBlock(destination.tsdfLayer().getBlockByIndex(kRetained),
                  retained_tsdf->getBlockByIndex(kRetained), true);
  expectSameBlock(destination.esdfLayer()->getBlockByIndex(kRetained),
                  retained_esdf->getBlockByIndex(kRetained), true);
  expectSameBlock(destination.esdfLayer()->getBlockByIndex(kRetainedEsdfOnly),
                  retained_esdf->getBlockByIndex(kRetainedEsdfOnly), true);

  destination.saveMap(directory.file("roundtrip.vxblx"));
  gbplanner2::TsdfBackend restored(config());
  restored.loadMap(directory.file("roundtrip.vxblx"));
  expectSameLayer(restored.tsdfLayer(), destination.tsdfLayer());
  expectSameLayer(*restored.esdfLayer(), *destination.esdfLayer());
}

TEST(MapPersistence, LegacyTsdfOnlyFileRebuildsEsdfFromTheMergedTsdf) {
  TemporaryDirectory directory;
  TsdfLayer legacy(.25f, 8);
  fillTsdf(&legacy, kIncoming, 1.5f, 1.f, {17, 34, 51, 255});
  legacy.getBlockByIndex(kIncoming).getVoxelByVoxelIndex({2, 2, 2}).distance = .125f;
  ASSERT_TRUE(voxblox::io::SaveLayer(legacy, directory.file("legacy.vxblx")));
  gbplanner2::TsdfBackend destination(config());
  fillDestination(&destination);
  const auto retained_tsdf = destination.copyTsdfLayer();
  destination.loadMap(directory.file("legacy.vxblx"));

  const auto* seed = destination.esdfVoxel({.625, .625, .625});
  const auto* target = destination.esdfVoxel({1.125, .875, .625});
  ASSERT_NE(seed, nullptr);
  ASSERT_NE(target, nullptr);
  EXPECT_TRUE(seed->observed);
  EXPECT_TRUE(seed->fixed);
  EXPECT_FALSE(seed->hallucinated);
  EXPECT_FLOAT_EQ(seed->distance, .125f);
  EXPECT_TRUE(target->observed);
  EXPECT_FALSE(target->fixed);
  EXPECT_FALSE(target->hallucinated);
  EXPECT_NEAR(target->distance, .125 + .25 * std::sqrt(5.), 1e-5);
  EXPECT_FALSE(destination.esdfLayer()->hasBlock(kRetainedEsdfOnly));
  ASSERT_TRUE(destination.tsdfLayer().hasBlock(kRetained));
  ASSERT_TRUE(destination.esdfLayer()->hasBlock(kRetained));
  expectSameBlock(destination.tsdfLayer().getBlockByIndex(kRetained),
                  retained_tsdf->getBlockByIndex(kRetained));
  const auto& retained_seed = destination.esdfLayer()->getBlockByIndex(kRetained)
                                 .getVoxelByVoxelIndex({2, 2, 2});
  EXPECT_TRUE(retained_seed.observed);
  EXPECT_TRUE(retained_seed.fixed);
  EXPECT_FALSE(retained_seed.hallucinated);
  EXPECT_FLOAT_EQ(retained_seed.distance, -.125f);
}

TEST(MapPersistence, TruncatedEsdfTailDoesNotChangeEitherLiveLayer) {
  TemporaryDirectory directory;
  gbplanner2::TsdfBackend source(config()), destination(config());
  fillSource(&source);
  fillDestination(&destination);
  const auto path = directory.file("truncated.vxblx");
  source.saveMap(path);
  const auto bytes = std::filesystem::file_size(path);
  ASSERT_GT(bytes, 1u);
  // Keep the complete valid TSDF prefix and cut inside the final ESDF block.
  std::filesystem::resize_file(path, bytes - 1);
  const auto before_tsdf = destination.copyTsdfLayer();
  const auto before_esdf = destination.copyEsdfLayer();

  EXPECT_THROW(destination.loadMap(path), std::runtime_error);
  expectSameLayer(destination.tsdfLayer(), *before_tsdf, true);
  expectSameLayer(*destination.esdfLayer(), *before_esdf, true);
}
