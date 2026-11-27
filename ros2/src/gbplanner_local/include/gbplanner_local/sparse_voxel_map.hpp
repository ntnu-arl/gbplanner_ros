#pragma once

#include "gbplanner_local/voxel_map.hpp"
#include "gbplanner_local/voxel_index_table.hpp"

namespace gbplanner2 {

// A world-aligned sparse snapshot. An absent cell is unknown, including cells
// between separately observed regions; storage extent never implies free space.
class SparseVoxelMap final : public MapQuery {
 public:
  static constexpr int kIndexLimit=1000000;
  explicit SparseVoxelMap(double resolution);
  double resolution() const override {return resolution_;}
  // These are the supported coordinate domain, not the observed-cell bounds.
  // Planning permissions remain the responsibility of BoundedMap/GeofenceMap.
  Eigen::Vector3d minimum() const override;
  Eigen::Vector3d maximum() const override;
  Eigen::Vector3i index(const Eigen::Vector3d& position) const;
  Eigen::Vector3d center(const Eigen::Vector3i& index) const;
  void set(const Eigen::Vector3i& index,VoxelStatus status);
  void setTsdf(const Eigen::Vector3i& index,double distance,double weight);
  // Combined occupancy/TSDF record for snapshot imports. Valid records have
  // the same result as set() then setTsdf(), with one cell-table lookup.
  void setObservation(const Eigen::Vector3i& index,VoxelStatus status,double distance,double weight);
  // Reserve contiguous storage for a snapshot without adding observed cells
  // or changing the unknown status of any absent coordinate.
  void reserve(std::size_t count) {cells_.reserve(count);}
  std::size_t size() const {return cells_.size();}
  VoxelStatus voxel(const Eigen::Vector3d& position) const override;
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& position) const override;
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& position) const override;
  std::optional<TsdfDistance> tsdfDistanceSimplexCached(
      const Eigen::Vector3d& position,TsdfQueryContext& context) const override;
  VoxelStatus box(const Eigen::Vector3d& position,const Eigen::Vector3d& size) const override;
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                   const Eigen::Vector3d& size) const override;
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit) const override;

 private:
  struct Cell {
    VoxelStatus status{VoxelStatus::Unknown};
    double distance{0.},weight{0.};
  };
  bool inside(const Eigen::Vector3i& index) const;
  bool contains(const Eigen::Vector3d& position) const;
  const Cell* cell(const Eigen::Vector3i& index) const;
  void updateFreeBlock(const Eigen::Vector3i& index,VoxelStatus status);
  bool fullFreeBlock(const Eigen::Vector3i& block) const;
  double resolution_;
  // Immutable published snapshots own their arrays. Replacing one map never
  // reuses memory still held by a concurrent validation reader.
  VoxelIndexTable<Cell> cells_;
  // One bit per explicitly Free voxel in a 4x4x4 block. Missing and TSDF-only
  // cells leave zero bits; unsupported cells in boundary blocks do likewise.
  VoxelIndexTable<std::uint64_t> free_blocks_;
};
}  // namespace gbplanner2
