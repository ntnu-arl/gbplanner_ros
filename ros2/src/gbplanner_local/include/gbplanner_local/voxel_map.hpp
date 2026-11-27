#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace gbplanner2 {
enum class VoxelStatus : int8_t { Unknown=-1, Free=0, Occupied=100 };
struct TsdfDistance {
  double distance{0.},weight{0.};
  bool interpolated{false};
};

class MapQuery;
// One synchronous query sequence on an immutable snapshot. Recreate this
// context after any map mutation; it owns copied observations, never cells.
// A set bit distinguishes a cached missing observation from an unread slot.
struct TsdfQueryContext {
  const MapQuery* owner{nullptr};
  Eigen::Vector3i lower{Eigen::Vector3i::Zero()};
  std::uint8_t initialized{0};
  // Vertex (x,y,z) uses x | (y<<1) | (z<<2), with each component in {0,1}.
  std::array<std::optional<TsdfDistance>,8> observations{};
  std::size_t hits{0},lookups{0};
};

// Backend-neutral query surface extracted from the upstream MapManager.
// A conservative occupancy backend is provided for isolated migration tests;
// gbplanner_voxblox can also populate this grid from the original TSDF layer.
// This class itself does not implement TSDF/ESDF integration.
class MapQuery {
 public:
  virtual ~MapQuery()=default;
  virtual double resolution() const=0;
  virtual Eigen::Vector3d minimum() const=0;
  virtual Eigen::Vector3d maximum() const=0;
  virtual VoxelStatus voxel(const Eigen::Vector3d& p) const=0;
  // Missing distance observations are distinct from an occupied/unknown cell.
  // Occupancy-only maps never manufacture a distance from their labels.
  virtual std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d&) const {
    return std::nullopt;
  }
  // Explicit ROS 2 terrain-field alternative. A fixed Freudenthal simplex
  // uses observed vertices only; no containing-voxel fallback is permitted.
  // Keeping it separate preserves the default trilinear/voxel query contract.
  virtual std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d&) const {
    return std::nullopt;
  }
  // Backends may reuse raw stencil observations while preserving the scalar
  // interpolation. Unmodified backends retain their ordinary query behavior.
  virtual std::optional<TsdfDistance> tsdfDistanceSimplexCached(
      const Eigen::Vector3d& p,TsdfQueryContext&) const {
    return tsdfDistanceSimplex(p);
  }
  // All-positive size means a swept axis-aligned body. Exactly zero in all
  // three axes means point/segment collision, including touched voxel faces.
  // Mixed zero/positive dimensions are invalid. Policy wrappers must keep
  // applying their fences and bounds to point motion as well.
  virtual VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size) const=0;
  virtual VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                           const Eigen::Vector3d& size) const=0;
  // Planning bounds may omit rewards without inventing occupied observations.
  virtual bool gainAllowed(const Eigen::Vector3d&) const {return true;}
  virtual void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                   const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit) const=0;
};

class VoxelMap final:public MapQuery {
 public:
  VoxelMap(double resolution,const Eigen::Vector3d& origin,const Eigen::Vector3i& dimensions);
  double resolution() const override { return resolution_; }
  Eigen::Vector3d minimum() const override { return origin_; }
  Eigen::Vector3d maximum() const override { return origin_+dimensions_.cast<double>()*resolution_; }
  VoxelStatus voxel(const Eigen::Vector3d& p) const override;
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p) const override;
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& p) const override;
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size) const override;
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                   const Eigen::Vector3d& size) const override;
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit) const override;
  void set(const Eigen::Vector3i& index,VoxelStatus value);
  // Stores the backend's raw metric TSDF and weight without changing occupancy.
  // A weight below 1e-6 is unobserved; zero clears this distance observation.
  void setTsdf(const Eigen::Vector3i& index,double distance,double weight);
  void fill(VoxelStatus value);
  bool inside(const Eigen::Vector3i& index) const;
  Eigen::Vector3i index(const Eigen::Vector3d& p) const;
  Eigen::Vector3d center(const Eigen::Vector3i& index) const;
  std::size_t size() const { return cells_.size(); }
  const Eigen::Vector3i& dimensions() const { return dimensions_; }
 private:
  VoxelStatus pointPath(const Eigen::Vector3d& a,const Eigen::Vector3d& b) const;
  std::size_t offset(const Eigen::Vector3i& index) const;
  double resolution_;
  Eigen::Vector3d origin_;
  Eigen::Vector3i dimensions_;
  std::vector<VoxelStatus> cells_;
  std::vector<double> tsdf_distances_,tsdf_weights_;
};
}  // namespace gbplanner2
