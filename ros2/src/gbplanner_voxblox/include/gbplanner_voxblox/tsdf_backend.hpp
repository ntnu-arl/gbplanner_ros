#pragma once
#include <gbplanner_local/voxel_map.hpp>
#include <voxblox/integrator/tsdf_integrator.h>
#include <memory>
#include <functional>
#include <limits>
#include <optional>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/mesh/mesh.h>
#include <voxblox/mesh/mesh_layer.h>

namespace gbplanner2 {
struct TsdfConfig {
  double resolution{0.25};
  Eigen::Vector3d origin{-5.,-5.,-1.5};
  Eigen::Vector3i dimensions{40,40,12};
  int voxels_per_side{16};
  double truncation_distance{0.75};
  double min_range{0.1}, max_range{6.0}, occupancy_factor{1.0};
  double max_weight{200.}, clearing_weight{0.01};
  int threads{1};
  std::string method{"fast"};
  bool weight_dropoff{true}, carving{true};
  // Unspecified preserves the adapter's earlier !weight_dropoff fallback;
  // an explicit false still lets symmetric weighting be selected on its own.
  std::optional<bool> use_const_weight{};
  bool use_symmetric_weight_dropoff{false}, weight_ray_by_range{false};
  bool allow_clear{true};
  std::string integration_order_mode{"mixed"};
  double start_voxel_subsampling_factor{2.0};
  int max_consecutive_ray_collisions{0}, clear_checks_every_n_frames{1};
  double max_integration_time_s{std::numeric_limits<float>::max()};
  bool anti_grazing{false};
  bool esdf_enabled{false};
  // Disable only for an auxiliary ESDF; ESDF-based planning requires fresh scans.
  bool esdf_update_on_scan{true};
  double esdf_max_distance{2.0};
  bool esdf_full_euclidean_distance{false};
  // -1 resolves to half the TSDF truncation distance, as in ros_params.h.
  double esdf_min_distance{-1.0};
  double esdf_default_distance{2.0};
  double esdf_min_diff{0.001};
  bool esdf_add_occupied_crust{false};
  bool esdf_clear_sphere_for_planning{false};
  double esdf_clear_sphere_radius{1.5};
  double esdf_occupied_sphere_radius{5.0};
  std::string distance_field{"tsdf"};
  std::string storage{"dense"};
  bool use_color{true};
  double mesh_min_weight{1e-4};
  bool mesh_use_color{true};
};
struct TsdfScanResult { std::size_t accepted_points{0}, skipped_points{0}; };
struct TsdfIntegrationTiming {
  double filter_wall_s{0.}, tsdf_wall_s{0.}, esdf_wall_s{0.}, total_wall_s{0.};
  bool esdf_updated{false}, completed{false};
};

// Native ROS-independent adapter. Observations use the original Voxblox integrator;
// explicit robot-box augmentation preserves the upstream unknown-only operation.
// Dense snapshots project a bounded part of that layer; sparse traversal visits
// every observed voxel without allocating an enclosing dense grid.
class TsdfBackend {
 public:
  explicit TsdfBackend(const TsdfConfig& config);
  TsdfScanResult integrate(const Eigen::Vector3d& origin, const Eigen::Quaterniond& rotation,
                          const std::vector<Eigen::Vector3d>& sensor_points);
  TsdfScanResult integrate(const Eigen::Vector3d& origin, const Eigen::Quaterniond& rotation,
                          const std::vector<Eigen::Vector3d>& sensor_points,
                          const std::vector<voxblox::Color>& sensor_colors);
  // Freespace endpoints describe a minimum clear distance, not an obstacle.
  TsdfScanResult integrateFreeSpace(const Eigen::Vector3d& origin,const Eigen::Quaterniond& rotation,
                                   const std::vector<Eigen::Vector3d>& sensor_points);
  // Consume all pending TSDF kEsdf changes without clearing kMesh or kMap.
  // Call serially with integration/pruning; a disabled ESDF is a no-op.
  void updateEsdf();
  // Most recent scan attempt; reset before validation. Timing is complete only
  // when completed is true. Explicit ESDF updates and spheres do not replace it.
  const TsdfIntegrationTiming& lastIntegrationTiming() const { return last_integration_timing_; }
  // Apply Voxblox's planning-time inner free sphere and outer occupied sphere
  // around a robot pose. The operation is a no-op when ESDF is disabled or
  // the feature is not enabled.
  void addNewRobotPosition(const Eigen::Vector3d& position);
  // Call between synchronous updates, in the original order: TSDF pruning,
  // optional robot clearing sphere, then ESDF pruning. The upstream predicate
  // measures from block origins to the float position, with a strict > cutoff.
  // ROS2 cache repair: mark surviving negative-offset neighbors kMesh because
  // their positive-corner triangles can depend on a removed block. If supplied,
  // removed is replaced with this call's actual deletion indices.
  std::size_t removeDistantTsdfBlocks(const Eigen::Vector3d& position,double max_distance,
                                    voxblox::BlockIndexList* removed=nullptr);
  std::size_t removeDistantEsdfBlocks(const Eigen::Vector3d& position,double max_distance);
  // Upstream getBoxAroundPoint lattice: snap float center first, then floor
  // float offsets starting at -half-size and advancing by one voxel.
  // Only weight<1e-6 TSDF cells are filled; known free/occupied cells survive.
  // A zero axis size visits its center index, as for upstream kNoBound.
  std::size_t clearUnknownBox(const Eigen::Vector3d& center,const Eigen::Vector3d& size);
  void reset();
  // Persist TSDF and, when enabled, ESDF in the same Voxblox file. Loading
  // replaces matching blocks in both layers and preserves other blocks;
  // legacy TSDF-only files rebuild ESDF from the complete merged TSDF.
  void saveMap(const std::string& file_path) const;
  void loadMap(const std::string& file_path);
  const voxblox::Layer<voxblox::TsdfVoxel>& tsdfLayer() const { return *layer_; }
  const voxblox::Layer<voxblox::EsdfVoxel>* esdfLayer() const { return esdf_layer_.get(); }
  std::unique_ptr<voxblox::Layer<voxblox::TsdfVoxel>> copyTsdfLayer() const;
  std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel>> copyEsdfLayer() const;
  // Commit a validated transport layer and reconnect the integrators to its
  // storage. TSDF import also rebuilds the derived ESDF used by the planner.
  void replaceTsdfLayer(std::unique_ptr<voxblox::Layer<voxblox::TsdfVoxel>> layer);
  void replaceEsdfLayer(std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel>> layer);
  voxblox::Mesh generateMesh() const;
  // Keep block meshes between updates; consume only the TSDF kMesh flag.
  void updateMeshLayer(voxblox::MeshLayer* mesh_layer, bool only_updated);
  // Refresh only requested blocks that have both cached mesh and live TSDF.
  // Used immediately after pruning; never allocate previously undisplayed meshes.
  void updateMeshBlocks(voxblox::MeshLayer* mesh_layer,const voxblox::BlockIndexList& blocks);
  void saveMesh(const std::string& file_path, const voxblox::Mesh& mesh) const;
  const voxblox::EsdfVoxel* esdfVoxel(const Eigen::Vector3d& position) const;
  VoxelStatus status(const Eigen::Vector3d& position) const;
  const voxblox::TsdfVoxel* voxel(const Eigen::Vector3d& position) const;
  std::shared_ptr<VoxelMap> snapshot() const;
  // World-zero lattice index; null pointers mean that distance field is
  // unobserved. Pointers remain owned by this backend during the callback.
  using ObservedVoxelVisitor=std::function<void(const Eigen::Vector3i&,VoxelStatus,
    const voxblox::TsdfVoxel*,const voxblox::EsdfVoxel*)>;
  void forEachObservedVoxel(const ObservedVoxelVisitor& visit) const;
  std::size_t allocatedBlocks() const { return layer_->getNumberOfAllocatedBlocks(); }
  static VoxelStatus classify(const voxblox::TsdfVoxel* voxel, double resolution, double factor);
  const TsdfConfig& config() const { return config_; }
 private:
  void initialize();
  void initializeIntegrators();
  void initializeEsdfIntegrator();
  void rebuildEsdf();
  TsdfScanResult integrateCloud(const Eigen::Vector3d& origin,const Eigen::Quaterniond& rotation,
                               const std::vector<Eigen::Vector3d>& sensor_points,
                               const std::vector<voxblox::Color>* sensor_colors,bool freespace);
  TsdfConfig config_;
  TsdfIntegrationTiming last_integration_timing_;
  std::unique_ptr<voxblox::Layer<voxblox::TsdfVoxel>> layer_;
  voxblox::TsdfIntegratorBase::Ptr integrator_;
  std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel>> esdf_layer_;
  std::unique_ptr<voxblox::EsdfIntegrator> esdf_integrator_;
};
}
