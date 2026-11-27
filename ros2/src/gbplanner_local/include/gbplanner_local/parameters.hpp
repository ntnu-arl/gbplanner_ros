// GBPlanner 2 local/aerial subset. Algorithm provenance: ../../../../UPSTREAM.md.
#pragma once
#include <planner_common/graph_base.h>
#include "gbplanner_local/gain_zone.hpp"
#include "gbplanner_local/ground_terrain.hpp"
#include "gbplanner_local/adaptive_bound.hpp"
#include "gbplanner_local/path_refinement.hpp"
#include "gbplanner_local/axis_sampling.hpp"
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>
#include <set>
#include <utility>
#include <vector>

namespace gbplanner2 {
// Static angular sensor model from SensorParamsBase. Camera here means the
// original fixed angular-grid frustum; dynamic camera/pixel models are separate.
struct SensorParameters {
  std::string name{"legacy"},type{"kLidar"};
  double range{3.},min_range{0.};
  Eigen::Vector3d center_offset{0,0,0},rotations{0,0,0};
  double horizontal_fov{2*M_PI},vertical_fov{M_PI/3};
  double horizontal_resolution{M_PI/12},vertical_resolution{M_PI/12};
  double frontier_threshold{.1};
  void validate() const {
    const auto positive=[](double v){return std::isfinite(v)&&v>0;};
    if(name.empty()||(type!="kLidar"&&type!="kCamera")||!positive(range)||
       !std::isfinite(min_range)||min_range<0||min_range>=range||
       !center_offset.allFinite()||!rotations.allFinite()||
       !positive(horizontal_fov)||horizontal_fov>2*M_PI+1e-9||
       !positive(vertical_fov)||vertical_fov>M_PI||
       !positive(horizontal_resolution)||!positive(vertical_resolution)||
       std::ceil(horizontal_fov/horizontal_resolution)*std::ceil(vertical_fov/vertical_resolution)>10000||
       !std::isfinite(frontier_threshold)||frontier_threshold<0||frontier_threshold>1)
      throw std::invalid_argument("Invalid static sensor parameters");
  }
};

// Gain-only ray traversal and the legacy axis-aligned per-voxel branch.
// Budgets reject a whole view instead of returning a partial gain score.
struct GainParameters {
  bool use_ray_model{true};
  std::string ray_method{"iterative"};
  bool nonuniform_ray_cast{true};
  // Constant-step weights and fixed snapshot bounds only; occupancy and
  // gainAllowed remain point queries. False retains the baseline evaluation.
  bool reuse_ray_invariants{false};
  double ray_cast_step_size_multiplier{1.};
  std::size_t max_ray_samples{1000000};
  double exp_gain_voxel_size{0.2};
  std::size_t max_voxels{1000000};
  void validate() const {
    if((ray_method!="iterative"&&ray_method!="dda")||
       !std::isfinite(ray_cast_step_size_multiplier)||ray_cast_step_size_multiplier<=0.||
       max_ray_samples==0||max_ray_samples>10000000||
       !std::isfinite(exp_gain_voxel_size)||exp_gain_voxel_size<=0.||
       max_voxels==0||max_voxels>10000000)
      throw std::invalid_argument("Invalid volumetric gain parameters");
  }
};

struct Parameters {
  // These are small synthetic-scene defaults, not settings for a real robot.
  Eigen::Vector3d local_min{-3.0, -3.0, -1.0}, local_max{3.0, 3.0, 1.0};
  // Original local-space extensions expand only the non-ray gain ROI, not
  // the sampling bounds. A local sphere supplies the same radius AABB.
  Eigen::Vector3d gain_min_extension{0,0,0},gain_max_extension{0,0,0};
  // Upstream BoundedSpaceParams::rotations: yaw, pitch, roll in radians.
  // This fixed world-to-sampling rotation does not follow the robot heading.
  Eigen::Vector3d local_rotations{0,0,0};
  Eigen::Vector3d robot_size{0.4, 0.4, 0.4}, center_offset{0, 0, 0};
  // kNoBound uses an exact point collision body, never an epsilon-sized box.
  // Requiring this flag prevents an accidental zero configuration from
  // silently disabling the robot footprint.
  bool point_robot{false};
  double edge_min{0.2}, edge_max{1.0}, overshoot{0.1};
  // getBestPath keeps the complete graph edge that first exceeds a limit.
  // Defaults match the upstream edge_max=1 and PlanningParams::v_max=.2.
  double max_velocity{0.2},traverse_length_max{1.0},traverse_time_max{5.0};
  double neighbor_radius{1.5}, neighbor_min{0.2}, neighbor_max{1.5};
  int max_vertices{100}, max_edges{2000}, max_attempts{4000};
  // Legacy synthetic fixtures cap raw draws. Upstream robot profiles instead
  // cap sampleVertex calls; each call accepts one sample or exhausts 1000 draws.
  // Only the selected budget applies, so a raw limit cannot truncate a round.
  std::string sampling_budget_mode{"raw_draws"};
  int max_sampling_rounds{10000};
  // Rrg::buildGraph counts sampleVertex calls, each trying up to 1000 raw
  // draws. This cutoff is checked after an accepted sample is expanded and
  // only while the graph still contains its root. Fully rejected calls skip it.
  int empty_graph_cutoff{1000};
  double sensor_range{3.0}, horizontal_fov{2*M_PI}, vertical_fov{M_PI/3};
  double horizontal_resolution{M_PI/12}, vertical_resolution{M_PI/12};
  double unknown_gain{1.0}, free_gain{0.0}, occupied_gain{0.0};
  double path_length_penalty{0.07}, path_direction_penalty{0.0}, frontier_threshold{0.1};
  double clustering_radius{2.0};
  bool graph_mode{true}, tangent_yaw{true}, leaf_gain_only{false};
  bool cluster_vertices_for_gain{false};
  uint32_t seed{42};
  std::string distribution{"uniform"};
  Eigen::Vector3d sample_mean{0,0,0}, sample_scale{1,1,0.5};
  SamplingOverrides axis_sampling,adaptive_axis_sampling;
  AdaptiveParameters adaptive;
  GainParameters gain;
  PathRefinementParameters refinement;
  // Empty means one sensor assembled from legacy scalar settings above.
  std::vector<SensorParameters> sensors;
  std::vector<GainZone> no_gain_zones;
  bool ground_robot{false};
  GroundParameters ground;

  GroundParameters groundParameters()const{
    auto parameters=ground;parameters.robot_size=robot_size;parameters.center_offset=center_offset;
    parameters.point_robot=point_robot;return parameters;
  }

  Eigen::Matrix3d worldToSampling() const {
    return (Eigen::AngleAxisd(local_rotations[0],Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(local_rotations[1],Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(local_rotations[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
  }

  SamplingAxes samplingAxes(bool adaptive_profile=false)const{
    SamplingAxes axes;
    for(int i=0;i<4;++i){
      if(i<3){
        axes[i].distribution=distribution;
        axes[i].minimum=local_min[i];axes[i].maximum=local_max[i];
        axes[i].mean=sample_mean[i];axes[i].scale=sample_scale[i];
      }else{axes[i].minimum=-M_PI;axes[i].maximum=M_PI;}
      axes[i]=axis_sampling[i].apply(axes[i]);
      if(adaptive_profile)axes[i]=adaptive_axis_sampling[i].apply(axes[i]);
    }
    return axes;
  }

  void validate() const {
    adaptive.validate();
    gain.validate();
    refinement.validate();
    if(ground_robot)groundParameters().validate();
    const auto positive=[](double v){ return std::isfinite(v) && v>0; };
    const auto nonnegative=[](double v){ return std::isfinite(v) && v>=0; };
    if (!local_min.allFinite() || !local_max.allFinite() || !local_rotations.allFinite() ||
        !gain_min_extension.allFinite() || !gain_max_extension.allFinite() ||
        !(local_min+gain_min_extension).allFinite() || !(local_max+gain_max_extension).allFinite() ||
        ((local_min+gain_min_extension).array()>=(local_max+gain_max_extension).array()).any() ||
        (local_min.array()>=local_max.array()).any() ||
        !robot_size.allFinite() ||
        (point_robot?!(robot_size.array()==0.).all():(robot_size.array()<=0.).any()) ||
        !center_offset.allFinite() || !positive(edge_min) || !positive(edge_max) ||
        edge_min>=edge_max || !nonnegative(overshoot) ||
        !positive(max_velocity) || !positive(traverse_length_max) || !positive(traverse_time_max) ||
        !positive(neighbor_radius) || !nonnegative(neighbor_min) ||
        !positive(neighbor_max) || neighbor_min>=neighbor_max ||
        max_vertices<2 || max_vertices>5000 || max_edges<1 || max_edges>100000 ||
        max_attempts<1 || max_attempts>1000000 ||
        (sampling_budget_mode!="raw_draws"&&sampling_budget_mode!="sampling_rounds") ||
        max_sampling_rounds<1 || max_sampling_rounds>1000000 ||
        empty_graph_cutoff<1 || !positive(sensor_range) ||
        !positive(horizontal_fov) || horizontal_fov>2*M_PI+1e-9 ||
        !positive(vertical_fov) || vertical_fov>M_PI ||
        !positive(horizontal_resolution) || !positive(vertical_resolution) ||
        std::ceil(horizontal_fov/horizontal_resolution)*std::ceil(vertical_fov/vertical_resolution)>10000 ||
        !nonnegative(unknown_gain) || !nonnegative(free_gain) || !nonnegative(occupied_gain) ||
        !nonnegative(path_length_penalty) || !nonnegative(path_direction_penalty) ||
        !nonnegative(clustering_radius) ||
        !nonnegative(frontier_threshold) || frontier_threshold>1 ||
        !sample_mean.allFinite() || !sample_scale.allFinite() || (sample_scale.array()<=0).any() ||
        (distribution!="uniform" && distribution!="normal" && distribution!="cauchy"))
      throw std::invalid_argument("Invalid local planner parameters");
    for(const auto& axis:samplingAxes())axis.validate();
    for(const auto& axis:samplingAxes(true))axis.validate();
    if(sensors.size()>32)throw std::invalid_argument("Too many exploration sensors");
    std::set<std::string> names;double rays=0.;
    for(const auto& sensor:sensors){
      sensor.validate();
      if(!names.insert(sensor.name).second)throw std::invalid_argument("Duplicate exploration sensor");
      rays+=std::ceil(sensor.horizontal_fov/sensor.horizontal_resolution)*
            std::ceil(sensor.vertical_fov/sensor.vertical_resolution);
    }
    if(rays>10000)throw std::invalid_argument("Aggregate sensor ray budget exceeded");
    if(no_gain_zones.size()>1000)throw std::invalid_argument("Too many no-gain zones");
    std::set<std::string> zone_names;
    for(const auto& zone:no_gain_zones){
      zone.validate();
      if(!zone_names.insert(zone.name).second)throw std::invalid_argument("Duplicate no-gain zone");
    }
  }
};

// Port of RandomSampler::generate: local XYZ samples are translated by the
// root after the inverse configured Y/P/R rotation. Cauchy components share
// one chi-square(1) draw, as in the upstream.
// Fixed seeds replace random_device to make migration experiments repeatable.
class LocalSampler {
 public:
  explicit LocalSampler(const Parameters& p):LocalSampler(p,p.samplingAxes()) {}
  LocalSampler(const Parameters& p,SamplingAxes axes):axes_(std::move(axes)),engine_(p.seed) {
    p.validate();sampling_to_world_=p.worldToSampling().transpose();
    for(const auto& axis:axes_){axis.validate();has_cauchy_=has_cauchy_||axis.distribution=="cauchy";}
  }
  StateVec generate(const StateVec& root) {
    StateVec out=root,offset;
    double chi=1.0;
    if(has_cauchy_) {
      std::chi_squared_distribution<double> dist(1.0);
      do { chi=dist(engine_); } while(chi<=0);
    }
    // Keep the previous XYZ-then-Heading draw order and distribution lifetime.
    // A constant consumes no random draw. Every Cauchy axis shares this chi.
    for(int i=0;i<4;++i) {
      const auto& axis=axes_[i];
      if(axis.distribution=="uniform")
        offset[i]=std::uniform_real_distribution<double>(axis.minimum,axis.maximum)(engine_);
      else if(axis.distribution=="normal")
        offset[i]=std::normal_distribution<double>(axis.mean,axis.scale)(engine_);
      else if(axis.distribution=="cauchy")
        offset[i]=std::normal_distribution<double>(0,axis.scale)(engine_)/std::sqrt(chi)+axis.mean;
      else offset[i]=axis.constant;
    }
    const Eigen::Vector3d xyz=offset.head<3>();
    out.head<3>()+=sampling_to_world_*xyz;
    out[3]+=offset[3];
    return out;
  }
  const SamplingAxes& axes()const{return axes_;}
 private:
  SamplingAxes axes_;
  bool has_cauchy_{false};
  Eigen::Matrix3d sampling_to_world_;
  std::mt19937 engine_;
};
}  // namespace gbplanner2
