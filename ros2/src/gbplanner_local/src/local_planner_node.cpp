#include "gbplanner_local/local_planner.hpp"
#include "gbplanner_local/planning_markers.hpp"
#include "gbplanner_local/frontier_markers.hpp"
#include "gbplanner_local/global_path_markers.hpp"
#include "gbplanner_local/refinement_markers.hpp"
#include "gbplanner_local/gain_markers.hpp"
#include "gbplanner_local/ground_edge_markers.hpp"
#include "gbplanner_local/state_history_markers.hpp"
#include "gbplanner_local/sampling_seed_sequence.hpp"
#include "gbplanner_local/search_planner.hpp"
#include "gbplanner_local/sensor_geometry.hpp"
#include "gbplanner_local/free_space.hpp"
#include "gbplanner_local/gate_planner.hpp"
#include "gbplanner_local/robot_bounds.hpp"
#include "gbplanner_local/path_validation.hpp"
#include "gbplanner_local/map_update_sequence.hpp"
#include "gbplanner_local/unknown_augmentation_schedule.hpp"
#include "gbplanner_local/exploration_direction.hpp"
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/frontier_switch.hpp"
#include "gbplanner_local/global_repositioning.hpp"
#include "gbplanner_local/path_refinement.hpp"
#include "gbplanner_local/geofence.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include "gbplanner_local/voxel_index_set.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <planner_msgs/srv/planner_geofence.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <planner_msgs/srv/planner_srv.hpp>
#include <planner_msgs/srv/planner_search.hpp>
#include <planner_msgs/srv/planner_go_to_waypoint.hpp>
#include <planner_msgs/srv/planner_request_path.hpp>
#include <planner_msgs/srv/planner_string_trigger.hpp>
#include <planner_msgs/srv/planner_set_planning_mode.hpp>
#include <planner_msgs/srv/plan_ahead.hpp>
#include <planner_msgs/srv/augment_free_box.hpp>
#include <planner_msgs/msg/voxel_map_update.hpp>
#include <planner_msgs/srv/planner_global.hpp>
#include <planner_msgs/srv/planner_homing.hpp>
#include <planner_msgs/srv/planner_set_homing_pos.hpp>
#include <planner_msgs/srv/planner_set_global_bound.hpp>
#include <planner_msgs/srv/planner_dynamic_global_bound.hpp>
#include <planner_msgs/srv/validate_path.hpp>
#include <planner_msgs/msg/robot_status.hpp>
#include <planner_semantic_msgs/msg/semantic_point.hpp>
#include <algorithm>
#include <array>
#include <cstring>
#include <functional>
#include <iomanip>
#include <sstream>
#include <optional>
#include <set>
#include <limits>
#include <map>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <atomic>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>

namespace gbplanner2 {
namespace {
// Used only after the owning ROS context has shut down. This is teardown,
// not malformed input or a recoverable cancellation of a map transaction.
struct MapShutdown final : std::exception {
  const char* what()const noexcept override{return "Map processing interrupted by shutdown";}
};
// Copy scalar evidence only; retaining diagnostics must not duplicate the
// result path or graph on the latency-sensitive service return path.
struct PlanPhaseMetrics {
  bool success,graph_build_completed,gain_evaluated,empty_graph_cutoff_reached;
  std::string reason,sampling_stop_reason;
  int vertices,edges,attempts,sampling_rounds,collision_rejections,gain_evaluations,reused_gains,best_id;
  double build_ms,evaluate_ms,gain_ms,refinement_ms,exploring_direction,path_length,gain,candidate_path_length;
  PathRefinementDiagnostics refinement;
  explicit PlanPhaseMetrics(const PlanResult& r)
      :success(r.success),graph_build_completed(r.graph_build_completed),gain_evaluated(r.gain_evaluated),
       empty_graph_cutoff_reached(r.empty_graph_cutoff_reached),reason(r.reason),sampling_stop_reason(r.sampling_stop_reason),
       vertices(r.vertices),edges(r.edges),attempts(r.attempts),sampling_rounds(r.sampling_rounds),
       collision_rejections(r.collision_rejections),gain_evaluations(r.gain_evaluations),reused_gains(r.reused_gains),best_id(r.best_id),
       build_ms(r.build_ms),evaluate_ms(r.evaluate_ms),gain_ms(r.gain_ms),refinement_ms(r.refinement_ms),
       exploring_direction(r.exploring_direction),path_length(r.path_length),gain(r.gain),
       candidate_path_length(r.candidate_path_length){
    // Phase metrics remain scalar: geometry belongs to the final request
    // result, not a second deep copy on the service timing path.
    refinement.attempted=r.refinement.attempted;refinement.modified=r.refinement.modified;
    refinement.reason=r.refinement.reason;refinement.removed_vertices=r.refinement.removed_vertices;
    refinement.adjusted_vertices=r.refinement.adjusted_vertices;
  }
};
// Wall time describes computation and dispatch latency, independently of the
// ROS clock used by mission budgeting. One record covers the entire callback.
struct PlanningServiceTiming {
  using Clock=std::chrono::steady_clock;
  Clock::time_point started{Clock::now()};
  double plan_ms{0.},absorb_local_ms{0.};
  bool ray_cache_enabled{false};
  ExactRayGainCacheStats ray_cache;
  TerrainQueryReuseStats terrain_cache;
  std::optional<PlanPhaseMetrics> phases;
  std::optional<SamplingSeed> sampling;
  struct Phase {
    double& elapsed;
    Clock::time_point started{Clock::now()};
    ~Phase(){elapsed+=std::chrono::duration<double,std::milli>(Clock::now()-started).count();}
  };
  template<class Callable> decltype(auto) measure(double& elapsed,Callable&& callback){
    Phase phase{elapsed};return std::forward<Callable>(callback)();
  }
  void log(const rclcpp::Logger& logger,const char* request,int map_revision)const {
    const double service_ms=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    RCLCPP_INFO(logger,"planner_request request=%s map_revision=%d plan_ms=%.3f absorb_local_ms=%.3f service_ms=%.3f ray_cache_enabled=%s ray_cache_hits=%zu ray_cache_evaluations=%zu ray_cache_entries=%zu",
      request,map_revision,plan_ms,absorb_local_ms,service_ms,ray_cache_enabled?"true":"false",
      ray_cache.hits,ray_cache.evaluations,ray_cache.entries);
    if(ray_cache_enabled)RCLCPP_INFO(logger,
      "terrain_query_reuse request=%s projection_hits=%zu projection_evaluations=%zu support_hits=%zu support_evaluations=%zu",
      request,terrain_cache.projection_hits,terrain_cache.projection_evaluations,
      terrain_cache.support_hits,terrain_cache.support_evaluations);
    if(phases)RCLCPP_INFO(logger,
      "planner_phases request=%s map_revision=%d build_ms=%.3f evaluate_ms=%.3f gain_ms=%.3f refinement_ms=%.3f vertices=%d edges=%d attempts=%d gain_evaluations=%d reused_gains=%d",
      request,map_revision,phases->build_ms,phases->evaluate_ms,phases->gain_ms,phases->refinement_ms,
      phases->vertices,phases->edges,phases->attempts,phases->gain_evaluations,phases->reused_gains);
  }
};
struct CaptureRayGainStats {
  const LocalPlanner& planner;
  PlanningServiceTiming& timing;
  CaptureRayGainStats(const LocalPlanner& planner,PlanningServiceTiming& timing):planner(planner),timing(timing){
    timing.ray_cache_enabled=true;
  }
  ~CaptureRayGainStats(){
    timing.ray_cache=planner.exactRayGainCacheStats();
    timing.terrain_cache=planner.terrainQueryReuseStats();
  }
};
}
class LocalPlannerNode final:public rclcpp::Node {
 public:
  LocalPlannerNode():Node("gbplanner_local",rclcpp::NodeOptions().use_clock_thread(true)) {
    home_diagnostic_directory_=parameter<std::string>("diagnostics.home_snapshot_directory","");
    validation_diagnostic_directory_=parameter<std::string>("diagnostics.validation_snapshot_directory","");
    declare_parameter<bool>("diagnostics.capture_next_validation_rejection",false);
    const auto map_trace=parameter<std::string>("diagnostics.map_trace_file","");
    if(!map_trace.empty()){
      map_trace_.open(map_trace,std::ios::out|std::ios::app);
      if(!map_trace_)throw std::invalid_argument("Cannot open diagnostics.map_trace_file");
      map_trace_timer_=create_wall_timer(std::chrono::seconds(1),[this]{
        traceMap("executor_tick",last_map_stamp_,0);
      });
    }
    frame_=parameter<std::string>("frame_id","map");
    if(frame_.empty())throw std::invalid_argument("Empty frame_id");
    resolution_=parameter<double>("map.resolution",0.25);
    map_storage_=parameter<std::string>("map.storage","dense");
    const auto max_snapshot_voxels=parameter<int64_t>("map.max_snapshot_voxels",4000000);
    if(max_snapshot_voxels<1||max_snapshot_voxels>100000000||
       max_snapshot_voxels>std::numeric_limits<int>::max())
      throw std::invalid_argument("map.max_snapshot_voxels must be in [1, 100000000]");
    max_snapshot_voxels_=static_cast<uint64_t>(max_snapshot_voxels);
    map_use_envelope_=parameter<bool>("map.use_update_envelope",false);
    augmentation_enabled_=parameter<bool>("planning.augment_free_voxels",false);
    augmentation_schedule_=UnknownAugmentationSchedule(integer("planning.augment_free_voxels_time",5));
    if(augmentation_enabled_&&!map_use_envelope_)
      throw std::invalid_argument("Unknown augmentation requires map.use_update_envelope");
    if(map_storage_!="dense"&&map_storage_!="sparse")
      throw std::invalid_argument("map.storage must be dense or sparse");
    origin_=vector("map.origin",{-5,-5,-1.5});
    auto dims=parameter<std::vector<int64_t>>("map.dimensions",{40,40,12});
    if(dims.size()!=3)throw std::invalid_argument("map.dimensions needs three integers");
    for(int i=0;i<3;++i){
      if(dims[i]<=0||dims[i]>10000)throw std::invalid_argument("Invalid map dimension");
      dimensions_[i]=static_cast<int>(dims[i]);
    }
    Eigen::Vector3d default_min,default_max;
    if(map_storage_=="sparse"){
      // A sparse layer has no enclosing allocated grid. Missing cells stay
      // unknown; workspace restrictions are configured independently below.
      SparseVoxelMap check(resolution_);default_min=check.minimum();default_max=check.maximum();
    }else{
      VoxelMap check(resolution_,origin_,dimensions_);default_min=check.minimum();default_max=check.maximum();
    }
    const auto global_min=vector("planning.global_min",default_min);
    const auto global_max=vector("planning.global_max",default_max);
    const auto global_type=parameter<std::string>("planning.global_type","kCuboid");
    const auto sphere_center=vector("planning.global_center",Eigen::Vector3d::Zero());
    const auto sphere_radius=parameter<double>("planning.global_radius",5.);
    if(global_type=="kSphere")default_global_bound_=GlobalBound::sphere(sphere_center,sphere_radius);
    else if(global_type=="kCuboid")default_global_bound_=GlobalBound::axisAligned(global_min,global_max);
    else throw std::invalid_argument("planning.global_type must be kCuboid or kSphere");
    global_bound_=default_global_bound_;
    p_.local_min=vector("sampling.min",p_.local_min);p_.local_max=vector("sampling.max",p_.local_max);
    p_.local_rotations=vector("sampling.rotations",p_.local_rotations);
    configureSamplingSpace("sampling",p_);
    p_.adaptive.enabled=parameter<bool>("planning.adaptive_exploration",false);
    p_.adaptive.type=parameter<std::string>("adaptive.type",p_.adaptive.type);
    p_.adaptive.range=parameter<double>("adaptive.local_pointcloud_range",p_.adaptive.range);
    p_.adaptive.size_max=parameter<double>("adaptive.bounding_box_size_max",p_.adaptive.size_max);
    p_.adaptive.scale_max=parameter<double>("adaptive.distribution_scaling_max",p_.adaptive.scale_max);
    p_.adaptive.leaf=parameter<double>("adaptive.voxel_filter_leaf_size",p_.adaptive.leaf);
    p_.robot_size=vector("robot.size",p_.robot_size);p_.center_offset=vector("robot.center_offset",p_.center_offset);
    robot_bounds_.physical_size=p_.robot_size;
    robot_bounds_.extension=vector("robot.size_extension",robot_bounds_.extension);
    robot_bounds_.min_extension=vector("robot.size_extension_min",robot_bounds_.min_extension);
    robot_bounds_.relax_ratio=parameter<double>("robot.relax_ratio",robot_bounds_.relax_ratio);
    robot_bounds_.validate();
    p_.refinement.enabled=parameter<bool>("planning.path_safety_enhance_enable",false);
    p_.refinement.safety_extension=vector("robot.safety_extension",p_.refinement.safety_extension);
    p_.refinement.interpolation_distance=parameter<double>("planning.path_interpolation_distance",p_.refinement.interpolation_distance);
    p_.refinement.relaxed_corridor_multiplier=parameter<double>("planning.relaxed_corridor_multiplier",p_.refinement.relaxed_corridor_multiplier);
    p_.refinement.planning_backward=parameter<bool>("planning.planning_backward",false);
    p_.refinement.homing_backward=parameter<bool>("planning.homing_backward",false);
    const auto robot_type=parameter<std::string>("robot.type","kAerial");
    if(robot_type!="kAerial"&&robot_type!="kGround")throw std::invalid_argument("robot.type must be kAerial or kGround");
    p_.ground_robot=robot_type=="kGround";
    p_.ground.max_ground_height=parameter<double>("ground.max_ground_height",p_.ground.max_ground_height);
    p_.ground.robot_height=parameter<double>("ground.robot_height",p_.ground.robot_height);
    p_.ground.max_inclination=parameter<double>("ground.max_inclination",p_.ground.max_inclination);
    p_.ground.max_negative_inclination=parameter<double>("ground.max_negative_inclination",p_.ground.max_negative_inclination);
    p_.ground.projection_max_length=parameter<double>("ground.projection_max_length",p_.ground.projection_max_length);
    p_.ground.interpolate_projection_distance=parameter<bool>("ground.interpolate_projection_distance",p_.ground.interpolate_projection_distance);
    const auto tsdf_projection=parameter<std::string>("ground.tsdf_projection_mode","upstream_step");
    if(tsdf_projection=="upstream_step")p_.ground.tsdf_projection_mode=GroundTsdfProjectionMode::UpstreamStep;
    else if(tsdf_projection=="zero_crossing")p_.ground.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossing;
    else if(tsdf_projection=="zero_crossing_simplex")p_.ground.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
    else throw std::invalid_argument("ground.tsdf_projection_mode must be upstream_step, zero_crossing or zero_crossing_simplex");
    if(p_.ground.interpolate_projection_distance&&p_.ground.tsdf_projection_mode!=GroundTsdfProjectionMode::UpstreamStep&&
       map_storage_=="dense"&&((origin_/resolution_).array()-(origin_/resolution_).array().round()).abs().maxCoeff()>1e-8)
      throw std::invalid_argument("zero_crossing requires a world-zero voxel lattice");
    const auto ray_projection=parameter<std::string>("ground.ray_projection_mode","voxel_surface");
    if(ray_projection=="voxel_surface")p_.ground.ray_projection_mode=GroundRayProjectionMode::VoxelSurface;
    else if(ray_projection=="upstream_sample")p_.ground.ray_projection_mode=GroundRayProjectionMode::UpstreamSample;
    else throw std::invalid_argument("ground.ray_projection_mode must be voxel_surface or upstream_sample");
    if(p_.ground.ray_projection_mode==GroundRayProjectionMode::UpstreamSample&&map_storage_=="dense"&&
       ((origin_/resolution_).array()-(origin_/resolution_).array().round()).abs().maxCoeff()>1e-8)
      throw std::invalid_argument("upstream_sample requires a world-zero voxel lattice");
    p_.ground.hanging_vertex_penalty=parameter<double>("ground.hanging_vertex_penalty",p_.ground.hanging_vertex_penalty);
    p_.distribution=parameter<std::string>("sampling.distribution",p_.distribution);
    p_.sample_mean=vector("sampling.mean",p_.sample_mean);p_.sample_scale=vector("sampling.scale",p_.sample_scale);
    p_.axis_sampling=configureAxisSampling("sampling",p_.samplingAxes());
    p_.adaptive_axis_sampling=configureAxisSampling("adaptive.sampling",p_.samplingAxes());
    auto seed=parameter<int64_t>("sampling.seed",42);
    if(seed<0||seed>UINT32_MAX)throw std::invalid_argument("Invalid sampling seed");
    p_.seed=static_cast<uint32_t>(seed);
    sampling_seed_mode_=parameter<std::string>("sampling.seed_mode","fixed");
    if(sampling_seed_mode_!="fixed"&&sampling_seed_mode_!="sequence")
      throw std::invalid_argument("sampling.seed_mode must be fixed or sequence");
    local_sampling_seeds_=std::make_unique<SamplingSeedSequence>(p_.seed,sampling_seed_mode_=="sequence");
    p_.max_vertices=integer("planning.max_vertices",p_.max_vertices);
    p_.max_edges=integer("planning.max_edges",p_.max_edges);
    p_.max_attempts=integer("planning.max_attempts",p_.max_attempts);
    p_.sampling_budget_mode=parameter<std::string>("planning.sampling_budget_mode",p_.sampling_budget_mode);
    p_.max_sampling_rounds=integer("planning.max_sampling_rounds",p_.max_sampling_rounds);
    p_.empty_graph_cutoff=integer("planning.empty_graph_cutoff",p_.empty_graph_cutoff);
    p_.edge_min=parameter<double>("planning.edge_min",p_.edge_min);
    p_.edge_max=parameter<double>("planning.edge_max",p_.edge_max);
    p_.max_velocity=parameter<double>("planning.v_max",p_.max_velocity);
    p_.traverse_length_max=parameter<double>("planning.traverse_length_max",p_.edge_max);
    p_.traverse_time_max=parameter<double>("planning.traverse_time_max",p_.edge_max/p_.max_velocity);
    p_.overshoot=parameter<double>("planning.overshoot",p_.overshoot);
    p_.neighbor_radius=parameter<double>("planning.neighbor_radius",p_.neighbor_radius);
    p_.neighbor_min=parameter<double>("planning.neighbor_min",p_.neighbor_min);
    p_.neighbor_max=parameter<double>("planning.neighbor_max",p_.neighbor_max);
    p_.graph_mode=parameter<bool>("planning.graph_mode",p_.graph_mode);
    p_.tangent_yaw=parameter<bool>("planning.tangent_yaw",p_.tangent_yaw);
    p_.leaf_gain_only=parameter<bool>("planning.leaf_gain_only",p_.leaf_gain_only);
    p_.cluster_vertices_for_gain=parameter<bool>("planning.cluster_vertices_for_gain",p_.cluster_vertices_for_gain);
    p_.clustering_radius=parameter<double>("planning.clustering_radius",p_.clustering_radius);
    p_.path_length_penalty=parameter<double>("planning.path_length_penalty",p_.path_length_penalty);
    p_.path_direction_penalty=parameter<double>("planning.path_direction_penalty",p_.path_direction_penalty);
    p_.sensor_range=parameter<double>("sensor.range",p_.sensor_range);
    p_.horizontal_fov=parameter<double>("sensor.horizontal_fov",p_.horizontal_fov);
    p_.vertical_fov=parameter<double>("sensor.vertical_fov",p_.vertical_fov);
    p_.horizontal_resolution=parameter<double>("sensor.horizontal_resolution",p_.horizontal_resolution);
    p_.vertical_resolution=parameter<double>("sensor.vertical_resolution",p_.vertical_resolution);
    p_.unknown_gain=parameter<double>("gain.unknown",p_.unknown_gain);
    p_.gain.use_ray_model=parameter<bool>("planning.use_ray_model_for_volumetric_gain",p_.gain.use_ray_model);
    p_.gain.ray_method=parameter<std::string>("planning.ray_cast_method",p_.gain.ray_method);
    p_.gain.nonuniform_ray_cast=parameter<bool>("planning.nonuniform_ray_cast",p_.gain.nonuniform_ray_cast);
    p_.gain.reuse_ray_invariants=parameter<bool>("planning.reuse_ray_invariants",p_.gain.reuse_ray_invariants);
    p_.gain.ray_cast_step_size_multiplier=parameter<double>("planning.ray_cast_step_size_multiplier",p_.gain.ray_cast_step_size_multiplier);
    p_.gain.max_ray_samples=integer("planning.max_gain_ray_samples",static_cast<int>(p_.gain.max_ray_samples));
    p_.gain.exp_gain_voxel_size=parameter<double>("planning.exp_gain_voxel_size",p_.gain.exp_gain_voxel_size);
    p_.gain.max_voxels=integer("planning.max_gain_voxels",static_cast<int>(p_.gain.max_voxels));
    p_.free_gain=parameter<double>("gain.free",p_.free_gain);
    p_.occupied_gain=parameter<double>("gain.occupied",p_.occupied_gain);
    p_.frontier_threshold=parameter<double>("gain.frontier_threshold",p_.frontier_threshold);
    const auto sensor_names=parameter<std::vector<std::string>>("sensor.names",{});
    const auto exploration_sensors=parameter<std::vector<std::string>>("planning.exploration_sensors",sensor_names);
    if(sensor_names.size()>32)throw std::invalid_argument("Too many planning sensors");
    std::map<std::string,SensorParameters> models;
    for(const auto& name:sensor_names){
      if(name.empty()||models.count(name)||
         name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")!=std::string::npos)
        throw std::invalid_argument("Invalid or duplicate planning sensor name");
      SensorParameters sensor;sensor.name=name;const std::string ns="sensor."+name+".";
      sensor.type=parameter<std::string>(ns+"type","kLidar");
      sensor.range=parameter<double>(ns+"range",p_.sensor_range);
      sensor.min_range=parameter<double>(ns+"min_range",0.);
      sensor.center_offset=vector(ns+"center_offset",Eigen::Vector3d::Zero());
      sensor.rotations=vector(ns+"rotations",Eigen::Vector3d::Zero());
      sensor.horizontal_fov=parameter<double>(ns+"horizontal_fov",p_.horizontal_fov);
      sensor.vertical_fov=parameter<double>(ns+"vertical_fov",p_.vertical_fov);
      sensor.horizontal_resolution=parameter<double>(ns+"horizontal_resolution",p_.horizontal_resolution);
      sensor.vertical_resolution=parameter<double>(ns+"vertical_resolution",p_.vertical_resolution);
      sensor.frontier_threshold=parameter<double>(ns+"frontier_threshold",p_.frontier_threshold);
      sensor.validate();models.emplace(name,sensor);
    }
    if(!sensor_names.empty()&&exploration_sensors.empty())throw std::invalid_argument("Exploration sensor list is empty");
    std::set<std::string> selected;
    for(const auto& name:exploration_sensors){
      const auto found=models.find(name);
      if(found==models.end()||!selected.insert(name).second)throw std::invalid_argument("Unknown or duplicate exploration sensor");
      p_.sensors.push_back(found->second);
    }
    const auto gain_zones=parameter<std::vector<std::string>>("gain.no_gain_zones",{});
    for(const auto& name:gain_zones){
      if(name.empty()||name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")!=std::string::npos)
        throw std::invalid_argument("Invalid no-gain zone name");
      GainZone zone;zone.name=name;const std::string ns="gain."+name+".";
      zone.type=parameter<std::string>(ns+"type","kCuboid");
      zone.center=vector(ns+"center",Eigen::Vector3d::Zero());
      zone.rotations=vector(ns+"rotations",Eigen::Vector3d::Zero());
      zone.minimum=vector(ns+"minimum",Eigen::Vector3d::Constant(-1.));
      zone.maximum=vector(ns+"maximum",Eigen::Vector3d::Constant(1.));
      zone.radius=parameter<double>(ns+"radius",1.);
      p_.no_gain_zones.push_back(zone);
    }
    p_.validate();
    configureFreeSpace();
    configureGate();
    // The upstream search sampler has a separate LocalSearch configuration.
    search_p_=p_;search_p_.adaptive.enabled=false;
    search_p_.local_min=vector("search.sampling.min",p_.local_min);
    search_p_.local_max=vector("search.sampling.max",p_.local_max);
    search_p_.local_rotations=vector("search.sampling.rotations",p_.local_rotations);
    configureSamplingSpace("search.sampling",search_p_);
    search_p_.distribution=parameter<std::string>("search.sampling.distribution",p_.distribution);
    search_p_.sample_mean=vector("search.sampling.mean",p_.sample_mean);
    search_p_.sample_scale=vector("search.sampling.scale",p_.sample_scale);
    search_p_.axis_sampling=configureAxisSampling("search.sampling",search_p_.samplingAxes(),search_p_.axis_sampling);
    const auto search_seed=parameter<int64_t>("search.sampling.seed",p_.seed);
    if(search_seed<0||search_seed>UINT32_MAX)throw std::invalid_argument("Invalid search sampling seed");
    search_p_.seed=static_cast<uint32_t>(search_seed);search_p_.validate();
    search_options_.max_vertices=integer("search.max_vertices",search_options_.max_vertices);
    search_options_.max_edges=integer("search.max_edges",search_options_.max_edges);
    search_options_.max_attempts=integer("search.max_attempts",search_options_.max_attempts);
    search_options_.empty_graph_cutoff=integer("search.empty_graph_cutoff",search_options_.empty_graph_cutoff);
    search_options_.max_target_candidates=integer("search.max_target_candidates",search_options_.max_target_candidates);
    search_options_.target_radius=parameter<double>("search.target_radius",search_options_.target_radius);
    search_options_.refine_path=parameter<bool>("search.refine_path",false);
    search_options_.validate();
    geofence_enabled_=parameter<bool>("planning.geofence_checking_enable",false);
    geofence_subscriber_enabled_=parameter<bool>("geofence.polygon_subscriber",true);
    auto configured_fences=std::make_shared<GeofenceSet>();
    // Port the original GeofenceParams/AreaList rectangular startup regions.
    const auto areas=parameter<std::vector<std::string>>("geofence.areas",{});
    if(areas.size()>1000)throw std::invalid_argument("Too many initial geofence areas");
    std::set<std::string> area_names;
    for(const auto& name:areas){
      if(name.empty()||!area_names.insert(name).second||
         name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")!=std::string::npos)
        throw std::invalid_argument("Invalid or duplicate geofence area name");
      const auto center=vector("geofence."+name+".center",Eigen::Vector3d::Zero());
      const auto size=vector("geofence."+name+".size",Eigen::Vector3d::Zero());
      if(!center.allFinite()||!size.allFinite()||size.x()<=0||size.y()<=0||size.z()<0)
        throw std::invalid_argument("Invalid initial geofence rectangle");
      std::vector<Eigen::Vector2d> polygon;
      for(const auto& sign:std::vector<Eigen::Vector2d>{{1,1},{1,-1},{-1,-1},{-1,1}})
        polygon.emplace_back(center.x()+sign.x()*size.x()/2,center.y()+sign.y()*size.y()/2);
      std::string reason;if(!configured_fences->add(polygon,&reason))throw std::invalid_argument(reason);
      geofence_polygons_.push_back(std::move(polygon));
    }
    geofences_=configured_fences;
    geofence_tf_=std::make_unique<tf2_ros::Buffer>(get_clock());
    geofence_listener_=std::make_shared<tf2_ros::TransformListener>(*geofence_tf_);
    if(parameter<bool>("global.enable",false))global_=std::make_unique<GlobalMemory>(p_);
    frontier_switch_.setEnabled(parameter<bool>("planning.auto_global_planner_enable",true));
    go_home_if_fully_explored_=parameter<bool>("planning.go_home_if_fully_explored",false);
    auto_home_=parameter<bool>("global.auto_home",true);
    mission_enabled_=parameter<bool>("mission.enabled",false);
    mission_auto_homing_=parameter<bool>("mission.auto_homing",true);
    const double mission_time_limit=parameter<double>("mission.time_limit",480.);
    landing_configured_=parameter<bool>("mission.auto_landing_enable",false);
    landing_enabled_=landing_configured_&&!p_.ground_robot;
    landing_time_limit_=parameter<double>("mission.time_budget_before_landing",mission_time_limit);
    if(!std::isfinite(landing_time_limit_)||landing_time_limit_<=0)
      throw std::invalid_argument("Invalid mission.time_budget_before_landing");
    // The original aerial auto-landing policy takes precedence over timed
    // homing. This does not disable auto_home_, which initializes graph home.
    if(landing_enabled_)mission_auto_homing_=false;
    mission_speed_=parameter<double>("mission.homing_speed",0.9);
    if(!std::isfinite(mission_speed_)||mission_speed_<=0)throw std::invalid_argument("Invalid mission.homing_speed");
    mission_=std::make_unique<MissionBudget>(mission_time_limit,
      parameter<double>("mission.battery_timeout",2.),parameter<bool>("mission.require_battery",false));
    if(mission_enabled_&&!global_)throw std::invalid_argument("Mission budget requires global.enable");
    mission_->observeClock(now().seconds());
    background_enabled_=parameter<bool>("global.background_enable",false);
    background_attempts_=integer("global.background_attempts",64);
    background_budget_mode_=parameter<std::string>("global.background_budget_mode","raw_draws");
    background_time_budget_=parameter<double>("global.background_time_budget",0.1);
    background_period_=parameter<double>("global.background_period",0.5);
    if((background_budget_mode_!="raw_draws"&&background_budget_mode_!="time")||
       background_attempts_<1||background_attempts_>10000||
       !std::isfinite(background_time_budget_)||background_time_budget_<=0||
       !std::isfinite(background_period_)||background_period_<.05)
      throw std::invalid_argument("Invalid background expansion budget/period");
    if(background_enabled_&&!global_)throw std::invalid_argument("Background expansion requires global.enable");
    const auto durable=rclcpp::QoS(1).reliable().transient_local();
    path_pub_=create_publisher<nav_msgs::msg::Path>("local_path",durable);
    graph_pub_=create_publisher<planner_msgs::msg::Graph>("local_graph",durable);
    metrics_pub_=create_publisher<std_msgs::msg::String>("planning_metrics",durable);
    ahead_metrics_pub_=create_publisher<std_msgs::msg::String>("plan_ahead_metrics",durable);
    sampling_status_pub_=create_publisher<std_msgs::msg::String>("sampling_status",durable);
    search_path_pub_=create_publisher<nav_msgs::msg::Path>("search_path",durable);
    search_graph_pub_=create_publisher<planner_msgs::msg::Graph>("search_graph",durable);
    search_metrics_pub_=create_publisher<std_msgs::msg::String>("search_metrics",durable);
    adaptive_metrics_pub_=create_publisher<std_msgs::msg::String>("adaptive_sampling",durable);
    sampling_marker_pub_=create_publisher<visualization_msgs::msg::Marker>("sampling_bound_marker",durable);
    sensor_frusta_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("sensor_frusta",durable);
    planning_diagnostics_enabled_=parameter<bool>("visualization.planning_diagnostics",false);
    const auto gain_vertex=parameter<int64_t>("visualization.gain_vertex_id",20);
    if(gain_vertex<-1||gain_vertex>1000000)throw std::invalid_argument("Invalid visualization.gain_vertex_id");
    gain_vertex_id_=static_cast<int>(gain_vertex);
    sampler_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/sampler",durable);
    projected_graph_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/planning_projected_graph",durable);
    failed_edges_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/planning_failed",durable);
    shortest_paths_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/shortest_paths",durable);
    best_paths_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/best_planning_paths",durable);
    frontier_clusters_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/shortest_path_clustering",durable);
    frontier_clustering_status_pub_=create_publisher<std_msgs::msg::String>("frontier_clustering",durable);
    global_paths_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/global_paths",durable);
    global_path_geometry_pub_=create_publisher<std_msgs::msg::String>("global_path_geometry",durable);
    homing_path_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/homing_path",durable);
    homing_path_geometry_pub_=create_publisher<std_msgs::msg::String>("homing_path_geometry",durable);
    publishGlobalPaths({},"clear",false,"not_requested");
    refinement_cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("vis/refinement_corridor",durable);
    refinement_planes_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/refinement_hyperplanes",durable);
    refined_path_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/mod_path",durable);
    refinement_geometry_pub_=create_publisher<std_msgs::msg::String>("refinement_geometry",durable);
    publishRefinement({},"clear",false);
    gain_rays_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/rays",durable);
    gain_samples_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/volumetric_gains",durable);
    gain_geometry_pub_=create_publisher<std_msgs::msg::String>("planning_gain_geometry",durable);
    ground_edge_markers_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/ground_edge_checks",durable);
    ground_edge_geometry_pub_=create_publisher<std_msgs::msg::String>("ground_edge_geometry",durable);
    publishGroundEdges({},"clear");
    no_gain_zones_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/no_gain_zone",durable);
    visualization_geometry_pub_=create_publisher<std_msgs::msg::String>("visualization_geometry",durable);
    {
      // Accepted PointCloud2 messages do not encode voxel side length. Publish
      // the configured geometry once so a late display cannot guess SMB sizes.
      std::ostringstream out;out<<std::setprecision(17)<<"{\"frame_id\":"<<std::quoted(frame_)
        <<",\"map_resolution\":"<<resolution_<<",\"robot_size\":["
        <<robot_bounds_.physical_size.x()<<','<<robot_bounds_.physical_size.y()<<','<<robot_bounds_.physical_size.z()
        <<"],\"robot_center_offset\":["<<p_.center_offset.x()<<','<<p_.center_offset.y()<<','<<p_.center_offset.z()<<"]}";
      std_msgs::msg::String geometry;geometry.data=out.str();visualization_geometry_pub_->publish(geometry);
    }
    map_update_pub_=create_publisher<std_msgs::msg::String>("map_updates",durable);
    accepted_map_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("accepted_voxel_map",durable);
    global_path_pub_=create_publisher<nav_msgs::msg::Path>("global_path",durable);
    waypoint_path_pub_=create_publisher<nav_msgs::msg::Path>("waypoint_path",durable);
    waypoint_metrics_pub_=create_publisher<std_msgs::msg::String>("waypoint_metrics",durable);
    gate_path_pub_=create_publisher<nav_msgs::msg::Path>("gate_path",durable);
    gate_metrics_pub_=create_publisher<std_msgs::msg::String>("gate_metrics",durable);
    global_graph_pub_=create_publisher<planner_msgs::msg::Graph>("global_graph",durable);
    state_history_pub_=create_publisher<std_msgs::msg::String>("state_history",durable);
    state_history_markers_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/state_history",durable);
    global_metrics_pub_=create_publisher<std_msgs::msg::String>("global_metrics",durable);
    geofence_marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/geofence",durable);
    geofence_service_=create_service<planner_msgs::srv::PlannerGeofence>("gbplanner/geofence",
      [this](std::shared_ptr<planner_msgs::srv::PlannerGeofence::Request> request,
             std::shared_ptr<planner_msgs::srv::PlannerGeofence::Response> response){
        try{
          if(!geofence_enabled_)throw std::invalid_argument("geofence_disabled");
          if(request->rectangles.empty()||request->rectangles.size()>1000)throw std::invalid_argument("invalid_rectangle_count");
          auto next=std::make_shared<GeofenceSet>(*geofences_);
          auto polygons=geofence_polygons_;
          for(const auto& rectangle:request->rectangles){
            const auto& c=rectangle.center;const auto& d=rectangle.size;
            if(!std::isfinite(c.x)||!std::isfinite(c.y)||!std::isfinite(c.z)||
               !std::isfinite(d.x)||!std::isfinite(d.y)||!std::isfinite(d.z)||d.x<=0||d.y<=0||d.z<0)
              throw std::invalid_argument("invalid_rectangle_geometry");
            geometry_msgs::msg::Polygon polygon;
            for(const auto& sign:std::vector<Eigen::Vector2d>{{1,1},{1,-1},{-1,-1},{-1,1}}){
              geometry_msgs::msg::Point32 point;point.x=c.x+sign.x()*d.x/2;point.y=c.y+sign.y()*d.y/2;point.z=c.z;
              polygon.points.push_back(point);
            }
            auto points=polygonInMap(polygon,request->header);
            std::string reason;if(!next->add(points,&reason))throw std::invalid_argument(reason);
            polygons.push_back(std::move(points));
          }
          applyGeofences(next,std::move(polygons));response->success=true;
        }catch(const std::exception& error){rejectGeofence(error.what());response->success=false;}
      });
    geofence_clear_service_=create_service<std_srvs::srv::Trigger>("gbplanner/clear_untraversable_zones",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        applyGeofences(std::make_shared<GeofenceSet>(),{});response->success=true;response->message="cleared";
      });
    geofence_enable_service_=create_service<std_srvs::srv::SetBool>("gbplanner/enable_untraversable_polygon_subscriber",
      [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request,std::shared_ptr<std_srvs::srv::SetBool::Response> response){
        geofence_subscriber_enabled_=request->data;response->success=true;response->message=request->data?"enabled":"disabled";
      });
    geofence_status_service_=create_service<std_srvs::srv::Trigger>("geofence_status",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        std::ostringstream out;out<<"{\"enabled\":"<<(geofence_enabled_?"true":"false")
          <<",\"subscriber_enabled\":"<<(geofence_subscriber_enabled_?"true":"false")
          <<",\"areas\":"<<geofences_->size()<<",\"revision\":"<<geofence_revision_
          <<",\"rejected\":"<<rejected_geofences_<<",\"ignored_polygons\":"<<ignored_geofences_
          <<",\"map_revision\":"<<map_revision_<<",\"observation_revision\":"<<observation_revision_<<"}";
        response->success=true;response->message=out.str();
      });
    geofence_sub_=create_subscription<geometry_msgs::msg::PolygonStamped>("/traversability_estimation/untraversable_polygon",100,
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr message){
        if(!geofence_enabled_||!geofence_subscriber_enabled_){++ignored_geofences_;return;}
        try{
          auto points=polygonInMap(message->polygon,message->header);auto next=std::make_shared<GeofenceSet>(*geofences_);
          std::string reason;if(!next->add(points,&reason))throw std::invalid_argument(reason);
          auto polygons=geofence_polygons_;polygons.push_back(std::move(points));applyGeofences(next,std::move(polygons));
        }catch(const std::exception& error){rejectGeofence(error.what());}
      });
    publishGeofences();
    global_bound_marker_pub_=create_publisher<visualization_msgs::msg::Marker>("global_bound_marker",durable);
    global_bound_service_=create_service<planner_msgs::srv::PlannerSetGlobalBound>("gbplanner/set_global_bound",
      [this](std::shared_ptr<planner_msgs::srv::PlannerSetGlobalBound::Request> request,
             std::shared_ptr<planner_msgs::srv::PlannerSetGlobalBound::Response> response){
        response->success=true;
        try{
          if(!request->get_current_bound){
            if(request->reset_to_default)applyGlobalBound(*default_global_bound_);
            else{
              Eigen::Vector3d lo=global_bound_->minimum(),hi=global_bound_->maximum();
              lo.x()=request->bound.min_val.x;lo.y()=request->bound.min_val.y;
              hi.x()=request->bound.max_val.x;hi.y()=request->bound.max_val.y;
              if(request->bound.use_z_val){lo.z()=request->bound.min_val.z;hi.z()=request->bound.max_val.z;}
              if(!odom_valid_)throw std::invalid_argument("odometry_missing_or_invalid");
              applyGlobalBound(GlobalBound::axisAligned(lo,hi));
            }
          }
        }catch(const std::exception& error){response->success=false;RCLCPP_WARN(get_logger(),"Global bound rejected: %s",error.what());}
        const auto lo=global_bound_->minimum(),hi=global_bound_->maximum();
        response->bound_ret.use_z_val=true;
        response->bound_ret.min_val.x=lo.x();response->bound_ret.min_val.y=lo.y();response->bound_ret.min_val.z=lo.z();
        response->bound_ret.max_val.x=hi.x();response->bound_ret.max_val.y=hi.y();response->bound_ret.max_val.z=hi.z();
      });
    dynamic_global_bound_service_=create_service<planner_msgs::srv::PlannerDynamicGlobalBound>("gbplanner/set_dynamic_global_bound",
      [this](std::shared_ptr<planner_msgs::srv::PlannerDynamicGlobalBound::Request> request,
             std::shared_ptr<planner_msgs::srv::PlannerDynamicGlobalBound::Response> response){
        response->success=false;
        try{
          if(request->reset_to_default)applyGlobalBound(*default_global_bound_);
          else{
            if(!odom_valid_)throw std::invalid_argument("odometry_missing_or_invalid");
            if(request->header.frame_id.empty())throw std::invalid_argument("empty_bound_frame");
            Eigen::Quaterniond rotation=Eigen::Quaterniond::Identity();Eigen::Vector3d translation=Eigen::Vector3d::Zero();
            if(request->header.frame_id!=frame_){
              const auto transform=geofence_tf_->lookupTransform(frame_,request->header.frame_id,rclcpp::Time(request->header.stamp));
              const auto& q=transform.transform.rotation;const auto& t=transform.transform.translation;
              rotation=Eigen::Quaterniond(q.w,q.x,q.y,q.z);translation={t.x,t.y,t.z};
              if(!rotation.coeffs().allFinite()||rotation.norm()<1e-8||!translation.allFinite())throw std::invalid_argument("invalid_bound_transform");
              rotation.normalize();
            }
            auto world=[&](const geometry_msgs::msg::Point& point)->Eigen::Vector3d{
              const Eigen::Vector3d p(point.x,point.y,point.z);
              if(!p.allFinite())throw std::invalid_argument("nonfinite_bound_corner");
              return rotation*p+translation;
            };
            applyGlobalBound(GlobalBound::fromCorners(world(request->center),world(request->front),world(request->left),world(request->up)));
          }
          response->success=true;
        }catch(const std::exception& error){RCLCPP_WARN(get_logger(),"Dynamic global bound rejected: %s",error.what());}
      });
    global_bound_status_service_=create_service<std_srvs::srv::Trigger>("global_bound",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=true;std::ostringstream out;out<<std::setprecision(12)<<"{\"revision\":"<<global_bound_revision_;
        out<<",\"type\":\""<<global_bound_->typeName()<<"\",\"radius\":"<<global_bound_->radius();
        const auto center=global_bound_->center(),half=global_bound_->halfSize();
        out<<",\"center\":["<<center.x()<<','<<center.y()<<','<<center.z()<<"],\"half_size\":["<<half.x()<<','<<half.y()<<','<<half.z()<<"],\"rotation\":[";
        for(int row=0;row<3;++row)for(int col=0;col<3;++col){if(row||col)out<<',';out<<global_bound_->rotation()(row,col);}
        out<<"]}";response->message=out.str();
      });
    publishGlobalBound();
    std_msgs::msg::Header visualization_header;visualization_header.frame_id=frame_;visualization_header.stamp=now();
    no_gain_zones_pub_->publish(noGainZoneMarkers(p_.no_gain_zones,visualization_header));
    publishPlanningPaths({});
    publishGainGeometry({},"clear");
    publishGroundEdges({},"clear");
    publishFrontierClustering({}, {}, "clear",false);
    if(map_use_envelope_)
      map_envelope_sub_=create_subscription<planner_msgs::msg::VoxelMapUpdate>("voxel_map_update",durable,
        [this](planner_msgs::msg::VoxelMapUpdate::ConstSharedPtr message){
          const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud(message,&message->map);
          if(receiveMapUpdate(*message,cloud)&&!home_diagnostic_directory_.empty())
            home_diagnostic_map_=sensor_msgs::msg::PointCloud2::ConstSharedPtr(message,&message->map);
          traceMap("callback_body_completed",message->map.header.stamp,message->map.data.size());
        });
    else map_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("voxel_map",durable,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message){
        if(receiveMap(*message,nullptr,nullptr,message)&&!home_diagnostic_directory_.empty())home_diagnostic_map_=message;
        traceMap("callback_body_completed",message->header.stamp,message->data.size());
      });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("odometry",10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message){
        traceMap("odometry_begin",message->header.stamp,0);
        odom_valid_=message->header.frame_id==frame_ && poseState(message->pose.pose,root_);
        free_space_odom_stamp_=odom_valid_?message->header.stamp:builtin_interfaces::msg::Time{};
        if(augmentation_enabled_&&odom_valid_)
          augmentOdometry();
        if(odom_valid_)publishSensorFrusta();
        traceMap("odometry_frusta_published",message->header.stamp,0);
        if(global_&&odom_valid_&&map_){recordGlobalOdometry();publishGlobalGraph();}
        traceMap("odometry_completed",message->header.stamp,0);
      });
    // Match Rrg::timerCallback: the exploration direction comes from measured
    // horizontal displacement, independently of heading, maps and global mode.
    direction_timer_=create_wall_timer(std::chrono::milliseconds(250),[this](){
      if(odom_valid_)exploration_direction_.update(root_);
    });
    direction_status_service_=create_service<std_srvs::srv::Trigger>("exploration_direction",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=true;std::ostringstream out;
        out<<std::setprecision(12)<<"{\"heading\":"<<exploration_direction_.heading()
           <<",\"samples\":"<<exploration_direction_.samples()
           <<",\"updates\":"<<exploration_direction_.updates()
           <<",\"path_direction_penalty\":"<<p_.path_direction_penalty<<'}';
        response->message=out.str();
      });
    validation_policy_=std::make_shared<const PathValidationPolicy>(PathValidationPolicy{
      frame_,p_.ground_robot,resolution_,robot_bounds_,p_.groundParameters(),p_.center_offset});
    mission_pub_=create_publisher<std_msgs::msg::String>("mission_status",durable);
    // Planning and map updates remain in the default mutually exclusive
    // group. Mission accounting has its own executor so a full RRG build
    // cannot delay battery expiry, the landing deadline or status delivery.
    mission_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    // Replies must remain live while the default planning callback waits for
    // the mapper's bounded, atomic root-box transaction.
    augmentation_client_=create_client<planner_msgs::srv::AugmentFreeBox>(
      "augment_free_box",rmw_qos_profile_services_default,mission_group_);
    rclcpp::SubscriptionOptions mission_subscription_options;
    mission_subscription_options.callback_group=mission_group_;
    mission_odom_sub_=create_subscription<nav_msgs::msg::Odometry>("odometry",10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message){
        PathValidationObservation observation;
        observation.valid=message->header.frame_id==validation_policy_->frame&&
            validation_policy_->poseState(message->pose.pose,observation.state);
        // Keep this short callback separate from both planning and validation:
        // even a long terrain query must not delay the next measured pose.
        validation_inputs_.publishObservation(observation);
        std::lock_guard<std::mutex> lock(mission_mutex_);
        mission_odom_valid_=observation.valid;
      },mission_subscription_options);
    battery_sub_=create_subscription<planner_msgs::msg::RobotStatus>("robot_status",10,
      [this](planner_msgs::msg::RobotStatus::ConstSharedPtr message){
        if(!mission_enabled_)return;
        const double stamp=rclcpp::Time(message->header.stamp).seconds();
        {
          std::lock_guard<std::mutex> lock(mission_mutex_);
          if(!mission_->updateBattery(message->time_remaining,stamp,now().seconds()))++rejected_battery_;
        }
        publishMission();
      },mission_subscription_options);
    mission_status_service_=create_service<std_srvs::srv::Trigger>("mission_state",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=true;response->message=missionStatus();
      },rmw_qos_profile_services_default,mission_group_);
    mission_timer_=create_wall_timer(std::chrono::milliseconds(100),[this](){publishMission();},mission_group_);
    augmentation_status_service_=create_service<std_srvs::srv::Trigger>("unknown_augmentation_status",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=true;std::ostringstream out;
        out<<"{\"enabled\":"<<(augmentation_enabled_?"true":"false")
          <<",\"completed_builds\":"<<augmentation_schedule_.completedBuilds()
          <<",\"build_limit\":"<<augmentation_schedule_.buildLimit()
          <<",\"odometry_window_open\":"<<(augmentation_schedule_.odometryWindowOpen()?"true":"false")
          <<",\"bound_mode\":"<<augmentation_bound_mode_
          <<",\"odometry_samples\":"<<augmentation_odometry_samples_
          <<",\"odometry_coalesced\":"<<augmentation_odometry_coalesced_
          <<",\"odometry_pending\":"<<(odometry_augmentation_?"true":"false")
          <<",\"requests\":"<<augmentation_requests_<<",\"changed_voxels\":"<<augmentation_changed_
          <<",\"failures\":"<<augmentation_failures_<<",\"last_reason\":"<<std::quoted(augmentation_last_reason_)
          <<",\"observation_revision\":"<<observation_revision_
          <<",\"map_revision\":"<<map_revision_
          <<",\"source_id\":"<<std::quoted(map_sequence_.current().source)
          <<",\"map_epoch\":"<<map_sequence_.current().epoch
          <<",\"update_id\":"<<map_sequence_.current().revision
          <<",\"observation_id\":"<<map_sequence_.current().observation
          <<",\"map_envelopes_received\":"<<map_envelopes_received_
          <<",\"map_envelopes_ignored\":"<<map_envelopes_ignored_
          <<",\"last_ignored_source_id\":"<<std::quoted(last_ignored_map_update_.source)
          <<",\"last_ignored_epoch\":"<<last_ignored_map_update_.epoch
          <<",\"last_ignored_update_id\":"<<last_ignored_map_update_.revision<<'}';
        response->message=out.str();
      });
    if(augmentation_enabled_)
      augmentation_poll_timer_=create_wall_timer(std::chrono::milliseconds(20),
        [this](){pollOdometryAugmentation();});
    // Rrg::stopMsgCallback clears the automatic trip regardless of Bool.data.
    // Keep this relative name compatible with the upstream PCI stop topic.
    planner_stop_sub_=create_subscription<std_msgs::msg::Bool>("planner_control_interface/stop_request",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr){global_repositioning_.clear();});
    for(const std::string name:{"planner_global","gbplanner/global"})
      compatibility_services_.push_back(create_service<planner_msgs::srv::PlannerGlobal>(name,
      [this](std::shared_ptr<planner_msgs::srv::PlannerGlobal::Request> request,
             std::shared_ptr<planner_msgs::srv::PlannerGlobal::Response> response){
        publishRefinement({},"clear",false);
        GlobalResult result;bool fully_explored_home=false;HomingPathDiagnostics homing_diagnostic;
        const bool automatic=request->automatic_repositioning;
        const bool apply_budget=automatic?mission_enabled_:!request->ignore_time;
        if(!global_)result.reason="global_disabled";
        else if(!map_||!odom_valid_)result.reason="map_or_odometry_missing";
        else if(automatic&&missionObservation().landing_due)result.reason="landing_requested";
        else if(automatic&&(request->id!=0||request->not_check_frontier))result.reason="invalid_automatic_global_request";
        else if(!automatic&&!request->ignore_time&&!mission_enabled_)result.reason="time_budget_disabled";
        else if(request->id<0)result.reason="invalid_global_target";
        else if(request->id>0&&!global_->graph().vertices_map_.count(request->id))result.reason="invalid_global_target";
        else if(request->not_check_frontier&&request->id==0)result.reason="explicit_target_required";
        else if(!RobotBounds::validMode(request->bound_mode))result.reason="unsupported_bound_mode";
        else try {
          selectGlobalBoundMode(request->bound_mode);
          LocalPlanner evaluator(map_,parametersForBound(p_,request->bound_mode));
          const auto budget=missionObservation().budget;
          if(apply_budget&&(!budget.valid||budget.remaining<=0))result.reason=budget.reason;
          else{
            GlobalTimeConstraint constraint{budget.remaining,mission_speed_,mission_auto_homing_,
              [this](){return missionObservation().budget;}};
            const auto retained=automatic?inspectRepositioning():GlobalRepositioningDecision{};
            const int target=retained.resume_global?retained.target:(request->id?request->id:-1);
            if(retained.state==GlobalRepositioningState::InvalidTarget||
               retained.state==GlobalRepositioningState::InvalidDistance)
              result.reason="global_repositioning_invalid_target";
            else result=global_->selectFrontier(root_,[&](const StateVec& state){return evaluator.globalRayGain(state);},
                target,apply_budget?&constraint:nullptr,
                request->id>0&&!request->not_check_frontier,captureRefinement(),captureGlobalPaths());
            // runGlobalPlanner's zero-frontier branch applies to this direct
            // service too, not only the local planner's fourth-round switch.
            // A retained but unreachable frontier is not fully explored.
            if(!retained.resume_global&&result.reason=="no_global_frontier"&&global_->frontierCount()==0){
              frontier_switch_.onGlobalFrontiers(false,request->not_check_frontier,go_home_if_fully_explored_);
              if(!request->not_check_frontier&&go_home_if_fully_explored_&&!landing_enabled_&&
                 global_->graph().vertices_map_.size()>1){
                fully_explored_home=true;result=global_->homing(root_,captureRefinement(),
                  captureHomingPath()?&homing_diagnostic:nullptr);
                if(result.success)result.reason="fully_explored_homing";
              }
            }
          }
        }catch(const std::exception& error){result.reason="global_error";RCLCPP_ERROR(get_logger(),"%s",error.what());}
        if(fully_explored_home)global_repositioning_.clear();
        else if(automatic&&result.success)global_repositioning_.activate(result.target);
        response->homing=fully_explored_home;
        response->path=publishGlobal(result,fully_explored_home?"homing":"frontier",&homing_diagnostic);
      }));
    for(const std::string name:{"exploration_homing","gbplanner/homing"})
      compatibility_services_.push_back(create_service<planner_msgs::srv::PlannerHoming>(name,
      [this](std::shared_ptr<planner_msgs::srv::PlannerHoming::Request> request,
             std::shared_ptr<planner_msgs::srv::PlannerHoming::Response> response){
        publishRefinement({},"clear",false);
        GlobalResult result;HomingPathDiagnostics homing_diagnostic;
        bool homing_called=false;
        if(request->header.frame_id!=frame_)result.reason="wrong_frame";
        else if(!global_)result.reason="global_disabled";
        else if(!map_||!odom_valid_)result.reason="map_or_odometry_missing";
        else if(!RobotBounds::validMode(request->bound_mode))result.reason="unsupported_bound_mode";
        else try{selectGlobalBoundMode(request->bound_mode);homing_called=true;
          result=global_->homing(root_,captureRefinement(),captureHomingPath()?&homing_diagnostic:nullptr);
        }catch(const std::exception& error){result.reason="homing_error";RCLCPP_ERROR(get_logger(),"%s",error.what());}
        // An accepted explicit return supersedes the previous exploration
        // task in this ROS 2 controller, including its retained destination.
        if(result.success)global_repositioning_.clear();
        if(homing_called&&!result.success&&home_diagnostic_map_)
          captureHomeSnapshot(*home_diagnostic_map_,"homing_failed",result.reason);
        response->path=publishGlobal(result,"homing",&homing_diagnostic);
      }));
    global_status_service_=create_service<std_srvs::srv::Trigger>("global_status",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=bool(global_);response->message="{\"enabled\":"+std::string(global_?"true":"false")+
          ",\"low_gain_iterations\":"+std::to_string(frontier_switch_.count())+
          ",\"repositioning_active\":"+(global_repositioning_.active()?"true":"false")+
          ",\"repositioning_target\":"+std::to_string(global_repositioning_.target())+
          ",\"bound_mode\":"+std::to_string(global_bound_mode_)+
          ",\"vertices\":"+std::to_string(global_?global_->graph().vertices_map_.size():0)+
          ",\"frontiers\":"+std::to_string(global_?global_->frontierCount():0)+
          ",\"edges\":"+std::to_string(global_?global_->edgeCount():0)+
          ",\"suspended_edges\":"+std::to_string(global_?global_->lastMapEdgeUpdate().pending:0)+
          ",\"last_invalidated_edges\":"+std::to_string(global_?global_->lastMapEdgeUpdate().invalidated:0)+
          ",\"last_restored_edges\":"+std::to_string(global_?global_->lastMapEdgeUpdate().restored:0)+"}";
      });
    sensor_status_service_=create_service<std_srvs::srv::Trigger>("sensor_gain",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=map_&&odom_valid_;
        if(!response->success){response->message="{\"reason\":\"map_or_odometry_missing\"}";return;}
        LocalPlanner evaluator(map_,p_);std::vector<SensorGain> gains;
        try{gains=evaluator.sensorGains(root_);}
        catch(const std::length_error& error){response->success=false;response->message="{\"reason\":\"gain_query_budget_exceeded\"}";return;}
        const auto& models=evaluator.sensorParameters();
        double total_gain=0.;bool frontier=false;
        std::ostringstream out;out<<std::setprecision(12)<<"{\"map_revision\":"<<map_revision_
          <<",\"clock_now_ns\":"<<now().nanoseconds()
          <<",\"odometry_stamp_ns\":"<<(int64_t(free_space_odom_stamp_.sec)*1000000000+free_space_odom_stamp_.nanosec)
          <<",\"root\":["<<root_[0]<<','<<root_[1]<<','<<root_[2]<<','<<root_[3]<<"],\"sensors\":[";
        for(std::size_t i=0;i<gains.size();++i){
          if(i)out<<',';const auto& entry=gains[i];const auto& model=models[i];
          total_gain+=entry.gain.gain;frontier=frontier||entry.gain.is_frontier;
          out<<"{\"name\":\""<<entry.name<<"\",\"type\":\""<<model.type<<"\",\"rays\":"<<entry.ray_count
            <<",\"gain_model\":\""<<entry.mode<<"\",\"voxel_count\":"<<entry.voxel_count
            <<",\"fov_voxel_count\":"<<entry.fov_voxel_count<<",\"visibility_queries\":"<<entry.visibility_queries
            <<",\"ray_method\":\""<<entry.ray_method<<"\",\"ray_grid\":["<<entry.ray_width<<','<<entry.ray_height<<']'
            <<",\"ray_queries\":"<<entry.raycast.queries<<",\"weighted_ray_samples\":"<<entry.raycast.weighted_samples
            <<",\"reused_free_prefixes\":"<<entry.raycast.reused_free_prefixes
            <<",\"reused_occupied_rays\":"<<entry.raycast.reused_occupied_rays
            <<",\"ray_step_increases\":"<<entry.raycast.step_increases
            <<",\"range\":"<<model.range<<",\"center_offset\":["<<model.center_offset.x()<<','<<model.center_offset.y()<<','<<model.center_offset.z()
            <<"],\"rotations\":["<<model.rotations.x()<<','<<model.rotations.y()<<','<<model.rotations.z()
            <<"],\"unknown\":"<<entry.gain.num_unknown_voxels<<",\"free\":"<<entry.gain.num_free_voxels
            <<",\"occupied\":"<<entry.gain.num_occupied_voxels<<",\"gain\":"<<entry.gain.gain
            <<",\"frontier\":"<<(entry.gain.is_frontier?"true":"false")<<'}';
        }
        out<<"],\"total_gain\":"<<total_gain<<",\"frontier\":"<<(frontier?"true":"false")
           <<",\"no_gain_zones\":"<<p_.no_gain_zones.size()<<'}';
        response->message=out.str();
      });
    for(const std::string name:{"planner","gbplanner"})
      compatibility_services_.push_back(create_service<planner_msgs::srv::PlannerSrv>(name,
      [this](const std::shared_ptr<planner_msgs::srv::PlannerSrv::Request> request,
             std::shared_ptr<planner_msgs::srv::PlannerSrv::Response> response){
        traceMap("local_request_begin",last_map_stamp_,0);
        PlanningServiceTiming timing;
        plan(*request,*response,timing);
        traceMap("local_response_ready",last_map_stamp_,0);
        publishGlobalGraph();
        timing.log(get_logger(),"local",map_revision_);
        traceMap("local_request_completed",last_map_stamp_,0);
      }));
    for(const std::string name:{"planner_search","gbplanner/search"})
      compatibility_services_.push_back(create_service<planner_msgs::srv::PlannerSearch>(name,
      [this](std::shared_ptr<planner_msgs::srv::PlannerSearch::Request> request,
             std::shared_ptr<planner_msgs::srv::PlannerSearch::Response> response){search(*request,*response);}));
    for(const std::string name:{"planner_passing_gate","gbplanner/passing_gate"})
      gate_services_.push_back(create_service<planner_msgs::srv::PlannerRequestPath>(name,
        [this](std::shared_ptr<planner_msgs::srv::PlannerRequestPath::Request>,
               std::shared_ptr<planner_msgs::srv::PlannerRequestPath::Response> response){passingGate(*response);}));
    for(const std::string name:{"planner_go_to_waypoint","gbplanner/go_to_waypoint"})
      waypoint_services_.push_back(create_service<planner_msgs::srv::PlannerGoToWaypoint>(name,
        [this](std::shared_ptr<planner_msgs::srv::PlannerGoToWaypoint::Request> request,
               std::shared_ptr<planner_msgs::srv::PlannerGoToWaypoint::Response> response){waypoint(*request,*response);}));
    plan_ahead_service_=create_service<planner_msgs::srv::PlanAhead>("planner_ahead",
      [this](const std::shared_ptr<planner_msgs::srv::PlanAhead::Request> request,
             std::shared_ptr<planner_msgs::srv::PlanAhead::Response> response){
        // Homing/connect and frontier absorption may retain graph changes even
        // when a budget/reference check makes planAhead return early.
        PlanningServiceTiming timing;
        planAhead(*request,*response,timing);publishGlobalGraph();
        timing.log(get_logger(),"ahead",map_revision_);
        publishAheadMetrics(*request,*response,timing);
      });
    set_home_service_=create_service<planner_msgs::srv::PlannerSetHomingPos>("gbplanner/set_homing_pos",
      [this](std::shared_ptr<planner_msgs::srv::PlannerSetHomingPos::Request>,
             std::shared_ptr<planner_msgs::srv::PlannerSetHomingPos::Response> response){
        response->success=global_&&map_&&odom_valid_&&global_->setHomingPosition(root_);
        if(response->success){
          publishGlobalGraph();publishGlobalPaths({},"clear",false,"home_changed");
          if(home_diagnostic_map_)captureHomeSnapshot(*home_diagnostic_map_,"home_set");
        }
      });
    status_service_=create_service<std_srvs::srv::Trigger>("map_status",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        std::ostringstream out;
        out<<"{\"map_revision\":"<<map_revision_<<",\"rejected_maps\":"<<rejected_maps_
          <<",\"known_cells\":"<<known_cells_<<",\"max_snapshot_voxels\":"<<max_snapshot_voxels_
          <<",\"last_rejected_map_reason\":"<<std::quoted(last_rejected_map_reason_)
          <<",\"tsdf_cells\":"<<tsdf_cells_<<",\"tsdf_fields\":"<<(tsdf_fields_?"true":"false")
          <<",\"storage\":"<<std::quoted(map_storage_)
          <<",\"ground_projection\":"<<std::quoted(p_.ground.interpolate_projection_distance?"tsdf":"occupancy")
          <<",\"ray_projection_mode\":"<<std::quoted(groundRayProjectionModeName(p_.ground.ray_projection_mode))
          <<",\"tsdf_projection_mode\":"<<std::quoted(groundTsdfProjectionModeName(p_.ground.tsdf_projection_mode))<<'}';
        res->success=bool(map_);res->message=out.str();
      });
    expansion_service_=create_service<std_srvs::srv::Trigger>("expand_global",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=expandBackground();response->message=backgroundStatus();
      });
    expansion_status_service_=create_service<std_srvs::srv::Trigger>("expansion_status",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=bool(global_);response->message=backgroundStatus();
      });
    validation_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    validation_service_=create_service<planner_msgs::srv::ValidatePath>("validate_path",
      [this](std::shared_ptr<planner_msgs::srv::ValidatePath::Request> request,
             std::shared_ptr<planner_msgs::srv::ValidatePath::Response> response){
        PathValidationInputs::Snapshot snapshot;
        PathValidationObservation observation;
        *response=validateCurrentPath(*validation_policy_,validation_inputs_,*request,&snapshot,&observation);
        captureValidationSnapshot(*request,*response,snapshot,observation);
      },rmw_qos_profile_services_default,validation_group_);
    // ROS 1 nh.createTimer follows simulated time. Pausing the simulation
    // must pause graph updates too; each callback's compute budget stays steady.
    if(background_enabled_)background_timer_=rclcpp::create_timer(
      this,get_clock(),rclcpp::Duration::from_seconds(background_period_),[this](){
        ++background_timer_ticks_;background_last_tick_ns_=now().nanoseconds();expandBackground();
      });
    configureSemantics();
    // A startup graph load publishes its restored semantic snapshot too.
    configureGraphStorage();
    planning_mode_service_=create_service<planner_msgs::srv::PlannerSetPlanningMode>("gbplanner/set_planning_trigger_mode",
      [this](std::shared_ptr<planner_msgs::srv::PlannerSetPlanningMode::Request> request,
             std::shared_ptr<planner_msgs::srv::PlannerSetPlanningMode::Response> response){
        response->success=request->planning_mode==request->K_MANUAL||request->planning_mode==request->K_AUTO;
        // Upstream Rrg stores this enum; it does not trigger motion itself.
        // The PCI remains responsible for the actual automatic control loop.
        if(response->success)planning_trigger_mode_=request->planning_mode;
      });
    RCLCPP_INFO(get_logger(),"Local %s RRG ready; waiting for explicit voxel map and odometry",p_.ground_robot?"ground":"aerial");
    // Add each independent group to its own executor, never the node/default
    // group. ROS time has the dedicated TimeSource thread selected above.
    rclcpp::ExecutorOptions mission_executor_options;
    mission_executor_options.context=get_node_base_interface()->get_context();
    mission_executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(mission_executor_options);
    mission_executor_->add_callback_group(mission_group_,get_node_base_interface());
    // ExecutorOptions owns a mutable MemoryStrategy shared_ptr. Reusing the
    // options would share wait-set storage across these concurrently spinning
    // executors, even though their callback groups are disjoint.
    rclcpp::ExecutorOptions validation_executor_options;
    validation_executor_options.context=mission_executor_options.context;
    validation_executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(validation_executor_options);
    validation_executor_->add_callback_group(validation_group_,get_node_base_interface());
    try{
      validation_thread_=std::thread([this]{
        try{
          const auto context=get_node_base_interface()->get_context();
          while(!validation_stopping_.load()&&rclcpp::ok(context))
            validation_executor_->spin_once(std::chrono::milliseconds(100));
        }catch(const std::exception& error){
          RCLCPP_ERROR(get_logger(),"Validation executor failed: %s",error.what());
          rclcpp::shutdown(get_node_base_interface()->get_context(),"Validation executor failed");
        }
      });
      mission_thread_=std::thread([this]{
        try{
          const auto context=get_node_base_interface()->get_context();
          while(!mission_stopping_.load()&&rclcpp::ok(context))
            mission_executor_->spin_once(std::chrono::milliseconds(100));
        }catch(const std::exception& error){
          RCLCPP_ERROR(get_logger(),"Mission executor failed: %s",error.what());
          rclcpp::shutdown(get_node_base_interface()->get_context(),"Mission executor failed");
        }
      });
    }catch(...){
      // If the second thread cannot start, join the first before constructor
      // unwinding destroys its executor or any state captured by its callback.
      validation_stopping_.store(true);mission_stopping_.store(true);
      if(validation_thread_.joinable())validation_thread_.join();
      if(mission_thread_.joinable())mission_thread_.join();
      throw;
    }
  }
  ~LocalPlannerNode() override {
    // A bounded spin_once also handles destruction before the new thread has
    // entered its executor; cancel() alone can lose that startup race.
    validation_stopping_.store(true);mission_stopping_.store(true);
    if(validation_thread_.joinable())validation_thread_.join();
    if(mission_thread_.joinable())mission_thread_.join();
  }
 private:
  void configureSemantics(){
    const auto durable=rclcpp::QoS(1).reliable().transient_local();
    semantic_path_pub_=create_publisher<nav_msgs::msg::Path>("semantic_path",durable);
    semantic_metrics_pub_=create_publisher<std_msgs::msg::String>("semantic_status",durable);
    semantic_targets_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("vis/semantic_targets",durable);
    semantic_sub_=create_subscription<planner_semantic_msgs::msg::SemanticPoint>("semantic_location",10,
      [this](planner_semantic_msgs::msg::SemanticPoint::ConstSharedPtr message){semanticLocation(*message);});
    semantic_status_service_=create_service<std_srvs::srv::Trigger>("semantic_state",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=bool(global_);response->message=semanticStatus();
      });
    publishSemanticTargets();
  }
  std::string semanticStatus()const{
    std::ostringstream out;out<<std::setprecision(12)<<"{\"received\":"<<semantic_received_
      <<",\"accepted\":"<<semantic_accepted_<<",\"success\":"<<(semantic_success_?"true":"false")
      <<",\"reason\":\""<<semantic_reason_<<"\",\"target_id\":"<<semantic_target_
      <<",\"attempts\":"<<semantic_attempts_<<",\"path_length_m\":"<<semantic_length_
      <<",\"bound_mode\":"<<global_bound_mode_<<",\"targets\":[";
    bool first=true;
    if(global_)for(const auto& item:global_->graph().vertices_map_){
      const auto* vertex=item.second;if(vertex->semantic_class.value==0)continue;
      if(!first)out<<',';first=false;const auto state=baseState(vertex->state);
      out<<"{\"id\":"<<vertex->id<<",\"class\":"<<vertex->semantic_class.value
        <<",\"position\":["<<state.x()<<','<<state.y()<<','<<state.z()<<"]}";
    }
    out<<"]}";return out.str();
  }
  void semanticLocation(const planner_semantic_msgs::msg::SemanticPoint& message){
    publishRefinement({},"clear",false);
    PathRefinementDiagnostics refinement;
    ++semantic_received_;semantic_success_=false;semantic_target_=-1;semantic_attempts_=0;semantic_length_=0.;
    nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=now();
    try{
      using Class=planner_semantic_msgs::msg::SemanticClass;
      if(!global_)throw std::invalid_argument("global_disabled");
      if(!map_||known_cells_==0)throw std::invalid_argument("map_missing");
      if(global_->graph().vertices_map_.empty())throw std::invalid_argument("global_graph_empty");
      if(message.type.value!=Class::K_STAIRCASE&&message.type.value!=Class::K_DOOR)
        throw std::invalid_argument("unsupported_semantic_class");
      if(message.header.frame_id.empty())throw std::invalid_argument("semantic_frame_missing");
      Eigen::Vector3d point(message.point.x,message.point.y,message.point.z);
      if(!point.allFinite())throw std::invalid_argument("invalid_semantic_point");
      if(message.header.frame_id!=frame_){
        const auto transform=geofence_tf_->lookupTransform(frame_,message.header.frame_id,rclcpp::Time(message.header.stamp));
        const auto& q=transform.transform.rotation;const auto& t=transform.transform.translation;
        Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);Eigen::Vector3d translation(t.x,t.y,t.z);
        if(!rotation.coeffs().allFinite()||rotation.norm()<1e-8||!translation.allFinite())
          throw std::invalid_argument("invalid_semantic_transform");
        point=rotation.normalized()*point+translation;
      }
      geometry_msgs::msg::Pose target_pose;target_pose.position.x=point.x();
      target_pose.position.y=point.y();target_pose.position.z=point.z();target_pose.orientation.w=1.;
      StateVec target;if(!poseState(target_pose,target))throw std::invalid_argument("invalid_semantic_point");
      const auto configured=parametersForPlanning(search_p_,global_bound_mode_,false);
      if(!validateSegment(target,target,configured.robot_size).empty())throw std::invalid_argument("semantic_target_not_free");
      // Original semanticsCallback searches at most five graph vertices within
      // twice the nearest distance. Sort explicitly so the choice is stable.
      std::vector<std::pair<double,Vertex*>> candidates;
      for(const auto& item:global_->graph().vertices_map_)
        candidates.emplace_back((item.second->state-target).head<3>().norm(),item.second);
      std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){
        return a.first==b.first?a.second->id<b.second->id:a.first<b.first;
      });
      const double range=2.*candidates.front().first+1e-8;
      semantic_reason_="semantic_connection_unavailable";
      for(const auto& candidate:candidates){
        if(semantic_attempts_>=5||candidate.first>range)break;
        ++semantic_attempts_;
        SearchPlanner planner(map_,configured,search_options_);
        auto result=planner.search(candidate.second->state,target);
        // A neighborhood fallback is useful to navigation, but must not move
        // the semantic label away from the observed object location.
        if(!result.success||!result.exact_target||result.path.empty())continue;
        const int id=global_->addSemanticPath(result.path,message.type.value);
        if(id<0)continue;
        refinement=std::move(result.refinement);
        semantic_target_=id;semantic_success_=true;semantic_reason_="ok";
        semantic_length_=result.path_length;++semantic_accepted_;
        for(const auto& planning:result.path){
          const auto state=baseState(planning);geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
          pose.pose.position.x=state.x();pose.pose.position.y=state.y();pose.pose.position.z=state.z();
          pose.pose.orientation.z=std::sin(state[3]/2);pose.pose.orientation.w=std::cos(state[3]/2);
          path.poses.push_back(pose);
        }
        break;
      }
    }catch(const tf2::TransformException&){semantic_reason_="semantic_transform_unavailable";}
    catch(const std::invalid_argument& error){semantic_reason_=error.what();}
    catch(const std::exception& error){semantic_reason_="semantic_connection_error";RCLCPP_WARN(get_logger(),"Semantic connection: %s",error.what());}
    publishGlobalGraph();
    semantic_path_pub_->publish(path);
    publishRefinement(refinement,"semantic",semantic_success_);
    std_msgs::msg::String status;status.data=semanticStatus();semantic_metrics_pub_->publish(status);
  }
  GlobalPersistenceResult graphStorage(bool load,const std::string& path){
    GlobalPersistenceResult result;
    if(!global_){result.reason="global_disabled";return result;}
    if(load){
      GlobalMemory::GainFunction gain;
      std::unique_ptr<LocalPlanner> evaluator;
      if(map_&&known_cells_>0){
        evaluator=std::make_unique<LocalPlanner>(map_,parametersForBound(p_,global_bound_mode_));
        gain=[&](const StateVec& state){return evaluator->globalRayGain(state);};
      }
      result=global_->loadGraph(path,frame_,gain);
      if(result.success){
        // A graph load replaces the ID namespace; no in-flight trip is part
        // of the serialized graph. Do not reuse an old ID in the new graph.
        global_repositioning_.clear();
        restored_graph_waiting_for_map_=known_cells_==0;
        clearPlanningDisplay();
        // Loading graph data does not advance Rrg's completed-build counter.
        // Preserve an existing eligibility, but do not create it on startup.
        publishGlobalGraph();
      }
    }else result=global_->saveGraph(path,frame_);
    last_graph_storage_=result;last_graph_operation_=load?"load":"save";
    if(!result.success)RCLCPP_WARN(get_logger(),"Global graph %s: %s",last_graph_operation_.c_str(),result.reason.c_str());
    return result;
  }
  void configureGraphStorage(){
    for(const bool load:{false,true}){
      const std::string name=load?"gbplanner/load_graph":"gbplanner/save_graph";
      graph_storage_services_.push_back(create_service<planner_msgs::srv::PlannerStringTrigger>(name,
        [this,load](std::shared_ptr<planner_msgs::srv::PlannerStringTrigger::Request> request,
                    std::shared_ptr<planner_msgs::srv::PlannerStringTrigger::Response> response){
          try{response->success=graphStorage(load,request->message).success;}
          catch(const std::exception& error){response->success=false;RCLCPP_WARN(get_logger(),"Graph storage: %s",error.what());}
        }));
    }
    graph_storage_status_=create_service<std_srvs::srv::Trigger>("global_graph_storage",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=bool(global_);std::ostringstream out;
        out<<"{\"operation\":\""<<last_graph_operation_<<"\",\"success\":"<<(last_graph_storage_.success?"true":"false")
          <<",\"map_revalidated\":"<<(last_graph_storage_.map_revalidated?"true":"false")
          <<",\"vertices\":"<<last_graph_storage_.vertices<<",\"active_edges\":"<<last_graph_storage_.active_edges
          <<",\"suspended_edges\":"<<last_graph_storage_.suspended_edges
          <<",\"frontiers\":"<<last_graph_storage_.frontiers
          <<",\"awaiting_map\":"<<(restored_graph_waiting_for_map_?"true":"false")<<'}';
        response->message=out.str();
      });
    const auto startup=parameter<std::string>("global.startup_graph","");
    if(!startup.empty()){
      const auto result=graphStorage(true,startup);
      if(!result.success)throw std::runtime_error("Startup global graph: "+result.reason);
    }
  }
  Parameters parametersForBound(const Parameters& base,int mode,bool scale_budget=true)const{
    auto configured=base;configured.robot_size=robot_bounds_.size(mode);
    configured.point_robot=RobotBounds::pointMode(mode);
    if(scale_budget){
      configured.max_vertices=RobotBounds::scaledBudget(base.max_vertices,mode);
      configured.max_edges=RobotBounds::scaledBudget(base.max_edges,mode);
    }
    configured.validate();return configured;
  }
  bool captureRefinement()const{
    const auto interested=[](const auto& publisher){
      return publisher&&(publisher->get_subscription_count()+publisher->get_intra_process_subscription_count()>0);
    };
    return planning_diagnostics_enabled_||interested(refinement_cloud_pub_)||
      interested(refinement_planes_pub_)||interested(refined_path_pub_)||interested(refinement_geometry_pub_);
  }
  bool captureGlobalPaths()const{
    const auto interested=[](const auto& publisher){
      return publisher&&(publisher->get_subscription_count()+publisher->get_intra_process_subscription_count()>0);
    };
    return planning_diagnostics_enabled_||interested(global_paths_pub_)||interested(global_path_geometry_pub_);
  }
  bool captureHomingPath()const{
    const auto interested=[](const auto& publisher){
      return publisher&&(publisher->get_subscription_count()+publisher->get_intra_process_subscription_count()>0);
    };
    return planning_diagnostics_enabled_||interested(homing_path_pub_)||interested(homing_path_geometry_pub_);
  }
  Parameters parametersForPlanning(const Parameters& base,int mode,bool scale_budget=true)const{
    auto configured=parametersForBound(base,mode,scale_budget);
    configured.refinement.capture_geometry=captureRefinement();return configured;
  }
  void publishRefinement(const PathRefinementDiagnostics& diagnostic,const std::string& request,bool response_success){
    std_msgs::msg::Header header;header.frame_id=frame_;header.stamp=now();
    refinement_cloud_pub_->publish(refinementCloud(diagnostic.geometry,header));
    refinement_planes_pub_->publish(refinementMarkers(diagnostic.geometry,header,response_success));
    refined_path_pub_->publish(refinedPathMarkers(diagnostic,header,response_success));
    std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha
      <<"{\"request\":"<<std::quoted(request)<<",\"map_revision\":"<<map_revision_
      <<",\"stamp_ns\":"<<rclcpp::Time(header.stamp).nanoseconds()<<",\"response_success\":"<<response_success
      <<",\"attempted\":"<<diagnostic.attempted<<",\"modified\":"<<diagnostic.modified
      <<",\"reason\":"<<std::quoted(diagnostic.reason)<<",\"geometry\":"
      <<refinementGeometryJson(diagnostic.geometry)<<",\"accepted_path\":"<<refinedPathJson(diagnostic)<<'}';
    std_msgs::msg::String status;status.data=out.str();refinement_geometry_pub_->publish(status);
  }
  void selectGlobalBoundMode(int mode){
    if(!RobotBounds::validMode(mode))throw std::invalid_argument("Invalid bound mode");
    if(global_bound_mode_==mode)return;
    if(global_)global_->setPlanningParameters(parametersForBound(p_,mode));
    global_bound_mode_=mode;
  }
  static std::string refinementStatus(const PathRefinementDiagnostics& value){
    std::ostringstream out;out<<"{\"attempted\":"<<(value.attempted?"true":"false")
      <<",\"modified\":"<<(value.modified?"true":"false")<<",\"reason\":\""<<value.reason
      <<"\",\"removed_vertices\":"<<value.removed_vertices<<",\"adjusted_vertices\":"<<value.adjusted_vertices<<'}';
    return out.str();
  }
  std::vector<Eigen::Vector2d> polygonInMap(const geometry_msgs::msg::Polygon& polygon,
                                           const std_msgs::msg::Header& header){
    if(header.frame_id.empty())throw std::invalid_argument("empty_geofence_frame");
    if(polygon.points.size()<3||polygon.points.size()>4096)throw std::invalid_argument("invalid_polygon_size");
    Eigen::Quaterniond rotation=Eigen::Quaterniond::Identity();Eigen::Vector3d translation=Eigen::Vector3d::Zero();
    if(header.frame_id!=frame_){
      // Zero timestamp preserves original latest-TF convention. A supplied
      // timestamp is honored; missing transforms reject the complete update.
      const auto transform=geofence_tf_->lookupTransform(frame_,header.frame_id,rclcpp::Time(header.stamp));
      const auto& q=transform.transform.rotation;const auto& t=transform.transform.translation;
      rotation=Eigen::Quaterniond(q.w,q.x,q.y,q.z);translation={t.x,t.y,t.z};
      if(!rotation.coeffs().allFinite()||rotation.norm()<1e-8||!translation.allFinite())throw std::invalid_argument("invalid_geofence_transform");
      rotation.normalize();
    }
    std::vector<Eigen::Vector2d> result;
    for(const auto& point:polygon.points){
      Eigen::Vector3d p(point.x,point.y,point.z);if(!p.allFinite())throw std::invalid_argument("nonfinite_geofence_point");
      p=rotation*p+translation;result.push_back(p.head<2>());
    }
    return result;
  }
  void rejectGeofence(const std::string& reason){++rejected_geofences_;RCLCPP_WARN(get_logger(),"Geofence rejected: %s",reason.c_str());}
  void refreshPlanningMap(){
    if(!observed_map_){map_.reset();return;}
    if(geofence_enabled_)map_=std::make_shared<GeofenceMap>(observed_map_,geofences_);
    else map_=observed_map_;
    map_=std::make_shared<BoundedMap>(map_,*global_bound_);
  }
  void applyGlobalBound(const GlobalBound& next){
    if(odom_valid_&&!next.containsBox(baseState(root_).head<3>()+p_.center_offset,p_.robot_size))
      throw std::invalid_argument("current_robot_box_outside_bound");
    global_bound_=next;++global_bound_revision_;refreshPlanningMap();
    clearPlanningDisplay();
    if(map_){
      ++map_revision_;publishValidationMap();
      if(global_){global_->updateMap(map_,TerrainQueryReuse::ImmutableSnapshot);publishGlobalGraph();}
      publishMapUpdate("global_bound");
      nav_msgs::msg::Path empty;empty.header.frame_id=frame_;empty.header.stamp=now();
      path_pub_->publish(empty);global_path_pub_->publish(empty);
    }
    publishGlobalBound();
  }
  void publishGlobalBound(){
    visualization_msgs::msg::Marker marker;marker.header.frame_id=frame_;marker.header.stamp=now();
    marker.ns="global_bound";marker.id=0;marker.type=global_bound_->type()==GlobalBound::Type::Sphere?
        visualization_msgs::msg::Marker::SPHERE:visualization_msgs::msg::Marker::CUBE;
    marker.action=visualization_msgs::msg::Marker::ADD;
    const Eigen::Vector3d center=global_bound_->center(),size=2.*global_bound_->halfSize();
    const Eigen::Quaterniond rotation(global_bound_->rotation());
    marker.pose.position.x=center.x();marker.pose.position.y=center.y();marker.pose.position.z=center.z();
    marker.pose.orientation.x=rotation.x();marker.pose.orientation.y=rotation.y();marker.pose.orientation.z=rotation.z();marker.pose.orientation.w=rotation.w();
    marker.scale.x=size.x();marker.scale.y=size.y();marker.scale.z=size.z();
    marker.color.r=.1;marker.color.g=.6;marker.color.b=.9;marker.color.a=.08;
    global_bound_marker_pub_->publish(marker);
  }
  void publishMapUpdate(const std::string& kind){
    std::ostringstream update;update<<"{\"map_revision\":"<<map_revision_<<",\"known_cells\":"<<known_cells_
      <<",\"observation_revision\":"<<observation_revision_<<",\"change_kind\":\""<<kind<<"\""
      <<",\"stamp_sec\":"<<last_map_stamp_.sec<<",\"stamp_nanosec\":"<<last_map_stamp_.nanosec<<"}";
    std_msgs::msg::String notification;notification.data=update.str();map_update_pub_->publish(notification);
  }
  void applyGeofences(std::shared_ptr<const GeofenceSet> next,std::vector<std::vector<Eigen::Vector2d>> polygons){
    if(next->size()>1000)throw std::invalid_argument("geofence_area_limit");
    geofences_=std::move(next);geofence_polygons_=std::move(polygons);++geofence_revision_;refreshPlanningMap();
    clearPlanningDisplay();
    if(map_){
      ++map_revision_;publishValidationMap();
      if(global_){global_->updateMap(map_,TerrainQueryReuse::ImmutableSnapshot);publishGlobalGraph();}
      // Policy edits invalidate active commands, but do not invent a new scan.
      publishMapUpdate("geofence");
      nav_msgs::msg::Path empty;empty.header.frame_id=frame_;empty.header.stamp=now();
      path_pub_->publish(empty);global_path_pub_->publish(empty);
    }
    publishGeofences();
  }
  void publishGeofences(){
    visualization_msgs::msg::MarkerArray message;visualization_msgs::msg::Marker clear;
    clear.header.frame_id=frame_;clear.header.stamp=now();clear.action=visualization_msgs::msg::Marker::DELETEALL;message.markers.push_back(clear);
    for(std::size_t i=0;geofence_enabled_&&i<geofence_polygons_.size();++i){
      visualization_msgs::msg::Marker marker;marker.header=clear.header;marker.ns="geofence";marker.id=i;
      marker.type=visualization_msgs::msg::Marker::LINE_STRIP;marker.action=visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w=1;marker.scale.x=.04;marker.color.r=1.;marker.color.g=.25;marker.color.b=.15;marker.color.a=.9;
      for(const auto& p:geofence_polygons_[i]){geometry_msgs::msg::Point q;q.x=p.x();q.y=p.y();marker.points.push_back(q);}
      if(!marker.points.empty())marker.points.push_back(marker.points.front());message.markers.push_back(marker);
    }
    geofence_marker_pub_->publish(message);
  }
  struct MissionObservation {
    BudgetSnapshot budget;
    double timestamp{0.};
    bool odom_valid{false},landing_due{false};
    int rejected_battery{0};
  };
  MissionObservation missionObservation(){
    std::lock_guard<std::mutex> lock(mission_mutex_);
    MissionObservation observed;
    // Read ROS time after acquiring the lock. A time captured before waiting
    // for another callback could otherwise look like a backwards clock jump.
    observed.timestamp=now().seconds();
    observed.budget=mission_->snapshot(observed.timestamp);
    observed.odom_valid=mission_odom_valid_;
    // Update the latch from this same snapshot, not from a cached budget that
    // predates a clock failure observed by the other executor.
    if(landing_enabled_&&observed.odom_valid&&observed.budget.clock_valid&&
       observed.budget.elapsed>=landing_time_limit_)landing_due_=true;
    observed.landing_due=landing_due_;
    observed.rejected_battery=rejected_battery_;
    return observed;
  }
  std::string missionStatus(){
    const auto observed=missionObservation();
    const auto& budget=observed.budget;
    const bool due=observed.landing_due;
    const std::string landing_reason=!landing_configured_?"disabled":
        p_.ground_robot?"ground_robot":due?"deadline_reached":
        !budget.clock_valid?budget.reason:!observed.odom_valid?"awaiting_odometry":"waiting_deadline";
    std::ostringstream out;out<<std::setprecision(12)<<"{\"enabled\":"<<(mission_enabled_?"true":"false")
      <<",\"valid\":"<<(budget.valid?"true":"false")<<",\"remaining_s\":"<<budget.remaining
      <<",\"elapsed_s\":"<<budget.elapsed<<",\"reason\":\""<<budget.reason<<"\""
      <<",\"stamp_s\":"<<observed.timestamp
      <<",\"reserve_s\":"<<MissionBudget::kReserveSeconds<<",\"rejected_battery\":"<<observed.rejected_battery
      <<",\"auto_homing_effective\":"<<(mission_auto_homing_?"true":"false")
      <<",\"landing\":{\"configured\":"<<(landing_configured_?"true":"false")
      <<",\"enabled\":"<<(landing_enabled_?"true":"false")
      <<",\"clock_valid\":"<<(budget.clock_valid?"true":"false")
      <<",\"due\":"<<(due?"true":"false")
      <<",\"remaining_s\":"<<std::max(0.,landing_time_limit_-budget.elapsed)
      <<",\"reason\":\""<<landing_reason<<"\"}}";
    return out.str();
  }
  void publishMission(){
    // Never enter the planning/map trace stream from the mission executor.
    // JSON construction and DDS publication are outside the budget lock.
    std_msgs::msg::String message;message.data=missionStatus();mission_pub_->publish(message);
  }
  void recordGlobalOdometry(){
    if(global_&&(auto_home_||!global_->graph().vertices_map_.empty()))global_->recordOdometry(root_);
  }
  GlobalRepositioningDecision inspectRepositioning(){
    if(!global_repositioning_.active())return {};
    const Vertex* vertex=nullptr;
    if(global_){
      const auto& vertices=global_->graph().vertices_map_;
      const auto found=vertices.find(global_repositioning_.target());
      if(found!=vertices.end())vertex=found->second;
    }
    const auto decision=global_repositioning_.inspect(vertex?(root_-vertex->state).head<3>().norm():0.,vertex!=nullptr);
    // A removed/replaced vertex cannot be resumed. The helper deliberately
    // leaves this ROS 2 graph-lifecycle decision to its owner.
    if(decision.state==GlobalRepositioningState::InvalidTarget)global_repositioning_.clear();
    return decision;
  }
  void configureGate(){
    gate_.enabled=parameter<bool>("gate.enable",false);
    gate_.center=vector("gate.center",gate_.center);
    gate_.heading=parameter<double>("gate.heading",gate_.heading);
    gate_.center_search_radius=parameter<double>("gate.center_search_radius",gate_.center_search_radius);
    gate_.center_search_step=parameter<double>("gate.center_search_step",gate_.center_search_step);
    gate_.line_search_length=parameter<double>("gate.line_search_length",gate_.line_search_length);
    gate_.line_search_range=parameter<double>("gate.line_search_range",gate_.line_search_range);
    gate_.line_search_step=parameter<double>("gate.line_search_step",gate_.line_search_step);
    for(int mode=RobotBounds::Extended;mode<=RobotBounds::Minimum;++mode)
      gate_.robot_sizes.push_back(robot_bounds_.size(mode));
    gate_use_tf_=parameter<bool>("gate.load_from_frame",false);
    gate_frame_=parameter<std::string>("gate.center_frame_id","gate_center");
    gate_offset_=vector("gate.frame_offset",Eigen::Vector3d::Zero());
    if(gate_frame_.empty()||!gate_offset_.allFinite())throw std::invalid_argument("Invalid gate frame settings");
    gate_.validate();
  }
  void passingGate(planner_msgs::srv::PlannerRequestPath::Response& response){
    GateResult result;response.bound.mode=RobotBounds::Extended;
    if(!gate_.enabled)result.reason="gate_disabled";
    else if(!map_||!odom_valid_)result.reason="map_or_odometry_missing";
    else try{
      Eigen::Vector3d center=gate_.center;
      if(gate_use_tf_){
        // The original gate interface has no timestamp. Use its latest-TF
        // convention and configured heading; the frame supplies position only.
        const auto transform=geofence_tf_->lookupTransform(frame_,gate_frame_,tf2::TimePointZero);
        const auto& translation=transform.transform.translation;
        center=Eigen::Vector3d(translation.x,translation.y,translation.z)+gate_offset_;
      }
      result=GatePlanner(map_,p_,gate_).plan(root_,center,gate_.heading);
      if(result.success&&global_){
        selectGlobalBoundMode(result.bound_mode);
        if(global_->graph().vertices_map_.empty())global_->setHomingPosition(root_);
        if(!global_->addReferencePath(result.path)){
          result.success=false;result.reason="reference_path_unavailable";result.path.clear();
        }
        publishGlobalGraph();
      }
    }catch(const std::exception& error){result.success=false;result.path.clear();result.reason="gate_error";
      RCLCPP_WARN(get_logger(),"Gate search: %s",error.what());}
    nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=now();
    if(result.success){
      response.bound.mode=result.bound_mode;
      for(const auto& planning:result.path){
        const auto state=baseState(planning);geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
        pose.pose.position.x=state.x();pose.pose.position.y=state.y();pose.pose.position.z=state.z();
        pose.pose.orientation.z=std::sin(state[3]/2);pose.pose.orientation.w=std::cos(state[3]/2);
        response.path.push_back(pose.pose);path.poses.push_back(pose);
      }
    }
    gate_path_pub_->publish(path);
    std::ostringstream out;out<<std::setprecision(12)<<"{\"success\":"<<(result.success?"true":"false")
      <<",\"reason\":\""<<result.reason<<"\",\"bound_mode\":"<<result.bound_mode
      <<",\"centers_tested\":"<<result.centers_tested<<",\"lines_tested\":"<<result.lines_tested
      <<",\"path_length_m\":"<<result.path_length<<",\"center\":["<<result.center.x()<<','<<result.center.y()<<','<<result.center.z()
      <<"],\"robot_size\":["<<result.robot_size.x()<<','<<result.robot_size.y()<<','<<result.robot_size.z()
      <<"],\"map_revision\":"<<map_revision_<<'}';
    std_msgs::msg::String metrics;metrics.data=out.str();gate_metrics_pub_->publish(metrics);
  }
  void configureFreeSpace(){
    free_space_enabled_=parameter<bool>("planning.freespace_cloud_enable",false);
    free_space_odom_timeout_=parameter<double>("free_space.odometry_timeout",2.);
    if(!std::isfinite(free_space_odom_timeout_)||free_space_odom_timeout_<=0.)
      throw std::invalid_argument("Invalid free_space.odometry_timeout");
    const auto names=parameter<std::vector<std::string>>("free_space.sensor_names",{});
    if(names.size()>32)throw std::invalid_argument("Too many free-space sensors");
    std::set<std::string> unique;double ray_budget=0.;
    for(const auto& name:names){
      if(name.empty()||!unique.insert(name).second||
         name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")!=std::string::npos)
        throw std::invalid_argument("Invalid or duplicate free-space sensor name");
      FreeSpaceSensor configured;auto& sensor=configured.sensor;sensor.name=name;
      const std::string ns="free_space."+name+".";
      // FreeFrustum models are independent of exploration/gain sensors.
      configured.frame=parameter<std::string>(ns+"frame","");
      sensor.type=parameter<std::string>(ns+"type",sensor.type);
      sensor.range=parameter<double>(ns+"range",sensor.range);
      sensor.min_range=parameter<double>(ns+"min_range",sensor.min_range);
      sensor.center_offset=vector(ns+"center_offset",sensor.center_offset);
      sensor.rotations=vector(ns+"rotations",sensor.rotations);
      sensor.horizontal_fov=parameter<double>(ns+"horizontal_fov",sensor.horizontal_fov);
      sensor.vertical_fov=parameter<double>(ns+"vertical_fov",sensor.vertical_fov);
      sensor.horizontal_resolution=parameter<double>(ns+"horizontal_resolution",sensor.horizontal_resolution);
      sensor.vertical_resolution=parameter<double>(ns+"vertical_resolution",sensor.vertical_resolution);
      sensor.frontier_threshold=parameter<double>(ns+"frontier_threshold",sensor.frontier_threshold);
      sensor.validate();if(configured.frame.empty())throw std::invalid_argument("Empty free-space sensor frame");
      ray_budget+=std::ceil(sensor.horizontal_fov/sensor.horizontal_resolution)*
          std::ceil(sensor.vertical_fov/sensor.vertical_resolution);
      free_space_sensors_.push_back(std::move(configured));
    }
    if(ray_budget>10000)throw std::invalid_argument("Aggregate free-space ray budget exceeded");
    if(free_space_enabled_&&free_space_sensors_.empty())throw std::invalid_argument("Free-space generation needs sensor models");
    free_space_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("freespace_pointcloud",rclcpp::SensorDataQoS());
    free_space_status_service_=create_service<std_srvs::srv::Trigger>("free_space_status",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        std::ostringstream out;out<<"{\"enabled\":"<<(free_space_enabled_?"true":"false")
          <<",\"sensors\":"<<free_space_sensors_.size()<<",\"clouds\":"<<free_space_clouds_
          <<",\"rays\":"<<free_space_rays_<<",\"blocked_rays\":"<<free_space_blocked_
          <<",\"points\":"<<free_space_points_<<",\"skipped_updates\":"<<free_space_skipped_
          <<",\"failed_updates\":"<<free_space_failed_<<"}";
        response->success=true;response->message=out.str();
      });
    free_space_timer_=create_wall_timer(std::chrono::milliseconds(500),[this]{publishFreeSpace();});
  }
  void publishFreeSpace(){
    if(!free_space_enabled_)return;
    const double stamp=double(free_space_odom_stamp_.sec)+1e-9*free_space_odom_stamp_.nanosec;
    const double age=now().seconds()-stamp;
    if(!map_||!odom_valid_||stamp<=0.||age<-.1||age>free_space_odom_timeout_){++free_space_skipped_;return;}
    try{
      // root_ is at planning height for ground robots. The free-frustum sensor
      // geometry and the stamped TF belong to the physical body pose instead.
      const auto clouds=buildFreeSpaceClouds(*map_,baseState(root_),free_space_sensors_);
      for(const auto& cloud:clouds){
        sensor_msgs::msg::PointCloud2 message;message.header.frame_id=cloud.frame;
        message.header.stamp=free_space_odom_stamp_;message.height=1;message.is_dense=true;
        sensor_msgs::PointCloud2Modifier modifier(message);
        modifier.setPointCloud2Fields(3,"x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(cloud.points.size());
        sensor_msgs::PointCloud2Iterator<float> x(message,"x"),y(message,"y"),z(message,"z");
        for(const auto& point:cloud.points){
          const Eigen::Vector3f value=point.cast<float>();
          if(!value.allFinite())throw std::invalid_argument("Free-space endpoint exceeds FLOAT32 range");
          *x=value.x();*y=value.y();*z=value.z();++x;++y;++z;
        }
        free_space_pub_->publish(message);++free_space_clouds_;
        free_space_rays_+=cloud.tested_rays;free_space_blocked_+=cloud.blocked_rays;
        free_space_points_+=cloud.points.size();
      }
    }catch(const std::exception& error){
      ++free_space_failed_;RCLCPP_WARN(get_logger(),"Free-space cloud skipped: %s",error.what());
    }
  }
  template<class T>T parameter(const std::string& name,const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
    return declare_parameter<T>(name,value,descriptor);
  }
  void configureSamplingSpace(const std::string& prefix,Parameters& parameters){
    const auto type=parameter<std::string>(prefix+".space_type","kCuboid");
    const double radius=parameter<double>(prefix+".radius",3.);
    const double extension=parameter<double>(prefix+".radius_extension",0.);
    parameters.gain_min_extension=vector(prefix+".min_extension",Eigen::Vector3d::Zero());
    parameters.gain_max_extension=vector(prefix+".max_extension",Eigen::Vector3d::Zero());
    if(type=="kSphere"){
      if(!std::isfinite(radius)||radius<=0||!std::isfinite(extension)||
         !std::isfinite(radius+extension)||radius+extension<=0)
        throw std::invalid_argument("Invalid "+prefix+" sphere radius");
      // The source local-sphere branch draws an axis-aligned radius cube;
      // it does not reject samples by radial distance. Its extension applies
      // only to the non-ray gain ROI. Keep those actual source semantics.
      parameters.local_min=Eigen::Vector3d::Constant(-radius);
      parameters.local_max=Eigen::Vector3d::Constant(radius);
      parameters.gain_min_extension=Eigen::Vector3d::Constant(-extension);
      parameters.gain_max_extension=Eigen::Vector3d::Constant(extension);
    }else if(type!="kCuboid")throw std::invalid_argument(prefix+".space_type must be kCuboid or kSphere");
  }
  SamplingOverrides configureAxisSampling(const std::string& prefix,const SamplingAxes& defaults,
                                           SamplingOverrides inherited={}){
    const auto& overrides=get_node_parameters_interface()->get_parameter_overrides();
    const std::array<std::string,4> names{"x","y","z","heading"};
    for(int i=0;i<4;++i){
      auto& axis=inherited[i];const auto& fallback=defaults[i];
      const std::string ns=prefix+"."+names[i]+".";
      const auto distribution=parameter<std::string>(ns+"distribution",fallback.distribution);
      if(overrides.count(ns+"distribution"))axis.distribution=distribution;
      const auto scalar=[&](const std::string& field,double value,std::optional<double>& target){
        const auto configured=parameter<double>(ns+field,value);
        if(overrides.count(ns+field))target=configured;
      };
      scalar("min",fallback.minimum,axis.minimum);scalar("max",fallback.maximum,axis.maximum);
      scalar("mean",fallback.mean,axis.mean);scalar("scale",fallback.scale,axis.scale);
      scalar("constant",fallback.constant,axis.constant);
    }
    return inherited;
  }
  int integer(const std::string& name,int value) {
    auto v=parameter<int64_t>(name,value);
    if(v<0||v>1000000)throw std::invalid_argument("Integer parameter out of range");
    return static_cast<int>(v);
  }
  Eigen::Vector3d vector(const std::string& name,const Eigen::Vector3d& value) {
    auto v=parameter<std::vector<double>>(name,{value.x(),value.y(),value.z()});
    if(v.size()!=3)throw std::invalid_argument(name+" requires three values");
    return Eigen::Vector3d(v[0],v[1],v[2]);
  }
  StateVec baseState(const StateVec& planning)const{
    StateVec result=planning;
    if(p_.ground_robot)result.z()+=p_.ground.robot_height-p_.ground.max_ground_height;
    return result;
  }
  // All external poses describe base_link. Graphs and algorithm paths retain
  // the upstream planning height; convert once at the ROS boundary.
  bool poseState(const geometry_msgs::msg::Pose& pose,StateVec& state)const {
    Eigen::Quaterniond q(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    state<<pose.position.x,pose.position.y,pose.position.z,0;
    if(!state.allFinite()||!q.coeffs().allFinite()||q.norm()<1e-8)return false;
    q.normalize();state[3]=std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
    if(p_.ground_robot)state.z()+=p_.ground.max_ground_height-p_.ground.robot_height;
    return true;
  }
  std::string validateSegment(const StateVec& a,const StateVec& b,const Eigen::Vector3d& size,
                              bool observed_first=false)const{
    const auto snapshot=std::make_shared<const PathValidationMap>(PathValidationMap{map_,map_revision_});
    return PathValidationContext(*validation_policy_,snapshot).validateSegment(a,b,size,observed_first);
  }
  void publishValidationMap(){
    std::shared_ptr<const std::string> metadata;
    if(!validation_diagnostic_directory_.empty())try{
      // Serialize on the map-owning executor. The validator must never read
      // mutable node members to describe an older immutable map revision.
      std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
      const auto vector=[&](const auto& value){
        out<<'[';for(int i=0;i<value.size();++i){if(i)out<<',';out<<value[i];}out<<']';
      };
      out<<"{\"frame\":"<<std::quoted(frame_)<<",\"storage\":"<<std::quoted(map_storage_)
         <<",\"resolution\":"<<resolution_<<",\"origin\":";vector(origin_);
      out<<",\"dimensions\":";vector(dimensions_);
      out<<",\"observation_revision\":"<<observation_revision_
         <<",\"observation_stamp_ns\":"<<rclcpp::Time(last_map_stamp_).nanoseconds()
         <<",\"map_use_update_envelope\":"<<map_use_envelope_;
      if(map_use_envelope_){
        const auto& source=map_sequence_.current();
        out<<",\"envelope\":{\"source_id\":"<<std::quoted(source.source)
           <<",\"map_epoch\":"<<source.epoch<<",\"update_id\":"<<source.revision
           <<",\"observation_id\":"<<source.observation<<",\"change_kind\":"<<std::quoted(source.kind)<<'}';
      }
      out<<",\"global_bound\":{\"revision\":"<<global_bound_revision_
         <<",\"type\":"<<std::quoted(global_bound_->typeName())<<",\"center\":";vector(global_bound_->center());
      out<<",\"half_size\":";vector(global_bound_->halfSize());
      out<<",\"radius\":"<<global_bound_->radius()<<",\"rotation\":[";
      for(int row=0;row<3;++row)for(int col=0;col<3;++col){if(row||col)out<<',';out<<global_bound_->rotation()(row,col);}
      out<<"]},\"geofence\":{\"enabled\":"<<geofence_enabled_<<",\"revision\":"<<geofence_revision_<<",\"polygons\":[";
      for(std::size_t i=0;i<geofence_polygons_.size();++i){if(i)out<<',';out<<'[';
        for(std::size_t j=0;j<geofence_polygons_[i].size();++j){if(j)out<<',';vector(geofence_polygons_[i][j]);}out<<']';}
      out<<"]}}";
      metadata=std::make_shared<const std::string>(out.str());
    }catch(const std::exception& error){
      RCLCPP_ERROR(get_logger(),"Validation snapshot metadata unavailable: %s",error.what());
    }
    validation_inputs_.publishMap(map_,map_revision_,validation_source_cloud_,std::move(metadata));
  }
  void captureValidationSnapshot(const planner_msgs::srv::ValidatePath::Request& request,
      const planner_msgs::srv::ValidatePath::Response& response,
      const PathValidationInputs::Snapshot& snapshot,const PathValidationObservation& observation){
    // Capture one rejected moving splice, or any rejection when explicitly armed.
    // The response and snapshot came from the same stable attempt; no extra query.
    if(validation_diagnostic_directory_.empty()||validation_diagnostic_captured_||response.valid||
       !snapshot)return;
    if(!(request.clearance_margin>0.)&&
       !get_parameter("diagnostics.capture_next_validation_rejection").as_bool())return;
    validation_diagnostic_captured_=true;
    try{
      if(!snapshot->source_cloud||!snapshot->diagnostic_metadata||snapshot->revision!=response.map_revision)
        throw std::runtime_error("Failure snapshot has no matching owned map bytes/metadata");
      const auto started=std::chrono::steady_clock::now();
      const auto directory=std::filesystem::path(validation_diagnostic_directory_)/
        ("validation_failed_revision"+std::to_string(snapshot->revision));
      std::filesystem::create_directories(directory.parent_path());
      if(!std::filesystem::create_directory(directory))throw std::runtime_error("Validation snapshot already exists");
      const auto& cloud=*snapshot->source_cloud;
      std::ofstream bytes(directory/"map.bin",std::ios::binary);
      bytes.write(reinterpret_cast<const char*>(cloud.data.data()),cloud.data.size());bytes.close();
      if(!bytes)throw std::runtime_error("Cannot write validation map bytes");
      std::ofstream out(directory/"snapshot.json");out<<std::setprecision(17)<<std::boolalpha;
      const auto number=[&](double value){if(std::isfinite(value))out<<value;else out<<"null";};
      const auto vector=[&](const auto& value){
        out<<'[';for(int i=0;i<value.size();++i){if(i)out<<',';number(value[i]);}out<<']';
      };
      const auto& policy=*validation_policy_;
      out<<"{\"status\":\"complete\",\"event\":\"validation_failed\",\"map_revision\":"<<snapshot->revision
         <<",\"response\":{\"valid\":"<<response.valid<<",\"reason\":"<<std::quoted(response.reason)
         <<",\"map_revision\":"<<response.map_revision<<",\"failure_segment\":"<<response.failure_segment
         <<"},\"request\":{\"frame\":"<<std::quoted(request.header.frame_id)
         <<",\"stamp_ns\":"<<(int64_t(request.header.stamp.sec)*1000000000LL+request.header.stamp.nanosec)
         <<",\"bound_mode\":"<<request.bound_mode<<",\"clearance_margin\":";number(request.clearance_margin);
      out<<",\"clearance_segments\":"<<request.clearance_segments
         <<",\"first_pose_is_observation\":"<<request.first_pose_is_observation<<",\"path\":[";
      for(std::size_t i=0;i<request.path.size();++i){if(i)out<<',';const auto& p=request.path[i];
        out<<"{\"position\":[";number(p.position.x);out<<',';number(p.position.y);out<<',';number(p.position.z);
        out<<"],\"quaternion_xyzw\":[";number(p.orientation.x);out<<',';number(p.orientation.y);out<<',';
        number(p.orientation.z);out<<',';number(p.orientation.w);out<<"]}";}
      out<<"]},\"observation\":{\"valid\":"<<observation.valid<<",\"planning_state\":";vector(observation.state);
      out<<"},\"map_metadata\":"<<*snapshot->diagnostic_metadata
         <<",\"cloud\":{\"frame\":"<<std::quoted(cloud.header.frame_id)
         <<",\"stamp_ns\":"<<rclcpp::Time(cloud.header.stamp).nanoseconds()
         <<",\"width\":"<<cloud.width<<",\"height\":"<<cloud.height<<",\"point_step\":"<<cloud.point_step
         <<",\"row_step\":"<<cloud.row_step<<",\"is_bigendian\":"<<cloud.is_bigendian
         <<",\"is_dense\":"<<cloud.is_dense<<",\"bytes\":"<<cloud.data.size()<<",\"fields\":[";
      for(std::size_t i=0;i<cloud.fields.size();++i){if(i)out<<',';const auto& f=cloud.fields[i];
        out<<"{\"name\":"<<std::quoted(f.name)<<",\"offset\":"<<f.offset
           <<",\"datatype\":"<<int(f.datatype)<<",\"count\":"<<f.count<<'}';}
      out<<"]},\"policy\":{\"frame\":"<<std::quoted(policy.frame)<<",\"ground_robot\":"<<policy.ground_robot
         <<",\"resolution\":"<<policy.resolution<<",\"center_offset\":";vector(policy.center_offset);
      const auto& bounds=policy.robot_bounds;
      out<<",\"robot_bounds\":{\"physical_size\":";vector(bounds.physical_size);
      out<<",\"extension\":";vector(bounds.extension);out<<",\"min_extension\":";vector(bounds.min_extension);
      out<<",\"relax_ratio\":"<<bounds.relax_ratio<<'}';
      if(RobotBounds::validMode(request.bound_mode)){
        out<<",\"robot_size\":";vector(bounds.size(request.bound_mode));
        out<<",\"margin_robot_size\":";vector(bounds.size(request.bound_mode)+Eigen::Vector3d::Constant(2.*request.clearance_margin));
      }
      const auto& g=policy.ground;
      out<<",\"ground\":{\"robot_size\":";vector(g.robot_size);
      out<<",\"center_offset\":";vector(g.center_offset);
      out<<",\"robot_height\":"<<g.robot_height<<",\"max_ground_height\":"<<g.max_ground_height
         <<",\"max_inclination\":"<<g.max_inclination<<",\"max_negative_inclination\":"<<g.max_negative_inclination
         <<",\"projection_max_length\":"<<g.projection_max_length<<",\"hanging_vertex_penalty\":"<<g.hanging_vertex_penalty
         <<",\"point_robot\":"<<g.point_robot<<",\"interpolate_projection_distance\":"<<g.interpolate_projection_distance
         <<",\"ray_projection_mode\":"<<std::quoted(groundRayProjectionModeName(g.ray_projection_mode))
         <<",\"tsdf_projection_mode\":"<<std::quoted(groundTsdfProjectionModeName(g.tsdf_projection_mode))<<"}}"
         <<",\"capture_wall_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<"}\n";
      out.close();if(!out)throw std::runtime_error("Cannot write validation snapshot metadata");
      RCLCPP_INFO(get_logger(),"Validation failure snapshot saved: %s",directory.c_str());
    }catch(const std::exception& error){
      // Do not replace the already computed response with diagnostic errors.
      RCLCPP_ERROR(get_logger(),"Validation failure capture failed: %s",error.what());
    }
  }
  // Opt-in failure capture runs on the same mutually exclusive callback as
  // map acceptance/homing. It binds exact input bytes, retained graph metadata
  // and current policy; asynchronous public topics cannot provide this identity.
  // Only one capture per event kind is written. Normal operation retains no
  // extra cloud and performs no diagnostic terrain queries or file writes.
  void captureHomeSnapshot(const sensor_msgs::msg::PointCloud2& cloud,const std::string& event,
                           const std::string& reason=""){
    if(home_diagnostic_directory_.empty()||!global_||!map_)return;
    try{
      if(home_diagnostic_captured_.count(event))return;
      const auto started=std::chrono::steady_clock::now();
      const auto diagnostic=global_->inspectHomeConnectivity();
      if(!diagnostic.present)return;
      home_diagnostic_captured_.insert(event);
      const auto directory=std::filesystem::path(home_diagnostic_directory_)/
        (event+"_revision"+std::to_string(map_revision_));
      std::filesystem::create_directories(directory.parent_path());
      if(!std::filesystem::create_directory(directory))throw std::runtime_error("Home snapshot already exists");
      std::ofstream bytes(directory/"map.bin",std::ios::binary);
      bytes.write(reinterpret_cast<const char*>(cloud.data.data()),cloud.data.size());bytes.close();
      if(!bytes)throw std::runtime_error("Cannot write home map snapshot");
      const auto saved=global_->saveGraph((directory/"graph.bin").string(),frame_);
      if(!saved.success)throw std::runtime_error("Cannot write home graph snapshot: "+saved.reason);
      std::ofstream out(directory/"snapshot.json");out<<std::setprecision(17)<<std::boolalpha;
      const auto vector=[&](const auto& value){
        out<<'[';for(int i=0;i<value.size();++i){if(i)out<<',';out<<value[i];}out<<']';
      };
      const auto number=[&](double value){if(std::isfinite(value))out<<value;else out<<"null";};
      const auto projection=[&](const GroundProjection& value){
        out<<"{\"status\":"<<std::quoted(groundStatusName(value.status))<<",\"state\":";
        vector(value.projected_state);out<<",\"ground_z\":";number(value.ground_z);out<<'}';
      };
      out<<"{\"status\":\"complete\",\"event\":"<<std::quoted(event)
         <<",\"reason\":"<<std::quoted(reason)
         <<",\"map_revision\":"<<map_revision_<<",\"observation_revision\":"<<observation_revision_
         <<",\"ros_time_ns\":"<<now().nanoseconds()<<",\"observation_stamp_ns\":"<<rclcpp::Time(last_map_stamp_).nanoseconds()
         <<",\"graph_revision\":"<<global_->snapshotRevision()<<",\"graph_vertices\":"<<saved.vertices
         <<",\"graph_active_edges\":"<<saved.active_edges<<",\"graph_suspended_edges\":"<<saved.suspended_edges
         <<",\"frame\":"<<std::quoted(frame_)<<",\"storage\":"<<std::quoted(map_storage_)
         <<",\"resolution\":"<<resolution_<<",\"origin\":";vector(origin_);
      out<<",\"dimensions\":";vector(dimensions_);
      out<<",\"cloud\":{\"frame\":"<<std::quoted(cloud.header.frame_id)
         <<",\"stamp_ns\":"<<rclcpp::Time(cloud.header.stamp).nanoseconds()
         <<",\"width\":"<<cloud.width<<",\"height\":"<<cloud.height<<",\"point_step\":"<<cloud.point_step
         <<",\"row_step\":"<<cloud.row_step<<",\"is_bigendian\":"<<cloud.is_bigendian
         <<",\"is_dense\":"<<cloud.is_dense<<",\"bytes\":"<<cloud.data.size()<<",\"fields\":[";
      for(std::size_t i=0;i<cloud.fields.size();++i){
        const auto& f=cloud.fields[i];if(i)out<<',';
        out<<"{\"name\":"<<std::quoted(f.name)<<",\"offset\":"<<f.offset
           <<",\"datatype\":"<<int(f.datatype)<<",\"count\":"<<f.count<<'}';
      }
      out<<"]},\"root\":";vector(root_);
      out<<",\"home\":";vector(diagnostic.home);
      out<<",\"home_active_degree\":"<<diagnostic.active_degree
         <<",\"home_body_status\":"<<int(diagnostic.body)<<",\"home_projection\":";
      projection(diagnostic.projected);
      if(diagnostic.ground){
        TerrainProjector terrain(map_,diagnostic.parameters);
        out<<",\"root_projection\":";projection(terrain.project(root_));
      }
      const auto& g=diagnostic.parameters;
      out<<",\"bound_mode\":"<<global_bound_mode_<<",\"ground_robot\":"<<diagnostic.ground
         <<",\"robot_size\":";vector(g.robot_size);
      out<<",\"center_offset\":";vector(g.center_offset);
      out<<",\"ground_parameters\":{\"robot_height\":"<<g.robot_height<<",\"max_ground_height\":"<<g.max_ground_height
         <<",\"max_inclination\":"<<g.max_inclination<<",\"max_negative_inclination\":"<<g.max_negative_inclination
         <<",\"projection_max_length\":"<<g.projection_max_length<<",\"hanging_vertex_penalty\":"<<g.hanging_vertex_penalty
         <<",\"point_robot\":"<<g.point_robot<<",\"interpolate_projection_distance\":"<<g.interpolate_projection_distance
         <<",\"ray_projection_mode\":"<<std::quoted(groundRayProjectionModeName(g.ray_projection_mode))
         <<",\"tsdf_projection_mode\":"<<std::quoted(groundTsdfProjectionModeName(g.tsdf_projection_mode))<<'}';
      out<<",\"global_bound\":{\"revision\":"<<global_bound_revision_
         <<",\"type\":"<<std::quoted(global_bound_->typeName())<<",\"center\":";vector(global_bound_->center());
      out<<",\"half_size\":";vector(global_bound_->halfSize());out<<",\"rotation\":[";
      for(int row=0;row<3;++row)for(int col=0;col<3;++col){if(row||col)out<<',';out<<global_bound_->rotation()(row,col);}
      out<<"]},\"geofence\":{\"enabled\":"<<geofence_enabled_<<",\"revision\":"<<geofence_revision_<<",\"polygons\":[";
      for(std::size_t i=0;i<geofence_polygons_.size();++i){if(i)out<<',';out<<'[';
        for(std::size_t j=0;j<geofence_polygons_[i].size();++j){if(j)out<<',';vector(geofence_polygons_[i][j]);}out<<']';}
      out<<"]},\"home_edges\":[";
      for(std::size_t i=0;i<diagnostic.edges.size();++i){
        const auto& edge=diagnostic.edges[i];if(i)out<<',';
        out<<"{\"neighbor\":"<<edge.neighbor<<",\"state\":";vector(edge.state);
        out<<",\"active\":"<<edge.active<<",\"fixed\":"<<edge.fixed
           <<",\"forward\":"<<std::quoted(groundStatusName(edge.forward))
           <<",\"reverse\":"<<std::quoted(groundStatusName(edge.reverse))<<",\"forward_length\":";
        number(edge.forward_length);out<<",\"reverse_length\":";number(edge.reverse_length);out<<'}';
      }
      out<<"],\"capture_wall_s\":"<<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<"}\n";
      out.close();if(!out)throw std::runtime_error("Cannot write home snapshot metadata");
      RCLCPP_INFO(get_logger(),"Home snapshot saved: %s",directory.c_str());
    }catch(const std::exception& error){
      // Diagnostics must never turn a successfully accepted map into a rejection.
      RCLCPP_ERROR(get_logger(),"Home snapshot capture failed: %s",error.what());
    }
  }
  void traceMap(const char* phase,const builtin_interfaces::msg::Time& stamp,std::size_t bytes){
    if(!map_trace_.is_open())return;
    const auto wall=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    timespec cpu{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu);
    // Optional local file avoids using the DDS path under investigation. A
    // flushed entry before/after each potentially blocking operation survives
    // a later stall; default runs incur only the unopened-stream check.
    map_trace_<<std::setprecision(15)<<"{\"phase\":\""<<phase<<"\",\"steady_s\":"<<wall
      <<",\"thread_cpu_s\":"<<(double(cpu.tv_sec)+double(cpu.tv_nsec)*1e-9)
      <<",\"stamp_ns\":"<<(int64_t(stamp.sec)*1000000000+stamp.nanosec)
      <<",\"map_revision\":"<<map_revision_<<",\"bytes\":"<<bytes<<"}\n";
    map_trace_.flush();
  }
  bool receiveMapUpdate(const planner_msgs::msg::VoxelMapUpdate& update,
                        sensor_msgs::msg::PointCloud2::ConstSharedPtr source_cloud={}){
    ++map_envelopes_received_;
    try{
      if(update.observation_stamp.nanosec>=1000000000u)
        throw std::invalid_argument("Invalid observation timestamp");
      MapUpdateSequence::Update metadata{update.source_id,update.map_epoch,update.update_id,update.observation_id,
        int64_t(update.observation_stamp.sec)*1000000000ll+update.observation_stamp.nanosec,update.change_kind};
      if(!map_sequence_.newer(metadata)){
        ++map_envelopes_ignored_;last_ignored_map_update_=metadata;return false;
      }
      return receiveMap(update.map,&metadata,&update.observation_stamp,std::move(source_cloud));
    }catch(const std::exception& error){
      ++rejected_maps_;last_rejected_map_reason_=error.what();
      RCLCPP_WARN(get_logger(),"Map update rejected: %s",error.what());return false;
    }
  }
  bool augmentBox(const Eigen::Vector3d& center,const Eigen::Vector3d& size,bool snapshot){
    ++augmentation_requests_;
    try{
      if(!augmentation_client_->service_is_ready())throw std::runtime_error("augmentation_service_unavailable");
      auto request=std::make_shared<planner_msgs::srv::AugmentFreeBox::Request>();
      request->header.frame_id=frame_;request->header.stamp=now();
      request->center.x=center.x();request->center.y=center.y();request->center.z=center.z();
      request->size.x=size.x();request->size.y=size.y();request->size.z=size.z();
      request->return_snapshot=snapshot;
      auto pending=augmentation_client_->async_send_request(request);
      if(pending.wait_for(std::chrono::seconds(2))!=std::future_status::ready){
        augmentation_client_->remove_pending_request(pending);
        throw std::runtime_error("augmentation_timeout");
      }
      const auto reply=pending.get();
      if(!reply->success)throw std::runtime_error("augmentation_rejected:"+reply->reason);
      augmentation_changed_+=reply->changed_voxels;
      if(snapshot){
        if(reply->snapshot.source_id!=reply->source_id||reply->snapshot.map_epoch!=reply->map_epoch||
           reply->snapshot.update_id!=reply->update_id||
           !receiveMapUpdate(reply->snapshot,
              sensor_msgs::msg::PointCloud2::ConstSharedPtr(reply,&reply->snapshot.map)))
          throw std::runtime_error("augmentation_snapshot_not_accepted");
      }
      augmentation_last_reason_=reply->reason;return true;
    }catch(const std::exception& error){
      ++augmentation_failures_;augmentation_last_reason_=error.what();
      RCLCPP_WARN(get_logger(),"Unknown-box augmentation: %s",error.what());return false;
    }
  }
  void pollOdometryAugmentation(){
    if(odometry_augmentation_){
      if(odometry_augmentation_->wait_for(std::chrono::seconds(0))==std::future_status::ready){
        try{
          const auto reply=odometry_augmentation_->get();
          if(!reply->success)throw std::runtime_error("augmentation_rejected:"+reply->reason);
          augmentation_changed_+=reply->changed_voxels;augmentation_last_reason_=reply->reason;
        }catch(const std::exception& error){++augmentation_failures_;augmentation_last_reason_=error.what();}
        odometry_augmentation_.reset();
      }else if(std::chrono::steady_clock::now()-odometry_augmentation_started_>std::chrono::seconds(2)){
        augmentation_client_->remove_pending_request(*odometry_augmentation_);
        odometry_augmentation_.reset();++augmentation_failures_;augmentation_last_reason_="augmentation_timeout";
      }
    }
  }
  void augmentOdometry(){
    pollOdometryAugmentation();
    if(!augmentation_schedule_.odometryWindowOpen())return;
    ++augmentation_odometry_samples_;
    // ROS 2 uses one in-flight box transaction. Intermediate odometry samples
    // are counted explicitly; the next accepted pose is sent once it completes.
    // This bounds DDS/service work at high odometry rates without claiming the
    // synchronous ROS 1 callback applied every intermediate pose.
    if(odometry_augmentation_){++augmentation_odometry_coalesced_;return;}
    if(!augmentation_client_->service_is_ready())return;
    const double age=now().seconds()-rclcpp::Time(free_space_odom_stamp_).seconds();
    if(age<-.1||age>free_space_odom_timeout_)return;
    // Rrg::setState uses the measured base pose, while the ROS 2 graph stores
    // ground planning height. Undo only that boundary conversion here.
    const Eigen::Vector3d center=baseState(root_).head<3>()+p_.center_offset;
    const Eigen::Vector3d size=robot_bounds_.size(augmentation_bound_mode_);
    auto request=std::make_shared<planner_msgs::srv::AugmentFreeBox::Request>();
    request->header.frame_id=frame_;request->header.stamp=free_space_odom_stamp_;
    request->center.x=center.x();request->center.y=center.y();request->center.z=center.z();
    request->size.x=size.x();request->size.y=size.y();request->size.z=size.z();
    request->return_snapshot=false;
    odometry_augmentation_started_=std::chrono::steady_clock::now();
    try{
      odometry_augmentation_.emplace(augmentation_client_->async_send_request(request));++augmentation_requests_;
    }catch(const std::exception& error){++augmentation_failures_;augmentation_last_reason_=error.what();}
  }
  bool augmentPlanningRoot(const StateVec& root,const Parameters& configured,int bound_mode){
    augmentation_bound_mode_=bound_mode;
    if(!augmentation_enabled_)return true;
    StateVec projected=root;
    if(p_.ground_robot){
      auto ground=configured.ground;ground.robot_size=configured.robot_size;
      ground.center_offset=configured.center_offset;ground.point_robot=configured.point_robot;
      const auto projection=TerrainProjector(map_,ground).project(root);
      // Do not invent a ground surface where the actual map has no support.
      // With support, preserve reset()'s original XY and projected planning Z,
      // including when the body box is Unknown or Occupied.
      if(!projection.support)return true;
      projected.z()=projection.projected_state.z();
    }
    const Eigen::Vector3d center=projected.head<3>()+configured.center_offset;
    if(map_->box(center,configured.robot_size)==VoxelStatus::Free)return true;
    // Separate from the odometry build-count window, exactly as Rrg::reset.
    // The mapper only fills unknown TSDF weights; occupied cells remain so.
    return augmentBox(center,configured.robot_size,true);
  }
  bool receiveMap(const sensor_msgs::msg::PointCloud2& message,
                  const MapUpdateSequence::Update* metadata=nullptr,
                  const builtin_interfaces::msg::Time* observation_stamp=nullptr,
                  sensor_msgs::msg::PointCloud2::ConstSharedPtr source_cloud={}) {
    const auto trace=[&](const char* phase){traceMap(phase,message.header.stamp,message.data.size());};
    const auto context=get_node_base_interface()->get_context();
    const auto shutdown_checkpoint=[&](){if(!rclcpp::ok(context))throw MapShutdown{};};
    // Record final local cleanup separately from executor dispatch and RMW
    // message handling, including rejected snapshots. The temporary duplicate
    // index is already released inside parse(), before the "parsed" marker.
    struct ReturnTrace {
      const decltype(trace)& report;
      ~ReturnTrace(){report("locals_destroyed");}
    } return_trace{trace};
    trace("received");
    try {
      shutdown_checkpoint();
      if(message.header.frame_id!=frame_||message.is_bigendian||message.height==0||message.point_step==0)
        throw std::invalid_argument("Map frame/endianness/shape invalid");
      const uint64_t count=uint64_t(message.width)*message.height;
      if(count>max_snapshot_voxels_)
        throw std::invalid_argument("Map snapshot voxel limit exceeded: "+std::to_string(count)+
          " > map.max_snapshot_voxels="+std::to_string(max_snapshot_voxels_));
      if(message.row_step<uint64_t(message.width)*message.point_step||
         message.data.size()!=uint64_t(message.row_step)*message.height)
        throw std::invalid_argument("Map byte length invalid");
      std::set<uint32_t> field_bytes;
      auto field=[&](const std::string& name,uint8_t datatype,uint32_t bytes){
        const sensor_msgs::msg::PointField* found=nullptr;
        for(const auto& f:message.fields)if(f.name==name){if(found)throw std::invalid_argument("Duplicate field");found=&f;}
        if(!found||found->datatype!=datatype||found->count!=1||uint64_t(found->offset)+bytes>message.point_step)
          throw std::invalid_argument("Map needs float32 x,y,z and int8 occupancy");
        for(uint32_t byte=found->offset;byte<found->offset+bytes;++byte)
          if(!field_bytes.insert(byte).second)throw std::invalid_argument("Overlapping map fields");
        return found->offset;
      };
      auto ox=field("x",7,4),oy=field("y",7,4),oz=field("z",7,4),os=field("occupancy",1,1);
      const auto has_field=[&](const std::string& name){
        return std::any_of(message.fields.begin(),message.fields.end(),
          [&](const auto& f){return f.name==name;});
      };
      const bool has_tsdf=has_field("tsdf_distance");
      if(has_tsdf!=has_field("tsdf_weight"))
        throw std::invalid_argument("Map TSDF distance and weight must occur together");
      uint32_t od=0,ow=0;
      if(has_tsdf){od=field("tsdf_distance",7,4);ow=field("tsdf_weight",7,4);}
      // Snapshot replacement is transactional; malformed data cannot partly replace a map.
      // Occupancy and its optional TSDF samples are carried by this same message;
      // never combine a current occupancy grid with a separately arriving field.
      int known=0,tsdf_known=0;
      // Both storage backends consume exactly one atomic snapshot. Sparse
      // samples use Voxblox's world-zero lattice rather than the dense origin.
      auto parse=[&](auto& next){
      // Exact full-coordinate membership includes unknown/zero-weight samples,
      // which sparse storage omits. Two contiguous allocations replace millions
      // of individually allocated nodes; release them before old-map cleanup
      // and global revalidation, when this temporary index is no longer needed.
      VoxelIndexSet seen(static_cast<std::size_t>(count));
      trace("parse_begin");
      for(uint32_t row=0;row<message.height;++row)for(uint32_t col=0;col<message.width;++col){
        // The 5M-cell native snapshots take seconds to import. Stop promptly
        // when ROS shuts down, while normal imports retain every sample.
        if((col&1023u)==0u)shutdown_checkpoint();
        const auto* data=message.data.data()+uint64_t(row)*message.row_step+uint64_t(col)*message.point_step;
        float x,y,z;int8_t status;std::memcpy(&x,data+ox,4);std::memcpy(&y,data+oy,4);std::memcpy(&z,data+oz,4);std::memcpy(&status,data+os,1);
        Eigen::Vector3d p(x,y,z);auto index=next.index(p);
        const auto center=next.center(index);
        for(int k=0;k<3;++k){
          // Centers travel as float32, so far-away sparse samples lose a
          // fraction of a voxel in encoding. Compare to that same encoding.
          const double encoded=map_storage_=="sparse"?double(float(center[k])):center[k];
          if(std::abs(encoded-p[k])>resolution_*1e-4)
            throw std::invalid_argument("Map samples must be aligned voxel centers");
        }
        if(!seen.insert({index.x(),index.y(),index.z()}))
          throw std::invalid_argument("Duplicate voxel");
        if(has_tsdf){
          float distance,weight;std::memcpy(&distance,data+od,4);std::memcpy(&weight,data+ow,4);
          if constexpr(std::is_same_v<std::decay_t<decltype(next)>,SparseVoxelMap>)
            next.setObservation(index,static_cast<VoxelStatus>(status),distance,weight);
          else {
            next.set(index,static_cast<VoxelStatus>(status));next.setTsdf(index,distance,weight);
          }
          if(weight>=1e-6f)++tsdf_known;
        }else next.set(index,static_cast<VoxelStatus>(status));
        if(status!=-1)++known;
      }
      };
      std::shared_ptr<const MapQuery> next;
      if(map_storage_=="sparse"){
        auto sparse=std::make_shared<SparseVoxelMap>(resolution_);
        sparse->reserve(static_cast<std::size_t>(count));parse(*sparse);next=std::move(sparse);
      }else{
        auto dense=std::make_shared<VoxelMap>(resolution_,origin_,dimensions_);parse(*dense);next=std::move(dense);
      }
      trace("parsed");
      shutdown_checkpoint();
      // A diagnostic retained cloud is valid only until the next committed
      // replacement. Subscription callbacks retain their owning message after
      // acceptance; service-injected replacements must never reuse old bytes.
      home_diagnostic_map_.reset();
      // The validation executor may consume this revision immediately after
      // publication. Bind owning bytes before that atomic commit, not in the
      // subscription callback after receiveMap returns. Direct/envelope/topic
      // and service-returned augmentation inputs all supply owning aliases.
      if(validation_diagnostic_directory_.empty())validation_source_cloud_.reset();
      else if(source_cloud&&source_cloud.get()==&message)validation_source_cloud_=std::move(source_cloud);
      else {
        validation_source_cloud_.reset();
        RCLCPP_ERROR(get_logger(),"Validation diagnostic source has no owning message");
      }
      observed_map_=std::move(next);refreshPlanningMap();known_cells_=known;++map_revision_;
      tsdf_cells_=tsdf_known;tsdf_fields_=has_tsdf;
      if(!metadata||map_sequence_.hasNewObservation(*metadata))observation_revision_=map_revision_;
      else if(metadata->source!=map_sequence_.current().source||metadata->epoch!=map_sequence_.current().epoch)
        observation_revision_=0;  // A replacement map must not inherit old sensor provenance.
      last_map_stamp_=observation_stamp?*observation_stamp:message.header.stamp;
      if(metadata)map_sequence_.commit(*metadata);
      // Include empty/reset snapshots. All wrappers and their policy values
      // are complete before the single map/revision object becomes visible.
      publishValidationMap();
      if(known_cells_==0){
        clearPlanningDisplay();
        // Clear durable display outputs: a late RViz subscriber must not see
        // an old route or local graph after the mapping session is reset.
        nav_msgs::msg::Path empty_path;empty_path.header.frame_id=frame_;empty_path.header.stamp=now();
        path_pub_->publish(empty_path);global_path_pub_->publish(empty_path);search_path_pub_->publish(empty_path);
        waypoint_path_pub_->publish(empty_path);gate_path_pub_->publish(empty_path);
        planner_msgs::msg::Graph empty_graph;empty_graph.header=empty_path.header;
        graph_pub_->publish(empty_graph);search_graph_pub_->publish(empty_graph);
      }
      if(global_){
        const auto home_before=global_->graph().edge_map_.find(0);
        const bool home_connected=home_before!=global_->graph().edge_map_.end()&&!home_before->second.empty();
        trace("global_update_begin");
        if(known_cells_==0&&!restored_graph_waiting_for_map_){global_->reset();global_repositioning_.clear();frontier_switch_=FrontierSwitch(frontier_switch_.enabled());local_triggered_=false;background_updates_=0;background_total_added_=0;background_last_={};}
        std::function<void(const char*)> update_progress;
        if(map_trace_.is_open())update_progress=trace;
        global_->updateMap(map_,TerrainQueryReuse::ImmutableSnapshot,update_progress,shutdown_checkpoint);
        trace("global_update_end");
        shutdown_checkpoint();
        const auto home_after=global_->graph().edge_map_.find(0);
        if(home_connected&&(home_after==global_->graph().edge_map_.end()||home_after->second.empty()))
          captureHomeSnapshot(message,"home_disconnected");
        if(known_cells_>0)restored_graph_waiting_for_map_=false;
        if(odom_valid_)recordGlobalOdometry();
        shutdown_checkpoint();
        trace("global_publish_begin");
        publishGlobalGraph();
        trace("global_publish_end");
      }
      // Acknowledge only snapshots that passed full schema/grid validation.
      // Controllers must not treat raw/rejected messages as fresh observations.
      shutdown_checkpoint();
      publishMapUpdate(metadata?metadata->kind:"observation");
      trace("ack_published");
      shutdown_checkpoint();
      accepted_map_pub_->publish(message);
      trace("accepted_published");
      RCLCPP_INFO(get_logger(),"Map revision %d: %d known cells",map_revision_,known_cells_);
      trace("completed");
      return true;
    }catch(const MapShutdown&){
      // A global recheck may already have begun: its object is discarded by
      // node teardown. Never report a rejected input or publish a partial ack.
      trace("shutdown_interrupted");return false;
    }catch(const std::exception& error){
      ++rejected_maps_;last_rejected_map_reason_=error.what();trace("rejected");
      RCLCPP_WARN(get_logger(),"Map rejected: %s",error.what());return false;
    }
  }
  void search(const planner_msgs::srv::PlannerSearch::Request& request,
              planner_msgs::srv::PlannerSearch::Response& response){
    publishRefinement({},"clear",false);
    SearchResult result;StateVec source,target;
    planner_msgs::msg::Graph graph;graph.header.frame_id=frame_;graph.header.stamp=now();
    if(request.header.frame_id!=frame_)result.reason="wrong_frame";
    else if(!RobotBounds::validMode(request.bound_mode))result.reason="unsupported_bound_mode";
    else if(!map_)result.reason="map_missing";
    else if(request.use_current_state&&!odom_valid_)result.reason="odometry_missing_or_invalid";
    else if(!poseState(request.target,target))result.reason="invalid_target";
    else if(!request.use_current_state&&!poseState(request.source,source))result.reason="invalid_source";
    else try{
      if(request.use_current_state)source=root_;
      augmentation_bound_mode_=request.bound_mode;
      SearchPlanner planner(map_,parametersForPlanning(search_p_,request.bound_mode,false),search_options_);
      result=planner.search(source,target);planner.graphMessage(graph);
      graph.header.frame_id=frame_;graph.header.stamp=now();
    }catch(const std::exception& error){result.reason="search_error";RCLCPP_ERROR(get_logger(),"%s",error.what());}
    // A search is a query; hypothetical poses must not change home, visited
    // history, global frontiers, or the currently selected exploration route.
    response.success=result.success;nav_msgs::msg::Path path;path.header=graph.header;
    if(result.success)for(const auto& planning:result.path){
      const auto state=baseState(planning);geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
      pose.pose.position.x=state.x();pose.pose.position.y=state.y();pose.pose.position.z=state.z();
      pose.pose.orientation.z=std::sin(state[3]/2);pose.pose.orientation.w=std::cos(state[3]/2);
      path.poses.push_back(pose);response.path.push_back(pose.pose);
    }
    search_path_pub_->publish(path);search_graph_pub_->publish(graph);
    publishRefinement(result.refinement,"search",result.success);
    std::ostringstream out;out<<std::setprecision(12)<<"{\"success\":"<<(result.success?"true":"false")
      <<",\"reason\":\""<<result.reason<<"\",\"exact_target\":"<<(result.exact_target?"true":"false")
      <<",\"endpoint_distance\":"<<result.endpoint_distance<<",\"path_length_m\":"<<result.path_length
      <<",\"vertices\":"<<result.vertices<<",\"edges\":"<<result.edges<<",\"attempts\":"<<result.attempts
      <<",\"sampling_rounds\":"<<result.sampling_rounds
      <<",\"empty_graph_cutoff_reached\":"<<(result.empty_graph_cutoff_reached?"true":"false")
      <<",\"collision_rejections\":"<<result.collision_rejections<<",\"target_candidates\":"<<result.target_candidates
      <<",\"target_id\":"<<result.target_id<<",\"build_ms\":"<<result.build_ms
      <<",\"map_revision\":"<<map_revision_<<",\"seed\":"<<search_p_.seed<<",\"bound_mode\":"<<request.bound_mode
      <<",\"refinement\":"<<refinementStatus(result.refinement)<<'}';
    std_msgs::msg::String metrics;metrics.data=out.str();search_metrics_pub_->publish(metrics);
  }
  // Upstream interpolates only the final returned geometry, after traversal
  // limits, refinement and reference insertion. Keep graph/gain inputs intact.
  // The helper retains every terrain knot and the exact endpoint and start yaw.
  template<class Result>
  void interpolateResultOutput(Result& result){
    if(!result.success)return;
    try{result.path=interpolateOutputPath(result.path,p_.refinement.interpolation_distance);}
    catch(const std::exception& error){
      result.success=false;result.path.clear();result.reason="output_interpolation_failed";
      RCLCPP_WARN(get_logger(),"Output interpolation failed: %s",error.what());
    }
  }
  void waypoint(const planner_msgs::srv::PlannerGoToWaypoint::Request& request,
                planner_msgs::srv::PlannerGoToWaypoint::Response& response){
    publishRefinement({},"clear",false);
    GlobalResult result;StateVec target;
    if(request.waypoint.header.frame_id!=frame_)result.reason="wrong_frame";
    else if(!global_)result.reason="global_disabled";
    else if(!map_)result.reason="map_missing";
    else if(!odom_valid_)result.reason="odometry_missing_or_invalid";
    else if(!poseState(request.waypoint.pose,target))result.reason="invalid_waypoint_pose";
    else if(!RobotBounds::validMode(request.bound_mode))result.reason="unsupported_bound_mode";
    else try{
      // Upstream ignores check_collision. Always validate motion, including
      // the attachment from the current position to the persistent graph.
      selectGlobalBoundMode(request.bound_mode);
      result=global_->routeToWaypoint(root_,target,15.,captureRefinement());publishGlobalGraph();
    }catch(const std::exception& error){result.reason="waypoint_error";RCLCPP_ERROR(get_logger(),"%s",error.what());}
    interpolateResultOutput(result);
    nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=now();
    if(result.success)for(const auto& planning:result.path){
      const auto state=baseState(planning);geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
      pose.pose.position.x=state.x();pose.pose.position.y=state.y();pose.pose.position.z=state.z();
      pose.pose.orientation.z=std::sin(state[3]/2);pose.pose.orientation.w=std::cos(state[3]/2);
      response.path.push_back(pose.pose);path.poses.push_back(pose);
    }
    waypoint_path_pub_->publish(path);
    publishRefinement(result.refinement,"waypoint",result.success);
    const double remaining=result.success?(result.path.back()-target).head<3>().norm():0.;
    std::ostringstream out;out<<std::setprecision(12)<<"{\"success\":"<<(result.success?"true":"false")
      <<",\"reason\":\""<<result.reason<<"\",\"target_id\":"<<result.target
      <<",\"endpoint_distance\":"<<remaining<<",\"path_length_m\":"<<result.distance
      <<",\"map_revision\":"<<map_revision_<<",\"refinement\":"<<refinementStatus(result.refinement)<<'}';
    std_msgs::msg::String metrics;metrics.data=out.str();waypoint_metrics_pub_->publish(metrics);
  }
  SamplingSeed beginLocalSampling(Parameters& configured,const char* caller) {
    // Only actual local graph builds consume a seed. Gain-only evaluators and
    // requests that return a retained global route leave this sequence alone.
    const auto sampling=local_sampling_seeds_->next();
    configured.seed=sampling.effective_seed;
    std::ostringstream out;
    out<<"{\"mode\":\""<<sampling_seed_mode_<<"\",\"base_seed\":"<<p_.seed
       <<",\"effective_seed\":"<<sampling.effective_seed<<",\"request_index\":"<<sampling.request_index
       <<",\"caller\":\""<<caller<<"\",\"map_revision\":"<<map_revision_<<'}';
    std_msgs::msg::String status;status.data=out.str();sampling_status_pub_->publish(status);
    return sampling;
  }
  void plan(const planner_msgs::srv::PlannerSrv::Request& request,planner_msgs::srv::PlannerSrv::Response& response,
            PlanningServiceTiming& timing) {
    publishPlanningPaths({});
    traceMap("local_paths_cleared",last_map_stamp_,0);
    publishGainGeometry({},"clear");
    traceMap("local_gain_cleared",last_map_stamp_,0);
    publishGroundEdges({},"clear");
    traceMap("local_ground_edges_cleared",last_map_stamp_,0);
    publishFrontierClustering({}, {}, "clear",false);
    traceMap("local_clustering_cleared",last_map_stamp_,0);
    publishRefinement({},"clear",false);
    traceMap("local_refinement_cleared",last_map_stamp_,0);
    const auto& requested_root=request.root_pose;
    const bool use_odometry=requested_root.position.x==0&&requested_root.position.y==0&&requested_root.position.z==0&&
      requested_root.orientation.x==0&&requested_root.orientation.y==0&&requested_root.orientation.z==0&&requested_root.orientation.w==0;
    PlanResult result;bool budget_home=false,repositioning=false;
    HomingPathDiagnostics homing_diagnostic;
    std::optional<GlobalResult> budget_home_path;
    bool refresh_local_output_budget=false;
    struct LocalBudgetCheck {
      const char* stage;
      MissionObservation observed;
      const char* decision;
      std::optional<double> home_distance;
    };
    std::vector<LocalBudgetCheck> local_budget_checks;
    // Only the real odometry-root request calls this guard. Reuse its checked
    // home route, with the original distance and 20 s threshold; a later guard
    // must not repeat homing or refinement merely to obtain a fresh clock.
    const auto refreshLocalBudget=[&](const char* stage){
      const auto observed=missionObservation();
      const auto& budget=observed.budget;
      const char* decision="continue";
      bool proceed=true;
      if(observed.landing_due){
        result.success=false;result.path.clear();result.path_length=0.;
        result.reason="landing_requested";budget_home=false;decision="reject";proceed=false;
      }else if(!budget.valid||!budget.clock_valid||budget.remaining<=0){
        result.success=false;result.path.clear();result.path_length=0.;
        result.reason=budget.reason;budget_home=true;decision="reject";proceed=false;
      }else if(mission_auto_homing_&&budget_home_path&&budget_home_path->success&&
          MissionBudget::shouldReturn(budget.remaining,budget_home_path->distance,mission_speed_)){
        const auto& home=*budget_home_path;
        result.success=true;result.path=home.path;result.path_length=home.distance;
        result.refinement=home.refinement;result.reason="mission_time_homing";
        budget_home=true;decision="homing";proceed=false;
      }
      local_budget_checks.push_back({stage,observed,decision,
        budget_home_path&&budget_home_path->success?
          std::optional<double>(budget_home_path->distance):std::nullopt});
      return proceed;
    };
    std::optional<SamplingSeed> local_sampling;
    ReferencePathDiagnostics reference_diagnostics;
    std::vector<StateVec> refused_reference_path;
    auto switching=frontier_switch_.observeLocal(false,false);
    std::string switch_reason="not_evaluated";
    response.status=-1;response.planning_bound_mode=request.bound_mode;
    StateVec root=root_;
    if(request.header.frame_id!=frame_)result.reason="wrong_frame";
    else if(missionObservation().landing_due)result.reason="landing_requested";
    else if(!RobotBounds::validMode(request.bound_mode))result.reason="unsupported_bound_mode";
    else if(!map_)result.reason="map_missing";
    else {
      const auto& pose=request.root_pose;
      if(use_odometry && !odom_valid_)result.reason="odometry_missing_or_invalid";
      else if(use_odometry&&global_&&!auto_home_&&global_->graph().vertices_map_.empty())result.reason="home_not_set";
      else if(!use_odometry && !poseState(pose,root))result.reason="invalid_root";
      else try {
        auto configured=parametersForPlanning(p_,request.bound_mode);
        if(!augmentPlanningRoot(root,configured,request.bound_mode))
          throw std::runtime_error(augmentation_last_reason_);
        if(global_&&use_odometry)selectGlobalBoundMode(request.bound_mode);
        bool run_local=true;
        if(mission_enabled_&&use_odometry){
          run_local=refreshLocalBudget("entry");
          if(run_local&&mission_auto_homing_){
            budget_home_path=global_->homing(root,captureRefinement(),captureHomingPath()?&homing_diagnostic:nullptr);
            // Homing may reconnect and refine the path. Charge that work to
            // ROS mission time before deciding whether local planning starts.
            run_local=refreshLocalBudget("post_home");
            if(run_local&&!budget_home_path->success){
              result.reason="budget_home_unreachable";run_local=false;budget_home=true;
            }
          }
        }
        if(run_local&&use_odometry&&global_&&global_repositioning_.active()){
          const auto retained=inspectRepositioning();
          if(retained.resume_global){
            // buildGraph -> reRunGlobalPlanner: an interrupted automatic
            // trip resumes the SAME vertex until actual odometry is <=5 m
            // away. Do not build a local graph or count another gain round.
            const auto budget=missionObservation().budget;
            GlobalTimeConstraint constraint{budget.remaining,mission_speed_,mission_auto_homing_,
              [this](){return missionObservation().budget;}};
            LocalPlanner evaluator(map_,configured);
            const auto route=global_->selectFrontier(root,[&](const StateVec& state){return evaluator.globalRayGain(state);},
              retained.target,mission_enabled_?&constraint:nullptr,false,captureRefinement(),captureGlobalPaths());
            result.success=route.success;result.path=route.path;result.path_length=route.distance;
            result.refinement=route.refinement;
            result.reason=route.success?"global_repositioning_resumed":"global_repositioning_"+route.reason;
            switch_reason="retained_target";run_local=false;repositioning=true;
            publishGlobal(route,"frontier");
          }else if(retained.state==GlobalRepositioningState::InvalidTarget||
                   retained.state==GlobalRepositioningState::InvalidDistance){
            result.reason="global_repositioning_invalid_target";run_local=false;repositioning=true;
          }
        }
        if(run_local){
          traceMap("local_sampling_begin",last_map_stamp_,0);
          local_sampling=beginLocalSampling(configured,"local");
          timing.sampling=local_sampling;
          // This callback owns one immutable map/policy snapshot until all
          // local and global gain queries finish; no cache crosses requests.
          LocalPlanner planner(map_,configured,planningDiagnosticsOptions(),ExactRayGainReuse::ImmutableSnapshot);
          CaptureRayGainStats cache_stats(planner,timing);
          traceMap("local_core_begin",last_map_stamp_,0);
          result=timing.measure(timing.plan_ms,[&]{
            try{return planner.plan(root,exploration_direction_.heading());}
            catch(...){publishPlanningDiagnostics(planner.diagnostics(),"local");throw;}
          });
          traceMap("local_core_completed",last_map_stamp_,0);
          // getBestPath's budget decision applies to an executable local
          // result; do not turn a failed build/gain/prefix into success.
          refresh_local_output_budget=mission_enabled_&&use_odometry&&result.success;
          timing.phases.emplace(result);
          augmentation_schedule_.recordGraphBuild(result.graph_build_completed);
          publishPlanningDiagnostics(planner.diagnostics(),"local");
          publishSamplingBound(result,"local");
          planner_msgs::msg::Graph graph;planner.graphMessage(graph);graph.header.frame_id=frame_;graph.header.stamp=now();
          graph_pub_->publish(graph);
          traceMap("local_graph_published",last_map_stamp_,0);
          // Explicit hypothetical roots must not alter actual mission history.
          if(global_&&use_odometry){
            switching=frontier_switch_.observeLocal(result.gain_evaluated,result.frontier_exists);
            switch_reason=switching.counted?"not_triggered":"not_evaluated";
            local_triggered_=local_triggered_||result.graph_build_completed;
            recordGlobalOdometry();
            traceMap("local_absorb_begin",last_map_stamp_,0);
            absorbLocalGraph(graph,planner,timing,"local");
            traceMap("local_absorb_completed",last_map_stamp_,0);
            if(switching.trigger_global){
              refresh_local_output_budget=false;
              // evaluateGraph switches even when local free-space gain is
              // positive. A fresh LocalPlanner must not reset this history.
              repositioning=true;GlobalResult global_result;
              const auto budget=missionObservation().budget;
              if(mission_enabled_&&(!budget.valid||budget.remaining<=0))global_result.reason=budget.reason;
              else{
                GlobalTimeConstraint constraint{budget.remaining,mission_speed_,mission_auto_homing_,
                  [this](){return missionObservation().budget;}};
                global_result=global_->selectFrontier(root,[&](const StateVec& state){return planner.globalRayGain(state);},
                    -1,mission_enabled_?&constraint:nullptr,false,captureRefinement(),captureGlobalPaths());
                if(global_result.reason=="no_global_frontier"&&global_->frontierCount()==0){
                  frontier_switch_.onGlobalFrontiers(false,false,go_home_if_fully_explored_);
                  if(go_home_if_fully_explored_&&!landing_enabled_){
                    global_result=global_->homing(root,captureRefinement(),captureHomingPath()?&homing_diagnostic:nullptr);
                    budget_home=true;repositioning=false;
                  }
                }
              }
              switch_reason=global_result.reason;
              if(global_result.success&&!budget_home)global_repositioning_.activate(global_result.target);
              publishGlobal(global_result,budget_home?"homing":"frontier",&homing_diagnostic);
              result.success=global_result.success;result.path=global_result.path;
              result.path_length=global_result.distance;result.refinement=global_result.refinement;
              result.reason=global_result.success?(budget_home?"fully_explored_homing":"local_frontier_repositioning"):
                  "local_frontier_switch_"+global_result.reason;
            }
            // Upstream decides homing after graph/gain work. This ROS 2 guard
            // also includes refinement and global absorption, and uses the
            // already checked home route from this same immutable map request.
            if(refresh_local_output_budget&&!refreshLocalBudget("post_local"))
              refresh_local_output_budget=false;
            // A selected route must survive the separate 3 m visited-frontier
            // exclusion, so a moving endpoint can already find its way home.
            // Repositioning/homing already routes over this global graph;
            // only a new local route needs insertion. This also preserves a
            // successful single-pose homing response when already at home.
            // Reject an expired outbound route before inserting its reference.
            // If time expires during insertion, the final guard below still
            // blocks execution. Stored checked geometry is not an observed
            // visit: addReferencePath leaves odometry, visited and home intact.
            traceMap("local_reference_begin",last_map_stamp_,0);
            if(result.success&&!budget_home&&!switching.trigger_global&&
               !global_->addReferencePath(result.path,nullptr,&reference_diagnostics)){
              refused_reference_path=result.path;
              result.success=false;result.path.clear();result.reason="reference_path_unavailable";
            }
            traceMap("local_reference_completed",last_map_stamp_,0);
            publishGlobalGraph();
          }else switch_reason=use_odometry?"global_disabled":"hypothetical_root";
        }else{
          planner_msgs::msg::Graph empty;empty.header.frame_id=frame_;empty.header.stamp=now();graph_pub_->publish(empty);
        }
      }catch(const std::length_error& e){result.success=false;result.path.clear();result.reason="gain_query_budget_exceeded";RCLCPP_WARN(get_logger(),"%s",e.what());}
      catch(const std::exception& e){result.success=false;result.path.clear();result.reason="planning_error";RCLCPP_ERROR(get_logger(),"%s",e.what());}
    }
    interpolateResultOutput(result);
    if(refresh_local_output_budget&&result.success&&!refreshLocalBudget("final")){
      // A final homing decision replaces the interpolated outbound path with
      // the original cached home geometry. Interpolate that route exactly once.
      if(budget_home&&result.success)interpolateResultOutput(result);
    }
    if(budget_home)global_repositioning_.clear();
    // Real local requests can replace a selected frontier route, including
    // budget-triggered homing or rejection after a global fallback. An
    // explicit hypothetical root leaves the active route display untouched.
    if(use_odometry&&(!repositioning||budget_home||!result.success))
      publishGlobalPaths({},budget_home?"homing":"local",result.success,result.reason,&homing_diagnostic);
    publishRefinement(result.refinement,"local",result.success);
    nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=now();
    if(budget_home)response.status=response.K_HOMING;
    else if(repositioning)response.status=response.K_REPOSITIONING;
    if(result.success){
      response.status=budget_home?response.K_HOMING:repositioning?response.K_REPOSITIONING:response.K_FORWARD;
      for(const auto& planning_state:result.path){
        const auto state=baseState(planning_state);
        geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
        pose.pose.position.x=state[0];pose.pose.position.y=state[1];pose.pose.position.z=state[2];
        pose.pose.orientation.z=std::sin(state[3]/2);pose.pose.orientation.w=std::cos(state[3]/2);
        response.path.push_back(pose.pose);path.poses.push_back(pose);
      }
    }
    // Empty paths clear the last published result after a failed request.
    path_pub_->publish(path);
    std::ostringstream out;out<<std::setprecision(12)<<"{\"success\":"<<(result.success?"true":"false")
      <<",\"reason\":\""<<result.reason<<"\",\"map_revision\":"<<map_revision_<<",\"seed\":"
      <<(local_sampling?local_sampling->effective_seed:p_.seed)
      <<",\"sampling_base_seed\":"<<p_.seed<<",\"sampling_seed_mode\":\""<<sampling_seed_mode_
      <<"\",\"sampling_request_index\":"<<(local_sampling?std::to_string(local_sampling->request_index):"null")
      <<",\"bound_mode\":"<<request.bound_mode
      <<",\"vertices\":"<<result.vertices<<",\"edges\":"<<result.edges<<",\"attempts\":"<<result.attempts
      <<",\"raw_draws\":"<<result.attempts
      <<",\"sampling_rounds\":"<<result.sampling_rounds
      <<",\"sampling_budget_mode\":"<<std::quoted(p_.sampling_budget_mode)
      <<",\"max_attempts\":"<<p_.max_attempts<<",\"max_sampling_rounds\":"<<p_.max_sampling_rounds
      <<",\"sampling_stop_reason\":"<<std::quoted(result.sampling_stop_reason)
      <<",\"empty_graph_cutoff\":"<<p_.empty_graph_cutoff
      <<",\"empty_graph_cutoff_reached\":"<<(result.empty_graph_cutoff_reached?"true":"false")
      <<",\"collision_rejections\":"<<result.collision_rejections<<",\"gain\":"<<result.gain
      <<",\"gain_evaluations\":"<<result.gain_evaluations<<",\"reused_gains\":"<<result.reused_gains
      <<",\"gain_evaluated\":"<<(result.gain_evaluated?"true":"false")
      <<",\"graph_build_completed\":"<<(result.graph_build_completed?"true":"false")
      <<",\"frontier_exists\":"<<(result.frontier_exists?"true":"false")
      <<",\"auto_global\":{\"enabled\":"<<(global_&&frontier_switch_.enabled()?"true":"false")
      <<",\"counted\":"<<(switching.counted?"true":"false")
      <<",\"triggered\":"<<(switching.trigger_global?"true":"false")
      <<",\"count_before\":"<<switching.count_before<<",\"count_after\":"<<frontier_switch_.count()
      <<",\"retained_active\":"<<(global_repositioning_.active()?"true":"false")
      <<",\"retained_target\":"<<global_repositioning_.target()
      <<",\"reason\":\""<<switch_reason<<"\"}"
      <<",\"hanging_vertices\":"<<result.hanging_vertices
      <<",\"candidate_hanging_vertices\":"<<result.candidate_hanging_vertices
      <<",\"candidate_path_length_m\":"<<result.candidate_path_length
      <<",\"best_id\":"<<result.best_id
      <<",\"executed_target_id\":"<<result.executed_target_id
      <<",\"truncated_for_hanging\":"<<(result.truncated_for_hanging?"true":"false")
      <<",\"truncated_for_budget\":"<<(result.truncated_for_budget?"true":"false")
      <<",\"traverse_length_m\":"<<result.traverse_length<<",\"traverse_time_s\":"<<result.traverse_time
      <<",\"gain_before_hanging_penalty\":"<<result.gain_before_hanging_penalty
      <<",\"unpenalized_gain\":"<<result.unpenalized_gain
      <<",\"direction_distance\":";
    // The upstream DTW helper can return infinity for an empty discretized
    // reference. Preserve its scoring semantics, but never emit inf/nan as
    // JSON numbers or replace an unavailable diagnostic with a finite value.
    if(std::isfinite(result.direction_distance))out<<result.direction_distance;
    else out<<"null";
    out<<",\"direction_distance_status\":\""<<(std::isfinite(result.direction_distance)?"finite":"non_finite")<<'"'
      <<",\"direction_factor\":"<<result.direction_factor
      <<",\"exploring_direction\":"<<result.exploring_direction
      <<",\"path_direction_penalty\":"<<p_.path_direction_penalty
      <<",\"ground_projection\":\""<<(p_.ground.interpolate_projection_distance?"tsdf":"occupancy")<<'"'
      <<",\"ray_projection_mode\":\""<<groundRayProjectionModeName(p_.ground.ray_projection_mode)<<'"'
      <<",\"tsdf_projection_mode\":\""<<groundTsdfProjectionModeName(p_.ground.tsdf_projection_mode)<<'"'
      <<",\"tsdf_cells\":"<<tsdf_cells_
      <<",\"gain_model\":\""<<(p_.gain.use_ray_model?"ray":"voxel")<<'"'
      <<",\"gain_ray_method\":\""<<(p_.gain.use_ray_model?p_.gain.ray_method:"none")<<'"'
      <<",\"nonuniform_ray_cast\":"<<(p_.gain.nonuniform_ray_cast?"true":"false")
      <<",\"ray_cast_step_size_multiplier\":"<<p_.gain.ray_cast_step_size_multiplier
      <<",\"exp_gain_voxel_size\":"<<p_.gain.exp_gain_voxel_size
      <<",\"gain_voxel_count\":"<<result.gain_voxel_count<<",\"gain_fov_voxel_count\":"<<result.gain_fov_voxel_count
      <<",\"gain_visibility_queries\":"<<result.gain_visibility_queries
      <<",\"path_length_m\":"<<result.path_length<<",\"build_ms\":"<<result.build_ms
      <<",\"evaluate_ms\":"<<result.evaluate_ms<<",\"gain_ms\":"<<result.gain_ms
      <<",\"refinement_ms\":"<<result.refinement_ms<<",\"adaptive\":"<<adaptiveStatus(result.adaptive)
      <<",\"refinement\":"<<refinementStatus(result.refinement);
    out<<",\"local_budget_checks\":[";
    for(std::size_t i=0;i<local_budget_checks.size();++i){
      if(i)out<<',';
      const auto& check=local_budget_checks[i];
      out<<"{\"stage\":"<<std::quoted(check.stage)<<",\"stamp_s\":"<<check.observed.timestamp
         <<",\"valid\":"<<(check.observed.budget.valid?"true":"false")
         <<",\"clock_valid\":"<<(check.observed.budget.clock_valid?"true":"false")
         <<",\"remaining_s\":"<<check.observed.budget.remaining
         <<",\"landing_due\":"<<(check.observed.landing_due?"true":"false")
         <<",\"decision\":"<<std::quoted(check.decision)<<",\"home_distance_m\":";
      if(check.home_distance)out<<*check.home_distance;else out<<"null";
      out<<'}';
    }
    out<<"],\"reference_path_diagnostics\":{\"reason\":\""<<reference_diagnostics.reason
       <<"\",\"segment\":"<<reference_diagnostics.segment<<",\"coordinates\":\"planning_height\"";
    const auto write_state=[&](const StateVec& state){
      out<<'['<<state[0]<<','<<state[1]<<','<<state[2]<<','<<state[3]<<']';
    };
    out<<",\"from\":";write_state(reference_diagnostics.from);
    out<<",\"to\":";write_state(reference_diagnostics.to);out<<"},\"planning_root\":";write_state(root);
    out<<",\"candidate_path\":[";
    for(std::size_t i=0;i<refused_reference_path.size();++i){
      if(i)out<<',';write_state(refused_reference_path[i]);
    }
    out<<"]}";
    std_msgs::msg::String metrics;metrics.data=out.str();metrics_pub_->publish(metrics);
  }
  void publishAheadMetrics(const planner_msgs::srv::PlanAhead::Request& request,
                           const planner_msgs::srv::PlanAhead::Response& response,
                           const PlanningServiceTiming& timing){
    std::ostringstream out;out<<std::setprecision(17)
      <<"{\"request\":\"ahead\",\"success\":"<<(response.success?"true":"false")
      <<",\"reason\":\""<<response.reason<<"\",\"map_revision\":"<<response.map_revision
      <<",\"map_stamp_sec\":"<<last_map_stamp_.sec<<",\"map_stamp_nanosec\":"<<last_map_stamp_.nanosec
      <<",\"known_cells\":"<<known_cells_<<",\"bound_mode\":"<<request.bound_mode
      <<",\"request_ros_s\":"<<(double(request.header.stamp.sec)+request.header.stamp.nanosec*1.e-9)
      <<",\"response_ros_s\":"<<now().seconds()<<",\"arrival_time_s\":";
    if(std::isfinite(request.arrival_time))out<<request.arrival_time;else out<<"null";
    out<<",\"planning_root\":";StateVec root;
    if(poseState(request.root_pose,root))out<<'['<<root[0]<<','<<root[1]<<','<<root[2]<<','<<root[3]<<']';
    else out<<"null";
    out<<",\"sampling_seed_mode\":\""<<sampling_seed_mode_<<"\",\"sampling_base_seed\":"<<p_.seed
      <<",\"sampling_budget_mode\":"<<std::quoted(p_.sampling_budget_mode)
      <<",\"max_attempts\":"<<p_.max_attempts<<",\"max_sampling_rounds\":"<<p_.max_sampling_rounds
      <<",\"empty_graph_cutoff\":"<<p_.empty_graph_cutoff
      <<",\"effective_seed\":"<<(timing.sampling?std::to_string(timing.sampling->effective_seed):"null")
      <<",\"sampling_request_index\":"<<(timing.sampling?std::to_string(timing.sampling->request_index):"null")
      <<",\"plan_ms\":"<<timing.plan_ms<<",\"absorb_local_ms\":"<<timing.absorb_local_ms
      <<",\"service_ms\":"<<std::chrono::duration<double,std::milli>(PlanningServiceTiming::Clock::now()-timing.started).count()
      <<",\"timing_clock\":\"steady\",\"gain_and_refinement_included_in_evaluate_ms\":true"
      <<",\"ray_cache\":{\"enabled\":"<<(timing.ray_cache_enabled?"true":"false")
      <<",\"hits\":"<<timing.ray_cache.hits<<",\"evaluations\":"<<timing.ray_cache.evaluations
      <<",\"entries\":"<<timing.ray_cache.entries<<"},\"terrain_cache\":{"
      <<"\"projection_hits\":"<<timing.terrain_cache.projection_hits
      <<",\"projection_evaluations\":"<<timing.terrain_cache.projection_evaluations
      <<",\"support_hits\":"<<timing.terrain_cache.support_hits
      <<",\"support_evaluations\":"<<timing.terrain_cache.support_evaluations<<"},\"plan\":";
    if(timing.phases){
      const auto& phase=*timing.phases;
      out<<"{\"success\":"<<(phase.success?"true":"false")<<",\"reason\":\""<<phase.reason
        <<"\",\"graph_build_completed\":"<<(phase.graph_build_completed?"true":"false")
        <<",\"gain_evaluated\":"<<(phase.gain_evaluated?"true":"false")
        <<",\"vertices\":"<<phase.vertices<<",\"edges\":"<<phase.edges<<",\"attempts\":"<<phase.attempts
        <<",\"raw_draws\":"<<phase.attempts
        <<",\"sampling_rounds\":"<<phase.sampling_rounds<<",\"collision_rejections\":"<<phase.collision_rejections
        <<",\"sampling_stop_reason\":"<<std::quoted(phase.sampling_stop_reason)
        <<",\"empty_graph_cutoff_reached\":"<<(phase.empty_graph_cutoff_reached?"true":"false")
        <<",\"gain_evaluations\":"<<phase.gain_evaluations<<",\"reused_gains\":"<<phase.reused_gains
        <<",\"best_id\":"<<phase.best_id<<",\"gain\":"<<phase.gain
        <<",\"candidate_path_length_m\":"<<phase.candidate_path_length
        <<",\"build_ms\":"<<phase.build_ms<<",\"evaluate_ms\":"<<phase.evaluate_ms
        <<",\"gain_ms\":"<<phase.gain_ms<<",\"refinement_ms\":"<<phase.refinement_ms
        <<",\"exploring_direction\":"<<phase.exploring_direction<<",\"path_length_m\":"<<phase.path_length
        <<",\"refinement\":"<<refinementStatus(phase.refinement)<<'}';
    }else out<<"null";
    out<<'}';std_msgs::msg::String metrics;metrics.data=out.str();ahead_metrics_pub_->publish(metrics);
  }
  void planAhead(const planner_msgs::srv::PlanAhead::Request& request,
                 planner_msgs::srv::PlanAhead::Response& response,PlanningServiceTiming& timing){
    publishRefinement({},"clear",false);
    PathRefinementDiagnostics refinement;
    planAheadImpl(request,response,timing,refinement);
    publishRefinement(refinement,"ahead",response.success);
  }
  void planAheadImpl(const planner_msgs::srv::PlanAhead::Request& request,
                 planner_msgs::srv::PlanAhead::Response& response,PlanningServiceTiming& timing,
                 PathRefinementDiagnostics& refinement){
    publishPlanningPaths({});
    publishGainGeometry({},"clear");
    publishGroundEdges({},"clear");
    publishFrontierClustering({}, {}, "clear",false);
    response.success=false;response.homing_required=false;response.global_required=false;response.map_revision=map_revision_;
    const double started=now().seconds();
    StateVec root;
    if(request.header.frame_id!=frame_){response.reason="wrong_frame";return;}
    if(!RobotBounds::validMode(request.bound_mode)){response.reason="unsupported_bound_mode";return;}
    if(!std::isfinite(request.arrival_time)||request.arrival_time<0){response.reason="invalid_arrival_time";return;}
    if(!poseState(request.root_pose,root)){response.reason="invalid_root";return;}
    if(missionObservation().landing_due){response.reason="landing_requested";return;}
    if(!map_||known_cells_==0){response.reason="map_missing";return;}
    if(!odom_valid_){response.reason="odometry_missing_or_invalid";return;}
    if(global_&&global_->graph().vertices_map_.empty()){response.reason="home_not_set";return;}
    try{
      auto configured=parametersForPlanning(p_,request.bound_mode);
      if(!augmentPlanningRoot(root,configured,request.bound_mode)){
        response.reason=augmentation_last_reason_;return;
      }
      response.map_revision=map_revision_;
      selectGlobalBoundMode(request.bound_mode);
      if(map_->box(baseState(root_).head<3>()+p_.center_offset,configured.robot_size)!=VoxelStatus::Free){
        response.reason="current_pose_not_free";return;
      }
      if(map_->box(baseState(root).head<3>()+p_.center_offset,configured.robot_size)!=VoxelStatus::Free){
        response.reason="future_root_not_free";return;
      }
      double home_distance=0.;
      if(mission_enabled_&&mission_auto_homing_){
        // homing() only adds checked graph connections. It does not record
        // odometry, change home or mark this speculative root as visited.
        const auto home=global_->homing(root);
        if(!home.success){response.reason="budget_home_unreachable";response.homing_required=true;return;}
        home_distance=home.distance;
      }
      auto feasible=[&](double local_distance){
        // Refresh after planning too: the same node-lifetime/battery budget
        // must age while an expensive graph is being built. Arrival counts
        // only travel still ahead of that refreshed snapshot. Keeping network
        // transit time in this estimate is intentionally conservative.
        const auto observed=missionObservation();
        const auto& budget=observed.budget;
        if(observed.landing_due){response.reason="landing_requested";response.global_required=false;return false;}
        if(!mission_enabled_)return true;
        if(!budget.valid||budget.remaining<=0){response.reason=budget.reason;response.homing_required=true;response.global_required=false;return false;}
        const double arrival=std::max(0.,request.arrival_time-std::max(0.,observed.timestamp-started));
        double spare=0.;
        if(!MissionBudget::sufficientAhead(budget.remaining,arrival,local_distance,home_distance,
                                          mission_speed_,mission_auto_homing_,spare)){
          response.reason="plan_ahead_time_insufficient";response.homing_required=true;response.global_required=false;return false;
        }
        return true;
      };
      if(!feasible(0.))return;
      // A speculative future root must not release an active global trip.
      // Check only the latest measured pose, and let PCI query the route from
      // its next actual endpoint instead of committing a local continuation.
      const auto retained=inspectRepositioning();
      if(retained.resume_global){
        response.global_required=true;response.reason="global_repositioning_pending";return;
      }
      if(retained.state==GlobalRepositioningState::InvalidTarget||
         retained.state==GlobalRepositioningState::InvalidDistance){
        response.reason="global_repositioning_invalid_target";return;
      }
      timing.sampling=beginLocalSampling(configured,"ahead");
      // Retain only exact gains across this plan -> absorbLocal sequence.
      LocalPlanner planner(map_,configured,planningDiagnosticsOptions(),ExactRayGainReuse::ImmutableSnapshot);
      CaptureRayGainStats cache_stats(planner,timing);
      auto result=timing.measure(timing.plan_ms,[&]{
        try{return planner.plan(root,exploration_direction_.heading());}
        catch(...){publishPlanningDiagnostics(planner.diagnostics(),"ahead");throw;}
      });
      timing.phases.emplace(result);
      refinement=std::move(result.refinement);
      augmentation_schedule_.recordGraphBuild(result.graph_build_completed);
      // A completed build enables background work even if gain evaluation or
      // a later continuation check refuses the path. Pre-build failures do not.
      if(global_&&result.graph_build_completed)local_triggered_=true;
      publishPlanningDiagnostics(planner.diagnostics(),"ahead");
      // The upstream PCI supplies current_path_.back() to the same planner
      // service/evaluateGraph. An evaluated ahead round therefore counts even
      // if NO_GAIN, a short path, or a later continuation refusal prevents
      // execution. Ordinary hypothetical PlannerSrv roots remain excluded.
      if(global_)
        response.global_required=frontier_switch_.observeLocal(result.gain_evaluated,result.frontier_exists).trigger_global;
      publishSamplingBound(result,"ahead");
      if(response.global_required){
        if(!feasible(0.))return;
        planner_msgs::msg::Graph graph;planner.graphMessage(graph);
        graph.header.frame_id=frame_;graph.header.stamp=now();
        // Retain checked frontier candidates, but do not add this discarded
        // local route as an execution reference or record the future root as
        // visited. The PCI will query global planning from its eventual pose.
        absorbLocalGraph(graph,planner,timing,"ahead");
        publishGlobalGraph();graph_pub_->publish(graph);
        if(!feasible(0.))return;
        response.reason="plan_ahead_global_required";return;
      }
      if(!result.success||result.path.size()<2){response.reason=result.reason;return;}
      if(!feasible(result.path_length))return;
      // Candidate vertices are real checked free space, not robot history.
      // In particular do not assign root_ or call recordGlobalOdometry here.
      planner_msgs::msg::Graph graph;planner.graphMessage(graph);
      graph.header.frame_id=frame_;graph.header.stamp=now();
      if(global_){
        absorbLocalGraph(graph,planner,timing,"ahead");
        if(!global_->addReferencePath(result.path)){response.reason="reference_path_unavailable";return;}
        publishGlobalGraph();
      }
      // Frontier absorption may evaluate more gain; include its cost too.
      if(!feasible(result.path_length))return;
      const auto output_path=interpolateOutputPath(result.path,p_.refinement.interpolation_distance);
      nav_msgs::msg::Path path;path.header=graph.header;
      for(const auto& planning_state:output_path){
        const auto state=baseState(planning_state);
        geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
        pose.pose.position.x=state[0];pose.pose.position.y=state[1];pose.pose.position.z=state[2];
        pose.pose.orientation.z=std::sin(state[3]/2);pose.pose.orientation.w=std::cos(state[3]/2);
        response.path.push_back(pose.pose);path.poses.push_back(pose);
      }
      // The follower must recheck this same cost against fresh, aged mission
      // status after asynchronous route validation, immediately before commit.
      // Old-route arrival is excluded: the follower measures its own latest ETA.
      response.budget_enabled=mission_enabled_;
      if(mission_enabled_){
        const double return_distance=mission_auto_homing_?result.path_length+home_distance:0.;
        response.required_time_after_root=(result.path_length+return_distance)/mission_speed_+
                                          MissionBudget::kReserveSeconds;
      }
      response.success=true;response.reason="ok";graph_pub_->publish(graph);path_pub_->publish(path);
    }catch(const std::exception& error){
      response.reason="plan_ahead_error";RCLCPP_ERROR(get_logger(),"%s",error.what());
    }
  }
  Parameters p_;double resolution_;Eigen::Vector3d origin_;Eigen::Vector3i dimensions_;
  std::vector<rclcpp::ServiceBase::SharedPtr> compatibility_services_;
  rclcpp::Subscription<planner_semantic_msgs::msg::SemanticPoint>::SharedPtr semantic_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr semantic_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr semantic_metrics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr semantic_targets_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr semantic_status_service_;
  std::string semantic_reason_{"no_request"};bool semantic_success_{false};
  int semantic_received_{0},semantic_accepted_{0},semantic_target_{-1},semantic_attempts_{0};double semantic_length_{0.};
  Parameters search_p_;SearchParameters search_options_;
  RobotBounds robot_bounds_;int global_bound_mode_{RobotBounds::Exact};
  bool restored_graph_waiting_for_map_{false};int planning_trigger_mode_{0};
  GlobalPersistenceResult last_graph_storage_;std::string last_graph_operation_{"none"};
  std::vector<rclcpp::Service<planner_msgs::srv::PlannerStringTrigger>::SharedPtr> graph_storage_services_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr graph_storage_status_;
  rclcpp::Service<planner_msgs::srv::PlannerSetPlanningMode>::SharedPtr planning_mode_service_;
  GateParameters gate_;bool gate_use_tf_{false};
  std::string gate_frame_;Eigen::Vector3d gate_offset_{Eigen::Vector3d::Zero()};
  std::vector<rclcpp::Service<planner_msgs::srv::PlannerRequestPath>::SharedPtr> gate_services_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gate_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gate_metrics_pub_;
  rclcpp::Service<planner_msgs::srv::PlannerSearch>::SharedPtr search_service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr search_path_pub_;
  rclcpp::Publisher<planner_msgs::msg::Graph>::SharedPtr search_graph_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr search_metrics_pub_;
  std::vector<rclcpp::Service<planner_msgs::srv::PlannerGoToWaypoint>::SharedPtr> waypoint_services_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoint_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_metrics_pub_;
  void publishSensorFrusta(){
    visualization_msgs::msg::MarkerArray markers;
    for(std::size_t i=0;i<p_.sensors.size();++i){
      const auto edges=sensorFrustum(p_.sensors[i],root_);if(edges.empty())continue;
      visualization_msgs::msg::Marker marker;marker.header.frame_id=frame_;marker.header.stamp=now();
      marker.ns="sensor_frustum";marker.id=static_cast<int>(i);marker.type=visualization_msgs::msg::Marker::LINE_LIST;
      marker.action=visualization_msgs::msg::Marker::ADD;marker.pose.orientation.w=1.;
      marker.scale.x=.015;marker.color.r=.9;marker.color.g=.65;marker.color.b=.2;marker.color.a=.7;
      for(const auto& edge:edges)for(const auto& point:{edge.first,edge.second}){
        geometry_msgs::msg::Point position;position.x=point.x();position.y=point.y();position.z=point.z();marker.points.push_back(position);
      }
      markers.markers.push_back(std::move(marker));
    }
    sensor_frusta_pub_->publish(markers);
  }
  std::string adaptiveStatus(const AdaptiveBoundResult& bound)const{
    std::ostringstream out;out<<std::setprecision(12)
      <<"{\"enabled\":"<<(p_.adaptive.enabled?"true":"false")
      <<",\"applied\":"<<(bound.applied?"true":"false")<<",\"reason\":\""<<bound.reason<<'"'
      <<",\"raw_points\":"<<bound.raw_points<<",\"filtered_points\":"<<bound.filtered_points;
    auto vector_field=[&](const char* name,const Eigen::Vector3d& value){
      out<<",\""<<name<<"\":["<<value.x()<<','<<value.y()<<','<<value.z()<<']';
    };
    vector_field("minimum",bound.min);vector_field("maximum",bound.max);
    vector_field("rotations",bound.rotations);vector_field("mean",bound.mean);
    vector_field("scale",bound.scale);vector_field("centroid",bound.centroid);
    vector_field("eigenvalues",bound.eigenvalues);
    out<<",\"rotation\":[";
    for(int row=0;row<3;++row)for(int col=0;col<3;++col){if(row||col)out<<',';out<<bound.rotation(row,col);}
    out<<"]}";return out.str();
  }
  void publishFrontierClustering(const FrontierClusteringDiagnostics& diagnostic,
      const FrontierUpdate& admission,const char* request,bool captured){
    std_msgs::msg::Header header;header.frame_id=frame_;header.stamp=now();
    frontier_clusters_pub_->publish(frontierClusterMarkers(diagnostic,header));
    // Clusters describe the frontier candidates BEFORE visited/existing-graph
    // exclusion. Publish admission separately; a visible path need not be added.
    std::ostringstream out;out<<std::setprecision(17)<<"{\"request\":"<<std::quoted(request)
      <<",\"map_revision\":"<<map_revision_<<",\"stamp_ns\":"<<rclcpp::Time(header.stamp).nanoseconds()
      <<",\"captured\":"<<(captured?"true":"false")
      <<",\"evaluated\":"<<(diagnostic.evaluated?"true":"false")
      <<",\"candidate_count\":"<<diagnostic.candidate_count<<",\"representative_ids\":[";
    for(std::size_t i=0;i<diagnostic.representative_ids.size();++i){if(i)out<<',';out<<diagnostic.representative_ids[i];}
    out<<"],\"candidates\":[";
    for(std::size_t i=0;i<diagnostic.candidates.size();++i){
      if(i)out<<',';const auto& candidate=diagnostic.candidates[i];
      out<<"{\"vertex_id\":"<<candidate.vertex_id<<",\"cluster_id\":"<<candidate.cluster_id<<",\"path\":[";
      for(std::size_t j=0;j<candidate.path.size();++j){
        if(j)out<<',';const auto& state=candidate.path[j];
        out<<'['<<state.x()<<','<<state.y()<<','<<state.z()<<','<<state[3]<<']';
      }
      out<<"]}";
    }
    out<<"],\"admission\":{\"candidates\":"<<admission.candidates<<",\"clusters\":"<<admission.clusters
      <<",\"near_existing\":"<<admission.near_existing<<",\"near_visited\":"<<admission.near_visited
      <<",\"invalid_paths\":"<<admission.invalid_paths<<",\"added\":"<<admission.added<<"}}";
    std_msgs::msg::String message;message.data=out.str();frontier_clustering_status_pub_->publish(message);
  }
  void absorbLocalGraph(const planner_msgs::msg::Graph& graph,const LocalPlanner& planner,
                        PlanningServiceTiming& timing,const char* request){
    const auto interested=[&](const auto& publisher){return publisher->get_subscription_count()>0||
      publisher->get_intra_process_subscription_count()>0;};
    const bool capture=planning_diagnostics_enabled_||interested(frontier_clusters_pub_)||interested(frontier_clustering_status_pub_);
    FrontierClusteringDiagnostics diagnostic;
    std::function<void(const char*)> progress;
    if(map_trace_.is_open())progress=[this](const char* phase){traceMap(phase,last_map_stamp_,0);};
    timing.measure(timing.absorb_local_ms,[&]{
      global_->absorbLocal(graph,[&](const StateVec& state){return planner.globalRayGain(state);},
                           capture?&diagnostic:nullptr,progress,&planner);
    });
    publishFrontierClustering(diagnostic,global_->lastFrontierUpdate(),request,capture);
  }
  PlanningDiagnosticsOptions planningDiagnosticsOptions()const{
    const auto interested=[&](const auto& publisher){
      return planning_diagnostics_enabled_||publisher->get_subscription_count()>0||
        publisher->get_intra_process_subscription_count()>0;
    };
    return {interested(sampler_pub_),interested(projected_graph_pub_),interested(failed_edges_pub_),
      interested(shortest_paths_pub_)||interested(best_paths_pub_),
      interested(gain_rays_pub_)||interested(gain_samples_pub_)||interested(gain_geometry_pub_),gain_vertex_id_,
      interested(ground_edge_markers_pub_)||interested(ground_edge_geometry_pub_)};
  }
  void publishGroundEdges(const GroundEdgeDiagnostics& geometry,const std::string& request){
    std_msgs::msg::Header header;header.frame_id=frame_;header.stamp=now();
    ground_edge_markers_pub_->publish(groundEdgeMarkers(geometry,header));
    std::ostringstream out;out<<"{\"request\":"<<std::quoted(request)
      <<",\"map_revision\":"<<map_revision_<<",\"stamp_ns\":"<<rclcpp::Time(header.stamp).nanoseconds()
      <<",\"geometry\":"<<groundEdgeDiagnosticsJson(geometry)<<'}';
    std_msgs::msg::String message;message.data=out.str();ground_edge_geometry_pub_->publish(message);
  }
  void publishGainGeometry(const GainGeometry& geometry,const std::string& request){
    std_msgs::msg::Header header;header.frame_id=frame_;header.stamp=now();
    gain_rays_pub_->publish(gainRayMarkers(geometry,header));
    gain_samples_pub_->publish(gainSampleMarkers(geometry,header));
    std::ostringstream out;out<<"{\"request\":"<<std::quoted(request)
      <<",\"map_revision\":"<<map_revision_<<",\"stamp_ns\":"<<rclcpp::Time(header.stamp).nanoseconds()
      <<",\"geometry\":"<<gainGeometryJson(geometry)<<'}';
    std_msgs::msg::String message;message.data=out.str();gain_geometry_pub_->publish(message);
  }
  void publishPlanningPaths(const PlanningDiagnostics& diagnostics){
    std_msgs::msg::Header header;header.frame_id=frame_;header.stamp=now();
    shortest_paths_pub_->publish(shortestPathMarkers(diagnostics,header));
    best_paths_pub_->publish(bestPathMarkers(diagnostics,header));
  }
  void clearPlanningDisplay(){
    publishRefinement({},"clear",false);
    publishFrontierClustering({}, {}, "clear",false);
    publishGlobalPaths({},"clear",false,"planning_display_cleared");
    PlanningDiagnostics empty;empty.options={true,true,true,true};
    publishPlanningDiagnostics(empty);
    visualization_msgs::msg::Marker bound;bound.header.frame_id=frame_;bound.header.stamp=now();
    bound.ns="sampling_bound";bound.id=0;bound.action=visualization_msgs::msg::Marker::DELETE;
    sampling_marker_pub_->publish(bound);
  }
  void publishPlanningDiagnostics(const PlanningDiagnostics& diagnostics,const std::string& request="clear"){
    publishGroundEdges(diagnostics.ground_edges,request);
    publishGainGeometry(diagnostics.gains,request);
    publishPlanningPaths(diagnostics);
    using Marker=visualization_msgs::msg::Marker;
    using MarkerArray=visualization_msgs::msg::MarkerArray;
    const auto stamp=now();
    const auto marker=[&](const char* name,int type,double size,double red,double green,double blue){
      Marker item;item.header.frame_id=frame_;item.header.stamp=stamp;
      item.ns=name;item.id=0;item.type=type;item.action=Marker::ADD;item.pose.orientation.w=1.;
      item.scale.x=size;item.scale.y=size;item.scale.z=size;
      item.color.r=red;item.color.g=green;item.color.b=blue;item.color.a=1.;return item;
    };
    const auto point=[](const Eigen::Vector3d& value){
      geometry_msgs::msg::Point out;out.x=value.x();out.y=value.y();out.z=value.z();return out;
    };
    const auto snapshot=[&]{
      MarkerArray out;Marker clear;clear.header.frame_id=frame_;clear.header.stamp=stamp;
      clear.action=Marker::DELETEALL;out.markers.push_back(clear);return out;
    };
    // Each topic is a complete snapshot, including empty plans. The leading
    // DELETEALL also makes a transient-local sample safe for late RViz clients.
    if(diagnostics.options.samples){
      auto out=snapshot();
      auto valid=marker("valid_samples",Marker::SPHERE_LIST,.2,0.,1.,0.);
      auto invalid=marker("invalid_samples",Marker::SPHERE_LIST,.2,244./255.,229./255.,66./255.);
      valid.points.reserve(diagnostics.valid_samples.size());invalid.points.reserve(diagnostics.invalid_samples.size());
      for(const auto& value:diagnostics.valid_samples)valid.points.push_back(point(value));
      for(const auto& value:diagnostics.invalid_samples)invalid.points.push_back(point(value));
      out.markers.push_back(std::move(valid));out.markers.push_back(std::move(invalid));sampler_pub_->publish(out);
    }
    if(diagnostics.options.projected_graph){
      auto out=snapshot();
      auto edges=marker("edges",Marker::LINE_LIST,.04,200./255.,100./255.,0.);
      auto vertices=marker("vertices",Marker::SPHERE_LIST,.3,125./255.,42./255.,104./255.);
      edges.points.reserve(2*diagnostics.projected_segments.size());vertices.points.reserve(diagnostics.projected_vertices.size());
      for(const auto& segment:diagnostics.projected_segments){
        edges.points.push_back(point(segment[0]));edges.points.push_back(point(segment[1]));
      }
      for(const auto& value:diagnostics.projected_vertices)vertices.points.push_back(point(value));
      out.markers.push_back(std::move(edges));out.markers.push_back(std::move(vertices));projected_graph_pub_->publish(out);
    }
    if(diagnostics.options.failed_edges){
      auto out=snapshot();auto edges=marker("failed_edges",Marker::LINE_LIST,.1,200./255.,0.,0.);
      edges.points.reserve(2*diagnostics.failed_edges.size());
      for(const auto& segment:diagnostics.failed_edges){
        edges.points.push_back(point(segment[0]));edges.points.push_back(point(segment[1]));
      }
      out.markers.push_back(std::move(edges));failed_edges_pub_->publish(out);
    }
    if(diagnostics.dropped_samples||diagnostics.dropped_failed_edges||diagnostics.dropped_projected_edges)
      RCLCPP_DEBUG(get_logger(),"Planning visualization capped: dropped %zu samples, %zu failed edges, %zu projected edges",
        diagnostics.dropped_samples,diagnostics.dropped_failed_edges,diagnostics.dropped_projected_edges);
  }
  void publishSamplingBound(const PlanResult& result,const char* request_kind){
    std_msgs::msg::String diagnostics;
    diagnostics.data="{\"request\":\""+std::string(request_kind)+"\",\"map_revision\":"+
      std::to_string(map_revision_)+",\"adaptive\":"+adaptiveStatus(result.adaptive)+"}";
    adaptive_metrics_pub_->publish(diagnostics);
    const auto& bound=result.adaptive;
    visualization_msgs::msg::Marker marker;marker.header.frame_id=frame_;marker.header.stamp=now();
    marker.ns="sampling_bound";marker.id=0;marker.type=visualization_msgs::msg::Marker::CUBE;
    if(!bound.min.allFinite()||!bound.max.allFinite()||(bound.min.array()>=bound.max.array()).any()){
      marker.action=visualization_msgs::msg::Marker::DELETE;sampling_marker_pub_->publish(marker);return;
    }
    marker.action=visualization_msgs::msg::Marker::ADD;
    const Eigen::Vector3d center=result.sampling_root.head<3>()+bound.rotation.transpose()*(.5*(bound.min+bound.max));
    const Eigen::Vector3d size=bound.max-bound.min;
    const Eigen::Quaterniond rotation(bound.rotation.transpose());
    marker.pose.position.x=center.x();marker.pose.position.y=center.y();marker.pose.position.z=center.z();
    marker.pose.orientation.x=rotation.x();marker.pose.orientation.y=rotation.y();marker.pose.orientation.z=rotation.z();marker.pose.orientation.w=rotation.w();
    marker.scale.x=size.x();marker.scale.y=size.y();marker.scale.z=size.z();
    marker.color.r=.7;marker.color.g=.3;marker.color.b=.9;marker.color.a=.1;
    sampling_marker_pub_->publish(marker);
  }
  bool expandBackground(){
    if(!global_||!map_||!odom_valid_||!local_triggered_)return false;
    traceMap("background_compute_begin",last_map_stamp_,0);
    LocalPlanner evaluator(map_,p_);
    // Upstream background insertion uses ordinary ray gain, unlike the
    // NoBound policy used when rechecking established global frontiers.
    const auto gain=[&](const StateVec& state){return evaluator.rayGain(state);};
    background_last_=background_budget_mode_=="time"
      ?global_->expandBackgroundTimed(background_time_budget_,gain)
      :global_->expandBackground(background_attempts_,gain);
    traceMap("background_compute_completed",last_map_stamp_,0);
    ++background_updates_;background_total_added_+=background_last_.added;publishGlobalGraph();
    traceMap("background_completed",last_map_stamp_,0);
    return true;
  }
  std::string backgroundStatus()const{
    std::ostringstream out;out<<"{\"timer_enabled\":"<<(background_enabled_?"true":"false")
      <<",\"budget_mode\":"<<std::quoted(background_budget_mode_)
      <<",\"time_budget_seconds\":"<<background_time_budget_
      <<",\"timer_clock\":\"ros\",\"timer_period_seconds\":"<<background_period_
      <<",\"compute_budget_clock\":\"steady\",\"trigger_policy\":\"completed_live_or_ahead_graph\""
      <<",\"timer_ticks\":"<<background_timer_ticks_<<",\"last_timer_ros_ns\":"<<background_last_tick_ns_
      <<",\"local_triggered\":"<<(local_triggered_?"true":"false")<<",\"updates\":"<<background_updates_
      <<",\"total_added\":"<<background_total_added_<<",\"clusters\":"<<background_last_.clusters
      <<",\"attempts\":"<<background_last_.attempts<<",\"added\":"<<background_last_.added
      <<",\"frontiers_added\":"<<background_last_.frontiers_added<<",\"near_visited\":"<<background_last_.near_visited
      <<",\"near_graph\":"<<background_last_.near_graph<<",\"outside_or_occupied\":"<<background_last_.outside_or_occupied
      <<",\"near_frontier\":"<<background_last_.near_frontier
      <<",\"collision_rejections\":"<<background_last_.collision_rejections
      <<",\"rounds\":"<<background_last_.rounds<<",\"sampling_batches\":"<<background_last_.sampling_batches
      <<",\"accepted_samples\":"<<background_last_.accepted_samples
      <<",\"hanging_rejections\":"<<background_last_.hanging_rejections
      <<",\"expansion_attempts\":"<<background_last_.expansion_attempts
      <<",\"elapsed_seconds\":"<<background_last_.elapsed_seconds
      <<",\"time_budget_reached\":"<<(background_last_.time_budget_reached?"true":"false")
      <<",\"stop_reason\":"<<std::quoted(background_last_.stop_reason)<<"}";return out.str();
  }
  void publishSemanticTargets(){
    using Marker=visualization_msgs::msg::Marker;
    using Class=planner_semantic_msgs::msg::SemanticClass;
    visualization_msgs::msg::MarkerArray snapshot;
    Marker clear;clear.header.frame_id=frame_;
    clear.action=Marker::DELETEALL;snapshot.markers.push_back(clear);
    Marker stairs;stairs.header=clear.header;stairs.ns="stairs";stairs.id=0;
    stairs.action=Marker::ADD;stairs.type=Marker::CUBE_LIST;stairs.pose.orientation.w=1.;
    stairs.scale.x=.7;stairs.scale.y=.7;stairs.scale.z=.7;
    stairs.color.r=1.;stairs.color.g=1.;stairs.color.a=1.;
    Marker door=stairs;door.ns="door";door.color.g=0.;door.color.b=1.;
    // Use the stored classes and sparse IDs directly. Graph coordinates keep
    // planning height; RViz receives the same base height as semantic_state.
    if(global_)for(const auto& item:global_->graph().vertices_map_){
      const auto* vertex=item.second;const auto semantic_class=vertex->semantic_class.value;
      if(semantic_class!=Class::K_STAIRCASE&&semantic_class!=Class::K_DOOR)continue;
      const auto state=baseState(vertex->state);geometry_msgs::msg::Point point;
      point.x=state.x();point.y=state.y();point.z=state.z();
      (semantic_class==Class::K_STAIRCASE?stairs.points:door.points).push_back(point);
    }
    // Each durable message is complete, so reset/load and late subscribers
    // cannot retain a class or target that disappeared from the global graph.
    snapshot.markers.push_back(std::move(stairs));snapshot.markers.push_back(std::move(door));
    if(semantic_targets_snapshot_&&snapshot==*semantic_targets_snapshot_)return;
    // Compare complete snapshots before assigning a publication timestamp.
    // DELETEALL remains present when a reset/load removes a semantic target.
    semantic_targets_snapshot_=snapshot;
    const auto stamp=now();for(auto& marker:snapshot.markers)marker.header.stamp=stamp;
    semantic_targets_pub_->publish(snapshot);
  }
  void publishStateHistory(){
    if(!global_||!state_history_pub_||!state_history_markers_pub_)return;
    if(visit_snapshot_revision_&&*visit_snapshot_revision_==global_->visitRevision())return;
    const auto& states=global_->visitedStates();
    const auto first=states.size()-std::min(states.size(),kDisplayedHistoryStates);
    const auto first_range=states.size()-std::min(states.size(),kDisplayedHistoryRanges);
    std_msgs::msg::Header header;header.frame_id=frame_;header.stamp=now();
    std::ostringstream out;out<<std::setprecision(17)
      <<"{\"scope\":\"algorithm_visit_history\",\"coordinate_convention\":\"planning_height\""
      <<",\"revision\":"<<global_->visitRevision()
      <<",\"stamp_ns\":"<<(int64_t(header.stamp.sec)*1000000000+header.stamp.nanosec)
      <<",\"new_event_spacing_m\":1,\"odometry_graph_spacing_m\":0.5"
      <<",\"frontier_exclusion_radius_m\":3,\"background_exclusion_radius_m\":5"
      <<",\"restored_prefix_count\":"<<global_->restoredHistorySize()
      <<",\"total_states\":"<<states.size()<<",\"first_index\":"<<first
      <<",\"max_displayed_states\":"<<kDisplayedHistoryStates
      <<",\"truncated\":"<<(first?"true":"false")
      <<",\"range_spheres_first_index\":"<<first_range
      <<",\"max_displayed_ranges\":"<<kDisplayedHistoryRanges
      <<",\"marker\":[";
    const auto& marker=global_->lastVisitedMarker();
    for(int axis=0;axis<4;++axis){if(axis)out<<',';out<<marker[axis];}
    out<<"],\"states\":[";
    for(std::size_t i=first;i<states.size();++i){
      if(i!=first)out<<',';out<<'[';
      for(int axis=0;axis<4;++axis){if(axis)out<<',';out<<states[i][axis];}
      out<<']';
    }
    out<<"]}";
    std_msgs::msg::String message;message.data=out.str();state_history_pub_->publish(message);
    state_history_markers_pub_->publish(stateHistoryMarkers(states,header));
    visit_snapshot_revision_=global_->visitRevision();
  }
  void publishGlobalGraph(){
    // History can change while every public Graph field stays identical.
    // Its independent revision must be considered before the graph cache.
    publishStateHistory();
    if(global_&&global_snapshot_revision_&&*global_snapshot_revision_==global_->snapshotRevision())return;
    publishSemanticTargets();
    if(!global_)return;
    planner_msgs::msg::Graph message;global_->graphMessage(message);
    // Revision avoids rebuilding at odometry frequency. Exact message-field
    // comparison also suppresses changes invisible in float32 edge weights.
    if(!global_graph_snapshot_||message.vertices!=global_graph_snapshot_->vertices||
       message.edges!=global_graph_snapshot_->edges){
      message.header.frame_id=frame_;message.header.stamp=now();global_graph_pub_->publish(message);
      global_graph_snapshot_=std::move(message);
    }
    // These topics are durable state snapshots; late subscribers receive the
    // retained sample. Path, metrics and action/task events publish separately.
    global_snapshot_revision_=global_->snapshotRevision();
  }
  void publishGlobalPaths(const GlobalPathDiagnostics& diagnostic,const std::string& request,
                          bool response_success,const std::string& reason,
                          const HomingPathDiagnostics* homing_diagnostic=nullptr){
    // A homing request replaces the frontier display. Each publication is a
    // complete durable snapshot, including failed requests and empty returns.
    const GlobalPathDiagnostics empty;
    const auto& visible=response_success&&request=="frontier"?diagnostic:empty;
    std_msgs::msg::Header header;header.frame_id=frame_;header.stamp=now();
    global_paths_pub_->publish(globalPathMarkers(visible,header));
    std::ostringstream out;out<<std::boolalpha<<"{\"request\":"<<std::quoted(request)
      <<",\"success\":"<<response_success<<",\"reason\":"<<std::quoted(reason)
      <<",\"frame_id\":"<<std::quoted(frame_)<<",\"map_revision\":"<<map_revision_
      <<",\"stamp_ns\":"<<rclcpp::Time(header.stamp).nanoseconds()
      <<",\"graph_paths\":"<<globalPathDiagnosticsJson(visible)<<'}';
    std_msgs::msg::String message;message.data=out.str();global_path_geometry_pub_->publish(message);
    const HomingPathDiagnostics empty_home;
    const auto& home=response_success&&request=="homing"&&homing_diagnostic?*homing_diagnostic:empty_home;
    homing_path_pub_->publish(homingPathMarkers(home,header));
    std::ostringstream home_json;home_json<<std::boolalpha<<"{\"request\":"<<std::quoted(request)
      <<",\"success\":"<<response_success<<",\"reason\":"<<std::quoted(reason)
      <<",\"frame_id\":"<<std::quoted(frame_)<<",\"map_revision\":"<<map_revision_
      <<",\"stamp_ns\":"<<rclcpp::Time(header.stamp).nanoseconds()
      <<",\"homing_path\":"<<homingPathDiagnosticsJson(home)<<'}';
    message.data=home_json.str();homing_path_geometry_pub_->publish(message);
  }
  std::vector<geometry_msgs::msg::Pose> publishGlobal(GlobalResult result,const std::string& mode,
                                                   const HomingPathDiagnostics* homing_diagnostic=nullptr){
    publishGlobalGraph();
    interpolateResultOutput(result);
    publishRefinement(result.refinement,mode,result.success);
    publishGlobalPaths(result.graph_paths,mode,result.success,result.reason,homing_diagnostic);
    nav_msgs::msg::Path path;path.header.frame_id=frame_;path.header.stamp=now();
    std::vector<geometry_msgs::msg::Pose> poses;
    if(result.success)for(const auto& planning_state:result.path){
      const auto state=baseState(planning_state);
      geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
      pose.pose.position.x=state.x();pose.pose.position.y=state.y();pose.pose.position.z=state.z();
      pose.pose.orientation.z=std::sin(state[3]/2);pose.pose.orientation.w=std::cos(state[3]/2);
      poses.push_back(pose.pose);path.poses.push_back(pose);
    }
    global_path_pub_->publish(path);
    std::ostringstream text;text<<std::setprecision(12)<<"{\"success\":"<<(result.success?"true":"false")
      <<",\"reason\":\""<<result.reason<<"\",\"mode\":\""<<mode<<"\",\"target\":"<<result.target
      <<",\"score\":"<<result.score<<",\"distance_m\":"<<result.distance
      <<",\"frontiers\":"<<(global_?global_->frontierCount():0)
      <<",\"refinement\":"<<refinementStatus(result.refinement)
      <<",\"reconnection\":{\"attempted\":"<<(result.reconnection.attempted?"true":"false")
      <<",\"modified\":"<<(result.reconnection.modified?"true":"false")
      <<",\"reason\":\""<<result.reconnection.reason<<"\""
      <<",\"sampled_vertices\":"<<result.reconnection.sampled_vertices
      <<",\"checked_edges\":"<<result.reconnection.checked_edges
      <<",\"accepted_shortcuts\":"<<result.reconnection.accepted_shortcuts
      <<",\"used_shortcuts\":"<<result.reconnection.used_shortcuts
      <<",\"compressed_runs\":"<<result.reconnection.compressed_runs
      <<",\"input_length_m\":"<<result.reconnection.input_length
      <<",\"output_length_m\":"<<result.reconnection.output_length<<"}}";
    std_msgs::msg::String metrics;metrics.data=text.str();global_metrics_pub_->publish(metrics);return poses;
  }
  std::unique_ptr<GlobalMemory> global_;
  std::string sampling_seed_mode_{"fixed"};
  std::unique_ptr<SamplingSeedSequence> local_sampling_seeds_;
  std::optional<std::uint64_t> global_snapshot_revision_;
  std::optional<std::uint64_t> visit_snapshot_revision_;
  std::optional<planner_msgs::msg::Graph> global_graph_snapshot_;
  std::optional<visualization_msgs::msg::MarkerArray> semantic_targets_snapshot_;
  bool free_space_enabled_{false};double free_space_odom_timeout_{2.};
  std::vector<FreeSpaceSensor> free_space_sensors_;
  builtin_interfaces::msg::Time free_space_odom_stamp_;
  std::size_t free_space_clouds_{0},free_space_rays_{0},free_space_blocked_{0},free_space_points_{0};
  std::size_t free_space_skipped_{0},free_space_failed_{0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_space_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr free_space_status_service_;
  rclcpp::TimerBase::SharedPtr free_space_timer_;
  std::mutex mission_mutex_;
  std::unique_ptr<MissionBudget> mission_;
  // Protected by mission_mutex_, along with the budget, landing_due_ and the
  // rejected-battery count. No map or graph object is shared with this group.
  bool mission_odom_valid_{false};
  bool mission_enabled_{false},mission_auto_homing_{true};double mission_speed_{0.9};int rejected_battery_{0};
  FrontierSwitch frontier_switch_;bool go_home_if_fully_explored_{false};
  GlobalRepositioning global_repositioning_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planner_stop_sub_;
  bool landing_configured_{false},landing_enabled_{false},landing_due_{false};
  double landing_time_limit_{480.};
  rclcpp::Subscription<planner_msgs::msg::RobotStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mission_odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mission_status_service_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
  rclcpp::CallbackGroup::SharedPtr mission_group_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> mission_executor_;
  std::atomic<bool> mission_stopping_{false};
  std::thread mission_thread_;
  bool background_enabled_{false},local_triggered_{false};int background_attempts_{64},background_updates_{0},background_total_added_{0};
  std::string background_budget_mode_{"raw_draws"};double background_time_budget_{0.1},background_period_{0.5};
  std::uint64_t background_timer_ticks_{0};std::int64_t background_last_tick_ns_{0};
  BackgroundUpdate background_last_;rclcpp::TimerBase::SharedPtr background_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr expansion_service_,expansion_status_service_;
  std::shared_ptr<const PathValidationPolicy> validation_policy_;
  PathValidationInputs validation_inputs_;
  rclcpp::CallbackGroup::SharedPtr validation_group_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> validation_executor_;
  std::atomic<bool> validation_stopping_{false};
  std::thread validation_thread_;
  rclcpp::Service<planner_msgs::srv::ValidatePath>::SharedPtr validation_service_;
  std::string frame_,map_storage_;std::shared_ptr<const MapQuery> map_;
  std::shared_ptr<const MapQuery> observed_map_;
  std::shared_ptr<const GeofenceSet> geofences_;
  std::vector<std::vector<Eigen::Vector2d>> geofence_polygons_;
  bool geofence_enabled_{false},geofence_subscriber_enabled_{true};
  int geofence_revision_{0},rejected_geofences_{0},ignored_geofences_{0},observation_revision_{0};
  builtin_interfaces::msg::Time last_map_stamp_;
  std::unique_ptr<tf2_ros::Buffer> geofence_tf_;
  std::shared_ptr<tf2_ros::TransformListener> geofence_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr geofence_sub_;
  rclcpp::Service<planner_msgs::srv::PlannerGeofence>::SharedPtr geofence_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr geofence_clear_service_,geofence_status_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr geofence_enable_service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr geofence_marker_pub_;
 StateVec root_{StateVec::Zero()};bool odom_valid_{false};
  int map_revision_{0},rejected_maps_{0},known_cells_{0};
  uint64_t max_snapshot_voxels_{4000000};
  std::string last_rejected_map_reason_{"none"};
  bool map_use_envelope_{false},augmentation_enabled_{false};
  uint64_t map_envelopes_received_{0},map_envelopes_ignored_{0};
  MapUpdateSequence::Update last_ignored_map_update_;
  MapUpdateSequence map_sequence_;
  UnknownAugmentationSchedule augmentation_schedule_;
  int augmentation_bound_mode_{RobotBounds::Extended};
  uint64_t augmentation_requests_{0},augmentation_changed_{0},augmentation_failures_{0};
  uint64_t augmentation_odometry_samples_{0},augmentation_odometry_coalesced_{0};
  std::string augmentation_last_reason_{"not_requested"};
  rclcpp::Client<planner_msgs::srv::AugmentFreeBox>::SharedPtr augmentation_client_;
  std::optional<rclcpp::Client<planner_msgs::srv::AugmentFreeBox>::FutureAndRequestId> odometry_augmentation_;
  std::chrono::steady_clock::time_point odometry_augmentation_started_;
  rclcpp::TimerBase::SharedPtr augmentation_poll_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr augmentation_status_service_;
  rclcpp::Subscription<planner_msgs::msg::VoxelMapUpdate>::SharedPtr map_envelope_sub_;
  int tsdf_cells_{0};bool tsdf_fields_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<planner_msgs::msg::Graph>::SharedPtr graph_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ahead_metrics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sampling_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr adaptive_metrics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sampling_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_frusta_pub_;
  bool planning_diagnostics_enabled_{false};
  int gain_vertex_id_{20};
  std::ofstream map_trace_;
  std::string home_diagnostic_directory_;
  std::string validation_diagnostic_directory_;
  // Only the default/planning executor writes this pointer. Readers receive
  // their own immutable copy inside PathValidationMap's atomic publication.
  sensor_msgs::msg::PointCloud2::ConstSharedPtr validation_source_cloud_;
  // Only the single validation executor accesses this bounded capture flag.
  bool validation_diagnostic_captured_{false};
  std::set<std::string> home_diagnostic_captured_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr home_diagnostic_map_;
  rclcpp::TimerBase::SharedPtr map_trace_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sampler_pub_,projected_graph_pub_,failed_edges_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr shortest_paths_pub_,best_paths_pub_,no_gain_zones_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_clusters_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr frontier_clustering_status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_paths_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr homing_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr homing_path_geometry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr global_path_geometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr refinement_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr refinement_planes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr refined_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr refinement_geometry_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gain_rays_pub_,gain_samples_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ground_edge_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ground_edge_geometry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gain_geometry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visualization_geometry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_update_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accepted_map_pub_;
  rclcpp::Service<planner_msgs::srv::PlannerSrv>::SharedPtr plan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sensor_status_service_;
  std::optional<GlobalBound> global_bound_,default_global_bound_;
  int global_bound_revision_{0};
  rclcpp::Service<planner_msgs::srv::PlannerSetGlobalBound>::SharedPtr global_bound_service_;
  rclcpp::Service<planner_msgs::srv::PlannerDynamicGlobalBound>::SharedPtr dynamic_global_bound_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_bound_status_service_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_bound_marker_pub_;
  ExplorationDirection exploration_direction_;
  rclcpp::TimerBase::SharedPtr direction_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr direction_status_service_;
  rclcpp::Service<planner_msgs::srv::PlanAhead>::SharedPtr plan_ahead_service_;
  rclcpp::Service<planner_msgs::srv::PlannerSetHomingPos>::SharedPtr set_home_service_;
  bool auto_home_{true};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_status_service_;
  rclcpp::Service<planner_msgs::srv::PlannerGlobal>::SharedPtr global_service_;
  rclcpp::Service<planner_msgs::srv::PlannerHoming>::SharedPtr homing_service_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<planner_msgs::msg::Graph>::SharedPtr global_graph_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_history_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr state_history_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr global_metrics_pub_;
};
}
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<gbplanner2::LocalPlannerNode>());}
  catch(const std::exception& error){std::cerr<<"Local planner startup failed: "<<error.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
