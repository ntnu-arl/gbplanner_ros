// Derived from GBPlanner 2 terrain projection, BSD-3-Clause.
#include "gbplanner_local/ground_terrain.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace gbplanner2 {
const char* groundRayProjectionModeName(GroundRayProjectionMode mode){
  switch(mode){
    case GroundRayProjectionMode::VoxelSurface:return "voxel_surface";
    case GroundRayProjectionMode::UpstreamSample:return "upstream_sample";
    default:throw std::invalid_argument("Invalid ground ray projection mode");
  }
}
const char* groundTsdfProjectionModeName(GroundTsdfProjectionMode mode){
  switch(mode){
    case GroundTsdfProjectionMode::UpstreamStep:return "upstream_step";
    case GroundTsdfProjectionMode::ZeroCrossing:return "zero_crossing";
    case GroundTsdfProjectionMode::ZeroCrossingSimplex:return "zero_crossing_simplex";
    default:throw std::invalid_argument("Invalid ground TSDF projection mode");
  }
}
void GroundParameters::validate()const{
  groundRayProjectionModeName(ray_projection_mode);
  groundTsdfProjectionModeName(tsdf_projection_mode);
  const auto positive=[](double v){return std::isfinite(v)&&v>0.;};
  if(!positive(max_ground_height)||!positive(robot_height)||!positive(projection_max_length)||
     !positive(max_inclination)||max_inclination>=M_PI/2||!positive(max_negative_inclination)||
     max_negative_inclination>=M_PI/2||!center_offset.allFinite()||!robot_size.allFinite()||
     (point_robot?!(robot_size.array()==0.).all():(robot_size.array()<=0.).any())||
     !std::isfinite(hanging_vertex_penalty)||hanging_vertex_penalty<0)
    throw std::invalid_argument("Invalid ground-terrain parameters");
}

const char* groundStatusName(GroundStatus status){
  switch(status){
    case GroundStatus::Supported:return "supported";
    case GroundStatus::Admissible:return "admissible";
    case GroundStatus::Hanging:return "hanging";
    case GroundStatus::NoGround:return "no_ground";
    case GroundStatus::Steep:return "steep";
    case GroundStatus::Unknown:return "unknown";
    case GroundStatus::Occupied:return "occupied";
    case GroundStatus::OffTerrain:return "off_terrain";
    default:return "invalid";
  }
}

struct TerrainProjector::QueryCache {
  using Key=std::array<std::uint64_t,3>;
  struct Hash {
    std::size_t operator()(const Key& key)const noexcept {
      std::uint64_t hash=0;
      for(auto bits:key){
        bits=(bits^(bits>>30))*UINT64_C(0xbf58476d1ce4e5b9);
        bits=(bits^(bits>>27))*UINT64_C(0x94d049bb133111eb);
        bits^=bits>>31;
        hash^=bits+UINT64_C(0x9e3779b97f4a7c15)+(hash<<6)+(hash>>2);
      }
      return static_cast<std::size_t>(hash);
    }
  };
  static Key key(const Eigen::Vector3d& point){
    Key result;
    static_assert(sizeof(double)==sizeof(std::uint64_t),"Expected 64-bit coordinates");
    for(int axis=0;axis<3;++axis)std::memcpy(&result[axis],&point[axis],sizeof(double));
    return result;
  }
  // Bound memory without approximating coordinates or evicting a verdict into
  // a different key. Once full, new queries simply use the original algorithm.
  static constexpr std::size_t kMaxEntries=65536;
  std::unordered_map<Key,GroundProjection,Hash> projections;
  std::unordered_map<Key,Hit,Hash> supports;
  TerrainQueryReuseStats stats;
};

TerrainProjector::TerrainProjector(std::shared_ptr<const MapQuery> map,GroundParameters parameters)
    :TerrainProjector(std::move(map),std::move(parameters),TerrainQueryReuse::Disabled){}
TerrainProjector::TerrainProjector(std::shared_ptr<const MapQuery> map,GroundParameters parameters,
                                   TerrainQueryReuse reuse)
    :map_(std::move(map)),p_(std::move(parameters)){
  p_.validate();if(!map_||!std::isfinite(map_->resolution())||map_->resolution()<=0)
    throw std::invalid_argument("Terrain projection needs a valid map");
  if(p_.ray_projection_mode==GroundRayProjectionMode::UpstreamSample){
    const float resolution=static_cast<float>(map_->resolution());
    if(!std::isfinite(resolution)||resolution<=0.f||!std::isfinite(1.f/resolution))
      throw std::invalid_argument("Upstream ground ray needs a representable float voxel size");
  }
  if(reuse==TerrainQueryReuse::ImmutableSnapshot)query_cache_=std::make_shared<QueryCache>();
}

void TerrainProjector::clearQueryReuse(){
  if(!query_cache_)return;
  query_cache_->projections.clear();query_cache_->supports.clear();query_cache_->stats={};
}
TerrainQueryReuseStats TerrainProjector::queryReuseStats()const{
  return query_cache_?query_cache_->stats:TerrainQueryReuseStats{};
}
TerrainProjector::Hit TerrainProjector::castDown(const Eigen::Vector3d& start)const{
  if(!query_cache_||!start.allFinite())return castDownUncached(start);
  auto& cache=*query_cache_;const auto key=QueryCache::key(start);
  const auto found=cache.supports.find(key);
  if(found!=cache.supports.end()){++cache.stats.support_hits;return found->second;}
  ++cache.stats.support_evaluations;
  const auto result=castDownUncached(start);
  if(cache.supports.size()<QueryCache::kMaxEntries)cache.supports.emplace(key,result);
  return result;
}
TerrainProjector::Hit TerrainProjector::castDownUncached(const Eigen::Vector3d& start)const{
  if(p_.interpolate_projection_distance)return p_.tsdf_projection_mode==GroundTsdfProjectionMode::UpstreamStep?
      castDownDistance(start):castDownZeroCrossing(start);
  if(p_.ray_projection_mode==GroundRayProjectionMode::UpstreamSample)return castDownUpstream(start);
  Hit result;result.distance=p_.projection_max_length;
  map_->ray(start,start-Eigen::Vector3d(0,0,p_.projection_max_length),
      [&](const Eigen::Vector3d& center,VoxelStatus status){
        if(status==VoxelStatus::Free)return true;
        result.status=status;
        // Occupancy snapshots contain finite cells, not signed distances.
        // Use the observed cell's upper face rather than inventing a TSDF.
        result.surface_z=center.z()+.5*map_->resolution();
        result.distance=std::max(0.,start.z()-result.surface_z);
        return false; // Unknown must not be skipped to find a hidden floor.
      });
  return result;
}

TerrainProjector::Hit TerrainProjector::castDownUpstream(const Eigen::Vector3d& start)const{
  const float voxel_size=static_cast<float>(map_->resolution());
  const float voxel_size_inv=1.0/voxel_size;
  const Eigen::Vector3d end=start-Eigen::Vector3d(0,0,p_.projection_max_length);
  const Eigen::Vector3d ray=end-start;
  const double ray_length=ray.norm();
  Hit result;result.surface_z=end.z();result.distance=ray_length;
  if(ray_length==0.)return result;
  const Eigen::Vector3d direction=ray/ray_length;
  // getRayStatus(start,end,true,end_voxel,dist) in voxblox_common_impl.cpp.
  // Both the float rounding and double(float(resolution))*0.9 step matter:
  // the returned support is this sampled query, never the cell's top face.
  for(double distance=0.;distance<ray_length;distance+=voxel_size*.9){
    const Eigen::Vector3d point=start+distance*direction;
    Eigen::Vector3d index;
    for(int axis=0;axis<3;++axis){
      const float scaled=static_cast<float>(point[axis])*voxel_size_inv;
      index[axis]=std::floor(scaled+1.e-6f);
    }
    // MapQuery uses double coordinates. Query the selected global cell's
    // center so a second double-based index operation cannot change the cell.
    const Eigen::Vector3d center=(index.array()+.5).matrix()*map_->resolution();
    const auto status=center.allFinite()?map_->voxel(center):VoxelStatus::Unknown;
    if(status==VoxelStatus::Free)continue;
    result.status=status;result.surface_z=point.z();
    result.distance=std::abs(start.z()-point.z());return result;
  }
  // Upstream queries the endpoint for TSDF metadata but always returns Free.
  // An occupied or unknown endpoint is not an additional support hit.
  return result;
}

TerrainProjector::Hit TerrainProjector::castDownDistance(const Eigen::Vector3d& start)const{
  Hit result;result.distance=p_.projection_max_length;
  Eigen::Vector3d point=start;
  while(start.z()-point.z()<p_.projection_max_length){
    const auto observation=map_->tsdfDistance(point);
    if(!observation){
      result.status=VoxelStatus::Unknown;result.surface_z=point.z();
      result.distance=start.z()-point.z();return result;
    }
    // getPointDistance clamps observed negative TSDF to .001. A negative
    // value must not be confused with the source's -1 unknown sentinel.
    const double distance=observation->distance<0.?.001:observation->distance;
    if(distance<=map_->resolution()){
      const double surface_z=point.z()-distance;
      const double traveled=start.z()-surface_z;
      if(traveled>p_.projection_max_length)return result;
      result.status=VoxelStatus::Occupied;result.surface_z=surface_z;
      result.distance=traveled;return result;
    }
    // Retain the source's 2 cm retreat from the distance-field step, but use
    // actual Z travel rather than adding the unreduced distance each time.
    // Very fine grids need a positive step when distance is below 2 cm.
    const double step=distance>.02?distance-.02:distance*.5;
    point.z()-=step;
  }
  return result;
}

TerrainProjector::Hit TerrainProjector::castDownZeroCrossing(const Eigen::Vector3d& start)const{
  // Projective TSDF values are not vertical travel distances. Query the raw
  // signed field on world-zero half-voxel planes, including both voxel faces
  // and interpolation-stencil boundaries, and take its first sign crossing.
  Hit result;result.distance=p_.projection_max_length;
  const double end_z=start.z()-p_.projection_max_length,step=.5*map_->resolution();
  const double z_tolerance=1.e-3*map_->resolution();
  // Reuse only raw stencil values within this ray and under the projector's
  // explicit immutable-snapshot contract. Every new ray starts empty.
  TsdfQueryContext context;
  const auto query=[&](double z){const Eigen::Vector3d point(start.x(),start.y(),z);
    return p_.tsdf_projection_mode==GroundTsdfProjectionMode::ZeroCrossingSimplex?
        (query_cache_?map_->tsdfDistanceSimplexCached(point,context):
                      map_->tsdfDistanceSimplex(point)):map_->tsdfDistance(point);};
  const auto unknown=[&](double z){Hit hit;hit.status=VoxelStatus::Unknown;
    hit.surface_z=z;hit.distance=start.z()-z;return hit;};
  const auto rejected=[&](){Hit hit;hit.rejected=true;return hit;};
  const auto next_lower_plane=[](double z,double spacing,double offset){
    double plane=(std::floor(z/spacing-offset)+offset)*spacing;
    if(plane>=z)plane-=spacing;
    return plane;
  };
  Eigen::Vector2d simplex_fraction=Eigen::Vector2d::Zero();
  if(p_.tsdf_projection_mode==GroundTsdfProjectionMode::ZeroCrossingSimplex){
    const Eigen::Vector2d grid=start.head<2>()/map_->resolution()-Eigen::Vector2d::Constant(.5);
    simplex_fraction=grid-grid.array().floor().matrix();
  }
  double upper_z=start.z();auto upper=query(upper_z);
  if(!upper||!std::isfinite(upper->distance))return unknown(upper_z);
  // No positive side has been observed. Do not invent a floor from a negative
  // starting sample, or retry from the side rays' artificially raised origins.
  if(upper->distance<=0.)return rejected();
  while(upper_z>end_z){
    double lower_z=std::floor(upper_z/step)*step;
    if(lower_z>=upper_z)lower_z-=step;
    if(p_.tsdf_projection_mode==GroundTsdfProjectionMode::ZeroCrossingSimplex){
      // Along a vertical ray the fixed tetrahedron changes at fz=fx/fy.
      // Include these knots so a negative interval cannot hide between two
      // positive half-voxel samples. Between knots this field is affine in Z.
      for(int axis=0;axis<2;++axis)
        lower_z=std::max(lower_z,next_lower_plane(upper_z,map_->resolution(),.5+simplex_fraction[axis]));
    }
    lower_z=std::max(lower_z,end_z);
    if(lower_z>=upper_z)return result;
    auto lower=query(lower_z);
    if(!lower||!std::isfinite(lower->distance))return unknown(lower_z);
    if(lower->distance<=0.){
      // Containing-voxel fallback can jump between unrelated signed values.
      // Require continuous interpolation throughout this first bracket; a
      // discontinuous crossing is unsupported, not a hanging candidate.
      if(!upper->interpolated||!lower->interpolated)return rejected();
      for(int iteration=0;upper_z-lower_z>z_tolerance&&iteration<64;++iteration){
        const double middle_z=lower_z+.5*(upper_z-lower_z);const auto middle=query(middle_z);
        if(!middle||!std::isfinite(middle->distance))return unknown(middle_z);
        if(!middle->interpolated)return rejected();
        if(middle->distance>0.){upper_z=middle_z;upper=middle;}
        else{lower_z=middle_z;lower=middle;}
      }
      if(upper_z-lower_z>z_tolerance)return rejected();
      const double denominator=upper->distance-lower->distance;
      if(!std::isfinite(denominator)||denominator<=0.)return rejected();
      const double fraction=-lower->distance/denominator;
      const double estimate=lower_z+fraction*(upper_z-lower_z);const auto check=query(estimate);
      if(!check||!std::isfinite(check->distance))return unknown(estimate);
      const double residual_tolerance=map_->resolution()*1.e-6+
          32.*std::numeric_limits<double>::epsilon()*std::max(std::abs(lower->distance),std::abs(upper->distance));
      if(!check->interpolated||std::abs(check->distance)>residual_tolerance)return rejected();
      result.status=VoxelStatus::Occupied;result.surface_z=estimate;
      result.distance=start.z()-estimate;result.lower_z=lower_z;result.upper_z=upper_z;return result;
    }
    upper_z=lower_z;upper=lower;
  }
  return result;
}

StateVec TerrainProjector::baseState(const StateVec& planning_state)const{
  StateVec base=planning_state;base.z()+=p_.robot_height-p_.max_ground_height;return base;
}

GroundStatus TerrainProjector::bodyStatus(const StateVec& state)const{
  const auto status=map_->box(baseState(state).head<3>()+p_.center_offset,p_.robot_size);
  if(status==VoxelStatus::Occupied)return GroundStatus::Occupied;
  if(status==VoxelStatus::Unknown)return GroundStatus::Unknown;
  return GroundStatus::Supported;
}

GroundProjection TerrainProjector::project(const StateVec& candidate)const{
  // Yaw does not enter any terrain query, but remains part of the returned
  // state. Nonfinite yaw must still take the original invalid-state branch.
  if(!query_cache_||!candidate.allFinite())return projectUncached(candidate);
  auto& cache=*query_cache_;const auto key=QueryCache::key(candidate.head<3>());
  const auto found=cache.projections.find(key);
  if(found!=cache.projections.end()){
    ++cache.stats.projection_hits;auto result=found->second;
    result.projected_state[3]=candidate[3];return result;
  }
  ++cache.stats.projection_evaluations;
  const auto result=projectUncached(candidate);
  // Only the actual input is a key: projecting an already projected TSDF
  // state can produce a different height and must remain a distinct query.
  if(cache.projections.size()<QueryCache::kMaxEntries)cache.projections.emplace(key,result);
  return result;
}
GroundProjection TerrainProjector::projectUncached(const StateVec& candidate)const{
  GroundProjection result;result.projected_state=candidate;
  result.ground_z=std::numeric_limits<double>::quiet_NaN();
  if(!candidate.allFinite())return result;
  const std::array<Eigen::Vector3d,5> offsets{{{0,0,0},{.5,0,.5},{-.5,0,.5},{0,.5,.5},{0,-.5,.5}}};
  int unknown=0;Hit central;
  for(std::size_t i=0;i<offsets.size();++i){
    const Eigen::Vector3d start=candidate.head<3>()+offsets[i];const Hit hit=castDown(start);
    if(i==0)central=hit;
    if(hit.rejected){result.status=GroundStatus::NoGround;return result;}
    if(hit.status==VoxelStatus::Unknown){++unknown;continue;}
    if(hit.status!=VoxelStatus::Occupied)continue;
    result.projected_state=candidate;result.projected_state.x()=start.x();result.projected_state.y()=start.y();
    // The original side-ray starts 0.5 m higher but subtracts its distance from
    // the unraised sample. Use world surface Z to remove that height bias.
    result.projected_state.z()=hit.surface_z+p_.max_ground_height;
    result.ground_z=hit.surface_z;result.projection_distance=hit.distance;result.ray_index=static_cast<int>(i);
    result.zero_crossing_lower_z=hit.lower_z;result.zero_crossing_upper_z=hit.upper_z;
    result.support=true;result.status=bodyStatus(result.projected_state);return result;
  }
  if(unknown==static_cast<int>(offsets.size())){
    result.status=GroundStatus::Hanging;result.hanging=true;result.ray_index=0;
    result.projection_distance=central.distance;
    result.projected_state.z()=central.surface_z+p_.max_ground_height;
  }else result.status=GroundStatus::NoGround;
  return result;
}

GroundEdge TerrainProjector::edge(const StateVec& from,const StateVec& to)const{
  return edgeImpl(from,to,GroundHangingPermission::Forbid);
}
GroundEdge TerrainProjector::validateFixedEdge(const StateVec& from,const StateVec& to,
                                               bool allow_height_attachment)const{
  GroundEdge result;
  if(!from.allFinite()||!to.allFinite())return result;
  const Eigen::Vector3d delta=(to-from).head<3>();
  const double length=delta.norm(),sample_count=std::ceil(length/(.5*map_->resolution()));
  if(!std::isfinite(length)||sample_count>=std::numeric_limits<int>::max())return result;
  // Collinear subdivision has the same swept volume. Reprojecting its knots
  // does not: TSDF distance stepping depends on the starting height. Check
  // this original sweep, not a second projected path with altered clearance.
  const auto occupancy=map_->path(baseState(from).head<3>()+p_.center_offset,
                                  baseState(to).head<3>()+p_.center_offset,p_.robot_size);
  if(occupancy!=VoxelStatus::Free){
    result.status=occupancy==VoxelStatus::Occupied?GroundStatus::Occupied:GroundStatus::Unknown;
    return result;
  }
  const double slope=std::atan2(delta.z(),delta.head<2>().norm());
  const bool height_attachment=allow_height_attachment&&
      (p_.interpolate_projection_distance||p_.ray_projection_mode==GroundRayProjectionMode::UpstreamSample)&&
      delta.head<2>().norm()<=std::min(1e-3,.01*map_->resolution())&&
      std::abs(delta.z())<=map_->resolution();
  if(!height_attachment&&(std::abs(slope)>p_.max_inclination+1e-9||
                          -slope>p_.max_negative_inclination+1e-9)){
    result.status=GroundStatus::Steep;return result;
  }
  const int count=std::max(1,static_cast<int>(sample_count));
  double first_support_height=0.;
  for(int i=0;i<=count;++i){
    const Eigen::Vector3d sample=from.head<3>()+(double(i)/count)*delta;
    // Support uses the central ray only. A side-ray observation cannot make
    // a hole below the actual commanded geometry traversable.
    const Hit support=castDown(sample);
    if(support.status==VoxelStatus::Unknown){result.status=GroundStatus::Hanging;return result;}
    if(support.status!=VoxelStatus::Occupied){result.status=GroundStatus::NoGround;return result;}
    const double supported_height=support.surface_z+p_.max_ground_height;
    if(i==0)first_support_height=supported_height;
    if(height_attachment&&i==count&&
       std::abs(supported_height-first_support_height)>map_->resolution()){
      result.status=GroundStatus::Steep;return result;
    }
    if(std::abs(supported_height-sample.z())>map_->resolution()+1e-8){
      result.status=GroundStatus::OffTerrain;return result;
    }
  }
  result.polyline={from,to};result.actual_length=length;
  result.max_uphill=std::max(0.,slope);result.max_downhill=std::max(0.,-slope);
  result.mean_inclination=std::abs(slope);result.status=GroundStatus::Admissible;
  return result;
}
GroundEdge TerrainProjector::candidateEdge(const StateVec& from,const StateVec& to,
                                         GroundHangingPermission permission)const{
  return edgeImpl(from,to,permission);
}
GroundEdge TerrainProjector::edgeImpl(const StateVec& from,const StateVec& to,
                                    GroundHangingPermission permission)const{
  GroundEdge result;
  if(!from.allFinite()||!to.allFinite())return result;
  const auto first=project(from),last=project(to);
  // Rrg::expandGraph authorizes intermediate unknown support only when one
  // logical endpoint is hanging. Resolve the permission once: intermediate
  // samples and overshoot endpoints cannot grant it during validation.
  const bool allow_hanging=permission==GroundHangingPermission::OriginalEndpointHanging||
      (permission==GroundHangingPermission::FromEndpoints&&(first.hanging||last.hanging));
  const auto allowed=[&](const GroundProjection& point){
    return point.status==GroundStatus::Supported||(allow_hanging&&point.status==GroundStatus::Hanging);
  };
  if(!allowed(first)){result.status=first.status;return result;}
  if(!allowed(last)){result.status=last.status;return result;}
  for(const auto* point:{&first,&last}){
    // project() returns Supported only after checking this exact projected
    // body's clearance. Hanging projections have not had that check yet.
    if(point->status==GroundStatus::Supported)continue;
    const auto body=bodyStatus(point->projected_state);
    if(body!=GroundStatus::Supported){result.status=body;return result;}
  }
  bool speculative=first.hanging||last.hanging;
  // Candidate sampling may relocate to a side-ray support. An executable
  // connection cannot silently relocate either requested endpoint.
  if((first.projected_state-from).head<2>().norm()>1e-9||
     (last.projected_state-to).head<2>().norm()>1e-9){result.status=GroundStatus::Hanging;return result;}
  const StateVec a=first.projected_state,b=last.projected_state;
  // Distance-guided projection stops within one voxel of the surface, so its
  // last estimate depends slightly on the starting height. Reprojecting one
  // site for a global-graph attachment must not create a vertical drive edge.
  // Collapse only the same XY site with sub-voxel height differences; moving
  // edges still undergo every directional slope and swept-body check below.
  if(p_.interpolate_projection_distance&&p_.tsdf_projection_mode==GroundTsdfProjectionMode::UpstreamStep&&!speculative&&
     (from-to).head<2>().norm()<1e-9&&std::abs(from.z()-to.z())<=map_->resolution()&&
     std::abs(a.z()-b.z())<=map_->resolution()){
    result.polyline={a};result.status=GroundStatus::Admissible;return result;
  }
  const Eigen::Vector3d delta=(b-a).head<3>();const double length=delta.norm();
  const double step=2.*map_->resolution();
  result.polyline.push_back(a);
  if(length>1e-10){
    for(double distance=step;distance<length;distance+=step){
      const double fraction=distance/length;StateVec sample=a;
      sample.head<3>()=a.head<3>()+fraction*delta;
      const auto projection=project(sample);
      if(!allowed(projection)){result.status=projection.status;return result;}
      speculative=speculative||projection.hanging;
      result.polyline.push_back(projection.projected_state);
    }
    // Avoid a tiny last segment after voxel-height quantization. This fixes
    // the original erase(end()) and always retains the projected endpoint.
    if(result.polyline.size()>1&&(result.polyline.back()-b).head<3>().norm()<.75*step)
      result.polyline.pop_back();
    result.polyline.push_back(b);
  }

  double angle_sum=0.;std::size_t segments=0;bool steep=false;
  for(std::size_t i=1;i<result.polyline.size();++i){
    const auto& start=result.polyline[i-1];auto& end=result.polyline[i];
    const Eigen::Vector3d segment=(end-start).head<3>();const double segment_length=segment.norm();
    if(segment_length<1e-10)continue;
    const double slope=std::atan2(segment.z(),segment.head<2>().norm());
    result.actual_length+=segment_length;angle_sum+=std::abs(slope);++segments;
    result.max_uphill=std::max(result.max_uphill,slope);result.max_downhill=std::max(result.max_downhill,-slope);
    result.mean_inclination=angle_sum/double(segments);
    if(std::abs(slope)>p_.max_inclination||-slope>p_.max_negative_inclination)steep=true;
    end[3]=std::atan2(segment.y(),segment.x());
  }
  if(steep){result.status=GroundStatus::Steep;return result;}
  for(std::size_t i=1;i<result.polyline.size();++i){
    const auto& start=result.polyline[i-1];const auto& end=result.polyline[i];
    const Eigen::Vector3d segment=(end-start).head<3>();const double segment_length=segment.norm();
    if(segment_length<1e-10)continue;

    // A side-ray fallback must not turn an unknown hole into a free bridge.
    // Check continuous support under the actual polyline at half-voxel steps,
    // with no fallback. Also catch a narrow known drop missed by coarse knots.
    const int count=std::max(1,static_cast<int>(std::ceil(segment_length/(.5*map_->resolution()))));
    for(int k=0;k<=count;++k){
      const Eigen::Vector3d sample=start.head<3>()+(double(k)/count)*segment;
      const Hit support=castDown(sample);
      if(support.status==VoxelStatus::Unknown){
        if(!allow_hanging){result.status=GroundStatus::Hanging;return result;}
        speculative=true;continue;
      }
      if(support.status!=VoxelStatus::Occupied){result.status=GroundStatus::NoGround;return result;}
      if(std::abs(sample.z()-support.surface_z-p_.max_ground_height)>map_->resolution()+1e-8){
        result.status=GroundStatus::Steep;return result;
      }
    }
    const auto occupancy=map_->path(baseState(start).head<3>()+p_.center_offset,
                                    baseState(end).head<3>()+p_.center_offset,p_.robot_size);
    if(occupancy!=VoxelStatus::Free){
      result.status=occupancy==VoxelStatus::Occupied?GroundStatus::Occupied:GroundStatus::Unknown;return result;
    }
  }
  result.speculative=speculative;
  result.status=speculative?GroundStatus::Hanging:GroundStatus::Admissible;return result;
}
}
