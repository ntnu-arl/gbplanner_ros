// Derived from gbplanner/src/rrg.cpp and planner_common/src/params.cpp,
// GBPlanner 2, copyright (c) 2020 UNR Autonomous Robots Lab, BSD-3-Clause.
// This stage ports the aerial, configured-cuboid local branch and static sensors.
#include "gbplanner_local/local_planner.hpp"
#include "gbplanner_local/trajectory.hpp"
#include "gbplanner_local/sensor_geometry.hpp"
#include "gbplanner_local/path_heading.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <queue>
#include <limits>

namespace gbplanner2 {
namespace {
bool expandGroundWithTerrain(GraphManager& graph,const std::shared_ptr<const MapQuery>& map,
    const Parameters& p,StateVec state,PlanResult& result,int max_edges,
    GroundEdgeCache* edges,bool allow_hanging,PlanningDiagnostics* diagnostics,
    const TerrainProjector& terrain);
}
void PlanningDiagnostics::reset(){
  valid_samples.clear();invalid_samples.clear();projected_vertices.clear();
  projected_segments.clear();failed_edges.clear();
  shortest_vertices.clear();best_path.clear();best_id=-1;best_gain=0.;
  gains=GainGeometry{};gains.enabled=options.gains;gains.requested_id=options.gain_vertex_id;
  ground_edges=GroundEdgeDiagnostics{};ground_edges.enabled=options.ground_edges;
  if(options.gains)gains.reason="not_entered";
  dropped_samples=0;dropped_failed_edges=0;dropped_projected_edges=0;
}
void PlanningDiagnostics::sample(const StateVec& state,bool valid){
  if(!options.samples)return;
  if(valid_samples.size()+invalid_samples.size()>=kMaxSamples){++dropped_samples;return;}
  (valid?valid_samples:invalid_samples).push_back(state.head<3>());
}
void PlanningDiagnostics::failedEdge(const Eigen::Vector3d& from,const Eigen::Vector3d& to){
  if(!options.failed_edges)return;
  if(failed_edges.size()>=kMaxFailedEdges){++dropped_failed_edges;return;}
  failed_edges.push_back({from,to});
}
void PlanningDiagnostics::projectedEdge(const GroundEdge& edge){
  if(!options.projected_graph||edge.polyline.size()<2)return;
  // Preserve whole checked polylines rather than drawing a truncated edge.
  if(edge.polyline.size()>kMaxProjectedVertices-projected_vertices.size()){
    ++dropped_projected_edges;return;
  }
  for(std::size_t i=0;i<edge.polyline.size();++i){
    projected_vertices.push_back(edge.polyline[i].head<3>());
    if(i)projected_segments.push_back({edge.polyline[i-1].head<3>(),edge.polyline[i].head<3>()});
  }
}
LocalPlanner::LocalPlanner(std::shared_ptr<const MapQuery> map,Parameters p,PlanningDiagnosticsOptions diagnostics,
                           ExactRayGainReuse gain_reuse)
    :map_(std::move(map)),configured_p_(p),p_(std::move(p)),
     exact_ray_gain_reuse_(gain_reuse==ExactRayGainReuse::ImmutableSnapshot) {
  diagnostics_.options=diagnostics;
  p_.validate(); if(!map_)throw std::invalid_argument("Map required");
  if(p_.ground_robot)terrain_=std::make_unique<TerrainProjector>(map_,p_.groundParameters(),
      exact_ray_gain_reuse_?TerrainQueryReuse::ImmutableSnapshot:TerrainQueryReuse::Disabled);
  world_to_sampling_=p_.worldToSampling();
  sensors_=p_.sensors;
  if(sensors_.empty()){
    SensorParameters legacy;legacy.range=p_.sensor_range;
    legacy.horizontal_fov=p_.horizontal_fov;legacy.vertical_fov=p_.vertical_fov;
    legacy.horizontal_resolution=p_.horizontal_resolution;legacy.vertical_resolution=p_.vertical_resolution;
    legacy.frontier_threshold=p_.frontier_threshold;sensors_.push_back(legacy);
  }
  // SensorParamsBase uses direct configured Y/P/R rotation despite its
  // rot_B2S name. Keep the original non-spherical vertical endpoint convention.
  for(const auto& sensor:sensors_){
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(sensor.rotations[0],Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(sensor.rotations[1],Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(sensor.rotations[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
    const bool no_extrinsics=sensor.rotations.isZero(0.)&&sensor.center_offset.isZero(0.);
    std::vector<Eigen::Vector3d> endpoints;
    std::size_t width=0,height=0;
    for(double v=-sensor.vertical_fov/2;v<sensor.vertical_fov/2-1e-10;v+=sensor.vertical_resolution){
      std::size_t row_width=0;
      for(double h=-sensor.horizontal_fov/2;h<sensor.horizontal_fov/2-1e-10;h+=sensor.horizontal_resolution){
        const Eigen::Vector3d point(sensor.range*std::cos(h),sensor.range*std::sin(h),sensor.range*std::sin(v));
        // Avoid extra arithmetic for legacy scalar configuration.
        if(no_extrinsics)endpoints.push_back(point);
        else endpoints.push_back(rotation*point+sensor.center_offset);
        ++row_width;
      }
      width=row_width;++height;
    }
    ray_grid_sizes_.emplace_back(width,height);
    rays_.push_back(std::move(endpoints));
  }
}
bool LocalPlanner::inside(const StateVec& s,const StateVec& root) const {
  if(!s.allFinite())return false;
  // Clip in the same coordinate frame in which samples were drawn. Comparing
  // rotated world offsets with the unrotated min/max would crop the workspace.
  const Eigen::Vector3d local=world_to_sampling_*(s.head<3>()-root.head<3>());
  if((local.array()<p_.local_min.array()).any()||(local.array()>p_.local_max.array()).any())return false;
  const Eigen::Vector3d c=(terrain_?terrain_->baseState(s):s).head<3>()+p_.center_offset;
  return (c.array()-0.5*p_.robot_size.array()>map_->minimum().array()).all() &&
         (c.array()+0.5*p_.robot_size.array()<map_->maximum().array()).all();
}
bool LocalPlanner::expand(StateVec state,PlanResult& r) {
  if(p_.ground_robot)return expandGroundWithTerrain(graph_,map_,p_,state,r,p_.max_edges,
      &ground_edges_,true,&diagnostics_,*terrain_);
  return expandAerial(graph_,map_,p_,state,r,p_.max_edges,&diagnostics_);
}
bool expandAerial(GraphManager& graph_,const std::shared_ptr<const MapQuery>& map_,
                  const Parameters& p_,StateVec state,PlanResult& r,int max_edges,PlanningDiagnostics* diagnostics) {
  // Rrg::expandGraph aerial branch: steer toward sample, test overshoot,
  // create a tree edge, then connect collision-free neighboring vertices.
  Vertex* nearest=nullptr;
  if(!graph_.getNearestVertex(&state,&nearest)) return false;
  Eigen::Vector3d origin=nearest->state.head<3>(),direction=state.head<3>()-origin;
  double distance=direction.norm();
  if(distance<=p_.edge_min)return false;
  if(distance>p_.edge_max)direction=p_.edge_max*direction.normalized();
  distance=direction.norm();state.head<3>()=origin+direction;
  const Eigen::Vector3d overshoot=p_.overshoot*direction.normalized();
  Eigen::Vector3d a=origin+p_.center_offset;
  if(nearest->id!=0)a-=overshoot;
  const Eigen::Vector3d b=origin+p_.center_offset+direction+overshoot;
  if(map_->path(a,b,p_.robot_size)!=VoxelStatus::Free){
    ++r.collision_rejections;if(diagnostics)diagnostics->failedEdge(a,b);return false;
  }
  auto vertex=std::make_unique<Vertex>(graph_.generateVertexID(),state);
  vertex->parent=nearest;vertex->distance=nearest->distance+distance;
  Vertex* added=vertex.get();graph_.addVertex(added);vertex.release();
  nearest->children.push_back(added);graph_.addEdge(added,nearest,distance);
  if(p_.graph_mode) {
    std::vector<Vertex*> neighbors;graph_.getNearestVertices(&state,p_.neighbor_radius,&neighbors);
    std::sort(neighbors.begin(),neighbors.end(),[](auto* lhs,auto* rhs){return lhs->id<rhs->id;});
    for(auto* v:neighbors) {
      if(v==added||v==nearest||graph_.getNumEdges()>=max_edges)continue;
      direction=v->state.head<3>()-state.head<3>();distance=direction.norm();
      if(distance<=p_.neighbor_min||distance>=p_.neighbor_max)continue;
      const Eigen::Vector3d extra=p_.overshoot*direction.normalized();
      a=state.head<3>()+p_.center_offset-extra;
      Eigen::Vector3d end=v->state.head<3>()+p_.center_offset;
      if(v->id!=0)end+=extra;
      if(map_->path(a,end,p_.robot_size)==VoxelStatus::Free)graph_.addEdge(added,v,distance);
      else{++r.collision_rejections;if(diagnostics)diagnostics->failedEdge(a,end);}
    }
  }
  return true;
}
bool expandGround(GraphManager& graph,const std::shared_ptr<const MapQuery>& map,
                  const Parameters& p,StateVec state,PlanResult& result,int max_edges,
                  GroundEdgeCache* edges,bool allow_hanging,PlanningDiagnostics* diagnostics){
  TerrainProjector terrain(map,p.groundParameters());
  return expandGroundWithTerrain(graph,map,p,state,result,max_edges,edges,allow_hanging,diagnostics,terrain);
}
namespace {
bool expandGroundWithTerrain(GraphManager& graph,const std::shared_ptr<const MapQuery>& map,
                  const Parameters& p,StateVec state,PlanResult& result,int max_edges,
                  GroundEdgeCache* edges,bool allow_hanging,PlanningDiagnostics* diagnostics,
                  const TerrainProjector& terrain){
  auto projection=terrain.project(state);
  const auto allowed=[&](const GroundProjection& point){
    // Supported includes project()'s body check under these same parameters.
    // Only hanging projections still need their observed-free body checked.
    return point.status==GroundStatus::Supported||
      (allow_hanging&&point.status==GroundStatus::Hanging&&
       map->box(terrain.baseState(point.projected_state).head<3>()+p.center_offset,p.robot_size)==VoxelStatus::Free);
  };
  if(!allowed(projection)){
    if(diagnostics&&(projection.support||projection.hanging))diagnostics->sample(projection.projected_state,false);
    ++result.collision_rejections;return false;
  }
  if(diagnostics)diagnostics->sample(projection.projected_state,true);
  state=projection.projected_state;Vertex* nearest=nullptr;
  if(!graph.getNearestVertex(&state,&nearest))return false;
  Eigen::Vector3d direction=(state-nearest->state).head<3>();double distance=direction.norm();
  if(distance<=p.edge_min)return false;
  if(distance>p.edge_max)state.head<3>()=nearest->state.head<3>()+p.edge_max*direction/distance;
  projection=terrain.project(state);
  if(!allowed(projection)){++result.collision_rejections;return false;}
  // Recompute after steering; do not preserve the original sample's stale
  // hanging label when the shortened endpoint actually has observed support.
  const bool hanging=projection.hanging;
  state=projection.projected_state;direction=(state-nearest->state).head<3>();distance=direction.norm();
  if(distance<=p.edge_min)return false;
  const auto usable=[&](const GroundEdge& edge){return allow_hanging?edge.candidateAdmissible():edge.admissible();};
  auto* capture=diagnostics&&diagnostics->options.ground_edges?&diagnostics->ground_edges:nullptr;
  // Observe the existing directed admission checks. This is not the upstream
  // evaluateGraph negative-slope rescoring pass and does not perform map queries.
  auto checked=[&](const StateVec& from,const StateVec& to,bool from_root,bool to_root,
                   bool endpoint_hanging,GroundEdgeRole role,GroundEdgeDirection direction){
    const auto check_id=capture?capture->beginCheck():0;
    const auto permission=allow_hanging&&endpoint_hanging?
        GroundHangingPermission::OriginalEndpointHanging:GroundHangingPermission::Forbid;
    GroundEdge edge=terrain.candidateEdge(from,to,permission);
    if(capture)capture->record(check_id,role,direction,false,from,to,from,to,edge,usable(edge));
    if(!usable(edge)){
      if(diagnostics)diagnostics->failedEdge(from.head<3>(),to.head<3>());
      if(capture)capture->finishCheck();
      return edge;
    }
    const Eigen::Vector3d delta=(to-from).head<3>();
    if(p.overshoot>0&&delta.norm()>1e-10){
      const Eigen::Vector3d extra=p.overshoot*delta.normalized();StateVec a=from,b=to;
      if(!from_root)a.head<3>()-=extra;
      if(!to_root)b.head<3>()+=extra;
      // The virtual extension is checked with the original graph pair's
      // permission; encountering a new hanging endpoint cannot relax it.
      const auto extended=terrain.candidateEdge(a,b,permission);
      if(capture)capture->record(check_id,role,direction,true,from,to,a,b,extended,usable(extended));
      if(!usable(extended)){
        edge.status=extended.status;edge.speculative=false;
        if(diagnostics)diagnostics->failedEdge(a.head<3>(),b.head<3>());
      }
      else if(extended.speculative){edge.status=GroundStatus::Hanging;edge.speculative=true;}
    }
    if(capture)capture->finishCheck();
    return edge;
  };
  const bool parent_hanging=nearest->is_hanging||hanging;
  const auto forward=checked(nearest->state,state,nearest->id==0,false,parent_hanging,
      GroundEdgeRole::Parent,GroundEdgeDirection::Forward);
  if(!usable(forward)){++result.collision_rejections;return false;}
  auto vertex=std::make_unique<Vertex>(graph.generateVertexID(),state);
  vertex->parent=nearest;vertex->distance=nearest->distance+forward.actual_length;vertex->is_hanging=hanging;
  auto* added=vertex.get();graph.addVertex(added);vertex.release();nearest->children.push_back(added);
  graph.addEdge(nearest,added,forward.actual_length);
  if(diagnostics)diagnostics->projectedEdge(forward);
  if(hanging)++result.hanging_vertices;
  if(edges){
    (*edges)[{nearest->id,added->id}]=forward;
    (*edges)[{added->id,nearest->id}]=checked(state,nearest->state,false,nearest->id==0,parent_hanging,
        GroundEdgeRole::Parent,GroundEdgeDirection::Reverse);
  }
  if(p.graph_mode){
    std::vector<Vertex*> neighbors;graph.getNearestVertices(&state,p.neighbor_radius,&neighbors);
    std::sort(neighbors.begin(),neighbors.end(),[](auto* a,auto* b){return a->id<b->id;});
    for(auto* neighbor:neighbors){
      if(neighbor==added||neighbor==nearest||graph.getNumEdges()>=max_edges)continue;
      const double distance=(neighbor->state-state).head<3>().norm();
      if(distance<=p.neighbor_min||distance>=p.neighbor_max)continue;
      const bool neighbor_hanging=added->is_hanging||neighbor->is_hanging;
      const auto outward=checked(state,neighbor->state,false,neighbor->id==0,neighbor_hanging,
          GroundEdgeRole::Neighbor,GroundEdgeDirection::Forward);
      const auto inward=checked(neighbor->state,state,neighbor->id==0,false,neighbor_hanging,
          GroundEdgeRole::Neighbor,GroundEdgeDirection::Reverse);
      if(!usable(outward)&&!usable(inward)){++result.collision_rejections;continue;}
      // GraphManager stores an undirected adjacency relation. Each actual
      // route uses the oriented edge cache and its own terrain length instead.
      graph.addEdge(added,neighbor,usable(outward)?outward.actual_length:inward.actual_length);
      if(diagnostics)diagnostics->projectedEdge(usable(outward)?outward:inward);
      if(edges){(*edges)[{added->id,neighbor->id}]=outward;(*edges)[{neighbor->id,added->id}]=inward;}
    }
  }
  return true;
}
}  // namespace

void LocalPlanner::groundShortestPaths(ShortestPathsReport& report){
  report=ShortestPathsReport();report.source_id=0;
  for(const auto& item:graph_.vertices_map_){
    report.parent_id_map[item.first]=item.first;
    report.distance_map[item.first]=std::numeric_limits<double>::max();
  }
  using Entry=std::pair<double,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> pending;
  report.distance_map[0]=0.;pending.push({0.,0});
  while(!pending.empty()){
    const auto [distance,id]=pending.top();pending.pop();
    if(distance>report.distance_map[id])continue;
    const auto found=graph_.edge_map_.find(id);if(found==graph_.edge_map_.end())continue;
    for(const auto& neighbor:found->second){
      const auto key=std::make_pair(id,neighbor.first);auto edge=ground_edges_.find(key);
      if(edge==ground_edges_.end()){
        const auto* from=graph_.getVertex(id);const auto* to=graph_.getVertex(neighbor.first);
        const auto permission=from->is_hanging||to->is_hanging?
            GroundHangingPermission::OriginalEndpointHanging:GroundHangingPermission::Forbid;
        edge=ground_edges_.emplace(key,terrain_->candidateEdge(from->state,to->state,permission)).first;
      }
      if(!edge->second.candidateAdmissible())continue;
      const double candidate=distance+edge->second.actual_length;
      if(candidate<report.distance_map[neighbor.first]){
        report.distance_map[neighbor.first]=candidate;report.parent_id_map[neighbor.first]=id;
        pending.push({candidate,neighbor.first});
      }
    }
  }
  report.status=true;
}

std::vector<StateVec> LocalPlanner::groundPath(int target,const ShortestPathsReport& report,
                                             bool executable_only,int* executed_target){
  std::vector<int> ids;graph_.getShortestPath(target,report,true,ids);std::vector<StateVec> path;
  if(ids.empty())return path;
  path.push_back(graph_.getVertex(ids.front())->state);
  if(executed_target)*executed_target=ids.front();
  for(std::size_t i=1;i<ids.size();++i){
    const auto edge=ground_edges_.find({ids[i-1],ids[i]});
    if(edge==ground_edges_.end()||!edge->second.candidateAdmissible())return {};
    if(executable_only&&(graph_.getVertex(ids[i])->is_hanging||!edge->second.admissible()))break;
    const auto& segment=edge->second.polyline;
    for(std::size_t j=1;j<segment.size();++j){
      StateVec point=segment[j];
      if(!p_.tangent_yaw)point[3]=graph_.getVertex(ids[i])->state[3];
      path.push_back(point);
    }
    if(executed_target)*executed_target=ids[i];
  }
  return path;
}

std::vector<StateVec> LocalPlanner::executablePath(int target,const ShortestPathsReport& report,
                                                  PlanResult& result){
  std::vector<int> ids;graph_.getShortestPath(target,report,true,ids);
  std::vector<StateVec> path;if(ids.empty())return path;
  path.push_back(graph_.getVertex(ids.front())->state);result.executed_target_id=ids.front();
  for(std::size_t i=1;i<ids.size();++i){
    const auto* next=graph_.getVertex(ids[i]);double distance=0.;
    if(terrain_){
      const auto edge=ground_edges_.find({ids[i-1],ids[i]});
      if(edge==ground_edges_.end()||!edge->second.candidateAdmissible())return {};
      if(next->is_hanging||!edge->second.admissible()){
        result.truncated_for_hanging=true;break;
      }
      // A projected edge can have many terrain knots. Its full polyline and
      // actual arc length form one graph edge, never several budget steps.
      const auto& segment=edge->second.polyline;
      for(std::size_t j=1;j<segment.size();++j){
        StateVec point=segment[j];if(!p_.tangent_yaw)point[3]=next->state[3];
        path.push_back(point);
      }
      distance=edge->second.actual_length;
    }else{
      distance=(next->state-graph_.getVertex(ids[i-1])->state).head<3>().norm();
      path.push_back(next->state);
    }
    result.executed_target_id=ids[i];
    result.traverse_length+=distance;result.traverse_time+=distance/p_.max_velocity;
    // Upstream tests strict > only after appending the next graph vertex.
    // The crossing edge is retained; the budget is not a clipping distance.
    if(result.traverse_length>p_.traverse_length_max||result.traverse_time>p_.traverse_time_max){
      result.truncated_for_budget=i+1<ids.size();break;
    }
  }
  return path;
}

void LocalPlanner::graphMessage(planner_msgs::msg::Graph& message){
  graph_.convertGraphToMsg(message);
  if(!terrain_)return;
  // Vertex.msg has no hanging flag. Suppress frontier eligibility outside
  // the strictly executable directed component, without changing local gains.
  std::vector<bool> reachable(graph_.vertices_map_.size(),false);std::queue<int> pending;
  if(reachable.empty())return;
  reachable[0]=true;pending.push(0);
  while(!pending.empty()){
    const int id=pending.front();pending.pop();const auto neighbors=graph_.edge_map_.find(id);
    if(neighbors==graph_.edge_map_.end())continue;
    for(const auto& neighbor:neighbors->second){
      if(reachable[neighbor.first]||graph_.getVertex(neighbor.first)->is_hanging)continue;
      const auto cached=ground_edges_.find({id,neighbor.first});
      if(cached==ground_edges_.end()||!cached->second.admissible())continue;
      reachable[neighbor.first]=true;pending.push(neighbor.first);
    }
  }
  for(auto& vertex:message.vertices)if(!reachable[vertex.id])vertex.is_frontier=false;
}
std::optional<double> LocalPlanner::validatedGroundLength(const std::shared_ptr<const MapQuery>& map,
    const GroundParameters& parameters,int from_id,int to_id,
    const StateVec& from,const StateVec& to) const {
  // A shared_ptr<const MapQuery> may alias a mutable map owned elsewhere.
  // Reuse is therefore limited to callers that already opted into the
  // request-scoped immutable snapshot contract, not just pointer equality.
  if(!exact_ray_gain_reuse_||!terrain_||map.get()!=map_.get()||
     !from.allFinite()||!to.allFinite())return std::nullopt;
  const auto& local=terrain_->parameters();
  if(parameters.max_ground_height!=local.max_ground_height||
     parameters.robot_height!=local.robot_height||
     !(parameters.center_offset.array()==local.center_offset.array()).all()||
     !(parameters.robot_size.array()==local.robot_size.array()).all()||
     parameters.point_robot!=local.point_robot||
     parameters.max_inclination!=local.max_inclination||
     parameters.max_negative_inclination!=local.max_negative_inclination||
     parameters.projection_max_length!=local.projection_max_length||
     parameters.hanging_vertex_penalty!=local.hanging_vertex_penalty||
     parameters.interpolate_projection_distance!=local.interpolate_projection_distance||
     parameters.ray_projection_mode!=local.ray_projection_mode||
     parameters.tsdf_projection_mode!=local.tsdf_projection_mode)return std::nullopt;
  const auto source=graph_.vertices_map_.find(from_id),target=graph_.vertices_map_.find(to_id);
  if(source==graph_.vertices_map_.end()||target==graph_.vertices_map_.end()||
     !(source->second->state.head<3>().array()==from.head<3>().array()).all()||
     !(target->second->state.head<3>().array()==to.head<3>().array()).all())return std::nullopt;
  const auto found=ground_edges_.find({from_id,to_id});
  if(found==ground_edges_.end())return std::nullopt;
  const auto& edge=found->second;
  // An Admissible candidate encountered no unknown support. Allowing hanging
  // lookahead therefore did not relax any check on this particular edge.
  // Overshoot failures overwrite status (or mark the edge speculative), so
  // they cannot produce a positive result here. Terrain/body geometry and
  // its length depend on XYZ, not the yaw later corrected for sensor scoring.
  if(!edge.admissible()||edge.speculative||!std::isfinite(edge.actual_length)||
     edge.actual_length<0.)return std::nullopt;
  return edge.actual_length;
}
std::vector<Eigen::Vector3d> LocalPlanner::sensorEndpoints(std::size_t index,const StateVec& state) const {
  if(!state.allFinite())throw std::invalid_argument("Invalid sensor viewpoint");
  const Eigen::Vector3d origin=state.head<3>();
  const Eigen::AngleAxisd body_rotation(state[3],Eigen::Vector3d::UnitZ());
  std::vector<Eigen::Vector3d> endpoints;endpoints.reserve(rays_.at(index).size());
  for(const auto& point:rays_.at(index))endpoints.push_back(origin+body_rotation*point);
  return endpoints;
}
std::vector<SensorGain> LocalPlanner::raySensorGains(const StateVec& state) const {
  return raySensorGains(state,true);
}
std::vector<SensorGain> LocalPlanner::raySensorGains(const StateVec& state,bool exclude_no_gain_zones,
                                                  GainGeometry* capture) const {
  if(!state.allFinite())throw std::invalid_argument("Invalid sensor viewpoint");
  std::vector<SensorGain> output;output.reserve(sensors_.size());
  // Original computeVolumetricGainRayModel starts every ray at the BODY origin,
  // even when endpoints include an offset. min_range is not used in this gain
  // branch. Preserve these semantics instead of silently changing the model.
  const Eigen::Vector3d origin=state.head<3>();
  const bool reuse_invariants=p_.gain.reuse_ray_invariants;
  // The fixed coordinate domain belongs to this map snapshot. Reward policy
  // is still queried at every original double point, independently of the
  // float coordinate used for occupancy below.
  const Eigen::Vector3d minimum=reuse_invariants?map_->minimum():Eigen::Vector3d::Zero();
  const Eigen::Vector3d maximum=reuse_invariants?map_->maximum():Eigen::Vector3d::Zero();
  std::size_t sampled=0;
  for(std::size_t index=0;index<sensors_.size();++index){
    const auto& sensor=sensors_[index];
    SensorGain item;item.name=sensor.name;item.ray_count=rays_[index].size();
    item.ray_method=p_.gain.ray_method;
    item.ray_width=ray_grid_sizes_[index].first;item.ray_height=ray_grid_sizes_[index].second;
    auto& result=item.gain;result.reset();
    SensorGainGeometry* geometry=nullptr;
    if(capture){capture->sensors.emplace_back();geometry=&capture->sensors.back();}
    try{
    const auto account=[&](const Eigen::Vector3d& point,VoxelStatus status,std::size_t weight){
        const bool in_bounds=(point.array()>=(reuse_invariants?minimum:map_->minimum()).array()).all()&&
                             (point.array()<(reuse_invariants?maximum:map_->maximum()).array()).all();
        const bool excluded=exclude_no_gain_zones&&std::any_of(p_.no_gain_zones.begin(),p_.no_gain_zones.end(),
            [&](const auto& zone){return zone.contains(point);});
        if(in_bounds&&map_->gainAllowed(point)&&!excluded){
          if(status==VoxelStatus::Unknown)result.num_unknown_voxels+=weight;
          else if(status==VoxelStatus::Occupied)result.num_occupied_voxels+=weight;
          else if(status==VoxelStatus::Free)result.num_free_voxels+=weight;
          if(geometry)capture->contribution(*geometry,point,status,weight);
        }
        return in_bounds;
    };
    const auto endpoints=sensorEndpoints(index,state);
    if(geometry)for(const auto& endpoint:endpoints)capture->endpoint(*geometry,endpoint);
    if(p_.gain.ray_method=="iterative"){
      item.raycast=iterativeGainRaycast(*map_,origin,endpoints,item.ray_width,item.ray_height,
          sensor.horizontal_resolution,sensor.vertical_resolution,
          {p_.gain.nonuniform_ray_cast,p_.gain.ray_cast_step_size_multiplier,p_.gain.max_ray_samples-sampled,reuse_invariants},
          [&](const Eigen::Vector3d& point,VoxelStatus status,std::size_t weight){account(point,status,weight);});
    }else for(const auto& endpoint:endpoints){
      map_->ray(origin,endpoint,[&](const auto& point,VoxelStatus status){
        if(sampled+item.raycast.weighted_samples>=p_.gain.max_ray_samples)
          throw std::length_error("Gain ray sample budget exceeded");
        ++item.raycast.queries;++item.raycast.weighted_samples;
        // The explicit DDA option preserves the earlier ROS 2 experiment mode.
        // Exclusion removes reward, never occlusion or collision geometry.
        return account(point,status,1)&&status!=VoxelStatus::Occupied;
      });
    }
    if(geometry)geometry->raycast_complete=true;
    sampled+=item.raycast.weighted_samples;
    result.gain=result.num_unknown_voxels*p_.unknown_gain+result.num_free_voxels*p_.free_gain+
                result.num_occupied_voxels*p_.occupied_gain;
    const double full_fov=(sensor.horizontal_fov/sensor.horizontal_resolution)*
        (sensor.vertical_fov/sensor.vertical_resolution)*sensor.range;
    const double resolution=map_->resolution();
    result.is_frontier=result.num_unknown_voxels*resolution/full_fov>=sensor.frontier_threshold;
    if(geometry){
      geometry->map_resolution=resolution;geometry->map_resolution_valid=true;
      geometry->result=item;geometry->complete=true;
    }
    }catch(...){if(geometry)geometry->result=item;throw;}
    output.push_back(std::move(item));
  }
  return output;
}
std::vector<SensorGain> LocalPlanner::voxelSensorGains(const StateVec& state) const {
  return voxelSensorGains(state,nullptr);
}
std::vector<SensorGain> LocalPlanner::voxelSensorGains(const StateVec& state,GainGeometry* capture) const {
  if(!state.allFinite())throw std::invalid_argument("Invalid sensor viewpoint");
  const double step=p_.gain.exp_gain_voxel_size;
  const Eigen::Vector3d origin=state.head<3>();
  const Eigen::Vector3d root=gain_root_valid_?gain_root_:origin;
  // Upstream reads fixed local_space_params_, not local_adaptive_params_.
  // Preserve this axis-aligned ROI even when sampling uses a PCA/rotated box.
  const Eigen::Vector3d local_min=(root+configured_p_.local_min+configured_p_.gain_min_extension).cwiseMax(map_->minimum());
  const Eigen::Vector3d local_max=(root+configured_p_.local_max+configured_p_.gain_max_extension).cwiseMin(map_->maximum());
  std::vector<SensorGain> output;output.reserve(sensors_.size());
  std::size_t scanned=0;
  for(const auto& sensor:sensors_){
    SensorGain item;item.name=sensor.name;item.mode="voxel";item.gain.reset();
    SensorGainGeometry* geometry=nullptr;
    if(capture){capture->sensors.emplace_back();geometry=&capture->sensors.back();}
    try{
    // Every sensor starts from the same local/global ROI. The original code
    // accidentally retained the preceding sensor's range clipping.
    const Eigen::Vector3d minimum=local_min.cwiseMax(origin-Eigen::Vector3d::Constant(sensor.range));
    const Eigen::Vector3d maximum=local_max.cwiseMin(origin+Eigen::Vector3d::Constant(sensor.range));
    if(geometry){geometry->minimum=minimum;geometry->maximum=maximum;geometry->roi_valid=true;geometry->step=step;}
    if((minimum.array()>=maximum.array()).any()){
      if(geometry){geometry->result=item;geometry->complete=true;}
      output.push_back(std::move(item));continue;
    }
    long double cells=1.;
    for(int axis=0;axis<3;++axis){
      if(minimum[axis]+step<=minimum[axis]||maximum[axis]+step<=maximum[axis])
        throw std::length_error("Volumetric gain step cannot advance at this coordinate");
      cells*=std::ceil((static_cast<long double>(maximum[axis])-minimum[axis])/step);
    }
    if(!std::isfinite(cells)||cells>p_.gain.max_voxels-scanned)
      throw std::length_error("Volumetric gain voxel budget exceeded");
    Eigen::Vector3d point;
    for(point.x()=minimum.x();point.x()<maximum.x();point.x()+=step)
      for(point.y()=minimum.y();point.y()<maximum.y();point.y()+=step)
        for(point.z()=minimum.z();point.z()<maximum.z();point.z()+=step){
          if(++scanned>p_.gain.max_voxels)
            throw std::length_error("Volumetric gain voxel budget exceeded");
          ++item.voxel_count;
          if(!sensorContains(sensor,state,point))continue;
          ++item.fov_voxel_count;
          const bool excluded=std::any_of(p_.no_gain_zones.begin(),p_.no_gain_zones.end(),
              [&](const auto& zone){return zone.contains(point);});
          // Apply the existing global workspace and no-gain policies to both
          // gain modes. Neither policy removes surfaces from visibility rays.
          if(!map_->gainAllowed(point)||excluded)continue;
          ++item.visibility_queries;
          if(geometry)capture->endpoint(*geometry,point);
          VoxelStatus ray_status=VoxelStatus::Free;
          map_->ray(origin,point,[&](const auto&,VoxelStatus status){
            // Original getRayStatus(...,true) stops at UNKNOWN as well as
            // occupied. Unknown does not reject this candidate: its endpoint
            // is still queried even if an obstacle lies beyond that unknown.
            if(status==VoxelStatus::Unknown||status==VoxelStatus::Occupied){
              ray_status=status;return false;
            }
            return true;
          });
          if(ray_status==VoxelStatus::Occupied)continue;
          const auto status=map_->voxel(point);
          if(status==VoxelStatus::Unknown)++item.gain.num_unknown_voxels;
          else if(status==VoxelStatus::Free)++item.gain.num_free_voxels;
          else if(status==VoxelStatus::Occupied)++item.gain.num_occupied_voxels;
          if(geometry)capture->contribution(*geometry,point,status,1);
        }
    item.gain.gain=item.gain.num_unknown_voxels*p_.unknown_gain+
        item.gain.num_free_voxels*p_.free_gain+item.gain.num_occupied_voxels*p_.occupied_gain;
    // The non-ray upstream branch never classifies frontiers.
    if(geometry){geometry->result=item;geometry->complete=true;}
    }catch(...){if(geometry)geometry->result=item;throw;}
    output.push_back(std::move(item));
  }
  return output;
}
std::vector<SensorGain> LocalPlanner::sensorGains(const StateVec& state) const {
  return sensorGains(state,nullptr);
}
std::vector<SensorGain> LocalPlanner::sensorGains(const StateVec& state,GainGeometry* capture) const {
  return p_.gain.use_ray_model?raySensorGains(state,true,capture):voxelSensorGains(state,capture);
}
VolumetricGain LocalPlanner::sumSensorGains(const std::vector<SensorGain>& gains) const {
  // Upstream sums sensor contributions, including overlaps, but evaluates each
  // frontier threshold separately and ORs the answers. Do not dilute a narrow
  // frontier-facing sensor by averaging it with a wide sensor looking elsewhere.
  VolumetricGain total;total.reset();
  for(const auto& item:gains){
    total.num_unknown_voxels+=item.gain.num_unknown_voxels;
    total.num_free_voxels+=item.gain.num_free_voxels;
    total.num_occupied_voxels+=item.gain.num_occupied_voxels;
    total.gain+=item.gain.gain;total.is_frontier=total.is_frontier||item.gain.is_frontier;
  }
  return total;
}
VolumetricGain LocalPlanner::gain(const StateVec& state) const {
  return sumSensorGains(sensorGains(state));
}
LocalPlanner::ExactRayGainKey LocalPlanner::exactRayGainKey(const StateVec& state) {
  static_assert(sizeof(double)==sizeof(std::uint64_t),"Exact gain keys require 64-bit doubles");
  ExactRayGainKey key;
  for(std::size_t i=0;i<key.size();++i)
    std::memcpy(&key[i],state.data()+i,sizeof(double));
  return key;
}
void LocalPlanner::rememberExactRayGain(const StateVec& state,const VolumetricGain& gain,
                                        bool exclude_no_gain_zones) const {
  if(!exact_ray_gain_reuse_)return;
  ++exact_ray_gain_evaluations_;
  auto& cache=exclude_no_gain_zones||p_.no_gain_zones.empty()?exact_ray_gains_:exact_global_ray_gains_;
  if(exact_ray_gains_.size()+exact_global_ray_gains_.size()<kMaxExactRayGains)
    cache.emplace(exactRayGainKey(state),gain);
}
VolumetricGain LocalPlanner::rayGain(const StateVec& state) const {
  return cachedRayGain(state,true);
}
VolumetricGain LocalPlanner::globalRayGain(const StateVec& state) const {
  return cachedRayGain(state,false);
}
VolumetricGain LocalPlanner::cachedRayGain(const StateVec& state,bool exclude_no_gain_zones) const {
  if(!state.allFinite())throw std::invalid_argument("Invalid sensor viewpoint");
  auto& cache=exclude_no_gain_zones||p_.no_gain_zones.empty()?exact_ray_gains_:exact_global_ray_gains_;
  if(exact_ray_gain_reuse_){
    const auto found=cache.find(exactRayGainKey(state));
    if(found!=cache.end()){
      ++exact_ray_gain_hits_;return found->second;
    }
  }
  // Store only a completed exact evaluation. Graph serialization can alter
  // yaw by an ULP; bitwise pose keys deliberately treat that as a cache miss.
  const auto value=sumSensorGains(raySensorGains(state,exclude_no_gain_zones));
  rememberExactRayGain(state,value,exclude_no_gain_zones);
  return value;
}
void LocalPlanner::computeExplorationGain(PlanResult& result){
  const int count=graph_.getNumVertices();
  auto* geometry=diagnostics_.options.gains?&diagnostics_.gains:nullptr;
  if(geometry){
    geometry->entered=true;geometry->reason="pending";
    if(geometry->requested_id>=0&&geometry->requested_id<count){
      geometry->vertex_id=geometry->requested_id;
      geometry->state=graph_.getVertex(geometry->vertex_id)->state;geometry->state_valid=true;
    }else if(geometry->requested_id!=-1)geometry->reason="target_missing";
  }
  const auto skipped=[&](int id){
    if(geometry&&geometry->requested_id==id)geometry->reason="leaf_only_skipped";
  };
  const auto evaluate=[&](int id){
    const auto& state=graph_.getVertex(id)->state;
    GainGeometry* capture=nullptr;
    if(geometry&&(geometry->requested_id==id||(geometry->requested_id==-1&&!geometry->evaluated))){
      capture=geometry;capture->vertex_id=id;capture->source_id=id;
      capture->state=state;capture->state_valid=true;capture->evaluated=true;capture->reason="evaluating";
    }
    try{
    const auto gains=sensorGains(state,capture);
    for(const auto& sensor:gains){
      result.gain_voxel_count+=sensor.voxel_count;
      result.gain_fov_voxel_count+=sensor.fov_voxel_count;
      result.gain_visibility_queries+=sensor.visibility_queries;
    }
    const auto value=sumSensorGains(gains);
    // Only this evaluator has visited the actual sensor rays. A clustered
    // neighbor's copied vol_gain is approximate and never enters this cache.
    // Voxel gain also depends on the planning ROI, so it is not cached here.
    if(p_.gain.use_ray_model)rememberExactRayGain(state,value);
    if(capture){
      capture->gain=value;capture->gain_valid=true;capture->reason="evaluated";
      capture->complete=!capture->truncated;
    }
    return value;
    }catch(...){
      if(capture){capture->reason="exception";capture->partial=true;capture->complete=false;}
      throw;
    }
  };
  try{
  if(!p_.cluster_vertices_for_gain){
    // Keep the existing ascending-ID evaluation order when clustering is off.
    for(int id=0;id<count;++id){
      auto* vertex=graph_.getVertex(id);
      if(!p_.leaf_gain_only||vertex->is_leaf_vertex){
        vertex->vol_gain=evaluate(id);++result.gain_evaluations;
      }else skipped(id);
    }
  }else{
    // Upstream pushes leaf IDs to the front while visiting ascending IDs:
    // evaluate leaves in reverse ID order, then non-leaves in ascending order.
    std::vector<int> order;order.reserve(count);
    for(int id=count-1;id>=0;--id)if(graph_.getVertex(id)->is_leaf_vertex)order.push_back(id);
    for(int id=0;id<count;++id)if(!graph_.getVertex(id)->is_leaf_vertex)order.push_back(id);
    std::vector<bool> assigned(count,false);
    for(int id:order){
      if(assigned[id])continue;
      assigned[id]=true;auto* vertex=graph_.getVertex(id);
      if(p_.leaf_gain_only&&!vertex->is_leaf_vertex){skipped(id);continue;}
      vertex->vol_gain=evaluate(id);++result.gain_evaluations;
      std::vector<Vertex*> neighbors;
      graph_.getNearestVertices(&vertex->state,p_.clustering_radius,&neighbors);
      for(auto* neighbor:neighbors){
        if(assigned[neighbor->id])continue;
        // Match upstream: a leaf representative may also supply nearby
        // non-leaf gains when leaf_gain_only is enabled. No yaw-based split.
        neighbor->vol_gain=vertex->vol_gain;assigned[neighbor->id]=true;++result.reused_gains;
        if(geometry&&geometry->requested_id==neighbor->id){
          geometry->reason="clustered_copy";geometry->source_id=id;
          geometry->gain=neighbor->vol_gain;geometry->gain_valid=true;
        }
      }
    }
  }
  }catch(...){
    // Failure at another viewpoint can leave the requested point unvisited.
    // A previously finished selected viewpoint remains a valid snapshot.
    if(geometry&&geometry->reason=="pending"){
      geometry->reason="exception";geometry->partial=true;
    }
    throw;
  }
  if(geometry&&geometry->reason=="pending")geometry->reason="no_evaluation";
  // Upstream marks only cluster representatives. Mark copied frontier gains
  // too so the exported graph agrees with every vertex's frontier flag.
  for(int id=0;id<count;++id){
    auto* vertex=graph_.getVertex(id);
    if(vertex->vol_gain.is_frontier)vertex->type=VertexType::kFrontier;
  }
}

PlanResult LocalPlanner::plan(const StateVec& root,double exploring_direction) {
  PlanResult r;graph_.reset();ground_edges_.clear();
  if(terrain_)terrain_->clearQueryReuse();
  exact_ray_gains_.clear();exact_global_ray_gains_.clear();
  exact_ray_gain_hits_=0;exact_ray_gain_evaluations_=0;
  diagnostics_.reset();
  gain_root_valid_=false;
  r.sampling_root=root;
  // Adaptive geometry belongs to one planning call. Always restore the
  // configured sampler before inspecting the new root or the current map.
  p_=configured_p_;world_to_sampling_=p_.worldToSampling();
  r.gain_mode=p_.gain.use_ray_model?"ray":"voxel";
  if(diagnostics_.options.gains){
    diagnostics_.gains.mode=r.gain_mode;
    diagnostics_.gains.policy="local_workspace_no_gain_zones";
  }
  r.gain_voxel_size=p_.gain.exp_gain_voxel_size;
  const auto use_configured_geometry=[&]{
    r.adaptive.min=p_.local_min;r.adaptive.max=p_.local_max;
    r.adaptive.rotations=p_.local_rotations;r.adaptive.rotation=world_to_sampling_;
    r.adaptive.mean=p_.sample_mean;r.adaptive.scale=p_.sample_scale;
  };
  r.adaptive.reason=p_.adaptive.enabled?"not_evaluated":"disabled";
  use_configured_geometry();
  if(!root.allFinite()){r.reason="invalid_root";return r;}
  if(!std::isfinite(exploring_direction)){r.reason="invalid_exploring_direction";return r;}
  r.exploring_direction=exploring_direction;
  StateVec planning_root=root;
  if(terrain_){
    const auto projection=terrain_->project(root);
    if(projection.status!=GroundStatus::Supported||
       (projection.projected_state-root).head<2>().norm()>1e-9){
      r.reason=std::string("ground_root_")+groundStatusName(projection.status);return r;
    }
    planning_root=projection.projected_state;
  }else if(map_->box(root.head<3>()+p_.center_offset,p_.robot_size)!=VoxelStatus::Free){
    r.reason="root_not_free";return r;
  }
  r.sampling_root=planning_root;
  gain_root_=planning_root.head<3>();gain_root_valid_=true;
  auto start=std::chrono::steady_clock::now();
  if(p_.adaptive.enabled){
    r.adaptive=AdaptiveBound(p_.adaptive).fromMap(*map_,planning_root.head<3>());
    if(r.adaptive.applied){
      // The PCA box replaces the configured box, as in Rrg::buildGraph.
      // Both the sampler and the clipping test use this same orientation.
      p_.local_min=r.adaptive.min;p_.local_max=r.adaptive.max;
      p_.local_rotations=r.adaptive.rotations;
      p_.sample_mean=r.adaptive.mean;p_.sample_scale=r.adaptive.scale;
      // These are the per-call parameter copies. Replace explicit XYZ ranges
      // and moments too; the next call starts again from configured_p_.
      applyAdaptiveSamplingGeometry(p_.axis_sampling,r.adaptive.min,r.adaptive.max,
                                    r.adaptive.mean,r.adaptive.scale);
      applyAdaptiveSamplingGeometry(p_.adaptive_axis_sampling,r.adaptive.min,r.adaptive.max,
                                    r.adaptive.mean,r.adaptive.scale);
      // The original caller applies Euler angles through setRotation(). Use
      // that reconstructed matrix for sampling, clipping and diagnostics alike.
      world_to_sampling_=p_.worldToSampling();r.adaptive.rotation=world_to_sampling_;
    }else use_configured_geometry();
  }
  graph_.addVertex(new Vertex(graph_.generateVertexID(),planning_root));
  LocalSampler sampler(p_,p_.samplingAxes(p_.adaptive.enabled));
  constexpr int kSampleVertexDrawLimit=1000;
  int draws_in_sampling_round=0;
  const bool round_budget=p_.sampling_budget_mode=="sampling_rounds";
  const auto sampling_budget_available=[&]{
    // Once started, a sampleVertex call may consume all of its 1000 draws.
    // max_attempts is deliberately inactive in upstream round-budget mode.
    return round_budget?(draws_in_sampling_round>0||r.sampling_rounds<p_.max_sampling_rounds):
                        r.attempts<p_.max_attempts;
  };
  while(sampling_budget_available() && graph_.getNumVertices()<p_.max_vertices && graph_.getNumEdges()<p_.max_edges){
    // An entirely rejected group starts a new round but, as upstream's
    // continue, skips the cutoff check. Accepted draws end the current call.
    if(draws_in_sampling_round==0)++r.sampling_rounds;
    ++r.attempts;
    if(++draws_in_sampling_round==kSampleVertexDrawLimit)draws_in_sampling_round=0;
    StateVec state=sampler.generate(planning_root);
    if(!inside(state,planning_root))continue;
    if(terrain_){
      const auto projection=terrain_->project(state);
      if(projection.status!=GroundStatus::Supported&&projection.status!=GroundStatus::Hanging){
        // Ground body failures already carry the projected state. A missing
        // support ray is not a body collision sample in the upstream display.
        if(projection.support)diagnostics_.sample(projection.projected_state,false);
        ++r.collision_rejections;continue;
      }
      state=projection.projected_state;if(!inside(state,planning_root))continue;
      // project() classifies unknown support without checking the body. As in
      // sampleVertex, a hanging candidate still needs an observed-free body
      // before this retry batch may finish; expandGround checks the edge later.
      if(projection.status==GroundStatus::Hanging&&
         map_->box(terrain_->baseState(state).head<3>()+p_.center_offset,p_.robot_size)!=VoxelStatus::Free){
        diagnostics_.sample(state,false);++r.collision_rejections;continue;
      }
    }else if(map_->box(state.head<3>()+p_.center_offset,p_.robot_size)!=VoxelStatus::Free){
      diagnostics_.sample(state,false);++r.collision_rejections;continue;
    }
    // Valid means sampleVertex accepted the state, even if steering or edge
    // validation subsequently prevents adding it to the graph.
    if(!terrain_)diagnostics_.sample(state,true);
    draws_in_sampling_round=0;
    expand(state,r);
    // Rrg::buildGraph checks only AFTER expandGraph: even a first accepted
    // sample above the cutoff gets its chance to establish the first edge.
    // Once the graph grows, this is not a consecutive-failure termination rule.
    if(r.sampling_rounds>=p_.empty_graph_cutoff&&graph_.getNumVertices()<=1){
      r.empty_graph_cutoff_reached=true;break;
    }
  }
  if(r.empty_graph_cutoff_reached)r.sampling_stop_reason="empty_graph_cutoff";
  else if(graph_.getNumVertices()>=p_.max_vertices)r.sampling_stop_reason="vertex_limit";
  else if(graph_.getNumEdges()>=p_.max_edges)r.sampling_stop_reason="edge_limit";
  else r.sampling_stop_reason=round_budget?"sampling_round_limit":"raw_draw_limit";
  r.graph_build_completed=true;
  auto built=std::chrono::steady_clock::now();
  r.build_ms=std::chrono::duration<double,std::milli>(built-start).count();
  r.vertices=graph_.getNumVertices();r.edges=graph_.getNumEdges();
  if(r.vertices<2){r.reason="no_feasible_path";return r;}
  // Assign before returning the value: a scope guard could update the local
  // object after its returned copy was made on a non-NRVO early return.
  const auto finishEvaluation=[&]{
    r.evaluate_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-built).count();
  };
  ShortestPathsReport report;
  if(terrain_)groundShortestPaths(report);else graph_.findShortestPaths(report);
  graph_.findLeafVertices(report);
  if(diagnostics_.options.paths)diagnostics_.shortest_vertices.reserve(r.vertices);
  for(int id=0;id<r.vertices;++id) {
    auto* v=graph_.getVertex(id);
    const int parent_id=graph_.getParentIDFromShortestPath(id,report);
    if(id && parent_id!=id && p_.tangent_yaw){
      const auto* parent=graph_.getVertex(parent_id);
      // Correct yaw before sensor gain evaluation, as in Rrg::correctYaw.
      // Reversing only the final output would score the wrong observation FOV.
      applySegmentHeading(parent->state,v->state,p_.refinement.planning_backward);
    }
    if(diagnostics_.options.paths&&report.status){
      const auto distance=report.distance_map.find(id);
      const auto parent_distance=report.distance_map.find(parent_id);
      // Disconnected vertices retain themselves as parent and max distance.
      // Read the report already used for scoring; never rebuild a display tree.
      if(distance!=report.distance_map.end()&&std::isfinite(distance->second)&&
         distance->second<std::numeric_limits<double>::max()&&
         parent_distance!=report.distance_map.end()&&std::isfinite(parent_distance->second)&&
         parent_distance->second<std::numeric_limits<double>::max()&&
         graph_.vertices_map_.count(parent_id)&&
         (id==report.source_id||parent_id!=id))
        diagnostics_.shortest_vertices.push_back({id,parent_id,v->state,v->is_leaf_vertex});
    }
  }
  const auto gain_started=std::chrono::steady_clock::now();
  computeExplorationGain(r);
  r.gain_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-gain_started).count();
  r.gain_evaluated=true;
  // Rrg::evaluateGraph: accumulate distance-discounted viewpoint gains, then
  // apply the trajectory/direction penalty once to the whole root-to-leaf path.
  // Root gain and possible overlap between viewpoints match the original rule.
  for(int id=1;id<r.vertices;++id) {
    auto* v=graph_.getVertex(id);if(!v->is_leaf_vertex)continue;
    std::vector<Vertex*> path;graph_.getShortestPath(id,report,true,path);
    if(path.size()<2)continue;
    double value=0.,before_hanging=0.;int hanging_count=0;
    for(auto* point:path){
      if(point->vol_gain.is_frontier&&!point->is_hanging)r.frontier_exists=true;
      const double contribution=point->vol_gain.gain*std::exp(-p_.path_length_penalty*graph_.getShortestDistance(point->id,report));
      before_hanging+=contribution;
      const bool hanging=terrain_&&point->is_hanging;if(hanging)++hanging_count;
      value+=contribution*std::exp(-(hanging?p_.ground.hanging_vertex_penalty:0.));
      point->vol_gain.accumulative_gain=value;
    }
    const double unpenalized=value;double direction_distance=0.,direction_factor=1.;
    if(p_.path_direction_penalty>0){
      Trajectory::PathType positions;positions.reserve(path.size());
      if(terrain_){
        const auto projected=groundPath(id,report);if(projected.size()<2)continue;
        for(const auto& point:projected)positions.push_back(point.head<3>());
      }else for(const auto* point:path)positions.push_back(point->state.head<3>());
      direction_distance=Trajectory::computeDistanceBetweenTrajectoryAndDirection(
          positions,exploring_direction,.2,true);
      direction_factor=std::exp(-p_.path_direction_penalty*direction_distance);
      value*=direction_factor;
    }
    if(value>r.gain){
      r.gain=value;r.best_id=id;r.unpenalized_gain=unpenalized;
      r.direction_distance=direction_distance;r.direction_factor=direction_factor;
      r.gain_before_hanging_penalty=before_hanging;r.candidate_hanging_vertices=hanging_count;
    }
  }
  if(r.best_id>=0) {
    std::vector<StateVec> candidate_path;
    r.candidate_path_length=graph_.getShortestDistance(r.best_id,report);
    // This gate applies to the complete selected candidate before either
    // hanging-prefix extraction or traversal limits, as in Rrg::getBestPath.
    constexpr double minimum_path_length=.3;
    // A rejected short candidate is still a scored planning decision. Capture
    // it only when requested; the disabled branch keeps the original gate cost.
    if(diagnostics_.options.paths||r.candidate_path_length>minimum_path_length){
      if(terrain_)candidate_path=groundPath(r.best_id,report);
      else graph_.getShortestPath(r.best_id,report,true,candidate_path);
    }
    if(diagnostics_.options.paths){
      diagnostics_.best_path=candidate_path;
      diagnostics_.best_id=r.best_id;diagnostics_.best_gain=r.gain;
    }
    if(r.candidate_path_length<=minimum_path_length){r.reason="best_path_too_short";finishEvaluation();return r;}
    r.path=executablePath(r.best_id,report,r);r.path_length=r.traverse_length;
    if(r.path.size()<2){r.path.clear();r.path_length=0.;r.reason="no_ground_attached_path";finishEvaluation();return r;}
    if(p_.path_direction_penalty==0){
      Trajectory::PathType positions;positions.reserve(candidate_path.size());
      for(const auto& state:candidate_path)positions.push_back(state.head<3>());
      r.direction_distance=Trajectory::computeDistanceBetweenTrajectoryAndDirection(
          positions,exploring_direction,.2,true);
    }
    r.success=true;r.reason=r.truncated_for_hanging?"ok_hanging_prefix":
        (r.truncated_for_budget?"ok_traverse_prefix":"ok");
    // Ground edge caches contain forward tangents from terrain projection.
    // Apply the selected policy to every output knot even without refinement.
    applyPathHeading(r.path,p_.tangent_yaw,p_.refinement.planning_backward);
    // Rrg::getBestPath refines only the selected executable prefix. Preserve
    // the sampled graph and gain score, while publishing its actual new length.
    const auto refinement_started=std::chrono::steady_clock::now();
    r.refinement=refinePlannedPath(map_,p_,r.path,r.path_length,false,false);
    r.refinement_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-refinement_started).count();
  } else r.reason="no_gain";
  finishEvaluation();
  return r;
}
}  // namespace gbplanner2
