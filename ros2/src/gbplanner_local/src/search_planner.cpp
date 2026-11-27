// Adapted from Rrg::search/findPathToConnect, GBPlanner 2, BSD-3-Clause.
#include "gbplanner_local/search_planner.hpp"
#include "gbplanner_local/path_heading.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace gbplanner2 {
void SearchParameters::validate() const {
  if(max_vertices<2||max_vertices>20000||max_edges<1||max_edges>100000||
     max_attempts<1||max_attempts>1000000||empty_graph_cutoff<1||
     max_target_candidates<0||max_target_candidates>20000||
     !std::isfinite(target_radius)||target_radius<=0.)
    throw std::invalid_argument("Invalid search parameters");
}

SearchPlanner::SearchPlanner(std::shared_ptr<const MapQuery> map,Parameters parameters,
                             SearchParameters search_parameters)
    :map_(std::move(map)),p_(std::move(parameters)),search_(search_parameters){
  p_.validate();search_.validate();if(!map_)throw std::invalid_argument("Search needs a map");
  world_to_sampling_=p_.worldToSampling();
  if(p_.ground_robot)terrain_=std::make_unique<TerrainProjector>(map_,p_.groundParameters());
}
bool SearchPlanner::project(const StateVec& input,StateVec& output,bool allow_relocation) const {
  if(!input.allFinite())return false;
  output=input;
  if(terrain_){
    const auto projection=terrain_->project(input);
    if(projection.status!=GroundStatus::Supported||
       (!allow_relocation&&(projection.projected_state-input).head<2>().norm()>1e-9))return false;
    output=projection.projected_state;
  }
  const auto body=terrain_?terrain_->baseState(output):output;
  return map_->box(body.head<3>()+p_.center_offset,p_.robot_size)==VoxelStatus::Free;
}
bool SearchPlanner::inside(const StateVec& state,const StateVec& source) const {
  const Eigen::Vector3d local=world_to_sampling_*(state-source).head<3>();
  return local.allFinite()&&(local.array()>=p_.local_min.array()).all()&&
      (local.array()<=p_.local_max.array()).all();
}
bool SearchPlanner::expand(const StateVec& state,PlanResult& counters){
  return terrain_?expandGround(graph_,map_,p_,state,counters,search_.max_edges,&ground_edges_,false)
                 :expandAerial(graph_,map_,p_,state,counters,search_.max_edges);
}
void SearchPlanner::shortestPaths(ShortestPathsReport& report){
  if(!terrain_){graph_.findShortestPaths(report);return;}
  report=ShortestPathsReport{};report.source_id=0;
  for(const auto& item:graph_.vertices_map_){
    report.parent_id_map[item.first]=item.first;
    report.distance_map[item.first]=std::numeric_limits<double>::infinity();
  }
  using Entry=std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> pending;
  report.distance_map[0]=0.;pending.emplace(0.,0);
  while(!pending.empty()){
    const auto [distance,id]=pending.top();pending.pop();
    if(distance>report.distance_map[id])continue;
    const auto neighbors=graph_.edge_map_.find(id);if(neighbors==graph_.edge_map_.end())continue;
    for(const auto& neighbor:neighbors->second){
      auto edge=ground_edges_.find({id,neighbor.first});
      if(edge==ground_edges_.end())edge=ground_edges_.emplace(std::make_pair(id,neighbor.first),
          terrain_->edge(graph_.getVertex(id)->state,graph_.getVertex(neighbor.first)->state)).first;
      if(!edge->second.admissible())continue;
      const double next=distance+edge->second.actual_length;
      if(next<report.distance_map[neighbor.first]){
        report.distance_map[neighbor.first]=next;report.parent_id_map[neighbor.first]=id;
        pending.emplace(next,neighbor.first);
      }
    }
  }
  report.status=true;
}
std::vector<StateVec> SearchPlanner::path(int target,const ShortestPathsReport& report) {
  std::vector<StateVec> output;
  if(!terrain_){
    graph_.getShortestPath(target,report,true,output);
    if(p_.tangent_yaw)for(std::size_t i=1;i<output.size();++i){
      const Eigen::Vector3d delta=(output[i]-output[i-1]).head<3>();
      if(delta.head<2>().norm()>1e-10)output[i][3]=std::atan2(delta.y(),delta.x());
    }
    return output;
  }
  std::vector<int> ids;graph_.getShortestPath(target,report,true,ids);
  if(ids.empty())return output;
  output.push_back(graph_.getVertex(ids.front())->state);
  for(std::size_t i=1;i<ids.size();++i){
    const auto edge=ground_edges_.find({ids[i-1],ids[i]});
    if(edge==ground_edges_.end()||!edge->second.admissible())return {};
    for(std::size_t knot=1;knot<edge->second.polyline.size();++knot){
      auto state=edge->second.polyline[knot];
      if(!p_.tangent_yaw)state[3]=graph_.getVertex(ids[i])->state[3];
      output.push_back(state);
    }
  }
  return output;
}
void SearchPlanner::graphMessage(planner_msgs::msg::Graph& message){graph_.convertGraphToMsg(message);}

SearchResult SearchPlanner::search(const StateVec& input_source,const StateVec& input_target){
  SearchResult result;graph_.reset();ground_edges_.clear();
  const auto started=std::chrono::steady_clock::now();
  const auto finish=[&](const std::string& reason){
    if(result.success&&search_.refine_path){
      // Optional ROS 2 extension: upstream search itself skips this pass.
      // Fix both endpoints so exact/neighborhood target semantics cannot drift.
      result.refinement=refinePlannedPath(map_,p_,result.path,result.path_length,true,true);
      result.endpoint_distance=(result.path.back()-input_target).head<3>().norm();
    }
    result.reason=reason;result.vertices=graph_.getNumVertices();result.edges=graph_.getNumEdges();
    result.build_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    return result;
  };
  if(!input_source.allFinite()||!input_target.allFinite())return finish("invalid_search_pose");
  const auto exact_arrival=[&](const StateVec& state){
    const Eigen::Vector3d delta=(state-input_target).head<3>();
    return terrain_?(delta.head<2>().norm()<1e-8&&std::abs(delta.z())<=map_->resolution()+1e-8)
                   :delta.norm()<1e-8;
  };
  StateVec source=input_source,target=input_target;
  // Ground searches always require actual support, even when the historical
  // optional source occupancy precheck is disabled.
  if(terrain_||search_.check_collision_at_source){
    if(!project(input_source,source))return finish("source_not_free_or_supported");
  }
  const bool target_free=project(input_target,target);
  if(!target_free)target=input_target;
  result.endpoint_distance=(source-input_target).head<3>().norm();
  graph_.addVertex(new Vertex(graph_.generateVertexID(),source));

  // Direct connection precedes random sampling and does not use an edge_max
  // steering limit. Ground paths are dense terrain curves, never a chord.
  GroundEdge direct;
  bool direct_free=false;
  if(target_free){
    if(terrain_){direct=terrain_->edge(source,target);direct_free=direct.admissible();}
    else direct_free=map_->path(source.head<3>()+p_.center_offset,
        target.head<3>()+p_.center_offset,p_.robot_size)==VoxelStatus::Free;
  }
  if(direct_free){
    if((source-target).head<3>().norm()<1e-9){
      result.path={source};result.target_id=0;
    }else{
      auto* root=graph_.getVertex(0);auto* end=new Vertex(graph_.generateVertexID(),target);
      end->parent=root;root->children.push_back(end);graph_.addVertex(end);
      const double length=terrain_?direct.actual_length:(target-source).head<3>().norm();
      graph_.addEdge(root,end,length);end->distance=length;result.target_id=end->id;
      if(terrain_){
        ground_edges_[{0,end->id}]=direct;ground_edges_[{end->id,0}]=terrain_->edge(target,source);
      }
      ShortestPathsReport report;shortestPaths(report);result.path=path(end->id,report);
      result.path_length=length;
    }
    // Upstream's immediate connection always aligns the terminal heading,
    // independently of the tangent-yaw option used by sampled search paths.
    // The shared helper retains yaw when no horizontal tangent exists.
    if(!terrain_)applyPathHeading(result.path,true,false);
    result.success=true;result.exact_target=exact_arrival(result.path.back());
    result.endpoint_distance=(result.path.back()-input_target).head<3>().norm();
    return finish(result.exact_target?"direct":"target_neighborhood");
  }

  LocalSampler sampler(p_);PlanResult counters;std::vector<int> candidates;
  constexpr int kSampleVertexDrawLimit=1000;
  bool stop_sampling=false;
  while(!stop_sampling&&result.attempts<search_.max_attempts&&graph_.getNumVertices()<search_.max_vertices&&
        graph_.getNumEdges()<search_.max_edges){
    StateVec state;bool accepted=false;
    for(int draw=0;draw<kSampleVertexDrawLimit&&result.attempts<search_.max_attempts;++draw){
      ++result.attempts;
      const auto sampled=sampler.generate(source);if(!inside(sampled,source))continue;
      if(!project(sampled,state,true)){++counters.collision_rejections;continue;}
      if(!inside(state,source))continue;
      accepted=true;break;
    }
    // findPathToConnect skips a completely rejected sampleVertex batch before
    // both its cutoff and loop_count++. Keep that distinct from buildGraph;
    // the raw-draw cap above still bounds a permanently blocked workspace.
    if(!accepted)continue;
    if(expand(state,counters)){
      const int id=graph_.getNumVertices()-1;
      // Check the added, steered vertex. The original tested its unsteered
      // sample, which could label a distant vertex as a target neighbor.
      if((graph_.getVertex(id)->state-target).head<3>().norm()<search_.target_radius){
        candidates.push_back(id);
        // Keep the original strict > comparison: default 5 accepts 6 candidates.
        if(static_cast<int>(candidates.size())>search_.max_target_candidates)stop_sampling=true;
      }
    }
    // The accepted sample gets its expansion opportunity before checking the
    // previous completed-round count. cutoff=7 stops after expansion eight,
    // only if the graph is still a lone root; later failures do not stop growth.
    if(result.sampling_rounds>=search_.empty_graph_cutoff&&graph_.getNumVertices()<=1){
      result.empty_graph_cutoff_reached=true;stop_sampling=true;
    }
    ++result.sampling_rounds;
  }
  result.collision_rejections=counters.collision_rejections;
  result.target_candidates=candidates.size();
  if(candidates.empty())return finish("no_feasible_path");

  int exact=-1;
  for(int id:candidates)if((graph_.getVertex(id)->state-target).head<3>().norm()<1e-8){exact=id;break;}
  if(exact<0&&target_free&&graph_.getNumVertices()<search_.max_vertices&&
     graph_.getNumEdges()<search_.max_edges){
    // Retain the original final expansion attempt. Steering may stop short;
    // such a vertex remains a neighborhood candidate, never an exact success.
    if(expand(target,counters)){
      const int id=graph_.getNumVertices()-1;
      const double remaining=(graph_.getVertex(id)->state-target).head<3>().norm();
      if(remaining<1e-8)exact=id;
      else if(remaining<search_.target_radius)candidates.push_back(id);
    }
  }
  result.collision_rejections=counters.collision_rejections;
  ShortestPathsReport report;shortestPaths(report);
  int selected=exact;
  if(selected>=0&&!std::isfinite(graph_.getShortestDistance(selected,report)))selected=-1;
  if(selected<0){
    double best=std::numeric_limits<double>::infinity();
    for(int id:candidates){
      const double distance=graph_.getShortestDistance(id,report);
      if(distance<best||(distance==best&&selected>=0&&id<selected)){best=distance;selected=id;}
    }
  }
  if(selected<0)return finish("target_neighborhood_unreachable");
  result.path=path(selected,report);if(result.path.empty())return finish("search_path_invalidated");
  result.target_id=selected;result.path_length=graph_.getShortestDistance(selected,report);
  result.endpoint_distance=(result.path.back()-input_target).head<3>().norm();
  result.exact_target=exact_arrival(result.path.back());result.success=true;
  return finish(result.exact_target?"exact_target":"target_neighborhood");
}
}  // namespace gbplanner2
