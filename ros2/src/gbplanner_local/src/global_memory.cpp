// Aerial/ground subset of GBPlanner 2 addFrontiers, addRefPathToGraph,
// runGlobalPlanner and searchHomingPath; mission corrections documented in UPSTREAM.md.
// Copyright (c) 2020 UNR Autonomous Robots Lab. BSD-3-Clause.
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/path_heading.hpp"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <queue>

namespace gbplanner2 {
namespace {
bool sameGraphGain(const VolumetricGain& a,const VolumetricGain& b){
  return a.num_unknown_voxels==b.num_unknown_voxels&&
    a.num_free_voxels==b.num_free_voxels&&a.num_occupied_voxels==b.num_occupied_voxels&&
    a.is_frontier==b.is_frontier;
}
}
GlobalMemory::GlobalMemory(Parameters p):p_(p){p_.validate();reset();}
void GlobalMemory::reset(){
  graph_=std::make_shared<GraphManager>();last_odometry_=nullptr;visited_.clear();frontier_update_={};
  last_visited_marker_.setZero();
  restored_history_size_=0;++visit_revision_;
  suspended_edges_.clear();suspended_frontiers_.clear();map_edge_update_={};
  ground_edges_.clear();fixed_ground_edges_.clear();
  background_sampler_=std::make_unique<LocalSampler>(p_);cluster_random_.seed(p_.seed);
  ++snapshot_revision_;
}
bool GlobalMemory::safe(const StateVec& a,const StateVec& b) const {
  return std::isfinite(directedLength(a,b));
}
bool GlobalMemory::projectState(const StateVec& input,StateVec& projected) const {
  if(!map_||!input.allFinite())return false;
  projected=input;
  if(!p_.ground_robot)return safe(input,input);
  if(!terrain_)return false;
  const auto result=terrain_->project(input);
  if(result.status!=GroundStatus::Supported||
     (result.projected_state-input).head<2>().norm()>1e-9)return false;
  projected=result.projected_state;return true;
}
double GlobalMemory::directedLength(const StateVec& a,const StateVec& b,bool preserve_geometry) const {
  const double unavailable=std::numeric_limits<double>::infinity();
  if(!map_||!a.allFinite()||!b.allFinite())return unavailable;
  if(p_.ground_robot){
    if(!terrain_)return unavailable;
    const auto edge=preserve_geometry?terrain_->validateFixedEdge(a,b,true):terrain_->edge(a,b);
    return edge.admissible()?edge.actual_length:unavailable;
  }
  return map_->path(a.head<3>()+p_.center_offset,b.head<3>()+p_.center_offset,p_.robot_size)==VoxelStatus::Free
    ?(b-a).head<3>().norm():unavailable;
}
double GlobalMemory::linkLength(const StateVec& a,const StateVec& b,bool preserve_geometry) const {
  const double forward=directedLength(a,b,preserve_geometry);
  return p_.ground_robot?std::min(forward,directedLength(b,a,preserve_geometry)):forward;
}
bool GlobalMemory::fixedGroundEdge(int from,int to) const {
  return fixed_ground_edges_.count(std::minmax(from,to))!=0;
}
void GlobalMemory::markFixedGroundEdge(int from,int to){
  if(!p_.ground_robot||from==to)return;
  fixed_ground_edges_.insert(std::minmax(from,to));
  ground_edges_.erase({from,to});ground_edges_.erase({to,from});
}
const GroundEdge& GlobalMemory::groundEdge(int from,int to) const {
  const auto key=std::make_pair(from,to);
  auto found=ground_edges_.find(key);
  if(found==ground_edges_.end()){
    auto* a=graph_->getVertex(from);auto* b=graph_->getVertex(to);
    GroundEdge edge;
    if(terrain_&&a&&b)edge=fixedGroundEdge(from,to)?
        terrain_->validateFixedEdge(a->state,b->state,true):terrain_->edge(a->state,b->state);
    found=ground_edges_.emplace(key,std::move(edge)).first;
  }
  return found->second;
}
HomeConnectivityDiagnostics GlobalMemory::inspectHomeConnectivity() const {
  HomeConnectivityDiagnostics result;
  result.ground=p_.ground_robot;result.parameters=p_.groundParameters();
  const auto& graph=*graph_;
  const auto home=graph.vertices_map_.find(0);
  if(home==graph.vertices_map_.end()||!home->second)return result;
  result.present=true;result.home=home->second->state;

  std::map<int,bool> neighbors;
  const auto active=graph.edge_map_.find(0);
  if(active!=graph.edge_map_.end())
    for(const auto& edge:active->second)neighbors[edge.first]=true;
  result.active_degree=neighbors.size();
  for(const auto& edge:suspended_edges_){
    const auto& pair=edge.first;
    if(pair.first==0)neighbors.emplace(pair.second,false);
    else if(pair.second==0)neighbors.emplace(pair.first,false);
  }
  result.edges.reserve(neighbors.size());
  for(const auto& [id,is_active]:neighbors){
    HomeEdgeDiagnostics edge;edge.neighbor=id;edge.active=is_active;
    edge.fixed=fixedGroundEdge(0,id);
    const auto vertex=graph.vertices_map_.find(id);
    if(vertex!=graph.vertices_map_.end()&&vertex->second)edge.state=vertex->second->state;
    result.edges.push_back(std::move(edge));
  }
  if(!map_||!result.ground)return result;

  // Deliberately avoid terrain_/groundEdge(): a diagnostic must not warm the
  // planner's projection/edge caches or insert absent graph entries.
  TerrainProjector terrain(map_,result.parameters);
  result.body=map_->box(terrain.baseState(result.home).head<3>()+
                       result.parameters.center_offset,result.parameters.robot_size);
  result.projected=terrain.project(result.home);
  for(auto& edge:result.edges){
    const auto vertex=graph.vertices_map_.find(edge.neighbor);
    if(vertex==graph.vertices_map_.end()||!vertex->second)continue;
    const auto forward=edge.fixed?terrain.validateFixedEdge(result.home,edge.state,true):
                                  terrain.edge(result.home,edge.state);
    const auto reverse=edge.fixed?terrain.validateFixedEdge(edge.state,result.home,true):
                                  terrain.edge(edge.state,result.home);
    edge.forward=forward.status;edge.reverse=reverse.status;
    edge.forward_length=forward.actual_length;edge.reverse_length=reverse.actual_length;
  }
  return result;
}
bool GlobalMemory::shortestPaths(GraphManager& graph,int source,ShortestPathsReport& report,
                                 bool reverse,const LocalPlanner* producer) const {
  if(!p_.ground_robot)return graph.findShortestPaths(source,report);
  report=ShortestPathsReport{};report.source_id=source;
  if(!terrain_||!graph.getVertex(source))return false;
  const double infinity=std::numeric_limits<double>::infinity();
  for(const auto& item:graph.vertices_map_){
    report.parent_id_map[item.first]=item.first;report.distance_map[item.first]=infinity;
  }
  using QueueEntry=std::pair<double,int>;
  std::priority_queue<QueueEntry,std::vector<QueueEntry>,std::greater<QueueEntry>> queue;
  report.distance_map[source]=0.;queue.emplace(0.,source);
  while(!queue.empty()){
    const auto [distance,id]=queue.top();queue.pop();
    if(distance>report.distance_map[id])continue;
    const auto neighbors=graph.edge_map_.find(id);
    if(neighbors==graph.edge_map_.end())continue;
    for(const auto& neighbor:neighbors->second){
      auto target=report.distance_map.find(neighbor.first);
      // Terrain lengths are nonnegative. Even a zero-length connection cannot
      // improve an equal or shorter label, so no geometry query is needed.
      // Keep the same strict relaxation rule and queue ordering for all edges
      // which can still improve a label, including reversed home searches.
      if(target==report.distance_map.end()||target->second<=distance)continue;
      const int from=reverse?neighbor.first:id,to=reverse?id:neighbor.first;
      double cost=infinity;
      if(&graph==graph_.get()){
        const auto& edge=groundEdge(from,to);if(edge.admissible())cost=edge.actual_length;
      }else{
        auto* a=graph.getVertex(from);auto* b=graph.getVertex(to);
        if(a&&b){
          const auto reused=producer?producer->validatedGroundLength(
              map_,p_.groundParameters(),from,to,a->state,b->state):std::nullopt;
          cost=reused?*reused:directedLength(a->state,b->state);
        }
      }
      if(!std::isfinite(cost))continue;
      const double next=distance+cost;
      if(next<target->second){
        target->second=next;report.parent_id_map[neighbor.first]=id;queue.emplace(next,neighbor.first);
      }
    }
  }
  report.status=true;return true;
}
void GlobalMemory::updateMap(std::shared_ptr<const MapQuery> map){
  updateMap(std::move(map),TerrainQueryReuse::Disabled);
}
void GlobalMemory::updateMap(std::shared_ptr<const MapQuery> map,TerrainQueryReuse reuse){
  updateMap(std::move(map),reuse,{});
}
void GlobalMemory::updateMap(std::shared_ptr<const MapQuery> map,TerrainQueryReuse reuse,
                            const std::function<void(const char*)>& progress){
  updateMap(std::move(map),reuse,progress,{});
}
void GlobalMemory::updateMap(std::shared_ptr<const MapQuery> map,TerrainQueryReuse reuse,
                            const std::function<void(const char*)>& progress,
                            const std::function<void()>& shutdown_checkpoint){
  if(shutdown_checkpoint)shutdown_checkpoint();
  if(progress)progress("global_replace_begin");
  map_=std::move(map);map_edge_update_={};
  terrain_.reset();ground_edges_.clear();
  if(p_.ground_robot&&map_)terrain_=std::make_unique<TerrainProjector>(map_,p_.groundParameters(),reuse);
  if(progress)progress("global_replace_end");
  if(shutdown_checkpoint)shutdown_checkpoint();
  if(progress)progress("global_active_edges_begin");
  // Preserve a removed edge's original endpoints and cost, not a fresh nearest-
  // neighbor shortcut. Unknown cells block recovery just like occupied cells.
  std::vector<std::pair<int,int>> remove;
  std::vector<std::pair<std::pair<int,int>,double>> reweight;
  for(const auto& item:graph_->edge_map_)for(const auto& edge:item.second){
    if(item.first>=edge.first)continue;
    if(shutdown_checkpoint)shutdown_checkpoint();
    double length=std::numeric_limits<double>::infinity();
    if(p_.ground_robot){
      const auto& forward=groundEdge(item.first,edge.first);
      const auto& backward=groundEdge(edge.first,item.first);
      if(forward.admissible())length=forward.actual_length;
      if(backward.admissible())length=std::min(length,backward.actual_length);
    }else length=directedLength(graph_->getVertex(item.first)->state,graph_->getVertex(edge.first)->state);
    if(!std::isfinite(length)){
      const auto key=std::make_pair(item.first,edge.first);
      if(suspended_edges_.size()<kMaxEdges)suspended_edges_[key]=edge.second;
      remove.push_back(key);
    }else if(p_.ground_robot&&std::abs(length-edge.second)>1e-10)
      reweight.push_back({{item.first,edge.first},length});
  }
  if(!remove.empty()||!reweight.empty())++snapshot_revision_;
  for(const auto& edge:remove)graph_->removeEdge(graph_->getVertex(edge.first),graph_->getVertex(edge.second));
  for(const auto& item:reweight){
    auto* a=graph_->getVertex(item.first.first);auto* b=graph_->getVertex(item.first.second);
    graph_->removeEdge(a,b);graph_->addEdge(a,b,item.second);
  }
  map_edge_update_.invalidated=remove.size();
  if(progress)progress("global_active_edges_end");
  if(progress)progress("global_suspended_edges_begin");
  // Restore only after a new accepted snapshot clears the entire swept robot
  // box. Do not change home, vertex history, frontier gain, or active missions.
  for(auto it=suspended_edges_.begin();it!=suspended_edges_.end();){
    if(shutdown_checkpoint)shutdown_checkpoint();
    auto* a=graph_->getVertex(it->first.first);auto* b=graph_->getVertex(it->first.second);
    if(!a||!b){fixed_ground_edges_.erase(it->first);it=suspended_edges_.erase(it);continue;}
    double length=std::numeric_limits<double>::infinity();
    bool reused=false;
    if(p_.ground_robot&&reuse==TerrainQueryReuse::ImmutableSnapshot){
      // This cache was cleared above. Only active links rechecked on THIS
      // snapshot have both entries, including links just suspended by that
      // check. Retain their fixed/projected geometry and directional verdicts;
      // older suspended links still need their first check on the new map.
      const auto forward=ground_edges_.find({a->id,b->id});
      const auto backward=ground_edges_.find({b->id,a->id});
      if(forward!=ground_edges_.end()&&backward!=ground_edges_.end()){
        const double unavailable=std::numeric_limits<double>::infinity();
        length=std::min(forward->second.admissible()?forward->second.actual_length:unavailable,
                        backward->second.admissible()?backward->second.actual_length:unavailable);
        reused=true;
      }
    }
    if(!reused)length=linkLength(a->state,b->state,fixedGroundEdge(a->id,b->id));
    if(std::isfinite(length)&&graph_->getNumEdges()<kMaxEdges){
      const int previous=graph_->getNumEdges();
      graph_->addEdge(a,b,p_.ground_robot?length:it->second);
      if(graph_->getNumEdges()>previous){++map_edge_update_.restored;++snapshot_revision_;}
      it=suspended_edges_.erase(it);
    }else ++it;
  }
  map_edge_update_.pending=suspended_edges_.size();
  if(progress)progress("global_suspended_edges_end");
}
Vertex* GlobalMemory::append(const StateVec& state,Vertex* parent,bool preserve_geometry){
  StateVec projected;
  if(graph_->getNumVertices()>=kMaxVertices)return nullptr;
  if(p_.ground_robot&&preserve_geometry){
    if(!terrain_||!terrain_->validateFixedEdge(state,state).admissible())return nullptr;
    projected=state;
  }else if(!projectState(state,projected))return nullptr;
  // Only the explicit immutable-map contract permits retaining these results
  // for neighbor expansion/routing. Keep the original disabled path intact.
  const bool retain_edges=parent&&p_.ground_robot&&map_&&terrain_&&terrain_->queryReuseEnabled()&&
      parent->state.allFinite()&&projected.allFinite();
  GroundEdge outward,returning;double length=0.;
  if(retain_edges){
    // Match linkLength's exact query order, directional policies and minimum
    // cost. append's projection above is intentionally not skipped.
    outward=preserve_geometry?terrain_->validateFixedEdge(parent->state,projected,true):
        terrain_->edge(parent->state,projected);
    returning=preserve_geometry?terrain_->validateFixedEdge(projected,parent->state,true):
        terrain_->edge(projected,parent->state);
    const double unavailable=std::numeric_limits<double>::infinity();
    length=std::min(outward.admissible()?outward.actual_length:unavailable,
                    returning.admissible()?returning.actual_length:unavailable);
  }else length=parent?linkLength(parent->state,projected,preserve_geometry):0.;
  if(!std::isfinite(length))return nullptr;
  auto vertex=std::make_unique<Vertex>(graph_->generateVertexID(),projected);
  vertex->parent=parent;
  if(parent)vertex->distance=parent->distance+length;
  auto* added=vertex.get();graph_->addVertex(added);vertex.release();
  ++snapshot_revision_; // Also covers the home vertex with no incident edges.
  if(parent){
    parent->children.push_back(added);graph_->addEdge(parent,added,length);
    if(preserve_geometry)markFixedGroundEdge(parent->id,added->id);
    if(retain_edges){
      // markFixedGroundEdge clears both entries, so retain only afterwards.
      // updateMap/reset already clear this cache before changing its inputs.
      ground_edges_.emplace(std::make_pair(parent->id,added->id),std::move(outward));
      ground_edges_.emplace(std::make_pair(added->id,parent->id),std::move(returning));
    }
  }
  return added;
}
void GlobalMemory::neighborEdges(Vertex* vertex){
  if(graph_->getNumEdges()>=kMaxEdges)return;
  const int previous_edges=graph_->getNumEdges();
  std::vector<Vertex*> near;graph_->getNearestVertices(&vertex->state,p_.neighbor_radius,&near);
  for(auto* other:near){
    if(graph_->getNumEdges()>=kMaxEdges)break;
    // Do not replace a retained reference segment with a newly projected
    // neighbor link on the same pair of graph vertices.
    if(fixedGroundEdge(vertex->id,other->id))continue;
    Eigen::Vector3d direction=other->state.head<3>()-vertex->state.head<3>();double distance=direction.norm();
    if(distance<=p_.edge_min||distance>=p_.edge_max)continue;
    StateVec a=vertex->state,b=other->state;
    a.head<3>()-=p_.overshoot*direction/distance;
    if(other->id!=0)b.head<3>()+=p_.overshoot*direction/distance;
    if(p_.ground_robot){
      // Keep the undirected topology when at least one orientation is usable;
      // the directed route search enforces downhill limits for every traversal.
      // Extension geometry is separate from the stored graph endpoints. With
      // zero overshoot it is already covered by the two strict cached checks.
      if(p_.overshoot>0.&&!safe(a,b)&&!safe(b,a))continue;
      const auto& forward=groundEdge(vertex->id,other->id);
      const auto& backward=groundEdge(other->id,vertex->id);
      double length=std::numeric_limits<double>::infinity();
      if(forward.admissible())length=forward.actual_length;
      if(backward.admissible())length=std::min(length,backward.actual_length);
      if(std::isfinite(length))graph_->addEdge(vertex,other,length);
      else{
        // Retain both orientations of admitted graph links, not an unbounded
        // collection of failed neighbor pairs outside the graph edge budget.
        ground_edges_.erase({vertex->id,other->id});
        ground_edges_.erase({other->id,vertex->id});
      }
    }else if(safe(a,b))graph_->addEdge(vertex,other,distance);
  }
  if(graph_->getNumEdges()!=previous_edges)++snapshot_revision_;
}
Vertex* GlobalMemory::connect(const StateVec& input){
  StateVec state;if(!projectState(input,state))return nullptr;
  Vertex* nearest=nullptr;
  if(!graph_->getNearestVertex(&state,&nearest))return nullptr;
  const double distance=(state-nearest->state).head<3>().norm();
  if(distance<1e-8){neighborEdges(nearest);return nearest;}
  if(p_.ground_robot){
    // The nearest site may only be reachable in the opposite direction. Check
    // other nearby attachments before declaring the current pose disconnected.
    std::vector<Vertex*> nearby;graph_->getNearestVertices(&state,p_.neighbor_radius,&nearby);
    double best=std::numeric_limits<double>::infinity();Vertex* anchor=nullptr;
    for(auto* candidate:nearby){
      const double length=directedLength(state,candidate->state);
      if(length<best||(length==best&&anchor&&candidate->id<anchor->id)){
        best=length;anchor=candidate;
      }
    }
    if(anchor){auto* added=append(state,anchor);if(added)neighborEdges(added);return added;}
  }else if(distance<=p_.neighbor_radius&&safe(state,nearest->state)){
    // Preserve the existing exact, collision-checked nearby attachment.
    auto* added=append(state,nearest);if(added)neighborEdges(added);return added;
  }
  // searchHomingPath/getGlobalPath/connectStateToGraph fall back to a single
  // expandGraph parent edge. Its maximum length is independent of the RRG
  // neighbor radius (SMB uses 3 m / 2.5 m). Never steer a distant real start
  // into a shorter fictitious start, or copy the upstream blind short link.
  if(distance<=p_.edge_min||distance>p_.edge_max||
     graph_->getNumVertices()>=kMaxVertices||graph_->getNumEdges()>=kMaxEdges)return nullptr;
  const Eigen::Vector3d extra=p_.overshoot*(state-nearest->state).head<3>()/distance;
  StateVec extended_anchor=nearest->state,extended_current=state;
  if(nearest->id!=0)extended_anchor.head<3>()-=extra;
  extended_current.head<3>()+=extra;
  GroundEdge outward,returning;
  double parent_length=distance;
  if(p_.ground_robot){
    // The parent must be admissible as in expandGraph, and its reverse must
    // actually carry this current -> anchor request. An undirected minimum
    // cost is insufficient when downhill and uphill limits differ.
    outward=terrain_->edge(nearest->state,state);
    returning=terrain_->edge(state,nearest->state);
    if(!outward.admissible()||!returning.admissible())return nullptr;
    const auto exact_endpoints=[](const GroundEdge& edge,const StateVec& from,const StateVec& to){
      return !edge.polyline.empty()&&(edge.polyline.front()-from).head<3>().norm()<=1e-9&&
        (edge.polyline.back()-to).head<3>().norm()<=1e-9;
    };
    if(!exact_endpoints(outward,nearest->state,state)||!exact_endpoints(returning,state,nearest->state))return nullptr;
    if(p_.overshoot>0&&(!terrain_->edge(extended_anchor,extended_current).admissible()||
                        !terrain_->edge(extended_current,extended_anchor).admissible()))return nullptr;
    parent_length=outward.actual_length;
  }else if(map_->path(extended_anchor.head<3>()+p_.center_offset,
                     extended_current.head<3>()+p_.center_offset,p_.robot_size)!=VoxelStatus::Free)return nullptr;
  // All checks precede mutation. Keep the already projected exact endpoint;
  // append() would project it again and choose a minimum-direction edge cost.
  auto vertex=std::make_unique<Vertex>(graph_->generateVertexID(),state);
  vertex->parent=nearest;vertex->distance=nearest->distance+parent_length;
  auto* added=vertex.get();graph_->addVertex(added);vertex.release();
  nearest->children.push_back(added);graph_->addEdge(nearest,added,parent_length);
  ++snapshot_revision_;
  if(p_.ground_robot){
    ground_edges_[{nearest->id,added->id}]=std::move(outward);
    ground_edges_[{added->id,nearest->id}]=std::move(returning);
  }
  if(p_.graph_mode)neighborEdges(added);
  return added;
}
bool GlobalMemory::setHomingPosition(const StateVec& input){
  // Rrg::setHomingPos only accepts an empty graph. Do not move vertex zero
  // underneath existing edges, or reinterpret this as a mid-mission rebase.
  StateVec state;
  if(graph_->getNumVertices()!=0||!projectState(input,state))return false;
  auto* home=append(state,nullptr);if(!home)return false;
  // Rrg::setHomingPos creates a graph anchor, not an observed-history event.
  last_odometry_=home;return true;
}
bool GlobalMemory::recordOdometry(const StateVec& input){
  StateVec state;if(!projectState(input,state))return false;
  // The physical visit still counts when the graph cannot retain another
  // odometry edge. Run E1 after insertion so it includes this callback's vertex.
  const auto finish=[&](bool recorded){applyVisitedEvent(state);return recorded;};
  if(graph_->getNumVertices()==0)return finish(setHomingPosition(state));
  // Upstream timerCallback backtracking spacing is 0.5 m. Revalidate the
  // observed edge before retaining it, and keep shorter moves for connect().
  if(last_odometry_&&(state-last_odometry_->state).head<3>().norm()<0.5)return finish(true);
  if(last_odometry_&&safe(last_odometry_->state,state)){
    auto* added=append(state,last_odometry_);if(!added)return finish(false);
    last_odometry_=added;neighborEdges(added);return finish(true);
  }
  auto* added=connect(state);if(!added)return finish(false);last_odometry_=added;return finish(true);
}
void GlobalMemory::applyVisitedEvent(StateVec state){
  // Rrg::timerCallback E1: each 1 m displacement retires every graph vertex
  // in the current 3 m neighborhood. Keep gain and topology for routing home.
  if((state-last_visited_marker_).head<3>().norm()<1.0)return;
  // Rrg::timerCallback appends history at the SAME 1 m gate as E1. The
  // independent 0.5 m backtracking spacing above only controls graph vertices.
  visited_.push_back(state);++visit_revision_;
  std::vector<Vertex*> nearby;graph_->getNearestVertices(&state,3.0,&nearby);
  bool retired_frontier=false;
  for(auto* vertex:nearby){
    retired_frontier=retired_frontier||vertex->type==VertexType::kFrontier;
    vertex->type=VertexType::kVisited;
  }
  if(retired_frontier)++snapshot_revision_;
  last_visited_marker_=state;
  for(auto it=suspended_frontiers_.begin();it!=suspended_frontiers_.end();){
    const auto* vertex=graph_->getVertex(*it);
    if(!vertex||vertex->type==VertexType::kVisited)it=suspended_frontiers_.erase(it);
    else ++it;
  }
}
void GlobalMemory::refresh(const GainFunction& gain){
  for(auto& item:graph_->vertices_map_){
    auto* vertex=item.second;
    if(vertex->type==VertexType::kVisited){suspended_frontiers_.erase(vertex->id);continue;}
    if(vertex->type!=VertexType::kFrontier&&!suspended_frontiers_.count(vertex->id))continue;
    const bool was_frontier=vertex->type==VertexType::kFrontier;
    if(!safe(vertex->state,vertex->state)){
      // A map obstacle or temporary policy can hide a still useful viewpoint.
      // Retain its identity but remove its active status and old gain.
      suspended_frontiers_.insert(vertex->id);
      vertex->type=VertexType::kUnvisited;
      if(was_frontier||!sameGraphGain(vertex->vol_gain,VolumetricGain{}))++snapshot_revision_;
      vertex->vol_gain.reset();continue;
    }
    // Clearance alone does not restore an old frontier: the environment may
    // have been observed while it was blocked. Always recompute current gain.
    auto current_gain=gain(vertex->state);
    if(was_frontier!=current_gain.is_frontier||!sameGraphGain(vertex->vol_gain,current_gain))
      ++snapshot_revision_;
    vertex->vol_gain=std::move(current_gain);
    vertex->type=vertex->vol_gain.is_frontier?VertexType::kFrontier:VertexType::kUnvisited;
    suspended_frontiers_.erase(vertex->id);
  }
}
bool GlobalMemory::addPath(const std::vector<Vertex*>& path,const LocalPlanner* producer){
  if(path.size()<2)return false;
  // Check the whole incoming path before mutation, including map changes.
  for(std::size_t i=1;i<path.size();++i){
    // Reuse only this original local edge's strict scalar verdict. Later
    // connect/append projections can change the global parent geometry.
    const auto reused=p_.ground_robot&&producer?producer->validatedGroundLength(
        map_,p_.groundParameters(),path[i-1]->id,path[i]->id,
        path[i-1]->state,path[i]->state):std::nullopt;
    if(!reused&&!safe(path[i-1]->state,path[i]->state))return false;
  }
  auto* parent=connect(path.front()->state);if(!parent)return false;
  for(std::size_t i=1;i<path.size();++i){
    if(p_.ground_robot){
      const auto edge=terrain_->edge(parent->state,path[i]->state);
      if(!edge.admissible())return false;
      for(std::size_t knot=1;knot<edge.polyline.size();++knot){
        auto* added=append(edge.polyline[knot],parent);if(!added)return false;
        parent=added;neighborEdges(added);
      }
      continue;
    }
    StateVec start=parent->state;Eigen::Vector3d delta=path[i]->state.head<3>()-start.head<3>();
    const int pieces=std::max(1,int(std::ceil(delta.norm())));
    // Preserve upstream 1 m densification, without retaining redundant long edges.
    for(int part=1;part<=pieces;++part){
      StateVec state=path[i]->state;state.head<3>()=start.head<3>()+delta*(double(part)/pieces);
      auto* added=append(state,parent);if(!added)return false;
      parent=added;neighborEdges(added);
    }
  }
  parent->type=VertexType::kFrontier;parent->vol_gain=path.back()->vol_gain;
  return true;
}
bool GlobalMemory::addReferencePath(const std::vector<StateVec>& path,int* endpoint_id,
                                    ReferencePathDiagnostics* diagnostics){
  if(endpoint_id)*endpoint_id=-1;
  if(diagnostics)*diagnostics=ReferencePathDiagnostics{};
  const auto fail=[&](const std::string& reason,int segment=-1,
                      const StateVec& from=StateVec::Zero(),const StateVec& to=StateVec::Zero()){
    if(diagnostics){diagnostics->reason=reason;diagnostics->segment=segment;
      diagnostics->from=from;diagnostics->to=to;}
    return false;
  };
  if(path.size()<2||path.size()>kMaxVertices)return fail("invalid_path_size");
  if(!map_)return fail("map_missing");
  if(!graph_->getNumVertices())return fail("global_graph_empty");
  // Check every original segment before changing graph state. Densification
  // preserves its exact geometry; unknown and occupied cells both reject it.
  StateVec first=path.front();
  if(p_.ground_robot){
    const auto checked=terrain_->validateFixedEdge(first,first);
    if(!checked.admissible())return fail(std::string("ground_segment_")+groundStatusName(checked.status),0,first,first);
  }else if(!projectState(path.front(),first))
    return fail("first_projection_failed",0,path.front(),path.front());
  std::vector<StateVec> dense{first};
  std::vector<int> dense_segments;
  if(diagnostics)dense_segments.push_back(0);
  for(std::size_t i=1;i<path.size();++i){
    if(p_.ground_robot){
      const auto edge=terrain_->validateFixedEdge(dense.back(),path[i]);
      if(!edge.admissible())
        return fail(std::string("ground_segment_")+groundStatusName(edge.status),int(i),dense.back(),path[i]);
      if(dense.size()+edge.polyline.size()>kMaxVertices)
        return fail("dense_vertex_limit",int(i),dense.back(),path[i]);
      if(diagnostics)dense_segments.insert(dense_segments.end(),edge.polyline.size()-1,int(i));
      dense.insert(dense.end(),std::next(edge.polyline.begin()),edge.polyline.end());
      continue;
    }
    if(!safe(path[i-1],path[i]))return fail("aerial_segment_blocked",int(i),path[i-1],path[i]);
    const Eigen::Vector3d delta=(path[i]-path[i-1]).head<3>();
    const double length=delta.norm();
    if(!std::isfinite(length)||length>kMaxVertices)
      return fail("invalid_segment_length",int(i),path[i-1],path[i]);
    if(length<=1e-9)continue;
    const int pieces=std::max(1,int(std::ceil(length)));
    if(dense.size()+pieces>kMaxVertices)return fail("dense_vertex_limit",int(i),path[i-1],path[i]);
    for(int part=1;part<=pieces;++part){
      StateVec state=path[i];
      state.head<3>()=path[i-1].head<3>()+delta*(double(part)/pieces);
      dense.push_back(state);
      if(diagnostics)dense_segments.push_back(int(i));
    }
  }
  Vertex* anchor=nullptr;
  if(!graph_->getNearestVertex(&dense.front(),&anchor))
    return fail("anchor_missing",0,dense.front(),dense.front());
  const double connector=(anchor->state-dense.front()).head<3>().norm();
  if(connector>p_.neighbor_radius)return fail("anchor_out_of_range",0,anchor->state,dense.front());
  // The existing graph anchor can use the same-site sub-voxel attachment
  // rule. Every original reference segment above was checked without it.
  if(!std::isfinite(linkLength(anchor->state,dense.front(),p_.ground_robot))){
    // A nearby historical vertex can have a different terrain height. Its
    // short fixed connector may fail even when another graph vertex has a
    // fully valid connection. Keep the original nearest choice when valid;
    // only ground failures search alternatives, without changing path knots
    // or the fixed-geometry collision, support and directional slope checks.
    auto* rejected=anchor;
    if(p_.ground_robot){
      std::vector<Vertex*> nearby;
      graph_->getNearestVertices(&dense.front(),p_.neighbor_radius,&nearby);
      std::sort(nearby.begin(),nearby.end(),[&](const Vertex* a,const Vertex* b){
        const double da=(a->state-dense.front()).head<3>().squaredNorm();
        const double db=(b->state-dense.front()).head<3>().squaredNorm();
        return da<db||(da==db&&a->id<b->id);
      });
      for(auto* candidate:nearby){
        if(candidate==rejected)continue;
        if(std::isfinite(linkLength(candidate->state,dense.front(),true))){
          anchor=candidate;break;
        }
      }
    }
    if(anchor==rejected)return fail("anchor_unreachable",0,rejected->state,dense.front());
  }
  // Reserve worst-case capacity before mutation; avoid a partially stored
  // execution route because a graph limit was discovered half-way through.
  if(graph_->getNumVertices()+dense.size()>kMaxVertices)return fail("global_vertex_limit");
  if(graph_->getNumEdges()+dense.size()>kMaxEdges)return fail("global_edge_limit");
  Vertex* parent=anchor;
  std::vector<Vertex*> reference_vertices;
  reference_vertices.reserve(dense.size());
  std::set<int> reference_ids;
  for(std::size_t i=0;i<dense.size();++i){
    const auto& state=dense[i];
    Vertex* exact=nullptr;
    graph_->getNearestVertex(&state,&exact);
    // Ground reference states include exact yaw (including pure turns).
    // Do not merge them into a nearby projected graph state with another Z.
    const bool same=exact&&(p_.ground_robot?(exact->state.array()==state.array()).all():
        (exact->state-state).head<3>().norm()<1e-8);
    if(same&&std::isfinite(directedLength(parent->state,exact->state,p_.ground_robot))){
      if(exact!=parent){
        const int previous_edges=graph_->getNumEdges();
        graph_->addEdge(parent,exact,directedLength(parent->state,exact->state,p_.ground_robot));
        if(graph_->getNumEdges()!=previous_edges)++snapshot_revision_;
        if(p_.ground_robot)markFixedGroundEdge(parent->id,exact->id);
      }
      parent=exact;
    }else{
      auto* added=append(state,parent,p_.ground_robot);
      if(!added)return fail("append_failed",diagnostics?dense_segments[i]:-1,parent->state,state);
      parent=added;
    }
    if(reference_ids.insert(parent->id).second)reference_vertices.push_back(parent);
  }
  // Rrg::addRefPathToGraph expands the neighborhood of every retained path
  // vertex, including a reused root. Finish the mandatory chain first so
  // optional shortcuts cannot consume capacity needed to preserve the route.
  // neighborEdges keeps fixed reference geometry intact and validates each
  // new connection against the current map and directed ground constraints.
  for(auto* vertex:reference_vertices)neighborEdges(vertex);
  // This is a reference route, not an observed visit or a candidate frontier.
  // Keep last_odometry_, visited_, vertex types and home unchanged.
  if(endpoint_id)*endpoint_id=parent->id;
  if(diagnostics)diagnostics->reason="ok";
  return true;
}
int GlobalMemory::addSemanticPath(const std::vector<StateVec>& path,int semantic_class){
  using Class=planner_semantic_msgs::msg::SemanticClass;
  if(path.empty()||(semantic_class!=Class::K_STAIRCASE&&semantic_class!=Class::K_DOOR))return -1;
  int endpoint=-1;
  if(path.size()==1){
    Vertex* existing=nullptr;
    if(!safe(path.front(),path.front())||!graph_->getNearestVertex(&path.front(),&existing)||
       (existing->state-path.front()).head<3>().norm()>1e-8)return -1;
    endpoint=existing->id;
  }else if(!addReferencePath(path,&endpoint))return -1;
  auto* vertex=graph_->getVertex(endpoint);
  if(vertex->semantic_class.value!=semantic_class)++snapshot_revision_;
  vertex->semantic_class.value=semantic_class;
  // Preserve home identity. Other semantic destinations follow the original
  // frontier lifecycle; later refresh still evaluates actual sensor gain.
  if(endpoint!=0&&vertex->type!=VertexType::kVisited){
    if(vertex->type!=VertexType::kFrontier)++snapshot_revision_;
    vertex->type=VertexType::kFrontier;
  }
  return endpoint;
}
void GlobalMemory::absorbLocal(const planner_msgs::msg::Graph& message,const GainFunction& gain,
                               FrontierClusteringDiagnostics* diagnostics){
  absorbLocal(message,gain,diagnostics,{});
}
void GlobalMemory::absorbLocal(const planner_msgs::msg::Graph& message,const GainFunction& gain,
                               FrontierClusteringDiagnostics* diagnostics,
                               const std::function<void(const char*)>& progress){
  absorbLocal(message,gain,diagnostics,progress,nullptr);
}
void GlobalMemory::absorbLocal(const planner_msgs::msg::Graph& message,const GainFunction& gain,
                               FrontierClusteringDiagnostics* diagnostics,
                               const std::function<void(const char*)>& progress,
                               const LocalPlanner* producer){
  if(diagnostics)*diagnostics=FrontierClusteringDiagnostics{};
  frontier_update_={};
  if(!map_||!graph_->getNumVertices())return;
  if(progress)progress("absorb_refresh_begin");
  refresh(gain);
  if(progress)progress("absorb_refresh_completed");
  auto local=std::make_shared<GraphManager>();local->convertMsgToGraph(message);
  if(!local->getVertex(0))return;
  if(progress)progress("absorb_shortest_paths_begin");
  ShortestPathsReport report;if(!shortestPaths(*local,0,report,false,producer))return;
  if(progress)progress("absorb_shortest_paths_completed");
  local->findLeafVertices(report);
  if(progress)progress("absorb_candidates_begin");
  std::vector<Vertex*> candidates;
  for(int id=1;id<local->getNumVertices();++id){
    auto* vertex=local->getVertex(id);if(!vertex)continue;
    if(vertex->is_leaf_vertex&&vertex->vol_gain.is_frontier&&
       std::isfinite(local->getShortestDistance(id,report))){
      vertex->vol_gain=gain(vertex->state);
      if(vertex->vol_gain.is_frontier)candidates.push_back(vertex);
    }
  }
  frontier_update_.candidates=candidates.size();
  if(progress)progress("absorb_candidates_completed");
  if(progress)progress("absorb_cluster_begin");
  auto ids=FrontierClusterer::cluster(local,report,candidates);
  if(progress)progress("absorb_cluster_completed");
  frontier_update_.clusters=ids.size();
  if(diagnostics){
    diagnostics->evaluated=true;diagnostics->candidate_count=candidates.size();
    diagnostics->representative_ids=ids;diagnostics->candidates.reserve(candidates.size());
    // Capture the already-evaluated assignment before existing/visited-path
    // exclusion. getShortestPath only walks this report; it never queries the
    // map or repeats Dijkstra, clustering or terrain projection.
    for(const auto* vertex:candidates){
      FrontierClusterPath candidate;candidate.vertex_id=vertex->id;candidate.cluster_id=vertex->cluster_id;
      local->getShortestPath(vertex->id,report,true,candidate.path);
      diagnostics->candidates.push_back(std::move(candidate));
    }
  }
  if(progress)progress("absorb_paths_begin");
  for(int id:ids){
    auto* vertex=local->getVertex(id);Vertex* nearest=nullptr;
    // Same 1 m existing-graph and 3 m visited-state exclusion as addFrontiers.
    if(graph_->getNearestVertexInRange(&vertex->state,1.0,&nearest)){++frontier_update_.near_existing;continue;}
    bool visited=false;for(const auto& state:visited_)if((state-vertex->state).head<3>().norm()<=3.0){visited=true;break;}
    if(visited){++frontier_update_.near_visited;continue;}
    std::vector<Vertex*> path;local->getShortestPath(id,report,true,path);
    if(addPath(path,producer))++frontier_update_.added;else ++frontier_update_.invalid_paths;
  }
  if(progress)progress("absorb_paths_completed");
}
void GlobalMemory::graphMessage(planner_msgs::msg::Graph& message){
  graph_->convertGraphToMsg(message);
  // E1 deliberately retains observation counts and gain on visited vertices.
  // The display flag must follow the same type filter as global selection.
  // Keep the original graph serialization unchanged for persistence/local use.
  for(auto& vertex:message.vertices)
    vertex.is_frontier=graph_->getVertex(vertex.id)->type==VertexType::kFrontier;
}
int GlobalMemory::frontierCount() const {
  int count=0;for(const auto& item:graph_->vertices_map_)if(item.second->type==VertexType::kFrontier)++count;return count;
}
GlobalResult GlobalMemory::route(const StateVec& current,int target,RouteKind kind,bool capture_refinement,bool capture_graph_paths,
                                 HomingPathDiagnostics* homing_diagnostic){
  GlobalResult result;result.target=target;
  if(!graph_->getVertex(target)){result.reason="invalid_global_target";return result;}
  auto* start=connect(current);if(!start){result.reason="current_not_connected";return result;}
  ShortestPathsReport report;shortestPaths(*graph_,start->id,report);
  graph_->getShortestPath(target,report,true,result.path);
  if(result.path.empty()){result.reason="global_target_unreachable";return result;}
  if(homing_diagnostic){
    try{
      // Reuse this request's directed report; never recompute a reverse home
      // tree or capture the expanded/refined execution geometry below.
      graph_->getShortestPath(target,report,true,homing_diagnostic->vertex_ids);
      homing_diagnostic->vertices=result.path;homing_diagnostic->captured=true;
    }catch(const std::exception&){*homing_diagnostic={};}
  }
  if(capture_graph_paths){
    // Keep the report's vertices before the route replaces the first state
    // or expands/refines its geometry. Diagnostic capture cannot reject an
    // otherwise executable route.
    try{
      graph_->getShortestPath(target,report,true,result.graph_paths.current_to_frontier_ids);
      result.graph_paths.current_to_frontier=result.path;
      result.graph_paths.captured=true;result.graph_paths.home_reason="not_evaluated";
    }catch(const std::exception&){result.graph_paths={};}
  }
  const auto finish=[&]{
    result.success=true;result.reason="ok";
    result.path.front()[3]=current[3];
    // runGlobalPlanner starts with forward tangents; searchHomingPath and
    // getGlobalPath first apply the optional heading-relative reverse policy.
    if(kind==RouteKind::GlobalFrontier)applyPathHeading(result.path,p_.tangent_yaw,false);
    else applyHomingPathHeading(result.path,p_.tangent_yaw,p_.refinement.homing_backward);
    if(p_.ground_robot&&kind==RouteKind::Homing)
      result.reconnection=reconnectGroundPath(*terrain_,result.path,result.distance);
    // All three use relaxed corridors. On success improveFreePath applies
    // planning_backward, including global and waypoint routes. On failure
    // refinePlannedPath retains the already assigned input headings.
    if(p_.refinement.enabled){
      auto configured=p_;
      configured.refinement.capture_geometry=capture_refinement;
      result.refinement=refinePlannedPath(map_,configured,result.path,result.distance,true,true);
    }
    result.path.front()[3]=current[3];
    // Only getHomingPath explicitly reapplies homing_backward after path
    // enhancement. Do not overwrite the other two producers' refined yaw.
    if(kind==RouteKind::Homing)
      applyHomingPathHeading(result.path,p_.tangent_yaw,p_.refinement.homing_backward);
    return result;
  };
  if(p_.ground_robot){
    const auto vertices=result.path;result.path.clear();
    std::vector<int> vertex_ids;graph_->getShortestPath(target,report,true,vertex_ids);
    StateVec first;
    if(!projectState(current,first)){result.reason="current_not_supported";return result;}
    result.path.push_back(first);
    // Route output stays at planning height. The ROS boundary alone converts
    // every knot to base_link height, preserving this complete terrain shape.
    for(std::size_t i=1;i<vertices.size();++i){
      const auto edge=fixedGroundEdge(vertex_ids[i-1],vertex_ids[i])?
          terrain_->validateFixedEdge(result.path.back(),vertices[i],true):
          terrain_->edge(result.path.back(),vertices[i]);
      if(!edge.admissible()){
        result.path.clear();result.reason="global_path_invalidated";return result;
      }
      result.distance+=edge.actual_length;
      for(std::size_t knot=1;knot<edge.polyline.size();++knot){
        auto state=edge.polyline[knot];
        if(!p_.tangent_yaw)state[3]=vertices[i][3];
        result.path.push_back(state);
      }
    }
    return finish();
  }
  result.path.front()=current;
  for(std::size_t i=1;i<result.path.size();++i){
    if(!safe(result.path[i-1],result.path[i])){result.path.clear();result.reason="global_path_invalidated";return result;}
    const Eigen::Vector3d delta=(result.path[i]-result.path[i-1]).head<3>();
    result.distance+=delta.norm();
  }
  return finish();
}
GlobalResult GlobalMemory::selectFrontier(const StateVec& current,const GainFunction& gain,int target,const GlobalTimeConstraint* time,
                                         bool require_frontier,bool capture_refinement,bool capture_graph_paths){
  GlobalResult result;if(!graph_->getNumVertices()){result.reason="global_graph_empty";return result;}
  if(time&&(!std::isfinite(time->remaining)||time->remaining<0||!std::isfinite(time->speed)||time->speed<=0)){
    result.reason="invalid_time_constraint";return result;
  }
  refresh(gain);
  // Upstream's explicit-target option checks whether any frontier remains;
  // the requested destination itself need not be a frontier vertex.
  if(require_frontier&&frontierCount()==0){result.reason="no_global_frontier";return result;}
  ShortestPathsReport home_report;
  const auto capture_return=[&]{
    if(!result.success){result.graph_paths={};return;}
    auto& diagnostic=result.graph_paths;
    if(!capture_graph_paths||!diagnostic.captured)return;
    try{
      // Reuse the budget's report when one exists. Otherwise do this one
      // reverse traversal only AFTER the selected route has passed its final
      // feasibility check. Never refine, reconnect, refresh gain or read the
      // mission clock just to draw a return graph chain.
      if(!(time&&time->include_home))shortestPaths(*graph_,0,home_report,true);
      graph_->getShortestPath(result.target,home_report,false,diagnostic.frontier_to_home_ids);
      graph_->getShortestPath(result.target,home_report,false,diagnostic.frontier_to_home);
      if(!home_report.status)diagnostic.home_reason="home_report_unavailable";
      else if(diagnostic.frontier_to_home.empty())diagnostic.home_reason="home_unreachable";
      else diagnostic.home_reason="ok";
    }catch(const std::exception&){diagnostic.home_reason="home_report_error";}
    if(diagnostic.home_reason!="ok"){
      diagnostic.frontier_to_home_ids.clear();diagnostic.frontier_to_home.clear();
    }
  };
  // Build this report only after connect() has added the actual current pose
  // and its neighbor edges. Those edges can shorten or reconnect a return
  // route even for aerial graphs. Reverse traversal preserves ground
  // candidate -> home costs when uphill/downhill admissibility differs.
  std::map<int,double> refined_home_lengths;
  std::string time_failure_reason="global_time_insufficient";
  auto feasible=[&](int id,double outward,double& spare){
    if(!time){spare=1.;return true;}
    time_failure_reason="global_time_insufficient";
    double home=0.;
    if(time->include_home){
      std::vector<int> home_path;graph_->getShortestPath(id,home_report,true,home_path);
      if(home_path.empty())return false;
      home=graph_->getShortestDistance(id,home_report);
      if(!std::isfinite(home))return false;
      if(p_.refinement.enabled){
        const auto cached=refined_home_lengths.find(id);
        if(cached!=refined_home_lengths.end())home=cached->second;
        else{
          // The home budget must include the same ground reconnection and
          // corridor pass as the future homing request. With refinement off,
          // the unshortened graph distance remains a conservative estimate.
          // Reverse the home-rooted report into candidate -> home order.
          std::reverse(home_path.begin(),home_path.end());
          std::vector<StateVec> returning{graph_->getVertex(home_path.front())->state};
          for(std::size_t i=1;i<home_path.size();++i){
            if(p_.ground_robot){
              const auto& edge=groundEdge(home_path[i-1],home_path[i]);
              if(!edge.admissible())return false;
              returning.insert(returning.end(),std::next(edge.polyline.begin()),edge.polyline.end());
            }else returning.push_back(graph_->getVertex(home_path[i])->state);
          }
          if(p_.ground_robot)reconnectGroundPath(*terrain_,returning,home);
          // Candidate return routes are only budget estimates, never the
          // request's selected route diagnostics.
          auto configured=p_;configured.refinement.capture_geometry=false;
          refinePlannedPath(map_,configured,returning,home,true,true);
          refined_home_lengths.emplace(id,home);
        }
      }
    }
    // The selected return route may itself need reconnection/refinement.
    // Read the live budget only after that work, including the explicit-target
    // branch's sole feasibility check. Paused ROS time must not age by wall time.
    double remaining=time->remaining;
    if(time->refresh_budget){
      const auto observed=time->refresh_budget();
      if(!observed.valid||!observed.clock_valid){
        spare=0.;
        time_failure_reason=!observed.reason.empty()&&observed.reason!="ok"?
            observed.reason:(observed.clock_valid?"mission_budget_invalid":"mission_clock_invalid");
        return false;
      }
      if(!std::isfinite(observed.remaining)||observed.remaining<0.){
        spare=0.;time_failure_reason="invalid_time_constraint";return false;
      }
      remaining=observed.remaining;
    }
    // Fix upstream ternary precedence: outbound + (auto_homing ? home : 0).
    return MissionBudget::sufficient(remaining,(outward+home)/time->speed,spare);
  };
  if(target>=0){
    result=route(current,target,RouteKind::GlobalFrontier,capture_refinement,capture_graph_paths);double spare=0.;
    if(time&&time->include_home)shortestPaths(*graph_,0,home_report,true);
    if(result.success&&!feasible(target,result.distance,spare)){
      result.success=false;result.path.clear();result.reason=time_failure_reason;
    }
    capture_return();
    return result;
  }
  auto* start=connect(current);if(!start){result.reason="current_not_connected";return result;}
  if(time&&time->include_home)shortestPaths(*graph_,0,home_report,true);
  ShortestPathsReport report;shortestPaths(*graph_,start->id,report);
  double best=-1;int best_id=-1;
  for(int id=1;id<graph_->getNumVertices();++id){
    auto* vertex=graph_->getVertex(id);if(!vertex||vertex->type!=VertexType::kFrontier)continue;
    std::vector<int> path;graph_->getShortestPath(id,report,true,path);
    double distance=graph_->getShortestDistance(id,report);
    if(path.size()<2||!std::isfinite(distance)||distance<0.1)continue;
    double spare=1.;if(!feasible(id,distance,spare))continue;
    double score=vertex->vol_gain.gain*std::exp(-0.01*distance)*spare;
    if(score>best){best=score;best_id=id;}
  }
  if(best_id<0){result.reason=time&&frontierCount()>0?time_failure_reason:"no_global_frontier";return result;}
  result=route(current,best_id,RouteKind::GlobalFrontier,capture_refinement,capture_graph_paths);result.score=best;
  // Corridor adjustment may change route length after the graph-based
  // ranking. Recheck the actual route against the remaining mission budget.
  double spare=0.;
  if(result.success&&!feasible(best_id,result.distance,spare)){
    result.success=false;result.path.clear();result.reason=time_failure_reason;
  }
  capture_return();
  return result;
}
GlobalResult GlobalMemory::homing(const StateVec& current,bool capture_refinement){
  return homing(current,capture_refinement,nullptr);
}
GlobalResult GlobalMemory::homing(const StateVec& current,bool capture_refinement,HomingPathDiagnostics* diagnostic){
  if(diagnostic)*diagnostic={};
  try{
    auto result=route(current,0,RouteKind::Homing,capture_refinement,false,diagnostic);
    if(!result.success&&diagnostic)*diagnostic={};
    return result;
  }catch(...){if(diagnostic)*diagnostic={};throw;}
}
GlobalResult GlobalMemory::routeToWaypoint(const StateVec& current,const StateVec& target,
                                           double max_target_distance,bool capture_refinement){
  GlobalResult result;
  if(!current.allFinite()||!target.allFinite()){
    result.reason="invalid_waypoint_pose";return result;
  }
  if(!std::isfinite(max_target_distance)||max_target_distance<0.){
    result.reason="invalid_waypoint_distance_limit";return result;
  }
  if(!map_){result.reason="map_missing";return result;}
  // Rrg::getGlobalPath needs an established global graph, not only its home.
  if(graph_->getNumVertices()<=1){result.reason="global_graph_empty";return result;}
  Vertex* nearest=nullptr;
  if(!graph_->getNearestVertex(&target,&nearest)||!nearest){
    result.reason="waypoint_vertex_missing";return result;
  }
  result.target=nearest->id;
  if((nearest->state-target).head<3>().norm()>max_target_distance){
    result.reason="waypoint_too_far_from_graph";return result;
  }
  // Choose the requested graph destination before attaching the current pose,
  // so a newly inserted start cannot replace it. Keep the original nearest
  // vertex semantics even when that vertex is unreachable; do not select a
  // different goal or append an unchecked straight leg to the waypoint.
  // route() checks the current connector and every route segment. For ground
  // robots it uses directed terrain costs and returns every terrain knot.
  return route(current,nearest->id,RouteKind::Waypoint,capture_refinement);
}
std::vector<StateVec> GlobalMemory::backgroundCenters(){
  // Port of Rrg::expandGlobalGraphTimerCallback: form random 10 m clusters of
  // unvisited vertices, sample around centroids, reject samples within 5 m of
  // visited states/existing graph/frontiers, then call shared robot expansion.
  std::vector<Vertex*> remaining;
  for(int id=0;id<graph_->getNumVertices();++id){
    auto* vertex=graph_->getVertex(id);
    if(vertex&&vertex->type==VertexType::kUnvisited)remaining.push_back(vertex);
  }
  std::vector<StateVec> centers;
  while(!remaining.empty()){
    auto* anchor=remaining[std::uniform_int_distribution<std::size_t>(0,remaining.size()-1)(cluster_random_)];
    Eigen::Vector3d sum=Eigen::Vector3d::Zero();int count=0;std::vector<Vertex*> next;
    for(auto* vertex:remaining){
      if((vertex->state-anchor->state).head<3>().squaredNorm()<=100.){sum+=vertex->state.head<3>();++count;}
      else next.push_back(vertex);
    }
    StateVec center;center<<sum/double(count),0.;centers.push_back(center);remaining=std::move(next);
  }
  return centers;
}
bool GlobalMemory::drawBackgroundSample(const StateVec& center,const Eigen::Matrix3d& world_to_sampling,
                                       StateVec& sample,bool& hanging,BackgroundUpdate& stats){
  ++stats.attempts;hanging=false;sample=background_sampler_->generate(center);
  if(p_.ground_robot){
    const auto projection=terrain_->project(sample);
    if(projection.status!=GroundStatus::Supported&&projection.status!=GroundStatus::Hanging){
      ++stats.outside_or_occupied;return false;
    }
    sample=projection.projected_state;hanging=projection.status==GroundStatus::Hanging;
  }
  const Eigen::Vector3d offset=world_to_sampling*(sample.head<3>()-center.head<3>());
  if((offset.array()<p_.local_min.array()).any()||(offset.array()>p_.local_max.array()).any()){
    ++stats.outside_or_occupied;return false;
  }
  // Rrg::sampleVertex(RandomSampler&,...) accepts unknown support only when
  // the projected body is free. Its caller discards hanging after the batch
  // has ended. A strict terrain edge would instead retry these samples;
  // project() itself deliberately leaves their body clearance unchecked.
  const bool free=hanging
    ?sample.allFinite()&&map_->box(terrain_->baseState(sample).head<3>()+p_.center_offset,
                                  p_.robot_size)==VoxelStatus::Free
    :safe(sample,sample);
  if(!free){++stats.outside_or_occupied;return false;}
  ++stats.accepted_samples;return true;
}
void GlobalMemory::expandBackgroundSample(const StateVec& sample,const GainFunction& gain,BackgroundUpdate& stats){
  bool visited=false;
  for(const auto& state:visited_)if((sample-state).head<3>().squaredNorm()<=25.){visited=true;break;}
  if(visited){++stats.near_visited;return;}
  Vertex* near=nullptr;
  if(graph_->getNearestVertexInRange(&sample,5.,&near)){++stats.near_graph;return;}
  // The original frontier-radius test is redundant when both radii equal 5 m;
  // retain it explicitly to preserve the sequence and make provenance clear.
  std::vector<Vertex*> frontier_neighbors;graph_->getNearestVertices(&sample,5.,&frontier_neighbors);
  bool frontier=false;for(auto* vertex:frontier_neighbors)if(vertex->type==VertexType::kFrontier){frontier=true;break;}
  if(frontier){++stats.near_frontier;return;}
  ++stats.expansion_attempts;
  PlanResult expansion;
  const int previous_vertices=graph_->getNumVertices(),previous_edges=graph_->getNumEdges();
  const bool expanded=p_.ground_robot
    ?expandGround(*graph_,map_,p_,sample,expansion,kMaxEdges)
    :expandAerial(*graph_,map_,p_,sample,expansion,kMaxEdges);
  if(graph_->getNumVertices()!=previous_vertices||graph_->getNumEdges()!=previous_edges)
    ++snapshot_revision_;
  if(expanded){
    auto* vertex=graph_->getVertex(graph_->getNumVertices()-1);
    vertex->vol_gain=gain(vertex->state);
    if(vertex->vol_gain.is_frontier){vertex->type=VertexType::kFrontier;++stats.frontiers_added;}
    ++stats.added;
  }
  stats.collision_rejections+=expansion.collision_rejections;
}
BackgroundUpdate GlobalMemory::expandBackground(int budget,const GainFunction& gain){
  if(budget<0||budget>100000)throw std::invalid_argument("Invalid background attempt budget");
  const auto start=std::chrono::steady_clock::now();BackgroundUpdate stats;
  const auto finish=[&](const char* reason){
    stats.stop_reason=reason;
    stats.elapsed_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    return stats;
  };
  if(!map_)return finish("map_missing");
  if(budget==0)return finish("draw_budget");
  if(graph_->getNumVertices()==0)return finish("graph_empty");
  const auto centers=backgroundCenters();stats.clusters=centers.size();
  if(centers.empty())return finish("no_unvisited_vertices");
  const Eigen::Matrix3d world_to_sampling=p_.worldToSampling();
  // Preserve the existing deterministic entry point: one draw per cluster,
  // rotating after every rejection as well as every accepted sample.
  for(int attempt=0;attempt<budget&&graph_->getNumVertices()<kMaxVertices&&graph_->getNumEdges()<kMaxEdges;++attempt){
    if(attempt%centers.size()==0)++stats.rounds;
    ++stats.sampling_batches;StateVec sample;bool hanging=false;
    if(!drawBackgroundSample(centers[attempt%centers.size()],world_to_sampling,sample,hanging,stats))continue;
    if(hanging)++stats.hanging_rejections;
    else expandBackgroundSample(sample,gain,stats);
  }
  return finish(stats.attempts==static_cast<std::uint64_t>(budget)?"draw_budget":"graph_limit");
}
BackgroundUpdate GlobalMemory::expandBackgroundTimed(double seconds,const GainFunction& gain,
                                                     const BackgroundClock& clock){
  if(!std::isfinite(seconds)||seconds<=0.)throw std::invalid_argument("Invalid background time budget");
  const auto now=[&]{return clock?clock():std::chrono::steady_clock::now();};
  const auto start=now();BackgroundUpdate stats;
  const auto finish=[&](const char* reason){
    stats.stop_reason=reason;stats.elapsed_seconds=std::chrono::duration<double>(now()-start).count();
    stats.time_budget_reached=stats.elapsed_seconds>=seconds;return stats;
  };
  if(!map_)return finish("map_missing");
  if(graph_->getNumVertices()==0)return finish("graph_empty");
  const auto centers=backgroundCenters();stats.clusters=centers.size();
  if(centers.empty())return finish("no_unvisited_vertices");
  const Eigen::Matrix3d world_to_sampling=p_.worldToSampling();
  const auto full=[&]{return graph_->getNumVertices()>=kMaxVertices||graph_->getNumEdges()>=kMaxEdges;};
  constexpr int kSampleVertexDrawLimit=1000;
  double elapsed=0.;
  // Rrg checks the PREVIOUS reading at the outer condition, updates it at the
  // start of that round, and then finishes ALL clusters. This is deliberately
  // a soft budget, including when clustering itself takes longer than it.
  // No deadline test is added inside a cluster's sampleVertex retry batch.
  while(elapsed<seconds&&!full()){
    elapsed=std::chrono::duration<double>(now()-start).count();++stats.rounds;
    for(const auto& center:centers){
      if(full())break;
      ++stats.sampling_batches;
      for(int draw=0;draw<kSampleVertexDrawLimit;++draw){
        StateVec sample;bool hanging=false;
        if(!drawBackgroundSample(center,world_to_sampling,sample,hanging,stats))continue;
        // Rrg's callback discards hanging BEFORE sparse/history checks, but
        // AFTER sampleVertex returned true. Either kind of rejection finishes
        // this batch instead of granting it another 999 draws.
        if(hanging)++stats.hanging_rejections;
        else expandBackgroundSample(sample,gain,stats);
        break;
      }
    }
  }
  return finish(full()?"graph_limit":"time_budget");
}
}
