// Adapted from GBPlanner 2 Rrg::reconnectPathBlindly, BSD-3-Clause.
// Copyright (c) 2020 UNR Autonomous Robots Lab.
#pragma once
#include "gbplanner_local/ground_terrain.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <utility>

namespace gbplanner2 {
struct GroundReconnectionDiagnostics {
  bool attempted{false},modified{false};
  std::string reason{"not_requested"};
  int sampled_vertices{0},checked_edges{0},accepted_shortcuts{0},used_shortcuts{0},compressed_runs{0};
  double input_length{0.},output_length{0.};
};

// Rebuild only the requested return route, never the persistent global graph.
// Retain upstream's 0.2 m sampling, 0.1 m neighbors and 0.01 rad straight-run
// threshold. Each directed edge is terrain checked; a blind connection across
// an unknown floor, obstacle, or forbidden downhill slope is not executable.
// Work limits or an unavailable replacement leave the original path intact.
inline GroundReconnectionDiagnostics reconnectGroundPath(
    const TerrainProjector& terrain,std::vector<StateVec>& path,double& path_length){
  GroundReconnectionDiagnostics diagnostics;
  if(path.size()<=2){diagnostics.reason="path_too_short";return diagnostics;}
  diagnostics.attempted=true;
  constexpr double kResolution=.2,kRadius=.1,kYawEpsilon=.01,kEpsilon=1e-8;
  constexpr std::size_t kMaxVertices=20000,kMaxChecks=100000,kMaxKnots=200000;
  constexpr double kPi=3.14159265358979323846;
  const auto length=[](const std::vector<StateVec>& points){
    double total=0.;
    for(std::size_t i=1;i<points.size();++i)total+=(points[i]-points[i-1]).head<3>().norm();
    return total;
  };
  const auto fail=[&](const char* reason){diagnostics.reason=reason;return diagnostics;};
  if(path.size()>kMaxVertices)return fail("vertex_budget_exceeded");
  for(const auto& state:path)if(!state.allFinite())return fail("invalid_path_pose");
  diagnostics.input_length=diagnostics.output_length=length(path);
  if(!std::isfinite(diagnostics.input_length))return fail("invalid_path_length");
  try{
    std::vector<StateVec> samples{path.front()};
    for(std::size_t i=1;i<path.size();++i){
      const Eigen::Vector3d delta=(path[i]-path[i-1]).head<3>();
      const double distance=delta.norm();
      // Preserve each original bend and the final endpoint. Upstream's
      // interpolation omitted the endpoint of its last segment.
      if(distance/kResolution>double(kMaxVertices-samples.size()))return fail("vertex_budget_exceeded");
      for(double offset=kResolution;offset<distance-kEpsilon;offset+=kResolution){
        const double fraction=offset/distance;
        StateVec point=path[i-1];point.head<3>()+=fraction*delta;
        point[3]+=fraction*std::remainder(path[i][3]-point[3],2.*kPi);
        const auto projected=terrain.project(point);
        if(projected.status!=GroundStatus::Supported||
           (projected.projected_state-point).head<2>().norm()>kEpsilon)
          return fail("sample_not_supported");
        samples.push_back(projected.projected_state);
      }
      if(samples.size()>=kMaxVertices)return fail("vertex_budget_exceeded");
      samples.push_back(path[i]);
    }
    diagnostics.sampled_vertices=static_cast<int>(samples.size());
    struct Edge {int from,to;GroundEdge terrain;bool shortcut;};
    std::vector<Edge> edges;
    std::vector<std::vector<int>> adjacency(samples.size());
    std::size_t stored_knots=0;
    bool exhausted=false;
    const auto check=[&](int from,int to,GroundEdge& edge){
      if(diagnostics.checked_edges>=static_cast<int>(kMaxChecks)){exhausted=true;return false;}
      ++diagnostics.checked_edges;edge=terrain.edge(samples[from],samples[to]);
      if(!edge.admissible()||edge.polyline.empty()||!std::isfinite(edge.actual_length))return false;
      // Reprojection must not introduce an unchecked join or move home.
      // A TSDF estimate which changes these coordinates simply retains the
      // already validated input route instead of fabricating a vertical leg.
      if((edge.polyline.front()-samples[from]).head<3>().norm()>kEpsilon||
         (edge.polyline.back()-samples[to]).head<3>().norm()>kEpsilon)return false;
      return true;
    };
    const auto add=[&](int from,int to,bool shortcut){
      GroundEdge edge;if(!check(from,to,edge))return false;
      if(stored_knots+edge.polyline.size()>kMaxKnots){exhausted=true;return false;}
      stored_knots+=edge.polyline.size();
      adjacency[from].push_back(static_cast<int>(edges.size()));
      edges.push_back({from,to,std::move(edge),shortcut});
      if(shortcut)++diagnostics.accepted_shortcuts;
      return true;
    };
    // The original route remains a candidate. Reverse links are checked
    // separately because uphill and downhill limits need not be equal.
    for(int i=1;i<static_cast<int>(samples.size());++i){
      if(!add(i-1,i,false))return fail(exhausted?"edge_budget_exceeded":"resampled_path_unavailable");
      add(i,i-1,false);
      if(exhausted)return fail("edge_budget_exceeded");
    }
    using Cell=std::array<long long,3>;
    std::map<Cell,std::vector<int>> buckets;
    std::size_t candidate_pairs=0;
    for(int i=0;i<static_cast<int>(samples.size());++i){
      Cell cell{};
      for(int axis=0;axis<3;++axis){
        const double coordinate=std::floor(samples[i][axis]/kRadius);
        if(std::abs(coordinate)>1e15)return fail("invalid_map_coordinate");
        cell[axis]=static_cast<long long>(coordinate);
      }
      for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z){
        const Cell near{cell[0]+x,cell[1]+y,cell[2]+z};
        const auto bucket=buckets.find(near);if(bucket==buckets.end())continue;
        for(int j:bucket->second){
          if(++candidate_pairs>kMaxChecks)return fail("neighbor_budget_exceeded");
          if(j+1==i||(samples[i]-samples[j]).head<3>().squaredNorm()>kRadius*kRadius)continue;
          add(i,j,true);add(j,i,true);
          if(exhausted)return fail("edge_budget_exceeded");
        }
      }
      buckets[cell].push_back(i);
    }
    using QueueEntry=std::pair<double,int>;
    std::priority_queue<QueueEntry,std::vector<QueueEntry>,std::greater<QueueEntry>> queue;
    std::vector<double> distances(samples.size(),std::numeric_limits<double>::infinity());
    std::vector<int> incoming(samples.size(),-1);
    distances.front()=0.;queue.emplace(0.,0);
    while(!queue.empty()){
      const auto [distance,vertex]=queue.top();queue.pop();
      if(distance>distances[vertex])continue;
      if(vertex+1==static_cast<int>(samples.size()))break;
      for(int id:adjacency[vertex]){
        const auto& edge=edges[id];const double candidate=distance+edge.terrain.actual_length;
        if(candidate<distances[edge.to]){
          distances[edge.to]=candidate;incoming[edge.to]=id;queue.emplace(candidate,edge.to);
        }
      }
    }
    std::vector<int> route;
    for(int vertex=static_cast<int>(samples.size())-1;vertex!=0;){
      const int id=incoming[vertex];if(id<0)return fail("temporary_graph_unreachable");
      route.push_back(id);vertex=edges[id].from;
    }
    std::reverse(route.begin(),route.end());
    std::vector<StateVec> output{path.front()};
    const auto append=[&](const GroundEdge& edge,int from,int to){
      if(output.size()+edge.polyline.size()>kMaxKnots)return false;
      double traveled=0.;
      for(std::size_t k=1;k<edge.polyline.size();++k){
        traveled+=(edge.polyline[k]-edge.polyline[k-1]).head<3>().norm();
        auto state=edge.polyline[k];
        const double fraction=edge.actual_length>kEpsilon?std::min(1.,traveled/edge.actual_length):1.;
        state[3]=samples[from][3]+fraction*std::remainder(samples[to][3]-samples[from][3],2.*kPi);
        output.push_back(state);
      }
      // A zero-length connection may still carry a requested endpoint yaw.
      if(edge.polyline.size()==1)output.back()[3]=samples[to][3];
      return true;
    };
    for(std::size_t begin=0;begin<route.size();){
      std::size_t end=begin+1;
      const auto& first=edges[route[begin]];
      const Eigen::Vector2d delta=(samples[first.to]-samples[first.from]).head<2>();
      const double yaw=std::atan2(delta.y(),delta.x());
      double old_length=first.terrain.actual_length;
      while(end<route.size()){
        const auto& edge=edges[route[end]];
        const Eigen::Vector2d direction=(samples[edge.to]-samples[edge.from]).head<2>();
        if(direction.norm()>kEpsilon&&
           std::abs(std::remainder(std::atan2(direction.y(),direction.x())-yaw,2.*kPi))>kYawEpsilon)break;
        old_length+=edge.terrain.actual_length;++end;
      }
      GroundEdge compressed;
      const int from=first.from,to=edges[route[end-1]].to;
      // Compression uses a new checked terrain polyline, not an endpoint
      // chord: hills and depressions retain all their intermediate heights.
      const bool compress=end>begin+1&&check(from,to,compressed)&&compressed.actual_length<=old_length+kEpsilon;
      if(exhausted)return fail("edge_budget_exceeded");
      if(compress){
        if(!append(compressed,from,to))return fail("output_budget_exceeded");
        ++diagnostics.compressed_runs;
      }else for(std::size_t i=begin;i<end;++i){
        const auto& edge=edges[route[i]];
        if(!append(edge.terrain,edge.from,edge.to))return fail("output_budget_exceeded");
      }
      for(std::size_t i=begin;i<end;++i)if(edges[route[i]].shortcut)++diagnostics.used_shortcuts;
      begin=end;
    }
    if((output.front()-path.front()).head<3>().norm()>kEpsilon||
       (output.back()-path.back()).head<3>().norm()>kEpsilon)return fail("endpoint_changed");
    output.front()=path.front();
    if(output.size()==1&&path.front()[3]!=path.back()[3])output.push_back(path.back());
    else output.back()=path.back();
    const double output_length=length(output);
    if(!std::isfinite(output_length)||output_length>diagnostics.input_length+kEpsilon)return fail("replacement_not_shorter");
    diagnostics.modified=output.size()!=path.size();
    if(!diagnostics.modified)for(std::size_t i=0;i<path.size();++i)
      if((output[i]-path[i]).norm()>kEpsilon){diagnostics.modified=true;break;}
    diagnostics.output_length=output_length;
    diagnostics.reason=diagnostics.modified?"reconnected":"unchanged";
    path=std::move(output);path_length=output_length;
    return diagnostics;
  }catch(const std::exception&){return fail("reconnection_error");}
}
}  // namespace gbplanner2
