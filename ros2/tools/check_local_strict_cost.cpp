// Small cost-reuse contract check; links the production planner library.
#include "gbplanner_local/local_planner.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
std::shared_ptr<VoxelMap> scene(bool ramp=false,bool hanging=false){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-4.,-2.,-1.),Eigen::Vector3i(80,40,40));
  for(int z=0;z<40;++z)for(int y=0;y<40;++y)for(int x=0;x<80;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    const double height=ramp?.3*std::clamp(point.x()+2.5,0.,4.):0.;
    auto status=point.z()<height?VoxelStatus::Occupied:VoxelStatus::Free;
    if(hanging&&point.x()>=0.&&point.z()<0.)status=VoxelStatus::Unknown;
    map->set(index,status);
  }
  return map;
}
Parameters parameters(double sample_x=1.){
  Parameters p;p.ground_robot=true;p.ground.max_ground_height=.8;p.ground.robot_height=.55;
  p.ground.max_inclination=.9;p.ground.max_negative_inclination=.9;
  p.robot_size={.2,.2,.2};p.edge_min=.05;p.edge_max=4.;p.overshoot=0.;p.graph_mode=false;
  p.local_min={-4.,-1.,-.5};p.local_max={4.,1.,1.};
  p.max_vertices=2;p.max_edges=8;p.max_attempts=2;
  p.sensor_range=1.;p.horizontal_resolution=.5;p.vertical_resolution=.5;
  for(auto& axis:p.axis_sampling){axis.distribution="constant";axis.constant=0.;}
  p.axis_sampling[0].constant=sample_x;return p;
}
std::pair<StateVec,StateVec> endpoints(const LocalPlanner& planner){
  require(planner.graph().vertices_map_.size()==2&&
          planner.graph().edge_map_.at(0).size()==1&&planner.graph().edge_map_.at(1).size()==1,
          "Expected one actual sampled graph edge");
  return {planner.graph().vertices_map_.at(0)->state,planner.graph().vertices_map_.at(1)->state};
}
}
int main(){try{
  auto map=scene();auto p=parameters();const StateVec root(-2.,0.,.8,.4);
  LocalPlanner planner(map,p,{},ExactRayGainReuse::ImmutableSnapshot);planner.plan(root);
  const auto [a,b]=endpoints(planner);TerrainProjector terrain(map,p.groundParameters());
  const auto forward=planner.validatedGroundLength(map,p.groundParameters(),0,1,a,b);
  const auto backward=planner.validatedGroundLength(map,p.groundParameters(),1,0,b,a);
  const auto fresh_forward=terrain.edge(a,b),fresh_backward=terrain.edge(b,a);
  require(forward&&backward&&fresh_forward.admissible()&&fresh_backward.admissible()&&
          *forward==fresh_forward.actual_length&&*backward==fresh_backward.actual_length,
          "Cached directed lengths differ from fresh strict terrain checks");

  StateVec yaw_a=a,yaw_b=b;yaw_a[3]=1.7;yaw_b[3]=-2.3;
  require(planner.validatedGroundLength(map,p.groundParameters(),0,1,yaw_a,yaw_b)==forward&&
          terrain.edge(yaw_a,yaw_b).actual_length==*forward,
          "Cost-only reuse incorrectly depends on sensor-scoring yaw");
  auto other_map=scene();
  require(!planner.validatedGroundLength(other_map,p.groundParameters(),0,1,a,b),
          "Equal contents of a different map were reused");
  require(!planner.validatedGroundLength(map,p.groundParameters(),1,0,a,b)&&
          !planner.validatedGroundLength(map,p.groundParameters(),0,9,a,b),
          "Wrong directed graph identity was reused");
  for(int axis=0;axis<3;++axis){
    StateVec changed=a;changed[axis]=std::nextafter(changed[axis],std::numeric_limits<double>::infinity());
    require(!planner.validatedGroundLength(map,p.groundParameters(),0,1,changed,b),
            "One-ULP source geometry mismatch was reused");
    changed=b;changed[axis]=std::nextafter(changed[axis],std::numeric_limits<double>::infinity());
    require(!planner.validatedGroundLength(map,p.groundParameters(),0,1,a,changed),
            "One-ULP target geometry mismatch was reused");
  }
  const std::vector<std::function<void(GroundParameters&)>> policy_changes{
    [](auto& v){v.max_ground_height+=.01;},[](auto& v){v.robot_height+=.01;},
    [](auto& v){v.center_offset.x()+=.01;},[](auto& v){v.center_offset.y()+=.01;},
    [](auto& v){v.center_offset.z()+=.01;},[](auto& v){v.robot_size.x()+=.01;},
    [](auto& v){v.robot_size.y()+=.01;},[](auto& v){v.robot_size.z()+=.01;},
    [](auto& v){v.point_robot=!v.point_robot;},[](auto& v){v.max_inclination+=.01;},
    [](auto& v){v.max_negative_inclination+=.01;},[](auto& v){v.projection_max_length+=.01;},
    [](auto& v){v.hanging_vertex_penalty+=.01;},
    [](auto& v){v.interpolate_projection_distance=!v.interpolate_projection_distance;},
    [](auto& v){v.ray_projection_mode=GroundRayProjectionMode::UpstreamSample;},
    [](auto& v){v.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;}
  };
  for(const auto& change:policy_changes){auto changed=p.groundParameters();change(changed);
    require(!planner.validatedGroundLength(map,changed,0,1,a,b),"Changed ground policy was reused");}
  LocalPlanner live(map,p);live.plan(root);const auto [live_a,live_b]=endpoints(live);
  require(!live.validatedGroundLength(map,p.groundParameters(),0,1,live_a,live_b),
          "Default mutable-map caller acquired an immutable cached verdict");

  auto ramp=scene(true);auto slope=p;slope.ground.max_negative_inclination=.1;
  LocalPlanner directed(ramp,slope,{},ExactRayGainReuse::ImmutableSnapshot);
  directed.plan(StateVec(-2.3,0.,.8,0.));const auto [lo,hi]=endpoints(directed);
  TerrainProjector slope_terrain(ramp,slope.groundParameters());
  const auto up=directed.validatedGroundLength(ramp,slope.groundParameters(),0,1,lo,hi);
  require(up&&slope_terrain.edge(lo,hi).admissible()&&
          *up==slope_terrain.edge(lo,hi).actual_length,
          "Uphill strict length was not reused exactly");
  require(!slope_terrain.edge(hi,lo).admissible()&&
          !directed.validatedGroundLength(ramp,slope.groundParameters(),1,0,hi,lo),
          "Reverse slope rejection became a positive cached cost");

  auto gap=scene(false,true);auto lookahead=parameters(3.);
  LocalPlanner candidate(gap,lookahead,{},ExactRayGainReuse::ImmutableSnapshot);
  candidate.plan(root);const auto [supported,hanging]=endpoints(candidate);
  require(candidate.graph().vertices_map_.at(1)->is_hanging,
          "Unknown-support fixture produced no hanging candidate");
  TerrainProjector gap_terrain(gap,lookahead.groundParameters());
  require(!gap_terrain.edge(supported,hanging).admissible()&&
          !candidate.validatedGroundLength(gap,lookahead.groundParameters(),0,1,supported,hanging)&&
          !candidate.validatedGroundLength(gap,lookahead.groundParameters(),1,0,hanging,supported),
          "Speculative hanging connection was reused as a strict edge");
  std::cout<<"{\"status\":\"passed\",\"directed_exact_costs\":true,\"yaw_cost_invariant\":true,"
    "\"map_identity_checked\":true,\"geometry_ulp_mismatches_rejected\":6,"
    "\"policy_variants_rejected\":"<<policy_changes.size()<<",\"mutable_map_opt_out\":true,"
    "\"reverse_slope_rejected\":true,\"hanging_rejected\":true}\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
