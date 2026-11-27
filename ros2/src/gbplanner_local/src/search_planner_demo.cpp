#include "gbplanner_local/search_planner.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
std::shared_ptr<VoxelMap> aerial(bool wall,bool gap){
  auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-5,-5,-1.5),Eigen::Vector3i(40,40,12));
  map->fill(VoxelStatus::Free);
  if(wall)for(int z=0;z<12;++z)for(int y=0;y<40;++y)for(int x=0;x<40;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    if(std::abs(point.x())<.26&&(!gap||point.y()<1.||point.y()>2.5))map->set(index,VoxelStatus::Occupied);
  }
  return map;
}
void validatePath(const MapQuery& map,const Parameters& p,const SearchResult& result){
  require(result.success&&!result.path.empty(),"Search did not return a path");
  for(std::size_t i=1;i<result.path.size();++i)
    require(map.path(result.path[i-1].head<3>()+p.center_offset,
                     result.path[i].head<3>()+p.center_offset,p.robot_size)==VoxelStatus::Free,
            "Search returned an obstructed segment");
}
void print(const char* label,const SearchResult& result){
  std::cout<<'"'<<label<<"\":{\"success\":"<<(result.success?"true":"false")
      <<",\"reason\":\""<<result.reason<<"\",\"exact_target\":"<<(result.exact_target?"true":"false")
      <<",\"endpoint_distance\":"<<result.endpoint_distance<<",\"path_length\":"<<result.path_length
      <<",\"vertices\":"<<result.vertices<<",\"edges\":"<<result.edges
      <<",\"attempts\":"<<result.attempts<<",\"target_candidates\":"<<result.target_candidates
      <<",\"sampling_rounds\":"<<result.sampling_rounds
      <<",\"empty_graph_cutoff_reached\":"<<(result.empty_graph_cutoff_reached?"true":"false")
      <<",\"path_points\":"<<result.path.size()<<'}';
}
}
int main(){
  try{
    Parameters p;p.local_min={-2.5,-3.5,-.6};p.local_max={6.5,4.5,.6};
    p.robot_size={.25,.25,.25};p.edge_min=.12;p.edge_max=.8;p.overshoot=0.;
    p.neighbor_radius=1.1;p.neighbor_min=.12;p.neighbor_max=1.1;p.seed=42;
    SearchParameters search;search.target_radius=.6;search.max_vertices=900;search.max_attempts=20000;
    const StateVec source(-2.,-.5,0.,.2),target(2.,-.5,0.,0.);
    auto open=aerial(false,false);SearchPlanner straight(open,p,search);
    const auto direct=straight.search(source,target);
    validatePath(*open,p,direct);
    require(direct.exact_target&&direct.attempts==0&&direct.vertices==2&&
            std::abs(direct.path_length-4.)<1e-8,"Direct search did not take the immediate connection");

    Parameters fixed_heading=p;fixed_heading.tangent_yaw=false;
    const StateVec angled_target(2.,1.5,0.,-1.2);
    SearchPlanner straight_fixed(open,fixed_heading,search);
    const auto direct_fixed=straight_fixed.search(source,angled_target);
    validatePath(*open,fixed_heading,direct_fixed);
    require(direct_fixed.exact_target&&direct_fixed.attempts==0&&direct_fixed.path.size()==2&&
            std::abs(direct_fixed.path.front()[3]-source[3])<1e-12&&
            std::abs(direct_fixed.path.back()[3]-std::atan2(2.,4.))<1e-12,
            "Direct aerial search must align the endpoint even when tangent yaw is disabled");
    const StateVec vertical_target(source.x(),source.y(),.75,-.9);
    const auto vertical=straight_fixed.search(source,vertical_target);
    validatePath(*open,fixed_heading,vertical);
    require(vertical.exact_target&&vertical.attempts==0&&vertical.path.size()==2&&
            std::abs(vertical.path.front()[3]-source[3])<1e-12&&
            std::abs(vertical.path.back()[3]-vertical_target[3])<1e-12,
            "Vertical direct search must preserve heading without a horizontal tangent");

    auto obstacle=aerial(true,true);SearchPlanner detour(obstacle,p,search);
    const auto routed=detour.search(source,target);validatePath(*obstacle,p,routed);
    require(routed.attempts>0&&routed.path_length>4.3,"Sampling did not find a genuine detour");
    double max_y=-100.;for(const auto& state:routed.path)max_y=std::max(max_y,state.y());
    require(max_y>1.,"Search bypassed the wall without using its passage");

    SearchPlanner detour_fixed(obstacle,fixed_heading,search);
    const auto routed_fixed=detour_fixed.search(source,target);validatePath(*obstacle,fixed_heading,routed_fixed);
    require(routed_fixed.attempts>0&&routed_fixed.path.size()==routed.path.size(),
            "Tangent yaw option unexpectedly changed the sampled route");
    bool retained_non_tangent=false;
    for(std::size_t i=0;i<routed_fixed.path.size();++i){
      const auto& state=routed_fixed.path[i];
      require((state-routed.path[i]).head<3>().norm()<1e-10,
              "Tangent yaw option changed a sampled waypoint position");
      bool preserved=false;
      for(const auto& item:detour_fixed.graph().vertices_map_){
        if((state-item.second->state).head<3>().norm()<1e-10){
          preserved=std::abs(state[3]-item.second->state[3])<1e-12;break;
        }
      }
      require(preserved,"Sampled search changed graph yaw with tangent correction disabled");
      if(i>0){
        const Eigen::Vector3d delta=(state-routed_fixed.path[i-1]).head<3>();
        if(delta.head<2>().norm()>1e-10&&
           std::abs(std::remainder(state[3]-std::atan2(delta.y(),delta.x()),2.*std::acos(-1.)))>1e-6)
          retained_non_tangent=true;
      }
    }
    require(retained_non_tangent,"Sampled search fixture did not distinguish the tangent yaw option");

    search.target_radius=1.;SearchPlanner nearby(obstacle,p,search);
    const auto neighborhood=nearby.search(source,StateVec(0.,-.5,0.,0.));
    validatePath(*obstacle,p,neighborhood);
    require(!neighborhood.exact_target&&neighborhood.reason=="target_neighborhood"&&
            neighborhood.endpoint_distance<search.target_radius,"Occupied target did not return explicit nearby fallback");

    search.target_radius=.6;auto closed=aerial(true,false);SearchPlanner sealed(closed,p,search);
    const auto blocked=sealed.search(source,target);
    require(!blocked.success&&blocked.path.empty(),"Sealed wall search unexpectedly succeeded");

    auto isolated=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-5,-5,-1.5),Eigen::Vector3i(40,40,12));
    isolated->fill(VoxelStatus::Occupied);
    for(int z=0;z<12;++z)for(int y=0;y<40;++y)for(int x=0;x<40;++x){
      const Eigen::Vector3i index(x,y,z);if((isolated->center(index)-source.head<3>()).norm()<.5)
        isolated->set(index,VoxelStatus::Free);
    }
    Parameters isolated_p=p;isolated_p.local_min={1.,-1.,-.5};isolated_p.local_max={2.,1.,.5};
    SearchParameters cutoff=search;cutoff.empty_graph_cutoff=80;cutoff.max_attempts=160;
    SearchPlanner rejected(isolated,isolated_p,cutoff);const auto exhausted=rejected.search(source,target);
    require(!exhausted.success&&exhausted.attempts==cutoff.max_attempts&&exhausted.vertices==1&&
            exhausted.sampling_rounds==0&&!exhausted.empty_graph_cutoff_reached,
            "Rejected samples must consume only the raw search budget");

    auto terrain=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-3,-2,-1),Eigen::Vector3i(60,40,40));
    for(int z=0;z<40;++z)for(int y=0;y<40;++y)for(int x=0;x<60;++x){
      const Eigen::Vector3i index(x,y,z);const auto point=terrain->center(index);
      const double height=.2+.12*std::sin(1.2*point.x());
      terrain->set(index,point.z()<height?VoxelStatus::Occupied:VoxelStatus::Free);
    }
    Parameters ground=p;ground.ground_robot=true;ground.ground.max_ground_height=.8;
    ground.ground.robot_height=.6;ground.ground.max_inclination=.9;ground.ground.max_negative_inclination=.9;
    TerrainProjector projector(terrain,ground.groundParameters());
    const auto a=projector.project(StateVec(-2.,0,1.5,0.));
    const auto b=projector.project(StateVec(2.,0,1.5,0.));
    SearchPlanner rolling(terrain,ground,search);const auto followed=rolling.search(a.projected_state,b.projected_state);
    require(followed.success&&followed.exact_target&&followed.path.size()>10,
            "Ground search did not retain its terrain polyline");
    double low=100.,high=-100.;
    for(const auto& state:followed.path){low=std::min(low,state.z());high=std::max(high,state.z());}
    require(high-low>.1,"Ground search flattened terrain");
    std::cout<<std::setprecision(12)<<'{';print("direct",direct);std::cout<<',';print("gap",routed);
    std::cout<<',';print("direct_tangent_disabled",direct_fixed);std::cout<<',';print("vertical_direct",vertical);
    std::cout<<',';print("gap_tangent_disabled",routed_fixed);
    std::cout<<",\"direct_tangent_disabled_yaw\":"<<direct_fixed.path.back()[3]
             <<",\"vertical_direct_yaw\":"<<vertical.path.back()[3];
    std::cout<<',';print("blocked_target",neighborhood);std::cout<<',';print("sealed",blocked);
    std::cout<<',';print("rejected_samples",exhausted);std::cout<<',';print("ground",followed);
    std::cout<<",\"ground_elevation_span\":"<<high-low<<"}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
