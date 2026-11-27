#include "gbplanner_local/ground_terrain.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace gbplanner2;
namespace {
std::shared_ptr<VoxelMap> scene(const std::string& shape){
  auto map=std::make_shared<VoxelMap>(.05,Eigen::Vector3d(-4,-1,-1),Eigen::Vector3i(160,40,80));
  for(int z=0;z<80;++z)for(int y=0;y<40;++y)for(int x=0;x<160;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);double height=0.;
    if(shape=="undulating")height=.25+.1*std::sin(1.3*point.x())+.05*std::cos(2.*point.y());
    else if(shape=="ramp")height=.5*std::clamp(point.x()+1.,0.,2.);
    else if(shape=="step")height=point.x()<0?0.:.4;
    VoxelStatus status=point.z()<height?VoxelStatus::Occupied:VoxelStatus::Free;
    if(shape=="gap"&&std::abs(point.x())<.75&&point.z()<.1)status=VoxelStatus::Unknown;
    map->set(index,status);
  }
  return map;
}
void number(double value){if(std::isfinite(value))std::cout<<value;else std::cout<<"null";}
void state(const StateVec& point){std::cout<<'['<<point[0]<<','<<point[1]<<','<<point[2]<<','<<point[3]<<']';}
void projection(const GroundProjection& result){
  std::cout<<"{\"status\":\""<<groundStatusName(result.status)<<"\",\"support\":"<<(result.support?"true":"false")
      <<",\"hanging\":"<<(result.hanging?"true":"false")<<",\"ground_z\":";number(result.ground_z);
  std::cout<<",\"ray_index\":"<<result.ray_index<<",\"state\":";state(result.projected_state);std::cout<<'}';
}
void edge(const GroundEdge& result,const TerrainProjector& terrain){
  std::cout<<"{\"status\":\""<<groundStatusName(result.status)<<"\",\"actual_length\":"<<result.actual_length
      <<",\"max_uphill\":"<<result.max_uphill<<",\"max_downhill\":"<<result.max_downhill
      <<",\"mean_inclination\":"<<result.mean_inclination<<",\"planning_polyline\":[";
  for(std::size_t i=0;i<result.polyline.size();++i){if(i)std::cout<<',';state(result.polyline[i]);}
  std::cout<<"],\"base_polyline\":[";
  for(std::size_t i=0;i<result.polyline.size();++i){if(i)std::cout<<',';state(terrain.baseState(result.polyline[i]));}
  std::cout<<"]}";
}
}

int main(){
  try{
    GroundParameters p;p.max_ground_height=.8;p.robot_height=.6;p.robot_size={.3,.3,.3};
    p.max_inclination=.9;p.max_negative_inclination=.9;
    TerrainProjector waves(scene("undulating"),p);
    const auto projected=waves.project(StateVec(0,0,2.,.3));
    const auto rolling=waves.edge(StateVec(-2.,0,2.,0),StateVec(2.,0,2.,0));
    if(!projected.support||!rolling.admissible()||rolling.polyline.size()<10)
      throw std::runtime_error("Undulating terrain did not produce a supported 3D route");
    double min_z=rolling.polyline.front().z(),max_z=min_z;
    for(const auto& point:rolling.polyline){min_z=std::min(min_z,point.z());max_z=std::max(max_z,point.z());}
    if(max_z-min_z<.1)throw std::runtime_error("Terrain route was flattened");

    p.max_negative_inclination=.35;TerrainProjector ramp(scene("ramp"),p);
    const auto uphill=ramp.edge(StateVec(-1.5,0,2.,0),StateVec(1.5,0,2.,0));
    const auto downhill=ramp.edge(StateVec(1.5,0,2.,0),StateVec(-1.5,0,2.,0));
    if(!uphill.admissible()||downhill.status!=GroundStatus::Steep)
      throw std::runtime_error("Ramp direction limits were not enforced");

    p.max_negative_inclination=.9;TerrainProjector ledge(scene("step"),p),gap(scene("gap"),p);
    const auto cliff=ledge.edge(StateVec(-1.,0,2.,0),StateVec(1.,0,2.,0));
    const auto unsupported=gap.project(StateVec(0,0,2.,0));
    const auto crossing=gap.edge(StateVec(-1.5,0,2.,0),StateVec(1.5,0,2.,0));
    if(cliff.status!=GroundStatus::Steep||!unsupported.hanging||crossing.admissible())
      throw std::runtime_error("A cliff or unknown support gap was accepted");
    // The first side ray finds known support; its raised origin must not
    // produce the original -0.5 m projected-height error.
    const auto side=gap.project(StateVec(.6,0,2.,0));
    if(!side.support||side.ray_index!=1||std::abs(side.projected_state.z()-.8)>1e-8)
      throw std::runtime_error("Side-ray support projection has a height bias");

    std::cout<<std::setprecision(15)<<"{\"resolution\":0.05,\"max_ground_height\":0.8,\"robot_height\":0.6,\"projection\":";
    projection(projected);std::cout<<",\"undulating\":";edge(rolling,waves);
    std::cout<<",\"uphill\":";edge(uphill,ramp);std::cout<<",\"downhill\":";edge(downhill,ramp);
    std::cout<<",\"cliff\":";edge(cliff,ledge);std::cout<<",\"unknown_projection\":";projection(unsupported);
    std::cout<<",\"unknown_crossing\":";edge(crossing,gap);std::cout<<",\"side_projection\":";projection(side);
    std::cout<<"}\n";return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
