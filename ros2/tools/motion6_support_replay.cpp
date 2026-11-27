// Offline diagnosis of motion6's frozen validation on accepted map revision 155.
// Calls the installed production TerrainProjector. Starts no ROS/vehicle process.
#include "gbplanner_local/ground_terrain.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
float floatLE(const uint8_t* p){
  const uint32_t bits=uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);
  float value;std::memcpy(&value,&bits,sizeof(value));return value;
}
std::shared_ptr<SparseVoxelMap> readMap(const char* filename){
  constexpr std::size_t cells=53388,stride=24;
  std::ifstream file(filename,std::ios::binary|std::ios::ate);
  require(bool(file)&&file.tellg()==std::streamoff(cells*stride),"Expected motion6 map155 (53388 cells)");
  std::vector<uint8_t> bytes(cells*stride);file.seekg(0);
  require(bool(file.read(reinterpret_cast<char*>(bytes.data()),bytes.size())),"Cannot read accepted map");
  auto map=std::make_shared<SparseVoxelMap>(.1);
  for(std::size_t i=0;i<cells;++i){const auto* c=bytes.data()+i*stride;
    const Eigen::Vector3d point(floatLE(c),floatLE(c+4),floatLE(c+8));
    const auto index=map->index(point);int8_t status;std::memcpy(&status,c+12,1);
    map->set(index,static_cast<VoxelStatus>(status));map->setTsdf(index,floatLE(c+16),floatLE(c+20));
  }
  require(map->size()==cells,"Duplicate or missing cells");return map;
}
struct Query {Eigen::Vector3d point;std::optional<TsdfDistance> value;};
class TraceMap final:public MapQuery {
 public:
  explicit TraceMap(std::shared_ptr<SparseVoxelMap> map):map_(std::move(map)){}
  double resolution()const override{return map_->resolution();}
  Eigen::Vector3d minimum()const override{return map_->minimum();}
  Eigen::Vector3d maximum()const override{return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{return map_->voxel(p);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p)const override{
    auto result=map_->tsdfDistance(p);queries.push_back({p,result});return result;
  }
  VoxelStatus box(const Eigen::Vector3d& p,const Eigen::Vector3d& size)const override{return map_->box(p,size);}
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override{
    return map_->path(a,b,size);
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{map_->ray(a,b,visit);}
  mutable std::vector<Query> queries;
 private:std::shared_ptr<SparseVoxelMap> map_;
};
void vectorJson(const Eigen::Vector3d& p){std::cout<<'['<<p.x()<<','<<p.y()<<','<<p.z()<<']';}
void number(double value){if(std::isfinite(value))std::cout<<value;else std::cout<<"null";}
void traceJson(const std::vector<Query>& queries){
  std::cout<<'[';bool first=true;for(const auto& q:queries){if(!first)std::cout<<',';first=false;
    std::cout<<"{\"point\":";vectorJson(q.point);std::cout<<",\"distance\":";
    if(q.value)number(q.value->distance);else std::cout<<"null";
    std::cout<<",\"weight\":";if(q.value)number(q.value->weight);else std::cout<<"null";
    std::cout<<",\"interpolated\":"<<(q.value&&q.value->interpolated?"true":"false")<<'}';
  }std::cout<<']';
}
void projectJson(const std::string& name,const StateVec& state,TerrainProjector& terrain,TraceMap& map){
  map.queries.clear();const auto result=terrain.project(state);
  std::cout<<"{\"name\":\""<<name<<"\",\"input_planning_xyz\":";vectorJson(state.head<3>());
  std::cout<<",\"status\":\""<<groundStatusName(result.status)<<"\",\"ray_index\":"<<result.ray_index
    <<",\"projected_xyz\":";vectorJson(result.projected_state.head<3>());
  std::cout<<",\"ground_z\":";number(result.ground_z);
  std::cout<<",\"queries\":";traceJson(map.queries);std::cout<<'}';
}
void edgeJson(const std::string& name,const StateVec& a,const StateVec& b,TerrainProjector& terrain,TraceMap& map){
  map.queries.clear();const auto result=terrain.validateFixedEdge(a,b,true);
  std::cout<<"{\"name\":\""<<name<<"\",\"status\":\""<<groundStatusName(result.status)
    <<"\",\"queries\":";traceJson(map.queries);std::cout<<'}';
}
}
int main(int argc,char** argv){try{
  require(argc==2,"Usage: motion6_support_replay motion6_accepted.bin");
  const auto raw=readMap(argv[1]);const auto map=std::make_shared<TraceMap>(raw);
  GroundParameters p;p.robot_height=.24;p.max_ground_height=.54;
  p.robot_size={1.17,1.17,.25};p.center_offset={0,0,.015};
  p.max_inclination=.4363323129985824;p.max_negative_inclination=.4363323129985824;
  p.projection_max_length=1.5;p.interpolate_projection_distance=true;
  p.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossing;
  TerrainProjector terrain(map,p);
  StateVec current,old;current<<.817455530166626,.17979110777378082,.401286780834198,-1.3478479005734028;
  old<<.8087021112442017,-.0003776156227104366,.4013959467411041,.006418558852157355;
  current.z()+=.30;old.z()+=.30;
  std::cout<<std::setprecision(17)<<"{\"status\":\"completed\",\"map_revision\":155,\"cells\":"<<raw->size()
    <<",\"body_sweep_status\":"<<static_cast<int>(map->path(terrain.baseState(current).head<3>()+p.center_offset,
      terrain.baseState(old).head<3>()+p.center_offset,p.robot_size))<<",\"edges\":[";
  edgeJson("frozen_connector",current,old,terrain,*map);std::cout<<',';
  edgeJson("current_self",current,current,terrain,*map);std::cout<<',';
  edgeJson("old_self",old,old,terrain,*map);
  std::cout<<"],\"projections\":[";
  const int count=std::max(1,static_cast<int>(std::ceil((old-current).head<3>().norm()/.05)));
  for(int i=0;i<=count;++i){if(i)std::cout<<',';
    StateVec sample=current+(double(i)/count)*(old-current);
    projectJson("connector_sample_"+std::to_string(i),sample,terrain,*map);
  }
  std::cout<<"]}\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
