// Offline replay of run10's recorded routes on its failure map revision 350.
// Uses the installed production PathRefiner; no ROS node or vehicle is started.
#include "gbplanner_local/path_refinement.hpp"
#include "gbplanner_local/parameters.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include "gbplanner_local/global_bound.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gbplanner2;
namespace {
constexpr std::size_t kCells=93330,kStride=24;
constexpr double kResolution=.1;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void key(std::istream& file,const char* expected){std::string actual;file>>actual;
  require(bool(file)&&actual==expected,"Unexpected fixture field");}
void vector3(std::istream& file,Eigen::Vector3d& value){for(int j=0;j<3;++j)file>>value[j];
  require(bool(file)&&value.allFinite(),"Invalid fixture vector");}
float floatLE(const uint8_t* p){
  static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"Requires IEEE float32");
  const uint32_t bits=uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);
  float value;std::memcpy(&value,&bits,sizeof(value));return value;
}
struct Snapshot {std::shared_ptr<SparseVoxelMap> map;std::vector<Eigen::Vector3d> occupied;};
Snapshot readMap(const char* filename){
  std::ifstream file(filename,std::ios::binary|std::ios::ate);
  require(bool(file)&&file.tellg()==std::streamoff(kCells*kStride),"Expected exact 93330-cell run10 snapshot");
  std::vector<uint8_t> bytes(kCells*kStride);file.seekg(0);
  require(bool(file.read(reinterpret_cast<char*>(bytes.data()),bytes.size())),"Cannot read saved map");
  Snapshot snapshot;snapshot.map=std::make_shared<SparseVoxelMap>(kResolution);
  for(std::size_t i=0;i<kCells;++i){
    const auto* cell=bytes.data()+i*kStride;
    const Eigen::Vector3d point(floatLE(cell),floatLE(cell+4),floatLE(cell+8));
    require(point.allFinite(),"Nonfinite saved voxel");
    const auto index=snapshot.map->index(point);const auto center=snapshot.map->center(index);
    require((center.cast<float>().cast<double>()-point).cwiseAbs().maxCoeff()<1.e-6,"Off-lattice saved voxel");
    int8_t status;std::memcpy(&status,cell+12,1);
    require(status==-1||status==0||status==100,"Invalid saved occupancy");
    snapshot.map->set(index,static_cast<VoxelStatus>(status));
    snapshot.map->setTsdf(index,floatLE(cell+16),floatLE(cell+20));
    if(status==100)snapshot.occupied.push_back(center);
  }
  require(snapshot.map->size()==kCells,"Duplicate or missing saved voxels");return snapshot;
}
struct Fixture {
  Parameters parameters;Eigen::Vector3d minimum,maximum;
  std::array<std::vector<StateVec>,3> routes;
};
Fixture readFixture(const char* filename){
  std::ifstream file(filename);require(bool(file),"Cannot open extracted fixture");Fixture f;
  auto& p=f.parameters;p.ground_robot=true;
  key(file,"map350");std::size_t cells=0;file>>cells;require(cells==kCells,"Wrong fixture map size");
  key(file,"robot_size");vector3(file,p.robot_size);
  key(file,"center_offset");vector3(file,p.center_offset);
  key(file,"ground");file>>p.ground.robot_height>>p.ground.max_ground_height
    >>p.ground.max_inclination>>p.ground.max_negative_inclination>>p.ground.projection_max_length;
  p.ground.interpolate_projection_distance=true;
  p.ground.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossing;
  key(file,"global_min");vector3(file,f.minimum);key(file,"global_max");vector3(file,f.maximum);
  key(file,"refinement");p.refinement.enabled=true;vector3(file,p.refinement.safety_extension);
  file>>p.refinement.interpolation_distance>>p.refinement.relaxed_corridor_multiplier;
  for(std::size_t route=0;route<f.routes.size();++route){
    key(file,route==0?"commanded":route==1?"planner":"failure_request");
    std::size_t count=0;file>>count;require(count>=2&&count<=20000,"Invalid fixture route size");
    for(std::size_t i=0;i<count;++i){StateVec state;
      for(int j=0;j<4;++j)file>>state[j];
      require(bool(file)&&state.allFinite(),"Invalid fixture pose");
      // Saved ROS routes use base_link height. PathRefiner uses planning height.
      state.z()+=p.ground.max_ground_height-p.ground.robot_height;f.routes[route].push_back(state);
    }
  }
  std::string extra;require(!(file>>extra),"Trailing fixture data");p.validate();return f;
}
const char* statusName(VoxelStatus value){return value==VoxelStatus::Free?"free":
  value==VoxelStatus::Occupied?"occupied":"unknown";}
void stateJson(const StateVec& value){std::cout<<'['<<value[0]<<','<<value[1]<<','<<value[2]<<','<<value[3]<<']';}
void routeJson(const std::vector<StateVec>& path,double height_offset){
  std::cout<<'[';for(std::size_t i=0;i<path.size();++i){if(i)std::cout<<',';
    StateVec state=path[i];state.z()+=height_offset;stateJson(state);}std::cout<<']';
}
double routeLength(const std::vector<StateVec>& path){double length=0.;
  for(std::size_t i=1;i<path.size();++i)length+=(path[i]-path[i-1]).head<3>().norm();
  return length;}

struct Clearance {double distance{std::numeric_limits<double>::infinity()};
  Eigen::Vector3d cell{Eigen::Vector3d::Zero()};std::size_t segment{0};double fraction{0.};};
// Exact minimum XY box-to-voxel distance along each linear segment, restricted
// to the part where their Z intervals overlap. This is an occupied-voxel
// diagnostic, not an unknown-space, planning-bound or tracking-margin proof.
Clearance clearance(const std::vector<StateVec>& path,const Parameters& p,
                    const std::vector<Eigen::Vector3d>& occupied){
  Clearance result;const Eigen::Vector3d half=p.robot_size*.5;
  const double height=p.ground.robot_height-p.ground.max_ground_height;
  for(std::size_t i=0;i+1<path.size()||i==0;++i){
    const std::size_t next=std::min(i+1,path.size()-1);
    Eigen::Vector3d start=path[i].head<3>()+p.center_offset;start.z()+=height;
    const Eigen::Vector3d delta=(path[next]-path[i]).head<3>();
    for(const auto& cell:occupied){
      const Eigen::Vector3d lo=cell-Eigen::Vector3d::Constant(kResolution*.5)-half;
      const Eigen::Vector3d hi=cell+Eigen::Vector3d::Constant(kResolution*.5)+half;
      double first=0.,last=1.;
      if(std::abs(delta.z())<1.e-15){if(start.z()<lo.z()||start.z()>hi.z())continue;}
      else{double a=(lo.z()-start.z())/delta.z(),b=(hi.z()-start.z())/delta.z();
        if(a>b)std::swap(a,b);
        first=std::max(first,a);last=std::min(last,b);if(first>last)continue;}
      std::vector<double> cuts{first,last};
      for(int axis=0;axis<2;++axis)if(std::abs(delta[axis])>1.e-15)
        for(double face:{lo[axis],hi[axis]}){const double at=(face-start[axis])/delta[axis];
          if(at>first&&at<last)cuts.push_back(at);}
      std::sort(cuts.begin(),cuts.end());
      const auto evaluate=[&](double at){const auto point=(start+at*delta).eval();double squared=0.;
        for(int axis=0;axis<2;++axis){const double d=std::max({lo[axis]-point[axis],0.,point[axis]-hi[axis]});squared+=d*d;}
        const double distance=std::sqrt(squared);if(distance<result.distance){
          result.distance=distance;result.cell=cell;result.segment=i;result.fraction=at;}};
      for(double at:cuts)evaluate(at);
      for(std::size_t j=1;j<cuts.size();++j){const double middle=.5*(cuts[j-1]+cuts[j]);
        double quadratic=0.,linear=0.;for(int axis=0;axis<2;++axis){
          const double value=start[axis]+middle*delta[axis];
          if(value>=lo[axis]&&value<=hi[axis])continue;
          const double face=value<lo[axis]?lo[axis]:hi[axis];
          quadratic+=delta[axis]*delta[axis];linear+=delta[axis]*(start[axis]-face);}
        if(quadratic>0.)evaluate(std::clamp(-linear/quadratic,cuts[j-1],cuts[j]));}
    }
    if(next+1>=path.size())break;
  }
  return result;
}
void clearanceJson(const Clearance& value){
  if(!std::isfinite(value.distance)){std::cout<<"null";return;}
  std::cout<<"{\"distance_m\":"<<value.distance<<",\"nearest_occupied_voxel_center\":["
    <<value.cell.x()<<','<<value.cell.y()<<','<<value.cell.z()<<"],\"segment\":"<<value.segment
    <<",\"segment_fraction\":"<<value.fraction<<'}';
}
void metricsJson(const std::vector<StateVec>& path,const Parameters& p,
                 const std::shared_ptr<const MapQuery>& map,const std::vector<Eigen::Vector3d>& occupied){
  const double offset=p.ground.robot_height-p.ground.max_ground_height;
  std::cout<<"{\"points\":"<<path.size()<<",\"length_m\":"<<routeLength(path)<<",\"path_planning_height\":";
  routeJson(path,0.);std::cout<<",\"path_base_height\":";routeJson(path,offset);
  std::cout<<",\"body_sweeps\":[";
  for(std::size_t i=0;i+1<path.size();++i){if(i)std::cout<<',';
    Eigen::Vector3d a=path[i].head<3>()+p.center_offset,b=path[i+1].head<3>()+p.center_offset;
    a.z()+=offset;b.z()+=offset;std::cout<<'"'<<statusName(map->path(a,b,p.robot_size))<<'"';}
  std::cout<<"],\"occupied_voxel_xy_clearance\":";clearanceJson(clearance(path,p,occupied));
  std::cout<<",\"endpoint_occupied_voxel_xy_clearance\":";
  clearanceJson(clearance({path.back()},p,occupied));std::cout<<'}';
}
}  // namespace
int main(int argc,char** argv){try{
  require(argc==3,"Usage: path_safety_replay RUN10_MAP.bin EXTRACTED_INPUT.txt");
  const auto snapshot=readMap(argv[1]);const auto fixture=readFixture(argv[2]);const auto& p=fixture.parameters;
  const auto map=std::make_shared<BoundedMap>(snapshot.map,GlobalBound::axisAligned(fixture.minimum,fixture.maximum));
  PathRefiner refiner(map,p,p.refinement);
  std::cout<<std::setprecision(17)<<"{\"status\":\"completed\",\"map_revision\":350,\"cells\":"
    <<snapshot.map->size()<<",\"occupied_cells\":"<<snapshot.occupied.size()
    <<",\"relaxed\":false,\"preserve_endpoint\":false,\"cases\":[";
  for(std::size_t i=0;i<fixture.routes.size();++i){if(i)std::cout<<',';
    const auto& input=fixture.routes[i];const auto output=refiner.refine(input,false,false);
    std::cout<<"{\"name\":\""<<(i==0?"commanded_route":i==1?"planner_route":"failure_request")
      <<"\",\"success\":"<<(output.success?"true":"false")<<",\"modified\":"<<(output.modified?"true":"false")
      <<",\"reason\":\""<<output.reason<<"\",\"removed_vertices\":"<<output.removed_vertices
      <<",\"adjusted_vertices\":"<<output.adjusted_vertices<<",\"endpoint_xy_displacement_m\":"
      <<(output.path.back()-input.back()).head<2>().norm()<<",\"input\":";
    metricsJson(input,p,map,snapshot.occupied);std::cout<<",\"output\":";
    metricsJson(output.path,p,map,snapshot.occupied);std::cout<<'}';
  }
  std::cout<<"]}\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
