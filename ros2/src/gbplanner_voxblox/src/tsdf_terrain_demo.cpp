#include "gbplanner_voxblox/tsdf_backend.hpp"
#include <gbplanner_local/ground_terrain.hpp>
#include <voxblox/interpolator/interpolator.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace gbplanner2;
namespace {
void require(bool condition,const std::string& message){
  if(!condition)throw std::runtime_error(message);
}

// Analytic geometry supplies sensor returns only. All distances, occupancy,
// and terrain estimates below come from the original Voxblox integrator.
double surface(double x,double slope){return .13+slope*x;}
std::string scenario(const std::string& name,double slope,const std::string& distance_field,bool weight_dropoff=true){
  TsdfConfig config;config.resolution=.2;config.origin={-3.,-3.,-1.};config.dimensions={30,30,25};
  config.truncation_distance=.6;config.max_range=6.;config.method="simple";
  config.distance_field=distance_field;
  config.weight_dropoff=weight_dropoff;
  TsdfBackend backend(config);std::size_t accepted=0;
  for(const Eigen::Vector3d sensor:std::vector<Eigen::Vector3d>{{-1.,0.,3.},{0.,0.,3.},{1.,0.,3.}}){
    std::vector<Eigen::Vector3d> returns;returns.reserve(131*131);
    for(int iy=-65;iy<=65;++iy)for(int ix=-65;ix<=65;++ix){
      const double x=.04*ix,y=.04*iy;
      returns.emplace_back(Eigen::Vector3d(x,y,surface(x,slope))-sensor);
    }
    accepted+=backend.integrate(sensor,Eigen::Quaterniond::Identity(),returns).accepted_points;
  }
  const auto map=backend.snapshot();std::size_t distance_cells=0,known_cells=0,tsdf_only=0,occupancy_only=0;
  // Mirror native voxel values solely for an independent query comparison;
  // this does not expose or modify the backend's live TSDF layer.
  voxblox::Layer<voxblox::TsdfVoxel> reference(config.resolution,config.voxels_per_side);
  for(int z=0;z<config.dimensions.z();++z)for(int y=0;y<config.dimensions.y();++y)for(int x=0;x<config.dimensions.x();++x){
    const auto center=map->center({x,y,z});const auto* raw=backend.voxel(center);
    const bool observed=TsdfBackend::classify(raw,config.resolution,config.occupancy_factor)!=VoxelStatus::Unknown;
    const auto distance=map->tsdfDistance(center);
    require(bool(distance)==observed,"Snapshot changed the observed TSDF set");
    if(observed){
      ++distance_cells;
      require(std::abs(distance->distance-raw->distance)<1e-6,"Snapshot changed a native TSDF distance");
      require(std::abs(distance->weight-raw->weight)<1e-5,"Snapshot changed a native TSDF weight");
      const voxblox::Point position=center.cast<float>();
      reference.allocateBlockPtrByCoordinates(position)->getVoxelByCoordinates(position)=*raw;
    }
    const bool known=map->voxel(center)!=VoxelStatus::Unknown;
    if(known)++known_cells;
    if(observed&&!known)++tsdf_only;
    if(known&&!observed)++occupancy_only;
  }
  const voxblox::Interpolator<voxblox::TsdfVoxel> interpolator(&reference);
  std::size_t query_count=0,interpolated_count=0,fallback_count=0;double query_error=0.;
  for(double z:{.17,.41,.83,1.37})for(double y:{-.37,.11,.43})for(double x:{-.83,-.23,.17,.67}){
    const Eigen::Vector3d point(x,y,z);voxblox::FloatingPoint native_distance;
    const bool native_interpolated=interpolator.getDistance(point.cast<float>(),&native_distance,true);
    const bool native_known=native_interpolated||interpolator.getDistance(point.cast<float>(),&native_distance,false);
    const auto value=map->tsdfDistance(point);
    require(bool(value)==native_known,"TSDF query observed state differs from Voxblox interpolator");
    if(!native_known)continue;
    ++query_count;if(native_interpolated)++interpolated_count;else ++fallback_count;
    require(value->interpolated==native_interpolated,"TSDF query fallback differs from Voxblox interpolator");
    const double error=std::abs(value->distance-native_distance);query_error=std::max(query_error,error);
    require(error<1e-5,"TSDF query differs from Voxblox interpolator");
  }
  GroundParameters occupancy_parameters;occupancy_parameters.max_ground_height=1.;
  occupancy_parameters.robot_height=.8;occupancy_parameters.robot_size={.2,.2,.2};
  occupancy_parameters.projection_max_length=3.;
  GroundParameters distance_parameters=occupancy_parameters;
  distance_parameters.interpolate_projection_distance=true;
  const TerrainProjector occupancy(map,occupancy_parameters),distance(map,distance_parameters);
  double occupancy_error=0.,distance_error=0.,occupancy_max=0.,distance_max=0.;std::size_t samples=0;
  std::ostringstream details;details<<std::setprecision(10)<<'[';
  for(double y:{-.4,0.,.4})for(double x:{-.8,-.4,0.,.4,.8}){
    StateVec candidate;candidate<<x,y,1.6,0.;
    const auto discrete=occupancy.project(candidate),interpolated=distance.project(candidate);
    require(discrete.status==GroundStatus::Supported,"Occupancy projection failed at "+std::to_string(x)+","+std::to_string(y));
    require(interpolated.status==GroundStatus::Supported,"TSDF projection failed at "+std::to_string(x)+","+std::to_string(y));
    require(discrete.ray_index==0&&interpolated.ray_index==0,"Fixture unexpectedly used a side support ray");
    const auto repeated=distance.project(interpolated.projected_state);
    const auto connector=distance.edge(interpolated.projected_state,repeated.projected_state);
    require(connector.admissible(),std::string("Reprojected same-site connector: ")+
      groundStatusName(connector.status)+" dz="+
      std::to_string(repeated.projected_state.z()-interpolated.projected_state.z()));
    const double truth=surface(x,slope),error_occupancy=std::abs(discrete.ground_z-truth),error_distance=std::abs(interpolated.ground_z-truth);
    occupancy_error+=error_occupancy;distance_error+=error_distance;
    occupancy_max=std::max(occupancy_max,error_occupancy);distance_max=std::max(distance_max,error_distance);
    if(samples++)details<<',';
    details<<"{\"x\":"<<x<<",\"y\":"<<y<<",\"surface_z\":"<<truth
           <<",\"occupancy_z\":"<<discrete.ground_z<<",\"tsdf_z\":"<<interpolated.ground_z<<'}';
  }
  details<<']';
  require(distance_error<occupancy_error,"TSDF projection did not improve the terrain estimate");
  std::ostringstream output;output<<std::setprecision(10)
    <<"{\"name\":\""<<name<<"\",\"distance_field\":\""<<distance_field<<"\",\"slope\":"<<slope
    <<",\"scans\":3,\"accepted_points\":"<<accepted<<",\"tsdf_cells\":"<<distance_cells<<",\"occupancy_cells\":"<<known_cells
    <<",\"tsdf_only_cells\":"<<tsdf_only<<",\"occupancy_only_cells\":"<<occupancy_only<<",\"samples\":"<<samples
    <<",\"reference_queries\":"<<query_count<<",\"interpolated_queries\":"<<interpolated_count
    <<",\"fallback_queries\":"<<fallback_count<<",\"reference_max_error\":"<<query_error
    <<",\"occupancy_mean_absolute_error\":"<<occupancy_error/samples<<",\"tsdf_mean_absolute_error\":"<<distance_error/samples
    <<",\"occupancy_max_absolute_error\":"<<occupancy_max<<",\"tsdf_max_absolute_error\":"<<distance_max
    <<",\"projections\":"<<details.str()<<'}';
  return output.str();
}
}
int main(){
  try{
    const auto plane=scenario("plane",0.,"tsdf");
    const auto slope=scenario("slope",.12,"tsdf");
    const auto esdf=scenario("slope_esdf",.12,"esdf");
    const auto constant=scenario("plane_constant_weight",0.,"tsdf",false);
    std::cout<<"{\"resolution\":0.2,\"source\":\"Voxblox.integratePointCloud\",\"scenarios\":["
             <<plane<<','<<slope<<','<<esdf<<','<<constant<<"]}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
