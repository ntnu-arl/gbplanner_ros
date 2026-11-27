// Global spherical workspace: body geometry, planning, and gain filtering.
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/local_planner.hpp"
#include "gbplanner_local/search_planner.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>

using namespace gbplanner2;
using V3=Eigen::Vector3d;
namespace {
void require(bool condition,const char* reason){if(!condition)throw std::runtime_error(reason);}
std::shared_ptr<VoxelMap> freeMap(){
  auto map=std::make_shared<VoxelMap>(.2,V3(-5.,-5.,-4.),Eigen::Vector3i(50,50,40));
  map->fill(VoxelStatus::Free);return map;
}
using Ray=std::vector<std::pair<V3,VoxelStatus>>;
Ray scan(const MapQuery& map,const V3& start,const V3& end){
  Ray result;map.ray(start,end,[&](const V3& point,VoxelStatus status){
    result.emplace_back(point,status);return status!=VoxelStatus::Occupied;
  });return result;
}
}

int main(){
  try{
    const V3 center(.4,-.2,.3);const double radius=2.;
    const auto sphere=GlobalBound::sphere(center,radius);
    require(sphere.type()==GlobalBound::Type::Sphere&&sphere.radius()==radius&&
            sphere.center()==center&&sphere.halfSize()==V3::Constant(radius)&&
            sphere.rotation()==Eigen::Matrix3d::Identity(),"Sphere geometry was not fully initialized");
    require(sphere.minimum()==center-V3::Constant(radius)&&
            sphere.maximum()==center+V3::Constant(radius),"Sphere AABB is incorrect");
    require(sphere.contains(center+V3(radius,0.,0.))&&
            !sphere.contains(center+V3(radius+.001,0.,0.)),"Sphere radial boundary is incorrect");
    const auto cuboid=GlobalBound::axisAligned(sphere.minimum(),sphere.maximum());
    require(cuboid.type()==GlobalBound::Type::Cuboid&&cuboid.radius()==0.,"Cuboid factory changed shape");

    auto map=freeMap();auto bounded=std::make_shared<BoundedMap>(map,sphere);
    const V3 body(.4,.4,.4),crossing=center+V3(1.7,.7,0.);
    require(sphere.contains(crossing)&&cuboid.containsBox(crossing,body),
            "Crossing fixture must have a center inside the sphere and body inside the AABB");
    require(map->box(crossing,body)==VoxelStatus::Free&&
            !sphere.containsBox(crossing,body)&&
            bounded->box(crossing,body)==VoxelStatus::Occupied&&
            bounded->path(center,crossing,body)==VoxelStatus::Occupied,
            "A robot corner was allowed beyond the spherical shell");
    require(bounded->path(center+V3(-1.,0.,0.),center+V3(1.,0.,0.),body)==VoxelStatus::Free,
            "A complete interior swept body was rejected");
    const V3 outside=center+V3(1.7,1.7,0.);
    require(cuboid.contains(outside)&&!sphere.contains(outside)&&
            !bounded->gainAllowed(outside),"The sphere was replaced by its enclosing cuboid");

    Parameters parameters;parameters.robot_size=body;parameters.local_min={-2.7,-2.7,-2.7};
    parameters.local_max={2.7,2.7,2.7};parameters.max_vertices=48;parameters.max_edges=1200;
    parameters.max_attempts=2000;parameters.free_gain=1.;parameters.sensor_range=3.5;
    parameters.horizontal_resolution=M_PI/6;parameters.vertical_resolution=M_PI/6;
    const StateVec root(center.x(),center.y(),center.z(),.3);
    LocalPlanner local(bounded,parameters);const auto plan=local.plan(root);
    require(plan.success&&plan.path.size()>1&&plan.vertices>2,"Local planning inside the sphere failed");
    for(const auto& item:local.graph().vertices_map_)
      require(sphere.containsBox(item.second->state.head<3>(),body),"Local graph has an out-of-sphere robot body");
    double maximum_corner_radius=0.;int corner_checks=0;
    for(std::size_t edge=1;edge<plan.path.size();++edge){
      require(bounded->path(plan.path[edge-1].head<3>(),plan.path[edge].head<3>(),body)==VoxelStatus::Free,
              "Selected local path failed the bounded map");
      // Independently enumerate all body corners along the returned path.
      for(int sample=0;sample<=8;++sample){
        const V3 point=(1.-sample/8.)*plan.path[edge-1].head<3>()+
            (sample/8.)*plan.path[edge].head<3>();
        for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1}){
          const double distance=(point+V3(x*body.x(),y*body.y(),z*body.z())*.5-center).norm();
          maximum_corner_radius=std::max(maximum_corner_radius,distance);++corner_checks;
          require(distance<=radius+1e-12,"Returned route moves a body corner outside the sphere");
        }
      }
    }
    SearchParameters search;search.max_vertices=40;search.max_edges=400;search.max_attempts=400;
    search.empty_graph_cutoff=100;search.max_target_candidates=1;search.target_radius=.01;
    const StateVec target(outside.x(),outside.y(),outside.z(),0.);
    require(SearchPlanner(map,parameters,search).search(root,target).success,
            "Unbounded scene cannot reach the outside target");
    const auto refused=SearchPlanner(bounded,parameters,search).search(root,target);
    require(!refused.success,"Search reached an outside target through the spherical boundary");

    // Put unknown observations only in voxels wholly outside the sphere.
    // The margin avoids classifying an inside query as unknown merely because
    // its voxel center lies just beyond the continuous spherical boundary.
    auto gain_map=freeMap();const double half_diagonal=std::sqrt(3.)*gain_map->resolution()*.5;
    for(int z=0;z<40;++z)for(int y=0;y<50;++y)for(int x=0;x<50;++x){
      const Eigen::Vector3i index(x,y,z);
      if((gain_map->center(index)-center).norm()>radius+half_diagonal+1e-9)
        gain_map->set(index,VoxelStatus::Unknown);
    }
    auto gain_bounded=std::make_shared<BoundedMap>(gain_map,sphere);
    auto gain_parameters=parameters;gain_parameters.free_gain=0.;
    int outside_ray_voxels=0,outside_volume_voxels=0;
    for(const bool ray_model:{true,false}){
      gain_parameters.gain.use_ray_model=ray_model;
      const auto raw=LocalPlanner(gain_map,gain_parameters).gain(root);
      const auto filtered=LocalPlanner(gain_bounded,gain_parameters).gain(root);
      require(raw.num_unknown_voxels>0&&raw.gain>0.,"Outside observations have no reference reward");
      require(filtered.num_unknown_voxels==0&&filtered.gain==0.,"Sphere admitted observation reward from outside");
      if(ray_model)outside_ray_voxels=raw.num_unknown_voxels;
      else outside_volume_voxels=raw.num_unknown_voxels;
    }

    // Gain extensions enlarge the fixed observation ROI, not the local samples.
    auto extension_map=freeMap();
    for(int z=0;z<40;++z)for(int y=0;y<50;++y)for(int x=0;x<50;++x){
      const Eigen::Vector3i index(x,y,z);
      if((extension_map->center(index)-center).norm()>1.2)
        extension_map->set(index,VoxelStatus::Unknown);
    }
    auto extension_bounded=std::make_shared<BoundedMap>(extension_map,sphere);
    auto base_parameters=gain_parameters;base_parameters.gain.use_ray_model=false;
    base_parameters.local_min=V3::Constant(-.5);base_parameters.local_max=V3::Constant(.5);
    auto extended_parameters=base_parameters;
    extended_parameters.gain_min_extension=V3::Constant(-1.);
    extended_parameters.gain_max_extension=V3::Constant(1.);
    const auto base_gain=LocalPlanner(extension_bounded,base_parameters).gain(root);
    const auto extended_gain=LocalPlanner(extension_bounded,extended_parameters).gain(root);
    require(base_gain.num_unknown_voxels==0&&base_gain.gain==0.&&
            extended_gain.num_unknown_voxels>0&&extended_gain.gain>base_gain.gain,
            "Gain extension did not expose unknown voxels beyond the sampling ROI");
    LocalSampler base_sampler(base_parameters),extended_sampler(extended_parameters);
    for(int sample=0;sample<32;++sample){
      const StateVec base=base_sampler.generate(root),extended=extended_sampler.generate(root);
      require(base==extended&&((base.head<3>()-center).cwiseAbs().array()<=.5+1e-12).all(),
              "Gain extension changed the local sample sequence or its bounds");
    }

    const V3 wall=center+V3(2.8,0.,0.);gain_map->set(gain_map->index(wall),VoxelStatus::Occupied);
    require(gain_bounded->voxel(wall)==VoxelStatus::Occupied&&!gain_bounded->gainAllowed(wall),
            "Workspace policy changed the observed outside wall");
    const auto raw_ray=scan(*gain_map,center,center+V3(3.5,0.,0.));
    const auto bounded_ray=scan(*gain_bounded,center,center+V3(3.5,0.,0.));
    require(!raw_ray.empty()&&raw_ray.back().second==VoxelStatus::Occupied&&raw_ray.size()==bounded_ray.size(),
            "Sphere changed physical ray occlusion");
    for(std::size_t i=0;i<raw_ray.size();++i)
      require(raw_ray[i].first==bounded_ray[i].first&&raw_ray[i].second==bounded_ray[i].second,
              "Sphere changed the physical ray samples");

    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"radius\":"<<radius
      <<",\"local_vertices\":"<<plan.vertices<<",\"path_length\":"<<plan.path_length
      <<",\"corner_checks\":"<<corner_checks<<",\"maximum_corner_radius\":"<<maximum_corner_radius
      <<",\"outside_target_rejected\":true,\"crossing_body_rejected\":true"
      <<",\"outside_ray_reward_voxels\":"<<outside_ray_voxels
      <<",\"outside_volume_reward_voxels\":"<<outside_volume_voxels
      <<",\"base_roi_unknown_voxels\":"<<base_gain.num_unknown_voxels
      <<",\"extended_roi_unknown_voxels\":"<<extended_gain.num_unknown_voxels
      <<",\"base_roi_gain\":"<<base_gain.gain<<",\"extended_roi_gain\":"<<extended_gain.gain
      <<",\"gain_extension_preserves_sampling\":true,\"identical_local_samples\":32"
      <<",\"bounded_unknown_reward\":0,\"outside_wall_occlusion_preserved\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
