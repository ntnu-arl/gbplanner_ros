// Exercise the distinct local / NoBound policies from Rrg's ray gain methods.
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/local_planner.hpp"
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){
  if(!condition)throw std::runtime_error(message);
}
bool sameGain(const VolumetricGain& a,const VolumetricGain& b){
  return a.num_unknown_voxels==b.num_unknown_voxels&&
         a.num_free_voxels==b.num_free_voxels&&
         a.num_occupied_voxels==b.num_occupied_voxels&&
         a.gain==b.gain&&a.is_frontier==b.is_frontier;
}
}

int main(){
  try{
    // The visible unknown slab begins at x=1. The local no-gain zone covers
    // that slab; the global workspace ends at x=2 while the sensor sees to x=4.
    auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-1.,-2.,-2.),
                                      Eigen::Vector3i(28,16,16));
    map->fill(VoxelStatus::Free);
    for(int x=0;x<28;++x)for(int y=0;y<16;++y)for(int z=0;z<16;++z){
      const Eigen::Vector3i index(x,y,z);
      if(map->center(index).x()>=1.)map->set(index,VoxelStatus::Unknown);
    }
    const auto bounded=std::make_shared<BoundedMap>(map,
        GlobalBound::axisAligned({-1.,-2.,-2.},{2.,2.,2.}));
    Parameters p;
    SensorParameters sensor;sensor.name="forward";sensor.range=4.;
    sensor.horizontal_fov=.2;sensor.vertical_fov=.2;
    sensor.horizontal_resolution=.1;sensor.vertical_resolution=.1;
    sensor.frontier_threshold=.05;p.sensors={sensor};
    p.gain.nonuniform_ray_cast=false;
    GainZone zone;zone.name="local_exclusion";
    zone.minimum={.75,-2.,-2.};zone.maximum={6.,2.,2.};p.no_gain_zones={zone};
    const StateVec viewpoint(.125,.125,.125,0.);

    LocalPlanner planner(bounded,p,{},ExactRayGainReuse::ImmutableSnapshot);
    const auto local=planner.rayGain(viewpoint);
    const auto global=planner.globalRayGain(viewpoint);
    require(local.gain==0.&&local.num_unknown_voxels==0&&!local.is_frontier,
            "Local no-gain zone did not suppress the unknown slab");
    require(global.gain>0.&&global.num_unknown_voxels>0&&global.is_frontier,
            "Global frontier gain incorrectly retained the local exclusion");
    require(sameGain(local,planner.rayGain(viewpoint))&&
            sameGain(global,planner.globalRayGain(viewpoint)),
            "Local and global cached gains contaminated one another");
    const auto separate_cache=planner.exactRayGainCacheStats();
    require(separate_cache.entries==2&&separate_cache.evaluations==2&&separate_cache.hits==2,
            "Distinct gain policies did not retain separate exact cache entries");

    LocalPlanner unbounded(map,p);
    const auto unrestricted=unbounded.globalRayGain(viewpoint);
    require(unrestricted.num_unknown_voxels>global.num_unknown_voxels,
            "Global workspace boundary did not clip distant unknown rewards");
    const auto known_only=std::make_shared<BoundedMap>(map,
        GlobalBound::axisAligned({-1.,-2.,-2.},{.75,2.,2.}));
    LocalPlanner outside(known_only,p);
    const auto outside_gain=outside.globalRayGain(viewpoint);
    require(outside_gain.num_unknown_voxels==0&&outside_gain.gain==0.&&!outside_gain.is_frontier,
            "NoBound counted unknown voxels outside the global workspace");

    // Local voxel mode uses its small ROI. NoBound must still cast sensor
    // rays over the larger global workspace, independently of that switch.
    Parameters voxel=p;voxel.gain.use_ray_model=false;
    voxel.local_min={-.4,-.4,-.4};voxel.local_max={.4,.4,.4};
    LocalPlanner voxel_planner(bounded,voxel);
    const auto local_voxel=voxel_planner.gain(viewpoint);
    const auto global_with_voxel=voxel_planner.globalRayGain(viewpoint);
    require(local_voxel.gain==0.&&sameGain(global_with_voxel,global),
            "Global frontier evaluation followed the local voxel mode or local ROI");

    // With no exclusions the two source methods are equivalent, so preserve
    // exact reuse between local scoring and the subsequent global refresh.
    Parameters plain=p;plain.no_gain_zones.clear();
    LocalPlanner shared(bounded,plain,{},ExactRayGainReuse::ImmutableSnapshot);
    const auto plain_local=shared.rayGain(viewpoint);
    const auto plain_global=shared.globalRayGain(viewpoint);
    const auto shared_cache=shared.exactRayGainCacheStats();
    require(sameGain(plain_local,global)&&sameGain(plain_global,global)&&
            shared_cache.entries==1&&shared_cache.evaluations==1&&shared_cache.hits==1,
            "Equivalent local/global policies no longer share exact evaluation");

    std::cout<<std::setprecision(12)
      <<"{\"local_ray_gain\":"<<local.gain
      <<",\"global_ray_gain\":"<<global.gain
      <<",\"global_unknown\":"<<global.num_unknown_voxels
      <<",\"unrestricted_unknown\":"<<unrestricted.num_unknown_voxels
      <<",\"outside_global_unknown\":"<<outside_gain.num_unknown_voxels
      <<",\"local_voxel_gain\":"<<local_voxel.gain
      <<",\"global_with_voxel_mode_gain\":"<<global_with_voxel.gain
      <<",\"separate_cache_entries\":"<<separate_cache.entries
      <<",\"shared_cache_entries\":"<<shared_cache.entries
      <<",\"passed\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
