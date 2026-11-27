// Compare the ROS 2 gain-only traversal with the corrected upstream method 1.
#include "gbplanner_local/local_planner.hpp"
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
using Entry=std::pair<Eigen::Vector3d,VoxelStatus>;
using Counts=std::array<std::size_t,3>;
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
Counts counts(const std::vector<Entry>& log){
  Counts result{};
  for(const auto& entry:log)++result[entry.second==VoxelStatus::Unknown?0:entry.second==VoxelStatus::Free?1:2];
  return result;
}
Counts counts(const VolumetricGain& gain){
  return {static_cast<std::size_t>(gain.num_unknown_voxels),static_cast<std::size_t>(gain.num_free_voxels),
          static_cast<std::size_t>(gain.num_occupied_voxels)};
}

// Independent transcription of voxblox_alt_impl.cpp:562-726. Keep its reversed
// y traversal, repeated log entries, float steps and neighbor assignments.
// Correct only the cache's (row,column) indexing, x+1 bounds, an explicit hit
// flag at distance zero, and clipping inherited distance to this ray's range.
std::vector<Entry> correctedOriginal(const MapQuery& map,const Eigen::Vector3d& pos,
    const std::vector<Eigen::Vector3d>& endpoints,int width,int height,
    double h_resolution,double v_resolution,bool nonuniform,double multiplier){
  std::vector<std::vector<double>> starting_points(height,std::vector<double>(width,0.));
  std::vector<std::vector<bool>> occupied_start(height,std::vector<bool>(width,false));
  std::vector<Entry> log;
  const float voxel_size=map.resolution();
  for(int y=height-1;y>=0;--y)for(int x=0;x<width;++x){
    float step_size=voxel_size*multiplier;
    const float original_step_size=step_size;
    const int endpoint_index=(height-y-1)*width+x;
    Eigen::Vector3d ray=endpoints[endpoint_index]-pos;
    const double ray_norm=ray.norm();
    ray/=ray_norm;
    const double cached=starting_points[y][x];
    const double ray_start_point=std::min(cached,ray_norm);
    if(occupied_start[y][x]&&cached<=ray_norm){
      log.emplace_back(pos+ray*ray_start_point,VoxelStatus::Occupied);continue;
    }
    double prev_step_dist=ray_start_point;
    for(double step=ray_start_point;step<=ray_norm;step+=step_size){
      if(nonuniform&&std::abs(prev_step_dist-step)>4.&&step<=12.){
        prev_step_dist=step;step_size+=original_step_size;
      }
      const Eigen::Vector3d point=pos+ray*step;
      const auto status=map.voxel(point.cast<float>().cast<double>());
      const int new_voxels_added=std::ceil(step_size/original_step_size);
      for(int i=0;i<new_voxels_added;++i)log.emplace_back(point,status);
      if(status==VoxelStatus::Unknown)continue;
      const double dx=step*h_resolution,dy=step*v_resolution;
      const bool occupied=status==VoxelStatus::Occupied;
      if(dx<=voxel_size&&x+1<width){
        starting_points[y][x+1]=std::min(ray_norm,step);occupied_start[y][x+1]=occupied;
      }
      if(dy<=voxel_size&&y>0){
        starting_points[y-1][x]=std::min(ray_norm,step);occupied_start[y-1][x]=occupied;
      }
      if(std::sqrt(dx*dx+dy*dy)<=voxel_size&&x+1<width&&y>0){
        starting_points[y-1][x+1]=std::min(ray_norm,step);occupied_start[y-1][x+1]=occupied;
      }
      if(occupied)break;
    }
  }
  return log;
}
std::shared_ptr<VoxelMap> scene(){
  auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-1.,-6.,-3.),Eigen::Vector3i(80,48,24));
  map->fill(VoxelStatus::Free);
  for(int z=0;z<24;++z)for(int y=0;y<48;++y)for(int x=0;x<80;++x){
    const Eigen::Vector3i index(x,y,z);const auto point=map->center(index);
    if(point.x()>=5.)map->set(index,VoxelStatus::Unknown);
    if(point.x()>=10.&&point.x()<11.)map->set(index,VoxelStatus::Occupied);
  }
  return map;
}
void printCounts(const Counts& value){std::cout<<'['<<value[0]<<','<<value[1]<<','<<value[2]<<']';}
}

int main(){
  try{
    auto map=scene();Parameters parameters;
    SensorParameters sensor;sensor.name="nonsquare";sensor.range=15.;
    sensor.horizontal_fov=.42;sensor.vertical_fov=.36;
    sensor.horizontal_resolution=.06;sensor.vertical_resolution=.12;
    parameters.sensors={sensor};const StateVec root(.125,.125,.125,0.);
    const LocalPlanner planner(map,parameters);
    const auto endpoints=planner.sensorEndpoints(0,root);
    require(endpoints.size()==21,"Expected a non-square 7 by 3 endpoint grid");
    const auto compare=[&](const MapQuery& source,bool nonuniform,double multiplier){
      const auto expected=correctedOriginal(source,root.head<3>(),endpoints,7,3,.06,.12,nonuniform,multiplier);
      std::vector<Entry> actual,baseline;
      GainRaycastDiagnostics diagnostics,baseline_diagnostics;
      for(bool reuse_invariants:{false,true}){
        actual.clear();
        diagnostics=iterativeGainRaycast(source,root.head<3>(),endpoints,7,3,.06,.12,
            {nonuniform,multiplier,1000000,reuse_invariants},[&](const Eigen::Vector3d& point,VoxelStatus status,std::size_t weight){
              for(std::size_t i=0;i<weight;++i)actual.emplace_back(point,status);
            });
        require(actual.size()==expected.size(),"Weighted sample count differs from corrected method 1");
        for(std::size_t i=0;i<actual.size();++i)
          require(actual[i].second==expected[i].second&&(actual[i].first-expected[i].first).norm()<1e-12,
                  "Sample order, position or occupancy differs from corrected method 1");
        if(!reuse_invariants){baseline=actual;baseline_diagnostics=diagnostics;continue;}
        require(actual.size()==baseline.size(),"Invariant reuse changes sample count");
        for(std::size_t i=0;i<actual.size();++i)
          require(actual[i].second==baseline[i].second&&(actual[i].first.array()==baseline[i].first.array()).all(),
                  "Invariant reuse changes exact sample sequence");
        require(diagnostics.queries==baseline_diagnostics.queries&&diagnostics.weighted_samples==baseline_diagnostics.weighted_samples&&
                diagnostics.reused_free_prefixes==baseline_diagnostics.reused_free_prefixes&&
                diagnostics.reused_occupied_rays==baseline_diagnostics.reused_occupied_rays&&
                diagnostics.step_increases==baseline_diagnostics.step_increases,"Invariant reuse changes traversal diagnostics");
      }
      return std::make_pair(diagnostics,counts(actual));
    };
    const auto nonuniform=compare(*map,true,1.);
    const auto uniform=compare(*map,false,1.);
    const auto half_step=compare(*map,true,.5);
    require(nonuniform.first.step_increases>0&&nonuniform.first.reused_free_prefixes>0&&
            nonuniform.first.weighted_samples>nonuniform.first.queries,
            "Nonuniform weighting or free-prefix reuse was not exercised");
    require(nonuniform.first.queries<uniform.first.queries&&uniform.first.step_increases==0&&
            half_step.first.queries!=nonuniform.first.queries,
            "Ray parameters do not change actual sample work");
    const auto gains=planner.sensorGains(root);
    require(gains.size()==1&&gains[0].ray_method=="iterative"&&gains[0].ray_width==7&&gains[0].ray_height==3&&
            counts(gains[0].gain)==nonuniform.second&&gains[0].raycast.queries==nonuniform.first.queries,
            "LocalPlanner did not use the configured iterative helper");
    require(nonuniform.second[0]>0&&nonuniform.second[2]>0,"Mixed scene did not exercise unknown and occupied samples");
    const auto blocked=correctedOriginal(*map,root.head<3>(),endpoints,7,3,.06,.12,true,1.);
    for(const auto& entry:blocked)require(entry.first.x()<11.,"Gain ray continued behind the occupied wall");

    // Compatibility mode follows the original ROS 2 DDA callback exactly.
    parameters.gain.ray_method="dda";
    const auto dda=LocalPlanner(map,parameters).sensorGains(root).front();
    std::vector<Entry> legacy;
    for(const auto& endpoint:endpoints)map->ray(root.head<3>(),endpoint,[&](const auto& point,VoxelStatus status){
      legacy.emplace_back(point,status);return status!=VoxelStatus::Occupied;
    });
    require(dda.ray_method=="dda"&&counts(dda.gain)==counts(legacy)&&
            dda.raycast.queries==legacy.size()&&dda.raycast.reused_free_prefixes==0,
            "Explicit DDA compatibility mode changed its gain counts");

    // The old negative-distance cache lost an obstacle at the ray origin (-0).
    map->set(map->index(root.head<3>()),VoxelStatus::Occupied);
    const auto zero_hit=compare(*map,true,1.);
    require(zero_hit.second==Counts{0,0,21}&&zero_hit.first.reused_occupied_rays>0&&
            zero_hit.first.queries<21,"Zero-distance occupied reuse was lost");
    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"reference_comparisons\":8,\"exact_invariant_mode_comparisons\":4,\"grid\":[7,3]"
      <<",\"nonuniform_queries\":"<<nonuniform.first.queries<<",\"uniform_queries\":"<<uniform.first.queries
      <<",\"half_step_queries\":"<<half_step.first.queries<<",\"weighted_samples\":"<<nonuniform.first.weighted_samples
      <<",\"reused_free_prefixes\":"<<nonuniform.first.reused_free_prefixes
      <<",\"step_increases\":"<<nonuniform.first.step_increases<<",\"unknown_free_occupied\":";
    printCounts(nonuniform.second);
    std::cout<<",\"dda_queries\":"<<dda.raycast.queries<<",\"zero_distance_reused_hits\":"<<zero_hit.first.reused_occupied_rays
      <<",\"wall_occlusion\":true,\"local_planner_integrated\":true,\"dda_preserved\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
