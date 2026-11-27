// Compare the production single-worker adapter with the original threaded
// SimpleTsdfIntegrator. Link to the installed Voxblox library; no ROS executor.
#include "gbplanner_voxblox/inline_simple_integrator.hpp"
#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Layer=voxblox::Layer<voxblox::TsdfVoxel>;
using Config=voxblox::TsdfIntegratorBase::Config;
using Clock=std::chrono::steady_clock;
void require(bool value,const char* message) {
  if(!value)throw std::runtime_error(message);
}
bool sameBits(float a,float b) {
  return std::memcmp(&a,&b,sizeof(float))==0;
}
struct Counts {std::size_t snapshots{0},blocks{0},voxels{0};};
void compare(const Layer& baseline,const Layer& candidate,Counts& counts) {
  voxblox::BlockIndexList indices;
  baseline.getAllAllocatedBlocks(&indices);
  require(indices.size()==candidate.getNumberOfAllocatedBlocks(),"Allocated block count changed");
  for(const auto& index:indices) {
    require(candidate.hasBlock(index),"Allocated block index changed");
    const auto& a=baseline.getBlockByIndex(index);
    const auto& b=candidate.getBlockByIndex(index);
    require(a.num_voxels()==b.num_voxels()&&a.has_data()==b.has_data()&&a.updated()==b.updated(),
            "Block shape or update metadata changed");
    for(std::size_t i=0;i<a.num_voxels();++i) {
      const auto& x=a.getVoxelByLinearIndex(i);
      const auto& y=b.getVoxelByLinearIndex(i);
      require(sameBits(x.distance,y.distance)&&sameBits(x.weight,y.weight)&&
              x.color.r==y.color.r&&x.color.g==y.color.g&&
              x.color.b==y.color.b&&x.color.a==y.color.a,
              "TSDF distance/weight/color bits changed");
      ++counts.voxels;
    }
    ++counts.blocks;
  }
  ++counts.snapshots;
}
void clearUpdateFlags(Layer& layer) {
  voxblox::BlockIndexList indices;layer.getAllAllocatedBlocks(&indices);
  for(const auto& index:indices)layer.getBlockByIndex(index).updated().reset();
}
struct Scan {
  voxblox::Transformation transform;
  voxblox::Pointcloud points;
  voxblox::Colors colors;
  bool freespace{false};
};
std::vector<Scan> scans(std::size_t count,std::size_t points_per_scan) {
  std::vector<Scan> result;
  for(std::size_t scan=0;scan<count;++scan) {
    Scan value;
    const Eigen::Quaternionf rotation(Eigen::AngleAxisf(float(scan)*.027f,Eigen::Vector3f::UnitZ()));
    value.transform=voxblox::Transformation(voxblox::Rotation(rotation),
        voxblox::Point(-.19f+.037f*scan,.113f-.017f*scan,.061f));
    value.freespace=scan%4==3;
    value.points.reserve(points_per_scan+4);value.colors.reserve(points_per_scan+4);
    for(std::size_t i=0;i<points_per_scan;++i) {
      // A deterministic nonuniform cloud visits overlapping rays repeatedly,
      // including capped weights, near/far returns and negative coordinates.
      const float azimuth=float((i*977)%4093)*.00153513f;
      const float elevation=float(int((i*37)%101)-50)*.008f;
      const float range=.7f+float((i*43+scan*19)%130)*.047f;
      value.points.emplace_back(range*std::cos(azimuth)*std::cos(elevation),
          range*std::sin(azimuth)*std::cos(elevation),range*std::sin(elevation)+.021f);
      value.colors.emplace_back(uint8_t(i*7+scan),uint8_t(i*13),uint8_t(i*29),uint8_t(255));
    }
    // Retain explicitly short and axial returns; both variants use the same
    // already-installed RayCaster correction in the core shared library.
    for(const auto& p:std::array<voxblox::Point,4>{{{.01f,.01f,.01f},
        {2.f,0.f,0.f},{0.f,-2.f,0.f},{0.f,0.f,6.f}}}) {
      value.points.push_back(p);value.colors.emplace_back(11,23,37);
    }
    result.push_back(std::move(value));
  }
  return result;
}
Config config(const std::string& order) {
  Config c;c.integrator_threads=1;c.integration_order_mode=order;
  c.default_truncation_distance=.45f;c.min_ray_length_m=.1f;c.max_ray_length_m=4.5f;
  c.max_weight=3.f;c.clearing_ray_weight_factor=.1f;c.use_weight_dropoff=true;
  return c;
}
template<class Integrator> double run(Integrator& integrator,const std::vector<Scan>& inputs) {
  const auto started=Clock::now();
  for(const auto& scan:inputs)
    integrator.integratePointCloud(scan.transform,scan.points,scan.colors,scan.freespace);
  return std::chrono::duration<double>(Clock::now()-started).count();
}
}  // namespace

int main() {
  try {
    Counts counts;
    const auto inputs=scans(8,1536);
    std::size_t cases=0;
    for(const std::string order:{"mixed","sorted"})for(int policy=0;policy<3;++policy) {
      auto c=config(order);
      if(policy==1) {c.voxel_carving_enabled=false;c.allow_clear=true;}
      if(policy==2) {c.allow_clear=false;c.use_symmetric_weight_dropoff=true;c.use_const_weight=true;}
      Layer original(.15f,8),inline_layer(.15f,8);
      voxblox::SimpleTsdfIntegrator baseline(c,&original);
      gbplanner2::InlineSimpleTsdfIntegrator candidate(c,&inline_layer);
      require(baseline.getConfig().allow_clear==candidate.getConfig().allow_clear,
              "Base constructor configuration changed");
      voxblox::Pointcloud empty_points;voxblox::Colors empty_colors;
      baseline.integratePointCloud(voxblox::Transformation(),empty_points,empty_colors);
      candidate.integratePointCloud(voxblox::Transformation(),empty_points,empty_colors);
      compare(original,inline_layer,counts);
      for(const auto& scan:inputs) {
        clearUpdateFlags(original);clearUpdateFlags(inline_layer);
        baseline.integratePointCloud(scan.transform,scan.points,scan.colors,scan.freespace);
        candidate.integratePointCloud(scan.transform,scan.points,scan.colors,scan.freespace);
        compare(original,inline_layer,counts);
      }
      ++cases;
    }
    // The >1 branch delegates unchanged. One ray avoids nondeterministic
    // competing updates so its exact output can still be compared.
    auto parallel_config=config("mixed");parallel_config.integrator_threads=2;
    Layer parallel_original(.15f,8),parallel_candidate(.15f,8);
    voxblox::SimpleTsdfIntegrator parallel_baseline(parallel_config,&parallel_original);
    gbplanner2::InlineSimpleTsdfIntegrator parallel_adapter(parallel_config,&parallel_candidate);
    voxblox::Pointcloud single{{1.2f,.7f,.4f}};voxblox::Colors color{{17,33,65}};
    parallel_baseline.integratePointCloud(voxblox::Transformation(),single,color);
    parallel_adapter.integratePointCloud(voxblox::Transformation(),single,color);
    compare(parallel_original,parallel_candidate,counts);

    // One paired throughput sample, not a stability or native-system claim.
    const auto benchmark_inputs=scans(20,4096);
    auto c=config("mixed");
    Layer original(.15f,8),inline_layer(.15f,8);
    voxblox::SimpleTsdfIntegrator baseline(c,&original);
    gbplanner2::InlineSimpleTsdfIntegrator candidate(c,&inline_layer);
    const double before=run(baseline,benchmark_inputs);
    const double after=run(candidate,benchmark_inputs);
    compare(original,inline_layer,counts);
    std::cout<<std::setprecision(12)
        <<"{\"status\":\"passed\",\"configuration_cases\":"<<cases
        <<",\"compared_snapshots\":"<<counts.snapshots<<",\"compared_blocks\":"<<counts.blocks
        <<",\"compared_voxels\":"<<counts.voxels
        <<",\"tsdf_and_color_bit_exact\":true,\"block_metadata_exact\":true"
        <<",\"multiple_worker_fallback_checked\":true,\"benchmark_scans\":"<<benchmark_inputs.size()
        <<",\"threaded_wall_s\":"<<before<<",\"inline_wall_s\":"<<after
        <<",\"benchmark_scope\":\"One sequential synthetic-cloud pair; no native throughput guarantee\"}\n";
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';return 1;
  }
}
