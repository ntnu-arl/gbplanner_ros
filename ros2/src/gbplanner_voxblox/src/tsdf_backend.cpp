#include "gbplanner_voxblox/tsdf_backend.hpp"
#include "gbplanner_voxblox/map_file.hpp"
#include "gbplanner_voxblox/inline_simple_integrator.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <filesystem>
#include <limits>
#include <unistd.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

namespace gbplanner2 {
TsdfBackend::TsdfBackend(const TsdfConfig& config):config_(config) {
  auto positive=[](double value){return std::isfinite(value)&&value>0.;};
  if(!positive(config.resolution)||!positive(config.truncation_distance)||
     config.truncation_distance<config.resolution||!positive(config.min_range)||
     !positive(config.max_range)||config.max_range<=config.min_range||
     !positive(config.max_weight)||!positive(config.clearing_weight)||config.clearing_weight>1.||
     !std::isfinite(config.occupancy_factor)||config.occupancy_factor<0.||
     config.threads<1||config.threads>64||config.voxels_per_side<2||config.voxels_per_side>64||
     (config.voxels_per_side&(config.voxels_per_side-1))!=0)
    throw std::invalid_argument("Invalid TSDF parameters");
  if(config.method!="simple"&&config.method!="merged"&&config.method!="fast")
    throw std::invalid_argument("Unknown TSDF integration method");
  config_.use_const_weight=config_.use_const_weight.value_or(!config_.weight_dropoff);
  if(!*config_.use_const_weight&&!config_.weight_dropoff&&!config_.use_symmetric_weight_dropoff)
    throw std::invalid_argument("At least one TSDF truncation-band weighting policy is required");
  if(config.integration_order_mode!="mixed"&&config.integration_order_mode!="sorted")
    throw std::invalid_argument("TSDF integration order must be mixed or sorted");
  if(!positive(config.start_voxel_subsampling_factor)||
     !std::isfinite(config.max_integration_time_s)||config.max_integration_time_s<0.||
     config.max_consecutive_ray_collisions<0||config.clear_checks_every_n_frames<1)
    throw std::invalid_argument("Invalid Fast TSDF integration parameters");
  if(!std::isfinite(config.mesh_min_weight)||config.mesh_min_weight<0.)
    throw std::invalid_argument("Mesh minimum weight must be finite and nonnegative");
  if(config.storage!="dense"&&config.storage!="sparse")
    throw std::invalid_argument("map.storage must be dense or sparse");
  if(config.storage=="dense"){
    // Voxblox's lattice is anchored at world zero. Misaligned snapshots would
    // sample different cells and misclassify an occupied boundary as free.
    if(!config.origin.allFinite())throw std::invalid_argument("Invalid grid origin");
    for(int i=0;i<3;++i){
      const double index=config.origin[i]/config.resolution;
      if(std::abs(index-std::round(index))>1e-6||std::abs(index)>1e6)
        throw std::invalid_argument("Planner grid must align with the Voxblox lattice");
    }
    VoxelMap shape(config.resolution,config.origin,config.dimensions);
    if(shape.size()>2000000)throw std::invalid_argument("Snapshot grid exceeds two million cells");
  }
  if(config.distance_field!="tsdf"&&config.distance_field!="esdf")
    throw std::invalid_argument("distance_field must be tsdf or esdf");
  if(config.distance_field=="esdf"&&!config.esdf_update_on_scan)
    throw std::invalid_argument("ESDF planning requires esdf_update_on_scan=true");
  if(!positive(config.esdf_max_distance)||config.esdf_max_distance<config.resolution)
    throw std::invalid_argument("ESDF maximum distance must exceed one voxel");
  // Resolve the original ROS server defaults once and retain the effective
  // values across reset/load and integrator reconstruction. Earlier versions
  // used one voxel for the fixed band and max_distance for default_distance.
  if(config_.esdf_min_distance==-1.)
    config_.esdf_min_distance=config_.truncation_distance/2.;
  if(!positive(config_.esdf_min_distance)||!positive(config_.esdf_default_distance)||
     !std::isfinite(config_.esdf_min_diff)||config_.esdf_min_diff<0.)
    throw std::invalid_argument("Invalid ESDF fixed-band, default distance or minimum change");
  config_.esdf_default_distance=std::max(config_.esdf_default_distance,config_.esdf_max_distance);
  if(!positive(config.esdf_clear_sphere_radius)||!positive(config.esdf_occupied_sphere_radius)||
     config.esdf_occupied_sphere_radius<config.esdf_clear_sphere_radius)
    throw std::invalid_argument("Invalid ESDF clear/occupied sphere radii");
  if(config.distance_field=="esdf")config_.esdf_enabled=true;
  initialize();
}
void TsdfBackend::initialize(){
  // Recreate the integrator as well: the fast integrator keeps per-frame hashes.
  integrator_.reset();esdf_integrator_.reset();esdf_layer_.reset();
  layer_=std::make_unique<voxblox::Layer<voxblox::TsdfVoxel>>(config_.resolution,config_.voxels_per_side);
  initializeIntegrators();
}
void TsdfBackend::initializeIntegrators(){
  last_integration_timing_={};
  voxblox::TsdfIntegratorBase::Config parameters;
  parameters.default_truncation_distance=config_.truncation_distance;
  parameters.min_ray_length_m=config_.min_range;parameters.max_ray_length_m=config_.max_range;
  parameters.max_weight=config_.max_weight;parameters.clearing_ray_weight_factor=config_.clearing_weight;
  parameters.use_weight_dropoff=config_.weight_dropoff;parameters.voxel_carving_enabled=config_.carving;
  parameters.use_const_weight=*config_.use_const_weight;
  parameters.use_symmetric_weight_dropoff=config_.use_symmetric_weight_dropoff;
  parameters.weight_ray_by_range=config_.weight_ray_by_range;
  parameters.allow_clear=config_.allow_clear;
  parameters.integrator_threads=config_.threads;parameters.integration_order_mode=config_.integration_order_mode;
  parameters.start_voxel_subsampling_factor=config_.start_voxel_subsampling_factor;
  parameters.max_consecutive_ray_collisions=config_.max_consecutive_ray_collisions;
  parameters.clear_checks_every_n_frames=config_.clear_checks_every_n_frames;
  parameters.max_integration_time_s=config_.max_integration_time_s;
  parameters.enable_anti_grazing=config_.anti_grazing;
  if(config_.method=="simple"&&parameters.integrator_threads==1)
    integrator_=std::make_shared<InlineSimpleTsdfIntegrator>(parameters,layer_.get());
  else integrator_=voxblox::TsdfIntegratorFactory::create(config_.method,parameters,layer_.get());
  // The core disables ordinary over-range clearing when carving is off.
  config_.allow_clear=integrator_->getConfig().allow_clear;
  esdf_integrator_.reset();esdf_layer_.reset();
  if(config_.esdf_enabled){
    esdf_layer_=std::make_unique<voxblox::Layer<voxblox::EsdfVoxel>>(config_.resolution,config_.voxels_per_side);
    initializeEsdfIntegrator();
  }
}
void TsdfBackend::initializeEsdfIntegrator(){
  if(esdf_layer_){
    voxblox::EsdfIntegrator::Config parameters;
    parameters.full_euclidean_distance=config_.esdf_full_euclidean_distance;
    parameters.min_distance_m=config_.esdf_min_distance;
    parameters.max_distance_m=config_.esdf_max_distance;
    parameters.default_distance_m=config_.esdf_default_distance;
    parameters.min_diff_m=config_.esdf_min_diff;
    parameters.add_occupied_crust=config_.esdf_add_occupied_crust;
    parameters.clear_sphere_radius=config_.esdf_clear_sphere_radius;
    parameters.occupied_sphere_radius=config_.esdf_occupied_sphere_radius;
    esdf_integrator_=std::make_unique<voxblox::EsdfIntegrator>(parameters,layer_.get(),esdf_layer_.get());
  }
}
void TsdfBackend::reset(){initialize();}
namespace {
template<class Voxel>std::size_t pruneDistantBlocks(voxblox::Layer<Voxel>* layer,
    const Eigen::Vector3d& position,double max_distance){
  const voxblox::Point center=position.cast<float>();
  if(!center.allFinite()||!std::isfinite(max_distance)||max_distance<0.)
    throw std::invalid_argument("Invalid block pruning position or distance");
  const auto before=layer->getNumberOfAllocatedBlocks();
  // Keep the original float block-origin subtraction and strict squared
  // distance comparison; voxel centers or bounding-box distance differ.
  layer->removeDistantBlocks(center,max_distance);
  return before-layer->getNumberOfAllocatedBlocks();
}
template<class Voxel>std::unique_ptr<voxblox::Layer<Voxel>> copyLayer(const voxblox::Layer<Voxel>& source){
  // The upstream copy constructor copies voxels only. Preserve block state
  // too: mergeBlock interprets has_data=false as an empty destination, and
  // Layer publication uses kMap to select blocks.
  auto result=std::make_unique<voxblox::Layer<Voxel>>(source);
  voxblox::BlockIndexList blocks;source.getAllAllocatedBlocks(&blocks);
  for(const auto& index:blocks){
    const auto& original=source.getBlockByIndex(index);
    auto& copy=result->getBlockByIndex(index);
    copy.set_has_data(original.has_data());copy.set_updated(original.updated());
  }
  return result;
}
}
std::unique_ptr<voxblox::Layer<voxblox::TsdfVoxel>> TsdfBackend::copyTsdfLayer()const{
  return copyLayer(*layer_);
}
std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel>> TsdfBackend::copyEsdfLayer()const{
  return esdf_layer_?copyLayer(*esdf_layer_):nullptr;
}
std::size_t TsdfBackend::removeDistantTsdfBlocks(const Eigen::Vector3d& position,double max_distance,
                                              voxblox::BlockIndexList* removed){
  // integratePointCloud joins its workers and drains temp_block_map_ before
  // returning. Its last-block handles are local; Fast's approximate index sets
  // reset on the next scan (the original default is every frame). Keep the
  // integrator and the surviving layer metadata instead of rebuilding them.
  voxblox::BlockIndexList before;layer_->getAllAllocatedBlocks(&before);
  const auto count=pruneDistantBlocks(layer_.get(),position,max_distance);
  if(removed)removed->clear();
  if(!count)return 0;
  for(const auto& index:before){
    if(layer_->hasBlock(index))continue;
    if(removed)removed->push_back(index);
    // Explicit ROS2 cache repair: upstream Marching Cubes uses positive cube
    // corners, so deleting B invalidates triangles owned by B-(dx,dy,dz).
    // The original layer deletion does not mark these surviving neighbors.
    for(unsigned mask=1;mask<8;++mask){
      voxblox::BlockIndex neighbor=index;bool representable=true;
      for(unsigned axis=0;axis<3;++axis)if(mask&(1u<<axis)){
        if(neighbor(axis)==std::numeric_limits<voxblox::IndexElement>::min()){
          representable=false;break;
        }
        --neighbor(axis);
      }
      if(representable&&layer_->hasBlock(neighbor))
        layer_->getBlockByIndex(neighbor).updated().set(voxblox::Update::kMesh);
    }
  }
  return count;
}
std::size_t TsdfBackend::removeDistantEsdfBlocks(const Eigen::Vector3d& position,double max_distance){
  if(!esdf_layer_)return 0;
  // updateEsdf/addNewRobotPosition synchronously finish ESDF propagation:
  // updateFromTsdfLayer consumes updated_blocks_, then processRaiseSet and
  // processOpenSet drain their queues. Prune only after that return point.
  return pruneDistantBlocks(esdf_layer_.get(),position,max_distance);
}
void TsdfBackend::replaceTsdfLayer(std::unique_ptr<voxblox::Layer<voxblox::TsdfVoxel>> layer){
  if(!layer||layer->voxels_per_side()!=static_cast<std::size_t>(config_.voxels_per_side)||
     std::abs(layer->voxel_size()-config_.resolution)>1e-5)
    throw std::invalid_argument("Incompatible transported TSDF layer");
  integrator_.reset();esdf_integrator_.reset();layer_=std::move(layer);
  initializeIntegrators();rebuildEsdf();
}
void TsdfBackend::replaceEsdfLayer(std::unique_ptr<voxblox::Layer<voxblox::EsdfVoxel>> layer){
  if(!config_.esdf_enabled||!layer||layer->voxels_per_side()!=static_cast<std::size_t>(config_.voxels_per_side)||
     std::abs(layer->voxel_size()-config_.resolution)>1e-5)
    throw std::invalid_argument("Incompatible transported ESDF layer");
  // The wire format preserves in_queue, but a remote queue cannot be copied
  // into this integrator. Clear only that scheduling bit, retaining distances,
  // parent directions, fixed and hallucinated voxel state.
  voxblox::BlockIndexList blocks;layer->getAllAllocatedBlocks(&blocks);
  for(const auto& index:blocks){
    auto& block=layer->getBlockByIndex(index);
    for(std::size_t i=0;i<block.num_voxels();++i)block.getVoxelByLinearIndex(i).in_queue=false;
  }
  esdf_integrator_.reset();esdf_layer_=std::move(layer);initializeEsdfIntegrator();
}
TsdfScanResult TsdfBackend::integrate(const Eigen::Vector3d& origin,const Eigen::Quaterniond& rotation,
                                    const std::vector<Eigen::Vector3d>& sensor_points){
  return integrateCloud(origin,rotation,sensor_points,nullptr,false);
}
TsdfScanResult TsdfBackend::integrate(const Eigen::Vector3d& origin,const Eigen::Quaterniond& rotation,
                                    const std::vector<Eigen::Vector3d>& sensor_points,
                                    const std::vector<voxblox::Color>& sensor_colors){
  return integrateCloud(origin,rotation,sensor_points,&sensor_colors,false);
}
TsdfScanResult TsdfBackend::integrateFreeSpace(const Eigen::Vector3d& origin,const Eigen::Quaterniond& rotation,
                                             const std::vector<Eigen::Vector3d>& sensor_points){
  return integrateCloud(origin,rotation,sensor_points,nullptr,true);
}
void TsdfBackend::updateEsdf(){
  if(esdf_integrator_)esdf_integrator_->updateFromTsdfLayer(true);
}
void TsdfBackend::addNewRobotPosition(const Eigen::Vector3d& position){
  if(!config_.esdf_enabled||!config_.esdf_clear_sphere_for_planning)return;
  if(!position.allFinite()||position.cwiseAbs().maxCoeff()/config_.resolution>1e6)
    throw std::invalid_argument("Invalid ESDF robot position");
  esdf_integrator_->addNewRobotPosition(position.cast<float>());
  // addNewRobotPosition queues changed hallucinated cells; resolve the
  // raise/open sets before the next planner query.
  updateEsdf();
  // The upstream integrator skips TSDF-absent sphere blocks while marking
  // updates. Their hallucinated cells still changed and must reach Layer
  // subscribers, including blocks reached by ESDF propagation.
  voxblox::BlockIndexList blocks;esdf_layer_->getAllAllocatedBlocks(&blocks);
  for(const auto& index:blocks){
    auto& block=esdf_layer_->getBlockByIndex(index);
    block.updated().set(voxblox::Update::kMap);block.set_has_data(true);
  }
}
TsdfScanResult TsdfBackend::integrateCloud(const Eigen::Vector3d& origin,const Eigen::Quaterniond& rotation,
                                         const std::vector<Eigen::Vector3d>& sensor_points,
                                         const std::vector<voxblox::Color>* sensor_colors,bool freespace){
  using Clock=std::chrono::steady_clock;
  last_integration_timing_={};
  const auto start=Clock::now();
  if(!origin.allFinite()||origin.cwiseAbs().maxCoeff()/config_.resolution>1e6||
     !rotation.coeffs().allFinite()||!std::isfinite(rotation.norm())||rotation.norm()<1e-8)
    throw std::invalid_argument("Invalid TSDF sensor transform");
  if(sensor_colors&&sensor_colors->size()!=sensor_points.size())
    throw std::invalid_argument("Point/color count mismatch");
  voxblox::Pointcloud points;points.reserve(sensor_points.size());TsdfScanResult result;
  voxblox::Colors colors;colors.reserve(sensor_points.size());
  for(std::size_t i=0;i<sensor_points.size();++i){
    const auto& point=sensor_points[i];
    if(!point.allFinite()||point.norm()<config_.min_range||point.norm()>1e5){++result.skipped_points;continue;}
    const voxblox::Point sensor_point=point.cast<float>();
    if(config_.weight_ray_by_range){
      // The upstream weight is 1/z^2 in sensor coordinates, not 1/range^2.
      // Reject undefined weights here: the core otherwise aborts on z=0.
      const float depth=std::abs(sensor_point.z());
      const float weight=depth>voxblox::kFloatEpsilon?1.0f/(depth*depth):0.0f;
      if(!std::isfinite(weight)||weight<=0.f){++result.skipped_points;continue;}
    }
    points.push_back(sensor_point);++result.accepted_points;
    // Filter geometry and color together so a NaN return cannot shift colors
    // onto a neighboring valid point. Uncolored sensors retain neutral gray.
    colors.push_back(sensor_colors&&config_.use_color?(*sensor_colors)[i]:voxblox::Color(160,160,160));
  }
  if(points.empty())throw std::invalid_argument("Cloud has no usable returns");
  const voxblox::Transformation transform(voxblox::Rotation(rotation.normalized().cast<float>()),origin.cast<float>());
  const auto filtered=Clock::now();
  last_integration_timing_.filter_wall_s=std::chrono::duration<double>(filtered-start).count();
  // Input points remain in sensor coordinates; the original integrator applies T_G_C.
  integrator_->integratePointCloud(transform,points,colors,freespace);
  const auto integrated=Clock::now();
  last_integration_timing_.tsdf_wall_s=std::chrono::duration<double>(integrated-filtered).count();
  if(esdf_integrator_&&config_.esdf_update_on_scan){
    updateEsdf();
    last_integration_timing_.esdf_wall_s=std::chrono::duration<double>(Clock::now()-integrated).count();
    last_integration_timing_.esdf_updated=true;
  }
  last_integration_timing_.total_wall_s=std::chrono::duration<double>(Clock::now()-start).count();
  last_integration_timing_.completed=true;
  return result;
}
std::size_t TsdfBackend::clearUnknownBox(const Eigen::Vector3d& center,const Eigen::Vector3d& size){
  if(!center.allFinite()||!size.allFinite()||(size.array()<0.).any()||
     center.cwiseAbs().maxCoeff()/config_.resolution>1e6)
    throw std::invalid_argument("Invalid unknown-clear box");
  // GBPlanner's getBoxAroundPoint first snaps the float center to the
  // world-zero lattice, then floors offsets -half, -half+1, ... <= half.
  // It is not floor((center +/- size/2)/resolution), nor a symmetric ceil
  // radius. For half=.5 the source covers offsets {-1,0}, not {-1,0,1}.
  const float voxel_size=layer_->voxel_size(),voxel_size_inv=1.0/voxel_size;
  const auto center_index=voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(
    center.cast<voxblox::FloatingPoint>(),voxel_size_inv);
  const Eigen::Vector3f half=(size/2/voxel_size).cast<voxblox::FloatingPoint>();
  const Eigen::Array3d radius=half.cast<double>().array().ceil();
  const Eigen::Array3d extent=(2.*half.cast<double>().array()).floor()+1.;
  if(!half.allFinite()||
     (center_index.cast<double>().array().abs()+radius).maxCoeff()>1e6||
     (extent<=0.).any()||extent.prod()>2000000.)
    throw std::invalid_argument("Unknown-clear box exceeds voxel budget");
  std::array<std::vector<int64_t>,3> offsets;
  for(int axis=0;axis<3;++axis){
    // Preserve the source's FloatingPoint increment, including its rounding
    // near fractional voxel boundaries. Validate before allocating map blocks.
    for(voxblox::FloatingPoint offset=-half[axis];offset<=half[axis];offset++)
      offsets[axis].push_back(static_cast<int64_t>(std::floor(offset)));
  }
  if(uint64_t(offsets[0].size())*offsets[1].size()*offsets[2].size()>2000000u)
    throw std::invalid_argument("Unknown-clear box exceeds voxel budget");
  std::size_t cleared=0;
  for(const auto x:offsets[0])for(const auto y:offsets[1])for(const auto z:offsets[2]){
      voxblox::BlockIndex block_index;voxblox::VoxelIndex voxel_index;
      voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
        center_index+voxblox::GlobalIndex(x,y,z),layer_->voxels_per_side(),&block_index,&voxel_index);
      auto block=layer_->allocateBlockPtrByIndex(block_index);
      auto& cell=block->getVoxelByVoxelIndex(voxel_index);
      // Upstream clearIfUnknown: never alter an observed free or occupied cell.
      // The small positive weight makes explicit augmentation visible to the
      // original mapper and lets later actual observations dominate it.
      if(cell.weight<1e-6f){
        cell.weight=1e-3f+1e-6f;
        cell.distance=static_cast<float>(config_.truncation_distance);
        block->set_has_data(true);block->updated().set();++cleared;
      }
    }
  if(cleared&&esdf_integrator_)rebuildEsdf();
  return cleared;
}
VoxelStatus TsdfBackend::classify(const voxblox::TsdfVoxel* voxel,double resolution,double factor){
  // Original GBPlanner map_manager_voxblox_impl: weight<1e-6 is unknown;
  // distance <= factor*voxel_size+1e-6 is occupied (including negative distances).
  if(!voxel||!std::isfinite(voxel->weight)||!std::isfinite(voxel->distance)||voxel->weight<1e-6)
    return VoxelStatus::Unknown;
  return voxel->distance<=factor*resolution+1e-6?VoxelStatus::Occupied:VoxelStatus::Free;
}
const voxblox::TsdfVoxel* TsdfBackend::voxel(const Eigen::Vector3d& position) const {
  if(!position.allFinite()||position.cwiseAbs().maxCoeff()/config_.resolution>1e6)return nullptr;
  return layer_->getVoxelPtrByCoordinates(position.cast<float>());
}
VoxelStatus TsdfBackend::status(const Eigen::Vector3d& position) const {
  if(config_.distance_field=="esdf"){
    const auto* value=esdfVoxel(position);
    if(!value||!value->observed||!std::isfinite(value->distance))return VoxelStatus::Unknown;
    return value->distance<=config_.occupancy_factor*config_.resolution+1e-6?VoxelStatus::Occupied:VoxelStatus::Free;
  }
  return classify(voxel(position),config_.resolution,config_.occupancy_factor);
}
std::shared_ptr<VoxelMap> TsdfBackend::snapshot() const {
  auto result=std::make_shared<VoxelMap>(config_.resolution,config_.origin,config_.dimensions);
  for(int z=0;z<config_.dimensions.z();++z)for(int y=0;y<config_.dimensions.y();++y)
    for(int x=0;x<config_.dimensions.x();++x){
      const Eigen::Vector3i index(x,y,z);const auto center=result->center(index);
      result->set(index,status(center));
      // Terrain projection always uses the observed TSDF, even when obstacle
      // classification uses ESDF. Keep both fields in the same map revision.
      const auto* observed=voxel(center);
      if(classify(observed,config_.resolution,config_.occupancy_factor)!=VoxelStatus::Unknown)
        result->setTsdf(index,observed->distance,observed->weight);
    }
  return result;
}
void TsdfBackend::forEachObservedVoxel(const ObservedVoxelVisitor& visit) const {
  auto observed_tsdf=[this](const voxblox::TsdfVoxel* value){
    return classify(value,config_.resolution,config_.occupancy_factor)!=VoxelStatus::Unknown;
  };
  auto observed_esdf=[](const voxblox::EsdfVoxel* value){
    return value&&value->observed&&std::isfinite(value->distance);
  };
  auto emit=[&](const voxblox::BlockIndex& block_index,const voxblox::VoxelIndex& local_index,
                const voxblox::TsdfVoxel* tsdf,const voxblox::EsdfVoxel* esdf){
    VoxelStatus state=VoxelStatus::Unknown;
    if(config_.distance_field=="esdf"){
      if(esdf)state=esdf->distance<=config_.occupancy_factor*config_.resolution+1e-6
        ?VoxelStatus::Occupied:VoxelStatus::Free;
    }else state=classify(tsdf,config_.resolution,config_.occupancy_factor);
    const Eigen::Vector3i index=block_index*config_.voxels_per_side+local_index;
    visit(index,state,tsdf,esdf);
  };
  voxblox::BlockIndexList blocks;layer_->getAllAllocatedBlocks(&blocks);
  for(const auto& block_index:blocks){
    const auto& block=layer_->getBlockByIndex(block_index);
    const auto esdf_block=esdf_layer_?esdf_layer_->getBlockPtrByIndex(block_index):nullptr;
    for(std::size_t i=0;i<block.num_voxels();++i){
      const auto* tsdf=&block.getVoxelByLinearIndex(i);
      const auto* esdf=esdf_block?&esdf_block->getVoxelByLinearIndex(i):nullptr;
      if(!observed_tsdf(tsdf))tsdf=nullptr;
      if(!observed_esdf(esdf))esdf=nullptr;
      if(tsdf||esdf)emit(block_index,block.computeVoxelIndexFromLinearIndex(i),tsdf,esdf);
    }
  }
  if(!esdf_layer_)return;
  esdf_layer_->getAllAllocatedBlocks(&blocks);
  for(const auto& block_index:blocks){
    // The first traversal already emitted the full observed union in every
    // TSDF block. Only ESDF blocks without a TSDF block remain to be visited.
    if(layer_->hasBlock(block_index))continue;
    const auto& block=esdf_layer_->getBlockByIndex(block_index);
    for(std::size_t i=0;i<block.num_voxels();++i){
      const auto* esdf=&block.getVoxelByLinearIndex(i);
      if(observed_esdf(esdf))emit(block_index,block.computeVoxelIndexFromLinearIndex(i),nullptr,esdf);
    }
  }
}
void TsdfBackend::rebuildEsdf(){
  if(esdf_integrator_)esdf_integrator_->updateFromTsdfLayerBatch();
}
const voxblox::EsdfVoxel* TsdfBackend::esdfVoxel(const Eigen::Vector3d& position) const {
  if(!esdf_layer_||!position.allFinite()||position.cwiseAbs().maxCoeff()/config_.resolution>1e6)return nullptr;
  return esdf_layer_->getVoxelPtrByCoordinates(position.cast<float>());
}
namespace {
// Write beside the destination, then rename, so a failed write keeps the old file.
template<class Writer>void writeFile(const std::string& name,const Writer& writer){
  if(name.empty())throw std::invalid_argument("File path is empty");
  const std::filesystem::path destination(name);
  if(!destination.parent_path().empty())std::filesystem::create_directories(destination.parent_path());
  const auto temporary=destination.string()+".tmp."+std::to_string(getpid());
  try {
    if(!writer(temporary))throw std::runtime_error("Could not write "+name);
    std::filesystem::rename(temporary,destination);
  } catch(...) {std::error_code error;std::filesystem::remove(temporary,error);throw;}
}
}
void TsdfBackend::saveMap(const std::string& path) const {
  writeFile(path,[this](const std::string& file){return writeMapFile(file,*layer_,esdf_layer_.get());});
}
void TsdfBackend::loadMap(const std::string& path){
  if(path.empty())throw std::invalid_argument("File path is empty");
  // Both layers use kReplace: file blocks overwrite matching indices while
  // other live blocks survive. Finish parsing even an unused ESDF section
  // before committing either layer, so a damaged tail leaves both untouched.
  auto next_tsdf=copyTsdfLayer();
  auto next_esdf=copyEsdfLayer();
  const bool has_esdf=readMapFile(path,next_tsdf.get(),next_esdf.get());
  integrator_.reset();esdf_integrator_.reset();layer_=std::move(next_tsdf);
  initializeIntegrators();
  if(has_esdf&&config_.esdf_enabled){
    // ESDF-only planning spheres and their signed parent directions cannot
    // be regenerated from the TSDF. Restore them, clearing only queue flags
    // whose corresponding in-memory propagation queues were not persisted.
    replaceEsdfLayer(std::move(next_esdf));
  }else{
    // Older TSDF-only files still work. Rebuild from the final merged layer,
    // including retained blocks, rather than only the blocks in the file.
    rebuildEsdf();
  }
}
voxblox::Mesh TsdfBackend::generateMesh() const {
  voxblox::MeshLayer mesh_layer(layer_->block_size());
  voxblox::MeshIntegratorConfig parameters;parameters.integrator_threads=config_.threads;
  parameters.use_color=config_.mesh_use_color;parameters.min_weight=config_.mesh_min_weight;
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesher(parameters,*layer_,&mesh_layer);
  // Generate the complete surface without consuming flags used by other outputs.
  mesher.generateMesh(false,false);
  voxblox::Mesh mesh;
  // Voxblox returns false for an empty surface. This is a valid map state
  // after clearing and must delete the previously published visualization.
  voxblox::convertMeshLayerToMesh(mesh_layer,&mesh,true);
  return mesh;
}
void TsdfBackend::updateMeshLayer(voxblox::MeshLayer* mesh_layer,bool only_updated){
  if(!mesh_layer||mesh_layer->block_size()!=layer_->block_size())
    throw std::invalid_argument("Mesh layer geometry does not match TSDF");
  voxblox::MeshIntegratorConfig parameters;parameters.integrator_threads=config_.threads;
  parameters.use_color=config_.mesh_use_color;parameters.min_weight=config_.mesh_min_weight;
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesher(parameters,layer_.get(),mesh_layer);
  mesher.generateMesh(only_updated,true);
}
void TsdfBackend::updateMeshBlocks(voxblox::MeshLayer* mesh_layer,const voxblox::BlockIndexList& blocks){
  if(!mesh_layer||mesh_layer->block_size()!=layer_->block_size())
    throw std::invalid_argument("Mesh layer geometry does not match TSDF");
  if(blocks.empty()||mesh_layer->getNumberOfAllocatedMeshes()==0)return;
  voxblox::MeshIntegratorConfig parameters;parameters.integrator_threads=config_.threads;
  parameters.use_color=config_.mesh_use_color;parameters.min_weight=config_.mesh_min_weight;
  voxblox::MeshIntegrator<voxblox::TsdfVoxel> mesher(parameters,layer_.get(),mesh_layer);
  voxblox::BlockIndexList cached_indices;mesh_layer->getAllAllocatedMeshes(&cached_indices);
  const voxblox::IndexSet cached(cached_indices.begin(),cached_indices.end());
  voxblox::IndexSet visited;
  for(const auto& index:blocks){
    if(!visited.insert(index).second||!layer_->hasBlock(index)||!cached.count(index))continue;
    // Reuse the original block mesher, without generateMesh's allocation of
    // every pending block. Uncached blocks keep kMesh for a later normal update.
    mesher.updateMeshForBlock(index);
    layer_->getBlockByIndex(index).updated().reset(voxblox::Update::kMesh);
  }
}
void TsdfBackend::saveMesh(const std::string& path,const voxblox::Mesh& mesh) const {
  writeFile(path,[&mesh](const std::string& file){return voxblox::outputMeshAsPly(file,mesh);});
}

}
