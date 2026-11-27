#pragma once

#include "gbplanner_voxblox/tsdf_backend.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace gbplanner2 {

// Optional, serialized mapper-side evidence. Read exact global voxel indices
// immediately before and after one normal backend integration, before pruning.
// No queries allocate map blocks, and no diagnostic invokes the integrator.
class VoxelScanCapture {
 public:
  VoxelScanCapture(const std::string& directory,
                   const std::vector<int64_t>& flattened_xyz,
                   const TsdfConfig& config,bool log_all_changes=true)
      : log_all_changes_(log_all_changes) {
    if(directory.empty())return;
    try {
      if(flattened_xyz.empty()||flattened_xyz.size()%3||flattened_xyz.size()>3*kMaxTargets)
        throw std::invalid_argument("Voxel capture needs 1..2048 complete XYZ indices");
      std::set<std::array<int64_t,3>> seen;
      for(std::size_t offset=0;offset<flattened_xyz.size();offset+=3){
        std::array<int64_t,3> key{};
        for(int axis=0;axis<3;++axis){
          key[axis]=flattened_xyz[offset+axis];
          if(key[axis]<-1000000||key[axis]>1000000)
            throw std::invalid_argument("Voxel capture index outside backend coordinate domain");
        }
        if(!seen.insert(key).second)throw std::invalid_argument("Duplicate voxel capture index");
        Target target;target.index=voxblox::GlobalIndex(key[0],key[1],key[2]);targets_.push_back(target);
      }
      before_.resize(targets_.size());after_.resize(targets_.size());
      changed_.resize(targets_.size());crossed_.resize(targets_.size());
      save_trigger_.resize(targets_.size());voxel_limited_.resize(targets_.size());
      global_limited_.resize(targets_.size());
      directory_=std::filesystem::absolute(directory);
      if(std::filesystem::exists(directory_))
        throw std::runtime_error("Voxel capture directory already exists; refusing to overwrite evidence");
      std::filesystem::create_directories(directory_.parent_path());
      if(!std::filesystem::create_directory(directory_))
        throw std::runtime_error("Cannot create voxel capture directory");
      owns_directory_=true;
      changes_.exceptions(std::ios::failbit|std::ios::badbit);
      changes_.open(directory_/"changes.jsonl");
      format(changes_);
      std::ofstream manifest;manifest.exceptions(std::ios::failbit|std::ios::badbit);
      manifest.open(directory_/"manifest.json");format(manifest);
      manifest<<"{\"status\":\"ready\",\"format\":\"GBPLANNER_VOXEL_SCAN_CAPTURE_V1\","
        <<"\"scope\":\"Exact before/after TSDF values around one synchronous integration; raw mapper input bytes on bounded Free-to-Occupied crossings\","
        <<"\"max_targets\":"<<kMaxTargets<<",\"target_count\":"<<targets_.size()
        <<",\"max_captures_per_voxel\":"<<kCaptureLimit
        <<",\"max_scan_captures\":"<<kGlobalCaptureLimit
        <<",\"log_all_changes\":"<<log_all_changes_<<",\"indices\":[";
      for(std::size_t i=0;i<targets_.size();++i){if(i)manifest<<',';vectorOut(manifest,targets_[i].index);}
      manifest<<"],\"constructor_config\":";configOut(manifest,config);
      manifest<<",\"occupancy_enum\":{\"unknown\":-1,\"free\":0,\"occupied\":100},"
        <<"\"limits\":[\"When log_all_changes is false, log only presence or occupancy changes; snapshots retain all target before/after values.\","
        <<"\"Every observed Free-to-Occupied crossing is logged, including crossings beyond either raw-cloud capture limit.\","
        <<"\"voxel_capture_limit_reached and global_capture_limit_reached independently identify limits active before this scan was saved.\","
        <<"\"Saved clouds are mapper inputs; the upstream adapter may already have applied its normal range gate.\","
        <<"\"No data is retained when this helper is not constructed.\"]}\n";
      manifest.close();enabled_=true;
    }catch(const std::exception& error){disable(error.what());}
    catch(...){disable("Unknown voxel capture initialization failure");}
  }

  bool enabled() const noexcept {return enabled_;}

  void captureBefore(const TsdfBackend& backend) noexcept {
    if(!enabled_)return;
    try{
      before_valid_=false;
      for(std::size_t i=0;i<targets_.size();++i)before_[i]=sample(backend,targets_[i].index);
      before_valid_=true;
    }catch(const std::exception& error){disable(error.what());}
    catch(...){disable("Unknown voxel capture before-read failure");}
  }

  void captureAfter(const TsdfBackend& backend,
                    const sensor_msgs::msg::PointCloud2& cloud,
                    const Eigen::Vector3d& raw_origin,
                    const Eigen::Quaterniond& raw_rotation,
                    const Eigen::Vector3d& integration_origin,
                    const Eigen::Quaterniond& integration_rotation,
                    bool freespace,uint64_t scan_sequence) noexcept {
    if(!enabled_)return;
    try{
      if(!before_valid_)throw std::runtime_error("Voxel capture after-read has no matching before-read");
      before_valid_=false;
      bool any_change=false,save_scan=false;
      for(std::size_t i=0;i<targets_.size();++i){
        after_[i]=sample(backend,targets_[i].index);
        changed_[i]=log_all_changes_?!same(before_[i],after_[i]):
          (before_[i].present!=after_[i].present||before_[i].occupancy!=after_[i].occupancy);
        any_change=any_change||changed_[i];
        crossed_[i]=before_[i].occupancy==VoxelStatus::Free&&after_[i].occupancy==VoxelStatus::Occupied;
        voxel_limited_[i]=crossed_[i]&&targets_[i].saved>=kCaptureLimit;
        global_limited_[i]=crossed_[i]&&capture_count_>=kGlobalCaptureLimit;
        save_trigger_[i]=crossed_[i]&&!voxel_limited_[i]&&!global_limited_[i];
        if(crossed_[i]){
          ++targets_[i].crossings;
          save_scan=save_scan||save_trigger_[i];
        }
      }
      if(!any_change)return;
      std::string snapshot_relative;
      if(save_scan){
        const auto name="scan_"+std::to_string(scan_sequence)+"_capture_"+std::to_string(++capture_count_);
        const auto destination=directory_/name;
        if(!std::filesystem::create_directory(destination))
          throw std::runtime_error("Voxel scan snapshot already exists");
        if(cloud.data.size()>static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()))
          throw std::runtime_error("Voxel scan cloud exceeds stream size");
        std::ofstream bytes;bytes.exceptions(std::ios::failbit|std::ios::badbit);
        bytes.open(destination/"cloud.bin.tmp",std::ios::binary);
        bytes.write(reinterpret_cast<const char*>(cloud.data.data()),
                    static_cast<std::streamsize>(cloud.data.size()));bytes.close();
        std::filesystem::rename(destination/"cloud.bin.tmp",destination/"cloud.bin");
        std::ofstream metadata;metadata.exceptions(std::ios::failbit|std::ios::badbit);
        metadata.open(destination/"snapshot.json.tmp");format(metadata);
        metadata<<"{\"status\":\"complete\",\"format\":\"GBPLANNER_VOXEL_SCAN_CAPTURE_V1\",";
        scanIdentity(metadata,cloud,freespace,scan_sequence);
        metadata<<",\"cloud\":";cloudOut(metadata,cloud);
        metadata<<",\"raw_transform\":";transformOut(metadata,raw_origin,raw_rotation);
        metadata<<",\"integration_transform\":";transformOut(metadata,integration_origin,integration_rotation);
        // Duplicate only the backend's scalar conversion, not its integration.
        // Kindr's Rotation construction and float position are the actual T_G_C
        // supplied to the original kernel, including its normalization rules.
        const voxblox::Transformation applied(
          voxblox::Rotation(integration_rotation.normalized().cast<float>()),integration_origin.cast<float>());
        metadata<<",\"integrator_transform_float\":";
        transformOut(metadata,applied.getPosition(),applied.getRotation().toImplementation());
        metadata<<",\"config\":";configOut(metadata,backend.config());
        metadata<<",\"layer\":{\"voxel_size\":";numberOut(metadata,backend.tsdfLayer().voxel_size());
        metadata<<",\"voxels_per_side\":"<<backend.tsdfLayer().voxels_per_side()<<'}';
        metadata<<",\"max_captures_per_voxel\":"<<kCaptureLimit
          <<",\"max_scan_captures\":"<<kGlobalCaptureLimit
          <<",\"scan_captures_saved\":"<<capture_count_
          <<",\"log_all_changes\":"<<log_all_changes_<<",\"voxels\":[";
        for(std::size_t i=0;i<targets_.size();++i){
          if(i)metadata<<',';
          voxelOut(metadata,i,after_[i],crossed_[i],save_trigger_[i],save_trigger_[i]);
        }
        metadata<<"]}\n";metadata.close();
        std::filesystem::rename(destination/"snapshot.json.tmp",destination/"snapshot.json");
        for(std::size_t i=0;i<targets_.size();++i)if(save_trigger_[i])++targets_[i].saved;
        snapshot_relative=name+"/snapshot.json";
      }
      changes_<<"{\"status\":\"observed\",";scanIdentity(changes_,cloud,freespace,scan_sequence);
      changes_<<",\"snapshot\":";
      if(snapshot_relative.empty())changes_<<"null";else stringOut(changes_,snapshot_relative);
      changes_<<",\"scan_captures_saved\":"<<capture_count_;
      changes_<<",\"changes\":[";bool first=true;
      for(std::size_t i=0;i<targets_.size();++i)if(changed_[i]){
        if(!first)changes_<<',';first=false;
        voxelOut(changes_,i,after_[i],crossed_[i],save_trigger_[i],false);
      }
      changes_<<"]}\n";changes_.flush();
    }catch(const std::exception& error){disable(error.what());}
    catch(...){disable("Unknown voxel capture after-write failure");}
  }

 private:
  static constexpr std::size_t kMaxTargets=2048;
  static constexpr uint64_t kCaptureLimit=4;
  static constexpr uint64_t kGlobalCaptureLimit=32;
  struct Sample {
    bool present{false};float distance{0.f},weight{0.f};
    std::array<uint8_t,4> color{};
    VoxelStatus occupancy{VoxelStatus::Unknown};
  };
  struct Target {voxblox::GlobalIndex index{voxblox::GlobalIndex::Zero()};uint64_t crossings{0},saved{0};};

  static Sample sample(const TsdfBackend& backend,const voxblox::GlobalIndex& index){
    Sample value;
    const auto* voxel=backend.tsdfLayer().getVoxelPtrByGlobalIndex(index);
    value.present=voxel!=nullptr;
    if(voxel){value.distance=voxel->distance;value.weight=voxel->weight;
      value.color={voxel->color.r,voxel->color.g,voxel->color.b,voxel->color.a};}
    value.occupancy=TsdfBackend::classify(voxel,backend.config().resolution,backend.config().occupancy_factor);
    return value;
  }
  static bool same(const Sample& a,const Sample& b){
    return a.present==b.present&&a.occupancy==b.occupancy&&a.color==b.color&&
      std::memcmp(&a.distance,&b.distance,sizeof(float))==0&&
      std::memcmp(&a.weight,&b.weight,sizeof(float))==0;
  }
  static void format(std::ostream& out){out<<std::setprecision(17)<<std::boolalpha;}
  static void numberOut(std::ostream& out,double value){if(std::isfinite(value))out<<value;else out<<"null";}
  static void stringOut(std::ostream& out,const std::string& value){
    static constexpr char digits[]="0123456789abcdef";
    out<<'"';
    for(unsigned char c:value){
      if(c=='"'||c=='\\')out<<'\\'<<c;
      else if(c<0x20)out<<"\\u00"<<digits[c>>4]<<digits[c&15];
      else out<<c;
    }
    out<<'"';
  }
  template<class Derived> static void vectorOut(std::ostream& out,const Eigen::MatrixBase<Derived>& value){
    out<<'[';for(Eigen::Index i=0;i<value.size();++i){if(i)out<<',';numberOut(out,value[i]);}out<<']';
  }
  template<class Position,class Quaternion>
  static void transformOut(std::ostream& out,const Position& origin,const Quaternion& rotation){
    out<<"{\"origin\":";vectorOut(out,origin);out<<",\"rotation_wxyz\":[";
    numberOut(out,rotation.w());out<<',';numberOut(out,rotation.x());out<<',';
    numberOut(out,rotation.y());out<<',';numberOut(out,rotation.z());out<<"]}";
  }
  static void sampleOut(std::ostream& out,const Sample& value){
    out<<"{\"present\":"<<value.present<<",\"distance\":";
    if(value.present)numberOut(out,value.distance);else out<<"null";
    out<<",\"weight\":";if(value.present)numberOut(out,value.weight);else out<<"null";
    out<<",\"color\":";
    if(value.present){out<<'[';for(int i=0;i<4;++i){if(i)out<<',';out<<unsigned(value.color[i]);}out<<']';}
    else out<<"null";
    out<<",\"occupancy\":"<<int(value.occupancy)<<'}';
  }
  void voxelOut(std::ostream& out,std::size_t i,const Sample& after,bool crossed,
                bool captured,bool include_pending_saved) const {
    const auto& target=targets_[i];
    out<<"{\"index\":";vectorOut(out,target.index);out<<",\"before\":";sampleOut(out,before_[i]);
    out<<",\"after\":";sampleOut(out,after);
    out<<",\"free_to_occupied\":"<<crossed<<",\"crossings_seen\":"<<target.crossings
       <<",\"captures_saved\":"<<(target.saved+(include_pending_saved?1:0))
       <<",\"capture_trigger\":"<<captured
       <<",\"capture_limit_reached\":"<<(voxel_limited_[i]||global_limited_[i])
       <<",\"voxel_capture_limit_reached\":"<<voxel_limited_[i]
       <<",\"global_capture_limit_reached\":"<<global_limited_[i]<<'}';
  }
  static void scanIdentity(std::ostream& out,const sensor_msgs::msg::PointCloud2& cloud,
                           bool freespace,uint64_t sequence){
    out<<"\"scan_sequence\":"<<sequence<<",\"source_frame\":";stringOut(out,cloud.header.frame_id);
    out<<",\"stamp_ns\":"<<(int64_t(cloud.header.stamp.sec)*1000000000LL+cloud.header.stamp.nanosec)
       <<",\"freespace\":"<<freespace;
  }
  static void cloudOut(std::ostream& out,const sensor_msgs::msg::PointCloud2& cloud){
    out<<"{\"data_file\":\"cloud.bin\",\"frame_id\":";stringOut(out,cloud.header.frame_id);
    out<<",\"stamp\":{\"sec\":"<<cloud.header.stamp.sec<<",\"nanosec\":"<<cloud.header.stamp.nanosec
       <<"},\"width\":"<<cloud.width<<",\"height\":"<<cloud.height
       <<",\"point_step\":"<<cloud.point_step<<",\"row_step\":"<<cloud.row_step
       <<",\"is_bigendian\":"<<cloud.is_bigendian<<",\"is_dense\":"<<cloud.is_dense
       <<",\"bytes\":"<<cloud.data.size()<<",\"fields\":[";
    for(std::size_t i=0;i<cloud.fields.size();++i){
      const auto& field=cloud.fields[i];if(i)out<<',';
      out<<"{\"name\":";stringOut(out,field.name);
      out<<",\"offset\":"<<field.offset<<",\"datatype\":"<<int(field.datatype)<<",\"count\":"<<field.count<<'}';
    }
    out<<"]}";
  }
  static void configOut(std::ostream& out,const TsdfConfig& c){
    out<<'{';bool first=true;
    const auto name=[&](const char* key){if(!first)out<<',';first=false;stringOut(out,key);out<<':';};
    const auto real=[&](const char* key,double value){name(key);numberOut(out,value);};
    const auto boolean=[&](const char* key,bool value){name(key);out<<value;};
    const auto integer=[&](const char* key,int value){name(key);out<<value;};
    const auto text=[&](const char* key,const std::string& value){name(key);stringOut(out,value);};
    real("resolution",c.resolution);name("origin");vectorOut(out,c.origin);
    name("dimensions");vectorOut(out,c.dimensions);integer("voxels_per_side",c.voxels_per_side);
    real("truncation_distance",c.truncation_distance);real("min_range",c.min_range);real("max_range",c.max_range);
    real("occupancy_factor",c.occupancy_factor);real("max_weight",c.max_weight);real("clearing_weight",c.clearing_weight);
    integer("threads",c.threads);text("method",c.method);boolean("weight_dropoff",c.weight_dropoff);boolean("carving",c.carving);
    name("use_const_weight");if(c.use_const_weight)out<<*c.use_const_weight;else out<<"null";
    boolean("use_symmetric_weight_dropoff",c.use_symmetric_weight_dropoff);boolean("weight_ray_by_range",c.weight_ray_by_range);
    boolean("allow_clear",c.allow_clear);text("integration_order_mode",c.integration_order_mode);
    real("start_voxel_subsampling_factor",c.start_voxel_subsampling_factor);
    integer("max_consecutive_ray_collisions",c.max_consecutive_ray_collisions);
    integer("clear_checks_every_n_frames",c.clear_checks_every_n_frames);real("max_integration_time_s",c.max_integration_time_s);
    boolean("anti_grazing",c.anti_grazing);boolean("esdf_enabled",c.esdf_enabled);
    real("esdf_max_distance",c.esdf_max_distance);boolean("esdf_full_euclidean_distance",c.esdf_full_euclidean_distance);
    real("esdf_min_distance",c.esdf_min_distance);real("esdf_default_distance",c.esdf_default_distance);
    real("esdf_min_diff",c.esdf_min_diff);boolean("esdf_add_occupied_crust",c.esdf_add_occupied_crust);
    boolean("esdf_clear_sphere_for_planning",c.esdf_clear_sphere_for_planning);
    real("esdf_clear_sphere_radius",c.esdf_clear_sphere_radius);real("esdf_occupied_sphere_radius",c.esdf_occupied_sphere_radius);
    text("distance_field",c.distance_field);text("storage",c.storage);boolean("use_color",c.use_color);
    real("mesh_min_weight",c.mesh_min_weight);boolean("mesh_use_color",c.mesh_use_color);out<<'}';
  }
  void disable(const char* reason) noexcept {
    enabled_=false;before_valid_=false;
    std::fprintf(stderr,"Voxel scan capture disabled: %s\n",reason);
    // A failed optional write never escapes into scan admission/integration.
    // The failure marker is best effort; stderr remains the fallback if the
    // destination itself is unwritable. Never touch a pre-existing directory.
    if(owns_directory_)try{
      std::ofstream failure(directory_/"failure.json");format(failure);
      failure<<"{\"status\":\"disabled_after_failure\",\"reason\":";
      stringOut(failure,reason);failure<<",\"snapshots_attempted\":"<<capture_count_<<"}\n";
    }catch(...){}
  }

  std::filesystem::path directory_;
  std::vector<Target> targets_;
  std::vector<Sample> before_,after_;
  std::vector<bool> changed_,crossed_,save_trigger_,voxel_limited_,global_limited_;
  std::ofstream changes_;
  uint64_t capture_count_{0};
  bool log_all_changes_{true};
  bool enabled_{false},before_valid_{false},owns_directory_{false};
};

}  // namespace gbplanner2
