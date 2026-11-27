// Read-only replay of captured Simple TSDF updates for selected voxels.
// Traversal follows Voxblox SimpleTsdfIntegrator::integrateFunction; every
// numerical/color update calls the linked, unmodified upstream implementation.
// Copyright (c) 2016, ETHZ ASL. BSD-3-Clause; see LICENSE.voxblox.
#include <voxblox/integrator/tsdf_integrator.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Tree=boost::property_tree::ptree;
using Key=std::array<std::int64_t,3>;
void require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
template<int N> Eigen::Matrix<double,N,1> vectorIn(const Tree& tree,const std::string& path){
  const auto& array=tree.get_child(path);require(array.size()==N,"Wrong vector size: "+path);
  Eigen::Matrix<double,N,1> value;int i=0;
  for(const auto& item:array){require(item.first.empty(),"Expected array: "+path);value[i++]=item.second.get_value<double>();}
  require(value.allFinite(),"Nonfinite vector: "+path);return value;
}
template<class T> void vectorOut(std::ostream& out,const T& value){
  out<<'[';for(int i=0;i<value.size();++i){if(i)out<<',';out<<value[i];}out<<']';
}
void numberOut(std::ostream& out,double value){if(std::isfinite(value))out<<value;else out<<"null";}
void stringOut(std::ostream& out,const std::string& value){
  out<<'"';for(unsigned char c:value){
    if(c=='"'||c=='\\')out<<'\\'<<c;
    else if(c<0x20){const char* hex="0123456789abcdef";out<<"\\u00"<<hex[c>>4]<<hex[c&15];}
    else out<<c;
  }out<<'"';
}
Key key(const voxblox::GlobalIndex& index){return {index.x(),index.y(),index.z()};}
bool sameFloat(float a,float b){std::uint32_t x,y;std::memcpy(&x,&a,4);std::memcpy(&y,&b,4);return x==y;}
bool sameColor(const voxblox::Color& a,const voxblox::Color& b){return a.r==b.r&&a.g==b.g&&a.b==b.b&&a.a==b.a;}
int occupancy(const voxblox::TsdfVoxel& voxel,double resolution,double factor){
  if(!std::isfinite(voxel.weight)||!std::isfinite(voxel.distance)||voxel.weight<1e-6)return -1;
  return voxel.distance<=factor*resolution+1e-6?100:0;
}
void voxelOut(std::ostream& out,const voxblox::TsdfVoxel& voxel,double resolution,double factor){
  out<<"{\"distance\":";numberOut(out,voxel.distance);out<<",\"weight\":";numberOut(out,voxel.weight);
  out<<",\"color\":["<<int(voxel.color.r)<<','<<int(voxel.color.g)<<','<<int(voxel.color.b)<<','<<int(voxel.color.a)
     <<"],\"occupancy\":"<<occupancy(voxel,resolution,factor)<<'}';
}
struct CapturedVoxel {bool present;voxblox::TsdfVoxel value;int occupancy;};
CapturedVoxel voxelIn(const Tree& tree){
  CapturedVoxel result{tree.get<bool>("present"),{},tree.get<int>("occupancy")};
  if(!result.present){require(result.occupancy==-1,"Absent voxel must be unknown");return result;}
  result.value.distance=tree.get<float>("distance");result.value.weight=tree.get<float>("weight");
  require(std::isfinite(result.value.distance)&&std::isfinite(result.value.weight)&&result.value.weight>=0.,
          "Nonfinite/negative captured voxel state");
  const auto color=vectorIn<4>(tree,"color");
  for(int i=0;i<4;++i)require(color[i]>=0&&color[i]<=255&&std::floor(color[i])==color[i],"Invalid color byte");
  result.value.color=voxblox::Color(color[0],color[1],color[2],color[3]);return result;
}
struct Configuration {
  double resolution,min_range,occupancy_factor;std::size_t voxels_per_side;
  voxblox::TsdfIntegratorBase::Config integrator;
};
Configuration configurationIn(const Tree& source){
  const auto& c=source.get_child("config");Configuration result;
  require(c.get<std::string>("method")=="simple"&&c.get<int>("threads")==1,
          "Exact scratch replay requires simple with one worker");
  require(!c.get<bool>("use_color"),"This replay currently supports the captured uncolored sensor mode only");
  result.resolution=c.get<double>("resolution");result.min_range=c.get<double>("min_range");
  result.occupancy_factor=c.get<double>("occupancy_factor");result.voxels_per_side=c.get<std::size_t>("voxels_per_side");
  require(std::isfinite(result.resolution)&&result.resolution>0.&&std::isfinite(result.min_range)&&result.min_range>0.&&
          std::isfinite(result.occupancy_factor)&&result.occupancy_factor>=0.&&result.voxels_per_side>0&&result.voxels_per_side<=1024,
          "Invalid captured map configuration");
  require(float(result.resolution)==source.get<float>("layer.voxel_size")&&
          result.voxels_per_side==source.get<std::size_t>("layer.voxels_per_side"),"Captured layer/config disagree");
  auto& p=result.integrator;
  p.default_truncation_distance=c.get<float>("truncation_distance");
  p.min_ray_length_m=result.min_range;p.max_ray_length_m=c.get<float>("max_range");
  p.max_weight=c.get<float>("max_weight");p.clearing_ray_weight_factor=c.get<float>("clearing_weight");
  p.use_weight_dropoff=c.get<bool>("weight_dropoff");p.voxel_carving_enabled=c.get<bool>("carving");
  p.use_const_weight=c.get<bool>("use_const_weight");p.use_symmetric_weight_dropoff=c.get<bool>("use_symmetric_weight_dropoff");
  p.weight_ray_by_range=c.get<bool>("weight_ray_by_range");p.allow_clear=c.get<bool>("allow_clear");
  p.integrator_threads=1;p.integration_order_mode=c.get<std::string>("integration_order_mode");
  p.start_voxel_subsampling_factor=c.get<float>("start_voxel_subsampling_factor");
  p.max_consecutive_ray_collisions=c.get<int>("max_consecutive_ray_collisions");
  p.clear_checks_every_n_frames=c.get<int>("clear_checks_every_n_frames");
  p.max_integration_time_s=c.get<float>("max_integration_time_s");p.enable_anti_grazing=c.get<bool>("anti_grazing");
  require(p.integration_order_mode=="mixed"||p.integration_order_mode=="sorted","Unsupported integration order");
  require(std::isfinite(p.default_truncation_distance)&&p.default_truncation_distance>0.&&
          std::isfinite(p.max_ray_length_m)&&p.max_ray_length_m>0.&&std::isfinite(p.max_weight)&&p.max_weight>0.&&
          std::isfinite(p.clearing_ray_weight_factor)&&p.clearing_ray_weight_factor>0.&&p.clearing_ray_weight_factor<=1.&&
          std::isfinite(p.start_voxel_subsampling_factor)&&p.start_voxel_subsampling_factor>0.,"Invalid integrator configuration");
  require(p.use_symmetric_weight_dropoff||p.use_const_weight||p.use_weight_dropoff,"No valid weight policy");
  return result;
}
float readFloat(const std::uint8_t* data,bool big_endian){
  std::uint32_t bits=0;
  if(big_endian)for(int i=0;i<4;++i)bits=(bits<<8)|data[i];
  else for(int i=0;i<4;++i)bits|=std::uint32_t(data[i])<<(8*i);
  float value;std::memcpy(&value,&bits,4);return value;
}
struct Cloud {
  voxblox::Pointcloud points;std::vector<std::size_t> original_indices;
  std::size_t rows{0},skipped{0};std::filesystem::path path;
};
Cloud cloudIn(const Tree& source,const std::filesystem::path& directory,const Configuration& config){
  const auto& c=source.get_child("cloud");Cloud result;
  const auto width=c.get<std::uint64_t>("width"),height=c.get<std::uint64_t>("height");
  const auto point_step=c.get<std::uint64_t>("point_step"),row_step=c.get<std::uint64_t>("row_step");
  require(width>0&&height>0&&height<=200000/width&&point_step>=12&&point_step<=UINT32_MAX&&
          row_step<=UINT32_MAX&&row_step>=width*point_step,
          "Invalid captured cloud shape");
  require(height<=std::numeric_limits<std::size_t>::max()/row_step&&row_step*height<=1024ull*1024*1024,
          "Captured cloud byte length overflow");
  const auto bytes=std::size_t(row_step*height);require(bytes==c.get<std::size_t>("bytes"),"Captured byte count mismatch");
  const auto filename=c.get<std::string>("data_file");
  require(std::filesystem::path(filename).filename()==filename,"Cloud data_file must be a sibling filename");
  result.path=directory/filename;require(std::filesystem::file_size(result.path)==bytes,"Cloud byte length differs from file");
  std::ifstream input(result.path,std::ios::binary);std::vector<std::uint8_t> data(bytes);
  require(bool(input.read(reinterpret_cast<char*>(data.data()),bytes)),"Cannot read captured cloud");
  std::array<std::size_t,3> offsets{};std::set<std::string> found;std::set<std::size_t> field_bytes;
  for(const auto& field:c.get_child("fields")){
    const auto& f=field.second;const auto name=f.get<std::string>("name");
    const auto position=std::string("xyz").find(name);
    if(name.size()!=1||position==std::string::npos)continue;
    const auto offset=f.get<std::size_t>("offset");
    require(found.insert(name).second&&f.get<int>("datatype")==7&&f.get<int>("count")==1&&offset<=point_step-4,
            "Captured XYZ requires nonoverlapping scalar float32 fields");
    for(std::size_t i=offset;i<offset+4;++i)require(field_bytes.insert(i).second,"Overlapping XYZ fields");
    offsets[position]=offset;
  }
  require(found.size()==3,"Missing XYZ fields");
  const bool big_endian=c.get<bool>("is_bigendian");result.rows=width*height;result.points.reserve(result.rows);
  // These double admission checks and subsequent float cast match exactly
  // TsdfBackend::integrateCloud, before the upstream point validity check.
  for(std::size_t row=0;row<height;++row)for(std::size_t col=0;col<width;++col){
    const auto* record=data.data()+row*row_step+col*point_step;
    const Eigen::Vector3d point(readFloat(record+offsets[0],big_endian),readFloat(record+offsets[1],big_endian),readFloat(record+offsets[2],big_endian));
    if(!point.allFinite()||point.norm()<config.min_range||point.norm()>1e5){++result.skipped;continue;}
    const voxblox::Point sensor_point=point.cast<float>();
    if(config.integrator.weight_ray_by_range){
      const float depth=std::abs(sensor_point.z());
      const float weight=depth>voxblox::kFloatEpsilon?1.0f/(depth*depth):0.0f;
      if(!std::isfinite(weight)||weight<=0.f){++result.skipped;continue;}
    }
    result.points.push_back(sensor_point);result.original_indices.push_back(row*width+col);
  }
  require(!result.points.empty(),"Captured cloud has no usable returns");return result;
}
struct RayEvidence {
  std::size_t ordered_index,filtered_point_index,original_point_index;
  voxblox::Point point_C,point_G;bool clearing;float sdf,ray_weight,clearing_weight;
  voxblox::TsdfVoxel before,after;
};
struct Target {
  voxblox::GlobalIndex index;CapturedVoxel before,expected;voxblox::TsdfVoxel state;
  std::size_t updates{0},negative_sdf_updates{0},clearing_updates{0};
  std::vector<RayEvidence> rays;
};
class ReplayIntegrator final:public voxblox::SimpleTsdfIntegrator {
 public:
  using voxblox::SimpleTsdfIntegrator::SimpleTsdfIntegrator;
  std::size_t rejected_points{0};std::uint64_t traversed_voxels{0};
  void replay(const voxblox::Transformation& transform,const Cloud& cloud,bool freespace,std::vector<Target>& targets){
    std::map<Key,std::size_t> lookup;
    for(std::size_t i=0;i<targets.size();++i)require(lookup.emplace(key(targets[i].index),i).second,"Duplicate target voxel");
    std::unique_ptr<voxblox::ThreadSafeIndex> order(voxblox::ThreadSafeIndexFactory::get(config_.integration_order_mode,cloud.points));
    const voxblox::Point origin=transform.getPosition();std::size_t point_index,ordered_index=0;
    while(order->getNextIndex(&point_index)){
      const auto sequence=ordered_index++;const auto& point_C=cloud.points[point_index];bool clearing;
      if(!isPointValid(point_C,freespace,&clearing)){++rejected_points;continue;}
      const voxblox::Point point_G=transform*point_C;
      voxblox::RayCaster ray(origin,point_G,clearing,config_.voxel_carving_enabled,
          config_.max_ray_length_m,voxel_size_inv_,config_.default_truncation_distance);
      const float ray_weight=getVoxelWeight(point_C),clearing_weight=config_.clearing_ray_weight_factor*ray_weight;
      require(std::isfinite(ray_weight)&&ray_weight>0.&&clearing_weight>0.&&clearing_weight<=ray_weight,
              "Invalid upstream ray weight");
      voxblox::GlobalIndex index;
      while(ray.nextRayIndex(&index)){
        ++traversed_voxels;const auto found=lookup.find(key(index));if(found==lookup.end())continue;
        auto& target=targets[found->second];const auto before=target.state;
        const auto center=voxblox::getCenterPointFromGridIndex(index,voxel_size_);
        const float sdf=computeDistance(origin,point_G,center);
        // No map lookup or surrogate math: this is the production numerical,
        // weighting, color, truncation and saturated-weight implementation.
        require(updateTsdfVoxel(origin,point_G,index,voxblox::Color(160,160,160),ray_weight,clearing_weight,&target.state),
                "Simple integrator unexpectedly aborted target update");
        ++target.updates;target.negative_sdf_updates+=sdf<0.f;target.clearing_updates+=clearing;
        if(target.rays.size()<200)target.rays.push_back({sequence,point_index,cloud.original_indices[point_index],
            point_C,point_G,clearing,sdf,ray_weight,clearing_weight,before,target.state});
      }
    }
  }
};
}  // namespace

int main(int argc,char** argv){
  try{
    require(argc==3,"Usage: replay_voxel_scan /path/to/snapshot.json /path/to/output.json");
    const auto source_path=std::filesystem::absolute(argv[1]),output_path=std::filesystem::absolute(argv[2]);
    require(!std::filesystem::exists(output_path),"Refusing to overwrite output");
    Tree source;boost::property_tree::read_json(source_path.string(),source);
    require(source.get<std::string>("status")=="complete","Capture is incomplete");
    const auto config=configurationIn(source);const auto cloud=cloudIn(source,source_path.parent_path(),config);
    const auto origin=vectorIn<3>(source,"integration_transform.origin");
    const auto q=vectorIn<4>(source,"integration_transform.rotation_wxyz");
    const Eigen::Quaterniond rotation(q[0],q[1],q[2],q[3]);
    require(std::isfinite(rotation.norm())&&rotation.norm()>=1e-8,"Invalid captured quaternion");
    // The adapter normalizes in double immediately before casting to kindr.
    const voxblox::Transformation transform(voxblox::Rotation(rotation.normalized().cast<float>()),origin.cast<float>());
    // Independently captured float transform is an identity check; do not
    // normalize it again and silently substitute a different transform.
    const auto captured_origin=vectorIn<3>(source,"integrator_transform_float.origin");
    const auto captured_q=vectorIn<4>(source,"integrator_transform_float.rotation_wxyz");
    const auto actual_q=transform.getRotation().toImplementation();
    for(int axis=0;axis<3;++axis)require(sameFloat(transform.getPosition()[axis],float(captured_origin[axis])),
                                      "Reconstructed float origin differs from captured integrator transform");
    require(sameFloat(actual_q.w(),float(captured_q[0]))&&sameFloat(actual_q.x(),float(captured_q[1]))&&
            sameFloat(actual_q.y(),float(captured_q[2]))&&sameFloat(actual_q.z(),float(captured_q[3])),
            "Reconstructed float quaternion differs from captured integrator transform");
    std::vector<Target> targets;
    for(const auto& entry:source.get_child("voxels")){
      const auto& value=entry.second;const auto index=vectorIn<3>(value,"index");
      for(int axis=0;axis<3;++axis)require(std::floor(index[axis])==index[axis]&&std::abs(index[axis])<=1000000.,"Invalid voxel index");
      Target target;target.index=index.cast<std::int64_t>();target.before=voxelIn(value.get_child("before"));
      target.expected=voxelIn(value.get_child("after"));target.state=target.before.value;targets.push_back(std::move(target));
    }
    require(!targets.empty()&&targets.size()<=2048,"Expected 1-2048 target voxels");
    voxblox::Layer<voxblox::TsdfVoxel> layer(config.resolution,config.voxels_per_side);
    ReplayIntegrator integrator(config.integrator,&layer);
    integrator.replay(transform,cloud,source.get<bool>("freespace"),targets);
    std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
    out<<"{\"status\":\"completed\",\"scope\":\"Single captured scan replay through original Simple TSDF ray traversal and voxel update; no robot, full-map reconstruction or scan-source attribution\",\"snapshot\":";
    stringOut(out,source_path.string());out<<",\"cloud_file\":";stringOut(out,cloud.path.string());
    out<<",\"scan_sequence\":"<<source.get<std::uint64_t>("scan_sequence")<<",\"stamp_ns\":"<<source.get<std::int64_t>("stamp_ns")
       <<",\"source_frame\":";stringOut(out,source.get<std::string>("source_frame"));
    out<<",\"raw_points\":"<<cloud.rows<<",\"adapter_accepted_points\":"<<cloud.points.size()
       <<",\"adapter_skipped_points\":"<<cloud.skipped<<",\"upstream_rejected_points\":"<<integrator.rejected_points
       <<",\"traversed_voxels\":"<<integrator.traversed_voxels<<",\"target_count\":"<<targets.size()
       <<",\"ray_evidence_limit_per_target\":200,\"captured_integrator_transform_exact\":true,\"effective_origin_float\":";vectorOut(out,transform.getPosition());
    const auto effective_q=transform.getRotation().toImplementation();
    out<<",\"effective_rotation_wxyz_float\":["<<effective_q.w()<<','<<effective_q.x()<<','<<effective_q.y()<<','<<effective_q.z()
       <<"],\"limitations\":[\"Only simple, one-worker, uncolored scans are supported.\","
       <<"\"Every target update is replayed; only the first 200 contributing rays per target are listed.\","
       <<"\"Block allocation/presence changes outside target ray visits are not reconstructed.\","
       <<"\"A numerical match validates scan-to-voxel integration, not the physical correctness of scan points or transforms.\"]"
       <<",\"voxels\":[";
    bool all_exact=true,all_occupancy=true;
    for(std::size_t i=0;i<targets.size();++i){
      const auto& t=targets[i];if(i)out<<',';
      const bool distance=sameFloat(t.state.distance,t.expected.value.distance),weight=sameFloat(t.state.weight,t.expected.value.weight);
      const bool color=sameColor(t.state.color,t.expected.value.color);
      const bool status=occupancy(t.state,config.resolution,config.occupancy_factor)==t.expected.occupancy;
      all_exact=all_exact&&distance&&weight&&color;all_occupancy=all_occupancy&&status;
      out<<"{\"index\":";vectorOut(out,t.index);out<<",\"center_float\":";
      vectorOut(out,voxblox::getCenterPointFromGridIndex(t.index,layer.voxel_size()));
      out<<",\"captured_before_present\":"<<t.before.present<<",\"captured_after_present\":"<<t.expected.present
         <<",\"before\":";voxelOut(out,t.before.value,config.resolution,config.occupancy_factor);
      out<<",\"captured_after\":";voxelOut(out,t.expected.value,config.resolution,config.occupancy_factor);
      out<<",\"replayed_after\":";voxelOut(out,t.state,config.resolution,config.occupancy_factor);
      out<<",\"distance_bit_exact\":"<<distance<<",\"weight_bit_exact\":"<<weight<<",\"color_exact\":"<<color
         <<",\"occupancy_matches\":"<<status<<",\"updates\":"<<t.updates<<",\"negative_sdf_updates\":"<<t.negative_sdf_updates
         <<",\"clearing_updates\":"<<t.clearing_updates<<",\"listed_rays\":"<<t.rays.size()<<",\"rays\":[";
      for(std::size_t j=0;j<t.rays.size();++j){
        const auto& r=t.rays[j];if(j)out<<',';
        out<<"{\"integration_order_index\":"<<r.ordered_index<<",\"filtered_point_index\":"<<r.filtered_point_index
           <<",\"original_point_index\":"<<r.original_point_index<<",\"point_C\":";vectorOut(out,r.point_C);
        out<<",\"point_G\":";vectorOut(out,r.point_G);out<<",\"origin\":";vectorOut(out,transform.getPosition());
        out<<",\"is_clearing\":"<<r.clearing<<",\"sdf\":";numberOut(out,r.sdf);
        out<<",\"ray_weight\":";numberOut(out,r.ray_weight);out<<",\"clearing_ray_weight\":";numberOut(out,r.clearing_weight);
        out<<",\"before\":";voxelOut(out,r.before,config.resolution,config.occupancy_factor);
        out<<",\"after\":";voxelOut(out,r.after,config.resolution,config.occupancy_factor);out<<'}';
      }
      out<<"]}";
    }
    out<<"],\"all_numerical_and_color_updates_exact\":"<<all_exact<<",\"all_occupancies_match\":"<<all_occupancy<<"}\n";
    std::ofstream output(output_path);output<<out.str();output.close();require(bool(output),"Cannot write replay output");
    std::cout<<"{\"status\":\""<<(all_exact&&all_occupancy?"matched":"mismatch")<<"\",\"output\":";
    stringOut(std::cout,output_path.string());std::cout<<",\"target_count\":"<<targets.size()<<"}\n";
    return all_exact&&all_occupancy?0:2;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
