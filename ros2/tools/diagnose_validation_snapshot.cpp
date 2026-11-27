// Read-only replay of captureValidationSnapshot. All admission decisions use
// production PathValidationContext and MapQuery implementations. The zero-
// clearance variant and sampled localization below never modify a live policy.
#include "gbplanner_local/path_validation.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include "gbplanner_local/voxel_index_set.hpp"
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/geofence.hpp"
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
#include <set>
#include <sstream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
using Tree=boost::property_tree::ptree;
using Service=planner_msgs::srv::ValidatePath;
void require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
template<int N> Eigen::Matrix<double,N,1> vectorIn(const Tree& tree,const std::string& path){
  const auto& values=path.empty()?tree:tree.get_child(path);
  require(values.size()==N,"Wrong vector size: "+path);
  Eigen::Matrix<double,N,1> result;int i=0;
  for(const auto& value:values){require(value.first.empty(),"Expected JSON array: "+path);result[i++]=value.second.get_value<double>();}
  require(result.allFinite(),"Nonfinite vector: "+path);return result;
}
template<class T> void vectorOut(std::ostream& out,const T& value){
  out<<'[';for(int i=0;i<value.size();++i){if(i)out<<',';out<<value[i];}out<<']';
}
void stringOut(std::ostream& out,const std::string& value){
  out<<'"';
  for(unsigned char c:value){
    if(c=='"'||c=='\\')out<<'\\'<<c;
    else if(c<0x20){const char* digits="0123456789abcdef";out<<"\\u00"<<digits[c>>4]<<digits[c&15];}
    else out<<c;
  }
  out<<'"';
}
const char* voxelName(VoxelStatus status){
  switch(status){case VoxelStatus::Free:return "free";case VoxelStatus::Occupied:return "occupied";case VoxelStatus::Unknown:return "unknown";}
  return "invalid";
}
struct BoundInput {GlobalBound bound;double reconstruction_error;};
BoundInput boundIn(const Tree& source){
  const auto& tree=source.get_child("global_bound");
  const auto center=vectorIn<3>(tree,"center"),half=vectorIn<3>(tree,"half_size");
  const auto values=vectorIn<9>(tree,"rotation");Eigen::Matrix3d rotation;
  for(int row=0;row<3;++row)for(int col=0;col<3;++col)rotation(row,col)=values[row*3+col];
  require((half.array()>0.).all(),"Nonpositive global bound extent");
  const auto type=tree.get<std::string>("type");
  const auto reconstruct=[&]{
    if(type=="kSphere"){
      require(half.x()==half.y()&&half.x()==half.z()&&tree.get<double>("radius")==half.x(),"Inconsistent sphere geometry");
      return GlobalBound::sphere(center,half.x());
    }
    require(type=="kCuboid","Unsupported global bound type: "+type);
    if((rotation.array()==Eigen::Matrix3d::Identity().array()).all())
      return GlobalBound::axisAligned(center-half,center+half);
    const Eigen::Vector3d corner=center-rotation*half;
    return GlobalBound::fromCorners(corner,corner+rotation.col(0)*(2.*half.x()),
        corner+rotation.col(1)*(2.*half.y()),corner+rotation.col(2)*(2.*half.z()));
  };
  const auto bound=reconstruct();
  const double error=std::max({(bound.center()-center).cwiseAbs().maxCoeff(),
      (bound.halfSize()-half).cwiseAbs().maxCoeff(),(bound.rotation()-rotation).cwiseAbs().maxCoeff()});
  return {bound,error};
}
struct CloudLayout {
  std::size_t count,bytes;
  std::uint32_t width,height,point_step,row_step,x,y,z,occupancy,distance{0},weight{0};
  bool has_tsdf{false};
};
CloudLayout layoutIn(const Tree& source){
  const auto& c=source.get_child("cloud");CloudLayout layout{};
  require(c.get<std::string>("frame")==source.get<std::string>("map_metadata.frame"),"Cloud frame differs from map frame");
  require(!c.get<bool>("is_bigendian"),"Only little-endian captured clouds are supported");
  layout.width=c.get<std::uint32_t>("width");layout.height=c.get<std::uint32_t>("height");
  layout.point_step=c.get<std::uint32_t>("point_step");layout.row_step=c.get<std::uint32_t>("row_step");
  require(layout.height>0&&layout.point_step>0,"Invalid cloud dimensions");
  const auto bytes=std::uint64_t(layout.row_step)*layout.height;
  require(std::uint64_t(layout.width)*layout.point_step<=layout.row_step&&
      bytes==c.get<std::uint64_t>("bytes")&&bytes<=std::numeric_limits<std::size_t>::max(),"Cloud byte count differs from dimensions");
  layout.bytes=static_cast<std::size_t>(bytes);
  layout.count=static_cast<std::size_t>(std::uint64_t(layout.width)*layout.height);
  std::map<std::string,const Tree*> fields;
  for(const auto& entry:c.get_child("fields")){
    require(entry.first.empty(),"Expected cloud field array");
    const auto name=entry.second.get<std::string>("name");
    require(fields.emplace(name,&entry.second).second,"Duplicate cloud field: "+name);
  }
  std::set<std::uint32_t> used;
  const auto field=[&](const std::string& name,int type,std::uint32_t size){
    const auto found=fields.find(name);require(found!=fields.end(),"Missing cloud field: "+name);
    const auto& f=*found->second;const auto offset=f.get<std::uint32_t>("offset");
    require(f.get<int>("datatype")==type&&f.get<int>("count")==1&&std::uint64_t(offset)+size<=layout.point_step,
        "Invalid cloud field: "+name);
    for(std::uint32_t i=0;i<size;++i)require(used.insert(offset+i).second,"Overlapping cloud fields");
    return offset;
  };
  layout.x=field("x",7,4);layout.y=field("y",7,4);layout.z=field("z",7,4);layout.occupancy=field("occupancy",1,1);
  layout.has_tsdf=fields.count("tsdf_distance")!=0;
  require(layout.has_tsdf==(fields.count("tsdf_weight")!=0),"Incomplete captured TSDF field pair");
  if(layout.has_tsdf){layout.distance=field("tsdf_distance",7,4);layout.weight=field("tsdf_weight",7,4);}
  return layout;
}
float floatLE(const std::uint8_t* data){
  static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"IEEE float32 required");
  const std::uint32_t bits=std::uint32_t(data[0])|(std::uint32_t(data[1])<<8)|
      (std::uint32_t(data[2])<<16)|(std::uint32_t(data[3])<<24);
  float value;std::memcpy(&value,&bits,4);return value;
}
std::shared_ptr<SparseVoxelMap> mapIn(const std::filesystem::path& path,const CloudLayout& layout,double resolution){
  require(std::filesystem::file_size(path)==layout.bytes,"map.bin length differs from snapshot metadata");
  std::ifstream input(path,std::ios::binary);require(bool(input),"Cannot open captured map");
  auto map=std::make_shared<SparseVoxelMap>(resolution);map->reserve(layout.count);
  VoxelIndexSet seen(layout.count);
  // Bounded sequential reads support row padding without retaining another
  // map-sized byte buffer; the record import uses the production setters.
  const auto capacity=std::min<std::size_t>(4096,std::max<std::uint32_t>(1,layout.width));
  require(capacity<=std::numeric_limits<std::size_t>::max()/layout.point_step,"Cloud record buffer overflow");
  std::vector<std::uint8_t> block(capacity*layout.point_step);
  for(std::uint32_t row=0;row<layout.height;++row){
    for(std::uint32_t column=0;column<layout.width;){
      const auto count=std::min<std::size_t>(capacity,layout.width-column);
      require(bool(input.read(reinterpret_cast<char*>(block.data()),count*layout.point_step)),"Truncated captured cloud");
      for(std::size_t j=0;j<count;++j){
        const auto* data=block.data()+j*layout.point_step;
        const Eigen::Vector3d point(floatLE(data+layout.x),floatLE(data+layout.y),floatLE(data+layout.z));
        const auto index=map->index(point);const auto center=map->center(index);
        for(int axis=0;axis<3;++axis)
          require(std::abs(double(float(center[axis]))-point[axis])<=resolution*1e-4,"Map record is not an aligned voxel center");
        require(seen.insert({index.x(),index.y(),index.z()}),"Duplicate captured voxel index");
        std::int8_t status;std::memcpy(&status,data+layout.occupancy,1);
        if(layout.has_tsdf)map->setObservation(index,static_cast<VoxelStatus>(status),
            floatLE(data+layout.distance),floatLE(data+layout.weight));
        else map->set(index,static_cast<VoxelStatus>(status));
      }
      column+=static_cast<std::uint32_t>(count);
    }
    input.ignore(layout.row_step-std::uint64_t(layout.width)*layout.point_step);
    require(bool(input),"Truncated cloud row padding");
  }
  return map;
}
PathValidationPolicy policyIn(const Tree& source){
  const auto& p=source.get_child("policy");PathValidationPolicy policy;
  policy.frame=p.get<std::string>("frame");policy.ground_robot=p.get<bool>("ground_robot");
  require(!policy.ground_robot,"Unsupported snapshot: this diagnostic only supports aerial robots");
  policy.resolution=p.get<double>("resolution");policy.center_offset=vectorIn<3>(p,"center_offset");
  require(std::isfinite(policy.resolution)&&policy.resolution>0.,"Invalid policy resolution");
  auto& bounds=policy.robot_bounds;bounds.physical_size=vectorIn<3>(p,"robot_bounds.physical_size");
  bounds.extension=vectorIn<3>(p,"robot_bounds.extension");bounds.min_extension=vectorIn<3>(p,"robot_bounds.min_extension");
  bounds.relax_ratio=p.get<double>("robot_bounds.relax_ratio");bounds.validate();
  require(policy.frame==source.get<std::string>("map_metadata.frame")&&
      policy.resolution==source.get<double>("map_metadata.resolution"),"Policy/map frame or resolution mismatch");
  // Ground-only parameters cannot affect the aerial production branch.
  return policy;
}
Service::Request requestIn(const Tree& source){
  const auto& r=source.get_child("request");Service::Request request;
  request.header.frame_id=r.get<std::string>("frame");
  const auto stamp=r.get<std::int64_t>("stamp_ns");
  const auto sec=stamp/1000000000LL-(stamp%1000000000LL<0);
  require(sec>=INT32_MIN&&sec<=INT32_MAX,"Request stamp outside ROS time range");
  request.header.stamp.sec=static_cast<std::int32_t>(sec);
  request.header.stamp.nanosec=static_cast<std::uint32_t>(stamp-sec*1000000000LL);
  request.bound_mode=r.get<std::int32_t>("bound_mode");
  request.clearance_margin=r.get<double>("clearance_margin");
  request.clearance_segments=r.get<std::uint32_t>("clearance_segments");
  request.first_pose_is_observation=r.get<bool>("first_pose_is_observation");
  require(RobotBounds::validMode(request.bound_mode),"Unsupported snapshot bound mode");
  require(std::isfinite(request.clearance_margin)&&request.clearance_margin>=0.&&request.clearance_margin<=1.,
      "Expected a captured path with clearance in the production range [0, 1]");
  for(const auto& entry:r.get_child("path")){
    require(entry.first.empty(),"Expected path array");
    const auto position=vectorIn<3>(entry.second,"position");
    const auto q=vectorIn<4>(entry.second,"quaternion_xyzw");
    geometry_msgs::msg::Pose pose;pose.position.x=position.x();pose.position.y=position.y();pose.position.z=position.z();
    pose.orientation.x=q[0];pose.orientation.y=q[1];pose.orientation.z=q[2];pose.orientation.w=q[3];
    request.path.push_back(pose);
  }
  require(!request.path.empty()&&request.path.size()<=20000,"Unsupported snapshot path size");
  return request;
}
void responseOut(std::ostream& out,const Service::Response& response){
  out<<"{\"valid\":"<<response.valid<<",\"reason\":";stringOut(out,response.reason);
  out<<",\"map_revision\":"<<response.map_revision<<",\"failure_segment\":"<<response.failure_segment<<'}';
}
void segmentOut(std::ostream& out,const PathValidationContext& wrapped_context,const PathValidationContext& raw_context,
    const MapQuery& wrapped,const SparseVoxelMap& raw,const GlobalBound& bound,const GeofenceSet& fences,bool fences_enabled,
    const StateVec& a,const StateVec& b,const Eigen::Vector3d& offset,const Eigen::Vector3d& size){
  const Eigen::Vector3d start=a.head<3>()+offset,end=b.head<3>()+offset;
  const auto wrapped_reason=wrapped_context.validateSegment(a,b,size),raw_reason=raw_context.validateSegment(a,b,size);
  const auto wrapped_status=wrapped.path(start,end,size),raw_status=raw.path(start,end,size);
  require(wrapped_reason.empty()==(wrapped_status==VoxelStatus::Free)&&raw_reason.empty()==(raw_status==VoxelStatus::Free),
      "Production aerial segment/context status disagreement");
  out<<"{\"body_size\":";vectorOut(out,size);
  out<<",\"wrapped_reason\":";stringOut(out,wrapped_reason.empty()?"ok":wrapped_reason);
  out<<",\"raw_sparse_reason\":";stringOut(out,raw_reason.empty()?"ok":raw_reason);
  out<<",\"wrapped_path_status\":";stringOut(out,voxelName(wrapped_status));
  out<<",\"raw_sparse_path_status\":";stringOut(out,voxelName(raw_status));
  out<<",\"global_bound_accepts_endpoints\":"<<(bound.containsBox(start,size)&&bound.containsBox(end,size))
     <<",\"geofence_intersects_path\":"<<(fences_enabled&&fences.violatesPath(start.head<2>(),end.head<2>(),size.head<2>()))<<'}';
}
void localizationOut(std::ostream& out,const MapQuery& wrapped,const SparseVoxelMap& raw,
                     const Eigen::Vector3d& start,const Eigen::Vector3d& end,const Eigen::Vector3d& size){
  // This is diagnostic sampling, not a substitute collision test. A sparse
  // path can fail at a swept-body cell absent from the centerline ray and at
  // a position between these boxes, even when both lists are empty.
  constexpr std::size_t limit=100;
  std::size_t visited=0,unknown=0,occupied=0,listed=0;
  std::ostringstream cells;cells<<std::setprecision(17);
  std::string ray_error;
  try{raw.ray(start,end,[&](const Eigen::Vector3d& center,VoxelStatus status){
    ++visited;unknown+=status==VoxelStatus::Unknown;occupied+=status==VoxelStatus::Occupied;
    if(status!=VoxelStatus::Free&&listed<limit){
      if(listed++)cells<<',';
      cells<<"{\"index\":";vectorOut(cells,raw.index(center));cells<<",\"center\":";vectorOut(cells,center);
      cells<<",\"status\":";stringOut(cells,voxelName(status));cells<<'}';
    }
    return true;
  });}catch(const std::exception& error){ray_error=error.what();}
  out<<"{\"scope\":\"Production ray centerline and sampled production box queries; neither enumerates the complete swept body. Empty lists do not establish a free path.\",\"centerline_ray\":{\"visited_cells\":"<<visited
     <<",\"unknown_cells\":"<<unknown<<",\"occupied_cells\":"<<occupied<<",\"output_limit\":"<<limit
     <<",\"error\":";stringOut(out,ray_error);out<<",\"nonfree_cells\":["<<cells.str()<<"]}";
  const double length=(end-start).norm();
  require(std::isfinite(length),"Nonfinite failed segment length");
  const std::size_t intervals=static_cast<std::size_t>(std::max(1.,std::min(10000.,std::ceil(length/(raw.resolution()*.25)))));
  std::size_t raw_nonfree=0,wrapped_nonfree=0;listed=0;
  out<<",\"sampled_body_boxes\":{\"sample_count\":"<<(intervals+1)<<",\"actual_step_m\":"<<length/intervals
     <<",\"output_limit\":"<<limit<<",\"nonfree_samples\":[";
  for(std::size_t i=0;i<=intervals;++i){
    const double t=static_cast<double>(i)/intervals;
    const Eigen::Vector3d center=i==0?start:i==intervals?end:Eigen::Vector3d(start+t*(end-start));
    const auto rs=raw.box(center,size),ws=wrapped.box(center,size);
    raw_nonfree+=rs!=VoxelStatus::Free;wrapped_nonfree+=ws!=VoxelStatus::Free;
    if((rs!=VoxelStatus::Free||ws!=VoxelStatus::Free)&&listed<limit){
      if(listed++)out<<',';
      out<<"{\"fraction\":"<<t<<",\"center\":";vectorOut(out,center);
      out<<",\"raw_sparse_box_status\":";stringOut(out,voxelName(rs));
      out<<",\"wrapped_box_status\":";stringOut(out,voxelName(ws));out<<'}';
    }
  }
  out<<"],\"raw_sparse_nonfree_count\":"<<raw_nonfree<<",\"wrapped_nonfree_count\":"<<wrapped_nonfree<<"}}";
}
} // namespace

int main(int argc,char** argv){try{
  require(argc==3,"Usage: diagnose_validation_snapshot /path/to/snapshot.json /path/to/output.json");
  const auto source_path=std::filesystem::absolute(argv[1]),output_path=std::filesystem::absolute(argv[2]);
  require(!std::filesystem::exists(output_path),"Refusing to overwrite output");
  std::ifstream source_file(source_path);require(bool(source_file),"Cannot open snapshot.json");
  std::ostringstream source_bytes;source_bytes<<source_file.rdbuf();
  std::istringstream source_stream(source_bytes.str());Tree source;boost::property_tree::read_json(source_stream,source);
  require(source.get<std::string>("status")=="complete"&&source.get<std::string>("event")=="validation_failed",
      "Expected a complete captureValidationSnapshot failure bundle");
  require(source.get<std::string>("map_metadata.storage")=="sparse","Unsupported snapshot: only sparse storage is supported");
  require(!source.get<bool>("response.valid"),"Expected a rejected request");
  const auto policy=policyIn(source);const auto request=requestIn(source);const auto layout=layoutIn(source);
  const auto& metadata=source.get_child("map_metadata");const auto bound=boundIn(metadata);
  // Refuse rounded reconstruction rather than claim an exact replay of a
  // slightly different rotated bound. Native axis-aligned bounds are exact.
  require(bound.reconstruction_error==0.,"Unsupported snapshot: global bound cannot be reconstructed exactly through production factories");
  const int revision=source.get<int>("map_revision");
  require(revision>=0&&std::uint64_t(revision)==source.get<std::uint64_t>("response.map_revision"),"Snapshot and captured response revisions differ");
  const auto map_path=source_path.parent_path()/"map.bin";
  const auto raw=mapIn(map_path,layout,policy.resolution);
  auto fences=std::make_shared<GeofenceSet>();
  for(const auto& polygon:metadata.get_child("geofence.polygons")){
    std::vector<Eigen::Vector2d> points;
    for(const auto& point:polygon.second)points.push_back(vectorIn<2>(point.second,""));
    std::string error;require(fences->add(points,&error),"Cannot restore captured geofence: "+error);
  }
  const bool fences_enabled=metadata.get<bool>("geofence.enabled");
  std::shared_ptr<const MapQuery> wrapped=raw;
  if(fences_enabled)wrapped=std::make_shared<GeofenceMap>(wrapped,fences);
  wrapped=std::make_shared<BoundedMap>(wrapped,bound.bound);
  const auto wrapped_snapshot=std::make_shared<const PathValidationMap>(PathValidationMap{wrapped,revision,{},{}});
  const auto raw_snapshot=std::make_shared<const PathValidationMap>(PathValidationMap{raw,revision,{},{}});
  const PathValidationContext wrapped_context(policy,wrapped_snapshot),raw_context(policy,raw_snapshot);
  PathValidationObservation observation;observation.valid=source.get<bool>("observation.valid");
  observation.state=vectorIn<4>(source,"observation.planning_state");
  std::vector<StateVec> states;
  for(const auto& pose:request.path){StateVec state;require(policy.poseState(pose,state),"Unsupported snapshot: invalid pose");states.push_back(state);}
  const Eigen::Vector3d robot_size=policy.robot_bounds.size(request.bound_mode);
  const Eigen::Vector3d margin_size=robot_size+Eigen::Vector3d::Constant(2.*request.clearance_margin);
  require((robot_size.array()==vectorIn<3>(source,"policy.robot_size").array()).all()&&
      (margin_size.array()==vectorIn<3>(source,"policy.margin_robot_size").array()).all(),"Reconstructed body dimensions differ from captured policy");
  Service::Response original,original_raw,zero,zero_raw;auto zero_request=request;zero_request.clearance_margin=0.;
  wrapped_context.validate(request,observation,original);raw_context.validate(request,observation,original_raw);
  wrapped_context.validate(zero_request,observation,zero);raw_context.validate(zero_request,observation,zero_raw);
  const bool valid_match=original.valid==source.get<bool>("response.valid");
  const bool reason_match=original.reason==source.get<std::string>("response.reason");
  const bool revision_match=original.map_revision==source.get<std::uint64_t>("response.map_revision");
  const bool segment_match=original.failure_segment==source.get<std::int32_t>("response.failure_segment");
  const bool all_match=valid_match&&reason_match&&revision_match&&segment_match;
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
  out<<"{\"status\":\"completed\",\"scope\":\"Read-only aerial sparse snapshot replay using production PathValidationContext; zero clearance is diagnostic only, not a live admission change.\",\"source_snapshot\":";
  stringOut(out,source_path.string());out<<",\"map_file\":";stringOut(out,map_path.string());
  out<<",\"map_records\":"<<layout.count<<",\"map_bytes\":"<<layout.bytes<<",\"stored_cells\":"<<raw->size()
     <<",\"has_tsdf_fields\":"<<layout.has_tsdf<<",\"global_bound_reconstruction_exact\":true"
     <<",\"geofence_enabled\":"<<fences_enabled<<",\"geofence_polygon_count\":"<<fences->size()
     <<",\"captured_snapshot\":"<<source_bytes.str()
     <<",\"response_match\":{\"valid\":"<<valid_match<<",\"reason\":"<<reason_match
     <<",\"map_revision\":"<<revision_match<<",\"failure_segment\":"<<segment_match<<",\"all\":"<<all_match<<'}'
     <<",\"original_clearance_margin\":"<<request.clearance_margin<<",\"original_margin_is_0_02\":"<<(request.clearance_margin==.02)
     <<",\"full_request\":{\"original_wrapped\":";responseOut(out,original);
  out<<",\"original_raw_sparse\":";responseOut(out,original_raw);
  out<<",\"zero_clearance_wrapped\":";responseOut(out,zero);
  out<<",\"zero_clearance_raw_sparse\":";responseOut(out,zero_raw);out<<'}';
  const Eigen::Vector3d observed_center=policy.baseState(observation.state).head<3>()+policy.center_offset;
  out<<",\"observation_body\":{\"center\":";vectorOut(out,observed_center);out<<",\"size\":";vectorOut(out,robot_size);
  out<<",\"wrapped_box_status\":";stringOut(out,voxelName(wrapped->box(observed_center,robot_size)));
  out<<",\"raw_sparse_box_status\":";stringOut(out,voxelName(raw->box(observed_center,robot_size)));out<<'}';
  out<<",\"segments\":[";
  for(std::size_t i=0;i<states.size();++i){
    if(i)out<<',';
    const auto next=std::min(i+1,states.size()-1);
    const bool has_margin=request.clearance_segments==0||i<request.clearance_segments;
    const Eigen::Vector3d size=has_margin?margin_size:robot_size;
    out<<"{\"index\":"<<i<<",\"next_index\":"<<next<<",\"terminal_self_check\":"<<(i==next)
       <<",\"clearance_applied\":"<<has_margin<<",\"planning_from\":";vectorOut(out,states[i]);
    out<<",\"planning_to\":";vectorOut(out,states[next]);
    out<<",\"original\":";segmentOut(out,wrapped_context,raw_context,*wrapped,*raw,bound.bound,*fences,fences_enabled,
        states[i],states[next],policy.center_offset,size);
    out<<",\"zero_clearance\":";segmentOut(out,wrapped_context,raw_context,*wrapped,*raw,bound.bound,*fences,fences_enabled,
        states[i],states[next],policy.center_offset,robot_size);
    out<<'}';
  }
  out<<"],\"replayed_failure_localization\":";
  if(original.failure_segment>=0&&static_cast<std::size_t>(original.failure_segment)<states.size()){
    const auto i=static_cast<std::size_t>(original.failure_segment),next=std::min(i+1,states.size()-1);
    const Eigen::Vector3d start=states[i].head<3>()+policy.center_offset,end=states[next].head<3>()+policy.center_offset;
    const Eigen::Vector3d size=(request.clearance_segments==0||i<request.clearance_segments)?margin_size:robot_size;
    out<<"{\"segment\":"<<i<<",\"original\":";localizationOut(out,*wrapped,*raw,start,end,size);
    out<<",\"zero_clearance\":";localizationOut(out,*wrapped,*raw,start,end,robot_size);out<<'}';
  }else out<<"null";
  out<<"}\n";
  std::ofstream output(output_path);require(bool(output),"Cannot open diagnostic output");output<<out.str();output.close();
  require(bool(output),"Cannot write diagnostic output");
  std::cout<<std::boolalpha<<"{\"status\":\"completed\",\"output\":";stringOut(std::cout,output_path.string());
  std::cout<<",\"response_matches_capture\":"<<all_match<<",\"original_response\":";responseOut(std::cout,original);
  std::cout<<",\"zero_clearance_response\":";responseOut(std::cout,zero);std::cout<<"}\n";
  return all_match?0:2;
}catch(const std::exception& error){
  std::cerr<<"{\"status\":\"error\",\"reason\":";stringOut(std::cerr,error.what());std::cerr<<"}\n";return 1;
}}
