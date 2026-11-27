// Read-only replay of one captureHomeSnapshot bundle. This does not rebuild
// GlobalMemory, repair a graph, or infer the cause of any earlier native run.
#include "gbplanner_local/sparse_voxel_map.hpp"
#include "gbplanner_local/voxel_index_set.hpp"
#include "gbplanner_local/ground_terrain.hpp"
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
using Key=std::array<int,3>;
void require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
Key key(const Eigen::Vector3i& value){return {value.x(),value.y(),value.z()};}
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
void numberOut(std::ostream& out,double value){if(std::isfinite(value))out<<value;else out<<"null";}
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
GroundParameters parametersIn(const Tree& source){
  GroundParameters p;const auto& g=source.get_child("ground_parameters");
  p.robot_size=vectorIn<3>(source,"robot_size");p.center_offset=vectorIn<3>(source,"center_offset");
  p.robot_height=g.get<double>("robot_height");p.max_ground_height=g.get<double>("max_ground_height");
  p.max_inclination=g.get<double>("max_inclination");p.max_negative_inclination=g.get<double>("max_negative_inclination");
  p.projection_max_length=g.get<double>("projection_max_length");p.hanging_vertex_penalty=g.get<double>("hanging_vertex_penalty");
  p.point_robot=g.get<bool>("point_robot");p.interpolate_projection_distance=g.get<bool>("interpolate_projection_distance");
  const auto ray=g.get<std::string>("ray_projection_mode"),tsdf=g.get<std::string>("tsdf_projection_mode");
  if(ray=="voxel_surface")p.ray_projection_mode=GroundRayProjectionMode::VoxelSurface;
  else if(ray=="upstream_sample")p.ray_projection_mode=GroundRayProjectionMode::UpstreamSample;
  else throw std::runtime_error("Unknown ground ray projection mode");
  if(tsdf=="upstream_step")p.tsdf_projection_mode=GroundTsdfProjectionMode::UpstreamStep;
  else if(tsdf=="zero_crossing")p.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossing;
  else if(tsdf=="zero_crossing_simplex")p.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  else throw std::runtime_error("Unknown ground TSDF projection mode");
  p.validate();return p;
}
struct BoundInput {GlobalBound bound;double reconstruction_error;bool exact;};
BoundInput boundIn(const Tree& source){
  const auto& tree=source.get_child("global_bound");
  const auto center=vectorIn<3>(tree,"center"),half=vectorIn<3>(tree,"half_size");
  const auto values=vectorIn<9>(tree,"rotation");Eigen::Matrix3d rotation;
  for(int row=0;row<3;++row)for(int col=0;col<3;++col)rotation(row,col)=values[row*3+col];
  require((half.array()>0.).all(),"Nonpositive global bound extent");
  const auto type=tree.get<std::string>("type");
  const auto reconstruct=[&]{
    if(type=="kSphere"){
      require(half.x()==half.y()&&half.x()==half.z(),"Sphere half-size differs by axis");
      return GlobalBound::sphere(center,half.x());
    }
    require(type=="kCuboid","Unknown global bound type");
    if((rotation.array()==Eigen::Matrix3d::Identity().array()).all())
      return GlobalBound::axisAligned(center-half,center+half);
    const Eigen::Vector3d corner=center-rotation*half;
    return GlobalBound::fromCorners(corner,corner+rotation.col(0)*(2.*half.x()),
        corner+rotation.col(1)*(2.*half.y()),corner+rotation.col(2)*(2.*half.z()));
  };
  const auto bound=reconstruct();
  const double error=std::max({(bound.center()-center).cwiseAbs().maxCoeff(),
      (bound.halfSize()-half).cwiseAbs().maxCoeff(),(bound.rotation()-rotation).cwiseAbs().maxCoeff()});
  return {bound,error,error==0.};
}
struct CloudLayout {std::size_t count,bytes;};
CloudLayout layoutIn(const Tree& source){
  const auto& c=source.get_child("cloud");
  require(c.get<std::string>("frame")==source.get<std::string>("frame"),"Snapshot cloud frame differs from planning frame");
  require(!c.get<bool>("is_bigendian"),"Only little-endian captured clouds are supported");
  const auto width=c.get<std::uint64_t>("width"),height=c.get<std::uint64_t>("height");
  require(width>0&&height>0&&width<=UINT32_MAX&&height<=UINT32_MAX,"Invalid cloud dimensions");
  require(c.get<std::uint64_t>("point_step")==24&&c.get<std::uint64_t>("row_step")==width*24,
          "Expected packed 24-byte records without row padding");
  require(height<=std::numeric_limits<std::size_t>::max()/(width*24),"Cloud size overflows host size");
  const std::size_t count=width*height,bytes=count*24;
  require(bytes==c.get<std::uint64_t>("bytes"),"Cloud byte count differs from dimensions");
  const std::map<std::string,std::pair<int,int>> expected{
    {"x",{0,7}},{"y",{4,7}},{"z",{8,7}},{"occupancy",{12,1}},
    {"tsdf_distance",{16,7}},{"tsdf_weight",{20,7}}};
  std::set<std::string> seen;
  for(const auto& entry:c.get_child("fields")){
    const auto& f=entry.second;const auto name=f.get<std::string>("name");
    const auto found=expected.find(name);
    require(found!=expected.end()&&seen.insert(name).second,"Unexpected or duplicate cloud field");
    require(f.get<int>("offset")==found->second.first&&f.get<int>("datatype")==found->second.second&&f.get<int>("count")==1,
            "Captured cloud field layout mismatch: "+name);
  }
  require(seen.size()==expected.size(),"Missing captured cloud fields");return {count,bytes};
}
float floatLE(const std::uint8_t* data){
  static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"IEEE float32 required");
  const std::uint32_t bits=std::uint32_t(data[0])|(std::uint32_t(data[1])<<8)|
      (std::uint32_t(data[2])<<16)|(std::uint32_t(data[3])<<24);
  float value;std::memcpy(&value,&bits,4);return value;
}
struct RawRecord {Eigen::Vector3d center;VoxelStatus status;double distance,weight;};
template<class Callback> void readRecords(const std::filesystem::path& path,const CloudLayout& layout,Callback callback){
  require(std::filesystem::file_size(path)==layout.bytes,"map.bin length differs from snapshot metadata");
  std::ifstream input(path,std::ios::binary);require(bool(input),"Cannot open captured map");
  // Bounded sequential reads avoid a second full map-sized byte allocation.
  std::array<std::uint8_t,24*4096> block;
  for(std::size_t offset=0;offset<layout.count;){
    const auto count=std::min<std::size_t>(4096,layout.count-offset);
    require(bool(input.read(reinterpret_cast<char*>(block.data()),count*24)),"Truncated captured map");
    for(std::size_t row=0;row<count;++row){
      const auto* data=block.data()+row*24;std::int8_t status;std::memcpy(&status,data+12,1);
      callback(RawRecord{{floatLE(data),floatLE(data+4),floatLE(data+8)},static_cast<VoxelStatus>(status),floatLE(data+16),floatLE(data+20)});
    }
    offset+=count;
  }
}
std::shared_ptr<SparseVoxelMap> mapIn(const std::filesystem::path& path,const CloudLayout& layout,double resolution){
  auto map=std::make_shared<SparseVoxelMap>(resolution);map->reserve(layout.count);
  VoxelIndexSet seen(layout.count);
  readRecords(path,layout,[&](const RawRecord& r){
    const auto index=map->index(r.center);
    const auto center=map->center(index);
    for(int axis=0;axis<3;++axis)
      require(std::abs(double(float(center[axis]))-r.center[axis])<=resolution*1e-4,"Map record is not an aligned voxel center");
    require(seen.insert(key(index)),"Duplicate captured voxel index");
    map->setObservation(index,r.status,r.distance,r.weight);
  });
  return map;
}
void projectionOut(std::ostream& out,const GroundProjection& p){
  out<<"{\"status\":";stringOut(out,groundStatusName(p.status));
  out<<",\"support\":"<<p.support<<",\"hanging\":"<<p.hanging<<",\"state\":";vectorOut(out,p.projected_state);
  out<<",\"ground_z\":";numberOut(out,p.ground_z);out<<",\"projection_distance\":";numberOut(out,p.projection_distance);
  out<<",\"ray_index\":"<<p.ray_index<<",\"zero_crossing_lower_z\":";numberOut(out,p.zero_crossing_lower_z);
  out<<",\"zero_crossing_upper_z\":";numberOut(out,p.zero_crossing_upper_z);out<<'}';
}
void edgeOut(std::ostream& out,const GroundEdge& edge){
  out<<"{\"status\":";stringOut(out,groundStatusName(edge.status));out<<",\"admissible\":"<<edge.admissible()
     <<",\"speculative\":"<<edge.speculative<<",\"actual_length\":";numberOut(out,edge.actual_length);
  out<<",\"max_uphill\":";numberOut(out,edge.max_uphill);out<<",\"max_downhill\":";numberOut(out,edge.max_downhill);
  out<<",\"polyline_points\":"<<edge.polyline.size()<<'}';
}
struct BodyCells {
  Eigen::Vector3d center;
  Eigen::Vector3i low,high;
  std::size_t cells{0},occupied{0},unknown{0};
  VoxelStatus raw_status{VoxelStatus::Unknown},policy_status{VoxelStatus::Unknown};
  bool outside_bound{false},intersects_geofence{false};
  std::vector<Eigen::Vector3i> listed;
};
BodyCells bodyCells(const SparseVoxelMap& raw,const MapQuery& map,const GlobalBound& bound,
                    const GeofenceSet& fences,bool fences_enabled,const Eigen::Vector3d& center,const Eigen::Vector3d& size){
  BodyCells result;result.center=center;result.raw_status=raw.box(center,size);result.policy_status=map.box(center,size);
  result.outside_bound=!bound.containsBox(center,size);
  result.intersects_geofence=fences_enabled&&fences.violatesBox(center.head<2>(),size.head<2>());
  const double resolution=raw.resolution();const Eigen::Vector3d half=size*.5;
  require(((center-half).array()>=raw.minimum().array()).all()&&((center+half).array()<raw.maximum().array()).all(),
          "Body extends beyond the sparse map index domain; cannot enumerate its unknown exterior");
  // These candidate origin, expanded-AABB arithmetic and inclusive endpoint
  // tests match SparseVoxelMap::box exactly. floor((center +/- half)/res)
  // would change touched-face behavior at decimal/nextafter boundaries.
  const Eigen::Vector3i radius=((half/resolution).array().floor()+1.).cast<int>();
  const Eigen::Vector3i current=raw.index(raw.center(raw.index(center)));
  result.low=(current-radius).cwiseMax(Eigen::Vector3i::Constant(-SparseVoxelMap::kIndexLimit));
  result.high=(current+radius).cwiseMin(Eigen::Vector3i::Constant(SparseVoxelMap::kIndexLimit));
  const auto touches=[&](int axis,int coordinate){
    const double lo=static_cast<double>(coordinate)*resolution-half[axis];
    const double hi=lo+resolution+size[axis];return !(center[axis]<lo||center[axis]>hi);
  };
  for(int axis=0;axis<3;++axis){
    while(result.low[axis]<=result.high[axis]&&!touches(axis,result.low[axis]))++result.low[axis];
    if(result.low[axis]>result.high[axis]){require(result.raw_status==VoxelStatus::Free,"Empty body enumeration disagrees with production box");return result;}
    while(result.high[axis]>result.low[axis]&&!touches(axis,result.high[axis]))--result.high[axis];
  }
  for(int z=result.low.z();z<=result.high.z();++z)for(int y=result.low.y();y<=result.high.y();++y)
    for(int x=result.low.x();x<=result.high.x();++x){
      const Eigen::Vector3i index(x,y,z);const auto status=raw.voxel(raw.center(index));++result.cells;
      if(status==VoxelStatus::Occupied)++result.occupied;
      if(status==VoxelStatus::Unknown)++result.unknown;
      if(status!=VoxelStatus::Free&&result.listed.size()<100)result.listed.push_back(index);
    }
  const auto aggregate=result.occupied?VoxelStatus::Occupied:(result.unknown?VoxelStatus::Unknown:VoxelStatus::Free);
  require(aggregate==result.raw_status,"Enumerated body cells disagree with production SparseVoxelMap::box");return result;
}
void bodyOut(std::ostream& out,const BodyCells& body,const SparseVoxelMap& map,const std::map<Key,RawRecord>& records){
  out<<"{\"center\":";vectorOut(out,body.center);out<<",\"index_low\":";vectorOut(out,body.low);
  out<<",\"index_high\":";vectorOut(out,body.high);out<<",\"raw_box_status\":";stringOut(out,voxelName(body.raw_status));
  out<<",\"policy_box_status\":";stringOut(out,voxelName(body.policy_status));
  out<<",\"outside_global_bound\":"<<body.outside_bound<<",\"intersects_geofence\":"<<body.intersects_geofence
     <<",\"touched_cells\":"<<body.cells<<",\"nonfree_count\":"<<(body.occupied+body.unknown)
     <<",\"occupied_count\":"<<body.occupied<<",\"unknown_count\":"<<body.unknown<<",\"output_limit\":100"
     <<",\"enumeration_matches_production_box\":true,\"nonfree_cells\":[";
  for(std::size_t i=0;i<body.listed.size();++i){
    if(i)out<<',';const auto& index=body.listed[i];const auto center=map.center(index);
    const auto record=records.find(key(index));const auto status=map.voxel(center);
    out<<"{\"index\":";vectorOut(out,index);out<<",\"center\":";vectorOut(out,center);
    out<<",\"occupancy\":"<<int(status)<<",\"status\":";stringOut(out,voxelName(status));
    out<<",\"record_present\":"<<(record!=records.end())<<",\"raw_tsdf_distance\":";
    if(record!=records.end())numberOut(out,record->second.distance);else out<<"null";
    out<<",\"raw_tsdf_weight\":";if(record!=records.end())numberOut(out,record->second.weight);else out<<"null";
    out<<",\"distance_observed\":"<<(record!=records.end()&&record->second.weight>=1e-6)<<'}';
  }
  out<<"]}";
}
bool capturedNumberMatches(const Tree& tree,const std::string& path,double actual){
  const auto value=tree.get<std::string>(path);
  return value=="null"?!std::isfinite(actual):tree.get<double>(path)==actual;
}
} // namespace

int main(int argc,char** argv){
  try{
    require(argc==3,"Usage: diagnose_home_snapshot /path/to/snapshot.json /path/to/output.json");
    const auto source_path=std::filesystem::absolute(argv[1]),output_path=std::filesystem::absolute(argv[2]);
    require(!std::filesystem::exists(output_path),"Refusing to overwrite output");
    Tree source;boost::property_tree::read_json(source_path.string(),source);
    require(source.get<std::string>("status")=="complete","Snapshot capture is not complete");
    require(source.get<std::string>("storage")=="sparse","This tool only restores captured SparseVoxelMap snapshots");
    require(source.get<bool>("ground_robot"),"This diagnostic requires a ground robot");
    const auto layout=layoutIn(source);const auto parameters=parametersIn(source);const auto bound=boundIn(source);
    const auto map_path=source_path.parent_path()/"map.bin";
    const auto raw=mapIn(map_path,layout,source.get<double>("resolution"));
    auto fences=std::make_shared<GeofenceSet>();
    for(const auto& polygon:source.get_child("geofence.polygons")){
      std::vector<Eigen::Vector2d> points;for(const auto& point:polygon.second)points.push_back(vectorIn<2>(point.second,""));
      std::string error;require(fences->add(points,&error),"Cannot restore captured geofence: "+error);
    }
    const bool fences_enabled=source.get<bool>("geofence.enabled");
    std::shared_ptr<const MapQuery> map=raw;
    if(fences_enabled)map=std::make_shared<GeofenceMap>(map,fences);
    map=std::make_shared<BoundedMap>(map,bound.bound);
    // Disabled cache is the same isolated diagnostic policy used at capture.
    TerrainProjector terrain(map,parameters);
    const StateVec root=vectorIn<4>(source,"root"),home=vectorIn<4>(source,"home");
    const auto projected_home=terrain.project(home);
    const auto original_body=bodyCells(*raw,*map,bound.bound,*fences,fences_enabled,
        terrain.baseState(home).head<3>()+parameters.center_offset,parameters.robot_size);
    const auto projected_body=bodyCells(*raw,*map,bound.bound,*fences,fences_enabled,
        terrain.baseState(projected_home.projected_state).head<3>()+parameters.center_offset,parameters.robot_size);
    std::set<Key> wanted;for(const auto* body:{&original_body,&projected_body})
      for(const auto& index:body->listed)wanted.insert(key(index));
    std::map<Key,RawRecord> raw_records;
    if(!wanted.empty())readRecords(map_path,layout,[&](const RawRecord& record){
      const auto index=key(raw->index(record.center));if(wanted.count(index))raw_records.emplace(index,record);
    });

    std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
    out<<"{\"status\":\"completed\",\"scope\":\"Read-only geometry diagnosis of this captured snapshot only; no graph reconstruction or native runtime replay\",\"source_snapshot\":";
    stringOut(out,source_path.string());out<<",\"map_file\":";stringOut(out,map_path.string());
    out<<",\"event\":";stringOut(out,source.get<std::string>("event"));out<<",\"captured_reason\":";stringOut(out,source.get<std::string>("reason",""));
    out<<",\"map_revision\":"<<source.get<int>("map_revision")<<",\"observation_revision\":"<<source.get<int>("observation_revision")
       <<",\"graph_revision\":"<<source.get<std::uint64_t>("graph_revision")<<",\"cloud_stamp_ns\":"<<source.get<std::int64_t>("cloud.stamp_ns")
       <<",\"map_records\":"<<layout.count<<",\"map_bytes\":"<<layout.bytes<<",\"stored_cells\":"<<raw->size()
       <<",\"global_bound_reconstruction_exact\":"<<bound.exact<<",\"global_bound_max_abs_reconstruction_error\":"<<bound.reconstruction_error
       <<",\"geofence_enabled\":"<<fences_enabled<<",\"geofence_polygon_count\":"<<fences->size()
       <<",\"limitations\":[\"Results apply only to the referenced snapshot, not to any earlier native run.\","
       <<"\"Graph persistence is not loaded; only captured home incident edges are checked.\","
       <<"\"GlobalBound reconstruction may round rotated geometry; the exactness flag reports this explicitly.\","
       <<"\"A failed support projection does not establish a usable reprojected home.\","
       <<"\"Nonfree cell lists include absent unknown cells and are capped at 100 per body; counts cover all touched cells.\"]";
    out<<",\"root\":";vectorOut(out,root);out<<",\"captured_home\":";vectorOut(out,home);
    out<<",\"root_projection_chain\":[";StateVec next=root;
    for(int step=0;step<3;++step){
      if(step)out<<',';const auto projection=terrain.project(next);
      out<<"{\"step\":"<<(step+1)<<",\"input\":";vectorOut(out,next);out<<",\"projection\":";projectionOut(out,projection);
      out<<",\"height_change\":";numberOut(out,projection.projected_state.z()-next.z());
      out<<",\"height_difference_from_captured_home\":";numberOut(out,projection.projected_state.z()-home.z());
      out<<",\"xyz_difference_from_captured_home\":";vectorOut(out,(projection.projected_state-home).head<3>());
      if(step==0){
        const auto captured=vectorIn<4>(source,"root_projection.state");
        out<<",\"captured_first_status_matches\":"<<(source.get<std::string>("root_projection.status")==groundStatusName(projection.status))
           <<",\"captured_first_state_exact\":"<<((captured.array()==projection.projected_state.array()).all());
      }
      out<<'}';next=projection.projected_state;
    }
    out<<"],\"home_projection\":";projectionOut(out,projected_home);
    const auto captured_home_projection=vectorIn<4>(source,"home_projection.state");
    out<<",\"home_projection_status_matches\":"<<(source.get<std::string>("home_projection.status")==groundStatusName(projected_home.status))
       <<",\"home_projection_state_exact\":"<<((captured_home_projection.array()==projected_home.projected_state.array()).all())
       <<",\"home_body_status_matches\":"<<(source.get<int>("home_body_status")==int(original_body.policy_status))
       <<",\"home_fixed_self\":";edgeOut(out,terrain.validateFixedEdge(home,home,true));
    out<<",\"projected_home_fixed_self\":";edgeOut(out,terrain.validateFixedEdge(projected_home.projected_state,projected_home.projected_state,true));
    out<<",\"robot_size\":";vectorOut(out,parameters.robot_size);out<<",\"center_offset\":";vectorOut(out,parameters.center_offset);
    out<<",\"original_home_body\":";bodyOut(out,original_body,*raw,raw_records);
    out<<",\"projected_home_body\":";bodyOut(out,projected_body,*raw,raw_records);
    out<<",\"home_edges\":[";bool first=true,statuses_match=true,lengths_match=true;
    std::set<int> neighbors;std::size_t active_count=0;
    for(const auto& entry:source.get_child("home_edges")){
      const auto& e=entry.second;const int id=e.get<int>("neighbor");require(id>0&&neighbors.insert(id).second,"Invalid or duplicate home neighbor");
      const StateVec neighbor=vectorIn<4>(e,"state");const bool fixed=e.get<bool>("fixed"),active=e.get<bool>("active");
      active_count+=active;
      const auto forward=fixed?terrain.validateFixedEdge(home,neighbor,true):terrain.edge(home,neighbor);
      const auto reverse=fixed?terrain.validateFixedEdge(neighbor,home,true):terrain.edge(neighbor,home);
      const bool forward_match=e.get<std::string>("forward")==groundStatusName(forward.status);
      const bool reverse_match=e.get<std::string>("reverse")==groundStatusName(reverse.status);
      const bool forward_length_match=capturedNumberMatches(e,"forward_length",forward.actual_length);
      const bool reverse_length_match=capturedNumberMatches(e,"reverse_length",reverse.actual_length);
      statuses_match=statuses_match&&forward_match&&reverse_match;
      lengths_match=lengths_match&&forward_length_match&&reverse_length_match;
      if(!first)out<<',';first=false;
      out<<"{\"neighbor\":"<<id<<",\"state\":";vectorOut(out,neighbor);out<<",\"active\":"<<active<<",\"fixed\":"<<fixed
         <<",\"captured_forward\":";stringOut(out,e.get<std::string>("forward"));out<<",\"captured_reverse\":";stringOut(out,e.get<std::string>("reverse"));
      out<<",\"forward\":";edgeOut(out,forward);out<<",\"reverse\":";edgeOut(out,reverse);
      out<<",\"forward_status_matches\":"<<forward_match<<",\"reverse_status_matches\":"<<reverse_match
         <<",\"forward_length_exact\":"<<forward_length_match<<",\"reverse_length_exact\":"<<reverse_length_match<<'}';
    }
    require(active_count==source.get<std::size_t>("home_active_degree"),"Captured home degree differs from incident active edges");
    out<<"],\"edge_count\":"<<neighbors.size()<<",\"home_active_degree\":"<<active_count
       <<",\"all_edge_statuses_match\":"<<statuses_match<<",\"all_edge_lengths_exact\":"<<lengths_match<<"}\n";
    std::ofstream output(output_path);require(bool(output),"Cannot open diagnostic output");output<<out.str();output.close();
    require(bool(output),"Cannot write diagnostic output");
    std::cout<<"{\"status\":\"completed\",\"output\":";stringOut(std::cout,output_path.string());
    std::cout<<",\"all_edge_statuses_match\":"<<std::boolalpha<<statuses_match<<",\"all_edge_lengths_exact\":"<<lengths_match<<"}\n";
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
