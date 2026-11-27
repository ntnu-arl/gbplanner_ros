// Same captured RMF bytes, production SparseVoxelMap imports and query semantics.
// This is a record-import/release microbenchmark, not receiveMap callback timing.
// Reuse the existing diagnostic's exact field, policy, bound and request readers
// within this translation unit; its CLI is renamed and is never executed here.
#define main validation_snapshot_diagnostic_unused_main
#include "diagnose_validation_snapshot.cpp"
#undef main

#include <openssl/evp.h>
#include <chrono>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>

namespace {
struct ClockValue {double wall,cpu;};
ClockValue clockValue(){
  timespec cpu{};require(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu)==0,"Cannot read thread CPU clock");
  return {std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),
          double(cpu.tv_sec)+double(cpu.tv_nsec)*1e-9};
}
struct Digest {
  EVP_MD_CTX* context=EVP_MD_CTX_new();
  Digest(){require(context&&EVP_DigestInit_ex(context,EVP_sha256(),nullptr)==1,"Cannot initialize SHA256");}
  ~Digest(){EVP_MD_CTX_free(context);}
  void bytes(const void* data,std::size_t count){require(EVP_DigestUpdate(context,data,count)==1,"SHA256 update failed");}
  void integer(std::uint64_t value){std::uint8_t encoded[8];for(int i=0;i<8;++i)encoded[i]=std::uint8_t(value>>(8*i));bytes(encoded,8);}
  void real(double value){static_assert(sizeof(double)==8&&std::numeric_limits<double>::is_iec559,"IEEE float64 required");std::uint64_t bits;std::memcpy(&bits,&value,8);integer(bits);}
  void text(const std::string& value){integer(value.size());bytes(value.data(),value.size());}
  std::string finish(){
    unsigned char result[EVP_MAX_MD_SIZE];unsigned length=0;
    require(EVP_DigestFinal_ex(context,result,&length)==1&&length==32,"SHA256 final failed");
    std::ostringstream out;out<<std::hex<<std::setfill('0');for(unsigned i=0;i<length;++i)out<<std::setw(2)<<unsigned(result[i]);return out.str();
  }
};
std::string bytesHash(const void* bytes,std::size_t count){Digest digest;digest.bytes(bytes,count);return digest.finish();}
void sampleDigest(Digest& digest,const std::optional<TsdfDistance>& sample){
  digest.integer(sample.has_value());
  if(sample){digest.real(sample->distance);digest.real(sample->weight);digest.integer(sample->interpolated);}
}
void responseDigest(Digest& digest,const Service::Response& response){
  digest.integer(response.valid);digest.text(response.reason);digest.integer(response.map_revision);
  digest.integer(static_cast<std::uint64_t>(response.failure_segment));
}
struct ImportResult {std::shared_ptr<SparseVoxelMap> map;std::size_t known{0},tsdf_known{0};};
ImportResult importRecords(const std::vector<std::uint8_t>& bytes,const CloudLayout& layout,double resolution){
  ImportResult result;result.map=std::make_shared<SparseVoxelMap>(resolution);result.map->reserve(layout.count);
  // Match receiveMap's float32 decode, world-zero index, encoded-center check,
  // full-coordinate duplicate rejection and combined occupancy/TSDF insertion.
  // Header/schema checks and disk reads are performed once outside this timer.
  VoxelIndexSet seen(layout.count);
  for(std::uint32_t row=0;row<layout.height;++row)for(std::uint32_t col=0;col<layout.width;++col){
    const auto* data=bytes.data()+std::uint64_t(row)*layout.row_step+std::uint64_t(col)*layout.point_step;
    float x,y,z;std::int8_t status;
    std::memcpy(&x,data+layout.x,4);std::memcpy(&y,data+layout.y,4);std::memcpy(&z,data+layout.z,4);std::memcpy(&status,data+layout.occupancy,1);
    const Eigen::Vector3d p(x,y,z);const auto index=result.map->index(p);
    const auto center=result.map->center(index);
    for(int k=0;k<3;++k)require(std::abs(double(float(center[k]))-p[k])<=resolution*1e-4,"Unaligned map center");
    require(seen.insert({index.x(),index.y(),index.z()}),"Duplicate voxel");
    if(layout.has_tsdf){
      float distance,weight;std::memcpy(&distance,data+layout.distance,4);std::memcpy(&weight,data+layout.weight,4);
      result.map->setObservation(index,static_cast<VoxelStatus>(status),distance,weight);
      if(weight>=1e-6f)++result.tsdf_known;
    }else result.map->set(index,static_cast<VoxelStatus>(status));
    if(status!=-1)++result.known;
  }
  return result;
}
struct Semantics {
  std::string occupancy,tsdf,simplex,path,segments,all;
  std::size_t free{0},occupied{0},unknown{0},tsdf_present{0},simplex_present{0};
  Service::Response response;
  bool capture_match{false};
};
Semantics evaluate(const Tree& source,const CloudLayout& layout,const std::vector<std::uint8_t>& bytes,
                   const std::shared_ptr<SparseVoxelMap>& map,const PathValidationPolicy& policy,
                   const Service::Request& request,const GlobalBound& bound){
  Digest occupancy,tsdf,simplex,paths,segments;Semantics result;
  for(std::uint32_t row=0;row<layout.height;++row)for(std::uint32_t col=0;col<layout.width;++col){
    const auto* data=bytes.data()+std::uint64_t(row)*layout.row_step+std::uint64_t(col)*layout.point_step;
    const Eigen::Vector3d input(floatLE(data+layout.x),floatLE(data+layout.y),floatLE(data+layout.z));
    const auto index=map->index(input);
    // Evaluate both encoded input and exact canonical center. Digest records
    // follow captured input order, independent of private table iteration order.
    for(const auto& point:std::array<Eigen::Vector3d,2>{input,map->center(index)}){
      for(int k=0;k<3;++k){occupancy.integer(static_cast<std::uint64_t>(index[k]));tsdf.integer(static_cast<std::uint64_t>(index[k]));simplex.integer(static_cast<std::uint64_t>(index[k]));}
      const auto status=map->voxel(point);occupancy.integer(static_cast<std::uint64_t>(status));
      result.free+=status==VoxelStatus::Free;result.occupied+=status==VoxelStatus::Occupied;result.unknown+=status==VoxelStatus::Unknown;
      const auto distance=map->tsdfDistance(point),surface=map->tsdfDistanceSimplex(point);
      sampleDigest(tsdf,distance);sampleDigest(simplex,surface);
      result.tsdf_present+=distance.has_value();result.simplex_present+=surface.has_value();
    }
  }
  const auto& metadata=source.get_child("map_metadata");
  auto fences=std::make_shared<GeofenceSet>();
  for(const auto& polygon:metadata.get_child("geofence.polygons")){
    std::vector<Eigen::Vector2d> points;for(const auto& point:polygon.second)points.push_back(vectorIn<2>(point.second,""));
    std::string error;require(fences->add(points,&error),"Cannot restore geofence: "+error);
  }
  std::shared_ptr<const MapQuery> wrapped=map;
  if(metadata.get<bool>("geofence.enabled"))wrapped=std::make_shared<GeofenceMap>(wrapped,fences);
  wrapped=std::make_shared<BoundedMap>(wrapped,bound);
  const int revision=source.get<int>("map_revision");
  const auto snapshot=std::make_shared<const PathValidationMap>(PathValidationMap{wrapped,revision,{},{}});
  const PathValidationContext context(policy,snapshot);
  PathValidationObservation observation;observation.valid=source.get<bool>("observation.valid");
  observation.state=vectorIn<4>(source,"observation.planning_state");
  context.validate(request,observation,result.response);responseDigest(paths,result.response);
  result.capture_match=result.response.valid==source.get<bool>("response.valid")&&
    result.response.reason==source.get<std::string>("response.reason")&&
    result.response.map_revision==source.get<std::uint64_t>("response.map_revision")&&
    result.response.failure_segment==source.get<std::int32_t>("response.failure_segment");
  std::vector<StateVec> states;
  for(const auto& pose:request.path){StateVec state;require(policy.poseState(pose,state),"Invalid captured pose");states.push_back(state);}
  const auto size=policy.robot_bounds.size(request.bound_mode);
  for(std::size_t i=0;i<states.size();++i){
    const auto j=std::min(i+1,states.size()-1);
    const Eigen::Vector3d segment_size=(request.clearance_segments==0||i<request.clearance_segments)?
      Eigen::Vector3d(size+Eigen::Vector3d::Constant(2.*request.clearance_margin)):size;
    segments.integer(i);segments.integer(j);segments.text(context.validateSegment(states[i],states[j],segment_size));
    segments.integer(static_cast<std::uint64_t>(wrapped->path(states[i].head<3>()+policy.center_offset,
      states[j].head<3>()+policy.center_offset,segment_size)));
  }
  result.occupancy=occupancy.finish();result.tsdf=tsdf.finish();result.simplex=simplex.finish();
  result.path=paths.finish();result.segments=segments.finish();
  Digest all;all.text(result.occupancy);all.text(result.tsdf);all.text(result.simplex);all.text(result.path);all.text(result.segments);
  all.integer(map->size());result.all=all.finish();return result;
}
void saveExclusive(const std::filesystem::path& path,const std::string& bytes){
  const int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0644);require(fd>=0,"Cannot exclusively create output");
  std::size_t offset=0;
  while(offset<bytes.size()){
    const auto written=write(fd,bytes.data()+offset,bytes.size()-offset);
    if(written<=0){close(fd);throw std::runtime_error("Cannot write complete output");}
    offset+=static_cast<std::size_t>(written);
  }
  require(close(fd)==0,"Cannot close output");
}
} // namespace

int main(int argc,char** argv){try{
  require(argc==4,"Usage: benchmark_rmf_map_parse SNAPSHOT_JSON OUTPUT_JSON LABEL (fixed three repeats)");
  const auto source_path=std::filesystem::absolute(argv[1]),output_path=std::filesystem::absolute(argv[2]);
  require(!std::filesystem::exists(output_path),"Refusing to overwrite output");
  const std::uint16_t endian=1;require(*reinterpret_cast<const std::uint8_t*>(&endian)==1,"Microbenchmark requires little-endian host like captured production input");
  std::ifstream source_file(source_path);require(bool(source_file),"Cannot open snapshot.json");
  std::ostringstream source_buffer;source_buffer<<source_file.rdbuf();const auto source_bytes=source_buffer.str();
  std::istringstream source_stream(source_bytes);Tree source;boost::property_tree::read_json(source_stream,source);
  require(source.get<std::string>("status")=="complete"&&source.get<std::string>("event")=="validation_failed","Expected complete validation failure capture");
  require(source.get<std::string>("map_metadata.storage")=="sparse","Requires captured sparse map");
  const auto layout=layoutIn(source);const auto policy=policyIn(source);const auto request=requestIn(source);
  const auto bound=boundIn(source.get_child("map_metadata"));require(bound.reconstruction_error==0.,"Bound reconstruction is not exact");
  require(source.get<int>("map_revision")>=0&&source.get<std::uint64_t>("map_revision")==source.get<std::uint64_t>("response.map_revision"),"Inconsistent captured revision");
  require((policy.robot_bounds.size(request.bound_mode).array()==vectorIn<3>(source,"policy.robot_size").array()).all(),"Robot size mismatch");
  const auto binary_path=source_path.parent_path()/"map.bin";
  require(std::filesystem::file_size(binary_path)==layout.bytes,"map.bin size differs from metadata");
  std::ifstream input(binary_path,std::ios::binary);require(bool(input),"Cannot open map.bin");
  std::vector<std::uint8_t> bytes(layout.bytes);
  require(bool(input.read(reinterpret_cast<char*>(bytes.data()),bytes.size())),"Cannot read full map.bin");
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
  out<<"{\"status\":\"completed\",\"label\":";stringOut(out,argv[3]);
  out<<",\"scope\":\"Same-input record_import/release microbenchmark, not production receiveMap callback or ROS throughput. Disk/JSON/schema parsing and semantic verification are outside import timing.\""
     <<",\"repeats\":3,\"warmup_runs\":0,\"separate_untimed_semantic_imports\":1,\"input_snapshot\":";stringOut(out,source_path.string());
  out<<",\"snapshot_sha256\":";stringOut(out,bytesHash(source_bytes.data(),source_bytes.size()));
  out<<",\"map_sha256\":";stringOut(out,bytesHash(bytes.data(),bytes.size()));
  out<<",\"map_records\":"<<layout.count<<",\"map_bytes\":"<<layout.bytes
     <<",\"resolution\":"<<policy.resolution<<",\"request_poses\":"<<request.path.size()
     <<",\"semantic_comparison_points\":"<<2*layout.count
     <<",\"hash_encoding\":\"SHA256; captured row order; input then canonical center; 8-byte little-endian integers and IEEE754 doubles; presence/interpolated flags included. Private table order excluded.\""
     <<",\"compiler\":";stringOut(out,__VERSION__);out<<",\"sizeof_sparse_map\":"<<sizeof(SparseVoxelMap)<<",\"runs\":[";
  for(int iteration=0;iteration<3;++iteration){
    const auto begin=clockValue();auto imported=importRecords(bytes,layout,policy.resolution);const auto parsed=clockValue();
    const auto stored=imported.map->size();
    // No query/context/wrapper is constructed in the timed rounds. Reset the
    // final owner immediately after import to measure the complete map release.
    require(imported.map.use_count()==1,"Map still has references before release timing");
    const auto release_begin=clockValue();imported.map.reset();const auto released=clockValue();
    if(iteration)out<<',';
    out<<"{\"iteration\":"<<iteration+1<<",\"record_import_wall_s\":"<<parsed.wall-begin.wall
       <<",\"record_import_thread_cpu_s\":"<<parsed.cpu-begin.cpu
       <<",\"release_wall_s\":"<<released.wall-release_begin.wall<<",\"release_thread_cpu_s\":"<<released.cpu-release_begin.cpu
       <<",\"stored_cells\":"<<stored<<",\"known_cells\":"<<imported.known<<",\"tsdf_known_cells\":"<<imported.tsdf_known<<'}';
  }
  auto semantic_map=importRecords(bytes,layout,policy.resolution);
  const auto semantic=evaluate(source,layout,bytes,semantic_map.map,policy,request,bound.bound);
  require(semantic.capture_match,"Full ValidatePath response differs from captured request");
  out<<"],\"semantics\":{\"stored_cells\":"<<semantic_map.map->size()
     <<",\"known_cells\":"<<semantic_map.known<<",\"tsdf_known_cells\":"<<semantic_map.tsdf_known
     <<",\"occupancy_query_counts\":{\"free\":"<<semantic.free<<",\"occupied\":"<<semantic.occupied<<",\"unknown\":"<<semantic.unknown<<'}'
     <<",\"tsdf_present_queries\":"<<semantic.tsdf_present<<",\"simplex_present_queries\":"<<semantic.simplex_present
     <<",\"occupancy_sha256\":";stringOut(out,semantic.occupancy);out<<",\"tsdf_sha256\":";stringOut(out,semantic.tsdf);
  out<<",\"simplex_sha256\":";stringOut(out,semantic.simplex);out<<",\"full_validation_sha256\":";stringOut(out,semantic.path);
  out<<",\"all_segments_sha256\":";stringOut(out,semantic.segments);out<<",\"semantic_sha256\":";stringOut(out,semantic.all);
  out<<",\"capture_response_match\":"<<semantic.capture_match<<",\"response\":";responseOut(out,semantic.response);out<<"}}\n";
  saveExclusive(output_path,out.str());
  std::cout<<"Saved "<<output_path<<" (three timed imports/releases, one separate untimed semantic import)\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
