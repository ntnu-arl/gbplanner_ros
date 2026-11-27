// Same-process old/new product serialization on the exact same immutable layers.
// This measures synthetic derived-product construction, not live ROS throughput.
#include "baseline_gbplanner_voxblox/map_visualization.hpp"
#include "gbplanner_voxblox/map_visualization.hpp"
#include <openssl/evp.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <typeinfo>
#include <fcntl.h>
#include <unistd.h>

namespace {
using Cloud=sensor_msgs::msg::PointCloud2;
using Tsdf=voxblox::Layer<voxblox::TsdfVoxel>;
using Esdf=voxblox::Layer<voxblox::EsdfVoxel>;
void require(bool ok,const std::string& message){if(!ok)throw std::runtime_error(message);}
void text(std::ostream& out,const std::string& value){
  out<<'"';for(unsigned char c:value){if(c=='"'||c=='\\')out<<'\\'<<c;else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<unsigned(c)<<std::dec;else out<<c;}out<<'"';
}
struct Digest {
  EVP_MD_CTX* context=EVP_MD_CTX_new();
  Digest(){require(context&&EVP_DigestInit_ex(context,EVP_sha256(),nullptr)==1,"SHA256 init failed");}
  ~Digest(){EVP_MD_CTX_free(context);}
  void bytes(const void* data,std::size_t size){require(EVP_DigestUpdate(context,data,size)==1,"SHA256 update failed");}
  void integer(std::uint64_t value){std::uint8_t words[8];for(unsigned i=0;i<8;++i)words[i]=std::uint8_t(value>>(8*i));bytes(words,8);}
  void string(const std::string& value){integer(value.size());bytes(value.data(),value.size());}
  void real(float value){std::uint32_t bits;std::memcpy(&bits,&value,4);integer(bits);}
  std::string finish(){unsigned char result[EVP_MAX_MD_SIZE];unsigned size=0;require(EVP_DigestFinal_ex(context,result,&size)==1&&size==32,"SHA256 final failed");std::ostringstream out;out<<std::hex<<std::setfill('0');for(unsigned i=0;i<size;++i)out<<std::setw(2)<<unsigned(result[i]);return out.str();}
};
std::string cloudHash(const Cloud& cloud){
  Digest h;h.integer(static_cast<std::uint64_t>(cloud.header.stamp.sec));h.integer(cloud.header.stamp.nanosec);h.string(cloud.header.frame_id);
  h.integer(cloud.height);h.integer(cloud.width);h.integer(cloud.fields.size());
  for(const auto& f:cloud.fields){h.string(f.name);h.integer(f.offset);h.integer(f.datatype);h.integer(f.count);}
  h.integer(cloud.is_bigendian);h.integer(cloud.point_step);h.integer(cloud.row_step);h.integer(cloud.is_dense);h.integer(cloud.data.size());h.bytes(cloud.data.data(),cloud.data.size());return h.finish();
}
std::uint32_t word(const Cloud& cloud,std::size_t offset){const auto* p=cloud.data.data()+offset;return std::uint32_t(p[0])|(std::uint32_t(p[1])<<8)|(std::uint32_t(p[2])<<16)|(std::uint32_t(p[3])<<24);}
float floatBits(std::uint32_t bits){float value;std::memcpy(&value,&bits,4);return value;}
void equalCloud(const Cloud& old,const Cloud& now,const std::string& label){
  require(old==now,"Complete PointCloud2 mismatch: "+label);
  require(now.height==1&&!now.is_bigendian&&now.fields.size()==4,"Cloud schema invalid: "+label);
  require(now.row_step==now.width*now.point_step&&now.data.size()==now.row_step,"Cloud byte count invalid: "+label);
  if(now.point_step==32)for(std::size_t i=0;i<now.width;++i){
    const auto offset=i*32;require(word(now,offset+12)==0x3f800000u,"RGB homogeneous field changed");
    require((word(now,offset+16)>>24)==255,"RGB alpha semantics changed");
    for(unsigned p=20;p<32;++p)require(now.data[offset+p]==0,"RGB padding is nonzero");
  }
}
enum Product {TsdfFull,EsdfFull,TsdfSlice,EsdfSlice,Traversable,Surface,ProductCount};
const std::array<const char*,ProductCount> names{{"tsdf_full","esdf_full","tsdf_slice","esdf_slice","traversable","surface_rgb"}};
Cloud product(bool old,Product kind,const Tsdf& tsdf,const Esdf& esdf,unsigned axis=2,float slice=0.f,float radius=.5f){
#define PRODUCT_CALL(function, ...) (old?baseline_gbplanner2::function(__VA_ARGS__):gbplanner2::function(__VA_ARGS__))
  switch(kind){
    case TsdfFull:return PRODUCT_CALL(createDistancePointcloudFromTsdfLayer,tsdf);
    case EsdfFull:return PRODUCT_CALL(createDistancePointcloudFromEsdfLayer,esdf);
    case TsdfSlice:return PRODUCT_CALL(createDistancePointcloudFromTsdfLayerSlice,tsdf,axis,slice);
    case EsdfSlice:return PRODUCT_CALL(createDistancePointcloudFromEsdfLayerSlice,esdf,axis,slice);
    case Traversable:return PRODUCT_CALL(createFreePointcloudFromEsdfLayer,esdf,radius);
    case Surface:return PRODUCT_CALL(createSurfacePointcloudFromTsdfLayer,tsdf);
    default:throw std::runtime_error("Invalid product");
  }
#undef PRODUCT_CALL
}
struct Layers {Tsdf tsdf;Esdf esdf;Layers(float size,std::size_t side):tsdf(size,side),esdf(size,side){}};
std::size_t semantic_checks=0,exception_checks=0;
void compareAll(const Layers& layers,const std::string& label,unsigned axis=2,float slice=0.f,float radius=.5f){
  for(int i=0;i<ProductCount;++i){auto p=static_cast<Product>(i);equalCloud(product(true,p,layers.tsdf,layers.esdf,axis,slice,radius),product(false,p,layers.tsdf,layers.esdf,axis,slice,radius),label+"/"+names[i]);++semantic_checks;}
}
struct ExceptionResult {std::string type,message;};
ExceptionResult rejected(bool old,Product p,const Layers& layers,unsigned axis,float slice,float radius){
  try{product(old,p,layers.tsdf,layers.esdf,axis,slice,radius);}
  catch(const std::invalid_argument& e){return {"invalid_argument",e.what()};}
  catch(const std::exception& e){return {typeid(e).name(),e.what()};}
  return {"none",""};
}
void exceptionSame(Product p,const Layers& layers,unsigned axis,float slice,float radius){
  const auto a=rejected(true,p,layers,axis,slice,radius),b=rejected(false,p,layers,axis,slice,radius);
  require(a.type=="invalid_argument"&&a.type==b.type&&a.message==b.message,"Invalid-parameter behavior changed");++exception_checks;
}
std::array<std::size_t,5> cornerCases(){
  Layers empty(.25f,4);compareAll(empty,"empty_layer");
  Layers unobserved(.25f,4);unobserved.tsdf.allocateBlockPtrByIndex({0,0,0});unobserved.esdf.allocateBlockPtrByIndex({0,0,0});compareAll(unobserved,"allocated_unobserved");
  Layers boundary(.25f,4);
  const float inf=std::numeric_limits<float>::infinity(),nan=floatBits(0x7fc12345u);
  const std::array<float,16> distances{{0.f,-0.f,nan,inf,-inf,.1875f,-.1875f,std::nextafter(.1875f,0.f),std::nextafter(-.1875f,0.f),.5f,std::nextafter(.5f,0.f),std::nextafter(.5f,inf),-.1f,.1f,.25f,-.25f}};
  const std::array<float,16> weights{{1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,0.f,-0.f,1e-3f,std::nextafter(1e-3f,inf),std::nextafter(0.f,1.f),nan,inf}};
  for(const voxblox::BlockIndex& index:std::array<voxblox::BlockIndex,4>{{{-1,0,-1},{0,0,0},{20,-2,0},{-4,3,2}}}){
    auto t=boundary.tsdf.allocateBlockPtrByIndex(index);auto e=boundary.esdf.allocateBlockPtrByIndex(index);
    for(std::size_t i=0;i<t->num_voxels();++i){
      auto& tv=t->getVoxelByLinearIndex(i);auto& ev=e->getVoxelByLinearIndex(i);
      tv.distance=ev.distance=distances[i%16];tv.weight=weights[i%16];ev.observed=i%19!=18;
      tv.color=voxblox::Color(std::uint8_t(i*13),std::uint8_t(i*29),std::uint8_t(i*61),std::uint8_t(i*7));
    }
  }
  // Exercise signed remainders and the exact inclusive slice slab boundary on
  // each axis. Full-layer traversals also preserve blocks far from the origin.
  for(unsigned axis=0;axis<3;++axis)for(float slice:{-.375f,-.25f,-.125f,-.1f,-0.f,0.f,.125f,.25f,.250001f,.250002f,.375f})
    compareAll(boundary,"boundary",axis,slice,.5f);
  for(float radius:{0.f,-0.f,.5f,std::nextafter(.5f,0.f),std::nextafter(.5f,inf)})compareAll(boundary,"radius",2,0.f,radius);
  for(Product p:{TsdfSlice,EsdfSlice}){
    exceptionSame(p,boundary,3,0.f,.5f);
    for(float invalid:{nan,inf,-inf})exceptionSame(p,boundary,2,invalid,.5f);
  }
  for(float invalid:{-1.f,nan,inf,-inf})exceptionSame(Traversable,boundary,2,0.f,invalid);
  std::array<std::size_t,5> special{};const auto cloud=product(false,TsdfFull,boundary.tsdf,boundary.esdf);
  for(std::size_t i=0;i<cloud.width;++i){auto bits=word(cloud,i*16+12);auto value=floatBits(bits);
    special[0]+=bits==0;special[1]+=bits==0x80000000u;special[2]+=std::isnan(value);special[3]+=value==inf;special[4]+=value==-inf;}
  require(std::all_of(special.begin(),special.end(),[](std::size_t n){return n>0;}),"Special float coverage incomplete");
  require(!cloud.is_dense,"Nonfinite intensity must mark cloud non-dense");return special;
}
void fillPerformance(Layers& layers){
  std::size_t serial=0;
  for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y)for(int z=-2;z<=2;++z){
    auto t=layers.tsdf.allocateBlockPtrByIndex({x,y,z});auto e=layers.esdf.allocateBlockPtrByIndex({x,y,z});
    for(std::size_t i=0;i<t->num_voxels();++i,++serial){
      auto& tv=t->getVoxelByLinearIndex(i);auto& ev=e->getVoxelByLinearIndex(i);
      tv.distance=float(int(serial%101)-30)*.01f;tv.weight=serial%97?1.f:1e-3f;
      tv.color=voxblox::Color(std::uint8_t(serial*13),std::uint8_t(serial*29),std::uint8_t(serial*61),std::uint8_t(serial));
      ev.distance=float(int(serial%157)-20)*.01f;ev.observed=serial%13!=0;
    }
  }
  require(serial==512000,"Unexpected performance dataset size");
}
std::string inputHash(const Layers& layers){
  Digest hash;hash.real(layers.tsdf.voxel_size());hash.integer(layers.tsdf.voxels_per_side());
  for(int which=0;which<2;++which){
    voxblox::BlockIndexList blocks;if(which==0)layers.tsdf.getAllAllocatedBlocks(&blocks);else layers.esdf.getAllAllocatedBlocks(&blocks);
    hash.integer(blocks.size());
    for(const auto& index:blocks){
      for(int axis=0;axis<3;++axis)hash.integer(static_cast<std::uint64_t>(index[axis]));
      if(which==0){const auto& block=layers.tsdf.getBlockByIndex(index);for(std::size_t i=0;i<block.num_voxels();++i){const auto& v=block.getVoxelByLinearIndex(i);hash.real(v.distance);hash.real(v.weight);hash.integer(v.color.r);hash.integer(v.color.g);hash.integer(v.color.b);hash.integer(v.color.a);}}
      else {const auto& block=layers.esdf.getBlockByIndex(index);for(std::size_t i=0;i<block.num_voxels();++i){const auto& v=block.getVoxelByLinearIndex(i);hash.real(v.distance);hash.integer(v.observed);}}
    }
  }
  return hash.finish();
}
struct Clock {double wall,cpu;};
Clock now(){timespec c{};require(clock_gettime(CLOCK_THREAD_CPUTIME_ID,&c)==0,"Thread clock failed");return {std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(),double(c.tv_sec)+double(c.tv_nsec)*1e-9};}
struct Timed {Cloud cloud;double wall,cpu;};
Timed timed(bool old,Product p,const Layers& layers){const auto a=now();auto cloud=product(old,p,layers.tsdf,layers.esdf);const auto b=now();return {std::move(cloud),b.wall-a.wall,b.cpu-a.cpu};}
void exclusive(const std::filesystem::path& path,const std::string& bytes){
  int fd=open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0644);require(fd>=0,"Cannot create exclusive result");
  for(std::size_t n=0;n<bytes.size();){auto written=write(fd,bytes.data()+n,bytes.size()-n);if(written<=0){close(fd);throw std::runtime_error("Result write failed");}n+=std::size_t(written);}require(close(fd)==0,"Result close failed");
}
} // namespace

int main(int argc,char** argv){try{
  require(argc==2,"Usage: benchmark_derived_products OUTPUT_JSON (fixed four alternating rounds)");
  const auto output=std::filesystem::absolute(argv[1]);require(!std::filesystem::exists(output),"Refusing to overwrite result");
  static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559,"IEEE754 float32 required");
  const auto special=cornerCases();Layers layers(.2f,16);fillPerformance(layers);const auto original_hash=inputHash(layers);
  std::ostringstream out;out<<std::setprecision(17)<<std::boolalpha;
  out<<"{\"status\":\"passed\",\"scope\":\"Same-process synthetic derived-product construction microbenchmark, not ROS serialization/DDS or whole-system speedup. Both versions visit the same immutable allocated layers.\",\"timed_region\":\"Product function including allocation, traversal, predicates and record construction; excludes input creation, byte comparison, hashing and cloud destruction.\",\"rounds\":4,\"order\":[\"baseline,candidate\",\"candidate,baseline\",\"baseline,candidate\",\"candidate,baseline\"],\"explicit_warmup_runs\":0,\"corner_checks_before_timing\":true,\"voxels_per_layer\":512000,\"blocks_per_layer\":125,\"voxels_per_side\":16,\"voxel_size_m\":0.2,\"slice_axis\":2,\"slice_position_m\":0,\"traversability_radius_m\":0.5,\"input_sha256\":";text(out,original_hash);
  out<<",\"input_hash_encoding\":\"SHA256 of voxel size, side count, actual block traversal order and all used TSDF/ESDF fields; length/integers uint64 LE, floats exact IEEE754 bits as uint64 LE.\",\"comparison\":\"Complete PointCloud2 operator equality: header, ordered fields, dimensions, endianness, strides, density and every data byte; never sorts records. RGB homogeneous word, opaque alpha and every padding byte also checked.\",\"compiler\":";text(out,__VERSION__);
  out<<",\"corner_cases\":{\"cloud_comparisons\":"<<semantic_checks<<",\"invalid_argument_comparisons\":"<<exception_checks<<",\"emitted_special_intensity_counts\":{\"positive_zero\":"<<special[0]<<",\"negative_zero\":"<<special[1]<<",\"nan\":"<<special[2]<<",\"positive_infinity\":"<<special[3]<<",\"negative_infinity\":"<<special[4]<<"}},\"products\":[";
  for(int k=0;k<ProductCount;++k){
    if(k)out<<',';
    const auto p=static_cast<Product>(k);out<<"{\"name\":";text(out,names[k]);out<<",\"runs\":[";std::string first_hash;
    for(int round=0;round<4;++round){
      Timed baseline,candidate;if(round%2==0){baseline=timed(true,p,layers);candidate=timed(false,p,layers);}else{candidate=timed(false,p,layers);baseline=timed(true,p,layers);}
      equalCloud(baseline.cloud,candidate.cloud,names[k]);const auto hash=cloudHash(baseline.cloud);if(round==0)first_hash=hash;else require(hash==first_hash,"Product changed between timing rounds");
      if(round)out<<',';
      out<<"{\"round\":"<<round+1<<",\"baseline_first\":"<<(round%2==0)<<",\"baseline_wall_s\":"<<baseline.wall<<",\"baseline_thread_cpu_s\":"<<baseline.cpu<<",\"candidate_wall_s\":"<<candidate.wall<<",\"candidate_thread_cpu_s\":"<<candidate.cpu<<",\"equal\":true,\"points\":"<<baseline.cloud.width<<",\"bytes\":"<<baseline.cloud.data.size()<<",\"point_step\":"<<baseline.cloud.point_step<<",\"is_dense\":"<<baseline.cloud.is_dense<<'}';
    }
    out<<"],\"cloud_sha256\":";text(out,first_hash);out<<'}';
  }
  require(inputHash(layers)==original_hash,"Input layers changed during benchmark");out<<"],\"input_unchanged\":true,\"all_products_byte_identical\":true}\n";exclusive(output,out.str());
  std::cout<<"Saved "<<output<<" (six products, four alternating rounds; every complete message identical)\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
