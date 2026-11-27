// Exact simplex interpolation and per-ray context dispatch on small maps.
#include "gbplanner_local/geofence.hpp"
#include "gbplanner_local/global_bound.hpp"
#include "gbplanner_local/ground_terrain.hpp"
#include "gbplanner_local/sparse_voxel_map.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool bits(double a,double b){return std::memcmp(&a,&b,sizeof(double))==0;}
bool same(const std::optional<TsdfDistance>& a,const std::optional<TsdfDistance>& b){
  return a.has_value()==b.has_value()&&(!a||(bits(a->distance,b->distance)&&
      bits(a->weight,b->weight)&&a->interpolated==b->interpolated));
}
bool scalar(double a,double b){return bits(a,b)||(std::isnan(a)&&std::isnan(b));}
bool same(const GroundProjection& a,const GroundProjection& b){
  if(a.status!=b.status||a.support!=b.support||a.hanging!=b.hanging||a.ray_index!=b.ray_index||
     !scalar(a.ground_z,b.ground_z)||!scalar(a.projection_distance,b.projection_distance)||
     !scalar(a.zero_crossing_lower_z,b.zero_crossing_lower_z)||
     !scalar(a.zero_crossing_upper_z,b.zero_crossing_upper_z))return false;
  for(int i=0;i<4;++i)if(!scalar(a.projected_state[i],b.projected_state[i]))return false;
  return true;
}
Eigen::Vector3d point(const Eigen::Vector3i& lower,const Eigen::Vector3d& fraction){
  return lower.cast<double>()+fraction+Eigen::Vector3d::Constant(.5);
}
void populate(SparseVoxelMap& map,double shift){
  for(int z=0;z<3;++z)for(int y=0;y<3;++y)for(int x=0;x<3;++x){
    const Eigen::Vector3i index(x,y,z);
    map.setTsdf(index,shift+.17*x-.31*y+.23*z+.07*x*y-.11*y*z,
                .2+.09*x+.13*y+.19*z);
  }
}
void equalQuery(const MapQuery& map,const Eigen::Vector3d& p,TsdfQueryContext& context){
  require(same(map.tsdfDistanceSimplex(p),map.tsdfDistanceSimplexCached(p,context)),
          "Cached simplex changed an exact distance, weight, or observation verdict");
}

class AnalyticMap:public MapQuery {
 public:
  mutable std::size_t simplex_queries{0},scalar_queries{0};
  double surface{0.};
  double resolution()const override{return .1;}
  Eigen::Vector3d minimum()const override{return {-10.,-10.,-10.};}
  Eigen::Vector3d maximum()const override{return {10.,10.,10.};}
  VoxelStatus voxel(const Eigen::Vector3d&)const override{return VoxelStatus::Free;}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p)const override{
    ++scalar_queries;return TsdfDistance{p.z()-surface,1.,true};
  }
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& p)const override{
    ++simplex_queries;return TsdfDistance{p.z()-surface,1.,true};
  }
  VoxelStatus box(const Eigen::Vector3d&,const Eigen::Vector3d&)const override{return VoxelStatus::Free;}
  VoxelStatus path(const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&)const override{
    return VoxelStatus::Free;
  }
  void ray(const Eigen::Vector3d&,const Eigen::Vector3d&,
      const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>&)const override{}
};
class ContextMap final:public AnalyticMap {
 public:
  mutable std::size_t context_queries{0},fresh_contexts{0};
  std::optional<TsdfDistance> tsdfDistanceSimplexCached(
      const Eigen::Vector3d& p,TsdfQueryContext& context)const override{
    ++context_queries;
    if(!context.owner){
      require(context.initialized==0,"New ray retained initialized stencil slots");
      ++fresh_contexts;context.owner=this;
    }
    require(context.owner==this,"Terrain context changed backend within a ray");
    return tsdfDistanceSimplex(p);
  }
};
GroundParameters parameters(){
  GroundParameters p;p.max_ground_height=.8;p.robot_height=.55;p.robot_size={.2,.2,.2};
  p.projection_max_length=2.;p.interpolate_projection_distance=true;
  p.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;return p;
}
}

int main(){try{
  auto map=std::make_shared<SparseVoxelMap>(1.);populate(*map,.05);
  TsdfQueryContext context;
  const Eigen::Vector3i lower=Eigen::Vector3i::Zero();
  // All six tetrahedra, equal-axis ties, and a cube face share a stencil.
  const std::array<Eigen::Vector3d,8> fractions{{{.75,.5,.25},{.75,.25,.5},
      {.5,.75,.25},{.25,.75,.5},{.5,.25,.75},{.25,.5,.75},
      {.5,.5,.5},{0.,.25,.75}}};
  for(const auto& fraction:fractions)equalQuery(*map,point(lower,fraction),context);
  require(context.owner==map.get()&&context.lookups<=8&&context.hits>0,
          "One cube did not reuse its raw stencil observations");
  const auto lookups=context.lookups,hits=context.hits;
  for(const auto& fraction:fractions)equalQuery(*map,point(lower,fraction),context);
  require(context.lookups==lookups&&context.hits>hits,"Repeated cube queries performed new lookups");

  const auto old_lookups=context.lookups;
  equalQuery(*map,point({1,0,0},{.75,.5,.25}),context);
  require((context.lower.array()==Eigen::Vector3i(1,0,0).array()).all()&&
          context.lookups>old_lookups,"Changing lower retained an unrelated stencil");
  equalQuery(*map,point(lower,{.75,.5,.25}),context);
  auto other=std::make_shared<SparseVoxelMap>(1.);populate(*other,3.);
  const auto before_backend=context.lookups;
  equalQuery(*other,point(lower,{.75,.5,.25}),context);
  require(context.owner==other.get()&&context.lookups>before_backend,
          "Changing backend reused values from another map");
  equalQuery(*map,point(lower,{.75,.5,.25}),context);

  // Only the endpoints carry weight on this diagonal; absent zero-coefficient
  // vertices must stay unread. A genuinely positive missing vertex fails.
  SparseVoxelMap diagonal(1.);diagonal.setTsdf({0,0,0},-.3,2.);
  diagonal.setTsdf({1,1,1},.7,4.);TsdfQueryContext diagonal_context;
  const auto diagonal_point=point(lower,{.5,.5,.5});
  equalQuery(diagonal,diagonal_point,diagonal_context);
  require(diagonal.tsdfDistanceSimplexCached(diagonal_point,diagonal_context).has_value()&&
          diagonal_context.lookups==2,"Zero coefficients queried absent vertices");
  auto near_diagonal=diagonal_point;
  near_diagonal.x()=std::nextafter(near_diagonal.x(),std::numeric_limits<double>::infinity());
  equalQuery(diagonal,near_diagonal,diagonal_context);
  require(!diagonal.tsdfDistanceSimplexCached(near_diagonal,diagonal_context),
          "A one-ULP positive coefficient was rounded to zero across a missing vertex");
  const auto missing_point=point(lower,{.75,.5,.25});
  equalQuery(diagonal,missing_point,diagonal_context);
  require(!diagonal.tsdfDistanceSimplexCached(missing_point,diagonal_context),
          "Missing positive-coefficient vertex was filled");
  const auto missing_lookups=diagonal_context.lookups;
  equalQuery(diagonal,missing_point,diagonal_context);
  require(diagonal_context.lookups==missing_lookups,"Missing observation was not cached");

  SparseVoxelMap zero_weight(1.);populate(zero_weight,.05);
  zero_weight.set({1,0,0},VoxelStatus::Free);zero_weight.setTsdf({1,0,0},.9,0.);
  TsdfQueryContext zero_context;equalQuery(zero_weight,missing_point,zero_context);
  require(!zero_weight.tsdfDistanceSimplexCached(missing_point,zero_context),
          "Occupancy-only zero-weight vertex supplied TSDF support");

  // Both wrappers constrain motion while still forwarding physical TSDF data.
  auto fences=std::make_shared<GeofenceSet>();
  require(fences->add({{.6,.6},{1.4,.6},{1.4,1.4},{.6,1.4}}),"Invalid fence fixture");
  auto fenced=std::make_shared<GeofenceMap>(map,fences);
  BoundedMap bounded(fenced,GlobalBound::axisAligned({-2.,-2.,-2.},{-.5,-.5,-.5}));
  TsdfQueryContext wrapped_context;
  const auto wrapped_point=point(lower,{.5,.5,.5});
  require(fenced->box(wrapped_point,Eigen::Vector3d::Zero())==VoxelStatus::Occupied&&
          bounded.box(wrapped_point,Eigen::Vector3d::Zero())==VoxelStatus::Occupied,
          "Wrapper fixture did not restrict motion");
  equalQuery(bounded,wrapped_point,wrapped_context);
  require(wrapped_context.owner==map.get()&&wrapped_context.lookups>0,
          "Wrappers did not pass the same context to the sparse backend");
  const auto wrapped_lookups=wrapped_context.lookups;
  equalQuery(bounded,wrapped_point,wrapped_context);
  require(wrapped_context.lookups==wrapped_lookups&&wrapped_context.hits>0,
          "Wrapper chain discarded the query context");

  AnalyticMap fallback;TsdfQueryContext fallback_context;
  require(same(fallback.tsdfDistanceSimplex(wrapped_point),
               fallback.tsdfDistanceSimplexCached(wrapped_point,fallback_context))&&
          fallback.simplex_queries==2&&fallback_context.owner==nullptr,
          "Default context method changed the ordinary backend fallback");
  auto counted=std::make_shared<ContextMap>();auto p=parameters();
  TerrainProjector live(counted,p),cached(counted,p,TerrainQueryReuse::ImmutableSnapshot);
  const StateVec input(.2,.3,.9,.7);const auto expected=live.project(input);
  require(expected.support&&counted->context_queries==0,
          "Default projector opted into immutable stencil caching");
  require(same(expected,cached.project(input))&&counted->context_queries>0&&
          counted->fresh_contexts==1,"Cached projection changed or did not open a ray context");
  auto second=input;second.z()=.91;const auto second_expected=live.project(second);
  require(same(second_expected,cached.project(second))&&counted->fresh_contexts==2,
          "A second support ray reused the previous ray's context");
  const auto contextual=counted->context_queries;counted->surface=.02;
  require(live.project(input).ground_z!=expected.ground_z&&counted->context_queries==contextual,
          "Disabled path retained a stale result or called the cached interface");
  p.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossing;
  TerrainProjector scalar_projector(counted,p,TerrainQueryReuse::ImmutableSnapshot);
  require(scalar_projector.project(input).support&&counted->scalar_queries>0&&
          counted->context_queries==contextual,"Nonsimplex projection entered the simplex cache");
  std::cout<<"{\"status\":\"passed\",\"bit_exact_simplex\":true,\"same_cube_lookups\":"<<lookups
    <<",\"missing_cached\":true,\"zero_coefficients_lazy\":true,\"zero_weight_unknown\":true,"
      "\"backend_and_cube_reset\":true,\"wrappers_delegate\":true,\"default_path_live\":true,"
      "\"per_ray_context\":true,\"nonsimplex_unchanged\":true}\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
