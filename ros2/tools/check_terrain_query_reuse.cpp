// Exact terrain-query reuse checks on a tiny analytic, counted map.
#include "gbplanner_local/ground_terrain.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool scalar(double a,double b){return a==b||(std::isnan(a)&&std::isnan(b));}
bool state(const StateVec& a,const StateVec& b){
  for(int i=0;i<4;++i)if(!scalar(a[i],b[i]))return false;
  return true;
}
bool same(const GroundProjection& a,const GroundProjection& b){
  return a.status==b.status&&a.support==b.support&&a.hanging==b.hanging&&
    state(a.projected_state,b.projected_state)&&scalar(a.ground_z,b.ground_z)&&
    scalar(a.projection_distance,b.projection_distance)&&a.ray_index==b.ray_index&&
    scalar(a.zero_crossing_lower_z,b.zero_crossing_lower_z)&&
    scalar(a.zero_crossing_upper_z,b.zero_crossing_upper_z);
}
bool same(const GroundEdge& a,const GroundEdge& b){
  if(a.status!=b.status||a.speculative!=b.speculative||a.actual_length!=b.actual_length||
     a.max_uphill!=b.max_uphill||a.max_downhill!=b.max_downhill||
     a.mean_inclination!=b.mean_inclination||a.polyline.size()!=b.polyline.size())return false;
  for(std::size_t i=0;i<a.polyline.size();++i)if(!state(a.polyline[i],b.polyline[i]))return false;
  return true;
}
class AnalyticMap final:public MapQuery {
 public:
  double scale{1.},slope{0.};bool unknown{false},unknown_positive_x{false};
  VoxelStatus body{VoxelStatus::Free};mutable std::size_t queries{0};
  double resolution()const override{++queries;return .1;}
  Eigen::Vector3d minimum()const override{return {-100.,-100.,-100.};}
  Eigen::Vector3d maximum()const override{return {100.,100.,100.};}
  bool missing(const Eigen::Vector3d& p)const{return unknown||(unknown_positive_x&&p.x()>0.&&p.z()<=0.);}
  VoxelStatus voxel(const Eigen::Vector3d& p)const override{
    ++queries;return missing(p)?VoxelStatus::Unknown:
      p.z()<slope*p.x()?VoxelStatus::Occupied:VoxelStatus::Free;
  }
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& p)const override{
    ++queries;if(missing(p))return std::nullopt;
    return TsdfDistance{scale*(p.z()-slope*p.x()),1.,true};
  }
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& p)const override{
    return tsdfDistance(p);
  }
  VoxelStatus box(const Eigen::Vector3d&,const Eigen::Vector3d&)const override{++queries;return body;}
  VoxelStatus path(const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&)const override{
    ++queries;return body;
  }
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
      const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    ++queries;
    if(missing(a)){visit(a,VoxelStatus::Unknown);return;}
    if(!visit(a,VoxelStatus::Free))return;
    const double height=slope*a.x();
    if(a.z()>=height&&b.z()<height)visit({a.x(),a.y(),height-.05},
      unknown_positive_x&&a.x()>0.?VoxelStatus::Unknown:VoxelStatus::Occupied);
  }
};
GroundParameters parameters(){
  GroundParameters p;p.max_ground_height=.8;p.robot_height=.55;p.robot_size={.2,.2,.2};
  p.max_inclination=.9;p.max_negative_inclination=.9;p.projection_max_length=2.;
  p.interpolate_projection_distance=true;p.tsdf_projection_mode=GroundTsdfProjectionMode::UpstreamStep;
  return p;
}
}
int main(){try{
  auto map=std::make_shared<AnalyticMap>();map->scale=.4;auto p=parameters();
  TerrainProjector baseline(map,p),cached(map,p,TerrainQueryReuse::ImmutableSnapshot);
  const StateVec input(.2,.3,.9,.7);const auto expected=baseline.project(input);
  require(same(expected,cached.project(input)),"Initial TSDF projection changed");
  const auto queries=map->queries;
  StateVec yaw=input;yaw[3]=-2.;auto expected_yaw=expected;expected_yaw.projected_state[3]=yaw[3];
  require(same(cached.project(yaw),expected_yaw)&&map->queries==queries,
          "Exact XYZ reuse lost yaw or queried the map");
  const auto before=cached.queryReuseStats();
  auto ulp=input;ulp.z()=std::nextafter(input.z(),std::numeric_limits<double>::infinity());
  const auto ulp_expected=baseline.project(ulp);
  require(same(cached.project(ulp),ulp_expected)&&
          cached.queryReuseStats().projection_evaluations==before.projection_evaluations+1,
          "A distinct one-ULP input was merged with a cached key");
  const auto second_expected=baseline.project(expected.projected_state);
  require(second_expected.projected_state.z()!=expected.projected_state.z(),
          "Fixture did not exercise non-idempotent upstream TSDF projection");
  require(same(second_expected,cached.project(expected.projected_state)),
          "Projected output was incorrectly treated as an already-cached input");
  auto invalid=input;invalid[3]=std::numeric_limits<double>::quiet_NaN();
  require(same(cached.project(invalid),baseline.project(invalid))&&
          cached.project(invalid).status==GroundStatus::Invalid,"Nonfinite yaw bypassed validity check");

  auto flat=std::make_shared<AnalyticMap>();auto exact=parameters();
  exact.tsdf_projection_mode=GroundTsdfProjectionMode::ZeroCrossingSimplex;
  TerrainProjector flat_base(flat,exact),flat_cached(flat,exact,TerrainQueryReuse::ImmutableSnapshot);
  const StateVec left(-1.,0.,.8,.2),right(1.,0.,.8,-.4);
  const auto fixed=flat_base.validateFixedEdge(left,right);
  require(fixed.admissible()&&same(fixed,flat_cached.validateFixedEdge(left,right)),
          "Fixed edge changed under exact support reuse");
  const auto support_before=flat_cached.queryReuseStats();
  require(same(fixed,flat_cached.validateFixedEdge(left,right)),"Repeated fixed edge changed");
  const auto support_after=flat_cached.queryReuseStats();
  require(support_after.support_hits>support_before.support_hits&&
          support_after.support_evaluations==support_before.support_evaluations,
          "Repeated fixed edge did not reuse exact support rays");

  auto ramp=std::make_shared<AnalyticMap>();ramp->slope=.3;auto directed=exact;
  directed.max_negative_inclination=.1;
  TerrainProjector ramp_base(ramp,directed),ramp_cached(ramp,directed,TerrainQueryReuse::ImmutableSnapshot);
  const StateVec lower(-1.,0.,.5,.7),upper(1.,0.,1.1,-.8);
  require(ramp_base.edge(lower,upper).admissible()&&!ramp_base.edge(upper,lower).admissible(),
          "Fixture did not isolate directed slope admission");
  require(same(ramp_base.edge(lower,upper),ramp_cached.edge(lower,upper))&&
          same(ramp_base.edge(upper,lower),ramp_cached.edge(upper,lower)),
          "Support reuse changed edge direction or slope verdict");

  auto gap=std::make_shared<AnalyticMap>();gap->unknown_positive_x=true;
  TerrainProjector gap_base(gap,exact),gap_cached(gap,exact,TerrainQueryReuse::ImmutableSnapshot);
  const auto hanging=gap_base.project(right);
  require(hanging.hanging&&same(hanging,gap_cached.project(right)),"Hanging projection changed");
  const StateVec beyond(2.,0.,.8,.1);
  const auto candidate=gap_base.candidateEdge(right,beyond);
  require(candidate.candidateAdmissible()&&candidate.speculative&&
          !gap_base.edge(right,beyond).admissible(),"Fixture did not distinguish strict and candidate edges");
  require(same(candidate,gap_cached.candidateEdge(right,beyond))&&
          same(gap_base.edge(right,beyond),gap_cached.edge(right,beyond)),
          "Cached support mixed strict and hanging permission policies");

  auto mutable_map=std::make_shared<AnalyticMap>();
  TerrainProjector live(mutable_map,exact),resettable(mutable_map,exact,TerrainQueryReuse::ImmutableSnapshot);
  require(live.project(left).status==GroundStatus::Supported&&
          resettable.project(left).status==GroundStatus::Supported,"Mutable fixture did not start free");
  mutable_map->body=VoxelStatus::Occupied;
  require(live.project(left).status==GroundStatus::Occupied,
          "Default constructor retained a stale mutable-map verdict");
  // Explicit end of the immutable query interval before map ownership changes.
  resettable.clearQueryReuse();
  const auto cleared=resettable.queryReuseStats();
  require(cleared.projection_hits==0&&cleared.projection_evaluations==0&&
          cleared.support_hits==0&&cleared.support_evaluations==0,
          "Reset retained cache accounting");
  require(same(live.project(left),resettable.project(left)),"Reset retained stale projection/body data");
  mutable_map->body=VoxelStatus::Free;mutable_map->unknown=true;resettable.clearQueryReuse();
  require(resettable.project(left).hanging&&same(live.project(left),resettable.project(left)),
          "Reset retained stale support ray data");
  std::cout<<"{\"status\":\"passed\",\"tsdf_non_idempotence_preserved\":true,"
    "\"exact_ulp_separation\":true,\"yaw_restored\":true,\"nonfinite_yaw_rejected\":true,"
    "\"directed_edges_unchanged\":true,\"hanging_permissions_unchanged\":true,"
    "\"default_mutable_map_live\":true,\"reset_clears_projection_and_support\":true,"
    "\"fixed_support_hits\":"<<support_after.support_hits-support_before.support_hits<<"}\n";
  return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
