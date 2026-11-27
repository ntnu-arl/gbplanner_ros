// Verify that gain geometry is an observation of existing computation.
#include "gbplanner_local/local_planner.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
bool samePath(const std::vector<StateVec>& a,const std::vector<StateVec>& b){
  if(a.size()!=b.size())return false;
  for(std::size_t i=0;i<a.size();++i)if((a[i]-b[i]).norm()!=0.)return false;
  return true;
}
bool sameGain(const VolumetricGain& a,const VolumetricGain& b){
  return a.gain==b.gain&&a.num_unknown_voxels==b.num_unknown_voxels&&
    a.num_free_voxels==b.num_free_voxels&&a.num_occupied_voxels==b.num_occupied_voxels&&
    a.is_frontier==b.is_frontier;
}
// Count all backend entry points, including resolution and policy queries.
class CountingMap final:public MapQuery {
 public:
  explicit CountingMap(std::shared_ptr<const MapQuery> map):map_(std::move(map)){}
  double resolution()const override {++queries[0];return map_->resolution();}
  Eigen::Vector3d minimum()const override {++queries[1];return map_->minimum();}
  Eigen::Vector3d maximum()const override {++queries[2];return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& point)const override {++queries[3];return map_->voxel(point);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& point)const override {
    ++queries[4];return map_->tsdfDistance(point);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& point)const override {
    ++queries[5];return map_->tsdfDistanceSimplex(point);
  }
  VoxelStatus box(const Eigen::Vector3d& point,const Eigen::Vector3d& size)const override {
    ++queries[6];return map_->box(point,size);
  }
  VoxelStatus path(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& size)const override {
    ++queries[7];return map_->path(a,b,size);
  }
  bool gainAllowed(const Eigen::Vector3d& point)const override {++queries[8];return map_->gainAllowed(point);}
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override {
    ++queries[9];map_->ray(a,b,visit);
  }
  mutable std::array<std::size_t,10> queries{};
 private:
  std::shared_ptr<const MapQuery> map_;
};
std::shared_ptr<VoxelMap> scene(){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-1.,-1.,-.5),Eigen::Vector3i(70,20,25));
  map->fill(VoxelStatus::Free);return map;
}
Parameters parameters(){
  Parameters p;p.local_min={-.5,-.5,-.1};p.local_max={4.5,.5,.1};
  p.robot_size={.1,.1,.1};p.edge_min=.05;p.edge_max=1.;p.overshoot=0.;
  p.max_vertices=5;p.max_attempts=10;p.graph_mode=false;
  p.free_gain=1.;p.path_length_penalty=0.;p.sensor_range=10.;
  p.horizontal_fov=.02;p.horizontal_resolution=.02;p.vertical_fov=.02;p.vertical_resolution=.02;
  p.traverse_length_max=100.;p.traverse_time_max=1000.;
  for(auto& axis:p.axis_sampling){axis.distribution="constant";axis.constant=0.;}
  p.axis_sampling[0].constant=4.;return p;
}
const StateVec root(0.,0.,.6,0.);
GainGeometry paired(const Parameters& p,int id){
  auto map=scene();auto off_map=std::make_shared<CountingMap>(map),on_map=std::make_shared<CountingMap>(map);
  PlanningDiagnosticsOptions options;options.gains=true;options.gain_vertex_id=id;
  LocalPlanner off(off_map,p),on(on_map,p,options);
  const auto a=off.plan(root),b=on.plan(root);
  require(a.success==b.success&&a.reason==b.reason&&a.vertices==b.vertices&&a.edges==b.edges&&
    a.attempts==b.attempts&&a.best_id==b.best_id&&a.gain==b.gain&&a.path_length==b.path_length&&
    a.gain_evaluations==b.gain_evaluations&&a.reused_gains==b.reused_gains&&
    a.gain_voxel_count==b.gain_voxel_count&&a.gain_fov_voxel_count==b.gain_fov_voxel_count&&
    a.gain_visibility_queries==b.gain_visibility_queries&&samePath(a.path,b.path),
    "Gain capture changed the plan or accounting");
  require(off.graph().edge_map_==on.graph().edge_map_,"Gain capture changed graph topology or weights");
  for(const auto& item:off.graph().vertices_map_){
    const auto* v=item.second;const auto* w=on.graph().vertices_map_.at(item.first);
    require((v->state-w->state).norm()==0.&&v->is_leaf_vertex==w->is_leaf_vertex&&
      sameGain(v->vol_gain,w->vol_gain)&&v->vol_gain.accumulative_gain==w->vol_gain.accumulative_gain,
      "Gain capture changed graph state or gains");
  }
  require(off_map->queries==on_map->queries,"Gain capture issued additional map queries");
  require(!off.diagnostics().gains.enabled&&off.diagnostics().gains.sensors.empty()&&
    off.diagnostics().gains.reason=="disabled","Disabled gain capture retained geometry");
  const auto& geometry=on.diagnostics().gains;
  if(geometry.gain_valid)require(sameGain(geometry.gain,on.graph().vertices_map_.at(geometry.vertex_id)->vol_gain),
                               "Captured gain differs from the graph vertex");
  return geometry;
}
void checkContributions(const GainGeometry& geometry){
  require(geometry.complete&&!geometry.partial&&geometry.gain_valid&&geometry.evaluated,
          "Expected complete evaluated gain geometry");
  std::size_t endpoints=0,samples=0;
  for(const auto& sensor:geometry.sensors){
    std::size_t unknown=0,free=0,occupied=0;
    for(const auto& sample:sensor.contributions){
      require(sample.weight>0&&sample.point.allFinite(),"Invalid captured contribution");
      if(sample.status==VoxelStatus::Unknown)unknown+=sample.weight;
      else if(sample.status==VoxelStatus::Free)free+=sample.weight;
      else if(sample.status==VoxelStatus::Occupied)occupied+=sample.weight;
    }
    require(unknown==static_cast<std::size_t>(sensor.result.gain.num_unknown_voxels)&&
      free==static_cast<std::size_t>(sensor.result.gain.num_free_voxels)&&
      occupied==static_cast<std::size_t>(sensor.result.gain.num_occupied_voxels),
      "Weighted contributions differ from original gain counters");
    require(sensor.complete&&sensor.total_samples==sensor.contributions.size()&&
      sensor.total_endpoints==sensor.endpoints.size(),"Complete sensor has missing geometry");
    endpoints+=sensor.endpoints.size();samples+=sensor.contributions.size();
  }
  require(endpoints==geometry.total_endpoints&&samples==geometry.total_samples,"Request totals differ");
}
}
int main(){
  try{
    auto p=parameters();const auto ray=paired(p,0);checkContributions(ray);
    require(ray.mode=="ray"&&ray.vertex_id==0&&ray.source_id==0&&ray.sensors.size()==1&&
      ray.policy=="local_workspace_no_gain_zones"&&ray.reason=="evaluated","Wrong ray viewpoint identity");
    const auto& sensor=ray.sensors.front();
    const Eigen::Vector3d endpoint=root.head<3>()+Eigen::Vector3d(
      p.sensor_range*std::cos(-p.horizontal_fov/2),p.sensor_range*std::sin(-p.horizontal_fov/2),
      p.sensor_range*std::sin(-p.vertical_fov/2));
    require(sensor.endpoints.size()==1&&(sensor.endpoints[0]-endpoint).norm()==0.&&
      sensor.raycast_complete&&sensor.map_resolution_valid&&sensor.map_resolution==.1&&!sensor.roi_valid,
      "Ray endpoint was replaced by a hit or an invented ROI");
    // An independent callback observes the original raycaster's doubles and
    // weights. Compare event-by-event: no snapping, sorting or deduplication.
    auto reference_map=scene();std::vector<GainContribution> expected;
    iterativeGainRaycast(*reference_map,root.head<3>(),{endpoint},1,1,p.horizontal_resolution,p.vertical_resolution,
      {p.gain.nonuniform_ray_cast,p.gain.ray_cast_step_size_multiplier,p.gain.max_ray_samples,false},
      [&](const Eigen::Vector3d& point,VoxelStatus status,std::size_t weight){
        if((point.array()>=reference_map->minimum().array()).all()&&
           (point.array()<reference_map->maximum().array()).all()&&reference_map->gainAllowed(point))
          expected.push_back({point,status,weight});
      });
    require(expected.size()==sensor.contributions.size(),"Captured sample stream has the wrong length");
    bool weighted=false;
    for(std::size_t i=0;i<expected.size();++i){
      const auto& actual=sensor.contributions[i];
      require((expected[i].point-actual.point).norm()==0.&&expected[i].status==actual.status&&
        expected[i].weight==actual.weight,"Captured sample order, coordinates or weights changed");
      weighted=weighted||actual.weight>1;
    }
    require(weighted,"Fixture did not exercise nonuniform sample weights");

    auto voxel_p=p;voxel_p.gain.use_ray_model=false;voxel_p.gain.exp_gain_voxel_size=.1;
    voxel_p.horizontal_fov=1.2;voxel_p.vertical_fov=.6;
    const auto voxel=paired(voxel_p,0);checkContributions(voxel);
    const auto& voxel_sensor=voxel.sensors.front();
    require(voxel.mode=="voxel"&&voxel_sensor.roi_valid&&voxel_sensor.step==.1&&
      !voxel_sensor.map_resolution_valid&&!voxel_sensor.raycast_complete&&
      (voxel_sensor.minimum-(root.head<3>()+voxel_p.local_min)).norm()==0.&&
      (voxel_sensor.maximum-(root.head<3>()+voxel_p.local_max)).norm()==0.&&
      voxel_sensor.endpoints.size()==voxel_sensor.result.visibility_queries&&voxel.total_samples>0,
      "Voxel ROI, sample step or visibility endpoints are incorrect");
    for(const auto& sample:voxel_sensor.contributions)require(sample.weight==1,"Voxel samples were expanded");

    auto clustered=p;clustered.cluster_vertices_for_gain=true;clustered.clustering_radius=10.;
    clustered.leaf_gain_only=true;const auto copy=paired(clustered,1);
    require(copy.reason=="clustered_copy"&&copy.vertex_id==1&&copy.source_id==4&&copy.state_valid&&
      copy.gain_valid&&!copy.evaluated&&!copy.complete&&copy.sensors.empty()&&copy.total_samples==0,
      "Clustered copy fabricated geometry or lost its representative");
    auto leaves=p;leaves.leaf_gain_only=true;const auto skipped=paired(leaves,0);
    require(skipped.reason=="leaf_only_skipped"&&!skipped.evaluated&&skipped.sensors.empty(),
      "Skipped nonleaf fabricated geometry");
    const auto first=paired(leaves,-1);require(first.vertex_id==4&&first.evaluated&&first.complete,
                                            "First actual evaluation selection ignored leaf policy");
    const auto missing=paired(p,999);require(missing.reason=="target_missing"&&!missing.state_valid&&
      missing.sensors.empty()&&!missing.evaluated,"Missing target fabricated geometry");

    // Two sensors share one request-wide cap; neither event counts nor gain
    // scores are capped. DDA avoids adjacent-ray shortcuts in this fixture.
    auto capped=p;capped.max_vertices=2;capped.gain.ray_method="dda";
    SensorParameters dense;dense.name="first";dense.range=1.;dense.horizontal_fov=1.;
    dense.horizontal_resolution=1./60.;dense.vertical_fov=.5;dense.vertical_resolution=.01;
    capped.sensors.push_back(dense);dense.name="second";capped.sensors.push_back(dense);
    const auto limited=paired(capped,0);
    require(limited.truncated&&limited.partial&&!limited.complete&&limited.gain_valid&&
      limited.total_endpoints>GainGeometry::kMaxEndpoints&&limited.total_samples>GainGeometry::kMaxSamples&&
      limited.total_endpoints-limited.dropped_endpoints==GainGeometry::kMaxEndpoints&&
      limited.total_samples-limited.dropped_samples==GainGeometry::kMaxSamples,
      "Request-wide geometry caps changed evaluation or hid truncation");
    std::size_t stored_endpoints=0,stored_samples=0;
    for(const auto& item:limited.sensors){
      require(item.complete,"Storage cap interrupted a sensor evaluation");
      stored_endpoints+=item.endpoints.size();stored_samples+=item.contributions.size();
    }
    require(stored_endpoints==GainGeometry::kMaxEndpoints&&stored_samples==GainGeometry::kMaxSamples,
      "Geometry capacity was applied per sensor instead of per request");

    PlanningDiagnosticsOptions options;options.gains=true;options.gain_vertex_id=0;
    auto failure_p=p;failure_p.gain.max_ray_samples=5;
    auto fail_off_map=std::make_shared<CountingMap>(scene()),fail_on_map=std::make_shared<CountingMap>(scene());
    LocalPlanner fail_off(fail_off_map,failure_p),fail_on(fail_on_map,failure_p,options);
    bool failed_off=false,failed_on=false;
    try{fail_off.plan(root);}catch(const std::length_error&){failed_off=true;}
    try{fail_on.plan(root);}catch(const std::length_error&){failed_on=true;}
    const auto& partial=fail_on.diagnostics().gains;
    require(failed_off&&failed_on&&fail_off_map->queries==fail_on_map->queries&&partial.partial&&
      partial.evaluated&&!partial.complete&&!partial.gain_valid&&partial.reason=="exception"&&
      partial.total_samples>0&&!partial.sensors.front().complete&&!partial.sensors.front().raycast_complete,
      "Exception discarded partial evidence or fabricated complete raycast totals");

    LocalPlanner cached(scene(),p,options,ExactRayGainReuse::ImmutableSnapshot);cached.plan(root);
    const auto before=cached.diagnostics().gains;
    cached.rayGain(root);cached.globalRayGain(root);cached.sensorGains(root);
    const auto& after=cached.diagnostics().gains;
    require(cached.exactRayGainCacheStats().hits==2&&after.total_samples==before.total_samples&&
      after.total_endpoints==before.total_endpoints&&after.sensors.size()==before.sensors.size()&&
      after.vertex_id==before.vertex_id&&after.reason==before.reason,
      "Cached/global/public gain queries overwrote the planning snapshot");
    StateVec invalid=root;invalid.x()=std::numeric_limits<double>::quiet_NaN();cached.plan(invalid);
    const auto& cleared=cached.diagnostics().gains;
    require(cleared.reason=="not_entered"&&!cleared.entered&&!cleared.evaluated&&cleared.sensors.empty()&&
      !cleared.state_valid&&cleared.total_samples==0,"Early return retained a stale gain snapshot");
    std::cout<<"{\"status\":\"passed\",\"ray_samples\":"<<ray.total_samples
      <<",\"voxel_samples\":"<<voxel.total_samples<<",\"capped_total_samples\":"<<limited.total_samples
      <<",\"capped_total_endpoints\":"<<limited.total_endpoints
      <<",\"weighted_order_preserved\":true,\"cluster_copy_has_no_geometry\":true"
      <<",\"partial_exception_retained\":true,\"query_path_graph_gain_unchanged\":true"
      <<",\"additional_map_queries\":0}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
