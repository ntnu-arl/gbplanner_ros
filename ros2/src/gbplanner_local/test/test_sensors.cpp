#include <gtest/gtest.h>
#include "gbplanner_local/local_planner.hpp"
#include <cmath>
#include <limits>

using namespace gbplanner2;

namespace {
std::shared_ptr<VoxelMap> scene(bool occupied_wall=false){
  auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-8,-8,-2),Eigen::Vector3i(64,64,16));
  map->fill(VoxelStatus::Free);
  for(int z=0;z<16;++z)for(int y=0;y<64;++y)for(int x=0;x<64;++x){
    Eigen::Vector3i index(x,y,z);const auto p=map->center(index);
    if(p.x()>=1.)map->set(index,VoxelStatus::Unknown);
    if(occupied_wall&&p.x()>=.5&&p.x()<.75)map->set(index,VoxelStatus::Occupied);
  }
  return map;
}
SensorParameters narrow(const std::string& name){
  SensorParameters s;s.name=name;s.range=3.;
  s.horizontal_fov=.4;s.vertical_fov=.4;s.horizontal_resolution=.2;s.vertical_resolution=.2;
  return s;
}
Eigen::Vector3d originalRotation(Eigen::Vector3d p,const Eigen::Vector3d& ypr){
  // Scalar application of the original direct Rz(Y)*Ry(P)*Rx(R).
  double c=std::cos(ypr[2]),s=std::sin(ypr[2]);
  p={p.x(),c*p.y()-s*p.z(),s*p.y()+c*p.z()};
  c=std::cos(ypr[1]);s=std::sin(ypr[1]);
  p={c*p.x()+s*p.z(),p.y(),-s*p.x()+c*p.z()};
  c=std::cos(ypr[0]);s=std::sin(ypr[0]);
  return {c*p.x()-s*p.y(),s*p.x()+c*p.y(),p.z()};
}
void equalGain(const VolumetricGain& a,const VolumetricGain& b){
  EXPECT_EQ(a.num_unknown_voxels,b.num_unknown_voxels);
  EXPECT_EQ(a.num_free_voxels,b.num_free_voxels);
  EXPECT_EQ(a.num_occupied_voxels,b.num_occupied_voxels);
  EXPECT_DOUBLE_EQ(a.gain,b.gain);EXPECT_EQ(a.is_frontier,b.is_frontier);
}
class RayOrigins final:public MapQuery{
 public:
  double resolution()const override{return .25;}
  Eigen::Vector3d minimum()const override{return {-8,-8,-8};}
  Eigen::Vector3d maximum()const override{return {8,8,8};}
  VoxelStatus voxel(const Eigen::Vector3d&)const override{return VoxelStatus::Free;}
  VoxelStatus box(const Eigen::Vector3d&,const Eigen::Vector3d&)const override{return VoxelStatus::Free;}
  VoxelStatus path(const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&)const override{return VoxelStatus::Free;}
  void ray(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override{
    origins.push_back(a);visit((a+b)*.5,VoxelStatus::Unknown);
  }
  mutable std::vector<Eigen::Vector3d> origins;
};
}

TEST(MultiSensor,OriginalExtrinsicOrderAndBodyOriginArePreserved){
  Parameters p;auto lidar=narrow("offset_lidar");
  lidar.range=2.;lidar.min_range=1.;
  lidar.rotations={.4,-.2,.3};lidar.center_offset={.2,-.1,.3};
  auto camera=lidar;camera.name="fixed_camera";camera.type="kCamera";p.sensors={lidar,camera};
  auto map=std::make_shared<RayOrigins>();LocalPlanner planner(map,p);
  const StateVec state(1.,-2.,.4,.7);
  const auto endpoints=planner.sensorEndpoints(0,state);
  ASSERT_EQ(endpoints.size(),4u);
  std::size_t index=0;
  for(double v:{-.2,0.})for(double h:{-.2,0.}){
    const Eigen::Vector3d ray(lidar.range*std::cos(h),lidar.range*std::sin(h),lidar.range*std::sin(v));
    const Eigen::Vector3d body=originalRotation(ray,lidar.rotations)+lidar.center_offset;
    const Eigen::Vector3d expected=state.head<3>()+originalRotation(body,{state[3],0,0});
    EXPECT_NEAR((endpoints[index++]-expected).norm(),0.,1e-12);
  }
  const auto camera_endpoints=planner.sensorEndpoints(1,state);
  ASSERT_EQ(camera_endpoints.size(),endpoints.size());
  for(std::size_t i=0;i<endpoints.size();++i)EXPECT_EQ(camera_endpoints[i],endpoints[i]);
  auto gains=planner.sensorGains(state);ASSERT_EQ(gains.size(),2u);
  ASSERT_EQ(map->origins.size(),8u);
  for(const auto& origin:map->origins)EXPECT_EQ(origin,state.head<3>());
  // min_range is an annotation property, not a ray-gain exclusion upstream.
  p.sensors[0].min_range=0.;LocalPlanner minimum_zero(map,p);
  equalGain(minimum_zero.sensorGains(state)[0].gain,gains[0].gain);
}

TEST(MultiSensor,OverlappingSensorsAreSummedWithoutDeduplication){
  Parameters p;p.unknown_gain=.7;p.free_gain=.2;p.occupied_gain=.1;
  p.sensors={narrow("a")};LocalPlanner single(scene(),p);const auto a=single.gain(StateVec::Zero());
  auto duplicate=p.sensors[0];duplicate.name="b";p.sensors.push_back(duplicate);
  LocalPlanner both(scene(),p);const auto total=both.gain(StateVec::Zero());
  EXPECT_EQ(total.num_unknown_voxels,2*a.num_unknown_voxels);
  EXPECT_EQ(total.num_free_voxels,2*a.num_free_voxels);
  EXPECT_EQ(total.num_occupied_voxels,2*a.num_occupied_voxels);
  EXPECT_DOUBLE_EQ(total.gain,2*a.gain);EXPECT_EQ(total.is_frontier,a.is_frontier);
  const auto per_sensor=both.sensorGains(StateVec::Zero());ASSERT_EQ(per_sensor.size(),2u);
  equalGain(per_sensor[0].gain,per_sensor[1].gain);
}

TEST(MultiSensor,FrontierThresholdIsPerSensorOrNotPooledAverage){
  Parameters p;auto front=narrow("front");front.range=2.;front.frontier_threshold=.4;
  auto back=narrow("back");back.range=6.;back.rotations={M_PI,0,0};back.frontier_threshold=.4;
  p.sensors={front,back};LocalPlanner planner(scene(),p);
  const auto each=planner.sensorGains(StateVec::Zero());ASSERT_EQ(each.size(),2u);
  EXPECT_TRUE(each[0].gain.is_frontier);EXPECT_FALSE(each[1].gain.is_frontier);
  EXPECT_TRUE(planner.gain(StateVec::Zero()).is_frontier);
  const double pooled=(each[0].gain.num_unknown_voxels+each[1].gain.num_unknown_voxels)*.25/
      ((.4/.2)*(.4/.2)*(front.range+back.range));
  EXPECT_LT(pooled,.4); // An averaged threshold would incorrectly hide this frontier.
}

TEST(MultiSensor,OcclusionStopsOnlyTheSensorLookingThroughWall){
  Parameters p;p.sensors={narrow("forward"),narrow("backward")};p.sensors[1].rotations={M_PI,0,0};
  LocalPlanner visible(scene(),p),blocked(scene(true),p);
  const auto before=visible.sensorGains(StateVec::Zero()),after=blocked.sensorGains(StateVec::Zero());
  ASSERT_GT(before[0].gain.num_unknown_voxels,0);
  EXPECT_EQ(after[0].gain.num_unknown_voxels,0);
  EXPECT_EQ(after[0].gain.num_occupied_voxels,4);
  equalGain(before[1].gain,after[1].gain);
}

TEST(MultiSensor,ExplicitSingleSensorMatchesLegacyGainAndPlanning){
  auto map=scene();Parameters legacy;legacy.max_vertices=30;
  legacy.sensor_range=2.7;legacy.horizontal_fov=1.2;legacy.vertical_fov=.8;
  legacy.horizontal_resolution=.2;legacy.vertical_resolution=.2;
  legacy.frontier_threshold=.12;
  Parameters explicit_sensor=legacy;SensorParameters sensor;
  sensor.name="explicit";sensor.range=legacy.sensor_range;
  sensor.horizontal_fov=legacy.horizontal_fov;sensor.vertical_fov=legacy.vertical_fov;
  sensor.horizontal_resolution=legacy.horizontal_resolution;sensor.vertical_resolution=legacy.vertical_resolution;
  sensor.frontier_threshold=legacy.frontier_threshold;explicit_sensor.sensors={sensor};
  LocalPlanner a(map,legacy),b(map,explicit_sensor);
  for(double yaw:{0.,.7,-2.1})equalGain(a.gain(StateVec(-.2,.1,0,yaw)),b.gain(StateVec(-.2,.1,0,yaw)));
  const auto first=a.plan(StateVec(-1,0,0,0)),second=b.plan(StateVec(-1,0,0,0));
  ASSERT_EQ(first.success,second.success);ASSERT_TRUE(first.success);
  EXPECT_EQ(first.vertices,second.vertices);EXPECT_EQ(first.edges,second.edges);
  EXPECT_DOUBLE_EQ(first.gain,second.gain);ASSERT_EQ(first.path.size(),second.path.size());
  for(std::size_t i=0;i<first.path.size();++i)EXPECT_EQ(first.path[i],second.path[i]);
}

TEST(MultiSensor,RejectsUnsupportedModelsAndUnboundedConfiguration){
  Parameters p;p.sensors={narrow("camera")};p.sensors[0].type="kZoom";
  EXPECT_THROW(p.validate(),std::invalid_argument);
  p.sensors[0].type="kCamera";p.sensors.push_back(p.sensors[0]);
  EXPECT_THROW(p.validate(),std::invalid_argument);
  p.sensors[1].name="other";p.sensors[1].rotations.x()=std::numeric_limits<double>::infinity();
  EXPECT_THROW(p.validate(),std::invalid_argument);
  p.sensors.resize(1);p.sensors[0].horizontal_resolution=.00001;
  EXPECT_THROW(p.validate(),std::invalid_argument);
}

// These tests compare observed map queries as well as gain values. They are
// candidate checks for the opt-in request cache; ordinary queries stay live.
TEST(ExactRayGainReuse,ReusesOnlyBitIdenticalPoseAndKeepsDefaultQueriesLive){
  Parameters p;p.sensors={narrow("cache")};p.gain.ray_method="dda";
  auto map=std::make_shared<RayOrigins>();
  LocalPlanner cached(map,p,{},ExactRayGainReuse::ImmutableSnapshot),ordinary(map,p);
  const StateVec state(.3,-.2,.1,.7);
  const auto first=cached.rayGain(state);const auto queries=map->origins.size();
  ASSERT_GT(queries,0u);
  equalGain(cached.rayGain(state),first);EXPECT_EQ(map->origins.size(),queries);
  EXPECT_EQ(cached.exactRayGainCacheStats().hits,1u);
  StateVec changed=state;changed[3]=std::nextafter(state[3],1.);
  equalGain(cached.rayGain(changed),ordinary.rayGain(changed));
  EXPECT_EQ(cached.exactRayGainCacheStats().evaluations,2u);
  const auto before=map->origins.size();ordinary.rayGain(state);ordinary.rayGain(state);
  EXPECT_EQ(map->origins.size(),before+2*queries);
  EXPECT_EQ(ordinary.exactRayGainCacheStats().entries,0u);
}

TEST(ExactRayGainReuse,ClusterCopiesDoNotSeedExactCacheOrChangeThePlannedGraph){
  Parameters p;p.sensors={narrow("cache")};p.gain.ray_method="dda";
  p.max_vertices=30;p.max_attempts=1500;p.cluster_vertices_for_gain=true;
  p.clustering_radius=100.;p.leaf_gain_only=true;
  auto map=scene();LocalPlanner ordinary(map,p);
  LocalPlanner cached(map,p,{},ExactRayGainReuse::ImmutableSnapshot);
  const StateVec root(-1.,0.,0.,0.);
  const auto baseline=ordinary.plan(root),result=cached.plan(root);
  ASSERT_GT(result.reused_gains,0);ASSERT_GT(result.gain_evaluations,0);
  EXPECT_EQ(result.success,baseline.success);EXPECT_EQ(result.vertices,baseline.vertices);
  EXPECT_EQ(result.edges,baseline.edges);EXPECT_EQ(result.attempts,baseline.attempts);
  EXPECT_DOUBLE_EQ(result.gain,baseline.gain);ASSERT_EQ(result.path.size(),baseline.path.size());
  for(std::size_t i=0;i<result.path.size();++i)EXPECT_EQ(result.path[i],baseline.path[i]);
  EXPECT_EQ(cached.exactRayGainCacheStats().entries,std::size_t(result.gain_evaluations));
  // Query every original pose exactly. Only actual representatives hit the
  // seeded cache; every cluster copy must perform its own precise ray query.
  for(int id=0;id<result.vertices;++id){
    const auto* vertex=cached.graph().vertices_map_.at(id);
    equalGain(cached.rayGain(vertex->state),ordinary.rayGain(vertex->state));
  }
  EXPECT_EQ(cached.exactRayGainCacheStats().hits,std::size_t(result.gain_evaluations));
  EXPECT_EQ(cached.exactRayGainCacheStats().evaluations,std::size_t(result.vertices));
}
