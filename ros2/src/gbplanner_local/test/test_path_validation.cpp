#include <gtest/gtest.h>
#include "gbplanner_local/path_validation.hpp"
#include "gbplanner_local/geofence.hpp"
#include "gbplanner_local/global_bound.hpp"
#include <chrono>
#include <condition_variable>
#include <future>
#include <limits>

using namespace gbplanner2;
namespace {
using V3=Eigen::Vector3d;
using Request=planner_msgs::srv::ValidatePath::Request;
using Response=planner_msgs::srv::ValidatePath::Response;
std::shared_ptr<VoxelMap> freeMap(){
  auto map=std::make_shared<VoxelMap>(.1,V3(-2,-2,-1),Eigen::Vector3i(40,40,30));
  map->fill(VoxelStatus::Free);return map;
}
geometry_msgs::msg::Pose pose(double x,double y=0.,double z=0.){
  geometry_msgs::msg::Pose p;p.position.x=x;p.position.y=y;p.position.z=z;p.orientation.w=1.;return p;
}
PathValidationPolicy policy(){PathValidationPolicy p;p.frame="map";p.resolution=.1;p.robot_bounds.physical_size={.2,.2,.2};return p;}
Request request(){Request r;r.header.frame_id="map";r.path={pose(-1.),pose(-.5),pose(1.)};return r;}
void observe(PathValidationInputs& inputs,double x=0.){inputs.publishObservation({StateVec(x,0,0,0),true});}

// The rendezvous is deliberate: map and odometry writers must remain usable
// while validation is inside a collision query, with no timing assumptions.
class PausedMap final:public MapQuery {
 public:
  explicit PausedMap(std::shared_ptr<const MapQuery> map):map_(std::move(map)){}
  double resolution()const override{return map_->resolution();}
  V3 minimum()const override{return map_->minimum();}
  V3 maximum()const override{return map_->maximum();}
  VoxelStatus voxel(const V3& p)const override{return map_->voxel(p);}
  VoxelStatus box(const V3& p,const V3& s)const override{return map_->box(p,s);}
  VoxelStatus path(const V3& a,const V3& b,const V3& s)const override{
    {std::unique_lock<std::mutex> lock(mutex_);entered_=true;condition_.notify_all();condition_.wait(lock,[this]{return released_;});}
    return map_->path(a,b,s);
  }
  void ray(const V3& a,const V3& b,const std::function<bool(const V3&,VoxelStatus)>& f)const override{map_->ray(a,b,f);}
  bool awaitQuery()const{std::unique_lock<std::mutex> lock(mutex_);return condition_.wait_for(lock,std::chrono::seconds(2),[this]{return entered_;});}
  void release()const{std::lock_guard<std::mutex> lock(mutex_);released_=true;condition_.notify_all();}
 private:
  std::shared_ptr<const MapQuery> map_;
  mutable std::mutex mutex_;mutable std::condition_variable condition_;
  mutable bool entered_{false},released_{false};
};

// Test backend changes the accepted snapshot during each query, reproducing
// sustained observation traffic deterministically without sleeping threads.
class UpdatingMap final:public MapQuery {
 public:
  explicit UpdatingMap(std::shared_ptr<const MapQuery> map):map_(std::move(map)){}
  std::function<void()> update;
  double resolution()const override{return map_->resolution();}
  V3 minimum()const override{return map_->minimum();}
  V3 maximum()const override{return map_->maximum();}
  VoxelStatus voxel(const V3& p)const override{return map_->voxel(p);}
  VoxelStatus box(const V3& p,const V3& s)const override{return map_->box(p,s);}
  VoxelStatus path(const V3& a,const V3& b,const V3& s)const override{update();return map_->path(a,b,s);}
  void ray(const V3& a,const V3& b,const std::function<bool(const V3&,VoxelStatus)>& f)const override{map_->ray(a,b,f);}
 private:std::shared_ptr<const MapQuery> map_;
};
}

TEST(PathValidation,RetainsMarginsModesPoliciesAndInputReasons){
  const auto p=policy();PathValidationInputs inputs;observe(inputs);auto map=freeMap();
  // Only the second commanded segment is near this obstacle. The current
  // robot and the first connector remain free with the requested extra margin.
  map->set(map->index({.55,.25,.05}),VoxelStatus::Occupied);inputs.publishMap(map,17);
  auto r=request();r.clearance_margin=.15;r.clearance_segments=1;
  auto result=validateCurrentPath(p,inputs,r);EXPECT_TRUE(result.valid)<<result.reason;EXPECT_EQ(result.map_revision,17u);
  r.clearance_segments=0;result=validateCurrentPath(p,inputs,r);EXPECT_FALSE(result.valid);EXPECT_EQ(result.reason,"path_not_free");EXPECT_EQ(result.failure_segment,1);
  r.clearance_margin=0.;r.bound_mode=RobotBounds::Point;EXPECT_TRUE(validateCurrentPath(p,inputs,r).valid);
  r.header.frame_id="odom";EXPECT_EQ(validateCurrentPath(p,inputs,r).reason,"wrong_frame");r.header.frame_id="map";
  r.path.front().orientation.w=0.;EXPECT_EQ(validateCurrentPath(p,inputs,r).reason,"invalid_pose");r=request();
  r.clearance_margin=std::numeric_limits<double>::quiet_NaN();EXPECT_EQ(validateCurrentPath(p,inputs,r).reason,"invalid_clearance_margin");
  r=request();r.bound_mode=99;EXPECT_EQ(validateCurrentPath(p,inputs,r).reason,"unsupported_bound_mode");
  r=request();auto fences=std::make_shared<GeofenceSet>();ASSERT_TRUE(fences->add({{.4,-.2},{.6,-.2},{.6,.2},{.4,.2}}));
  inputs.publishMap(std::make_shared<GeofenceMap>(freeMap(),fences),18);
  result=validateCurrentPath(p,inputs,r);EXPECT_FALSE(result.valid);EXPECT_EQ(result.reason,"path_not_free");EXPECT_EQ(result.map_revision,18u);
  inputs.publishMap(std::make_shared<BoundedMap>(freeMap(),GlobalBound::axisAligned({-2,-2,-1},{.8,2,2})),19);
  EXPECT_EQ(validateCurrentPath(p,inputs,r).reason,"path_not_free");
}

TEST(PathValidation,InFlightValidationRechecksNewMapAndLatestObservationWithoutBlockingWriters){
  const auto p=policy();PathValidationInputs inputs;observe(inputs);
  const auto paused=std::make_shared<PausedMap>(freeMap());inputs.publishMap(paused,20);
  const auto old=inputs.mapSnapshot();
  PathValidationInputs::Snapshot validated;PathValidationObservation seen;
  auto pending=std::async(std::launch::async,[&]{return validateCurrentPath(p,inputs,request(),&validated,&seen);});
  const bool entered=paused->awaitQuery();
  auto replacement=freeMap();replacement->set(replacement->index({.55,.05,.05}),VoxelStatus::Occupied);
  auto cloud=std::make_shared<sensor_msgs::msg::PointCloud2>();cloud->data={1,2,3};
  inputs.publishMap(replacement,21,cloud);observe(inputs,1.5);paused->release();
  ASSERT_TRUE(entered);const auto result=pending.get();
  EXPECT_FALSE(result.valid);EXPECT_EQ(result.reason,"path_not_free");EXPECT_EQ(result.map_revision,21u);
  ASSERT_TRUE(validated);EXPECT_EQ(validated->revision,21);EXPECT_EQ(validated->source_cloud.get(),cloud.get());
  EXPECT_EQ(result.failure_segment,1);EXPECT_EQ(seen.state.x(),1.5);
  EXPECT_EQ(old->revision,20);EXPECT_EQ(old->map.get(),paused.get());
  // Validation also uses the independent latest pose rather than a planner's
  // retained root. An accepted empty map/reset cannot retain old free geometry.
  inputs.publishObservation({StateVec(9,0,0,0),true});EXPECT_EQ(validateCurrentPath(p,inputs,request()).reason,"current_pose_not_free");
  observe(inputs);auto empty=std::make_shared<VoxelMap>(.1,V3(-2,-2,-1),Eigen::Vector3i(40,40,30));inputs.publishMap(empty,22);
  const auto reset=validateCurrentPath(p,inputs,request());EXPECT_FALSE(reset.valid);EXPECT_EQ(reset.map_revision,22u);
}

TEST(PathValidation,RepeatedConcurrentUpdatesAreBoundedAndNeverReturnSupersededSuccess){
  const auto p=policy();PathValidationInputs inputs;observe(inputs);
  const auto changing=std::make_shared<UpdatingMap>(freeMap());int revision=30;
  changing->update=[&]{inputs.publishMap(changing,++revision);};inputs.publishMap(changing,revision);
  const auto result=validateCurrentPath(p,inputs,request());
  EXPECT_FALSE(result.valid);EXPECT_EQ(result.reason,"map_changed_during_validation");
  // Three complete attempts, with all three segments checked on each attempt.
  EXPECT_EQ(revision,39);EXPECT_EQ(result.map_revision,36u);
}

TEST(PathValidation,GroundBoundaryAndObservedConnectorKeepTerrainSemantics){
  auto map=std::make_shared<VoxelMap>(.05,V3(-2,-1,-1),Eigen::Vector3i(80,40,60));
  for(int z=0;z<60;++z)for(int y=0;y<40;++y)for(int x=0;x<80;++x){
    const Eigen::Vector3i index(x,y,z);const auto center=map->center(index);
    map->set(index,center.z()<0.?VoxelStatus::Occupied:VoxelStatus::Free);map->setTsdf(index,center.z(),1.);
  }
  for(const auto mode:{GroundTsdfProjectionMode::UpstreamStep,GroundTsdfProjectionMode::ZeroCrossing,GroundTsdfProjectionMode::ZeroCrossingSimplex}){
    auto p=policy();p.ground_robot=true;p.resolution=.05;p.robot_bounds.physical_size={.3,.3,.3};
    p.ground.robot_size=p.robot_bounds.physical_size;p.ground.max_ground_height=.8;p.ground.robot_height=.6;
    p.ground.interpolate_projection_distance=true;p.ground.tsdf_projection_mode=mode;
    PathValidationInputs inputs;inputs.publishMap(map,50);const PathValidationContext context(p,inputs.mapSnapshot());
    const auto projected=TerrainProjector(map,p.ground).project(StateVec(-.5,0,1.5,0));ASSERT_TRUE(projected.support);
    auto start=projected.projected_state;auto end=start;end.x()=.5;
    const auto base=context.baseState(start);inputs.publishObservation({start,true});
    Request r;r.header.frame_id="map";r.path={pose(start.x(),0,base.z()),pose(end.x(),0,base.z())};
    const auto valid=validateCurrentPath(p,inputs,r);EXPECT_TRUE(valid.valid)<<groundTsdfProjectionModeName(mode)<<":"<<valid.reason;
    StateVec round_trip;ASSERT_TRUE(context.poseState(r.path.front(),round_trip));EXPECT_NEAR((round_trip-start).norm(),0.,1e-12);
    r.path.front().position.z+=.2;r.path.back().position.z+=.2;
    const auto off=validateCurrentPath(p,inputs,r);EXPECT_FALSE(off.valid)<<groundTsdfProjectionModeName(mode);EXPECT_EQ(off.reason,"path_off_terrain");
    // A short observed connector may normalize its sub-voxel height error.
    // Ordinary commanded waypoints retain the original directional slope check.
    auto observed=start;observed.z()+=.03;auto route=start;route.x()+=.02;
    EXPECT_EQ(context.validateSegment(observed,route,p.ground.robot_size),"terrain_slope_exceeded");
    EXPECT_TRUE(context.validateObservedGroundConnection(observed,route,p.ground.robot_size).empty())<<groundTsdfProjectionModeName(mode);
  }
}
