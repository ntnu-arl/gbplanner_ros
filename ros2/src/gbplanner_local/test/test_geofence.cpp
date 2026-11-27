#include <gtest/gtest.h>
#include "gbplanner_local/geofence.hpp"
#include "gbplanner_local/global_memory.hpp"
#include <boost/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace gbplanner2;
namespace upstream_geofence_reference {
namespace bg=boost::geometry;
// Only declarations are adapted. The getPathStatus method below is extracted
// verbatim from the pinned GBPlanner file as an independent geometry reference.
struct Polygon2d {
  using Point2dType=bg::model::d2::point_xy<double>;
  using Polygon2dType=bg::model::polygon<Point2dType>;
  Polygon2dType polygon_;
  bool doIntersect(Polygon2dType poly) const {return bg::intersects(polygon_,poly);}
};
struct GeofenceArea {Polygon2d polygon;};
struct GeofenceManager {
  enum CoordinateStatus {kOK=0,kViolated=1};
  std::vector<GeofenceArea> geofence_list_;
  CoordinateStatus getPathStatus(const Eigen::Vector2d&,const Eigen::Vector2d&,
                                 const Eigen::Vector2d&) const;
};
GeofenceManager::CoordinateStatus GeofenceManager::getPathStatus(
    const Eigen::Vector2d& start, const Eigen::Vector2d& end,
    const Eigen::Vector2d& box_size) const {
  if (geofence_list_.empty()) return CoordinateStatus::kOK;

  // Form a polygon along the path.
  Polygon2d::Polygon2dType path_polygon;

  Eigen::Vector2d p1(start.x() + box_size.x() / 2.0,
                     start.y() + box_size.y() / 2.0);
  Eigen::Vector2d p2(start.x() + box_size.x() / 2.0,
                     start.y() - box_size.y() / 2.0);
  Eigen::Vector2d p3(start.x() - box_size.x() / 2.0,
                     start.y() - box_size.y() / 2.0);
  Eigen::Vector2d p4(start.x() - box_size.x() / 2.0,
                     start.y() + box_size.y() / 2.0);

  Eigen::Vector2d p_dir = end - start;
  Eigen::Vector2d p1_end;
  p1_end = p1 + p_dir;
  Eigen::Vector2d p2_end;
  p2_end = p2 + p_dir;
  Eigen::Vector2d p3_end;
  p3_end = p3 + p_dir;
  Eigen::Vector2d p4_end;
  p4_end = p4 + p_dir;

  // Check the orientation of the line to form the convex hull.
  p_dir.normalize();
  double orientation = std::atan2(p_dir.y(), p_dir.x());
  if ((orientation >= M_PI_2) && (orientation <= M_PI)) {
    // Case 1
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p2.x(), p2.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p3.x(), p3.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p3_end.x(), p3_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p4_end.x(), p4_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p1_end.x(), p1_end.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
  } else if ((orientation >= 0) && (orientation < M_PI_2)) {
    // Case 2
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p2.x(), p2.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p3.x(), p3.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p4.x(), p4.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p4_end.x(), p4_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p1_end.x(), p1_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p2_end.x(), p2_end.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p2.x(), p2.y()));
  } else if ((orientation >= -M_PI_2) && (orientation < 0)) {
    // Case 3
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p1_end.x(), p1_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p2_end.x(), p2_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p3_end.x(), p3_end.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p3.x(), p3.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p4.x(), p4.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
  } else if ((orientation >= -M_PI) && (orientation < -M_PI_2)) {
    // Case 4
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p2.x(), p2.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p2_end.x(), p2_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p3_end.x(), p3_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p4_end.x(), p4_end.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p4.x(), p4.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
  }

  // Compare a rectange and a convex shape.
  CoordinateStatus status = CoordinateStatus::kOK;
  for (auto& p : geofence_list_) {
    if (p.polygon.doIntersect(path_polygon)) {
      status = CoordinateStatus::kViolated;
      break;
    }
  }
  return status;
}
}  // namespace upstream_geofence_reference
namespace {
using V2=Eigen::Vector2d;
using V3=Eigen::Vector3d;
std::vector<V2> rectangle(double xmin,double ymin,double xmax,double ymax){
  return {{xmin,ymin},{xmax,ymin},{xmax,ymax},{xmin,ymax}};
}
std::shared_ptr<VoxelMap> freeMap(){
  auto map=std::make_shared<VoxelMap>(.25,V3(-5,-5,-2),Eigen::Vector3i(40,40,16));
  map->fill(VoxelStatus::Free);return map;
}
VolumetricGain branchGain(const StateVec& state){
  VolumetricGain gain;gain.is_frontier=true;gain.num_unknown_voxels=10;
  gain.gain=state.x()>3?20:10;return gain;
}
planner_msgs::msg::Graph frontierBranches(){
  GraphManager graph;graph.addVertex(new Vertex(0,StateVec::Zero()));
  for(int i=1;i<=4;++i){
    graph.addVertex(new Vertex(i,StateVec(i,0,0,0)));
    graph.addEdge(graph.getVertex(i-1),graph.getVertex(i),1);
    graph.addVertex(new Vertex(i+4,StateVec(0,i,0,0)));
    graph.addEdge(graph.getVertex(i==1?0:i+3),graph.getVertex(i+4),1);
  }
  graph.getVertex(4)->vol_gain=branchGain(graph.getVertex(4)->state);
  graph.getVertex(8)->vol_gain=branchGain(graph.getVertex(8)->state);
  planner_msgs::msg::Graph message;graph.convertGraphToMsg(message);return message;
}
}
TEST(GeofenceGeometry,MatchesOriginalNonzeroPathsInEveryQuadrant){
  const auto points=rectangle(-.5,-.5,.5,.5);
  GeofenceSet fences;ASSERT_TRUE(fences.add(points));
  upstream_geofence_reference::GeofenceManager reference;
  upstream_geofence_reference::GeofenceArea area;
  for(const auto& p:points)boost::geometry::append(area.polygon.polygon_.outer(),
    upstream_geofence_reference::Polygon2d::Point2dType(p.x(),p.y()));
  boost::geometry::correct(area.polygon.polygon_);reference.geofence_list_.push_back(area);
  // Both crossing and offset paths, including axes and reversal. Footprints are
  // anisotropic so an accidental circle or broad AABB cannot pass by symmetry.
  for(const V2 direction:std::vector<V2>{{1,1},{-1,1},{-1,-1},{1,-1},{1,0},{-1,0},{0,1},{0,-1}})
    for(const V2 offset:std::vector<V2>{{0,0},{2,0},{0,2},{2,2}}){
      const V2 start=offset-2*direction,end=offset+2*direction,size(.6,.3);
      const bool expected=reference.getPathStatus(start,end,size)==reference.kViolated;
      EXPECT_EQ(fences.violatesPath(start,end,size),expected);
      EXPECT_EQ(fences.violatesPath(end,start,size),expected);
    }
}
TEST(GeofenceGeometry,BoundaryTouchAndStationaryFootprintAreBlocked){
  GeofenceSet fences;ASSERT_TRUE(fences.add(rectangle(0,0,1,1)));
  EXPECT_TRUE(fences.violatesBox({-.25,.5},{.5,.5}));
  EXPECT_FALSE(fences.violatesBox({-.2501,.5},{.5,.5}));
  EXPECT_TRUE(fences.violatesPath({.5,.5},{.5,.5},{.2,.2}));
  EXPECT_FALSE(fences.violatesPath({2,2},{2,2},{.2,.2}));
  EXPECT_TRUE(fences.violatesPath({-.25,-2},{-.25,2},{.5,.5}));
  EXPECT_FALSE(fences.violatesPath({-.3,-2},{-.3,2},{.5,.5}));
}
TEST(GeofenceGeometry,ConcaveAreasKeepTheOpenNotch){
  GeofenceSet fences;
  ASSERT_TRUE(fences.add({{0,0},{3,0},{3,1},{1,1},{1,3},{0,3}}));
  EXPECT_FALSE(fences.violatesBox({2,2},{.4,.4}));
  EXPECT_FALSE(fences.violatesPath({2,2},{2,4},{.4,.4}));
  EXPECT_TRUE(fences.violatesPath({2,2},{.5,2},{.4,.4}));
  EXPECT_TRUE(fences.violatesBox({.5,2},{.4,.4}));
}
TEST(GeofenceGeometry,WindingAndRepeatedClosingPointsDoNotChangeRegion){
  auto points=rectangle(-1,-1,1,1);GeofenceSet forward,reverse;
  ASSERT_TRUE(forward.add(points));
  std::reverse(points.begin(),points.end());points.push_back(points.front());points.push_back(points.front());
  ASSERT_TRUE(reverse.add(points));
  for(double x:{-2.,-1.,0.,1.,2.})for(double y:{-2.,-1.,0.,1.,2.})
    EXPECT_EQ(forward.violatesBox({x,y},{.2,.3}),reverse.violatesBox({x,y},{.2,.3}));
  auto independent=forward;independent.clear();
  EXPECT_EQ(independent.size(),0u);EXPECT_EQ(forward.size(),1u);
}
TEST(GeofenceGeometry,InvalidAdditionsLeaveExistingAreasUnchanged){
  GeofenceSet fences;std::string error;
  ASSERT_TRUE(fences.add(rectangle(0,0,1,1),&error));EXPECT_TRUE(error.empty());
  const double nan=std::numeric_limits<double>::quiet_NaN();
  for(const auto& points:std::vector<std::vector<V2>>{
        {},{{0,0}},{{0,0},{1,1}},{{0,0},{1,0},{2,0}},
        {{0,0},{1,1},{0,1},{1,0}},{{0,0},{1,0},{nan,1}}}){
    EXPECT_FALSE(fences.add(points,&error));EXPECT_FALSE(error.empty());
    EXPECT_EQ(fences.size(),1u);EXPECT_TRUE(fences.violatesBox({.5,.5},{.2,.2}));
  }
  EXPECT_THROW(fences.violatesBox({nan,0},{.2,.2}),std::invalid_argument);
  EXPECT_THROW(fences.violatesPath({0,0},{1,1},{0,.2}),std::invalid_argument);
}
TEST(GeofenceMap,ColumnConstraintAppliesAtEveryHeightIncludingVerticalMotion){
  auto map=freeMap();auto fences=std::make_shared<GeofenceSet>();
  ASSERT_TRUE(fences->add(rectangle(-.5,-.5,.5,.5)));GeofenceMap constrained(map,fences);
  EXPECT_EQ(constrained.box({0,0,1},{.4,.4,.4}),VoxelStatus::Occupied);
  EXPECT_EQ(constrained.path({0,0,-1},{0,0,1},{.4,.4,.4}),VoxelStatus::Occupied);
  EXPECT_EQ(constrained.path({2,0,-1},{2,0,1},{.4,.4,.4}),VoxelStatus::Free);
  EXPECT_EQ(constrained.resolution(),map->resolution());
  EXPECT_EQ(constrained.minimum(),map->minimum());EXPECT_EQ(constrained.maximum(),map->maximum());
  EXPECT_THROW(constrained.path({0,0,std::numeric_limits<double>::quiet_NaN()},{0,0,0},{.4,.4,.4}),std::invalid_argument);
}
TEST(GeofenceMap,PoliciesDoNotInventSensorReturnsOrEraseUnknownSpace){
  auto map=freeMap();auto fences=std::make_shared<GeofenceSet>();
  ASSERT_TRUE(fences->add(rectangle(-.5,-.5,.5,.5)));GeofenceMap constrained(map,fences);
  EXPECT_EQ(constrained.voxel({0,0,0}),VoxelStatus::Free);
  std::vector<VoxelStatus> original,wrapped;
  map->ray({-2,0,0},{2,0,0},[&](auto,auto status){original.push_back(status);return true;});
  constrained.ray({-2,0,0},{2,0,0},[&](auto,auto status){wrapped.push_back(status);return true;});
  EXPECT_EQ(wrapped,original);EXPECT_GT(wrapped.size(),10u);
  map->set(map->index({2,0,0}),VoxelStatus::Unknown);
  EXPECT_EQ(constrained.box({2,0,0},{.4,.4,.4}),VoxelStatus::Unknown);
  map->set(map->index({2,0,0}),VoxelStatus::Occupied);
  EXPECT_EQ(constrained.box({2,0,0},{.4,.4,.4}),VoxelStatus::Occupied);
}
TEST(GeofenceMap,ClearingFenceRestoresRetainedGlobalEdges){
  auto map=freeMap();auto empty=std::make_shared<GeofenceSet>();
  GlobalMemory memory{Parameters{}};memory.updateMap(std::make_shared<GeofenceMap>(map,empty));
  for(double x:{-2.,-1.,0.,1.,2.})ASSERT_TRUE(memory.recordOdometry(StateVec(x,0,0,0)));
  const auto vertices=memory.graph().vertices_map_.size();const auto edges=memory.edgeCount();
  auto blocked=std::make_shared<GeofenceSet>(*empty);
  ASSERT_TRUE(blocked->add(rectangle(.35,-4,.65,4)));
  memory.updateMap(std::make_shared<GeofenceMap>(map,blocked));
  EXPECT_FALSE(memory.homing(StateVec(2,0,0,0)).success);ASSERT_GT(memory.lastMapEdgeUpdate().pending,0u);
  memory.updateMap(std::make_shared<GeofenceMap>(map,empty));
  const auto result=memory.homing(StateVec(2,0,0,0));ASSERT_TRUE(result.success)<<result.reason;
  EXPECT_NEAR(result.distance,4.,1e-12);EXPECT_EQ(memory.graph().vertices_map_.size(),vertices);
  EXPECT_EQ(memory.edgeCount(),edges);EXPECT_EQ(memory.lastMapEdgeUpdate().pending,0u);
}

TEST(GeofenceMap,ClearedFenceReevaluatesAndRestoresThePreferredFrontier){
  auto map=freeMap();auto empty=std::make_shared<GeofenceSet>();
  GlobalMemory memory{Parameters{}};memory.updateMap(std::make_shared<GeofenceMap>(map,empty));
  ASSERT_TRUE(memory.recordOdometry(StateVec::Zero()));memory.absorbLocal(frontierBranches(),branchGain);
  ASSERT_EQ(memory.frontierCount(),2);
  auto selected=memory.selectFrontier(StateVec::Zero(),branchGain);
  ASSERT_TRUE(selected.success);EXPECT_NEAR(selected.path.back().x(),4.,1e-12);
  const auto vertices=memory.graph().vertices_map_.size();
  auto blocked=std::make_shared<GeofenceSet>();ASSERT_TRUE(blocked->add(rectangle(3.75,-.5,4.25,.5)));
  memory.updateMap(std::make_shared<GeofenceMap>(map,blocked));
  selected=memory.selectFrontier(StateVec::Zero(),branchGain);
  ASSERT_TRUE(selected.success);EXPECT_NEAR(selected.path.back().y(),4.,1e-12);
  EXPECT_EQ(memory.frontierCount(),1);
  // Repeated planning while blocked must retain the paused candidate without
  // counting it as active or accidentally discarding its recovery identity.
  selected=memory.selectFrontier(StateVec::Zero(),branchGain);
  ASSERT_TRUE(selected.success);EXPECT_NEAR(selected.path.back().y(),4.,1e-12);
  memory.updateMap(std::make_shared<GeofenceMap>(map,empty));
  selected=memory.selectFrontier(StateVec::Zero(),branchGain);
  ASSERT_TRUE(selected.success);EXPECT_NEAR(selected.path.back().x(),4.,1e-12);
  EXPECT_NEAR(selected.score,20*std::exp(-.04),1e-12);
  EXPECT_EQ(memory.frontierCount(),2);EXPECT_EQ(memory.graph().vertices_map_.size(),vertices);
}
TEST(GeofenceMap,ClearedFenceDoesNotRestoreAFrontierWhoseGainDisappeared){
  auto map=freeMap();auto empty=std::make_shared<GeofenceSet>();
  GlobalMemory memory{Parameters{}};memory.updateMap(std::make_shared<GeofenceMap>(map,empty));
  ASSERT_TRUE(memory.recordOdometry(StateVec::Zero()));memory.absorbLocal(frontierBranches(),branchGain);
  auto blocked=std::make_shared<GeofenceSet>();ASSERT_TRUE(blocked->add(rectangle(3.75,-.5,4.25,.5)));
  memory.updateMap(std::make_shared<GeofenceMap>(map,blocked));
  ASSERT_TRUE(memory.selectFrontier(StateVec::Zero(),branchGain).success);
  EXPECT_EQ(memory.frontierCount(),1);
  memory.updateMap(std::make_shared<GeofenceMap>(map,empty));
  int reconsidered=0;
  auto observed=[&](const StateVec& state){
    if(state.x()>3){++reconsidered;return VolumetricGain{};}
    return branchGain(state);
  };
  auto selected=memory.selectFrontier(StateVec::Zero(),observed);
  ASSERT_TRUE(selected.success);EXPECT_NEAR(selected.path.back().y(),4.,1e-12);
  EXPECT_EQ(reconsidered,1);EXPECT_EQ(memory.frontierCount(),1);
  // The candidate has now been conclusively observed. Further refreshes must
  // not revive it from the old suspended set or reuse the old high gain.
  selected=memory.selectFrontier(StateVec::Zero(),branchGain);
  ASSERT_TRUE(selected.success);EXPECT_NEAR(selected.path.back().y(),4.,1e-12);
  EXPECT_EQ(memory.frontierCount(),1);
}
