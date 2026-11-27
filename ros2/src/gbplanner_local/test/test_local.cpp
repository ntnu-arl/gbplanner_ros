#include <gtest/gtest.h>
#include "gbplanner_local/local_planner.hpp"
#include <limits>
#include <set>
#include "upstream_reference.hpp"
#include "gbplanner_local/scan_integrator.hpp"

using namespace gbplanner2;
namespace {
std::shared_ptr<VoxelMap> scene(bool unknown=true,bool wall=true) {
  auto map=std::make_shared<VoxelMap>(0.25,Eigen::Vector3d(-5,-5,-1.5),Eigen::Vector3i(40,40,12));
  map->fill(VoxelStatus::Free);
  for(int z=0;z<12;++z)for(int y=0;y<40;++y)for(int x=0;x<40;++x){
    Eigen::Vector3i i(x,y,z);auto c=map->center(i);
    if(unknown && c.x()>=1.5)map->set(i,VoxelStatus::Unknown);
    if(wall && std::abs(c.x())<0.25 && std::abs(c.y())>1.0)map->set(i,VoxelStatus::Occupied);
  }
  return map;
}
Parameters small(){Parameters p;p.max_vertices=50;p.max_attempts=1500;return p;}
}
TEST(VoxelMap,IndexingAndBounds){
  VoxelMap map(0.5,{-1,-1,-1},{4,4,4});
  EXPECT_EQ(map.index({-0.01,0,0}),Eigen::Vector3i(1,2,2));
  EXPECT_EQ(map.voxel({0,0,0}),VoxelStatus::Unknown);
  map.set({2,2,2},VoxelStatus::Free);
  EXPECT_EQ(map.voxel({0.1,0.1,0.1}),VoxelStatus::Free);
  EXPECT_EQ(map.voxel({1,0,0}),VoxelStatus::Unknown);
  EXPECT_THROW(map.set({4,0,0},VoxelStatus::Free),std::invalid_argument);
  EXPECT_THROW(map.set({0,0,0},static_cast<VoxelStatus>(5)),std::invalid_argument);
  EXPECT_THROW((VoxelMap(0,{-1,-1,-1},{4,4,4})),std::invalid_argument);
}
TEST(VoxelMap,SweptBoxDoesNotSkipThinWall){
  auto map=scene(false,true);
  EXPECT_EQ(map->path({-2,2,0},{2,2,0},{0.4,0.4,0.4}),VoxelStatus::Occupied);
  EXPECT_EQ(map->path({-2,0,0},{2,0,0},{0.4,0.4,0.4}),VoxelStatus::Free);
  EXPECT_EQ(map->path({2,2,0},{-2,2,0},{0.4,0.4,0.4}),VoxelStatus::Occupied);
  EXPECT_EQ(map->box({0,0.9,0},{0.4,0.4,0.4}),VoxelStatus::Occupied);
  EXPECT_EQ(map->box({4.9,0,0},{0.4,0.4,0.4}),VoxelStatus::Unknown);
}
TEST(VoxelMap,UnknownSpaceBlocksMotion){
  auto map=scene();
  EXPECT_EQ(map->path({0,0,0},{2,0,0},{0.4,0.4,0.4}),VoxelStatus::Unknown);
  EXPECT_EQ(map->box({1.5,0,0},{0.4,0.4,0.4}),VoxelStatus::Unknown);
}
TEST(VoxelMap,RayVisitsCellsAndStopsOnObstacle){
  VoxelMap map(1,{0,0,0},{6,2,2});map.fill(VoxelStatus::Free);map.set({3,0,0},VoxelStatus::Occupied);
  std::vector<int> visited;
  map.ray({0.5,0.5,0.5},{5.5,0.5,0.5},[&](auto p,auto s){visited.push_back(map.index(p).x());return s!=VoxelStatus::Occupied;});
  EXPECT_EQ(visited,(std::vector<int>{0,1,2,3}));
  visited.clear();map.ray({5.5,0.5,0.5},{0.5,0.5,0.5},[&](auto p,auto){visited.push_back(map.index(p).x());return true;});
  EXPECT_EQ(visited,(std::vector<int>{5,4,3,2,1,0}));
}
TEST(ScanIntegrator,CarvesFreeRayAndPreservesUnknownBehindHit){
  VoxelMap map(1,{0,0,0},{8,3,3});ScanIntegrator scanner(0.1,6);
  auto result=scanner.integrate(map,{0.5,1.5,1.5},{{3.5,1.5,1.5}});
  EXPECT_EQ(result.accepted_points,1u);EXPECT_EQ(result.free_updates,3u);EXPECT_EQ(result.occupied_updates,1u);
  for(int x=0;x<3;++x)EXPECT_EQ(map.voxel({x+0.5,1.5,1.5}),VoxelStatus::Free);
  EXPECT_EQ(map.voxel({3.5,1.5,1.5}),VoxelStatus::Occupied);
  EXPECT_EQ(map.voxel({4.5,1.5,1.5}),VoxelStatus::Unknown);
  EXPECT_EQ(map.voxel({1.5,0.5,1.5}),VoxelStatus::Unknown);
}
TEST(ScanIntegrator,HitWinsAndPointOrderDoesNotMatter){
  VoxelMap a(1,{0,0,0},{8,3,3}),b(1,{0,0,0},{8,3,3});ScanIntegrator scanner(0.1,6);
  std::vector<Eigen::Vector3d> points{{3.5,1.5,1.5},{5.5,1.5,1.5}};
  scanner.integrate(a,{0.5,1.5,1.5},points);std::reverse(points.begin(),points.end());scanner.integrate(b,{0.5,1.5,1.5},points);
  EXPECT_EQ(a.voxel({3.5,1.5,1.5}),VoxelStatus::Occupied);
  for(int x=0;x<8;++x)EXPECT_EQ(a.voxel({x+0.5,1.5,1.5}),b.voxel({x+0.5,1.5,1.5}));
  // A later scan may clear an old occupied cell; no confidence model yet.
  scanner.integrate(a,{0.5,1.5,1.5},{{5.5,1.5,1.5}});
  EXPECT_EQ(a.voxel({3.5,1.5,1.5}),VoxelStatus::Free);
}
TEST(ScanIntegrator,RangeTruncationAndInvalidReturns){
  VoxelMap map(1,{0,0,0},{8,3,3});ScanIntegrator scanner(0.1,3);
  auto result=scanner.integrate(map,{0.5,1.5,1.5},{{7.5,1.5,1.5},{0.5,1.5,1.5},
    {std::numeric_limits<double>::quiet_NaN(),0,0}});
  EXPECT_EQ(result.accepted_points,1u);EXPECT_EQ(result.skipped_points,2u);EXPECT_EQ(result.occupied_updates,0u);
  EXPECT_EQ(map.voxel({3.5,1.5,1.5}),VoxelStatus::Free);
  EXPECT_EQ(map.voxel({4.5,1.5,1.5}),VoxelStatus::Unknown);
  EXPECT_THROW(scanner.integrate(map,{-1,0,0},{}),std::invalid_argument);
  EXPECT_THROW((ScanIntegrator(2,1)),std::invalid_argument);
}
TEST(Sampler,RepeatableUniformAndHeavyTail){
  for(const auto& name:{"uniform","normal","cauchy"}) {
    auto p=small();p.distribution=name;LocalSampler a(p),b(p);
    StateVec root(3,4,5,0.3);
    for(int i=0;i<100;++i){auto x=a.generate(root),y=b.generate(root);EXPECT_EQ(x,y);EXPECT_TRUE(x.allFinite());
      if(p.distribution=="uniform"){EXPECT_TRUE(((x-root).head<3>().array()>=p.local_min.array()).all());EXPECT_TRUE(((x-root).head<3>().array()<=p.local_max.array()).all());}
    }
  }
}
TEST(Parameters,RejectInvalidInputs){
  auto p=small();p.edge_max=0;EXPECT_THROW(p.validate(),std::invalid_argument);
  p=small();p.local_max=p.local_min;EXPECT_THROW(p.validate(),std::invalid_argument);
  p=small();p.path_length_penalty=std::numeric_limits<double>::quiet_NaN();EXPECT_THROW(p.validate(),std::invalid_argument);
  p=small();p.horizontal_resolution=1e-10;EXPECT_THROW(p.validate(),std::invalid_argument);
}
TEST(Gain,AllKnownHasNoUnknownGain){
  auto map=scene(false,false);LocalPlanner planner(map,small());
  auto gain=planner.gain(StateVec(0,0,0,0));EXPECT_DOUBLE_EQ(gain.gain,0);EXPECT_EQ(gain.num_unknown_voxels,0);
}
TEST(Gain,WallOccludesUnknownVoxels){
  auto map=scene(true,false);auto p=small();p.horizontal_fov=0.1;p.vertical_fov=0.1;
  p.horizontal_resolution=0.1;p.vertical_resolution=0.1;
  LocalPlanner visible(map,p);EXPECT_GT(visible.gain(StateVec(0,0,0,0)).num_unknown_voxels,0);
  for(int y=0;y<40;++y)for(int z=0;z<12;++z)map->set({22,y,z},VoxelStatus::Occupied);
  LocalPlanner hidden(map,p);auto gain=hidden.gain(StateVec(0,0,0,0));
  EXPECT_EQ(gain.num_unknown_voxels,0);EXPECT_GT(gain.num_occupied_voxels,0);
}
TEST(LocalPlanner,RejectRootAndNoGain){
  LocalPlanner blocked(scene(),small());
  for(const auto& root:{StateVec(0,2,0,0),StateVec(3,0,0,0)}){
    const auto result=blocked.plan(root);EXPECT_EQ(result.reason,"root_not_free");
    EXPECT_FALSE(result.graph_build_completed);
  }
  const auto invalid=blocked.plan(StateVec(std::numeric_limits<double>::quiet_NaN(),0,0,0));
  EXPECT_EQ(invalid.reason,"invalid_root");EXPECT_FALSE(invalid.graph_build_completed);
  const auto direction=blocked.plan(StateVec(-2,0,0,0),std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(direction.reason,"invalid_exploring_direction");EXPECT_FALSE(direction.graph_build_completed);
  LocalPlanner known(scene(false,false),small());auto result=known.plan(StateVec(-2,0,0,0));
  EXPECT_FALSE(result.success);EXPECT_EQ(result.reason,"no_gain");EXPECT_TRUE(result.path.empty());
  EXPECT_TRUE(result.graph_build_completed);
}
TEST(LocalPlanner,RepeatablePlanAndIndependentPathChecks){
  auto map=scene();LocalPlanner planner(map,small());StateVec root(-2,0,0,0);
  auto a=planner.plan(root);ASSERT_TRUE(a.success)<<a.reason;ASSERT_GE(a.path.size(),2u);
  auto b=planner.plan(root);ASSERT_TRUE(b.success);ASSERT_EQ(a.path.size(),b.path.size());
  EXPECT_TRUE(a.graph_build_completed);EXPECT_TRUE(b.graph_build_completed);
  EXPECT_DOUBLE_EQ(a.gain,b.gain);EXPECT_EQ(a.vertices,b.vertices);EXPECT_EQ(a.edges,b.edges);
  double length=0;
  for(std::size_t i=0;i<a.path.size();++i){
    EXPECT_EQ(a.path[i],b.path[i]);
    if(!i)continue;
    Eigen::Vector3d delta=a.path[i].head<3>()-a.path[i-1].head<3>();length+=delta.norm();
    // Independent dense footprint probe (the implementation uses segment/AABB intersection).
    for(double t=0;t<=1;t+=0.01){
      Eigen::Vector3d c=a.path[i-1].head<3>()+t*delta;
      for(double dx:{-0.2,0.0,0.2})for(double dy:{-0.2,0.0,0.2})for(double dz:{-0.2,0.0,0.2})
        EXPECT_EQ(map->voxel(c+Eigen::Vector3d(dx,dy,dz)),VoxelStatus::Free);
    }
  }
  EXPECT_NEAR(length,a.path_length,1e-9);EXPECT_GT(a.gain,0);EXPECT_LE(a.vertices,small().max_vertices);
}
TEST(LocalPlanner,DiscountedGainMatchesPathSum){
  auto p=small();LocalPlanner planner(scene(),p);auto result=planner.plan(StateVec(-2,0,0,0));
  ASSERT_TRUE(result.success);double score=0,distance=0;
  for(std::size_t i=0;i<result.path.size();++i){
    if(i)distance+=(result.path[i]-result.path[i-1]).head<3>().norm();
    score+=planner.gain(result.path[i]).gain*std::exp(-p.path_length_penalty*distance);
  }
  EXPECT_NEAR(score,result.gain,1e-8);
}
TEST(LocalPlanner,SeveralSeedsAndTreeAblation){
  for(uint32_t seed=0;seed<5;++seed)for(bool graph:{false,true}){
    auto p=small();p.seed=seed;p.graph_mode=graph;
    auto map=scene();LocalPlanner planner(map,p);auto result=planner.plan(StateVec(-2,0,0,0));
    ASSERT_TRUE(result.success)<<seed<<" graph="<<graph<<" "<<result.reason;
    if(!graph)EXPECT_EQ(result.edges,result.vertices-1);
    else EXPECT_GE(result.edges,result.vertices-1);
    for(const auto& entry:planner.graph().edge_map_)for(const auto& edge:entry.second){
      const auto* a=planner.graph().vertices_map_.at(entry.first);
      const auto* b=planner.graph().vertices_map_.at(edge.first);
      EXPECT_EQ(map->path(a->state.head<3>(),b->state.head<3>(),p.robot_size),VoxelStatus::Free);
    }
  }
}
TEST(LocalPlanner,MatchesExtractedUpstreamAerialExpansion){
  // Replay precisely the same accepted samples to the extracted original
  // expandGraph; compare every vertex and undirected weighted edge.
  for(uint32_t seed=0;seed<10;++seed)for(bool graph:{false,true}) {
    auto p=small();p.seed=seed;p.graph_mode=graph;p.max_edges=100000;
    auto map=scene();StateVec root(-2,0,0,0);
    LocalPlanner port(map,p);auto result=port.plan(root);
    auto reference=std::make_shared<GraphManager>();reference->addVertex(new Vertex(0,root));
    upstream_reference::ReferenceRrg upstream(map,p);LocalSampler sampler(p);
    for(int attempt=0;attempt<p.max_attempts&&reference->getNumVertices()<p.max_vertices;++attempt){
      auto state=sampler.generate(root);Eigen::Vector3d c=state.head<3>()+p.center_offset;
      if((c.array()-0.5*p.robot_size.array()<=map->minimum().array()).any()||
         (c.array()+0.5*p.robot_size.array()>=map->maximum().array()).any())continue;
      if(map->box(c,p.robot_size)!=VoxelStatus::Free)continue;
      ExpandGraphReport rep;upstream.expandGraph(reference,state,rep);
    }
    ASSERT_EQ(reference->getNumVertices(),result.vertices)<<seed<<" graph="<<graph;
    ASSERT_EQ(reference->getNumEdges(),result.edges)<<seed<<" graph="<<graph;
    for(int id=0;id<result.vertices;++id){
      auto* expected=reference->getVertex(id);auto* actual=port.graph().vertices_map_.at(id);
      EXPECT_LT((expected->state-actual->state).head<3>().norm(),1e-12);
      auto a=reference->edge_map_[id];auto b=port.graph().edge_map_.at(id);
      std::sort(a.begin(),a.end());std::sort(b.begin(),b.end());
      ASSERT_EQ(a.size(),b.size());
      for(std::size_t k=0;k<a.size();++k){EXPECT_EQ(a[k].first,b[k].first);EXPECT_NEAR(a[k].second,b[k].second,1e-12);}
    }
  }
}


namespace {
// Independent elementary rotations, applying upstream
// (Rz(yaw)*Ry(pitch)*Rx(roll))^-1 without the implementation helper.
Eigen::Vector3d upstreamSampleRotation(Eigen::Vector3d value,const Eigen::Vector3d& rotations){
  double c=std::cos(rotations.x()),s=std::sin(rotations.x());
  value={c*value.x()+s*value.y(),-s*value.x()+c*value.y(),value.z()};
  c=std::cos(rotations.y());s=std::sin(rotations.y());
  value={c*value.x()-s*value.z(),value.y(),s*value.x()+c*value.z()};
  c=std::cos(rotations.z());s=std::sin(rotations.z());
  return {value.x(),c*value.y()+s*value.z(),-s*value.y()+c*value.z()};
}
}
TEST(RotatedSampling,MatchesUpstreamYprOrderForAllDistributions){
  const StateVec root(2.,-3.,.4,1.1);
  for(const auto* distribution:{"uniform","normal","cauchy"}){
    Parameters plain;plain.distribution=distribution;
    plain.local_min={-.8,-.3,-.2};plain.local_max={2.1,.6,.4};
    Parameters rotated=plain;rotated.local_rotations={.7,-.35,.2};
    LocalSampler reference(plain),port(rotated);
    for(int i=0;i<100;++i){
      const auto unrotated=reference.generate(root),actual=port.generate(root);
      const Eigen::Vector3d expected=root.head<3>()+upstreamSampleRotation((unrotated-root).head<3>(),rotated.local_rotations);
      EXPECT_NEAR((actual.head<3>()-expected).norm(),0.,1e-11)<<distribution<<" sample "<<i;
      EXPECT_DOUBLE_EQ(actual[3],unrotated[3]);
      const Eigen::Vector3d recovered=rotated.worldToSampling()*(actual-root).head<3>();
      EXPECT_NEAR((recovered-(unrotated-root).head<3>()).norm(),0.,1e-11);
    }
  }
}
TEST(RotatedSampling,FixedRotationDoesNotFollowRootHeading){
  Parameters p;p.local_rotations={M_PI/2,0,0};
  p.local_min={1.,-.1,-.1};p.local_max={2.,.1,.1};
  LocalSampler a(p),b(p);
  for(int i=0;i<40;++i){
    const auto first=a.generate(StateVec(0,0,0,0));
    const auto turned=b.generate(StateVec(0,0,0,1.7));
    EXPECT_LT(std::abs(first.x()),.1+1e-12);
    EXPECT_GE(first.y(),-2.-1e-12);EXPECT_LE(first.y(),-1.+1e-12);
    EXPECT_NEAR((first-turned).head<3>().norm(),0.,1e-12);
    EXPECT_NEAR(turned[3]-first[3],1.7,1e-12);
  }
}
TEST(RotatedSampling,ZeroRotationPreservesLegacyDrawsExactly){
  Parameters p;LocalSampler sampler(p);std::mt19937 generator(p.seed);
  const StateVec root(2.,-1.,.4,.2);
  for(int i=0;i<100;++i){
    StateVec expected=root;
    for(int axis=0;axis<3;++axis)
      expected[axis]+=std::uniform_real_distribution<double>(p.local_min[axis],p.local_max[axis])(generator);
    expected[3]+=std::uniform_real_distribution<double>(-M_PI,M_PI)(generator);
    EXPECT_EQ(sampler.generate(root),expected);
  }
}
TEST(RotatedSampling,LocalGraphUsesRotatedCuboidWithoutWorldAxisCropping){
  auto p=small();p.local_min={-2.,-.2,-.2};p.local_max={2.,.2,.2};
  p.local_rotations={-M_PI/2,0,0};p.free_gain=1.;
  const StateVec root(0,0,0,.8);LocalPlanner planner(scene(false,false),p);
  const auto result=planner.plan(root);ASSERT_TRUE(result.success)<<result.reason;
  ASSERT_EQ(result.vertices,p.max_vertices);
  double min_y=0,max_y=0;
  for(const auto& entry:planner.graph().vertices_map_){
    const Eigen::Vector3d world=entry.second->state.head<3>()-root.head<3>();
    const Eigen::Vector3d local=p.worldToSampling()*world;
    EXPECT_TRUE((local.array()>=p.local_min.array()-1e-12).all());
    EXPECT_TRUE((local.array()<=p.local_max.array()+1e-12).all());
    EXPECT_LE(std::abs(world.x()),.2+1e-12);
    min_y=std::min(min_y,world.y());max_y=std::max(max_y,world.y());
  }
  EXPECT_LT(min_y,-1.);EXPECT_GT(max_y,1.);
}
TEST(RotatedSampling,RejectsNonfiniteRotation){
  Parameters p;p.local_rotations[1]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(p.validate(),std::invalid_argument);
  EXPECT_THROW((LocalSampler(p)),std::invalid_argument);
}
