#include <gtest/gtest.h>
#include "gbplanner_voxblox/tsdf_backend.hpp"
#include <limits>
using namespace gbplanner2;
namespace {
TsdfConfig config(){TsdfConfig c;c.resolution=.2;c.truncation_distance=.6;c.origin={-4.,-4.,-2.};c.dimensions={40,40,20};c.method="simple";return c;}
std::vector<Eigen::Vector3d> wall(){std::vector<Eigen::Vector3d> points;
 for(int y=-15;y<=15;++y)for(int z=-10;z<=10;++z)points.emplace_back(2.,y*.05,z*.05);return points;}
}
TEST(Tsdf, OriginalClassificationBoundary){
 voxblox::TsdfVoxel v;v.weight=1.;v.distance=.2;
 EXPECT_EQ(TsdfBackend::classify(&v,.2,1.),VoxelStatus::Occupied);
 v.distance=.2001;EXPECT_EQ(TsdfBackend::classify(&v,.2,1.),VoxelStatus::Free);
 v.distance=-.5;EXPECT_EQ(TsdfBackend::classify(&v,.2,1.),VoxelStatus::Occupied);
 v.weight=1e-7;EXPECT_EQ(TsdfBackend::classify(&v,.2,1.),VoxelStatus::Unknown);
 EXPECT_EQ(TsdfBackend::classify(nullptr,.2,1.),VoxelStatus::Unknown);
 v.weight=1.;v.distance=std::numeric_limits<float>::quiet_NaN();EXPECT_EQ(TsdfBackend::classify(&v,.2,1.),VoxelStatus::Unknown);
}
TEST(Tsdf, WallSignedDistanceAndUnknownBehind){
 TsdfBackend map(config());map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),wall());
 auto near=map.voxel({1.9,.1,.1});ASSERT_NE(near,nullptr);EXPECT_GT(near->weight,0.);EXPECT_NEAR(near->distance,.1,.025);
 EXPECT_EQ(map.status({.9,.1,.1}),VoxelStatus::Free);
 EXPECT_EQ(map.status({1.9,.1,.1}),VoxelStatus::Occupied);
 EXPECT_EQ(map.status({3.1,.1,.1}),VoxelStatus::Unknown);
 auto behind=map.voxel({2.1,.1,.1});ASSERT_NE(behind,nullptr);EXPECT_LT(behind->distance,0.);
 EXPECT_EQ(map.snapshot()->voxel({1.9,.1,.1}),VoxelStatus::Occupied);
}
TEST(Tsdf, TransformRotatesAndTranslatesMeasurement){
 TsdfBackend map(config());Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ()));
 map.integrate({1.,0.,0.},q,wall());
 EXPECT_EQ(map.status({.9,1.9,.1}),VoxelStatus::Occupied);
 EXPECT_EQ(map.status({.9,.9,.1}),VoxelStatus::Free);
 EXPECT_EQ(map.status({2.9,.1,.1}),VoxelStatus::Unknown);
}
TEST(Tsdf, RepeatedMeasurementsAccumulateBoundedWeight){
 auto c=config();c.max_weight=2.;TsdfBackend map(c);
 map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),{{2.,.1,.1}});
 auto first=map.voxel({1.9,.1,.1});ASSERT_NE(first,nullptr);double w=first->weight;
 map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),{{2.,.1,.1}});
 EXPECT_GT(map.voxel({1.9,.1,.1})->weight,w);
 for(int i=0;i<5;++i)map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),{{2.,.1,.1}});
 EXPECT_LE(map.voxel({1.9,.1,.1})->weight,2.);
}
TEST(Tsdf, OutOfRangeReturnClearsOnlyToRange){
 auto c=config();c.max_range=2.;TsdfBackend map(c);
 map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),{{4.,.1,.1}});
 EXPECT_EQ(map.status({.9,.1,.1}),VoxelStatus::Free);
 EXPECT_EQ(map.status({3.1,.1,.1}),VoxelStatus::Unknown);
}
TEST(Tsdf, ResetRecreatesFastIntegratorState){
 auto c=config();c.method="fast";TsdfBackend map(c);map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),wall());
 auto before=map.snapshot();ASSERT_GT(map.allocatedBlocks(),0u);map.reset();EXPECT_EQ(map.allocatedBlocks(),0u);
 EXPECT_EQ(map.status({1.9,.1,.1}),VoxelStatus::Unknown);
 map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),wall());auto after=map.snapshot();
 for(int z=0;z<20;++z)for(int y=0;y<40;++y)for(int x=0;x<40;++x)
   EXPECT_EQ(before->voxel(before->center({x,y,z})),after->voxel(after->center({x,y,z})));
}
TEST(Tsdf, RejectInvalidShapeParametersAndTransform){
 auto c=config();c.origin.x()=.1;EXPECT_THROW(TsdfBackend m(c),std::invalid_argument);
 c=config();c.method="invented";EXPECT_THROW(TsdfBackend m(c),std::invalid_argument);
 c=config();c.voxels_per_side=7;EXPECT_THROW(TsdfBackend m(c),std::invalid_argument);
 c=config();c.max_range=0.;EXPECT_THROW(TsdfBackend m(c),std::invalid_argument);
 TsdfBackend map(config());EXPECT_THROW(map.integrate({0.,0.,0.},Eigen::Quaterniond(0.,0.,0.,0.),wall()),std::invalid_argument);
 EXPECT_EQ(map.allocatedBlocks(),0u);
}
TEST(Tsdf, InvalidPointsSkippedWithoutAllocatingNaNVoxels){
 TsdfBackend map(config());double nan=std::numeric_limits<double>::quiet_NaN();
 auto result=map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),{{2.,.1,.1},{nan,0.,0.},{.001,0.,0.}});
 EXPECT_EQ(result.accepted_points,1u);EXPECT_EQ(result.skipped_points,2u);
 EXPECT_THROW(map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),{{nan,0.,0.}}),std::invalid_argument);
}
TEST(Tsdf, FreeSpaceEndpointsAndUnknownOnlyBox){
 auto c=config();c.esdf_enabled=true;
 TsdfBackend clearing(c);
 const auto result=clearing.integrateFreeSpace({0.,0.,0.},Eigen::Quaterniond::Identity(),wall());
 EXPECT_EQ(result.accepted_points,wall().size());
 EXPECT_EQ(clearing.status({.9,.1,.1}),VoxelStatus::Free);
 EXPECT_EQ(clearing.status({1.9,.1,.1}),VoxelStatus::Unknown); // No endpoint obstacle.

 TsdfBackend observed(c);observed.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),wall());
 const auto* obstacle=observed.voxel({1.9,.1,.1});ASSERT_NE(obstacle,nullptr);
 const float distance=obstacle->distance,weight=obstacle->weight;
 const auto* free=observed.voxel({1.5,.1,.1});ASSERT_NE(free,nullptr);
 const float free_distance=free->distance,free_weight=free->weight;
 ASSERT_EQ(observed.status({1.5,.1,.1}),VoxelStatus::Free);
 EXPECT_GT(observed.clearUnknownBox({2.5,.1,.1},{1.8,.4,.4}),0u);
 EXPECT_EQ(observed.voxel({1.9,.1,.1})->distance,distance);
 EXPECT_EQ(observed.voxel({1.9,.1,.1})->weight,weight);
 EXPECT_EQ(observed.status({1.9,.1,.1}),VoxelStatus::Occupied);
 EXPECT_EQ(observed.voxel({1.5,.1,.1})->distance,free_distance);
 EXPECT_EQ(observed.voxel({1.5,.1,.1})->weight,free_weight);
 const auto* cleared=observed.voxel({3.1,.1,.1});ASSERT_NE(cleared,nullptr);
 EXPECT_FLOAT_EQ(cleared->weight,1e-3f+1e-6f);
 EXPECT_FLOAT_EQ(cleared->distance,c.truncation_distance);
 EXPECT_EQ(observed.status({3.1,.1,.1}),VoxelStatus::Free);
 const auto* esdf=observed.esdfVoxel({3.1,.1,.1});ASSERT_NE(esdf,nullptr);EXPECT_TRUE(esdf->observed);
 EXPECT_EQ(observed.clearUnknownBox({2.5,.1,.1},{1.8,.4,.4}),0u);
}
TEST(Tsdf, UnknownBoxUsesSourceCenterLatticeAndFractionalOffsets){
 TsdfBackend map(config());
 // A .2 m box at x=.19 covers indices {-1,0} at .2 m resolution.
 // A world-space bounds implementation would instead cover {0,1}.
 EXPECT_EQ(map.clearUnknownBox({.19,.19,.19},{.2,.2,.2}),8u);
 EXPECT_EQ(map.status({-.1,-.1,-.1}),VoxelStatus::Free);
 EXPECT_EQ(map.status({.1,.1,.1}),VoxelStatus::Free);
 EXPECT_EQ(map.status({.3,.1,.1}),VoxelStatus::Unknown);
 EXPECT_EQ(map.status({.1,.3,.1}),VoxelStatus::Unknown);
 EXPECT_EQ(map.status({.1,.1,.3}),VoxelStatus::Unknown);
 TsdfBackend narrow(config());
 // With a quarter-voxel half-size, the source visits only offset -1.
 EXPECT_EQ(narrow.clearUnknownBox({.19,.19,.19},{.1,.1,.1}),1u);
 EXPECT_EQ(narrow.status({-.1,-.1,-.1}),VoxelStatus::Free);
 EXPECT_EQ(narrow.status({.1,.1,.1}),VoxelStatus::Unknown);
 TsdfBackend extended(config());
 EXPECT_EQ(extended.clearUnknownBox({.19,.19,.19},{.4,.4,.4}),27u);
 EXPECT_EQ(extended.status({.3,.3,.3}),VoxelStatus::Free);
 EXPECT_EQ(extended.status({.5,.1,.1}),VoxelStatus::Unknown);
 TsdfBackend point(config());
 EXPECT_EQ(point.clearUnknownBox({.19,.19,.19},{0.,0.,0.}),1u);
 EXPECT_EQ(point.status({.1,.1,.1}),VoxelStatus::Free);
 EXPECT_EQ(point.status({-.1,.1,.1}),VoxelStatus::Unknown);
 EXPECT_THROW(point.clearUnknownBox({.19,.19,.19},{-.1,0.,0.}),std::invalid_argument);
}
TEST(Esdf, PlanningClearSpheresUseHallucinatedInnerAndOuterSets){
 auto c=config();c.esdf_enabled=true;c.esdf_clear_sphere_for_planning=true;
 c.esdf_clear_sphere_radius=.4;c.esdf_occupied_sphere_radius=1.0;
 TsdfBackend map(c);
 map.addNewRobotPosition({0.,0.,0.});
 voxblox::BlockIndexList all_blocks,updated_blocks;
 map.esdfLayer()->getAllAllocatedBlocks(&all_blocks);
 map.esdfLayer()->getAllUpdatedBlocks(voxblox::Update::kMap,&updated_blocks);
 EXPECT_FALSE(all_blocks.empty());
 EXPECT_EQ(all_blocks.size(),updated_blocks.size()); // ESDF-only sphere reaches Layer readers.
 std::size_t inner_free=0,outer_occupied=0;
 map.forEachObservedVoxel([&](const Eigen::Vector3i&,VoxelStatus,
     const voxblox::TsdfVoxel*,const voxblox::EsdfVoxel* esdf){
   if(!esdf||!esdf->observed||!esdf->hallucinated)return;
   if(esdf->distance>0.)++inner_free;
   if(esdf->distance<0.)++outer_occupied;
 });
 EXPECT_GT(inner_free,0u);EXPECT_GT(outer_occupied,0u);

 // A later real TSDF return must replace hallucinated ESDF values in its
 // observed band; the planning spheres do not overwrite that evidence.
 map.integrate({0.,0.,0.},Eigen::Quaterniond::Identity(),{{1.5,.1,.1}});
 map.addNewRobotPosition({0.,0.,0.});
 bool real_esdf=false;
 map.forEachObservedVoxel([&](const Eigen::Vector3i&,VoxelStatus,
     const voxblox::TsdfVoxel* tsdf,const voxblox::EsdfVoxel* esdf){
   if(tsdf&&tsdf->weight>1e-6f&&esdf&&esdf->observed&&!esdf->hallucinated)real_esdf=true;
 });
 EXPECT_TRUE(real_esdf);

 auto disabled=c;disabled.esdf_clear_sphere_for_planning=false;TsdfBackend no_clear(disabled);
 no_clear.addNewRobotPosition({0.,0.,0.});
 EXPECT_EQ(no_clear.esdfVoxel({0.,0.,0.}),nullptr);
}
