// Opt-in geometry follows actual refinement work without issuing map queries.
#include "gbplanner_local/path_refinement.hpp"
#include "gbplanner_local/parameters.hpp"
#include "gbplanner_local/refinement_markers.hpp"
#include <algorithm>
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
  for(std::size_t i=0;i<a.size();++i)if((a[i]-b[i]).norm()>1e-12)return false;
  return true;
}
// Analytic corridor with two occupied half-spaces. The collision methods
// evaluate the same walls as voxel(), including the rotated robot AABB extent.
class CorridorMap final:public MapQuery {
 public:
  Eigen::Matrix3d rotation;
  Eigen::Vector3d origin{2.,-1.,.7};
  mutable std::array<std::size_t,10> queries{};
  mutable std::vector<Eigen::Vector3d> occupied_queries;
  std::size_t reject_path_query{0};
  bool retain_occupied{true};
  CorridorMap(){
    rotation=(Eigen::AngleAxisd(.6,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(-.35,Eigen::Vector3d::UnitY())).toRotationMatrix();
  }
  double resolution()const override {++queries[0];return .1;}
  Eigen::Vector3d minimum()const override {++queries[1];return Eigen::Vector3d::Constant(-20.);}
  Eigen::Vector3d maximum()const override {++queries[2];return Eigen::Vector3d::Constant(200.);}
  VoxelStatus voxel(const Eigen::Vector3d& point)const override {
    ++queries[3];const bool occupied=std::abs((rotation.transpose()*(point-origin)).y())>=.55;
    if(occupied&&retain_occupied)occupied_queries.push_back(point);
    return occupied?VoxelStatus::Occupied:VoxelStatus::Free;
  }
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d&)const override {++queries[4];return std::nullopt;}
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d&)const override {++queries[5];return std::nullopt;}
  VoxelStatus collision(const Eigen::Vector3d& from,const Eigen::Vector3d& to,const Eigen::Vector3d& size)const{
    const double extent=rotation.col(1).cwiseAbs().dot(size)*.5;
    const double lateral=std::max(std::abs((rotation.transpose()*(from-origin)).y()),
                                  std::abs((rotation.transpose()*(to-origin)).y()));
    return lateral+extent<.55?VoxelStatus::Free:VoxelStatus::Occupied;
  }
  VoxelStatus box(const Eigen::Vector3d& point,const Eigen::Vector3d& size)const override {
    ++queries[6];return collision(point,point,size);
  }
  VoxelStatus path(const Eigen::Vector3d& from,const Eigen::Vector3d& to,const Eigen::Vector3d& size)const override {
    ++queries[7];
    if(reject_path_query&&queries[7]==reject_path_query)return VoxelStatus::Occupied;
    return collision(from,to,size);
  }
  bool gainAllowed(const Eigen::Vector3d&)const override {++queries[8];return true;}
  void ray(const Eigen::Vector3d& from,const Eigen::Vector3d& to,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit)const override {
    ++queries[9];const int count=std::max(1,static_cast<int>(std::ceil((to-from).norm()/.1)));
    for(int i=0;i<=count;++i){const Eigen::Vector3d point=from+(to-from)*(double(i)/count);
      if(!visit(point,voxel(point)))break;}
  }
  StateVec state(double x)const {StateVec point;point.head<3>()=origin+rotation*Eigen::Vector3d(x,0.,0.);point[3]=.6;return point;}
};
Parameters parameters(){
  Parameters p;p.robot_size={.1,.1,.1};p.refinement.enabled=true;
  p.refinement.safety_extension={.4,1.,1.};p.refinement.interpolation_distance=.5;return p;
}
struct Run {std::shared_ptr<CorridorMap> map;PathRefinementResult result;};
Run run(const Parameters& parameters,const std::vector<StateVec>& input,bool capture,
        std::size_t reject_path_query=0,bool retain_occupied=true){
  auto map=std::make_shared<CorridorMap>();map->reject_path_query=reject_path_query;map->retain_occupied=retain_occupied;
  auto refinement=parameters.refinement;refinement.capture_geometry=capture;
  auto result=PathRefiner(map,parameters,refinement).refine(input,false,false);
  return {std::move(map),std::move(result)};
}
void sameRun(const Run& off,const Run& on){
  const auto& a=off.result;const auto& b=on.result;
  require(a.success==b.success&&a.modified==b.modified&&a.reason==b.reason&&a.path_length==b.path_length&&
          a.removed_vertices==b.removed_vertices&&a.adjusted_vertices==b.adjusted_vertices&&samePath(a.path,b.path),
          "Geometry capture changed the refinement result");
  require(off.map->queries==on.map->queries,"Geometry capture changed map query counts");
  require(!a.geometry.captured&&a.geometry.attempts.empty()&&a.geometry.attempt_count==0&&
          a.geometry.plane_count==0&&a.geometry.free_point_count==0,"Disabled capture retained geometry");
}
void checkCounts(const RefinementGeometry& geometry){
  std::size_t planes=0,points=0;
  for(const auto& attempt:geometry.attempts){planes+=attempt.planes.size();points+=attempt.free_points.size();}
  require(geometry.attempts.size()<=RefinementGeometry::kMaxAttempts&&planes<=RefinementGeometry::kMaxPlanes&&
          points<=RefinementGeometry::kMaxFreePoints,"Geometry exceeded its request-wide storage limits");
  require(geometry.attempt_count==geometry.attempts.size()+geometry.dropped_attempts&&
          geometry.plane_count==planes+geometry.dropped_planes&&
          geometry.free_point_count==points+geometry.dropped_free_points,
          "Dropped geometry counts do not account for the complete request");
}
}

int main(){
  try{
    auto p=parameters();CorridorMap scene;
    const std::vector<StateVec> input{scene.state(0.),scene.state(1.),scene.state(2.)};
    const auto off=run(p,input,false),on=run(p,input,true);sameRun(off,on);
    const auto& geometry=on.result.geometry;
    require(on.result.success&&geometry.captured&&geometry.path_accepted&&!geometry.attempts.empty(),
            "Successful refinement produced no accepted geometry snapshot");
    checkCounts(geometry);std::size_t obstacle_planes=0,free_points=0,centroids=0;
    bool rotated=false;
    for(const auto& attempt:geometry.attempts){
      require((attempt.center-.5*(attempt.start+attempt.target)).norm()<1e-12,
              "Attempt center does not describe its actual endpoints");
      rotated=rotated||(attempt.rotation-Eigen::Matrix3d::Identity()).norm()>.1;
      if(attempt.centroid_valid){
        ++centroids;Eigen::Vector3d sum=Eigen::Vector3d::Zero();
        for(const auto& point:attempt.free_points)sum+=point;
        require(!attempt.free_points.empty()&&
                (sum/double(attempt.free_points.size())-attempt.centroid).norm()<1e-10&&
                (attempt.adjusted-attempt.centroid).norm()<1e-12,
                "Centroid was not computed from the captured Free cross-section");
      }
      for(const auto& plane:attempt.planes){
        require(std::abs(plane.normal.dot(plane.tangent)-plane.offset)<1e-9,
                "World plane does not pass through its actual tangent");
        if(plane.obstacle){
          ++obstacle_planes;
          require(std::any_of(on.map->occupied_queries.begin(),on.map->occupied_queries.end(),
              [&](const auto& point){return (point-plane.tangent).norm()<1e-9;}),
              "Obstacle tangent was fabricated instead of retaining a queried occupied point");
        }
        for(const auto& point:attempt.free_points)
          require(plane.normal.dot(point)-plane.offset<=-.05+1e-9,
                  "Captured cross-section point violates an actual half-space");
      }
      for(const auto& point:attempt.free_points){
        ++free_points;
        require(std::abs((on.map->rotation.transpose()*(point-on.map->origin)).y())<.55,
                "Captured point was not in the map's free corridor");
      }
    }
    require(rotated&&obstacle_planes>0&&free_points>0&&centroids>0,
            "Scene did not exercise rotated obstacle planes and free cross-section centroids");

    // Reject only the final validation query. Earlier local modifications were
    // accepted, but the full refinement must retain the original input path.
    const std::size_t final_query=off.map->queries[7];
    const auto failed_off=run(p,input,false,final_query),failed_on=run(p,input,true,final_query);
    sameRun(failed_off,failed_on);
    require(!failed_on.result.success&&failed_on.result.reason=="modified_path_not_free"&&
            !failed_on.result.geometry.path_accepted&&samePath(failed_on.result.path,input)&&
            std::any_of(failed_on.result.geometry.attempts.begin(),failed_on.result.geometry.attempts.end(),
              [](const auto& attempt){return attempt.locally_accepted;}),
            "Local acceptance was mistaken for acceptance of the complete failed refinement");
    p.refinement.capture_geometry=true;
    auto failed_map=std::make_shared<CorridorMap>();failed_map->reject_path_query=final_query;
    auto retained=input;double retained_length=2.;
    const auto diagnostic=refinePlannedPath(failed_map,p,retained,retained_length,false,false);
    require(diagnostic.geometry.captured&&!diagnostic.geometry.path_accepted&&diagnostic.accepted_path.empty()&&samePath(retained,input)&&
            retained_length==2.,"refinePlannedPath lost failure geometry or changed the retained safe path");

    // Capture at the production adoption boundary. Later response interpolation
    // must not rewrite this chain or add a single map query to the refinement.
    auto capture_map=std::make_shared<CorridorMap>(),plain_map=std::make_shared<CorridorMap>();
    auto captured_path=input,plain_path=input;double captured_length=2.,plain_length=2.;
    auto capture_parameters=p;capture_parameters.refinement.capture_geometry=true;
    const auto captured=refinePlannedPath(capture_map,capture_parameters,captured_path,captured_length,false,false);
    auto plain_parameters=p;plain_parameters.refinement.capture_geometry=false;
    const auto plain=refinePlannedPath(plain_map,plain_parameters,plain_path,plain_length,false,false);
    require(captured.geometry.path_accepted&&samePath(captured.accepted_path,captured_path)&&
            samePath(captured_path,plain_path)&&captured_length==plain_length&&
            captured.reason==plain.reason&&captured.modified==plain.modified&&plain.accepted_path.empty()&&
            capture_map->queries==plain_map->queries,"Accepted-path capture changed the refinement or map queries");
    for(std::size_t i=0;i<captured_path.size();++i)
      require((captured.accepted_path[i].array()==captured_path[i].array()).all(),"Captured StateVec was altered");
    const auto interpolated=interpolateOutputPath(captured_path,.07);
    require(interpolated.size()>captured_path.size()&&samePath(captured.accepted_path,captured_path),
            "Output interpolation overwrote the original refined chain");
    std_msgs::msg::Header header;header.frame_id="map";header.stamp.sec=17;
    const auto markers=refinedPathMarkers(captured,header,true);
    using Marker=visualization_msgs::msg::Marker;
    require(markers.markers.size()==2&&markers.markers[0].action==Marker::DELETEALL,
            "Refined path is not a complete marker snapshot");
    const auto& line=markers.markers[1];
    require(line.ns=="mod_path"&&line.id==0&&line.type==Marker::LINE_LIST&&line.action==Marker::ADD&&
            line.scale.x==.25&&line.color.r==float(244./255.)&&line.color.g==float(217./255.)&&
            line.color.b==float(66./255.)&&line.color.a==1.f&&line.header==header&&
            line.points.size()==2*(captured_path.size()-1),"Refined path changed the upstream display style");
    for(std::size_t i=1;i<captured_path.size();++i)
      for(std::size_t side=0;side<2;++side)
        require(line.points[2*(i-1)+side]==markerPoint(captured_path[i-1+side].head<3>()),
                "Marker points differ from the adopted chain");
    require(refinedPathMarkers(captured,header,false).markers.size()==1&&
            refinedPathMarkers(diagnostic,header,true).markers.size()==1&&
            refinedPathMarkers(plain,header,true).markers.size()==1&&
            refinedPathMarkers({},header,false).markers.size()==1,
            "Failed/uncaptured/cleared refinement retained a displayed path");

    auto disabled_parameters=p;disabled_parameters.refinement.enabled=false;
    const auto disabled=run(disabled_parameters,input,true);
    require(disabled.result.success&&!disabled.result.geometry.captured&&disabled.result.geometry.attempts.empty(),
            "Disabled refinement retained an attempted geometry snapshot");
    auto invalid=input;invalid[0].x()=std::numeric_limits<double>::quiet_NaN();
    const auto invalid_result=run(p,invalid,true);
    require(!invalid_result.result.success&&!invalid_result.result.geometry.captured&&
            invalid_result.result.geometry.attempts.empty(),"Invalid input retained stale geometry");
    const auto early=run(p,input,true,1);
    require(!early.result.success&&!early.result.geometry.captured&&early.result.geometry.attempts.empty(),
            "An original-path rejection claimed an attempted corridor");

    std::vector<StateVec> long_path;
    for(int i=0;i<=80;++i)long_path.push_back(scene.state(i));
    const auto bounded_off=run(p,long_path,false,0,false),bounded_on=run(p,long_path,true,0,false);
    sameRun(bounded_off,bounded_on);checkCounts(bounded_on.result.geometry);
    const auto& bounded=bounded_on.result.geometry;
    require(bounded.attempt_count>RefinementGeometry::kMaxAttempts&&bounded.dropped_attempts>0&&
            bounded.dropped_planes>0&&bounded.dropped_free_points>0&&bounded.truncated,
            "Long refinement did not exercise request-wide bounded capture");
    std::cout<<"{\"status\":\"passed\",\"rotated_world_planes\":true,\"actual_obstacle_tangents\":"
      <<obstacle_planes<<",\"free_cross_section_points\":"<<free_points<<",\"centroids\":"<<centroids
      <<",\"final_rejection_preserves_local_attempts\":true,\"path_and_status_unchanged\":true"
      <<",\"additional_map_queries\":0,\"bounded_attempts\":"<<bounded.attempt_count
      <<",\"dropped_attempts\":"<<bounded.dropped_attempts<<",\"dropped_planes\":"<<bounded.dropped_planes
      <<",\"dropped_free_points\":"<<bounded.dropped_free_points<<",\"disabled_and_invalid_empty\":true"
      <<",\"accepted_path_exact\":true,\"refined_points\":"<<captured_path.size()
      <<",\"interpolated_points\":"<<interpolated.size()
      <<",\"upstream_mod_path_markers\":true,\"failed_response_clears_mod_path\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
