// Adapted from GBPlanner 2 Rrg::modifyPath / improveFreePath, BSD-3-Clause.
// Copyright (c) 2020 UNR Autonomous Robots Lab.
#include "gbplanner_local/path_refinement.hpp"
#include "gbplanner_local/parameters.hpp"
#include "gbplanner_local/path_heading.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace gbplanner2 {
namespace {
constexpr std::size_t kMaxPathPoints=20000,kMaxCorridorSamples=2000000;
double length(const std::vector<StateVec>& path){
  double result=0.;
  for(std::size_t i=1;i<path.size();++i)result+=(path[i]-path[i-1]).head<3>().norm();
  return result;
}
}
void PathRefinementParameters::validate()const{
  if(!safety_extension.allFinite()||(safety_extension.array()<=0.).any()||
     !std::isfinite(interpolation_distance)||interpolation_distance<=0.||
     !std::isfinite(relaxed_corridor_multiplier)||relaxed_corridor_multiplier<=0.)
    throw std::invalid_argument("Invalid path refinement parameters");
}
PathRefiner::PathRefiner(std::shared_ptr<const MapQuery> map,const Parameters& robot,
                         PathRefinementParameters parameters)
    :map_(std::move(map)),p_(std::move(parameters)),robot_size_(robot.robot_size),
     center_offset_(robot.center_offset),tangent_yaw_(robot.tangent_yaw){
  p_.validate();robot.validate();
  if(!map_)throw std::invalid_argument("Path refinement needs a map");
  if(robot.ground_robot)terrain_=std::make_unique<TerrainProjector>(map_,robot.groundParameters());
}
bool PathRefiner::safe(const StateVec& start,const StateVec& end)const{
  if(!start.allFinite()||!end.allFinite())return false;
  if(terrain_)return terrain_->edge(start,end).admissible();
  return map_->path(start.head<3>()+center_offset_,end.head<3>()+center_offset_,robot_size_)==VoxelStatus::Free;
}
bool PathRefiner::projected(StateVec& state)const{
  if(!state.allFinite())return false;
  if(!terrain_)return safe(state,state);
  const auto projection=terrain_->project(state);
  if(projection.status!=GroundStatus::Supported||
     (projection.projected_state-state).head<2>().norm()>1e-8)return false;
  state=projection.projected_state;return true;
}
bool PathRefiner::densify(const std::vector<StateVec>& input,std::vector<StateVec>& output)const{
  output.clear();if(input.empty())return false;
  output.push_back(input.front());
  const double step=2.*p_.interpolation_distance;
  for(std::size_t i=1;i<input.size();++i){
    const Eigen::Vector3d delta=(input[i]-input[i-1]).head<3>();const double distance=delta.norm();
    if(!std::isfinite(distance)||distance/step>kMaxPathPoints)return false;
    // Keep the original interpolation spacing and successor yaw, but retain
    // the exact final endpoint omitted by the upstream interpolation loop.
    for(double at=step;at<distance-1e-9;at+=step){
      if(output.size()>=kMaxPathPoints)return false;
      StateVec point=input[i];point.head<3>()=input[i-1].head<3>()+delta*(at/distance);
      if(!projected(point)||!safe(output.back(),point))return false;
      output.push_back(point);
    }
    if(output.size()>=kMaxPathPoints||!safe(output.back(),input[i]))return false;
    output.push_back(input[i]);
  }
  return true;
}
bool PathRefiner::obstacles(const Eigen::Vector3d& center,const Eigen::Vector3d& axis,
                            const Eigen::Vector3d& size,std::vector<Eigen::Vector3d>& points)const{
  points.clear();const double resolution=map_->resolution();
  if(!center.allFinite()||!axis.allFinite()||!size.allFinite()||axis.norm()<1e-10||
     !std::isfinite(resolution)||resolution<=0.)return false;
  // Same rotated lattice sampling as extractLocalMapAlongAxis. Anchor to the
  // actual snapshot lattice, which can have a nonzero map origin.
  const Eigen::Array3d counts=(size.array()/(2.*resolution)).ceil()*2.;
  if((counts<0.).any()||(counts>kMaxCorridorSamples).any()||
     (counts+1.).prod()>kMaxCorridorSamples)return false;
  const Eigen::Vector3i n=counts.cast<int>().matrix();
  const Eigen::Vector3d half=n.cast<double>()*resolution*.5;
  const Eigen::Vector3d snapped=map_->minimum()+
      (((center-map_->minimum())/resolution).array().floor()+.5).matrix()*resolution;
  const Eigen::Quaterniond rotation=Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(),axis.normalized());
  for(int x=0;x<=n.x();++x)for(int y=0;y<=n.y();++y)for(int z=0;z<=n.z();++z){
    const Eigen::Vector3d point=rotation*(Eigen::Vector3d(x,y,z)*resolution-half)+snapped;
    if(map_->voxel(point)==VoxelStatus::Occupied)points.push_back(point);
  }
  return true;
}
bool PathRefiner::modify(const std::vector<Eigen::Vector3d>& obstacles,
                         const Eigen::Vector3d& start,const Eigen::Vector3d& target,
                         Eigen::Vector3d& adjusted,RefinementGeometry* geometry,
                         RefinementAttemptGeometry* attempt)const{
  const auto finish=[&](bool success,const char* reason){
    if(attempt){attempt->modify_succeeded=success;attempt->reason=reason;attempt->adjusted=adjusted;}
    return success;
  };
  if(attempt)attempt->modify_attempted=true;
  adjusted=target;const Eigen::Vector3d delta=target-start;const double radius=delta.norm()*.5;
  if(radius<1e-8)return finish(true,"degenerate_segment");
  const Eigen::Vector3d center=.5*(start+target),direction=delta.normalized();
  const double yaw=std::atan2(direction.y(),direction.x());
  const double pitch=-std::atan2(direction.z(),direction.head<2>().norm());
  const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())).toRotationMatrix();
  if(attempt)attempt->rotation=rotation;
  const auto record_plane=[&](const Eigen::Vector3d& normal,const Eigen::Vector3d& tangent,bool obstacle){
    if(!geometry)return;
    ++geometry->plane_count;
    if(!attempt||geometry->plane_count-geometry->dropped_planes>RefinementGeometry::kMaxPlanes){
      ++geometry->dropped_planes;geometry->truncated=true;return;
    }
    RefinementPlane plane;plane.normal=rotation*normal;plane.offset=1.+plane.normal.dot(center);
    // This is the original bounding-box face point or the obstacle selected by
    // closest/tangent below, never a synthetic projection of the center.
    plane.tangent=rotation*tangent+center;plane.obstacle=obstacle;
    attempt->planes.push_back(std::move(plane));
  };
  const Eigen::Vector3d half(radius+p_.safety_extension.x(),p_.safety_extension.y(),p_.safety_extension.z());
  std::vector<Eigen::Vector3d> planes,points;
  for(int axis=0;axis<3;++axis)for(double sign:{-1.,1.}){
    Eigen::Vector3d plane=Eigen::Vector3d::Zero();plane[axis]=sign/half[axis];planes.push_back(plane);
    if(geometry){Eigen::Vector3d tangent=Eigen::Vector3d::Zero();tangent[axis]=sign*half[axis];record_plane(plane,tangent,false);}
  }
  // PCL was only a point container and affine-transform utility here. Eigen
  // applies exactly the same yaw/pitch transform without changing the method.
  for(const auto& world:obstacles){
    const Eigen::Vector3d point=rotation.transpose()*(world-center);
    if(std::all_of(planes.begin(),planes.end(),[&](const auto& plane){return plane.dot(point)-1.<=.05;}))
      points.push_back(point);
  }
  if(points.empty())return finish(true,"no_obstacles");
  const auto closest=std::min_element(points.begin(),points.end(),
      [](const auto& a,const auto& b){return a.squaredNorm()<b.squaredNorm();});
  if(!closest->allFinite()||closest->squaredNorm()<.01)return finish(false,"obstacle_at_center");
  Eigen::Vector3d axes=Eigen::Vector3d::Constant(radius);
  if(closest->squaredNorm()<radius*radius){
    const double denominator=1.-closest->x()*closest->x()/(radius*radius);
    if(denominator<=1e-12)return finish(false,"ellipsoid_denominator_invalid");
    const double cross=(closest->y()*closest->y()+closest->z()*closest->z())/denominator;
    if(!std::isfinite(cross)||cross<=1e-12)return finish(false,"ellipsoid_cross_section_invalid");
    axes.y()=axes.z()=std::sqrt(cross);
    planes.push_back(closest->cwiseQuotient(axes.cwiseProduct(axes)));
    if(geometry)record_plane(planes.back(),*closest,true);
  }
  bool complete=false;
  for(int iteration=0;iteration<50;++iteration){
    std::vector<Eigen::Vector3d> reduced;reduced.reserve(points.size());
    const Eigen::Vector3d plane=planes.back();
    for(const auto& point:points)if(plane.dot(point)-1.<-.05)reduced.push_back(point);
    if(reduced.empty()){complete=true;break;}
    const auto tangent=std::min_element(reduced.begin(),reduced.end(),[&](const auto& a,const auto& b){
      return a.cwiseQuotient(axes).squaredNorm()<b.cwiseQuotient(axes).squaredNorm();
    });
    const double extension=tangent->cwiseQuotient(axes).squaredNorm();
    if(!std::isfinite(extension)||extension<1e-12)return finish(false,"tangent_extension_invalid");
    const Eigen::Vector3d next=tangent->cwiseQuotient(axes.cwiseProduct(axes)*extension);
    if(!next.allFinite())return finish(false,"tangent_plane_invalid");
    if(geometry)record_plane(next,*tangent,true);
    planes.push_back(next);points=std::move(reduced);
  }
  if(!complete)return finish(false,"plane_iteration_limit");
  const double sample_count=std::ceil(2.*p_.safety_extension.y()/.1)*
      std::ceil(2.*p_.safety_extension.z()/.1);
  if(!std::isfinite(sample_count)||sample_count>kMaxCorridorSamples)return finish(false,"cross_section_sample_limit");
  Eigen::Vector3d sum=Eigen::Vector3d::Zero();std::size_t count=0;
  for(double y=-p_.safety_extension.y();y<p_.safety_extension.y();y+=.1)
    for(double z=-p_.safety_extension.z();z<p_.safety_extension.z();z+=.1){
      const Eigen::Vector3d point(radius,y,z);
      if(!std::all_of(planes.begin(),planes.end(),[&](const auto& plane){return plane.dot(point)-1.<=-.05;}))continue;
      const Eigen::Vector3d world=rotation*point+center;
      if(map_->voxel(world)==VoxelStatus::Free){
        sum+=world;++count;
        if(geometry){
          ++geometry->free_point_count;
          if(attempt&&geometry->free_point_count-geometry->dropped_free_points<=RefinementGeometry::kMaxFreePoints)
            attempt->free_points.push_back(world);
          else{++geometry->dropped_free_points;geometry->truncated=true;}
        }
      }
    }
  if(!count)return finish(false,"no_free_cross_section");
  adjusted=sum/double(count);
  if(attempt){attempt->centroid=adjusted;attempt->centroid_valid=adjusted.allFinite();}
  return finish(adjusted.allFinite(),adjusted.allFinite()?"centroid_computed":"centroid_invalid");
}
PathRefinementResult PathRefiner::refine(const std::vector<StateVec>& input,
                                         bool relaxed,bool preserve_endpoint)const{
  PathRefinementResult result;result.path=input;
  if(input.empty()||input.size()>kMaxPathPoints){result.reason="invalid_path_size";return result;}
  for(const auto& state:input)if(!state.allFinite()){result.reason="invalid_path_pose";return result;}
  result.path_length=length(input);
  for(std::size_t i=0;i<input.size();++i)if(!safe(input[i?i-1:0],input[i])){
    result.reason="original_path_not_free";return result;
  }
  if(!p_.enabled||input.size()<2){result.success=true;result.reason=p_.enabled?"single_pose":"disabled";return result;}
  std::vector<StateVec> coarse;
  if(!densify(input,coarse)){result.reason="interpolation_unavailable";return result;}
  bool removed=true;
  while(removed&&coarse.size()>2){
    removed=false;
    for(std::size_t i=0;i+2<coarse.size();++i){
      if((coarse[i+1]-coarse[i]).head<3>().norm()<.5&&safe(coarse[i],coarse[i+2])){
        coarse.erase(coarse.begin()+i+1);++result.removed_vertices;removed=true;break;
      }
    }
  }
  std::vector<StateVec> proposed{coarse.front()};
  RefinementGeometry* geometry=p_.capture_geometry?&result.geometry:nullptr;
  if(geometry)geometry->captured=true;
  for(std::size_t i=1;i<coarse.size();++i){
    const StateVec& previous=proposed.back();const StateVec& original=coarse[i];
    StateVec next=original;
    const bool fixed_endpoint=preserve_endpoint&&i+1==coarse.size();
    if(!fixed_endpoint){
      RefinementAttemptGeometry* attempt=nullptr;
      if(geometry){
        ++geometry->attempt_count;
        if(geometry->attempts.size()<RefinementGeometry::kMaxAttempts){
          geometry->attempts.emplace_back();attempt=&geometry->attempts.back();attempt->segment_index=i;
          attempt->start=previous.head<3>();attempt->target=original.head<3>();
          attempt->center=.5*(attempt->start+attempt->target);attempt->adjusted=attempt->target;
        }else{++geometry->dropped_attempts;geometry->truncated=true;}
      }
      StateVec parallel=original;parallel.head<3>()+=previous.head<3>()-coarse[i-1].head<3>();
      StateVec target=original;bool available=false;
      if(projected(parallel)&&safe(previous,parallel)){target=parallel;available=true;}
      else if(safe(previous,original))available=true;
      if(available){
        if(attempt){attempt->target=target.head<3>();attempt->center=.5*(previous.head<3>()+target.head<3>());
          attempt->adjusted=attempt->target;attempt->reason="obstacle_extraction_failed";}
        const Eigen::Vector3d axis=(original-coarse[i-1]).head<3>();
        Eigen::Vector3d extension=p_.safety_extension;
        // Match upstream: relaxed expands obstacle extraction laterally, but
        // the modifyPath cross-section keeps the configured safety box.
        if(relaxed)extension.y()*=p_.relaxed_corridor_multiplier;
        const Eigen::Vector3d size(axis.norm()+2.*extension.x(),2.*extension.y(),2.*extension.z());
        std::vector<Eigen::Vector3d> points;Eigen::Vector3d adjusted;
        if(obstacles(.5*(previous.head<3>()+target.head<3>()),axis,size,points)&&
           modify(points,previous.head<3>(),target.head<3>(),adjusted,geometry,attempt)){
          StateVec candidate=target;candidate.head<3>()=adjusted;
          if(projected(candidate)&&safe(previous,candidate)&&
             (i+1==coarse.size()||safe(candidate,coarse[i+1]))){
            next=candidate;if(attempt)attempt->locally_accepted=true;
          }else if(attempt)attempt->reason="candidate_rejected";
        }
      }
    }
    if(!safe(previous,next)){result.reason="modified_path_not_free";return result;}
    if((next-original).head<3>().norm()>1e-8)++result.adjusted_vertices;
    proposed.push_back(next);
  }
  std::vector<StateVec> output{proposed.front()};
  for(std::size_t i=1;i<proposed.size();++i){
    if(terrain_){
      const auto edge=terrain_->edge(output.back(),proposed[i]);
      if(!edge.admissible()||output.size()+edge.polyline.size()>kMaxPathPoints){result.reason="modified_terrain_not_admissible";return result;}
      // Return every terrain knot, not a chord above/below the surface. Unlike
      // upstream's relaxed branch, a steep edge never becomes executable.
      for(std::size_t k=1;k<edge.polyline.size();++k){
        auto state=edge.polyline[k];
        if(!tangent_yaw_)state[3]=proposed[i][3];
        output.push_back(state);
      }
    }else output.push_back(proposed[i]);
  }
  if(preserve_endpoint&&(output.back()-input.back()).head<3>().norm()>1e-8){
    result.reason="endpoint_projection_changed";return result;
  }
  for(std::size_t i=1;i<output.size();++i){
    if(!safe(output[i-1],output[i])){result.reason="modified_path_not_free";return result;}
  }
  output.front()[3]=input.front()[3];
  applyPathHeading(output,tangent_yaw_,p_.planning_backward);
  result.modified=output.size()!=input.size();
  if(!result.modified)for(std::size_t i=0;i<output.size();++i)
    if((output[i]-input[i]).norm()>1e-8){result.modified=true;break;}
  result.path=std::move(output);result.path_length=length(result.path);
  if(geometry)geometry->path_accepted=true;
  result.success=true;result.reason=result.modified?"refined":"unchanged";return result;
}
PathRefinementDiagnostics refinePlannedPath(std::shared_ptr<const MapQuery> map,
    const Parameters& parameters,std::vector<StateVec>& path,double& path_length,
    bool relaxed,bool preserve_endpoint){
  PathRefinementDiagnostics diagnostics;
  if(!parameters.refinement.enabled)return diagnostics;
  diagnostics.attempted=true;
  try{
    auto refined=PathRefiner(std::move(map),parameters,parameters.refinement)
        .refine(path,relaxed,preserve_endpoint);
    diagnostics.reason=refined.reason;
    diagnostics.geometry=std::move(refined.geometry);
    if(refined.success){
      diagnostics.modified=refined.modified;
      diagnostics.removed_vertices=refined.removed_vertices;
      diagnostics.adjusted_vertices=refined.adjusted_vertices;
      if(parameters.refinement.capture_geometry){
        // Optional display storage must not reject an already checked route.
        try{diagnostics.accepted_path=refined.path;}
        catch(const std::exception&){diagnostics.accepted_path.clear();}
      }
      path=std::move(refined.path);path_length=refined.path_length;
    }
  }catch(const std::exception&){diagnostics.reason="refinement_error";}
  return diagnostics;
}

std::vector<StateVec> interpolateOutputPath(const std::vector<StateVec>& input,
    double spacing,std::size_t max_points){
  if(!std::isfinite(spacing)||spacing<=0.||max_points==0)
    throw std::invalid_argument("Invalid output interpolation parameters");
  if(input.size()>max_points)throw std::length_error("Output path point budget exceeded");
  for(const auto& state:input)
    if(!state.allFinite())throw std::invalid_argument("Nonfinite output path pose");
  if(input.empty())return {};
  std::vector<StateVec> output;
  output.reserve(input.size());output.push_back(input.front());
  for(std::size_t i=1;i<input.size();++i){
    const Eigen::Vector3d delta=(input[i]-input[i-1]).head<3>();
    const double distance=delta.norm();
    if(!std::isfinite(distance))throw std::invalid_argument("Nonfinite output path segment");
    // Add spacing, 2*spacing, ... strictly inside this original segment.
    // Do not resample across its endpoint, merge short terrain segments, or
    // substitute equal-sized subdivisions for the upstream fixed spacing.
    const long double steps=distance>0.?
        std::max(0.L,std::ceil(static_cast<long double>(distance)/spacing)-1.L):0.L;
    if(!std::isfinite(steps)||steps>=static_cast<long double>(max_points-output.size()))
      throw std::length_error("Output path point budget exceeded");
    for(std::size_t j=1;j<=static_cast<std::size_t>(steps);++j){
      const double at=static_cast<double>(j)*spacing;
      // Floating multiplication can round an interior sample onto the exact
      // endpoint; that endpoint is appended below with its original state.
      if(at>=distance)break;
      StateVec point=input[i];
      point.head<3>()=input[i-1].head<3>()+delta*(at/distance);
      output.push_back(point);
    }
    output.push_back(input[i]);
  }
  return output;
}
}
