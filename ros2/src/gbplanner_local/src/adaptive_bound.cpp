// Adapted from ntnu-arl/adaptive_obb_ros at
// 7dc24c973d233f63d73f2a8548e6f5227bc422bb. Original source is retained under
// ros2/upstream/adaptive_obb_ros; its package.xml declares license "TODO".
#include "gbplanner_local/adaptive_bound.hpp"
#include "gbplanner_local/voxel_map.hpp"
#include <Eigen/Geometry>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace gbplanner2 {
void AdaptiveParameters::validate() const {
  const auto positive=[](double value){return std::isfinite(value)&&value>0.;};
  // Upstream accepts these names but implements only kPca. Never execute its
  // uninitialized rotation/variance branches for kMvbb or kAabb.
  if(type!="kPca")throw std::invalid_argument("Adaptive bound supports only implemented type kPca");
  if(!positive(range)||!positive(size_max)||!positive(scale_max)||!positive(leaf)||
     !std::isfinite(static_cast<float>(leaf))||static_cast<float>(leaf)<=0.f)
    throw std::invalid_argument("Invalid adaptive bound parameters");
}

AdaptiveBound::AdaptiveBound(AdaptiveParameters parameters):p_(std::move(parameters)){
  p_.validate();
}

AdaptiveBoundResult AdaptiveBound::fromMap(const MapQuery& map,const Eigen::Vector3d& root) const {
  if(!p_.enabled){AdaptiveBoundResult result;result.reason="disabled";return result;}
  if(!root.allFinite()){AdaptiveBoundResult result;result.reason="invalid_root";return result;}
  std::vector<Eigen::Vector3d> cloud;
  const double pi=std::acos(-1.),angular_resolution=5.*pi/180.;
  cloud.reserve(2700);
  // Original MapManager::getLocalPointcloud(center,range,0,cloud,false):
  // full-sphere rays ignore free/unknown cells and retain the first occupied
  // voxel center. This input is independent of gain policies and sensor FOV.
  for(double theta=0.;theta<2.*pi;theta+=angular_resolution){
    for(double phi=0.;phi<pi;phi+=angular_resolution){
      const Eigen::Vector3d endpoint=root+p_.range*Eigen::Vector3d(
          std::cos(theta)*std::sin(phi),std::sin(theta)*std::sin(phi),std::cos(phi));
      map.ray(root,endpoint,[&](const Eigen::Vector3d& point,VoxelStatus status){
        if(status!=VoxelStatus::Occupied)return true;
        cloud.push_back(point);return false;
      });
    }
  }
  return fromCloud(cloud,root);
}

AdaptiveBoundResult AdaptiveBound::fromCloud(const std::vector<Eigen::Vector3d>& cloud,
                                            const Eigen::Vector3d& root) const {
  AdaptiveBoundResult result;result.raw_points=cloud.size();
  if(!p_.enabled){result.reason="disabled";return result;}
  if(!root.allFinite()){result.reason="invalid_root";return result;}
  if(cloud.size()<3){result.reason="insufficient_points";return result;}
  using Cloud=pcl::PointCloud<pcl::PointXYZI>;
  Cloud::Ptr input(new Cloud),filtered(new Cloud);
  input->reserve(cloud.size());
  for(const auto& point:cloud){
    if(!point.allFinite()){result.reason="nonfinite_cloud";return result;}
    const auto converted=point.cast<float>().eval();
    if(!converted.allFinite()){result.reason="point_out_of_range";return result;}
    pcl::PointXYZI sample;
    sample.x=converted.x();sample.y=converted.y();sample.z=converted.z();sample.intensity=250.f;
    input->push_back(sample);
  }
  try{
    // Keep the original PCL float conversion, VoxelGrid centroids and PCA
    // implementation. Replacing them with an Eigen-only PCA changes the path.
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(input);
    voxel_filter.setLeafSize(p_.leaf,p_.leaf,p_.leaf);
    voxel_filter.filter(*filtered);
    result.filtered_points=filtered->size();
    if(filtered->size()<3){result.reason="insufficient_filtered_points";return result;}

    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(filtered);
    Eigen::Matrix3d rotation=pca.getEigenVectors().cast<double>();
    const double determinant=rotation.determinant();
    if(!rotation.allFinite()||!std::isfinite(determinant)||std::abs(determinant)<1e-12){
      result.reason="invalid_pca_rotation";return result;
    }
    rotation=rotation/determinant;
    rotation.transposeInPlace();
    result.eigenvalues=pca.getEigenValues().cast<double>();
    Eigen::Vector3d variance=result.eigenvalues/(double(filtered->size())-1.);
    // Original rotationsSafetyCheck swaps rows and their associated variance.
    if(std::abs(rotation(2,0))>.8){
      Eigen::Matrix3d permutation;permutation<<0,1,0,0,0,1,1,0,0;
      rotation=(permutation*rotation).eval();variance=(permutation*variance).eval();
    }
    // The source's variance.cwiseAbs() has no assignment. Preserve positive
    // inputs exactly; a singular/negative spectrum falls back to configured
    // sampling rather than producing NaN or a zero-width sampling interval.
    if(!variance.allFinite()||(variance.array()<=0.).any()){
      result.reason="degenerate_cloud";return result;
    }
    const Eigen::Vector3d sigma=variance.cwiseSqrt();
    const double sigma_max=sigma.maxCoeff();
    if(!std::isfinite(sigma_max)||sigma_max<=0.){
      result.reason="degenerate_cloud";return result;
    }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*filtered,centroid);
    result.centroid=centroid.head<3>().cast<double>();
    const Eigen::Vector3d offset=rotation*(result.centroid-root);
    // These assignments intentionally overwrite the original bounds. The
    // Gaussian/Cauchy mean remains zero even when the cuboid is offset.
    result.min=offset-sigma/sigma_max*p_.size_max/2.;
    result.max=offset+sigma/sigma_max*p_.size_max/2.;
    result.rotations=rotation.eulerAngles(2,1,0);
    result.scale=sigma/sigma_max*p_.scale_max;
    result.mean=Eigen::Vector3d::Zero();result.rotation=rotation;
    if(!result.centroid.allFinite()||!result.min.allFinite()||!result.max.allFinite()||
       !result.rotations.allFinite()||!result.scale.allFinite()||
       (result.min.array()>=result.max.array()).any()||(result.scale.array()<=0.).any()){
      result.reason="invalid_adaptive_geometry";return result;
    }
    result.applied=true;result.reason="ok";return result;
  }catch(const std::exception&){
    result.reason="pcl_error";return result;
  }
}
}  // namespace gbplanner2
