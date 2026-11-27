#include "gbplanner_local/adaptive_bound.hpp"
#include <Eigen/Geometry>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){
  if(!condition)throw std::runtime_error(message);
}
void vectorJson(const Eigen::Vector3d& value){
  std::cout<<'['<<value.x()<<','<<value.y()<<','<<value.z()<<']';
}
// Independent transcription of the numerical statements in the fixed source
// constructBoundingBox, with its ROS map acquisition supplied by this fixture.
AdaptiveBoundResult original(const std::vector<Eigen::Vector3d>& points,
                             const Eigen::Vector3d& root,const AdaptiveParameters& p){
  using Cloud=pcl::PointCloud<pcl::PointXYZI>;
  Cloud::Ptr input(new Cloud),filtered(new Cloud);
  for(const auto& point:points){
    pcl::PointXYZI sample;sample.x=point.x();sample.y=point.y();sample.z=point.z();sample.intensity=250;
    input->push_back(sample);
  }
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setInputCloud(input);filter.setLeafSize(p.leaf,p.leaf,p.leaf);filter.filter(*filtered);
  pcl::PCA<pcl::PointXYZI> pca;pca.setInputCloud(filtered);
  Eigen::Matrix3d rotation=pca.getEigenVectors().cast<double>();
  rotation=rotation/rotation.determinant();rotation.transposeInPlace();
  Eigen::Vector3d variance=pca.getEigenValues().cast<double>()/(double(filtered->size())-1.);
  if(std::abs(rotation(2,0))>.8){
    Eigen::Matrix3d swap;swap<<0,1,0,0,0,1,1,0,0;
    rotation=(swap*rotation).eval();variance=(swap*variance).eval();
  }
  Eigen::Vector4f center;pcl::compute3DCentroid(*filtered,center);
  const Eigen::Vector3d offset=rotation*(center.head<3>().cast<double>()-root);
  const Eigen::Vector3d sigma=variance.cwiseSqrt();const double maximum=sigma.maxCoeff();
  AdaptiveBoundResult result;result.applied=true;result.filtered_points=filtered->size();
  result.rotation=rotation;result.centroid=center.head<3>().cast<double>();
  result.eigenvalues=pca.getEigenValues().cast<double>();
  result.min=offset-sigma/maximum*p.size_max/2.;
  result.max=offset+sigma/maximum*p.size_max/2.;
  result.rotations=rotation.eulerAngles(2,1,0);result.mean=Eigen::Vector3d::Zero();
  result.scale=sigma/maximum*p.scale_max;return result;
}
}

int main(){
  try{
    AdaptiveParameters parameters;parameters.enabled=true;parameters.leaf=.3;
    parameters.size_max=8.;parameters.scale_max=3.;
    const Eigen::Matrix3d corridor_rotation=(Eigen::AngleAxisd(.63,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(.31,Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(-.17,Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d corridor_center(1.2,-.7,.9),root(.2,-.1,.1);
    std::vector<Eigen::Vector3d> cloud;
    const auto point=[&](double x,double y,double z){cloud.push_back(corridor_center+corridor_rotation*Eigen::Vector3d(x,y,z));};
    // Walls, ceiling and floor of a long tilted 3D corridor, with close samples
    // that make the upstream VoxelGrid stage materially change point count.
    for(int ix=-30;ix<=30;++ix){
      for(int iz=-5;iz<=5;++iz){point(.2*ix,-1.2,.14*iz);point(.2*ix,1.2,.14*iz);}
      for(int iy=-6;iy<=6;++iy){point(.2*ix,.2*iy,-.7);point(.2*ix,.2*iy,.7);}
    }
    const auto result=AdaptiveBound(parameters).fromCloud(cloud,root);
    const auto reference=original(cloud,root,parameters);
    require(result.applied,"Tilted corridor did not produce adaptive bounds");
    require(result.raw_points==cloud.size()&&result.filtered_points<result.raw_points&&result.filtered_points>100,
            "PCL downsampling did not retain a useful reduced cloud");
    const double reference_error=std::max({(result.min-reference.min).norm(),(result.max-reference.max).norm(),
        (result.rotations-reference.rotations).norm(),(result.mean-reference.mean).norm(),
        (result.scale-reference.scale).norm(),(result.rotation-reference.rotation).norm(),
        (result.centroid-reference.centroid).norm(),(result.eigenvalues-reference.eigenvalues).norm()});
    require(result.filtered_points==reference.filtered_points&&reference_error<1e-12,
            "Adapted core differs from the original PCL numerical path");
    Eigen::Index longest_axis;(result.max-result.min).maxCoeff(&longest_axis);
    const double axis_alignment=std::abs(result.rotation.row(longest_axis).dot(corridor_rotation.col(0)));
    require(axis_alignment>.98,"Adaptive box does not follow the tilted corridor");
    require(std::abs((result.max-result.min).maxCoeff()-parameters.size_max)<1e-12,
            "Longest adaptive side does not equal configured maximum");
    require(result.mean.norm()==0.,"Sampling mean changed from the upstream zero convention");
    const auto empty=AdaptiveBound(parameters).fromCloud({},root);
    const auto repeated=AdaptiveBound(parameters).fromCloud(std::vector<Eigen::Vector3d>(10,root),root);
    require(!empty.applied&&!repeated.applied,"Insufficient observations did not fall back");
    bool unsupported_rejected=false;
    try{auto unsupported=parameters;unsupported.type="kMvbb";AdaptiveBound unused(unsupported);}
    catch(const std::invalid_argument&){unsupported_rejected=true;}
    require(unsupported_rejected,"Unimplemented upstream type was accepted");
    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"raw_points\":"<<result.raw_points
      <<",\"filtered_points\":"<<result.filtered_points<<",\"original_reference_max_error\":"<<reference_error
      <<",\"corridor_axis_alignment\":"<<axis_alignment<<",\"rotation\":[";
    for(int row=0;row<3;++row){if(row)std::cout<<',';vectorJson(result.rotation.row(row).transpose());}
    std::cout<<"],\"min\":";vectorJson(result.min);std::cout<<",\"max\":";vectorJson(result.max);
    std::cout<<",\"rotations\":";vectorJson(result.rotations);std::cout<<",\"mean\":";vectorJson(result.mean);
    std::cout<<",\"scale\":";vectorJson(result.scale);std::cout<<",\"centroid\":";vectorJson(result.centroid);
    std::cout<<",\"eigenvalues\":";vectorJson(result.eigenvalues);
    std::cout<<",\"empty_reason\":\""<<empty.reason<<"\",\"repeated_reason\":\""<<repeated.reason
      <<"\",\"unsupported_type_rejected\":true}\n";return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
