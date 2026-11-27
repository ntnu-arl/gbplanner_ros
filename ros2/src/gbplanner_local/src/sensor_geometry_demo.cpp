#include "gbplanner_local/sensor_geometry.hpp"
#include "gbplanner_local/parameters.hpp"
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* reason){if(!condition)throw std::runtime_error(reason);}
Eigen::Matrix3d rotation(const SensorParameters& sensor){
  return (Eigen::AngleAxisd(sensor.rotations[0],Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(sensor.rotations[1],Eigen::Vector3d::UnitY())*
          Eigen::AngleAxisd(sensor.rotations[2],Eigen::Vector3d::UnitX())).toRotationMatrix();
}
// Independent fixed-source isInsideFOV statements, including float angles.
bool originalContains(const SensorParameters& sensor,const StateVec& state,const Eigen::Vector3d& point){
  const Eigen::Vector3d pos=rotation(sensor).inverse()*
      Eigen::AngleAxisd(-state[3],Eigen::Vector3d::UnitZ())*(point-state.head<3>())-sensor.center_offset;
  const float length=pos.norm();if(length>sensor.range)return false;
  if(sensor.type=="kCamera"){
    const double h=sensor.horizontal_fov/2.,v=sensor.vertical_fov/2.;
    Eigen::Matrix<double,3,4> corners,normals;
    corners.col(0)=Eigen::Vector3d(std::cos(h),std::sin(h),std::sin(v));
    corners.col(1)=Eigen::Vector3d(std::cos(h),-std::sin(h),std::sin(v));
    corners.col(2)=Eigen::Vector3d(std::cos(h),-std::sin(h),-std::sin(v));
    corners.col(3)=Eigen::Vector3d(std::cos(h),std::sin(h),-std::sin(v));
    for(int i=0;i<4;++i)normals.col(i)=corners.col(i).cross(corners.col((i+1)%4));
    for(int i=0;i<4;++i)if(pos.dot(normals.col(i))<=0)return false;
  }else{
    const float horizontal=std::atan2(static_cast<float>(pos.y()),static_cast<float>(pos.x()));
    const float vertical=std::asin(static_cast<float>(pos.z())/length);
    if(std::abs(horizontal)>sensor.horizontal_fov/2.||std::abs(vertical)>sensor.vertical_fov/2.)return false;
  }
  return true;
}
void pointJson(const Eigen::Vector3d& point){std::cout<<'['<<point.x()<<','<<point.y()<<','<<point.z()<<']';}
}

int main(){
  try{
    SensorParameters camera;camera.name="camera";camera.type="kCamera";
    camera.range=4.;camera.min_range=1.;camera.horizontal_fov=1.2;camera.vertical_fov=.8;camera.validate();
    const StateVec origin=StateVec::Zero();
    require(sensorContains(camera,origin,Eigen::Vector3d(2.,0.,0.)),"Camera rejected a forward point");
    require(!sensorContains(camera,origin,Eigen::Vector3d(-2.,0.,0.)),"Camera accepted a rear point");
    require(!sensorContains(camera,origin,Eigen::Vector3d::Zero()),"Camera accepted its strict-boundary apex");
    const Eigen::Vector3d corner(std::cos(.6),std::sin(.6),std::sin(.4));
    require(!sensorContains(camera,origin,corner),"Camera accepted an exact corner boundary");
    const Eigen::Vector3d top_edge(std::cos(.6),0.,std::sin(.4));
    // The analytic plane is not necessarily an exact zero after trig/cross
    // products. Preserve the source's sign decision at floating-point zero,
    // then independently check points on both sides of the plane.
    require(sensorContains(camera,origin,top_edge)==originalContains(camera,origin,top_edge),
            "Camera upper-plane rounding differs from the original");
    require(sensorContains(camera,origin,top_edge-Eigen::Vector3d(0,0,1e-7))&&
            !sensorContains(camera,origin,top_edge+Eigen::Vector3d(0,0,1e-7)),
            "Camera upper-plane inside/outside decision is incorrect");
    require(!sensorContains(camera,origin,Eigen::Vector3d(4.01,0.,0.)),"Camera accepted a point past maximum range");
    require(sensorContains(camera,origin,Eigen::Vector3d(.2,0.,0.)),"Volume FOV unexpectedly applies min_range");
    require(sensorContains(camera,origin,Eigen::Vector3d(4.,0.,0.))&&
            !sensorContains(camera,origin,Eigen::Vector3d(4.01,0.,0.)),"Camera range boundary changed");

    camera.rotations={.4,-.2,.1};camera.center_offset={.25,-.15,.2};
    const StateVec viewpoint(1.2,-.8,.4,.7);
    const Eigen::Matrix3d to_world=Eigen::AngleAxisd(viewpoint[3],Eigen::Vector3d::UnitZ())*rotation(camera);
    const Eigen::Vector3d forward=viewpoint.head<3>()+to_world*(Eigen::Vector3d(2.,0.,0.)+camera.center_offset);
    require(sensorContains(camera,viewpoint,forward),"Camera extrinsics changed the original offset order");
    const auto edges=sensorFrustum(camera,viewpoint);
    require(edges.size()==8,"Camera frustum needs four rays and four rim edges");
    const Eigen::Vector3d expected=viewpoint.head<3>()+
        Eigen::AngleAxisd(viewpoint[3],Eigen::Vector3d::UnitZ())*(rotation(camera)*(camera.range*corner));
    require((edges.front().second-expected).norm()<1e-12,"Frustum corner differs from original transform");
    for(std::size_t i=0;i<4;++i){
      require((edges[i].first-viewpoint.head<3>()).norm()<1e-12,"Frustum apex is not the robot origin");
      require((edges[4+i].first-edges[i].second).norm()<1e-12&&
              (edges[4+i].second-edges[(i+1)%4].second).norm()<1e-12,"Frustum rim is disconnected");
    }
    SensorParameters lidar=camera;lidar.name="lidar";lidar.type="kLidar";
    lidar.rotations.setZero();lidar.center_offset.setZero();lidar.horizontal_fov=1.4;lidar.vertical_fov=.7;
    require(sensorContains(lidar,origin,Eigen::Vector3d(2.,.2,.2)),"Lidar rejected its forward sector");
    require(!sensorContains(lidar,origin,Eigen::Vector3d(0.,2.,0.))&&
            !sensorContains(lidar,origin,Eigen::Vector3d(1.,0.,1.)),"Lidar accepted an out-of-FOV point");
    require(sensorContains(lidar,origin,Eigen::Vector3d::Zero()),"Lidar origin compatibility changed");
    require(sensorFrustum(lidar,origin).empty(),"Lidar invented a camera pyramid");

    int comparisons=0;
    for(const auto& sensor:std::array<SensorParameters,2>{camera,lidar})
      for(int ix=-4;ix<=4;++ix)for(int iy=-4;iy<=4;++iy)for(int iz=-4;iz<=4;++iz){
        const Eigen::Vector3d point=viewpoint.head<3>()+Eigen::AngleAxisd(viewpoint[3],Eigen::Vector3d::UnitZ())*
            Eigen::Vector3d(.7*ix,.7*iy,.7*iz);
        require(sensorContains(sensor,viewpoint,point)==originalContains(sensor,viewpoint,point),
                "Static FOV differs from the original formula");++comparisons;
      }
    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"original_comparisons\":"<<comparisons
      <<",\"camera_strict_boundary\":true,\"original_offset_order\":true,\"lidar_sector\":true"
      <<",\"frustum_edges\":"<<edges.size()<<",\"first_corner\":";
    pointJson(edges.front().second);std::cout<<"}\n";return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
