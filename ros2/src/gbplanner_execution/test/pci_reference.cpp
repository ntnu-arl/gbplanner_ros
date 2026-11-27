#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#define ROS_WARN_COND(...) ((void)0)
#define ROS_INFO_COND(...) ((void)0)
// The original translation unit obtains floating abs through ROS/tf headers.
// Make that intended overload explicit in this ROS-free reference harness.
using std::abs;
namespace tf {
struct Quaternion {
  double yaw{0};
  void setEuler(double,double,double angle){yaw=angle;}
  double x()const{return 0;} double y()const{return 0;}
  double z()const{return std::sin(yaw/2);} double w()const{return std::cos(yaw/2);}
};
using Vector3=Eigen::Vector3d;
struct Pose {Quaternion q;Vector3 p;Pose(Quaternion quat,Vector3 pos):q(quat),p(pos){}};
double getYaw(const geometry_msgs::msg::Quaternion& q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
void poseTFToMsg(const Pose& in,geometry_msgs::msg::Pose& out){
  out.position.x=in.p.x();out.position.y=in.p.y();out.position.z=in.p.z();
  out.orientation.x=in.q.x();out.orientation.y=in.q.y();out.orientation.z=in.q.z();out.orientation.w=in.q.w();
}
}
class Reference {
 public:
  double v_max_{1},yaw_rate_max_{.15},dt_{.05};
  geometry_msgs::msg::Pose current_pose_;
  std::vector<geometry_msgs::msg::Pose> executing_path_;
  bool concatenate_path_enable_{false};
  bool planAhead(){return false;}
  double calculateDistance(const geometry_msgs::msg::Pose& a,const geometry_msgs::msg::Pose& b){
    return Eigen::Vector3d(a.position.x-b.position.x,a.position.y-b.position.y,a.position.z-b.position.z).norm();
  }
  void allocateYawAlongPath(std::vector<geometry_msgs::msg::Pose>&)const;
  void interpolatePath(const std::vector<geometry_msgs::msg::Pose>&,std::vector<geometry_msgs::msg::Pose>&);
  bool reconnectPath(const std::vector<geometry_msgs::msg::Pose>&,std::vector<geometry_msgs::msg::Pose>&);
};
#include "upstream_pci.inc"
geometry_msgs::msg::Pose readPose(){
  geometry_msgs::msg::Pose p;double yaw;
  std::cin>>p.position.x>>p.position.y>>p.position.z>>yaw;
  p.orientation.z=std::sin(yaw/2);p.orientation.w=std::cos(yaw/2);return p;
}
int main(){
  int mode,count;Reference r;std::cin>>mode>>r.v_max_>>r.yaw_rate_max_>>r.dt_>>count;
  if(count<1||count>100)return 2;
  r.current_pose_=readPose();std::vector<geometry_msgs::msg::Pose> input,output;
  for(int i=0;i<count;++i)input.push_back(readPose());
  if(mode==0){r.allocateYawAlongPath(input);output=input;}
  else if(mode==1)r.interpolatePath(input,output);
  else if(mode==2){if(!r.reconnectPath(input,output)){std::cout<<-1;return 0;}}
  else return 3;
  std::cout<<std::setprecision(17)<<output.size()<<'\n';
  for(const auto& p:output)std::cout<<p.position.x<<' '<<p.position.y<<' '<<p.position.z<<' '<<tf::getYaw(p.orientation)<<'\n';
}
