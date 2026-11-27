#include "gbplanner_local/scan_integrator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <set>
#include <sstream>

namespace gbplanner2 {
class OccupancyMapperNode final:public rclcpp::Node {
 public:
  OccupancyMapperNode():Node("occupancy_mapper") {
    frame_=parameter<std::string>("frame_id","map");
    sensor_frame_=parameter<std::string>("sensor_frame","lidar");
    if(frame_.empty())throw std::invalid_argument("Empty frame name");
    double resolution=parameter<double>("map.resolution",0.25);
    auto origin=parameter<std::vector<double>>("map.origin",{-5,-5,-1.5});
    auto dimensions=parameter<std::vector<int64_t>>("map.dimensions",{40,40,12});
    if(origin.size()!=3||dimensions.size()!=3)throw std::invalid_argument("Map shape requires three components");
    for(auto d:dimensions)if(d<1||d>10000)throw std::invalid_argument("Invalid map dimension");
    map_=std::make_shared<VoxelMap>(resolution,Eigen::Vector3d(origin[0],origin[1],origin[2]),
      Eigen::Vector3i(dimensions[0],dimensions[1],dimensions[2]));
    min_range_=parameter<double>("sensor.min_range",0.1);max_range_=parameter<double>("sensor.max_range",6.0);
    integrator_=std::make_unique<ScanIntegrator>(min_range_,max_range_);
    tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    map_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("voxel_map",rclcpp::QoS(1).reliable().transient_local());
    configureInputs();
    configureFreeSpace();
    status_service_=create_service<std_srvs::srv::Trigger>("mapper_status",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=scans_>0||freespace_scans_>0;
        response->message="{\"integrated_scans\":"+std::to_string(scans_)+",\"rejected_scans\":"+
          std::to_string(rejected_)+",\"known_cells\":"+std::to_string(known_)+
          ",\"accepted_points\":"+std::to_string(last_.accepted_points)+
          ",\"skipped_points\":"+std::to_string(last_.skipped_points)+",\"input_sources\":"+inputStatus()+
          ",\"freespace\":"+freeSpaceStatus()+"}";
      });
    reset_service_=create_service<std_srvs::srv::Trigger>("reset_map",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        map_->fill(VoxelStatus::Unknown);known_=0;last_={};scans_=0;resetInputStats();
        freespace_scans_=freespace_rejected_=freespace_empty_=0;last_freespace_={};publish(now());
        response->success=true;response->message="Occupancy map reset to unknown";
      });
    RCLCPP_INFO(get_logger(),"Occupancy ray mapper ready: %zu input(s) -> %s",inputs_.size(),frame_.c_str());
  }
 private:
  template<class T>T parameter(const std::string& name,const T& value){
    rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
    return declare_parameter<T>(name,value,descriptor);
  }
  struct InputSource {
    std::string name,topic,frame;
    uint64_t integrated{0},rejected{0};
  };
  static std::string jsonString(const std::string& value){
    std::string result="\"";
    const char* hex="0123456789abcdef";
    for(unsigned char c:value){
      if(c=='"'||c=='\\'){result+='\\';result+=char(c);}
      else if(c<0x20){result+="\\u00";result+=hex[c>>4];result+=hex[c&15];}
      else result+=char(c);
    }
    return result+"\"";
  }
  std::string inputStatus()const{
    std::ostringstream out;out<<"{";
    for(std::size_t i=0;i<inputs_.size();++i){
      const auto& source=inputs_[i];if(i)out<<",";
      out<<jsonString(source.name)<<":{\"topic\":"<<jsonString(source.topic)
         <<",\"frame\":"<<jsonString(source.frame)
         <<",\"integrated_scans\":"<<source.integrated
         <<",\"rejected_scans\":"<<source.rejected<<"}";
    }
    out<<"}";return out.str();
  }
  void resetInputStats(){
    for(auto& source:inputs_){source.integrated=0;source.rejected=0;}
  }
  std::string freeSpaceStatus()const{
    std::ostringstream out;out<<"{\"enabled\":"<<(freespace_enabled_?"true":"false")
      <<",\"integrated_clouds\":"<<freespace_scans_<<",\"rejected_clouds\":"<<freespace_rejected_
      <<",\"empty_clouds\":"<<freespace_empty_<<",\"accepted_points\":"<<last_freespace_.accepted_points
      <<",\"skipped_points\":"<<last_freespace_.skipped_points<<",\"free_updates\":"<<last_freespace_.free_updates<<"}";
    return out.str();
  }
  void configureFreeSpace(){
    freespace_enabled_=parameter<bool>("use_freespace_pointcloud",false);
    std::vector<std::string> frames;for(const auto& input:inputs_)frames.push_back(input.frame);
    frames=parameter<std::vector<std::string>>("freespace.sensor_frames",frames);
    for(const auto& frame:frames){
      if(frame.empty())throw std::invalid_argument("Empty freespace sensor frame");
      freespace_frames_.insert(frame);
    }
    if(freespace_enabled_&&freespace_frames_.empty())throw std::invalid_argument("Freespace input needs sensor frames");
    if(!freespace_enabled_)return;
    freespace_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("freespace_pointcloud",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message){receiveFreeSpace(*message);});
    for(const auto& input:inputs_)if(input.topic==freespace_sub_->get_topic_name())
      throw std::invalid_argument("Freespace and obstacle-return input topics must differ");
  }
  void configureInputs(){
    const auto names=parameter<std::vector<std::string>>("input_sensors",{});
    if(names.size()>16)throw std::invalid_argument("At most 16 input sensors are supported");
    if(names.empty()){
      if(sensor_frame_.empty())throw std::invalid_argument("Empty sensor frame name");
      inputs_.push_back({"legacy","points",sensor_frame_});
    }else{
      std::set<std::string> seen;
      for(const auto& name:names){
        if(name.empty()||!seen.insert(name).second||
           name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")!=std::string::npos)
          throw std::invalid_argument("Invalid or duplicate input sensor name");
        auto topic=parameter<std::string>("input."+name+".topic","");
        auto frame=parameter<std::string>("input."+name+".frame","");
        if(topic.empty()||frame.empty())throw std::invalid_argument("Each input sensor needs an explicit topic and frame");
        inputs_.push_back({name,topic,frame});
      }
    }
    // One serialized mapper integrates every source into the same backend.
    // Resolve names through ROS first so aliases/remaps cannot subscribe twice
    // to one stream and count/fuse a cloud more than once.
    std::set<std::string> resolved_topics;
    for(std::size_t i=0;i<inputs_.size();++i){
      auto subscription=create_subscription<sensor_msgs::msg::PointCloud2>(
        inputs_[i].topic,rclcpp::SensorDataQoS(),
        [this,i](sensor_msgs::msg::PointCloud2::ConstSharedPtr message){receive(*message,i);});
      const std::string resolved=subscription->get_topic_name();
      if(!resolved_topics.insert(resolved).second)throw std::invalid_argument("Input sensor topics must be unique after ROS name resolution");
      inputs_[i].topic=resolved;cloud_subscriptions_.push_back(std::move(subscription));
    }
  }
  void receive(const sensor_msgs::msg::PointCloud2& message,std::size_t source_index){
    auto& source=inputs_.at(source_index);
    try {
      if(message.header.frame_id!=source.frame)throw std::invalid_argument("Wrong sensor frame");
      last_=integrateMessage(message,false);++scans_;++source.integrated;publish(message.header.stamp);
      RCLCPP_INFO(get_logger(),"Integrated scan %d: %zu returns, %zu known voxels",scans_,last_.accepted_points,known_);
    }catch(const std::exception& error){++rejected_;++source.rejected;RCLCPP_WARN(get_logger(),"Scan rejected (%s): %s",source.name.c_str(),error.what());}
  }
  void receiveFreeSpace(const sensor_msgs::msg::PointCloud2& message){
    try{
      if(!freespace_frames_.count(message.header.frame_id))throw std::invalid_argument("Unconfigured freespace sensor frame");
      if(message.width==0&&message.height==1&&message.row_step==0&&message.data.empty()){++freespace_empty_;return;}
      last_freespace_=integrateMessage(message,true);++freespace_scans_;publish(message.header.stamp);
    }catch(const std::exception& error){++freespace_rejected_;RCLCPP_WARN(get_logger(),"Freespace cloud rejected: %s",error.what());}
  }
  IntegrationResult integrateFreeSpace(const Eigen::Vector3d& origin,const std::vector<Eigen::Vector3d>& endpoints){
    if(!origin.allFinite()||!map_->inside(map_->index(origin)))throw std::invalid_argument("Sensor origin outside map");
    if(max_range_/map_->resolution()>30000)throw std::invalid_argument("Freespace integration budget exceeded");
    IntegrationResult result;
    for(const auto& endpoint:endpoints){
      if(!endpoint.allFinite()){++result.skipped_points;continue;}
      const Eigen::Vector3d delta=endpoint-origin;const double distance=delta.norm();
      if(!std::isfinite(distance)||distance<=min_range_){++result.skipped_points;continue;}
      ++result.accepted_points;
      const Eigen::Vector3d end=distance<=max_range_?endpoint:Eigen::Vector3d(origin+delta*(max_range_/distance));
      // Synthetic endpoints express a lower bound on clear distance. Unlike
      // obstacle returns, their endpoint cell is never marked occupied. This
      // occupancy adapter has no TSDF evidence weights: retain a previously
      // observed surface and stop there instead of erasing it with augmentation.
      map_->ray(origin,end,[&](const Eigen::Vector3d& point,VoxelStatus status){
        const auto index=map_->index(point);if(!map_->inside(index)||status==VoxelStatus::Occupied)return false;
        if(status==VoxelStatus::Unknown){map_->set(index,VoxelStatus::Free);++result.free_updates;}
        return true;
      });
    }
    return result;
  }
  IntegrationResult integrateMessage(const sensor_msgs::msg::PointCloud2& message,bool freespace){
      if(message.header.frame_id.empty()||message.is_bigendian||message.height==0||message.point_step==0)
        throw std::invalid_argument("Sensor frame, endian or shape invalid");
      if(message.header.stamp.sec==0 && message.header.stamp.nanosec==0)
        throw std::invalid_argument("Cloud timestamp must be explicit; zero would request latest TF");
      if(message.width==0||uint64_t(message.width)*message.height>200000 ||
         message.row_step<uint64_t(message.width)*message.point_step||
         message.data.size()!=uint64_t(message.row_step)*message.height)
        throw std::invalid_argument("Cloud data size invalid");
      std::set<uint32_t> field_bytes;
      auto field=[&](const std::string& name){
        const sensor_msgs::msg::PointField* result=nullptr;
        for(const auto& f:message.fields)if(f.name==name){if(result)throw std::invalid_argument("Duplicate cloud field");result=&f;}
        if(!result||result->datatype!=sensor_msgs::msg::PointField::FLOAT32||result->count!=1||
           uint64_t(result->offset)+4>message.point_step)throw std::invalid_argument("Cloud requires float32 XYZ");
        for(uint32_t b=result->offset;b<result->offset+4;++b)
          if(!field_bytes.insert(b).second)throw std::invalid_argument("Overlapping cloud fields");
        return result->offset;
      };
      auto ox=field("x"),oy=field("y"),oz=field("z");
      // Each source uses its own acquisition timestamp, not a synchronized
      // bundle or the newest timestamp of another sensor. An older interleaved
      // cloud remains valid if TF has that time; the snapshot retains its stamp.
      const auto transform=tf_buffer_->lookupTransform(frame_,message.header.frame_id,rclcpp::Time(message.header.stamp));
      const auto& t=transform.transform.translation;const auto& q=transform.transform.rotation;
      Eigen::Vector3d origin(t.x,t.y,t.z);Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
      if(!rotation.coeffs().allFinite()||rotation.norm()<1e-8)throw std::invalid_argument("Invalid sensor transform");
      rotation.normalize();std::vector<Eigen::Vector3d> endpoints;
      endpoints.reserve(uint64_t(message.width)*message.height);
      for(uint32_t row=0;row<message.height;++row)for(uint32_t col=0;col<message.width;++col){
        const auto* data=message.data.data()+uint64_t(row)*message.row_step+uint64_t(col)*message.point_step;
        float x,y,z;std::memcpy(&x,data+ox,4);std::memcpy(&y,data+oy,4);std::memcpy(&z,data+oz,4);
        endpoints.emplace_back(origin+rotation*Eigen::Vector3d(x,y,z));
      }
      return freespace?integrateFreeSpace(origin,endpoints):integrator_->integrate(*map_,origin,endpoints);
  }
  void publish(const builtin_interfaces::msg::Time& stamp){
    sensor_msgs::msg::PointCloud2 message;message.header.frame_id=frame_;message.header.stamp=stamp;
    message.height=1;message.is_dense=true;message.point_step=16;
    for(int i=0;i<3;++i){sensor_msgs::msg::PointField field;field.name=std::string(1,"xyz"[i]);field.offset=i*4;field.datatype=7;field.count=1;message.fields.push_back(field);}
    sensor_msgs::msg::PointField status;status.name="occupancy";status.offset=12;status.datatype=1;status.count=1;message.fields.push_back(status);
    known_=0;
    for(int z=0;z<map_->dimensions().z();++z)for(int y=0;y<map_->dimensions().y();++y)for(int x=0;x<map_->dimensions().x();++x){
      const auto center=map_->center({x,y,z});const auto state=map_->voxel(center);
      if(state==VoxelStatus::Unknown)continue;
      const std::size_t offset=message.data.size();message.data.resize(offset+16,0);
      float fx=center.x(),fy=center.y(),fz=center.z();int8_t value=static_cast<int8_t>(state);
      std::memcpy(message.data.data()+offset,&fx,4);std::memcpy(message.data.data()+offset+4,&fy,4);
      std::memcpy(message.data.data()+offset+8,&fz,4);std::memcpy(message.data.data()+offset+12,&value,1);++known_;
    }
    message.width=known_;message.row_step=message.width*message.point_step;map_pub_->publish(message);
  }
  std::string frame_,sensor_frame_;std::shared_ptr<VoxelMap> map_;std::unique_ptr<ScanIntegrator> integrator_;
  double min_range_{.1},max_range_{6.};bool freespace_enabled_{false};
  std::set<std::string> freespace_frames_;
  std::size_t freespace_scans_{0},freespace_rejected_{0},freespace_empty_{0};IntegrationResult last_freespace_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr freespace_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<InputSource> inputs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subscriptions_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_,reset_service_;
  int scans_{0},rejected_{0};std::size_t known_{0};IntegrationResult last_;
};
}
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<gbplanner2::OccupancyMapperNode>());}
  catch(const std::exception& error){std::cerr<<"Mapper startup failed: "<<error.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
