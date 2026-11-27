#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <btBulletDynamicsCommon.h>
#include <rotors_control/lee_position_controller.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <algorithm>
using Vector=Eigen::Vector3d;
using Clock=std::chrono::steady_clock;
static Vector eigen(const btVector3& v){return {v.x(),v.y(),v.z()};}
static btVector3 bullet(const Vector& v){return {btScalar(v.x()),btScalar(v.y()),btScalar(v.z())};}
struct Setpoint {double time; mav_msgs::EigenTrajectoryPoint state;};
struct LidarSource {
  std::string name,frame;
  btTransform body_from_sensor;
  int azimuth_min,azimuth_max;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  std::size_t scans{0};
};
class DynamicsNode:public rclcpp::Node {
 public:
  DynamicsNode():Node("gbplanner_dynamics") {
    auto vec=[this](const std::string& name,const std::vector<double>& defaults){
      const auto v=declare_parameter<std::vector<double>>(name,defaults);
      if(v.size()!=3)throw std::invalid_argument(name+" needs 3 components");return Vector(v[0],v[1],v[2]);};
    mass_=declare_parameter<double>("mass",.5265);inertia_=vec("inertia",{.01,.01,.01});
    auto& vehicle=lee_.vehicle_parameters_;auto& gains=lee_.controller_parameters_;
    vehicle.mass_=mass_;vehicle.inertia_=inertia_.asDiagonal();
    gains.position_gain_=vec("position_gain",{2.,2.,2.});gains.velocity_gain_=vec("velocity_gain",{1.2,1.2,1.2});
    gains.attitude_gain_=vec("attitude_gain",{.7,.7,.015});gains.angular_rate_gain_=vec("angular_rate_gain",{.12,.12,.012});
    auto angles=declare_parameter<std::vector<double>>("rotor_angles",{-.785398,2.35619,.78539,-2.35619});
    auto directions=declare_parameter<std::vector<int64_t>>("rotor_directions",{1,1,-1,-1});
    double arm=declare_parameter<double>("arm_length",.1),force=declare_parameter<double>("force_constant",8.54858e-6);
    double moment=declare_parameter<double>("moment_constant",.016);
    if(angles.size()!=4||directions.size()!=4)throw std::invalid_argument("Four rotor configuration required");
    vehicle.rotor_configuration_.rotors.clear();
    for(int i=0;i<4;++i)vehicle.rotor_configuration_.rotors.emplace_back(angles[i],arm,force,moment,directions[i]);
    lee_.InitializeParameters();allocation_=gains.allocation_matrix_;
    up_=declare_parameter<double>("motor_time_up",.0125);down_=declare_parameter<double>("motor_time_down",.025);
    motor_max_=declare_parameter<double>("motor_max_speed",2000.);
    rotor_drag_=declare_parameter<double>("rotor_drag",8.06428e-5);rolling_=declare_parameter<double>("rolling_moment",1e-6);
    step_=declare_parameter<double>("physics_step",.002);lidar_period_=declare_parameter<double>("lidar_period",.15);
    lidar_layout_=declare_parameter<std::string>("lidar_layout","legacy");
    if(lidar_layout_!="legacy"&&lidar_layout_!="dual")throw std::invalid_argument("lidar_layout must be legacy or dual");
    const auto body_size=vec("body_size",{.25,.25,.113});const auto position=vec("initial_position",{-2.,0.,0.});
    double yaw=declare_parameter<double>("initial_yaw",M_PI/2);
    if(!(mass_>0&&step_>0&&step_<=.01&&up_>0&&down_>0&&lidar_period_>.02)||
       (inertia_.array()<=0).any()||(body_size.array()<=0).any())throw std::invalid_argument("Invalid dynamics configuration");
    collision_=std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher_=std::make_unique<btCollisionDispatcher>(collision_.get());broadphase_=std::make_unique<btDbvtBroadphase>();
    solver_=std::make_unique<btSequentialImpulseConstraintSolver>();
    world_=std::make_unique<btDiscreteDynamicsWorld>(dispatcher_.get(),broadphase_.get(),solver_.get(),collision_.get());
    world_->setGravity(btVector3(0,0,-9.81));
    // Collision objects define both physics and laser returns.
    box({-.25,-5,-1.5},{.25,-1,1.5});box({-.25,1,-1.5},{.25,5,1.5});
    box({-5,-5,-1.5},{-4.75,5,1.5});box({4.75,-5,-1.5},{5,5,1.5});
    box({-5,-5,-1.5},{5,-4.75,1.5});box({-5,4.75,-1.5},{5,5,1.5});
    box({-5,-5,-1.5},{5,5,-1.25});box({-5,-5,1.25},{5,5,1.5});
    btTransform initial;initial.setIdentity();initial.setOrigin(bullet(position));initial.setRotation(btQuaternion(btVector3(0,0,1),yaw));
    body_=addBody(body_size*.5,initial,mass_,inertia_,2);body_->setActivationState(DISABLE_DEACTIVATION);body_->setDamping(0,0);
    body_->setCcdMotionThreshold(.02);body_->setCcdSweptSphereRadius(.05);
    motors_=Eigen::Vector4d::Constant(std::sqrt(mass_*9.81/(4*force)));
    desired_.position_W=position;desired_.yaw=yaw;last_position_=position;
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("odometry",20);
    const auto add_lidar=[this](const std::string& name,const std::string& frame,
        const std::string& topic,double offset,double yaw,int azimuth_min,int azimuth_max){
      btTransform extrinsic;extrinsic.setIdentity();extrinsic.setOrigin(btVector3(offset,0,0));
      extrinsic.setRotation(btQuaternion(btVector3(0,0,1),yaw));
      lidars_.push_back({name,frame,extrinsic,azimuth_min,azimuth_max,
          create_publisher<sensor_msgs::msg::PointCloud2>(topic,rclcpp::SensorDataQoS()),0});
    };
    if(lidar_layout_=="legacy")add_lidar("legacy","lidar","points",0,0,-180,180);
    else{
      add_lidar("front","front_lidar","/front/points",.2,0,-90,90);
      add_lidar("rear","rear_lidar","/rear/points",-.2,M_PI,-90,90);
    }
    telemetry_pub_=create_publisher<std_msgs::msg::String>("dynamics_status",10);
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    command_sub_=create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("command/trajectory",10,
      [this](trajectory_msgs::msg::MultiDOFJointTrajectory::ConstSharedPtr m){receive(*m);});
    status_srv_=create_service<std_srvs::srv::Trigger>("dynamics_state",[this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){r->success=true;r->message=status();});
    auto file=declare_parameter<std::string>("telemetry_file","");
    if(!file.empty()){const std::filesystem::path path(file);if(!path.parent_path().empty())std::filesystem::create_directories(path.parent_path());
      log_.open(file);if(!log_)throw std::runtime_error("Cannot create telemetry");
      log_<<"time,x,y,z,vx,vy,vz,qx,qy,qz,qw,wx,wy,wz,target_x,target_y,target_z,tracking_error,thrust,rotor0,rotor1,rotor2,rotor3,contacts,commands,distance\n";}
    last_tick_=Clock::now();last_lidar_=last_tick_-std::chrono::seconds(1);last_output_=last_tick_;
    timer_=create_wall_timer(std::chrono::milliseconds(5),[this](){tick();});
    RCLCPP_INFO(get_logger(),"Original Lee + Bullet ready: initial hover, mass %.4f kg, lidar layout %s",mass_,lidar_layout_.c_str());
  }
  ~DynamicsNode(){for(auto& body:bodies_)world_->removeRigidBody(body.get());}
 private:
  btRigidBody* addBody(const Vector& half,const btTransform& pose,double mass,const Vector& inertia,int group){
    auto shape=std::make_unique<btBoxShape>(bullet(half));shape->setMargin(.001);auto motion=std::make_unique<btDefaultMotionState>(pose);
    btRigidBody::btRigidBodyConstructionInfo info(mass,motion.get(),shape.get(),bullet(inertia));auto body=std::make_unique<btRigidBody>(info);auto* result=body.get();
    world_->addRigidBody(result,group,group==2?1:3);shapes_.push_back(std::move(shape));motions_.push_back(std::move(motion));bodies_.push_back(std::move(body));return result;}
  void box(const Vector& lo,const Vector& hi){btTransform p;p.setIdentity();p.setOrigin(bullet((lo+hi)*.5));addBody((hi-lo)*.5,p,0,Vector::Zero(),1);}
  void receive(const trajectory_msgs::msg::MultiDOFJointTrajectory& message){
    try{if(message.header.frame_id!="map"||message.points.empty())throw std::invalid_argument("trajectory shape/frame");
      std::vector<Setpoint> next;double previous=-1;
      for(const auto& p:message.points){double time=p.time_from_start.sec+p.time_from_start.nanosec*1e-9;
        if(p.transforms.size()!=1||time<=previous)throw std::invalid_argument("trajectory time/transform");
        const auto& t=p.transforms.front();Eigen::Quaterniond q(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z);
        mav_msgs::EigenTrajectoryPoint state;state.position_W={t.translation.x,t.translation.y,t.translation.z};
        if(!state.position_W.allFinite()||!q.coeffs().allFinite()||q.norm()<1e-8)throw std::invalid_argument("nonfinite trajectory");q.normalize();
        state.yaw=std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
        if(!p.velocities.empty()){auto& v=p.velocities.front();state.velocity_W={v.linear.x,v.linear.y,v.linear.z};state.yaw_rate=v.angular.z;}
        if(!p.accelerations.empty()){auto& v=p.accelerations.front();state.acceleration_W={v.linear.x,v.linear.y,v.linear.z};}
        next.push_back({time,state});previous=time;}
      commands_=std::move(next);command_start_=Clock::now();command_index_=0;desired_=commands_[0].state;++command_count_;
      RCLCPP_INFO(get_logger(),"Physical trajectory %zu: %zu samples, %.2f s",command_count_,commands_.size(),commands_.back().time);
    }catch(const std::exception& e){++rejected_commands_;RCLCPP_WARN(get_logger(),"%s",e.what());}}
  void updateDesired(Clock::time_point now){if(commands_.empty())return;const double elapsed=std::chrono::duration<double>(now-command_start_).count();
    while(command_index_+1<commands_.size()&&commands_[command_index_+1].time<=elapsed)++command_index_;
    // RotorS timed sample holding; command derivatives are preserved.
    desired_=commands_[command_index_].state;}
  void physics(){
    auto pose=body_->getWorldTransform();auto q=pose.getRotation();rotors_control::EigenOdometry odom;
    odom.position=eigen(pose.getOrigin());odom.orientation=Eigen::Quaterniond(q.w(),q.x(),q.y(),q.z());const Eigen::Matrix3d R=odom.orientation.toRotationMatrix();
    odom.velocity=R.transpose()*eigen(body_->getLinearVelocity());odom.angular_velocity=R.transpose()*eigen(body_->getAngularVelocity());
    lee_.SetOdometry(odom);lee_.SetTrajectoryPoint(desired_);Eigen::VectorXd requested;lee_.CalculateRotorVelocities(&requested);
    for(int i=0;i<4;++i){const double target=std::clamp(requested[i],0.,motor_max_);const double alpha=std::exp(-step_/(target>motors_[i]?up_:down_));motors_[i]=alpha*motors_[i]+(1-alpha)*target;}
    const Eigen::Vector4d wrench=allocation_*motors_.array().square().matrix();thrust_=wrench[3];
    const Vector velocity=eigen(body_->getLinearVelocity()),lateral=velocity-R.col(2)*velocity.dot(R.col(2));
    body_->applyCentralForce(bullet(R.col(2)*thrust_-rotor_drag_*motors_.sum()*lateral));
    body_->applyTorque(bullet(R*wrench.head<3>()-rolling_*motors_.sum()*lateral));world_->stepSimulation(step_,0,step_);++steps_;
    const Vector position=eigen(body_->getWorldTransform().getOrigin());distance_+=(position-last_position_).norm();last_position_=position;
    max_error_=std::max(max_error_,(position-desired_.position_W).norm());
    double tilt=std::acos(std::clamp(double(body_->getWorldTransform().getBasis().getColumn(2).z()),-1.,1.));max_tilt_=std::max(max_tilt_,tilt);
    bool contact=false;for(int i=0;i<dispatcher_->getNumManifolds();++i){auto* m=dispatcher_->getManifoldByIndexInternal(i);if(m->getBody0()!=body_&&m->getBody1()!=body_)continue;
      for(int j=0;j<m->getNumContacts();++j)if(m->getContactPoint(j).getDistance()<0)contact=true;}if(contact)++contacts_;
  }
  void output(const rclcpp::Time& stamp){
    const auto pose=body_->getWorldTransform();const auto p=pose.getOrigin();const auto q=pose.getRotation();
    nav_msgs::msg::Odometry odom;odom.header.frame_id="map";odom.child_frame_id="base_link";odom.header.stamp=stamp;
    odom.pose.pose.position.x=p.x();odom.pose.pose.position.y=p.y();odom.pose.pose.position.z=p.z();
    odom.pose.pose.orientation.x=q.x();odom.pose.pose.orientation.y=q.y();odom.pose.pose.orientation.z=q.z();odom.pose.pose.orientation.w=q.w();
    const auto v=pose.getBasis().transpose()*body_->getLinearVelocity(),w=pose.getBasis().transpose()*body_->getAngularVelocity();
    odom.twist.twist.linear.x=v.x();odom.twist.twist.linear.y=v.y();odom.twist.twist.linear.z=v.z();
    odom.twist.twist.angular.x=w.x();odom.twist.twist.angular.y=w.y();odom.twist.twist.angular.z=w.z();odom_pub_->publish(odom);
    for(const auto& lidar:lidars_){
      // Retain all three attitude axes when moving each rigid sensor mount.
      const auto sensor_pose=lidar_layout_=="legacy"?pose:pose*lidar.body_from_sensor;
      const auto origin=sensor_pose.getOrigin();const auto rotation=sensor_pose.getRotation();
      geometry_msgs::msg::TransformStamped t;t.header=odom.header;t.child_frame_id=lidar.frame;
      t.transform.translation.x=origin.x();t.transform.translation.y=origin.y();t.transform.translation.z=origin.z();
      t.transform.rotation.x=rotation.x();t.transform.rotation.y=rotation.y();t.transform.rotation.z=rotation.z();t.transform.rotation.w=rotation.w();
      tf_->sendTransform(t);
    }
    std_msgs::msg::String message;message.data=status();telemetry_pub_->publish(message);
    if(log_){auto v=body_->getLinearVelocity(),w=body_->getAngularVelocity();log_<<std::setprecision(12)<<steps_*step_<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<v.x()<<','<<v.y()<<','<<v.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<w.x()<<','<<w.y()<<','<<w.z()<<','<<desired_.position_W.x()<<','<<desired_.position_W.y()<<','<<desired_.position_W.z()<<','<<(eigen(p)-desired_.position_W).norm()<<','<<thrust_;
      for(int i=0;i<4;++i)log_<<','<<motors_[i];log_<<','<<contacts_<<','<<command_count_<<','<<distance_<<'\n';log_.flush();}
  }
  void scan(const rclcpp::Time& stamp){
    const auto body_pose=body_->getWorldTransform();pending_.clear();pending_.reserve(lidars_.size());
    for(auto& lidar:lidars_){
      const auto pose=lidar_layout_=="legacy"?body_pose:body_pose*lidar.body_from_sensor;
      sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id=lidar.frame;cloud.header.stamp=stamp;cloud.height=1;cloud.point_step=12;cloud.is_dense=true;
      for(int i=0;i<3;++i){sensor_msgs::msg::PointField f;f.name=std::string(1,"xyz"[i]);f.offset=i*4;f.datatype=7;f.count=1;cloud.fields.push_back(f);}
      for(int elevation=-70;elevation<=70;elevation+=4)for(int azimuth=lidar.azimuth_min;azimuth<lidar.azimuth_max;azimuth+=2){const double v=elevation*M_PI/180,h=azimuth*M_PI/180;
        btVector3 direction(std::cos(v)*std::cos(h),std::cos(v)*std::sin(h),std::sin(v));const btVector3 from=pose.getOrigin(),to=from+pose.getBasis()*direction*7.;
        btCollisionWorld::ClosestRayResultCallback ray(from,to);ray.m_collisionFilterGroup=1;ray.m_collisionFilterMask=1;world_->rayTest(from,to,ray);
        double range=ray.hasHit()?7.*ray.m_closestHitFraction:7.;float xyz[3]={float(direction.x()*range),float(direction.y()*range),float(direction.z()*range)};
        const auto offset=cloud.data.size();cloud.data.resize(offset+12);std::memcpy(cloud.data.data()+offset,xyz,12);++cloud.width;}
      cloud.row_step=cloud.width*12;pending_.push_back(std::move(cloud));++lidar.scans;
    }
    // Both clouds retain this capture pose/stamp while the vehicle moves during
    // the same transport delay used by the legacy single-lidar simulation.
    pending_at_=Clock::now();has_pending_=true;++scans_;
  }
  std::string status() const {auto p=body_->getWorldTransform().getOrigin();std::ostringstream s;s<<std::setprecision(12)
      <<"{\"physics\":\"Bullet\",\"steps\":"<<steps_<<",\"simulated_seconds\":"<<steps_*step_<<",\"commands\":"<<command_count_
      <<",\"rejected_commands\":"<<rejected_commands_<<",\"scans\":"<<scans_<<",\"contact_steps\":"<<contacts_<<",\"distance_m\":"<<distance_
      <<",\"maximum_tracking_error_m\":"<<max_error_<<",\"maximum_tilt_rad\":"<<max_tilt_<<",\"speed_mps\":"<<body_->getLinearVelocity().length()
      <<",\"position\":["<<p.x()<<','<<p.y()<<','<<p.z()<<"],\"thrust_newtons\":"<<thrust_
      <<",\"lidar_layout\":\""<<lidar_layout_<<"\",\"sensor_scans\":{";
    for(std::size_t i=0;i<lidars_.size();++i){if(i)s<<',';s<<'\"'<<lidars_[i].name<<"\":"<<lidars_[i].scans;}
    s<<"}}";return s.str();}
  void tick(){auto clock=Clock::now();accumulator_+=std::chrono::duration<double>(clock-last_tick_).count();last_tick_=clock;updateDesired(clock);
    while(accumulator_>=step_){physics();accumulator_-=step_;}
    if(has_pending_&&std::chrono::duration<double>(clock-pending_at_).count()>=.025){
      for(std::size_t i=0;i<pending_.size();++i)lidars_[i].publisher->publish(pending_[i]);has_pending_=false;}
    if(std::chrono::duration<double>(clock-last_output_).count()>=.02){auto stamp=now();output(stamp);last_output_=clock;
      if(!has_pending_&&std::chrono::duration<double>(clock-last_lidar_).count()>=lidar_period_){scan(stamp);last_lidar_=clock;}}}
  double mass_,up_,down_,motor_max_,rotor_drag_,rolling_,step_,lidar_period_,accumulator_{0},distance_{0},thrust_{0},max_error_{0},max_tilt_{0};
  Vector inertia_,last_position_;Eigen::Vector4d motors_;Eigen::Matrix4Xd allocation_;rotors_control::LeePositionController lee_;
  mav_msgs::EigenTrajectoryPoint desired_;std::vector<Setpoint> commands_;std::size_t command_index_{0},command_count_{0},rejected_commands_{0},steps_{0},contacts_{0},scans_{0};
  std::unique_ptr<btDefaultCollisionConfiguration> collision_;std::unique_ptr<btCollisionDispatcher> dispatcher_;std::unique_ptr<btBroadphaseInterface> broadphase_;
  std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;std::unique_ptr<btDiscreteDynamicsWorld> world_;std::vector<std::unique_ptr<btCollisionShape>> shapes_;
  std::vector<std::unique_ptr<btDefaultMotionState>> motions_;std::vector<std::unique_ptr<btRigidBody>> bodies_;btRigidBody* body_;
  Clock::time_point last_tick_,last_output_,last_lidar_,command_start_,pending_at_;
  std::string lidar_layout_;std::vector<LidarSource> lidars_;std::vector<sensor_msgs::msg::PointCloud2> pending_;
  bool has_pending_{false};std::ofstream log_;rclcpp::TimerBase::SharedPtr timer_;std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_srv_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr command_sub_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<DynamicsNode>());}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
