// Native Fortress IMU observations with the source RMF stochastic models.
// RotorS formulas: Copyright 2015 Fadri Furrer, Michael Burri, Mina Kamel,
// Janosch Nikolic, Markus Achtelik; 2016 Geoffrey Hunter. Apache-2.0.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Source: ntnu-arl/rotors_simulator e8c6f170de51620f572664656f10603334bf5057,
// rotors_gazebo_plugins/src/gazebo_imu_plugin.cpp.
// Hector model reference: tu-darmstadt-ros-pkg/hector_gazebo
// de947fb1e253b114d259e8690456792d91b93347 (0.5.4), BSD-3-Clause.
// Original Hector dependency was not pinned by the RMF/lidar repository.
// Hector formulas: Copyright (c) 2012, Johannes Meyer, TU Darmstadt.
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Flight Systems and Automatic Control group,
//   TU Darmstadt, nor the names of its contributors may be used to endorse or
//   promote products derived from this software without specific prior
//   written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// This adaptation neither subscribes to odometry nor synthesizes inertial
// measurements from poses. Only native sensor_msgs/Imu observations enter.
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <hector_gazebo_plugins/srv/set_bias.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double kPi=3.14159265358979323846;
constexpr double kDefaultGyroRandomWalk=2.*4./3600./180.*kPi;
using Vector=Eigen::Vector3d;

Vector vector(const geometry_msgs::msg::Vector3& value){return {value.x,value.y,value.z};}
void assign(geometry_msgs::msg::Vector3& out,const Vector& value){out.x=value.x();out.y=value.y();out.z=value.z();}
void assign(geometry_msgs::msg::Quaternion& out,const Eigen::Quaterniond& value){
  out.w=value.w();out.x=value.x();out.y=value.y();out.z=value.z();
}

struct RotorParameters {
  double gyro_density{0.},gyro_walk{kDefaultGyroRandomWalk},gyro_tau{1000.},gyro_turn_on{0.};
  double accel_density{0.},accel_walk{0.},accel_tau{300.},accel_turn_on{0.};
};
struct HectorParameters {
  Vector accel_offset{Vector::Zero()},rate_offset{Vector::Zero()};
  Vector accel_scale{Vector::Ones()},rate_scale{Vector::Ones()};
  Vector accel_drift{Vector::Constant(.005)},accel_noise{Vector::Constant(.005)};
  Vector rate_drift{Vector::Constant(.005)},rate_noise{Vector::Constant(.005)};
  Vector accel_frequency{Vector::Constant(1./3600.)},rate_frequency{Vector::Constant(1./3600.)};
  double yaw_offset{0.},yaw_scale{1.},yaw_drift{0.},yaw_noise{0.},yaw_frequency{1./3600.};
};
struct Stream {
  std::string name,frame,input,output;
  bool hector{false};double nominal_period{.002};
  RotorParameters rotor;HectorParameters hector_parameters;
  // RotorS uses one default_random_engine per plugin. Preserve independent
  // streams and the turn-on draw order; cross-library bit parity is not claimed.
  std::default_random_engine engine;
  std::normal_distribution<double> normal{0.,1.};
  Vector gyro_bias{Vector::Zero()},accel_bias{Vector::Zero()};
  Vector gyro_turn_on{Vector::Zero()},accel_turn_on{Vector::Zero()};
  Vector hector_accel_variance{Vector::Zero()},hector_rate_variance{Vector::Zero()};
  double yaw_bias{0.};
  int64_t last_stamp{0};
  std::uint64_t received{0},published{0},rejected{0},interval_gaps{0};
  double last_interval{0.},maximum_interval{0.};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr bias_publisher;
  std::string calibration_service,bias_topic,last_request{"none"};
  bool last_request_accepted{true};
  std::uint64_t calibration_calls{0},set_accel_bias_calls{0},set_rate_bias_calls{0};
  std::uint64_t rejected_bias_requests{0},bias_published{0};
  std::uint64_t parameter_updates{0};

  Vector currentAccelBias()const{return accel_bias+hector_parameters.accel_offset;}
  Vector currentRateBias()const{return gyro_bias+hector_parameters.rate_offset;}
  double currentYawBias()const{return yaw_bias+hector_parameters.yaw_offset;}

  void initialize(){
    if(hector){
      // Hector fills these two covariance diagonals once in Load(). Only its
      // orientation covariance follows subsequent noise-parameter changes.
      hector_accel_variance=hector_parameters.accel_noise.cwiseProduct(hector_parameters.accel_noise);
      hector_rate_variance=hector_parameters.rate_noise.cwiseProduct(hector_parameters.rate_noise);
      for(int i=0;i<3;++i)accel_bias[i]=hector_parameters.accel_drift[i]*normal(engine);
      for(int i=0;i<3;++i)gyro_bias[i]=hector_parameters.rate_drift[i]*normal(engine);
      yaw_bias=hector_parameters.yaw_drift*normal(engine);
    }else{
      for(int i=0;i<3;++i){
        gyro_turn_on[i]=rotor.gyro_turn_on*normal(engine);
        accel_turn_on[i]=rotor.accel_turn_on*normal(engine);
      }
    }
  }

  void addRotorNoise(Vector& acceleration,Vector& rate,double dt){
    // Exact discrete Gauss-Markov driving covariance. The independent
    // turn-on offset remains constant; native GaussianNoiseModel would decay
    // its initial bias along with the dynamic state and is not equivalent.
    const double gyro_phi=std::exp(-dt/rotor.gyro_tau);
    const double gyro_sigma=rotor.gyro_walk*std::sqrt(-.5*rotor.gyro_tau*std::expm1(-2.*dt/rotor.gyro_tau));
    const double gyro_white=rotor.gyro_density/std::sqrt(dt);
    for(int i=0;i<3;++i){
      gyro_bias[i]=gyro_phi*gyro_bias[i]+gyro_sigma*normal(engine);
      rate[i]+=gyro_bias[i]+gyro_white*normal(engine)+gyro_turn_on[i];
    }
    const double accel_phi=std::exp(-dt/rotor.accel_tau);
    const double accel_sigma=rotor.accel_walk*std::sqrt(-.5*rotor.accel_tau*std::expm1(-2.*dt/rotor.accel_tau));
    const double accel_white=rotor.accel_density/std::sqrt(dt);
    for(int i=0;i<3;++i){
      accel_bias[i]=accel_phi*accel_bias[i]+accel_sigma*normal(engine);
      acceleration[i]+=accel_bias[i]+accel_white*normal(engine)+accel_turn_on[i];
    }
  }

  double hectorError(double& bias,double drift,double frequency,double noise,double offset,double dt){
    // Preserve Hector's dt-scaled innovation, not the RotorS sqrt(dt) law.
    bias=std::exp(-dt*frequency)*bias+dt*std::sqrt(2.*frequency)*drift*normal(engine);
    return offset+bias+noise*normal(engine);
  }
  Eigen::Quaterniond addHectorNoise(Vector& acceleration,Vector& rate,Eigen::Quaterniond& orientation,double dt,double gravity){
    const auto& p=hector_parameters;
    for(int i=0;i<3;++i)acceleration[i]=acceleration[i]*p.accel_scale[i]+
      hectorError(accel_bias[i],p.accel_drift[i],p.accel_frequency[i],p.accel_noise[i],p.accel_offset[i],dt);
    for(int i=0;i<3;++i)rate[i]=rate[i]*p.rate_scale[i]+
      hectorError(gyro_bias[i],p.rate_drift[i],p.rate_frequency[i],p.rate_noise[i],p.rate_offset[i],dt);
    // Hector updates yaw white noise but its pseudo-AHRS quaternion uses
    // current bias only, not that white noise and not integrated gyro bias.
    // yaw ScaleError is intentionally unused: the source calls update(dt),
    // never the value-scaling operator. Zero amplitudes still consume draws.
    (void)hectorError(yaw_bias,p.yaw_drift,p.yaw_frequency,p.yaw_noise,p.yaw_offset,dt);
    const Vector world_bias=orientation*currentAccelBias();
    const double yaw_error=currentYawBias();
    Eigen::Quaterniond error=Eigen::Quaterniond(std::cos(yaw_error/2.),0.,0.,std::sin(yaw_error/2.))*
      Eigen::Quaterniond(1.,.5*world_bias.y()/gravity,-.5*world_bias.x()/gravity,0.);
    error.normalize();orientation=(error*orientation).normalized();
    return error;
  }
};

class RmfImuAdapter final:public rclcpp::Node {
 public:
  RmfImuAdapter():Node("gbplanner_rmf_imu_adapter"){
    gravity_=positive("gravity_magnitude",9.81);
    const auto seed=staticParameter<int64_t>("random_seed",1);
    if(seed<0||seed>UINT32_MAX)throw std::invalid_argument("Invalid IMU random_seed");
    auto adis=rotorStream("adis","imu_link","/rmf/imu",true);
    auto ground_truth=rotorStream("ground_truth","imugt_link","/rmf/ground_truth/imu",false);
    auto os0=baseStream("os0","os0_imu","/os0_cloud_node/imu",.01);
    declareHectorParameters(*os0);
    streams_.push_back(std::move(adis));streams_.push_back(std::move(ground_truth));streams_.push_back(std::move(os0));
    if(staticParameter<bool>("smb_os1.enabled",false)){
      auto os1=baseStream("smb_os1","os1_imu","/os1_cloud_node/imu",.01);
      declareHectorParameters(*os1);
      streams_.push_back(std::move(os1));
    }
    for(auto& item:streams_){
      auto* stream=item.get();stream->engine.seed(static_cast<unsigned int>(seed));stream->initialize();
      stream->publisher=create_publisher<sensor_msgs::msg::Imu>(stream->output,rclcpp::SensorDataQoS());
      // Match either reliable or best-effort ros_gz_bridge inputs. A large
      // bounded queue preserves high-rate samples; acquisition gaps remain
      // visible in the status and are never filled from ground-truth poses.
      stream->subscription=create_subscription<sensor_msgs::msg::Imu>(stream->input,
        rclcpp::SensorDataQoS().keep_last(1000),[this,stream](sensor_msgs::msg::Imu::ConstSharedPtr raw){observe(*stream,*raw);});
      if(stream->hector)createHectorInterfaces(*stream);
    }
    const auto status_service=staticParameter<std::string>("status_service","rmf/imu_state");
    if(status_service.empty())throw std::invalid_argument("IMU status service cannot be empty");
    status_=create_service<std_srvs::srv::Trigger>(status_service,
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=true;response->message=status();
      });
    parameter_callback_=add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters){return updateHectorParameters(parameters);});
    RCLCPP_INFO(get_logger(),"Native IMU streams awaiting sensor observations; source noise is applied once");
  }
 private:
  struct HectorBinding {
    Stream* stream;
    Vector HectorParameters::*vector_member;
    double HectorParameters::*scalar_member;
    double minimum,maximum;
  };
  template<typename T>
  T staticParameter(const std::string& name,const T& fallback){
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true;
    descriptor.description="Applied at startup; changing this parameter requires restarting the node";
    return declare_parameter<T>(name,fallback,descriptor);
  }
  rcl_interfaces::msg::ParameterDescriptor hectorDescriptor(bool three_axes,double minimum,double maximum){
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description=three_axes?"Hector sensor model, independent X/Y/Z values":"Hector yaw sensor model";
    descriptor.additional_constraints="Runtime values must be finite and in ["+
      std::to_string(minimum)+", "+std::to_string(maximum)+"]"+
      (three_axes?"; exactly three double values":"; a double value")+
      "; update atomically with Hector parameters only";
    // Do not install a floating_point_range here: the source loads startup
    // values before dynamic_reconfigure and does not clamp them to cfg limits.
    return descriptor;
  }
  void declareHectorVector(Stream& s,const std::string& suffix,Vector HectorParameters::*member,
                           double fallback,double minimum,double maximum,bool nonnegative_initial){
    const auto name=s.name+"."+suffix;
    const auto values=declare_parameter<std::vector<double>>(name,{fallback,fallback,fallback},
      hectorDescriptor(true,minimum,maximum));
    if(values.size()!=3)throw std::invalid_argument("IMU parameter needs three axes: "+name);
    for(double value:values)
      if(!std::isfinite(value)||(nonnegative_initial&&value<0.))
        throw std::invalid_argument("Invalid IMU axis parameter "+name);
    s.hector_parameters.*member=Vector(values[0],values[1],values[2]);
    hector_bindings_.emplace(name,HectorBinding{&s,member,nullptr,minimum,maximum});
  }
  void declareHectorScalar(Stream& s,const std::string& suffix,double HectorParameters::*member,
                           double fallback,double minimum,double maximum,bool nonnegative_initial){
    const auto name=s.name+"."+suffix;
    const double value=declare_parameter<double>(name,fallback,hectorDescriptor(false,minimum,maximum));
    if(!std::isfinite(value)||(nonnegative_initial&&value<0.))
      throw std::invalid_argument("Invalid IMU parameter "+name);
    s.hector_parameters.*member=value;
    hector_bindings_.emplace(name,HectorBinding{&s,nullptr,member,minimum,maximum});
  }
  void declareHectorParameters(Stream& s){
    s.hector=true;
    struct Field {
      const char* name;
      Vector HectorParameters::*accel;
      Vector HectorParameters::*rate;
      double HectorParameters::*yaw;
      double vector_default,yaw_default,minimum,maximum;
      bool nonnegative_initial;
    };
    const std::array<Field,5> fields{{
      {"Offset",&HectorParameters::accel_offset,&HectorParameters::rate_offset,&HectorParameters::yaw_offset,
        0.,0.,-10.,10.,false},
      {"ScaleError",&HectorParameters::accel_scale,&HectorParameters::rate_scale,&HectorParameters::yaw_scale,
        1.,1.,0.,2.,false},
      {"Drift",&HectorParameters::accel_drift,&HectorParameters::rate_drift,&HectorParameters::yaw_drift,
        .005,0.,0.,10.,true},
      {"GaussianNoise",&HectorParameters::accel_noise,&HectorParameters::rate_noise,&HectorParameters::yaw_noise,
        .005,0.,0.,10.,true},
      {"DriftFrequency",&HectorParameters::accel_frequency,&HectorParameters::rate_frequency,&HectorParameters::yaw_frequency,
        1./3600.,1./3600.,0.,1.,true}
    }};
    // ROS 1's first callback only exports the loaded model (vector means);
    // it must not overwrite our startup arrays or the 1/3600 frequency default.
    // Later ROS 1 updates broadcast all five scalar fields to all three axes.
    // This native ROS 2 extension instead changes only the named arrays/scalars,
    // preserving unrelated parameters and each axis's independent settings.
    for(const auto& field:fields){
      declareHectorVector(s,std::string("accel.")+field.name,field.accel,field.vector_default,
        field.minimum,field.maximum,field.nonnegative_initial);
      declareHectorVector(s,std::string("rate.")+field.name,field.rate,field.vector_default,
        field.minimum,field.maximum,field.nonnegative_initial);
      declareHectorScalar(s,std::string("yaw.")+field.name,field.yaw,field.yaw_default,
        field.minimum,field.maximum,field.nonnegative_initial);
    }
  }
  rcl_interfaces::msg::SetParametersResult updateHectorParameters(const std::vector<rclcpp::Parameter>& parameters){
    rcl_interfaces::msg::SetParametersResult result;result.successful=true;
    const auto reject=[&](const std::string& reason){result.successful=false;result.reason=reason;return result;};
    const bool has_hector=std::any_of(parameters.begin(),parameters.end(),
      [this](const rclcpp::Parameter& parameter){return hector_bindings_.count(parameter.get_name())!=0;});
    if(!has_hector)return result; // In particular, leave pure use_sim_time to rclcpp.
    // Humble offers only a pre-commit callback. A mixed batch could pass here
    // and then fail another callback (notably TimeSource's use_sim_time check).
    // Restrict this transaction to our declared Hector fields, which no other
    // callback owns. Parameter services, sensor input and bias services share
    // this node's default mutually exclusive callback group.
    for(const auto& parameter:parameters)
      if(hector_bindings_.count(parameter.get_name())==0)
        return reject("Hector dynamic parameters cannot be mixed with other parameters in one transaction");
    std::set<std::string> names;
    std::map<Stream*,HectorParameters> staged;
    for(const auto& parameter:parameters){
      const auto& name=parameter.get_name();
      if(!names.insert(name).second)return reject("Duplicate Hector parameter: "+name);
      const auto& binding=hector_bindings_.at(name);
      const auto valid=[&](double value){return std::isfinite(value)&&value>=binding.minimum&&value<=binding.maximum;};
      auto& candidate=staged.emplace(binding.stream,binding.stream->hector_parameters).first->second;
      if(binding.vector_member){
        if(parameter.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
          return reject("Hector parameter requires a double array: "+name);
        const auto& values=parameter.as_double_array();
        if(values.size()!=3||!std::all_of(values.begin(),values.end(),valid))
          return reject("Hector parameter needs three finite axes within its runtime range: "+name);
        candidate.*(binding.vector_member)=Vector(values[0],values[1],values[2]);
      }else{
        if(parameter.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE||!valid(parameter.as_double()))
          return reject("Hector parameter requires a finite double within its runtime range: "+name);
        candidate.*(binding.scalar_member)=parameter.as_double();
      }
    }
    // Commit only after every candidate passes. No drift/error reset, random
    // draw, clock/sample change or publication occurs during configuration.
    for(const auto& candidate:staged){
      candidate.first->hector_parameters=candidate.second;
      ++candidate.first->parameter_updates;
    }
    ++hector_parameter_update_batches_;
    return result;
  }
  void createHectorInterfaces(Stream& s){
    // The default mutually exclusive group serializes these callbacks with
    // observe(). No sample, timestamp or random draw is produced by a reset.
    s.calibration_service=staticParameter<std::string>(s.name+".calibration_service",s.output+"/calibrate");
    s.bias_topic=staticParameter<std::string>(s.name+".bias_topic",s.output+"/bias");
    if(!s.bias_topic.empty())
      s.bias_publisher=create_publisher<sensor_msgs::msg::Imu>(s.bias_topic,rclcpp::SensorDataQoS());
    if(!s.calibration_service.empty())
      calibration_services_.push_back(create_service<std_srvs::srv::Empty>(s.calibration_service,
        [&s](std::shared_ptr<std_srvs::srv::Empty::Request>,std::shared_ptr<std_srvs::srv::Empty::Response>){
          // Hector's calibrate is rateModel.reset(0), not an estimator and
          // not a reset of fixed offset, acceleration, yaw, time or the RNG.
          s.gyro_bias.setZero();++s.calibration_calls;
          s.last_request="calibrate";s.last_request_accepted=true;
        }));
    for(bool acceleration:{true,false}){
      const auto name=s.output+(acceleration?"/set_accel_bias":"/set_rate_bias");
      bias_services_.push_back(create_service<hector_gazebo_plugins::srv::SetBias>(name,
        [this,&s,acceleration](std::shared_ptr<hector_gazebo_plugins::srv::SetBias::Request> request,
                              std::shared_ptr<hector_gazebo_plugins::srv::SetBias::Response>){
          if(acceleration)++s.set_accel_bias_calls;else ++s.set_rate_bias_calls;
          s.last_request=acceleration?"set_accel_bias":"set_rate_bias";
          const Vector requested=vector(request->bias);
          s.last_request_accepted=requested.allFinite();
          if(!s.last_request_accepted){
            ++s.rejected_bias_requests;
            RCLCPP_WARN(get_logger(),"Ignoring nonfinite %s for IMU %s",s.last_request.c_str(),s.name.c_str());
            return;
          }
          // SetBias resets current_drift, not drift+offset. Keep fixed offsets
          // and sample white noise only in observe(), as the source does.
          if(acceleration)s.accel_bias=requested;else s.gyro_bias=requested;
        }));
    }
  }
  double nonnegative(const std::string& name,double fallback){
    const double v=staticParameter<double>(name,fallback);
    if(!std::isfinite(v)||v<0)throw std::invalid_argument("Invalid IMU parameter "+name);
    return v;
  }
  double positive(const std::string& name,double fallback){
    const double v=nonnegative(name,fallback);
    if(v==0)throw std::invalid_argument("Invalid zero IMU parameter "+name);
    return v;
  }
  std::unique_ptr<Stream> baseStream(const std::string& name,const std::string& frame,const std::string& topic,double period){
    auto s=std::make_unique<Stream>();s->name=name;
    s->frame=staticParameter<std::string>(name+".frame_id",frame);
    s->input=staticParameter<std::string>(name+".input_topic","/rmf/imu/"+name+"/raw");
    s->output=staticParameter<std::string>(name+".output_topic",topic);
    s->nominal_period=positive(name+".nominal_period",period);
    if(s->frame.empty()||s->input.empty()||s->output.empty()||
       get_node_topics_interface()->resolve_topic_name(s->input)==
       get_node_topics_interface()->resolve_topic_name(s->output))throw std::invalid_argument("Invalid IMU frame/topics");
    return s;
  }
  std::unique_ptr<Stream> rotorStream(const std::string& name,const std::string& frame,const std::string& topic,bool adis){
    auto s=baseStream(name,frame,topic,.002);auto& p=s->rotor;const auto ns=name+".";
    p.gyro_density=nonnegative(ns+"gyroscope_noise_density",adis?.0003394:0.);
    // The original macro's gyroscopeRandomWalk is ignored by its plugin.
    // Keep the plugin default even on the nominal ground-truth output.
    p.gyro_walk=nonnegative(ns+"gyroscope_random_walk",kDefaultGyroRandomWalk);
    p.gyro_tau=positive(ns+"gyroscope_bias_correlation_time",1000.);
    p.gyro_turn_on=nonnegative(ns+"gyroscope_turn_on_bias_sigma",adis?.0087:0.);
    p.accel_density=nonnegative(ns+"accelerometer_noise_density",adis?.004:0.);
    p.accel_walk=nonnegative(ns+"accelerometer_random_walk",adis?.006:0.);
    p.accel_tau=positive(ns+"accelerometer_bias_correlation_time",300.);
    p.accel_turn_on=nonnegative(ns+"accelerometer_turn_on_bias_sigma",adis?.1960:0.);
    return s;
  }
  void observe(Stream& s,const sensor_msgs::msg::Imu& raw){
    ++s.received;
    const int64_t stamp=int64_t(raw.header.stamp.sec)*1000000000LL+raw.header.stamp.nanosec;
    Vector acceleration=vector(raw.linear_acceleration),rate=vector(raw.angular_velocity);
    Eigen::Quaterniond orientation(raw.orientation.w,raw.orientation.x,raw.orientation.y,raw.orientation.z);
    if(stamp<=0||stamp<=s.last_stamp||!acceleration.allFinite()||!rate.allFinite()||
       !orientation.coeffs().allFinite()||orientation.norm()<1e-8){++s.rejected;return;}
    // No wall-time or odometry-based replacement for missing sensor samples.
    const double dt=s.last_stamp?(stamp-s.last_stamp)*1e-9:s.nominal_period;
    if(s.last_stamp&&dt>1.5*s.nominal_period)++s.interval_gaps;
    s.last_stamp=stamp;s.last_interval=dt;s.maximum_interval=std::max(s.maximum_interval,dt);
    orientation.normalize();
    sensor_msgs::msg::Imu out=raw;out.header.frame_id=s.frame;
    Eigen::Quaterniond orientation_error=Eigen::Quaterniond::Identity();
    out.angular_velocity_covariance.fill(0.);out.linear_acceleration_covariance.fill(0.);out.orientation_covariance.fill(0.);
    if(s.hector){
      orientation_error=s.addHectorNoise(acceleration,rate,orientation,dt,gravity_);
      const auto& p=s.hector_parameters;
      for(int i=0;i<3;++i){
        out.angular_velocity_covariance[4*i]=s.hector_rate_variance[i];
        out.linear_acceleration_covariance[4*i]=s.hector_accel_variance[i];
      }
      out.orientation_covariance[0]=p.accel_noise.x()*p.accel_noise.x()/(gravity_*gravity_);
      out.orientation_covariance[4]=p.accel_noise.y()*p.accel_noise.y()/(gravity_*gravity_);
      out.orientation_covariance[8]=p.yaw_noise*p.yaw_noise;
    }else{
      s.addRotorNoise(acceleration,rate,dt);
      // The source publishes continuous density squared, not discrete sample
      // variance or total bias uncertainty, and marks all orientation entries
      // -1 although it carries the sensor orientation. Preserve that contract.
      out.orientation_covariance.fill(-1.);
      for(int i=0;i<3;++i){
        out.angular_velocity_covariance[4*i]=s.rotor.gyro_density*s.rotor.gyro_density;
        out.linear_acceleration_covariance[4*i]=s.rotor.accel_density*s.rotor.accel_density;
      }
    }
    assign(out.linear_acceleration,acceleration);assign(out.angular_velocity,rate);assign(out.orientation,orientation);
    s.publisher->publish(out);++s.published;
    if(s.bias_publisher){
      sensor_msgs::msg::Imu bias;bias.header=out.header;
      assign(bias.linear_acceleration,s.currentAccelBias());assign(bias.angular_velocity,s.currentRateBias());
      // This is the same error quaternion used above, not the full attitude.
      // Hector leaves all three bias covariance arrays at their zero defaults.
      assign(bias.orientation,orientation_error);
      s.bias_publisher->publish(bias);++s.bias_published;
    }
  }
  void writeHectorParameters(std::ostream& out,const Stream& stream)const{
    out<<'{';bool first_group=true;
    for(const auto* group:{"accel","rate","yaw"}){
      if(!first_group)out<<',';first_group=false;
      out<<std::quoted(group)<<":{";bool first_field=true;
      const std::string prefix=stream.name+"."+group+".";
      for(const auto& entry:hector_bindings_){
        if(entry.second.stream!=&stream||entry.first.compare(0,prefix.size(),prefix)!=0)continue;
        if(!first_field)out<<',';first_field=false;
        out<<std::quoted(entry.first.substr(prefix.size()))<<':';
        if(entry.second.vector_member){
          const Vector& value=stream.hector_parameters.*(entry.second.vector_member);
          out<<'['<<value.x()<<','<<value.y()<<','<<value.z()<<']';
        }else out<<(stream.hector_parameters.*(entry.second.scalar_member));
      }
      out<<'}';
    }
    out<<'}';
  }
  std::string status()const{
    std::ostringstream out;out<<std::setprecision(std::numeric_limits<double>::max_digits10)
      <<"{\"measurement_source\":\"native_gazebo_imu\",\"hector_parameter_update_batches\":"
      <<hector_parameter_update_batches_<<",\"streams\":{";
    bool first=true;
    for(const auto& s:streams_){
      if(!first)out<<',';first=false;
      out<<'"'<<s->name<<"\":{\"received\":"<<s->received<<",\"published\":"<<s->published
         <<",\"rejected\":"<<s->rejected<<",\"source_stamp_s\":"<<s->last_stamp*1e-9
         <<",\"last_interval_s\":"<<s->last_interval<<",\"maximum_interval_s\":"<<s->maximum_interval
         <<",\"interval_gaps\":"<<s->interval_gaps<<",\"nominal_period_s\":"<<s->nominal_period;
      if(s->hector){
        const auto triple=[&](const Vector& v){out<<'['<<v.x()<<','<<v.y()<<','<<v.z()<<']';};
        out<<",\"hector\":{\"calibration_service\":"<<std::quoted(s->calibration_service)
           <<",\"set_accel_bias_service\":"<<std::quoted(s->output+"/set_accel_bias")
           <<",\"set_rate_bias_service\":"<<std::quoted(s->output+"/set_rate_bias")
           <<",\"bias_topic\":"<<std::quoted(s->bias_topic)
           <<",\"calibration_calls\":"<<s->calibration_calls
           <<",\"set_accel_bias_calls\":"<<s->set_accel_bias_calls
           <<",\"set_rate_bias_calls\":"<<s->set_rate_bias_calls
           <<",\"rejected_bias_requests\":"<<s->rejected_bias_requests
           <<",\"bias_published\":"<<s->bias_published<<",\"accel_bias\":";
        triple(s->currentAccelBias());out<<",\"rate_bias\":";triple(s->currentRateBias());
        out<<",\"yaw_bias\":"<<s->currentYawBias()<<",\"accel_drift\":";
        triple(s->accel_bias);out<<",\"rate_drift\":";triple(s->gyro_bias);
        out<<",\"yaw_drift\":"<<s->yaw_bias<<",\"last_request\":"<<std::quoted(s->last_request)
           <<",\"last_request_accepted\":"<<(s->last_request_accepted?"true":"false")
           <<",\"parameter_updates\":"<<s->parameter_updates<<",\"startup_accel_covariance\":";
        triple(s->hector_accel_variance);out<<",\"startup_rate_covariance\":";triple(s->hector_rate_variance);
        out<<",\"parameters\":";writeHectorParameters(out,*s);out<<'}';
      }
      out<<'}';
    }
    out<<"}}";return out.str();
  }
  double gravity_{9.81};std::vector<std::unique_ptr<Stream>> streams_;
  std::vector<rclcpp::Service<std_srvs::srv::Empty>::SharedPtr> calibration_services_;
  std::vector<rclcpp::Service<hector_gazebo_plugins::srv::SetBias>::SharedPtr> bias_services_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_;
  std::map<std::string,HectorBinding> hector_bindings_;
  std::uint64_t hector_parameter_update_batches_{0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
}

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<RmfImuAdapter>());}
  catch(const std::exception& error){std::cerr<<"RMF IMU adapter failed: "<<error.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
