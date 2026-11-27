// ROS 2 transport for the pinned, unchanged RotorS Lee controller.
// Gazebo owns all rigid bodies, rotor lag, force application and sensors.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <actuator_msgs/msg/actuators.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <rotors_control/lee_position_controller.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <sstream>

using Steady = std::chrono::steady_clock;
struct TimedCommand {
  int64_t offset_ns;
  mav_msgs::EigenTrajectoryPoint point;
};

class RmfController : public rclcpp::Node {
 public:
  RmfController() : Node("gbplanner_rmf_controller") {
    frame_ = declare_parameter("frame_id", "map");
    body_frame_ = declare_parameter("robot_frame", "base_link");
    timeout_ = declare_parameter("odom_timeout", .5);
    clock_order_tolerance_ = declare_parameter("clock_order_tolerance", .1);
    max_speed_ = declare_parameter("motor_max_speed", 2000.);
    takeoff_height_ = declare_parameter("flight.takeoff_height", 1.);
    takeoff_duration_ = declare_parameter("flight.takeoff_duration", 4.);
    observation_height_ = declare_parameter("flight.takeoff_observation_height", 0.);
    observation_duration_ = declare_parameter("flight.takeoff_observation_duration", 1.);
    return_duration_ = declare_parameter("flight.takeoff_return_duration", 2.);
    landing_speed_ = declare_parameter("flight.landing_speed", .2);
    landing_max_descent_ = declare_parameter("flight.landing_max_descent", 5.);
    for (const auto value : {takeoff_height_, takeoff_duration_, observation_duration_,
                            return_duration_, landing_speed_, landing_max_descent_})
      if (!std::isfinite(value) || value <= 0.) throw std::invalid_argument("RMF flight profile");
    if (!std::isfinite(observation_height_) || observation_height_ < 0.)
      throw std::invalid_argument("RMF observation height");
    takeoff_total_duration_ = takeoff_duration_ +
        (observation_height_ > 0. ? observation_duration_+return_duration_ : 0.);
    auto vector = [this](const std::string& name, std::vector<double> defaults) {
      const auto values = declare_parameter(name, defaults);
      if (values.size() != 3 || !std::all_of(values.begin(), values.end(),
          [](double v) { return std::isfinite(v); })) throw std::invalid_argument(name);
      return Eigen::Vector3d(values[0], values[1], values[2]);
    };
    auto& vehicle = lee_.vehicle_parameters_;
    auto& gains = lee_.controller_parameters_;
    vehicle.mass_ = declare_parameter("mass", .5265);
    const auto inertia = vector("inertia", {.01, .01, .01});
    vehicle.inertia_ = inertia.asDiagonal();
    gains.position_gain_ = vector("position_gain", {2., 2., 2.});
    gains.velocity_gain_ = vector("velocity_gain", {1.2, 1.2, 1.2});
    gains.attitude_gain_ = vector("attitude_gain", {.7, .7, .015});
    gains.angular_rate_gain_ = vector("angular_rate_gain", {.12, .12, .012});
    const auto angles = declare_parameter<std::vector<double>>("rotor_angles",
        {-.785398, 2.35619, .78539, -2.35619});
    const auto directions = declare_parameter<std::vector<int64_t>>("rotor_directions", {1, 1, -1, -1});
    const double arm = declare_parameter("arm_length", .1);
    const double force = declare_parameter("force_constant", 8.54858e-6);
    const double moment = declare_parameter("moment_constant", .016);
    if (angles.size() != 4 || directions.size() != 4 || (inertia.array() <= 0).any() ||
        !std::isfinite(vehicle.mass_) || vehicle.mass_ <= 0 ||
        !std::isfinite(timeout_) || timeout_ <= 0 ||
        !std::isfinite(clock_order_tolerance_) || clock_order_tolerance_ < 0 || clock_order_tolerance_ > .1 ||
        !std::isfinite(max_speed_) || max_speed_ <= 0 ||
        !std::isfinite(arm) || arm <= 0 || !std::isfinite(force) || force <= 0 ||
        !std::isfinite(moment) || moment <= 0) throw std::invalid_argument("RMF vehicle parameters");
    vehicle.rotor_configuration_.rotors.clear();
    for (size_t i = 0; i < 4; ++i) {
      if (!std::isfinite(angles[i]) || std::abs(directions[i]) != 1)
        throw std::invalid_argument("RMF rotor direction/angle");
      vehicle.rotor_configuration_.rotors.emplace_back(angles[i], arm, force, moment, directions[i]);
    }
    lee_.InitializeParameters();
    motors_ = create_publisher<actuator_msgs::msg::Actuators>("rmf/command/motor_speed", 10);
    states_ = create_publisher<std_msgs::msg::String>("rmf_controller_state", 10);
    odometry_ = create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { observe(*m); });
    trajectory_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
        "command/trajectory", 10,
        [this](trajectory_msgs::msg::MultiDOFJointTrajectory::ConstSharedPtr m) { receive(*m); });
    status_ = create_service<std_srvs::srv::Trigger>("rmf_controller_state",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
          out->success = true; out->message = state();
        });
    disarm_ = create_service<std_srvs::srv::Trigger>("rmf/disarm",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
          commands_.clear(); flight_state_ = "IDLE"; flight_reason_ = "explicit_disarm";
          mode_ = "DISARMED"; publish(Eigen::Vector4d::Zero());
          out->success = true; out->message = "Trajectory cleared; zero motor command sent";
        });
    takeoff_ = create_service<std_srvs::srv::Trigger>("rmf/takeoff",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
          ++takeoff_requests_;
          out->success = beginTakeoff(); out->message = flight_reason_;
        });
    land_ = create_service<std_srvs::srv::Trigger>("rmf/land",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
          ++landing_requests_;
          out->success = beginLanding(); out->message = flight_reason_;
        });
    land_compatibility_ = create_service<std_srvs::srv::Empty>("land_srv",
        [this](std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) {
          ++landing_requests_;
          // Original request has no response payload. Physical completion is
          // separately observable; never report a transport ACK as touchdown.
          if (!beginLanding()) RCLCPP_WARN(get_logger(), "Landing request rejected: %s", flight_reason_.c_str());
        });
    hold_ = create_service<std_srvs::srv::Trigger>("rmf/hold",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
          // This explicitly interrupts takeoff or exploration. Landing and
          // fault ownership cannot be cleared by a delayed mission hold.
          out->success = fresh() && (flight_state_ == "TAKING_OFF" || flight_state_ == "AIRBORNE");
          if (out->success) { holdPosition(); flight_state_ = "AIRBORNE"; flight_reason_ = "explicit_hold"; }
          out->message = out->success ? flight_reason_ : "hold_requires_fresh_takeoff_or_airborne_state";
        });
    contact_ = create_subscription<ros_gz_interfaces::msg::Contacts>("rmf/contact/base", rclcpp::SensorDataQoS(),
        [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr m) { contact(*m); });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      const auto now = get_clock()->now().nanoseconds();
      const bool simulated = get_parameter("use_sim_time").as_bool();
      const auto receipt_age = std::chrono::duration<double>(Steady::now()-last_receipt_).count();
      if (now < clock_high_water_) {
        commands_.clear(); flight_state_ = "FAULT"; flight_reason_ = "clock_reversed";
        mode_ = "CLOCK_REVERSED"; publish(Eigen::Vector4d::Zero());
      } else if (last_receipt_ != Steady::time_point{}) {
        // A frozen physics clock must retain its last motor input. Cutting
        // thrust on wall delay causes a fall when simulation resumes, even
        // if its acquisition-time observation is still only milliseconds old.
        // Advancing simulation without fresh odometry still trips the same
        // .5 s source timeout; hardware keeps the wall receipt watchdog.
        const bool expired = simulated ? now-last_stamp_ > timeout_*1.e9 : receipt_age > timeout_;
        if (expired) {
          mode_ = "WAITING_ODOMETRY"; publish(Eigen::Vector4d::Zero());
        } else if (simulated && receipt_age > timeout_) {
          mode_ = "SIMULATION_WAIT";
        }
      }
      std_msgs::msg::String out; out.data = state(); states_->publish(out);
    });
  }

  void stopMotors() { publish(Eigen::Vector4d::Zero()); }

 private:
  static int64_t ns(const builtin_interfaces::msg::Time& t) {
    return int64_t(t.sec)*1000000000LL+t.nanosec;
  }
  static Eigen::Vector3d vector(const geometry_msgs::msg::Vector3& v) { return {v.x, v.y, v.z}; }
  static bool finite(const geometry_msgs::msg::Twist& v) {
    return vector(v.linear).allFinite() && vector(v.angular).allFinite();
  }

  bool fresh() {
    const auto now = get_clock()->now().nanoseconds();
    return last_stamp_ > 0 && now >= clock_high_water_ &&
        now-last_stamp_ <= timeout_*1.e9 && last_stamp_-now <= clock_order_tolerance_*1.e9 &&
        (get_parameter("use_sim_time").as_bool() ||
         std::chrono::duration<double>(Steady::now()-last_receipt_).count() <= timeout_);
  }

  double yaw() const {
    const auto& q = odom_.orientation;
    return std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
  }

  bool supported(int64_t stamp) const {
    return support_stamp_ > 0 && stamp-support_stamp_ <= 100000000LL && support_stamp_-stamp <= 100000000LL;
  }

  void contact(const ros_gz_interfaces::msg::Contacts& message) {
    const int64_t stamp = ns(message.header.stamp);
    if (stamp <= contact_stamp_ || last_stamp_ <= 0 || std::abs(stamp-last_stamp_) > 100000000LL) return;
    contact_stamp_ = stamp;
    bool support = false, outside_support = false;
    // Native contact positions are world coordinates. The retained source
    // body is .25 m wide and .113 m tall. Only contact at its lower surface
    // with an upright airframe is evidence of support, not a side/roof strike.
    const bool upright = odom_.orientation.toRotationMatrix()(2,2) >= std::cos(M_PI/6.);
    for (const auto& pair : message.contacts) {
      // A contact without a measured position cannot establish support.
      if (pair.positions.empty()) outside_support = true;
      for (const auto& p : pair.positions) {
        const Eigen::Vector3d local = odom_.orientation.conjugate() * (Eigen::Vector3d(p.x,p.y,p.z)-odom_.position);
        if (upright && local.allFinite() && std::abs(local.x()) <= .145 && std::abs(local.y()) <= .145 &&
            std::abs(local.z()+.0565) <= .02) support = true;
        else outside_support = true;
      }
    }
    // A supporting point must not hide a simultaneous side/roof strike.
    if (outside_support) support_stamp_ = 0;
    else if (support) support_stamp_ = stamp;
    if (outside_support && (flight_state_ == "TAKING_OFF" || flight_state_ == "LANDING"))
      faultHold("contact_outside_support_surface");
  }

  void holdPosition() {
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = odom_.position; point.yaw = yaw();
    commands_ = {{0,point}}; origin_ns_ = last_stamp_; command_index_ = 0; desired_ = point;
  }

  void faultHold(const std::string& reason) {
    holdPosition(); flight_state_ = "FAULT"; flight_reason_ = reason;
  }

  bool beginTakeoff() {
    if (!fresh()) { flight_reason_ = "takeoff_requires_fresh_odometry"; return false; }
    if (!(flight_state_ == "GROUNDED" || flight_state_ == "LANDED") ||
        !supported(last_stamp_) || odom_.velocity.norm() > .05 || motor_speeds_.maxCoeff() > 1.) {
      flight_reason_ = "takeoff_requires_observed_ground_rest"; return false;
    }
    flight_origin_ = odom_.position; flight_yaw_ = yaw(); flight_start_ = last_stamp_;
    flight_state_ = "TAKING_OFF"; flight_reason_ = "takeoff_accepted"; touchdown_confirmed_ = false;
    support_since_ = 0;
    commands_.clear();
    auto append_vertical = [this](double start, double end, double offset, double duration) {
      const int count = static_cast<int>(std::ceil(duration/.05));
      const double height = end-start;
      for (int i=commands_.empty()?0:1;i<=count;++i) {
        const double u=static_cast<double>(i)/count;
        mav_msgs::EigenTrajectoryPoint point;
        point.position_W=flight_origin_; point.yaw=flight_yaw_;
        point.position_W.z() += start+height*(10*std::pow(u,3)-15*std::pow(u,4)+6*std::pow(u,5));
        point.velocity_W.z() = height*(30*u*u-60*std::pow(u,3)+30*std::pow(u,4))/duration;
        point.acceleration_W.z() = height*(60*u-180*u*u+120*std::pow(u,3))/(duration*duration);
        commands_.push_back({static_cast<int64_t>(std::llround((offset+u*duration)*1.e9)),point});
      }
    };
    const double peak = takeoff_height_+observation_height_;
    append_vertical(0.,peak,0.,takeoff_duration_);
    if (observation_height_ > 0.) {
      // Observe from above the final hover point, then return along the same
      // takeoff column to cover OS0's upper near-field blind area. This does
      // not fill map cells or change the subsequent planning bounds.
      append_vertical(peak,peak,takeoff_duration_,observation_duration_);
      append_vertical(peak,takeoff_height_,takeoff_duration_+observation_duration_,return_duration_);
    }
    origin_ns_ = flight_start_; command_index_ = 0;
    return true;
  }

  bool beginLanding() {
    if (flight_state_ == "LANDING" || flight_state_ == "LANDED") return true;
    if (!fresh()) { flight_reason_ = "landing_requires_fresh_odometry"; return false; }
    // land_srv has no home coordinates: descend at the current XY/yaw. The
    // PCI cancels and confirms relinquishing its trajectory before handoff.
    flight_origin_ = odom_.position; flight_yaw_ = yaw(); flight_start_ = last_stamp_;
    flight_state_ = "LANDING"; flight_reason_ = "landing_accepted"; touchdown_confirmed_ = false;
    support_since_ = 0; holdPosition();
    return true;
  }

  bool updateFlight(int64_t stamp) {
    if (flight_state_ == "LANDING") {
      const double elapsed = std::max(0.,(stamp-flight_start_)*1.e-9);
      if (elapsed > landing_max_descent_/landing_speed_+5.) {
        faultHold("landing_touchdown_timeout"); return true;
      }
      if (supported(stamp) && support_stamp_ >= flight_start_) {
        if (support_since_ == 0) { support_since_ = stamp; holdPosition(); }
        desired_ = commands_.front().point;
        if (stamp-support_since_ >= 200000000LL && odom_.velocity.norm() <= .05 && odom_.angular_velocity.norm() <= .05) {
          commands_.clear(); flight_state_ = "LANDED"; flight_reason_ = "native_support_and_rest_confirmed";
          touchdown_confirmed_ = true; mode_ = "LANDED"; publish(Eigen::Vector4d::Zero()); return false;
        }
      } else {
        support_since_ = 0;
        // One second acceleration ramp, then constant sink rate. No target
        // ground height is invented: actual supporting contact ends descent.
        const double descent=landing_speed_*(elapsed<1.? .5*elapsed*elapsed : elapsed-.5);
        if (descent > landing_max_descent_) faultHold("landing_descent_limit_without_touchdown");
        else {
          desired_ = mav_msgs::EigenTrajectoryPoint();
          desired_.position_W = flight_origin_; desired_.position_W.z() -= descent;
          desired_.yaw = flight_yaw_; desired_.velocity_W.z() = -landing_speed_*std::min(1.,elapsed);
          desired_.acceleration_W.z() = elapsed<1.? -landing_speed_ : 0.;
        }
      }
    } else if (flight_state_ == "TAKING_OFF") {
      const double elapsed=(stamp-flight_start_)*1.e-9;
      // Own the value; an auto Eigen expression would retain a reference to
      // the temporary offset after that temporary has been destroyed.
      const Eigen::Vector3d target=flight_origin_+Eigen::Vector3d(0,0,takeoff_height_);
      const double angle=std::atan2(std::sin(yaw()-flight_yaw_),std::cos(yaw()-flight_yaw_));
      if (elapsed>=takeoff_total_duration_ && (odom_.position-target).norm()<=.05 && std::abs(angle)<=.1 &&
          odom_.velocity.norm()<=.05 && odom_.angular_velocity.norm()<=.05) {
        flight_state_="AIRBORNE";flight_reason_="takeoff_endpoint_observed";
      } else if (elapsed>takeoff_total_duration_+10.) faultHold("takeoff_endpoint_timeout");
    }
    if ((flight_state_ == "TAKING_OFF" || flight_state_ == "LANDING") &&
        (odom_.position-flight_origin_).head<2>().norm() > .15)
      faultHold("vertical_flight_left_original_path_tube");
    return true;
  }

  void receive(const trajectory_msgs::msg::MultiDOFJointTrajectory& m) {
    try {
      const auto now = get_clock()->now().nanoseconds();
      if (flight_state_ == "TAKING_OFF" || flight_state_ == "LANDING" || flight_state_ == "LANDED" || flight_state_ == "FAULT")
        throw std::invalid_argument("flight mode owns output: "+flight_state_);
      if (m.header.frame_id != frame_ || m.points.empty() || m.points.size() > 20000 ||
          m.joint_names.size() != 1 || m.joint_names.front() != body_frame_)
        throw std::invalid_argument("trajectory frame/joint/size");
      if (last_stamp_ <= 0 || now-last_stamp_ > timeout_*1.e9 || last_stamp_-now > clock_order_tolerance_*1.e9 ||
          (!get_parameter("use_sim_time").as_bool() &&
           std::chrono::duration<double>(Steady::now()-last_receipt_).count() > timeout_))
        throw std::invalid_argument("trajectory without fresh odometry");
      const int64_t origin = ns(m.header.stamp) == 0 ? now : ns(m.header.stamp);
      if (origin-now > timeout_*1.e9) throw std::invalid_argument("future trajectory origin");
      std::vector<TimedCommand> candidate;
      int64_t previous = -1;
      for (const auto& p : m.points) {
        const int64_t offset = int64_t(p.time_from_start.sec)*1000000000LL+p.time_from_start.nanosec;
        if (offset <= previous || p.transforms.size() != 1 || p.velocities.size() > 1 || p.accelerations.size() > 1)
          throw std::invalid_argument("trajectory sample time/shape");
        const auto& t = p.transforms.front();
        Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
        mav_msgs::EigenTrajectoryPoint point;
        point.position_W = vector(t.translation);
        if (!point.position_W.allFinite() || !q.coeffs().allFinite() || q.norm() < 1.e-8)
          throw std::invalid_argument("nonfinite trajectory pose");
        q.normalize();
        point.yaw = std::atan2(2*(q.w()*q.z()+q.x()*q.y()), 1-2*(q.y()*q.y()+q.z()*q.z()));
        if (!p.velocities.empty()) {
          if (!finite(p.velocities.front())) throw std::invalid_argument("nonfinite trajectory velocity");
          point.velocity_W = vector(p.velocities.front().linear);
          point.yaw_rate = p.velocities.front().angular.z;
        }
        if (!p.accelerations.empty()) {
          if (!finite(p.accelerations.front())) throw std::invalid_argument("nonfinite trajectory acceleration");
          point.acceleration_W = vector(p.accelerations.front().linear);
        }
        candidate.push_back({offset, point}); previous = offset;
      }
      commands_ = std::move(candidate); origin_ns_ = origin; command_index_ = 0;
      flight_state_ = "AIRBORNE"; flight_reason_ = "external_trajectory";
      desired_ = commands_.front().point;
      ++accepted_commands_; mode_ = "TRACKING";
      RCLCPP_INFO(get_logger(), "RMF trajectory %zu: %zu points, %.3f simulated seconds",
          accepted_commands_, commands_.size(), commands_.back().offset_ns*1.e-9);
    } catch (const std::invalid_argument& e) {
      ++rejected_commands_;
      RCLCPP_WARN(get_logger(), "Rejected RMF trajectory: %s", e.what());
    }
  }

  void observe(const nav_msgs::msg::Odometry& m) {
    const auto now = get_clock()->now().nanoseconds();
    const auto stamp = ns(m.header.stamp);
    if (now < clock_high_water_) {
      // A simulation reset invalidates the old trajectory. Require a new
      // command after the new clock has caught up; never replay old motor data.
      commands_.clear(); flight_state_ = "FAULT"; flight_reason_ = "clock_reversed";
      mode_ = "CLOCK_REVERSED"; publish(Eigen::Vector4d::Zero());
      ++rejected_odometry_; return;
    }
    clock_high_water_ = now;
    const auto& p = m.pose.pose.position;
    const auto& q = m.pose.pose.orientation;
    rotors_control::EigenOdometry observed;
    observed.position = {p.x, p.y, p.z};
    observed.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    // The external adapter publishes body-frame twist, as RotorS expects.
    observed.velocity = vector(m.twist.twist.linear);
    observed.angular_velocity = vector(m.twist.twist.angular);
    if (m.header.frame_id != frame_ || m.child_frame_id != body_frame_ || stamp <= last_stamp_ || stamp <= 0 ||
        // Clock and odometry use separate DDS channels. Match the existing
        // executor's bounded ordering allowance; never refresh on old stamps.
        stamp-now > clock_order_tolerance_*1.e9 || now-stamp > timeout_*1.e9 || !observed.position.allFinite() ||
        !observed.orientation.coeffs().allFinite() || observed.orientation.norm() < 1.e-8 ||
        !observed.velocity.allFinite() || !observed.angular_velocity.allFinite()) {
      ++rejected_odometry_; return;
    }
    observed.orientation.normalize(); odom_ = observed;
    last_stamp_ = stamp; last_receipt_ = Steady::now(); ++accepted_odometry_;
    if (commands_.empty()) {
      if (flight_state_ == "IDLE" || flight_state_ == "GROUNDED") {
        if (supported(stamp) && odom_.velocity.norm()<=.05 && odom_.angular_velocity.norm()<=.05) {
          if (support_since_==0) support_since_=stamp;
          if (stamp-support_since_>=200000000LL) { flight_state_="GROUNDED";flight_reason_="native_support_and_rest_confirmed"; }
        } else { support_since_=0;flight_state_="IDLE"; }
      }
      mode_ = flight_state_ == "LANDED" ? "LANDED" : "IDLE";
      publish(Eigen::Vector4d::Zero()); return;
    }
    // Use acquisition time for both feedback and timed sample holding. The
    // original trajectory derivatives are preserved; no physics is integrated.
    const int64_t elapsed = std::max<int64_t>(0, stamp-origin_ns_);
    while (command_index_+1 < commands_.size() && commands_[command_index_+1].offset_ns <= elapsed)
      ++command_index_;
    desired_ = commands_[command_index_].point;
    if (!updateFlight(stamp)) return;
    lee_.SetOdometry(odom_); lee_.SetTrajectoryPoint(desired_);
    Eigen::VectorXd output; lee_.CalculateRotorVelocities(&output);
    if (output.size() != 4 || !output.allFinite()) {
      commands_.clear(); flight_state_="FAULT";flight_reason_="invalid_control";
      mode_ = "INVALID_CONTROL"; publish(Eigen::Vector4d::Zero()); return;
    }
    mode_ = command_index_+1 == commands_.size() ? "HOLDING" : "TRACKING";
    publish(output.cwiseMax(0.).cwiseMin(max_speed_));
  }

  void publish(const Eigen::Vector4d& speeds) {
    actuator_msgs::msg::Actuators message;
    message.header.stamp = get_clock()->now(); message.header.frame_id = body_frame_;
    message.velocity.assign(speeds.data(), speeds.data()+4);
    motors_->publish(message); motor_speeds_ = speeds;
  }

  std::string state() const {
    std::ostringstream s; s << std::setprecision(12);
    s << "{\"state\":\"" << mode_ << "\",\"clock\":\"ROS acquisition time\""
      << ",\"use_sim_time\":" << (get_parameter("use_sim_time").as_bool()?"true":"false")
      << ",\"accepted_commands\":" << accepted_commands_ << ",\"rejected_commands\":" << rejected_commands_
      << ",\"accepted_odometry\":" << accepted_odometry_ << ",\"rejected_odometry\":" << rejected_odometry_
      << ",\"odometry_stamp_s\":" << last_stamp_*1.e-9 << ",\"trajectory_origin_s\":" << origin_ns_*1.e-9
      << ",\"sample_index\":" << command_index_ << ",\"samples\":" << commands_.size()
      << ",\"flight\":{\"state\":\"" << flight_state_ << "\",\"reason\":\"" << flight_reason_ << "\""
      << ",\"touchdown_confirmed\":" << (touchdown_confirmed_?"true":"false")
      << ",\"takeoff_requests\":" << takeoff_requests_ << ",\"landing_requests\":" << landing_requests_
      << ",\"takeoff_height_m\":" << takeoff_height_
      << ",\"takeoff_peak_height_m\":" << takeoff_height_+observation_height_
      << ",\"takeoff_profile_duration_s\":" << takeoff_total_duration_
      << ",\"last_support_stamp_s\":" << support_stamp_*1.e-9 << '}'
      << ",\"model_mass_parameter\":" << lee_.vehicle_parameters_.mass_
      << ",\"desired_position\":[" << desired_.position_W.x() << ',' << desired_.position_W.y() << ',' << desired_.position_W.z()
      << "],\"desired_yaw\":" << desired_.yaw << ",\"position\":[" << odom_.position.x() << ',' << odom_.position.y() << ',' << odom_.position.z()
      << "],\"motor_speed_rad_s\":[" << motor_speeds_[0] << ',' << motor_speeds_[1] << ',' << motor_speeds_[2] << ',' << motor_speeds_[3] << "]}";
    return s.str();
  }

  rotors_control::LeePositionController lee_;
  rotors_control::EigenOdometry odom_;
  mav_msgs::EigenTrajectoryPoint desired_;
  std::string frame_, body_frame_, mode_{"IDLE"};
  double timeout_, max_speed_, clock_order_tolerance_;
  double takeoff_height_,takeoff_duration_,landing_speed_,landing_max_descent_,flight_yaw_{0.};
  double observation_height_,observation_duration_,return_duration_,takeoff_total_duration_;
  Eigen::Vector3d flight_origin_{Eigen::Vector3d::Zero()};
  std::string flight_state_{"IDLE"},flight_reason_{"waiting_for_observed_support"};
  bool touchdown_confirmed_{false};
  int64_t flight_start_{0},support_stamp_{0},contact_stamp_{0},support_since_{0};
  size_t takeoff_requests_{0},landing_requests_{0};
  int64_t last_stamp_{0}, origin_ns_{0}, clock_high_water_{0};
  Steady::time_point last_receipt_{};
  size_t command_index_{0}, accepted_commands_{0}, rejected_commands_{0}, accepted_odometry_{0}, rejected_odometry_{0};
  std::vector<TimedCommand> commands_;
  Eigen::Vector4d motor_speeds_{Eigen::Vector4d::Zero()};
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motors_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr states_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_,land_,hold_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr land_compatibility_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

static std::atomic_bool stopping{false};
static void onSignal(int) { stopping.store(true); }
int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal); std::signal(SIGTERM, onSignal);
  auto node = std::make_shared<RmfController>();
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  while (rclcpp::ok() && !stopping.load()) executor.spin_once(std::chrono::milliseconds(100));
  if (rclcpp::ok()) node->stopMotors();
  executor.remove_node(node); node.reset(); rclcpp::shutdown();
  return 0;
}
