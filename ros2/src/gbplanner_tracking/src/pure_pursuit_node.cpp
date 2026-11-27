// SMB pure-pursuit ROS 2 adapter. Algorithm provenance is recorded in SOURCE.json.
#include "gbplanner_tracking/pure_pursuit.hpp"
#include "gbplanner_tracking/ground_maneuver.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <planner_msgs/action/path_follower.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace gbplanner_tracking {
namespace {
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

bool readPose(const geometry_msgs::msg::Pose& input, Pose& pose) {
  const auto& q = input.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || norm < 1e-8) return false;
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  pose << input.position.x, input.position.y, input.position.z,
      std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return pose.allFinite();
}

std::string goalId(const rclcpp_action::GoalUUID& id) {
  std::ostringstream out;
  for (auto v : id) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(v);
  return out.str();
}
}  // namespace

class PurePursuitNode final : public rclcpp::Node {
 public:
  using Action = planner_msgs::action::PathFollower;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using Steady = std::chrono::steady_clock;

  PurePursuitNode() : Node("gbplanner_pure_pursuit") {
    clock_ = get_clock();
    Parameters p;
    p.critical_angle = declare_parameter("critical_angle", 0.52);
    p.curvature_scaling_angle = declare_parameter("curvature_scaling_angle", p.curvature_scaling_angle);
    p.dt = declare_parameter("dt", 0.05);
    p.desired_vel = declare_parameter("desired_vel", 0.5);
    p.max_lin_vel = declare_parameter("max_lin_vel", 0.5);
    p.max_ang_vel = declare_parameter("max_ang_vel", 0.5);
    p.look_ahead_distance = declare_parameter("look_ahead_distance", 0.5);
    p.look_ahead_error_margin = declare_parameter("look_ahead_error_margin", 0.2);
    p.goal_reaching_threshold = declare_parameter("goal_reaching_threshold", 0.04);
    p.preserve_corners = declare_parameter("preserve_corners", false);
    const auto steering = declare_parameter("steering_curvature", std::string("upstream_regularized"));
    if (steering == "upstream_regularized") p.steering_curvature = SteeringCurvature::UpstreamRegularized;
    else if (steering == "signed_geometric") p.steering_curvature = SteeringCurvature::SignedGeometric;
    else throw std::invalid_argument("steering_curvature must be upstream_regularized or signed_geometric");
    rotation_hold_yaw_governor_ = declare_parameter("rotation_hold_yaw_governor", false);
    fixed_frame_ = declare_parameter("fixed_frame", std::string("map"));
    robot_frame_ = declare_parameter("robot_frame", std::string("base_link"));
    yaw_threshold_ = declare_parameter("goal_yaw_threshold", 0.1);
    odom_timeout_ = declare_parameter("odom_timeout", 0.5);
    max_points_ = declare_parameter("max_points", 20000);
    if (fixed_frame_.empty() || robot_frame_.empty() || !std::isfinite(yaw_threshold_) ||
        yaw_threshold_ <= 0 || yaw_threshold_ > 3.141592653589793 ||
        !std::isfinite(odom_timeout_) || odom_timeout_ <= 0 ||
        max_points_ < 1 || max_points_ > 20000 || p.dt < 0.001) {
      throw std::invalid_argument("invalid tracking parameters");
    }
    controller_ = std::make_unique<PurePursuitController>(p);
    terminal_ = std::make_unique<TerminalPosePolicy>(p.goal_reaching_threshold, yaw_threshold_);
    alignment_ = std::make_unique<TerminalAlignmentController>(
        declare_parameter("terminal_velocity_integral_gain", 0.0));
    allow_ground_maneuver_ = declare_parameter("allow_ground_maneuver", false);
    maneuver_caps_.max_speed = declare_parameter("ground_maneuver_max_speed", .3);
    maneuver_caps_.max_yaw_rate = declare_parameter("ground_maneuver_max_yaw_rate", .4);
    maneuver_caps_.max_acceleration = declare_parameter("ground_maneuver_max_acceleration", .3);
    maneuver_caps_.max_yaw_acceleration = declare_parameter("ground_maneuver_max_yaw_acceleration", .6);
    maneuver_caps_.total_timeout = declare_parameter("ground_maneuver_max_duration", 30.);
    maneuver_caps_.max_reference_length = declare_parameter("ground_maneuver_max_path_length", 5.);
    maneuver_caps_.position_tolerance = p.goal_reaching_threshold;
    maneuver_caps_.yaw_tolerance = yaw_threshold_;
    maneuver_caps_.segment_position_tolerance=std::min(maneuver_caps_.segment_position_tolerance,p.goal_reaching_threshold);
    maneuver_caps_.segment_yaw_tolerance=std::min(maneuver_caps_.segment_yaw_tolerance,yaw_threshold_);
    maneuver_caps_.maximum_step_dt = p.dt;
    maneuver_caps_.odometry_timeout = odom_timeout_;
    // Construct once to validate local caps; goals can only tighten them.
    GroundManeuverController checked_maneuver_caps(maneuver_caps_);
    active_speed_ = std::min(p.desired_vel, p.max_lin_vel);
    commands_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    carrots_ = create_publisher<geometry_msgs::msg::PoseStamped>("carrot_point", 10);
    status_ = create_publisher<std_msgs::msg::String>("tracking_status",
        rclcpp::QoS(1).reliable().transient_local());
    odom_ = create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odometry(*m); });
    state_service_ = create_service<std_srvs::srv::Trigger>("tracking_state",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          response->success = true;
          response->message = payload();
        });
    server_ = rclcpp_action::create_server<Action>(this, "pci_output_path",
        [this](const rclcpp_action::GoalUUID& id, std::shared_ptr<const Action::Goal> goal) {
          return accept(id, *goal);
        },
        [this](std::shared_ptr<Handle> handle) {
          if (active_ != handle) return rclcpp_action::CancelResponse::REJECT;
          if (ground_maneuver_active_ && maneuver_controller_) {
            maneuver_controller_->cancel(maneuver_motion_time_);
            return rclcpp_action::CancelResponse::ACCEPT;
          }
          // Stop at the cancellation request; terminal transition follows once
          // rclcpp_action has moved the handle into CANCELING.
          alignment_->reset();
          rotation_hold_.reset();
          publishCommand(0, 0);
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<Handle> handle) { accepted(std::move(handle)); });
    timer_ = create_wall_timer(std::chrono::duration<double>(p.dt), [this]() { tick(); });
    transition("IDLE", "ready");
  }

  void stopForShutdown() {
    shutting_down_ = true;
    alignment_->reset();
    rotation_hold_.reset();
    if (active_ && ground_maneuver_active_ && maneuver_controller_)
      maneuver_controller_->abort("shutdown",maneuver_motion_time_);
    else if (active_) finish(false, "shutdown");
    else publishCommand(0, 0);
  }

  bool shutdownPending() const { return ground_maneuver_active_ && active_ != nullptr; }

  void finishShutdown() {
    if (shutdownPending()) {
      auto command=maneuver_controller_->command();
      command.status=ManeuverStatus::Failed;
      command.stop_confirmed=false;
      command.reason="shutdown_stop_unconfirmed";
      finishManeuver(command);
    } else publishCommand(0.,0.);
  }

 private:
  double odomAge() const {
    if (!have_odom_) return std::numeric_limits<double>::infinity();
    return std::chrono::duration<double>(Steady::now() - odom_received_).count();
  }

  bool simulatedClock() const { return clock_->ros_time_is_active(); }

  bool observeClock(double ros_now) const {
    if (!simulatedClock()) {
      ros_clock_high_water_ = -std::numeric_limits<double>::infinity();
      ros_clock_reversed_ = false;
      return true;
    }
    if (ros_now < ros_clock_high_water_) {
      ros_clock_reversed_ = true;
      return false;
    }
    ros_clock_high_water_ = ros_now;
    // Catching up alone does not restore authority: a newer accepted source
    // sample must also arrive before the reversal flag is cleared.
    return true;
  }

  bool fresh() const {
    const double ros_now = clock_->now().seconds();
    if (!observeClock(ros_now) || ros_clock_reversed_) return false;
    if (!have_odom_) return false;
    const double source_age = ros_now - odom_stamp_;
    // Preserve the source-stamp tolerance for asynchronously delivered /clock
    // and odometry. A backwards receipt clock never grants fresh authority.
    if (source_age < -0.1 || source_age > odom_timeout_) return false;
    if (!simulatedClock()) return odomAge() <= odom_timeout_;
    const double receipt_age = ros_now - odom_received_ros_;
    return receipt_age >= 0 && receipt_age <= odom_timeout_;
  }

  bool commandFresh() const {
    // Even while simulated time is stopped, old observations must not keep a
    // nonzero command alive or authorize a newly accepted action.
    return fresh() && odomAge() <= odom_timeout_;
  }

  void odometry(const nav_msgs::msg::Odometry& message) {
    Pose pose;
    const double stamp = rclcpp::Time(message.header.stamp).seconds();
    const double ros_now = clock_->now().seconds();
    if (!observeClock(ros_now)) return;
    const double age = ros_now - stamp;
    if (message.header.frame_id != fixed_frame_ ||
        (!message.child_frame_id.empty() && message.child_frame_id != robot_frame_) ||
        stamp <= 0 || age < -0.1 || age > odom_timeout_ ||
        !std::isfinite(message.twist.twist.linear.x) || !readPose(message.pose.pose, pose)) {
      return;
    }
    // Duplicate source samples must not renew command authority indefinitely.
    if (have_odom_ && (stamp <= odom_stamp_ ||
        (simulatedClock() && ros_now < odom_received_ros_))) return;
    current_ = pose;
    // nav_msgs/Odometry twist is expressed in child_frame_id (base_link),
    // unlike its fixed-frame pose. Do not rotate this body velocity again.
    actual_body_vx_ = message.twist.twist.linear.x;
    actual_body_vy_ = message.twist.twist.linear.y;
    actual_yaw_rate_ = message.twist.twist.angular.z;
    odom_stamp_ = stamp;
    odom_received_ros_ = ros_now;
    odom_received_ = Steady::now();
    have_odom_ = true;
    ros_clock_reversed_ = false;
    // Keep every accepted source sample for the current ground action's
    // checkpoint ledger. The control timer may run slower than odometry;
    // this callback neither commands motion nor advances a primitive.
    if (active_ && ground_maneuver_active_ && execution_mode_==Action::Goal::GROUND_MANEUVER &&
        maneuver_controller_ && !active_->is_canceling() && !shutting_down_ &&
        ros_now-maneuver_started_ros_<=maneuver_controller_->parameters().total_timeout) {
      const auto before=maneuver_controller_->command();
      maneuver_controller_->observePassThrough(maneuverObservation(),commandFresh());
      const auto& observed=maneuver_controller_->command();
      if (observed.pass_through_consumed!=before.pass_through_consumed ||
          observed.pass_through_missed!=before.pass_through_missed) {
        maneuver_last_=observed;
        transition(state_,reason_);
      }
    }
  }

  ManeuverObservation maneuverObservation() const {
    return {current_,actual_body_vx_,actual_body_vy_,actual_yaw_rate_,odom_stamp_};
  }

  ManeuverPlan readManeuver(const Action::Goal& goal, ManeuverParameters& parameters) const {
    const auto& m = goal.maneuver;
    const std::size_t size = goal.path.size();
    auto require = [](bool okay,const char* reason) {
      if (!okay) throw std::invalid_argument(reason);
    };
    require(allow_ground_maneuver_ && !goal.manual_path,"ground_maneuver_not_enabled_or_manual");
    require(std::isfinite(goal.pre_execution_timeout) && goal.pre_execution_timeout>0.,"invalid_pre_execution_budget");
    require(size>=2 && m.reference_times.size()==size && m.forward_velocities.size()==size &&
            m.yaw_rates.size()==size && !m.segments.empty() && m.segments.size()<=maneuver_caps_.max_segments,
            "invalid_maneuver_shapes");
    for (const auto limits : {std::pair<double,double>{m.max_speed,maneuver_caps_.max_speed},
         {m.max_yaw_rate,maneuver_caps_.max_yaw_rate},{m.max_acceleration,maneuver_caps_.max_acceleration},
         {m.max_yaw_acceleration,maneuver_caps_.max_yaw_acceleration},{m.max_duration,maneuver_caps_.total_timeout}})
      require(std::isfinite(limits.first) && limits.first>0. && limits.first<=limits.second+1e-12,
              "maneuver_limit_exceeds_local_cap");
    require(m.max_speed<=controller_->parameters().max_lin_vel &&
            m.max_yaw_rate<=controller_->parameters().max_ang_vel,"maneuver_exceeds_tracker_velocity_cap");
    require(std::isfinite(m.control_offset) && std::abs(m.control_offset)<=.5,"invalid_control_offset");
    require(goal.vel_max==0. || std::abs(goal.vel_max-m.max_speed)<=1e-12,"maneuver_velocity_contract_mismatch");
    require(std::isfinite(m.reference_times[0]) && m.reference_times[0]==0.,"reference_must_start_at_zero");
    parameters = maneuver_caps_;
    parameters.max_speed=m.max_speed; parameters.max_yaw_rate=m.max_yaw_rate;
    parameters.max_acceleration=m.max_acceleration; parameters.max_yaw_acceleration=m.max_yaw_acceleration;
    parameters.total_timeout=m.max_duration; parameters.segment_timeout=m.max_duration;
    // Bounded brake/confirmation allowance is independent of motion duration.
    parameters.stop_timeout=std::max(m.max_speed/m.max_acceleration,m.max_yaw_rate/m.max_yaw_acceleration)+2.;
    std::vector<Pose> reference;
    reference.reserve(size);
    double path_length=0.;
    for (std::size_t i=0; i<size; ++i) {
      Pose point;
      require(goal.path[i].header.frame_id==fixed_frame_ && readPose(goal.path[i].pose,point),"invalid_reference_pose");
      const auto& quaternion=goal.path[i].pose.orientation;
      require(std::abs(quaternion.x)<=1e-8 && std::abs(quaternion.y)<=1e-8,"nonplanar_reference_pose");
      require(std::isfinite(m.reference_times[i]) && std::isfinite(m.forward_velocities[i]) &&
              std::isfinite(m.yaw_rates[i]) && std::abs(m.forward_velocities[i])<=m.max_speed+1e-12 &&
              std::abs(m.yaw_rates[i])<=m.max_yaw_rate+1e-12,"invalid_reference_velocity");
      if (i) {
        require(std::abs(point[2]-reference.front()[2])<=1e-8,"reference_height_changed");
        const double dt=m.reference_times[i]-m.reference_times[i-1];
        require(dt>0. && dt<=.05+1e-9,"invalid_reference_time_step");
        require(std::abs(m.forward_velocities[i]-m.forward_velocities[i-1])<=m.max_acceleration*dt+1e-9 &&
                std::abs(m.yaw_rates[i]-m.yaw_rates[i-1])<=m.max_yaw_acceleration*dt+1e-9,
                "reference_acceleration_exceeds_limit");
        const double v=.5*(m.forward_velocities[i]+m.forward_velocities[i-1]);
        const double w=.5*(m.yaw_rates[i]+m.yaw_rates[i-1]);
        const auto& previous=reference.back();
        const double half=.5*w*dt, middle=previous[3]+half;
        const double distance=v*dt*(half==0. ? 1. : std::sin(half)/half);
        const double chord=2.*m.control_offset*std::sin(half);
        Pose integrated(previous[0]+distance*std::cos(middle)+chord*std::sin(middle),
                        previous[1]+distance*std::sin(middle)-chord*std::cos(middle),
                        previous[2],wrap(previous[3]+w*dt));
        require((integrated-point).head<3>().norm()<=1e-6 && std::abs(wrap(integrated[3]-point[3]))<=1e-6,
                "reference_not_generated_by_declared_twist");
        path_length+=dt*std::hypot(v,m.control_offset*w);
      }
      reference.push_back(point);
    }
    require(m.reference_times.back()<=m.max_duration+1e-9 && path_length<=parameters.max_reference_length+1e-9,
            "reference_resource_limit");
    ManeuverPlan plan;
    plan.control_offset=m.control_offset;
    plan.reference_times.assign(m.reference_times.begin(),m.reference_times.end());
    plan.reference_forward_velocities.assign(m.forward_velocities.begin(),m.forward_velocities.end());
    plan.reference_yaw_rates.assign(m.yaw_rates.begin(),m.yaw_rates.end());
    // Admission assertion only: real map approval belongs to the bridge's
    // ValidatePath gate. This value is not exposed as validation evidence.
    plan.geometry_validated=true;
    std::size_t previous_end=0, pass_through_count=0;
    for (const auto& segment : m.segments) {
      require(segment.kind<=1 && segment.stop_at_end && segment.start_index==previous_end &&
              segment.end_index>segment.start_index && segment.end_index<size && std::isfinite(segment.signed_amount),
              "invalid_primitive_boundaries");
      require(segment.kind==0 || segment.pass_through_distances.empty(),"invalid_maneuver_pass_through");
      require(segment.pass_through_distances.size()<=64-pass_through_count,"maneuver_pass_through_limit");
      double previous_distance=0.;
      for (double distance : segment.pass_through_distances) {
        require(std::isfinite(distance) && distance>previous_distance &&
                distance<std::abs(segment.signed_amount),"invalid_maneuver_pass_through");
        previous_distance=distance;
      }
      pass_through_count+=segment.pass_through_distances.size();
      for (const auto index : {segment.start_index,segment.end_index})
        require(std::abs(m.forward_velocities[index])<=1e-8 && std::abs(m.yaw_rates[index])<=1e-8,
                "primitive_boundary_not_stopped");
      double integral=0.;
      for (std::size_t i=segment.start_index+1; i<=segment.end_index; ++i) {
        const bool pivot=segment.kind==1;
        const double v0=m.forward_velocities[i-1],v1=m.forward_velocities[i];
        const double w0=m.yaw_rates[i-1],w1=m.yaw_rates[i];
        require(pivot ? (std::abs(v0)<=1e-8 && std::abs(v1)<=1e-8) :
                        (std::abs(w0)<=1e-8 && std::abs(w1)<=1e-8),"mixed_primitive_reference");
        const double rate0=pivot?w0:v0,rate1=pivot?w1:v1;
        require(rate0*segment.signed_amount>=-1e-10 && rate1*segment.signed_amount>=-1e-10,
                "reference_violates_declared_direction");
        integral+=.5*(rate0+rate1)*(m.reference_times[i]-m.reference_times[i-1]);
      }
      require(std::abs(integral-segment.signed_amount)<=1e-6,"primitive_amount_mismatch");
      ManeuverSegment adapted{segment.kind==0?ManeuverKind::Straight:ManeuverKind::Pivot,segment.signed_amount,
                              reference[segment.start_index],reference[segment.end_index],true,
                              segment.pass_through_distances,segment.start_index,segment.end_index};
      plan.segments.push_back(std::move(adapted));
      previous_end=segment.end_index;
    }
    require(previous_end+1==size,"incomplete_primitive_coverage");
    GroundManeuverController admission(parameters);
    admission.start(plan,maneuverObservation(),clock_->now().seconds());
    return plan;
  }

  rclcpp_action::GoalResponse accept(const rclcpp_action::GoalUUID& id,
                                     const Action::Goal& goal) {
    const auto received=Steady::now();
    last_goal_rejection_.clear();
    bool valid = !shutting_down_ && !reserved_ && !active_ && commandFresh() &&
        !goal.path.empty() && goal.path.size() <= static_cast<std::size_t>(max_points_) &&
        std::isfinite(goal.vel_max) && goal.vel_max >= 0 &&
        (goal.execution_mode==Action::Goal::PATH || goal.execution_mode==Action::Goal::GROUND_MANEUVER);
    Pose pose;
    if (valid) {
      for (const auto& point : goal.path) {
        if (point.header.frame_id != fixed_frame_ || !readPose(point.pose, pose)) {
          valid = false;
          last_goal_rejection_="invalid_path_pose_or_frame";
          break;
        }
      }
    }
    if (valid && goal.execution_mode==Action::Goal::GROUND_MANEUVER) {
      try {
        if (goal.bound_mode<0 || goal.bound_mode>4) throw std::invalid_argument("invalid_bound_mode");
        pending_maneuver_plan_=readManeuver(goal,pending_maneuver_parameters_);
        maneuver_admission_received_=received;
        maneuver_admission_budget_=goal.pre_execution_timeout;
        if (std::chrono::duration<double>(Steady::now()-received).count()>=maneuver_admission_budget_)
          throw std::invalid_argument("pre_execution_timeout");
        last_goal_rejection_.clear();
      } catch (const std::exception& e) {
        valid=false;
        last_goal_rejection_=e.what();
      }
    }
    if (!valid) {
      ++rejected_;
      if (last_goal_rejection_.empty()) last_goal_rejection_="tracker_not_ready_or_invalid_goal";
      // A rejected goal never reaches accepted()/transition(). Publish the
      // rejection diagnostic now, without changing an active goal's state or
      // command. Otherwise topic observers retain an old IDLE/ready snapshot.
      std_msgs::msg::String message;
      message.data=payload();
      status_->publish(message);
      RCLCPP_WARN(get_logger(),"Goal rejected: %s",last_goal_rejection_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Reserve during the goal callback, before the accepted callback is queued.
    reserved_ = true;
    last_goal_id_ = goalId(id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void accepted(std::shared_ptr<Handle> handle) {
    active_ = std::move(handle);
    reserved_ = false;
    ++accepted_;
    const auto goal = active_->get_goal();
    execution_mode_=goal->execution_mode;
    ground_maneuver_active_=false;
    if (execution_mode_==Action::Goal::GROUND_MANEUVER) {
      try {
        if (std::chrono::duration<double>(Steady::now()-maneuver_admission_received_).count()>=maneuver_admission_budget_ ||
            !commandFresh())
          throw std::invalid_argument("maneuver_admission_expired_or_odometry_lost");
        maneuver_controller_=std::make_unique<GroundManeuverController>(pending_maneuver_parameters_);
        maneuver_motion_time_=clock_->now().seconds();
        maneuver_started_ros_=maneuver_motion_time_;
        maneuver_controller_->start(pending_maneuver_plan_,maneuverObservation(),maneuver_motion_time_);
        maneuver_last_=maneuver_controller_->command();
        maneuver_plan_=pending_maneuver_plan_;
        ground_maneuver_active_=true;
        maneuver_stop_wall_started_=false;
        maneuver_paused_inhibit_=false;
        endpoint_=maneuver_plan_.segments.back().expected_end;
        remaining_=goal->path.size();
        bound_mode_=goal->bound_mode;
        manual_path_=false;
        controller_->reset(); terminal_->reset(); alignment_->reset(); rotation_hold_.reset();
        transition("MANEUVER_RUNNING","explicit_ground_maneuver_accepted");
      } catch (const std::exception& e) {
        finish(false,std::string("maneuver_not_dispatched:")+e.what());
      }
      return;
    }
    maneuver_controller_.reset();
    std::vector<Pose> path;
    path.reserve(goal->path.size());
    for (const auto& point : goal->path) {
      Pose pose;
      readPose(point.pose, pose);  // Already checked synchronously in accept().
      path.push_back(pose);
    }
    if (goal->vel_max > 0) active_speed_ = std::min(goal->vel_max, controller_->parameters().max_lin_vel);
    controller_->setPath(path, active_speed_);
    terminal_->reset();
    alignment_->reset();
    rotation_hold_.reset();
    odometry_paused_ = false;
    endpoint_ = path.back();
    remaining_ = path.size();
    bound_mode_ = goal->bound_mode;
    manual_path_ = goal->manual_path;
    transition("TRACKING", "goal_accepted");
    if (shutting_down_ || !commandFresh()) finish(false, "odometry_lost_before_tracking");
  }

  void publishCommand(double v, double w) {
    geometry_msgs::msg::Twist command;
    command.linear.x = v;
    command.angular.z = w;
    last_v_ = v;
    last_w_ = w;
    commands_->publish(command);
  }

  void publishCarrot(const Pose& carrot) {
    // Transform using the same odometry sample used for control, rather than
    // combining the latest TF pose with a different-time odometry message.
    const double c = std::cos(current_[3]), s = std::sin(current_[3]);
    const double dx = carrot[0] - current_[0], dy = carrot[1] - current_[1];
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = robot_frame_;
    message.header.stamp = rclcpp::Time(static_cast<int64_t>(odom_stamp_ * 1e9));
    message.pose.position.x = c * dx + s * dy;
    message.pose.position.y = -s * dx + c * dy;
    message.pose.position.z = carrot[2] - current_[2];
    const double yaw = wrap(carrot[3] - current_[3]);
    message.pose.orientation.z = std::sin(yaw / 2);
    message.pose.orientation.w = std::cos(yaw / 2);
    carrots_->publish(message);
  }

  void tick() {
    observeClock(clock_->now().seconds());
    if (!active_) return;
    if (ground_maneuver_active_) { tickManeuver(); return; }
    if (active_->is_canceling()) { finish(false, "cancelled"); return; }
    if (!fresh()) { finish(false, "odometry_lost"); return; }
    if (!commandFresh()) {
      // Simulated time can advance more slowly than wall time or pause. Keep
      // the action and path, but stop until a strictly newer odometry sample
      // arrives. No controller step or integral update uses the old sample.
      publishCommand(0, 0);
      if (!odometry_paused_) {
        state_before_pause_ = state_;
        odometry_paused_ = true;
        transition("PAUSED", "odometry_receipt_delayed");
      }
      return;
    }
    const bool resumed = odometry_paused_;
    odometry_paused_ = false;
    auto terminal_mode = TerminalMode::Tracking;
    if (controller_->parameters().preserve_corners && controller_->pendingCorners()) {
      // A returning route may end near its start. Endpoint proximity cannot
      // bypass an unvisited gate. step() may release the last gate below; the
      // next tick can then enter the ordinary terminal policy.
      terminal_->reset();
    } else {
      terminal_mode = terminal_->update(current_, endpoint_);
    }
    const auto command = controller_->step(current_, terminal_->recoveryDirection());
    remaining_ = command.remaining;
    dist_to_goal_ = (endpoint_ - current_).head<2>().norm();
    yaw_error_ = wrap(endpoint_[3] - current_[3]);
    publishCarrot(command.carrot);
    double v = command.v, w = command.w;
    if (terminal_mode == TerminalMode::Reached) { remaining_ = 0; finish(true, "goal_reached"); return; }
    if (terminal_mode == TerminalMode::Aligning) {
      rotation_hold_.reset();
      remaining_ = 0;
      if (state_ != "ALIGNING" && !(resumed && state_before_pause_ == "ALIGNING"))
        alignment_->reset();
      const auto alignment = alignment_->step(current_, endpoint_, actual_body_vx_,
          active_speed_, controller_->parameters().max_ang_vel, controller_->parameters().dt);
      v = alignment.v;
      w = alignment.w;
      if (rotation_hold_yaw_governor_) {
        // The same longitudinal priority applies while turning at the final
        // pose. Keep the alignment controller's v and integral law unchanged.
        w *= longitudinalYawScale(current_, endpoint_.head<2>(),
                                  controller_->parameters().goal_reaching_threshold);
      }
    } else {
      alignment_->reset();
      if (const auto direction = terminal_->recoveryDirection()) {
        rotation_hold_.reset();
        // The path controller still advances once per tick for path metrics.
        // Recover terminal XY directly; applying its approach and curvature
        // slowdowns again can leave a slipped chassis crawling for seconds.
        const auto correction = terminalPositionCommand(current_, endpoint_, *direction,
            active_speed_, controller_->parameters().max_ang_vel,
            controller_->parameters().critical_angle,
            controller_->parameters().goal_reaching_threshold);
        v = correction.v;
        w = correction.w;
      } else if (command.rotation_only) {
        // ROS 2 chassis adaptation: hold this turn's initial XY against
        // longitudinal drift. The optional governor gives that correction
        // priority by reducing yaw as its body-X offset reaches tolerance.
        if (rotation_hold_yaw_governor_) {
          const auto hold = rotation_hold_.governedStep(current_, actual_body_vx_,
              active_speed_, w, controller_->parameters().goal_reaching_threshold);
          v = hold.v;
          w = hold.w;
        } else {
          v = rotation_hold_.step(current_, actual_body_vx_, active_speed_);
        }
      } else {
        rotation_hold_.reset();
      }
    }
    if (!std::isfinite(v) || !std::isfinite(w)) { finish(false, "invalid_command"); return; }
    publishCommand(v, w);
    // Transition events describe the command computed from this observation,
    // rather than the command from the preceding control mode.
    if (terminal_mode == TerminalMode::Aligning && state_ != "ALIGNING")
      transition("ALIGNING", resumed ? "odometry_resumed" : "final_heading");
    else if (terminal_mode == TerminalMode::Tracking && state_ != "TRACKING")
      transition("TRACKING", resumed ? "odometry_resumed" : "position_correction");
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->remaining_waypoints = static_cast<int32_t>(remaining_);
    feedback->dist_to_goal = static_cast<float>(dist_to_goal_);
    feedback->estimated_time_remaining = static_cast<float>(
        command.remaining_distance / active_speed_ +
        std::abs(yaw_error_) / controller_->parameters().max_ang_vel);
    active_->publish_feedback(feedback);
  }

  void finishManeuver(const ManeuverCommand& command) {
    if (!active_) return;
    maneuver_last_=command;
    // The core has already produced zero command and either observed the
    // stop or explicitly reported stop_unconfirmed. Do not use PATH finish.
    publishCommand(0.,0.);
    const bool cancelled=active_->is_canceling() && command.status==ManeuverStatus::Cancelled && command.stop_confirmed;
    const bool success=command.status==ManeuverStatus::Succeeded && command.stop_confirmed && !active_->is_canceling();
    auto result=std::make_shared<Action::Result>(); result->success=success;
    if (cancelled) { active_->canceled(result); ++cancelled_; }
    else if (success) { active_->succeed(result); ++succeeded_; }
    else { active_->abort(result); ++aborted_; }
    active_.reset(); ground_maneuver_active_=false;
    if (success) remaining_=0;
    transition(cancelled?"CANCELED":success?"SUCCEEDED":"ABORTED",command.reason);
  }

  void tickManeuver() {
    const auto wall_now=Steady::now();
    const double ros_now=clock_->now().seconds();
    const double previous_motion_time=maneuver_motion_time_;
    if (std::isfinite(ros_now) && ros_now>maneuver_motion_time_) maneuver_motion_time_=ros_now;
    const bool fresh_command=commandFresh() && std::isfinite(actual_body_vy_) && std::isfinite(actual_yaw_rate_);
    if (active_->is_canceling()) maneuver_controller_->cancel(maneuver_motion_time_);
    if (shutting_down_) maneuver_controller_->abort("shutdown",maneuver_motion_time_);
    if (!fresh_command) maneuver_controller_->abort("maneuver_odometry_lost",maneuver_motion_time_);
    auto command=maneuver_controller_->step(maneuverObservation(),maneuver_motion_time_);
    const bool checkpoint_changed=command.pass_through_consumed!=maneuver_last_.pass_through_consumed;
    const bool capture_changed=command.terminal_capture_active!=maneuver_last_.terminal_capture_active ||
                               command.terminal_capture_preparing!=maneuver_last_.terminal_capture_preparing;
    const bool measured_brake_changed=command.measured_brake_active!=maneuver_last_.measured_brake_active ||
                                      command.measured_brake_count!=maneuver_last_.measured_brake_count;
    maneuver_last_=command;
    if (command.status==ManeuverStatus::Braking && !maneuver_stop_wall_started_) {
      maneuver_stop_wall_started_=true;
      maneuver_stop_wall_at_=wall_now;
    }
    const bool finished=command.status==ManeuverStatus::Succeeded || command.status==ManeuverStatus::Cancelled ||
                        command.status==ManeuverStatus::Failed;
    if (finished) { finishManeuver(command); return; }
    // A paused/regressed simulation has no physical-time slew increment.
    // Suppress stale transport output instead of replaying a nonzero command;
    // this is an emergency pause inhibit, never a stop-confirmation claim.
    // On resumption the additional output limiter ramps from the inhibited
    // value using ROS elapsed time, so no stored command jumps back on.
    const double ros_dt=std::clamp(maneuver_motion_time_-previous_motion_time,0.,maneuver_caps_.maximum_step_dt);
    if (!fresh_command && ros_dt==0.) {
      maneuver_paused_inhibit_=true;
      publishCommand(0.,0.);
    } else {
      const auto& p=maneuver_controller_->parameters();
      const double v=std::clamp(command.v,last_v_-p.max_acceleration*ros_dt,last_v_+p.max_acceleration*ros_dt);
      const double w=std::clamp(command.w,last_w_-p.max_yaw_acceleration*ros_dt,last_w_+p.max_yaw_acceleration*ros_dt);
      publishCommand(v,w);
    }
    if (maneuver_stop_wall_started_) {
      const auto& p=maneuver_controller_->parameters();
      const double wall_budget=std::max(p.max_speed/p.max_acceleration,p.max_yaw_rate/p.max_yaw_acceleration)+
                               p.stop_timeout+odom_timeout_;
      if (std::chrono::duration<double>(wall_now-maneuver_stop_wall_at_).count()>wall_budget) {
        command.status=ManeuverStatus::Failed;
        command.stop_confirmed=false;
        command.reason+="_wall_stop_unconfirmed";
        finishManeuver(command); return;
      }
    }
    dist_to_goal_=(endpoint_-current_).head<2>().norm();
    yaw_error_=wrap(endpoint_[3]-current_[3]);
    const auto goal=active_->get_goal();
    const auto index=std::min(command.segment_index,goal->maneuver.segments.size()-1);
    remaining_=goal->path.size()-goal->maneuver.segments[index].start_index;
    publishCarrot(maneuver_plan_.segments[index].expected_end);
    const std::string state=command.status==ManeuverStatus::Braking?"MANEUVER_BRAKING":
                            command.status==ManeuverStatus::Settling?"MANEUVER_SETTLING":"MANEUVER_RUNNING";
    if (state_!=state || reason_!=command.reason || checkpoint_changed || capture_changed || measured_brake_changed)
      transition(state,command.reason);
    auto feedback=std::make_shared<Action::Feedback>();
    feedback->remaining_waypoints=static_cast<int32_t>(remaining_);
    feedback->dist_to_goal=static_cast<float>(dist_to_goal_);
    feedback->estimated_time_remaining=static_cast<float>(std::max(0.,goal->maneuver.max_duration-
                                                (maneuver_motion_time_-maneuver_started_ros_)));
    active_->publish_feedback(feedback);
  }

  void finish(bool success, const std::string& reason) {
    odometry_paused_ = false;
    alignment_->reset();
    rotation_hold_.reset();
    publishCommand(0, 0);
    if (!active_) return;
    const bool cancelled = active_->is_canceling();
    auto result = std::make_shared<Action::Result>();
    result->success = success && !cancelled;
    if (cancelled) { active_->canceled(result); ++cancelled_; }
    else if (success) { active_->succeed(result); ++succeeded_; }
    else { active_->abort(result); ++aborted_; }
    active_.reset();
    controller_->reset();
    terminal_->reset();
    transition(cancelled ? "CANCELED" : (success ? "SUCCEEDED" : "ABORTED"), reason);
  }

  std::string payload() const {
    std::ostringstream out;
    out << std::setprecision(10) << "{\"state\":\"" << state_ << "\",\"reason\":\"" << reason_
        << "\",\"goal_id\":\"" << last_goal_id_ << "\",\"active\":" << (active_ ? "true" : "false")
        << ",\"remaining_waypoints\":" << remaining_ << ",\"dist_to_goal\":" << dist_to_goal_
        << ",\"odometry_stamp_s\":" << std::setprecision(17) << odom_stamp_ << std::setprecision(10)
        << ",\"yaw_error\":" << yaw_error_ << ",\"odom_age\":";
    const double age = odomAge();
    if (std::isfinite(age)) out << age; else out << "null";
    const double ros_now = clock_->now().seconds();
    observeClock(ros_now);
    out << ",\"odometry_paused\":" << (odometry_paused_ ? "true" : "false")
        << ",\"odometry_freshness\":{\"use_sim_time\":" << (simulatedClock() ? "true" : "false")
        << ",\"source_age_s\":";
    if (have_odom_) out << ros_now - odom_stamp_; else out << "null";
    out << ",\"receipt_ros_age_s\":";
    if (have_odom_) out << ros_now - odom_received_ros_; else out << "null";
    out << ",\"receipt_steady_age_s\":";
    if (std::isfinite(age)) out << age; else out << "null";
    out << ",\"timeout_s\":" << odom_timeout_
        << ",\"clock_reversed\":" << (ros_clock_reversed_ ? "true" : "false")
        << ",\"paused\":" << (odometry_paused_ ? "true" : "false") << "}";
    out << ",\"last_command\":{\"linear_x\":" << last_v_ << ",\"angular_z\":" << last_w_ << "}"
        << ",\"position_tolerance\":" << controller_->parameters().goal_reaching_threshold
        << ",\"yaw_tolerance\":" << yaw_threshold_
        << ",\"alignment_exit_distance\":" << terminal_->alignmentExitDistance()
        << ",\"position_correction_direction\":"
        << (terminal_->recoveryDirection() ? std::to_string(*terminal_->recoveryDirection()) : "null")
        << ",\"position_correction_law\":\"latched_bearing_distance\""
        << ",\"terminal_position_gain\":2,\"terminal_velocity_damping\":0.5"
        << ",\"terminal_velocity_integral_gain\":" << alignment_->integralGain()
        << ",\"terminal_velocity_integral_mps\":" << alignment_->integralVelocity()
        << ",\"terminal_linear_limit\":" << std::min(active_speed_, 0.1)
        << ",\"rotation_only_hold_active\":" << (rotation_hold_.anchor() ? "true" : "false")
        << ",\"rotation_only_hold_anchor_xy\":";
    if (const auto& anchor = rotation_hold_.anchor())
      out << '[' << anchor->x() << ',' << anchor->y() << ']';
    else out << "null";
    out
        << ",\"preserve_corners\":" << (controller_->parameters().preserve_corners ? "true" : "false")
        << ",\"steering_curvature\":\"" << steeringCurvatureName(controller_->parameters().steering_curvature) << "\""
        << ",\"rotation_hold_yaw_governor\":" << (rotation_hold_yaw_governor_ ? "true" : "false")
        << ",\"speed\":" << (ground_maneuver_active_?maneuver_controller_->parameters().max_speed:active_speed_)
        << ",\"bound_mode\":" << bound_mode_
        << ",\"manual_path\":" << (manual_path_ ? "true" : "false")
        << ",\"execution_mode\":" << unsigned(execution_mode_)
        << ",\"allow_ground_maneuver\":" << (allow_ground_maneuver_?"true":"false")
        << ",\"ground_maneuver\":{\"active\":" << (ground_maneuver_active_?"true":"false")
        << ",\"clock\":\"ros_motion_with_wall_watchdog\",\"motion_time_s\":" << maneuver_motion_time_
        << ",\"paused_transport_inhibit\":" << (maneuver_paused_inhibit_?"true":"false")
        << ",\"segment_index\":" << maneuver_last_.segment_index
        << ",\"terminal_capture_preparing\":" << (maneuver_last_.terminal_capture_preparing?"true":"false")
        << ",\"terminal_capture_active\":" << (maneuver_last_.terminal_capture_active?"true":"false")
        << ",\"terminal_capture_progress\":" << maneuver_last_.terminal_capture_progress
        << ",\"measured_brake_active\":" << (maneuver_last_.measured_brake_active?"true":"false")
        << ",\"measured_brake_count\":" << maneuver_last_.measured_brake_count
        << ",\"measured_brake_stop_distance\":" << maneuver_last_.measured_brake_stop_distance
        << ",\"measured_brake_remaining\":" << maneuver_last_.measured_brake_remaining
        << ",\"pass_through_total\":" << maneuver_last_.pass_through_total
        << ",\"pass_through_consumed\":" << maneuver_last_.pass_through_consumed
        << ",\"segment_pass_through_total\":" << maneuver_last_.segment_pass_through_total
        << ",\"segment_pass_through_consumed\":" << maneuver_last_.segment_pass_through_consumed
        << ",\"pass_through_last_consumed_stamp_s\":" << std::setprecision(17)
        << maneuver_last_.pass_through_last_consumed_stamp_s
        << ",\"pass_through_missed\":" << (maneuver_last_.pass_through_missed?"true":"false")
        << ",\"pass_through_miss_stamp_s\":" << maneuver_last_.pass_through_miss_stamp_s << std::setprecision(10)
        << ",\"reference_error\":" << maneuver_last_.reference_error
        << ",\"stop_confirmed\":" << (maneuver_last_.stop_confirmed?"true":"false")
        << ",\"max_speed_cap\":" << maneuver_caps_.max_speed
        << ",\"max_yaw_rate_cap\":" << maneuver_caps_.max_yaw_rate
        << ",\"max_acceleration_cap\":" << maneuver_caps_.max_acceleration
        << ",\"max_yaw_acceleration_cap\":" << maneuver_caps_.max_yaw_acceleration
        << ",\"max_duration_cap\":" << maneuver_caps_.total_timeout << "}"
        << ",\"last_goal_rejection\":" << std::quoted(last_goal_rejection_)
        << ",\"accepted\":" << accepted_ << ",\"rejected\":" << rejected_
        << ",\"succeeded\":" << succeeded_ << ",\"cancelled\":" << cancelled_
        << ",\"aborted\":" << aborted_ << "}";
    return out.str();
  }

  void transition(const std::string& state, const std::string& reason) {
    state_ = state;
    reason_ = reason;
    std_msgs::msg::String message;
    message.data = payload();
    status_->publish(message);
    RCLCPP_INFO(get_logger(), "%s: %s", state.c_str(), reason.c_str());
  }

  std::unique_ptr<PurePursuitController> controller_;
  std::unique_ptr<TerminalPosePolicy> terminal_;
  std::unique_ptr<TerminalAlignmentController> alignment_;
  std::unique_ptr<GroundManeuverController> maneuver_controller_;
  ManeuverParameters maneuver_caps_, pending_maneuver_parameters_;
  ManeuverPlan pending_maneuver_plan_, maneuver_plan_;
  ManeuverCommand maneuver_last_;
  RotationOnlyHold rotation_hold_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::shared_ptr<Handle> active_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commands_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrots_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string fixed_frame_, robot_frame_, last_goal_id_, state_ = "IDLE", reason_ = "ready";
  std::string state_before_pause_;
  Pose current_ = Pose::Zero(), endpoint_ = Pose::Zero();
  Steady::time_point odom_received_{};
  Steady::time_point maneuver_admission_received_{},maneuver_stop_wall_at_{};
  double odom_stamp_ = 0, odom_received_ros_ = 0, odom_timeout_ = 0.5, yaw_threshold_ = 0.1, active_speed_ = 0.5;
  double actual_body_vx_ = 0,actual_body_vy_ = 0,actual_yaw_rate_ = 0;
  double maneuver_motion_time_ = 0,maneuver_started_ros_ = 0;
  double maneuver_admission_budget_ = 0;
  double last_v_ = 0, last_w_ = 0, dist_to_goal_ = 0, yaw_error_ = 0;
  bool have_odom_ = false, reserved_ = false, shutting_down_ = false, manual_path_ = false;
  bool rotation_hold_yaw_governor_ = false;
  bool odometry_paused_ = false;
  bool allow_ground_maneuver_ = false,ground_maneuver_active_ = false;
  bool maneuver_stop_wall_started_ = false,maneuver_paused_inhibit_ = false;
  uint8_t execution_mode_ = Action::Goal::PATH;
  std::string last_goal_rejection_;
  mutable double ros_clock_high_water_ = -std::numeric_limits<double>::infinity();
  mutable bool ros_clock_reversed_ = false;
  std::size_t remaining_ = 0;
  int max_points_ = 20000, bound_mode_ = 3;
  uint64_t accepted_ = 0, rejected_ = 0, succeeded_ = 0, cancelled_ = 0, aborted_ = 0;
};
}  // namespace gbplanner_tracking

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  try {
    auto node = std::make_shared<gbplanner_tracking::PurePursuitNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // Keep DDS live for measured maneuver braking and its action result.
    node->stopForShutdown();
    const auto until = std::chrono::steady_clock::now() +
        (node->shutdownPending()?std::chrono::milliseconds(6000):std::chrono::milliseconds(100));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < until) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    node->finishShutdown();
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("gbplanner_pure_pursuit"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
