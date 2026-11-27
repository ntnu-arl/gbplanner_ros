#include "gbplanner_smb/smb_vehicle.hpp"
#include "gbplanner_smb/triangle_mesh.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <csignal>
#include <ctime>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }
}

namespace gbplanner_smb {
using Clock = std::chrono::steady_clock;
using Vector = Eigen::Vector3d;

double threadCpuSeconds() {
#if defined(CLOCK_THREAD_CPUTIME_ID)
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) == 0)
    return static_cast<double>(value.tv_sec) + static_cast<double>(value.tv_nsec) * 1.e-9;
#endif
  // Do not substitute process CPU: DDS worker threads would contaminate the
  // scan measurement. Platforms without this clock keep wall timing only.
  return std::numeric_limits<double>::quiet_NaN();
}

class SmbNode final : public rclcpp::Node {
 public:
  SmbNode() : Node("gbplanner_smb") {
    VehicleParams p;
    p.chassis_mass = declare_parameter("chassis_mass", p.chassis_mass);
    p.chassis_size = vectorParameter("chassis_size", p.chassis_size);
    p.wheel_mass = declare_parameter("wheel_mass", p.wheel_mass);
    p.wheel_radius = declare_parameter("wheel_radius", p.wheel_radius);
    p.wheel_width = declare_parameter("wheel_width", p.wheel_width);
    p.front_wheel_x = declare_parameter("front_wheel_x", p.front_wheel_x);
    p.rear_wheel_x = declare_parameter("rear_wheel_x", p.rear_wheel_x);
    p.wheel_y = declare_parameter("wheel_y", p.wheel_y);
    p.wheel_z = declare_parameter("wheel_z", p.wheel_z);
    p.wheel_track = declare_parameter("wheel_track", p.wheel_track);
    p.max_linear_velocity = declare_parameter("max_linear_velocity", p.max_linear_velocity);
    p.max_angular_velocity = declare_parameter("max_angular_velocity", p.max_angular_velocity);
    p.max_linear_acceleration = declare_parameter("max_linear_acceleration", p.max_linear_acceleration);
    p.max_angular_acceleration = declare_parameter("max_angular_acceleration", p.max_angular_acceleration);
    p.angular_velocity_feedback_gain = declare_parameter(
        "angular_velocity_feedback_gain", p.angular_velocity_feedback_gain);
    angular_velocity_feedback_gain_ = p.angular_velocity_feedback_gain;
    p.max_wheel_torque = declare_parameter("max_wheel_torque", p.max_wheel_torque);
    p.wheel_friction = declare_parameter("wheel_friction", p.wheel_friction);
    p.ground_friction = declare_parameter("ground_friction", p.ground_friction);
    p.rolling_friction = declare_parameter("rolling_friction", p.rolling_friction);
    p.spinning_friction = declare_parameter("spinning_friction", p.spinning_friction);
    p.initial_position = vectorParameter("initial_position", p.initial_position);
    const double yaw = declare_parameter("initial_yaw", 0.0);
    if (!std::isfinite(yaw)) throw std::invalid_argument("initial_yaw");
    p.initial_orientation = Eigen::AngleAxisd(yaw, Vector::UnitZ());
    p.ground_normal = vectorParameter("ground_normal", p.ground_normal);
    p.ground_offset = declare_parameter("ground_offset", p.ground_offset);
    p.add_ground = declare_parameter("add_ground", p.add_ground);
    add_ground_ = p.add_ground;
    room_ = declare_parameter("room", true);
    const auto collision_mesh = declare_parameter("collision_mesh", std::string(""));
    physics_step_ = declare_parameter("physics_step", 0.002);
    command_timeout_ = declare_parameter("command_timeout", 0.3);
    lidar_period_ = declare_parameter("lidar_period", 0.2);
    lidar_range_ = declare_parameter("lidar_range", 6.0);
    frame_ = declare_parameter("frame_id", std::string("map"));
    robot_frame_ = declare_parameter("robot_frame", std::string("base_link"));
    sensor_frame_ = declare_parameter("sensor_frame", std::string("smb_lidar"));
    sensor_offset_ = vectorParameter("sensor_offset", {0.0, 0.0, 0.3});
    if (!std::isfinite(physics_step_) || physics_step_ <= 0 || physics_step_ > .01 ||
        !std::isfinite(command_timeout_) || command_timeout_ <= 0 ||
        !std::isfinite(lidar_period_) || lidar_period_ < .05 ||
        !std::isfinite(lidar_range_) || lidar_range_ <= .1 ||
        frame_.empty() || robot_frame_.empty() || sensor_frame_.empty()) {
      throw std::invalid_argument("invalid SMB node parameters");
    }
    vehicle_ = std::make_unique<SmbVehicle>(p);
    if (!collision_mesh.empty()) {
      const auto mesh = readTriangleMesh(collision_mesh);
      vehicle_->addTriangleMesh(mesh.vertices, mesh.triangles);
      mesh_vertices_ = mesh.vertices.size();
      mesh_triangles_ = mesh.triangles.size();
    }
    if (room_) {
      // Compact two-room scene; these boxes generate contacts and laser returns.
      vehicle_->addBox({0, -3, 1}, {10, .2, 2});
      vehicle_->addBox({0, 3, 1}, {10, .2, 2});
      vehicle_->addBox({-5, 0, 1}, {.2, 6, 2});
      vehicle_->addBox({5, 0, 1}, {.2, 6, 2});
      vehicle_->addBox({0, -2, 1}, {.2, 2, 2});
      vehicle_->addBox({0, 2, 1}, {.2, 2, 2});
    }
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr message) { receive(*message); });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 20);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>("smb_status", 10);
    state_service_ = create_service<std_srvs::srv::Trigger>("smb_state",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          response->success = true;
          response->message = payload(vehicle_->state());
        });
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    // The rigid lidar mount never changes. A latched static transform also
    // reaches listeners that join after construction, without a 50 Hz write.
    static_tf_->sendTransform(transform(now(), robot_frame_, sensor_frame_, sensor_offset_,
                                        Eigen::Quaterniond::Identity()));
    state_ = vehicle_->state();
    last_position_ = state_.position;
    last_tick_ = last_output_ = last_lidar_ = Clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "SMB ready: 40 kg chassis, four driven wheel contacts; Bullet physics");
  }

 private:
  struct PhaseTiming {
    bool ran{false};
    double wall_s{0.};
    double cpu_s{std::numeric_limits<double>::quiet_NaN()};
  };
  // Fixed-size observations; no per-output history or dynamic allocation.
  static constexpr std::size_t kOutputPhases = 5;
  // A completed physics step is indivisible. This budget limits catch-up
  // batches, not the duration of a single Bullet step and state extraction.
  static constexpr double kPhysicsWallBudgetSeconds = 0.005;
  static constexpr double kPhysicsBacklogLimitSeconds = 0.05;
  struct OutputPhaseStats {
    uint64_t samples{0}, cpu_samples{0};
    double max_wall_s{0.}, max_cpu_s{0.};
    double cpu_at_max_wall_s{std::numeric_limits<double>::quiet_NaN()};
  };
  struct SlowTick {
    int64_t start_ros_stamp_ns{0};
    uint64_t tick_id{0};
    int physics_steps{0};
    double entry_gap_s{0.}, wall_s{0.};
    double cpu_s{std::numeric_limits<double>::quiet_NaN()};
    PhaseTiming physics, output, scan, pending_publish;
    std::array<PhaseTiming, kOutputPhases> output_subphases{};
  };

  Vector vectorParameter(const std::string& name, const Vector& defaults) {
    auto value = declare_parameter<std::vector<double>>(name,
        {defaults.x(), defaults.y(), defaults.z()});
    if (value.size() != 3) throw std::invalid_argument(name + " needs 3 values");
    Vector result(value[0], value[1], value[2]);
    if (!result.allFinite()) throw std::invalid_argument(name + " must be finite");
    return result;
  }

  void receive(const geometry_msgs::msg::Twist& command) {
    const double values[] = {command.linear.x, command.linear.y, command.linear.z,
        command.angular.x, command.angular.y, command.angular.z};
    if (!std::all_of(std::begin(values), std::end(values),
                     [](double v) { return std::isfinite(v); }) ||
        std::abs(command.linear.y) > 1e-8 || std::abs(command.linear.z) > 1e-8 ||
        std::abs(command.angular.x) > 1e-8 || std::abs(command.angular.y) > 1e-8) {
      requested_v_ = requested_w_ = 0;
      ++rejected_commands_;
      return;
    }
    requested_v_ = command.linear.x;
    requested_w_ = command.angular.z;
    last_command_ = Clock::now();
    received_command_ = true;
    ++commands_;
  }

  geometry_msgs::msg::TransformStamped transform(const rclcpp::Time& stamp,
      const std::string& parent, const std::string& child, const Vector& p,
      const Eigen::Quaterniond& q) const {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = p.x();
    t.transform.translation.y = p.y();
    t.transform.translation.z = p.z();
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    return t;
  }

  void output(const rclcpp::Time& stamp,
              std::array<PhaseTiming, kOutputPhases>& phases) {
    const auto& s = state_;
    const auto measure = [&phases](std::size_t index, const auto& operation) {
      const auto started = Clock::now();
      const double cpu_started = threadCpuSeconds();
      operation();
      const double cpu = threadCpuSeconds() - cpu_started;
      phases[index] = {true, std::chrono::duration<double>(Clock::now()-started).count(),
          std::isfinite(cpu) && cpu >= 0. ? cpu : std::numeric_limits<double>::quiet_NaN()};
    };
    measure(0, [&]() {
      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = frame_;
      odom.child_frame_id = robot_frame_;
      odom.header.stamp = stamp;
      odom.pose.pose.position.x = s.position.x();
      odom.pose.pose.position.y = s.position.y();
      odom.pose.pose.position.z = s.position.z();
      odom.pose.pose.orientation.x = s.orientation.x();
      odom.pose.pose.orientation.y = s.orientation.y();
      odom.pose.pose.orientation.z = s.orientation.z();
      odom.pose.pose.orientation.w = s.orientation.w();
      const Vector v = s.orientation.conjugate() * s.linear_velocity;
      const Vector w = s.orientation.conjugate() * s.angular_velocity;
      odom.twist.twist.linear.x = v.x();
      odom.twist.twist.linear.y = v.y();
      odom.twist.twist.linear.z = v.z();
      odom.twist.twist.angular.x = w.x();
      odom.twist.twist.angular.y = w.y();
      odom.twist.twist.angular.z = w.z();
      odom_pub_->publish(odom);
    });
    measure(1, [&]() {
      tf_->sendTransform(transform(stamp, frame_, robot_frame_, s.position, s.orientation));
    });
    measure(2, [&]() {
      sensor_msgs::msg::JointState joints;
      joints.header.stamp = stamp;
      joints.name = {"LF_wheel_joint", "RF_wheel_joint", "LH_wheel_joint", "RH_wheel_joint"};
      for (const auto& wheel : s.wheels) {
        joints.position.push_back(wheel.angle);
        joints.velocity.push_back(wheel.angular_velocity);
      }
      joint_pub_->publish(joints);
    });
    std_msgs::msg::String status;
    measure(3, [&]() { status.data = payload(s); });
    measure(4, [&]() { status_pub_->publish(status); });
    for (std::size_t i = 0; i < phases.size(); ++i) {
      const auto& phase = phases[i];
      auto& stats = output_phase_stats_[i];
      ++stats.samples;
      if (std::isfinite(phase.cpu_s)) {
        ++stats.cpu_samples;
        stats.max_cpu_s = std::max(stats.max_cpu_s, phase.cpu_s);
      }
      if (phase.wall_s > stats.max_wall_s) {
        stats.max_wall_s = phase.wall_s;
        stats.cpu_at_max_wall_s = phase.cpu_s;
      }
    }
  }

  void scan(const rclcpp::Time& stamp) {
    const Vector origin = state_.position + state_.orientation * sensor_offset_;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = sensor_frame_;
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.point_step = 12;
    for (int i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField field;
      field.name = std::string(1, "xyz"[i]);
      field.offset = i * 4;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);
    }
    constexpr double kPi = 3.14159265358979323846;
    for (int elevation = -60; elevation <= 30; elevation += 5) {
      for (int azimuth = -180; azimuth < 180; azimuth += 3) {
        const double v = elevation * kPi / 180, h = azimuth * kPi / 180;
        const Vector direction(std::cos(v) * std::cos(h), std::cos(v) * std::sin(h), std::sin(v));
        const auto hit = vehicle_->sensorRaycast(origin, origin + state_.orientation * direction * lidar_range_);
        // Only physical returns are integrated as occupied surfaces. Missing
        // and self-occluded returns do not become surfaces or clearing rays.
        if (!hit.hit || hit.range < .1) continue;
        const Vector point = direction * hit.range;
        const float xyz[] = {float(point.x()), float(point.y()), float(point.z())};
        const auto offset = cloud.data.size();
        cloud.data.resize(offset + 12);
        std::memcpy(cloud.data.data() + offset, xyz, 12);
        ++cloud.width;
      }
    }
    cloud.row_step = cloud.width * cloud.point_step;
    pending_cloud_ = std::move(cloud);
    pending_at_ = Clock::now();
    has_pending_ = true;
    ++scans_;
  }

  std::string payload(const VehicleState& s) const {
    std::ostringstream out;
    out << std::setprecision(10)
        << "{\"physics\":\"Bullet\",\"vehicle\":\"SMB_four_wheel\",\"simulated_seconds\":" << s.simulated_time
        << ",\"wheel_contacts\":" << s.wheel_contacts << ",\"chassis_contacts\":" << s.chassis_contacts
        << ",\"chassis_contact_steps\":" << chassis_contact_steps_ << ",\"commands\":" << commands_
        << ",\"rejected_commands\":" << rejected_commands_ << ",\"scans\":" << scans_
        << ",\"timed_out\":" << (timed_out_ ? "true" : "false")
        << ",\"scene\":{\"infinite_ground\":" << (add_ground_ ? "true" : "false")
        << ",\"room_boxes\":" << (room_ ? "true" : "false")
        << ",\"mesh_vertices\":" << mesh_vertices_ << ",\"mesh_triangles\":" << mesh_triangles_ << "}"
        << ",\"timing\":{\"max_tick_gap_s\":" << max_tick_gap_
        << ",\"late_ticks\":" << late_ticks_
        << ",\"max_physics_s\":" << max_physics_
        << ",\"physics_wall_budget_s\":" << kPhysicsWallBudgetSeconds
        << ",\"physics_backlog_limit_s\":" << kPhysicsBacklogLimitSeconds
        << ",\"physics_budget_exhausted_ticks\":" << physics_budget_exhausted_ticks_
        << ",\"physics_backlog_s\":" << accumulator_
        << ",\"max_physics_backlog_s\":" << max_physics_backlog_
        << ",\"dropped_sim_time_s\":" << dropped_sim_time_
        << ",\"physics_step_measurement\":\"step_state_and_accounting\""
        << ",\"physics_step_timing_samples\":" << physics_step_timing_samples_
        << ",\"physics_step_cpu_samples\":" << physics_step_cpu_samples_
        << ",\"max_physics_step_wall_s\":" << max_physics_step_wall_
        << ",\"max_physics_step_cpu_s\":";
    if (physics_step_cpu_samples_) out << max_physics_step_cpu_; else out << "null";
    out << ",\"max_output_s\":" << max_output_
        << ",\"max_scan_s\":" << max_scan_
        << ",\"scan_timing_samples\":" << scan_timing_samples_
        << ",\"scan_cpu_samples\":" << scan_cpu_samples_
        << ",\"scan_cpu_clock\":\"" << (scan_cpu_samples_ ? "CLOCK_THREAD_CPUTIME_ID" : "unavailable") << '\"'
        << ",\"total_scan_wall_s\":" << total_scan_wall_
        << ",\"total_scan_cpu_s\":";
    if (scan_cpu_samples_) out << total_scan_cpu_; else out << "null";
    out << ",\"max_scan_cpu_s\":";
    if (scan_cpu_samples_) out << max_scan_cpu_; else out << "null";
    out << ",\"cpu_at_max_scan_s\":";
    if (std::isfinite(cpu_at_max_scan_)) out << cpu_at_max_scan_; else out << "null";
    // Completed-tick timing is published on the next output/service response.
    // These observations locate stalls; they do not repair or mask their cause.
    const auto finite_or_null = [&out](double value) {
      if (std::isfinite(value)) out << value; else out << "null";
    };
    out << ",\"tick_timing_samples\":" << tick_timing_samples_
        << ",\"tick_cpu_samples\":" << tick_cpu_samples_
        << ",\"max_tick_wall_s\":" << max_tick_wall_
        << ",\"max_tick_cpu_s\":";
    if (tick_cpu_samples_) out << max_tick_cpu_; else out << "null";
    out << ",\"cpu_at_max_tick_s\":"; finite_or_null(cpu_at_max_tick_);
    out << ",\"pending_publish_timing_samples\":" << pending_publish_timing_samples_
        << ",\"pending_publish_cpu_samples\":" << pending_publish_cpu_samples_
        << ",\"max_pending_publish_wall_s\":" << max_pending_publish_wall_
        << ",\"max_pending_publish_cpu_s\":";
    if (pending_publish_cpu_samples_) out << max_pending_publish_cpu_; else out << "null";
    out << ",\"cpu_at_max_pending_publish_s\":"; finite_or_null(cpu_at_max_pending_publish_);
    out << ",\"max_tick_gap_previous_tick_wall_s\":"; finite_or_null(gap_previous_tick_wall_);
    out << ",\"max_tick_gap_previous_tick_cpu_s\":"; finite_or_null(gap_previous_tick_cpu_);
    out << ",\"max_tick_gap_idle_s\":"; finite_or_null(gap_idle_);
    static constexpr const char* output_names[] = {
        "odometry", "dynamic_tf", "joints", "status_payload", "status_publish"};
    out << ",\"output_subphases\":{";
    for (std::size_t i = 0; i < output_phase_stats_.size(); ++i) {
      if (i) out << ',';
      const auto& stats = output_phase_stats_[i];
      out << '\"' << output_names[i] << "\":{\"samples\":" << stats.samples
          << ",\"cpu_samples\":" << stats.cpu_samples
          << ",\"max_wall_s\":" << stats.max_wall_s << ",\"max_cpu_s\":";
      if (stats.cpu_samples) out << stats.max_cpu_s; else out << "null";
      out << ",\"cpu_at_max_wall_s\":"; finite_or_null(stats.cpu_at_max_wall_s);
      out << '}';
    }
    out << '}';
    // Complete events, oldest first. The current output cannot include its
    // unfinished tick; a later status/service response carries that event.
    out << ",\"slow_ticks\":[";
    const auto phase_value = [&out, &finite_or_null](const PhaseTiming& timing) {
      out << "{\"ran\":" << (timing.ran ? "true" : "false")
          << ",\"wall_s\":" << timing.wall_s << ",\"cpu_s\":";
      finite_or_null(timing.cpu_s);out << '}';
    };
    const auto phase = [&out, &phase_value](const char* name, const PhaseTiming& timing) {
      out << ",\"" << name << "\":";phase_value(timing);
    };
    for (std::size_t i = 0; i < slow_tick_count_; ++i) {
      if (i) out << ',';
      const auto& event = slow_ticks_[(slow_tick_next_ + slow_ticks_.size() - slow_tick_count_ + i) % slow_ticks_.size()];
      out << "{\"tick_id\":" << event.tick_id << ",\"start_ros_stamp_ns\":" << event.start_ros_stamp_ns
          << ",\"physics_steps\":" << event.physics_steps << ",\"entry_gap_s\":" << event.entry_gap_s
          << ",\"wall_s\":" << event.wall_s << ",\"cpu_s\":";
      finite_or_null(event.cpu_s);
      phase("physics", event.physics);phase("output", event.output);
      phase("scan", event.scan);phase("pending_publish", event.pending_publish);
      if (event.output.ran) {
        out << ",\"output_subphases\":{";
        for (std::size_t j = 0; j < event.output_subphases.size(); ++j) {
          if (j) out << ',';
          out << '\"' << output_names[j] << "\":";phase_value(event.output_subphases[j]);
        }
        out << '}';
      }
      out << '}';
    }
    out << ']';
    out << '}' << ",\"position\":[" << s.position.x() << ',' << s.position.y() << ',' << s.position.z() << ']'
        << ",\"speed_mps\":" << s.linear_velocity.norm() << ",\"distance_m\":" << distance_
        << ",\"command_linear\":" << s.command_linear << ",\"command_angular\":" << s.command_angular
        << ",\"angular_velocity_feedback_gain\":" << angular_velocity_feedback_gain_
        << ",\"angular_velocity_correction\":" << s.angular_velocity_correction
        << ",\"applied_angular_command\":" << s.applied_angular_command
        << ",\"wheel_speed\":[";
    for (std::size_t i = 0; i < 4; ++i) { if (i) out << ','; out << s.wheels[i].angular_velocity; }
    out << "],\"wheel_target\":[";
    for (std::size_t i = 0; i < 4; ++i) { if (i) out << ','; out << s.wheels[i].target_angular_velocity; }
    const auto vector = [&out, &finite_or_null](const Vector& value, bool valid = true) {
      out << '[';
      for (int axis = 0; axis < 3; ++axis) {
        if (axis) out << ',';
        if (valid) finite_or_null(value[axis]); else out << "null";
      }
      out << ']';
    };
    const auto measurement = [&out, &finite_or_null](double value, bool valid) {
      if (valid) finite_or_null(value); else out << "null";
    };
    out << "],\"dynamics\":{\"physics_step_s\":";
    finite_or_null(s.last_physics_step);
    out << ",\"sample\":\"last_completed_physics_step\""
        << ",\"joint_effort\":\"total_joint_feedback_on_wheel_projected_on_step_start_hinge_axis_not_motor_only\""
        << ",\"contact_set\":\"manifold_points_with_distance_le_0.002_m\""
        << ",\"contact_impulse\":\"sum_on_wheel_in_current_body_frame_sliding_rows_only_no_rolling_or_spinning_torque\""
        << ",\"slip\":\"post_step_wheel_minus_scene_tangent_velocity_at_local_contact_anchors_unweighted_mean_and_rms\""
        << ",\"body_linear_velocity_mps\":";
    vector(s.body_linear_velocity);
    out << ",\"body_angular_velocity_radps\":";
    vector(s.body_angular_velocity);
    out << ",\"wheels\":[";
    static constexpr const char* names[] = {"LF", "RF", "LH", "RH"};
    for (std::size_t i = 0; i < s.wheels.size(); ++i) {
      if (i) out << ',';
      const auto& wheel = s.wheels[i];
      out << "{\"name\":\"" << names[i] << "\",\"joint_feedback_valid\":"
          << (wheel.joint_feedback_valid ? "true" : "false")
          << ",\"joint_feedback_axis_torque_Nm\":";
      measurement(wheel.joint_axis_torque, wheel.joint_feedback_valid);
      out << ",\"joint_feedback_axis_impulse_Nms\":";
      measurement(wheel.joint_axis_impulse, wheel.joint_feedback_valid);
      out << ",\"configured_motor_impulse_limit_Nms\":";
      measurement(wheel.motor_impulse_limit, wheel.joint_feedback_valid);
      out << ",\"contact_diagnostics_valid\":" << (wheel.contact_diagnostics_valid ? "true" : "false")
          << ",\"contact_points\":" << wheel.contact_points
          << ",\"normal_impulse_sum_Ns\":";
      measurement(wheel.normal_impulse_sum, wheel.contact_diagnostics_valid);
      out << ",\"tangent_impulse_magnitude_sum_Ns\":";
      measurement(wheel.tangent_impulse_magnitude_sum, wheel.contact_diagnostics_valid);
      out << ",\"normal_impulse_body_Ns\":";
      vector(wheel.normal_impulse_body, wheel.contact_diagnostics_valid);
      out << ",\"tangent_impulse_body_Ns\":";
      vector(wheel.tangent_impulse_body, wheel.contact_diagnostics_valid);
      out << ",\"contact_slip_body_mean_mps\":";
      vector(wheel.contact_slip_body_mean, wheel.contact_diagnostics_valid);
      out << ",\"contact_slip_speed_rms_mps\":";
      measurement(wheel.contact_slip_speed_rms, wheel.contact_diagnostics_valid);
      out << '}';
    }
    out << "]}}";
    return out.str();
  }

  void tick() {
    // A process-group SIGINT can be followed by launch's forwarded SIGINT.
    // Keep our handler installed while the triangle BVH and DDS are destroyed;
    // request shutdown from the normal executor thread, never from the handler.
    if (stop_requested) {
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }
    const auto clock = Clock::now();
    const double tick_cpu_started = threadCpuSeconds();
    const double tick_gap = std::chrono::duration<double>(clock-last_tick_).count();
    SlowTick event;event.start_ros_stamp_ns = now().nanoseconds();
    event.tick_id = tick_timing_samples_ + 1;event.entry_gap_s = tick_gap;
    if (tick_gap > max_tick_gap_) {
      max_tick_gap_ = tick_gap;
      if (tick_timing_samples_) {
        gap_previous_tick_wall_ = last_tick_wall_;
        gap_previous_tick_cpu_ = last_tick_cpu_;
        // A large entry gap can be spent inside the previous callback or
        // between callbacks. Preserve the pair from the same maximum event.
        // "Idle" is between ticks, not CPU idle; other callbacks may run here.
        gap_idle_ = std::chrono::duration<double>(clock-last_tick_end_).count();
      }
    }
    if (tick_gap > .5) ++late_ticks_;
    // Bound total debt, including work left by the previous budgeted batch.
    // Record discarded wall-time debt; no unexecuted step advances the vehicle
    // or its simulated clock, position, or velocity.
    const double requested_debt = accumulator_ + std::max(0.0, tick_gap);
    dropped_sim_time_ += std::max(0.0, requested_debt - kPhysicsBacklogLimitSeconds);
    accumulator_ = std::min(kPhysicsBacklogLimitSeconds, requested_debt);
    last_tick_ = clock;
    timed_out_ = !received_command_ || std::chrono::duration<double>(clock - last_command_).count() > command_timeout_;
    const auto physics_started = Clock::now();
    const double physics_cpu_started = threadCpuSeconds();
    while (accumulator_ >= physics_step_) {
      const auto step_started = Clock::now();
      const double step_cpu_started = threadCpuSeconds();
      vehicle_->step(physics_step_, timed_out_ ? 0 : requested_v_, timed_out_ ? 0 : requested_w_);
      ++event.physics_steps;
      accumulator_ -= physics_step_;
      state_ = vehicle_->state();
      distance_ += (state_.position - last_position_).norm();
      last_position_ = state_.position;
      if (state_.chassis_contacts) ++chassis_contact_steps_;
      const double step_cpu = threadCpuSeconds() - step_cpu_started;
      const double step_wall = std::chrono::duration<double>(Clock::now()-step_started).count();
      ++physics_step_timing_samples_;
      max_physics_step_wall_ = std::max(max_physics_step_wall_, step_wall);
      if (std::isfinite(step_cpu) && step_cpu >= 0.) {
        ++physics_step_cpu_samples_;
        max_physics_step_cpu_ = std::max(max_physics_step_cpu_, step_cpu);
      }
      // Always finish one due step and its state/accounting before yielding.
      // A slow single step may exceed this threshold; it is never shortened.
      if (accumulator_ >= physics_step_ &&
          std::chrono::duration<double>(Clock::now()-physics_started).count() >= kPhysicsWallBudgetSeconds) {
        ++physics_budget_exhausted_ticks_;
        break;
      }
    }
    // Both backlog observations describe debt remaining after physics work.
    max_physics_backlog_ = std::max(max_physics_backlog_, accumulator_);
    const double physics_cpu = threadCpuSeconds() - physics_cpu_started;
    event.physics = {true, std::chrono::duration<double>(Clock::now()-physics_started).count(),
        std::isfinite(physics_cpu) && physics_cpu >= 0. ? physics_cpu : std::numeric_limits<double>::quiet_NaN()};
    max_physics_ = std::max(max_physics_, std::chrono::duration<double>(Clock::now()-clock).count());
    if (has_pending_ && std::chrono::duration<double>(clock - pending_at_).count() >= .025) {
      const auto publish_started = Clock::now();
      const double publish_cpu_started = threadCpuSeconds();
      cloud_pub_->publish(pending_cloud_);
      const double publish_cpu = threadCpuSeconds() - publish_cpu_started;
      const double publish_wall = std::chrono::duration<double>(Clock::now()-publish_started).count();
      event.pending_publish = {true, publish_wall,
          std::isfinite(publish_cpu) && publish_cpu >= 0. ? publish_cpu : std::numeric_limits<double>::quiet_NaN()};
      ++pending_publish_timing_samples_;
      if (std::isfinite(publish_cpu) && publish_cpu >= 0.) {
        ++pending_publish_cpu_samples_;
        max_pending_publish_cpu_ = std::max(max_pending_publish_cpu_, publish_cpu);
      }
      if (publish_wall > max_pending_publish_wall_) {
        max_pending_publish_wall_ = publish_wall;
        cpu_at_max_pending_publish_ = std::isfinite(publish_cpu) && publish_cpu >= 0. ? publish_cpu :
            std::numeric_limits<double>::quiet_NaN();
      }
      has_pending_ = false;
    }
    if (std::chrono::duration<double>(clock - last_output_).count() >= .02) {
      const auto stamp = now();
      const auto output_started = Clock::now();
      const double output_cpu_started = threadCpuSeconds();
      output(stamp, event.output_subphases);
      const double output_cpu = threadCpuSeconds() - output_cpu_started;
      event.output = {true, std::chrono::duration<double>(Clock::now()-output_started).count(),
          std::isfinite(output_cpu) && output_cpu >= 0. ? output_cpu : std::numeric_limits<double>::quiet_NaN()};
      max_output_ = std::max(max_output_, event.output.wall_s);
      last_output_ = clock;
      if (!has_pending_ && std::chrono::duration<double>(clock - last_lidar_).count() >= lidar_period_) {
        const auto scan_started = Clock::now();
        const double scan_cpu_started = threadCpuSeconds();
        scan(stamp);
        const double scan_cpu = threadCpuSeconds() - scan_cpu_started;
        const double scan_wall = std::chrono::duration<double>(Clock::now()-scan_started).count();
        event.scan = {true, scan_wall,
            std::isfinite(scan_cpu) && scan_cpu >= 0. ? scan_cpu : std::numeric_limits<double>::quiet_NaN()};
        ++scan_timing_samples_;
        total_scan_wall_ += scan_wall;
        if (std::isfinite(scan_cpu) && scan_cpu >= 0.) {
          ++scan_cpu_samples_;
          total_scan_cpu_ += scan_cpu;
          max_scan_cpu_ = std::max(max_scan_cpu_, scan_cpu);
        }
        if (scan_wall > max_scan_) {
          max_scan_ = scan_wall;
          cpu_at_max_scan_ = std::isfinite(scan_cpu) && scan_cpu >= 0. ? scan_cpu :
              std::numeric_limits<double>::quiet_NaN();
        }
        last_lidar_ = clock;
      }
    }
    const double tick_cpu = threadCpuSeconds() - tick_cpu_started;
    last_tick_end_ = Clock::now();
    last_tick_wall_ = std::chrono::duration<double>(last_tick_end_-clock).count();
    last_tick_cpu_ = std::isfinite(tick_cpu) && tick_cpu >= 0. ? tick_cpu :
        std::numeric_limits<double>::quiet_NaN();
    ++tick_timing_samples_;
    if (std::isfinite(last_tick_cpu_)) {
      ++tick_cpu_samples_;
      max_tick_cpu_ = std::max(max_tick_cpu_, last_tick_cpu_);
    }
    if (last_tick_wall_ > max_tick_wall_) {
      max_tick_wall_ = last_tick_wall_;
      cpu_at_max_tick_ = last_tick_cpu_;
    }
    if (last_tick_wall_ >= .05 || tick_gap >= .1) {
      event.wall_s = last_tick_wall_;event.cpu_s = last_tick_cpu_;
      slow_ticks_[slow_tick_next_] = event;
      slow_tick_next_ = (slow_tick_next_ + 1) % slow_ticks_.size();
      slow_tick_count_ = std::min(slow_tick_count_ + 1, slow_ticks_.size());
    }
  }

  std::unique_ptr<SmbVehicle> vehicle_;
  VehicleState state_;
  Vector sensor_offset_, last_position_;
  std::string frame_, robot_frame_, sensor_frame_;
  bool add_ground_{true}, room_{true};
  std::size_t mesh_vertices_{0}, mesh_triangles_{0};
  double physics_step_, command_timeout_, lidar_period_, lidar_range_;
  double angular_velocity_feedback_gain_ = 0.;
  double requested_v_ = 0, requested_w_ = 0, accumulator_ = 0, distance_ = 0;
  double max_physics_backlog_ = 0, dropped_sim_time_ = 0;
  double max_physics_step_wall_ = 0, max_physics_step_cpu_ = 0;
  uint64_t physics_budget_exhausted_ticks_ = 0;
  uint64_t physics_step_timing_samples_ = 0, physics_step_cpu_samples_ = 0;
  double max_tick_gap_ = 0, max_physics_ = 0, max_output_ = 0, max_scan_ = 0;
  double total_scan_wall_ = 0, total_scan_cpu_ = 0, max_scan_cpu_ = 0;
  double cpu_at_max_scan_ = std::numeric_limits<double>::quiet_NaN();
  double max_tick_wall_ = 0, max_tick_cpu_ = 0;
  std::array<OutputPhaseStats, kOutputPhases> output_phase_stats_{};
  double max_pending_publish_wall_ = 0, max_pending_publish_cpu_ = 0;
  double cpu_at_max_tick_ = std::numeric_limits<double>::quiet_NaN();
  double cpu_at_max_pending_publish_ = std::numeric_limits<double>::quiet_NaN();
  double last_tick_wall_ = std::numeric_limits<double>::quiet_NaN();
  double last_tick_cpu_ = std::numeric_limits<double>::quiet_NaN();
  double gap_previous_tick_wall_ = std::numeric_limits<double>::quiet_NaN();
  double gap_previous_tick_cpu_ = std::numeric_limits<double>::quiet_NaN();
  double gap_idle_ = std::numeric_limits<double>::quiet_NaN();
  uint64_t scan_timing_samples_ = 0, scan_cpu_samples_ = 0;
  uint64_t tick_timing_samples_ = 0, tick_cpu_samples_ = 0;
  uint64_t pending_publish_timing_samples_ = 0, pending_publish_cpu_samples_ = 0;
  std::array<SlowTick, 8> slow_ticks_{};
  std::size_t slow_tick_next_{0}, slow_tick_count_{0};
  uint64_t late_ticks_ = 0;
  bool received_command_ = false, timed_out_ = true, has_pending_ = false;
  Clock::time_point last_tick_, last_tick_end_, last_command_, last_output_, last_lidar_, pending_at_;
  uint64_t commands_ = 0, rejected_commands_ = 0, scans_ = 0, chassis_contact_steps_ = 0;
  sensor_msgs::msg::PointCloud2 pending_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
};
}  // namespace gbplanner_smb

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  try { rclcpp::spin(std::make_shared<gbplanner_smb::SmbNode>()); }
  catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("gbplanner_smb"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
