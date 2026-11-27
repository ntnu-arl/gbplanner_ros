// Standalone diagnostic: link this translation unit to smb_vehicle and
// pure_pursuit. It deliberately adds no dependency between the ROS packages.
#include "gbplanner_smb/smb_vehicle.hpp"
#include "gbplanner_tracking/pure_pursuit.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using gbplanner_smb::SmbVehicle;
using gbplanner_smb::VehicleParams;
using gbplanner_smb::VehicleState;
using gbplanner_tracking::Pose;
using gbplanner_tracking::TerminalMode;

namespace {
constexpr double kPhysicsStep = 0.002;
constexpr double kControlStep = 0.05;
constexpr int kControlSubsteps = 25;
constexpr int kSettleSteps = 500;
constexpr int kBudgetSteps = 22500;  // 45 s, excluding the 1 s initial settling.
constexpr double kPositionTolerance = 0.04;
constexpr double kYawTolerance = 0.1;
constexpr double kRequestedSpeed = 0.3;

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

Pose pose(const VehicleState& state) {
  const auto rotation = state.orientation.toRotationMatrix();
  return {state.position.x(), state.position.y(), state.position.z(),
          std::atan2(rotation(1, 0), rotation(0, 0))};
}

Eigen::Vector3d bodyLinear(const VehicleState& state) {
  return state.orientation.conjugate() * state.linear_velocity;
}

Eigen::Vector3d bodyAngular(const VehicleState& state) {
  return state.orientation.conjugate() * state.angular_velocity;
}

const char* modeName(TerminalMode mode) {
  switch (mode) {
    case TerminalMode::Tracking: return "TRACKING";
    case TerminalMode::Aligning: return "ALIGNING";
    case TerminalMode::Reached: return "REACHED";
  }
  throw std::logic_error("invalid terminal mode");
}

void requireFinite(const VehicleState& state) {
  if (!state.position.allFinite() || !state.orientation.coeffs().allFinite() ||
      !state.linear_velocity.allFinite() || !state.angular_velocity.allFinite())
    throw std::runtime_error("nonfinite Bullet vehicle state");
  for (const auto& wheel : state.wheels)
    if (!std::isfinite(wheel.angular_velocity) ||
        !std::isfinite(wheel.target_angular_velocity))
      throw std::runtime_error("nonfinite Bullet wheel state");
}

void poseJson(std::ostream& out, const Pose& value) {
  out << '[' << value[0] << ", " << value[1] << ", " << value[2] << ", " << value[3] << ']';
}

std::vector<Pose> readRoute(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open route CSV: " + path.string());
  std::vector<Pose> route;
  std::string line;
  std::size_t line_number = 0;
  bool first_content = true;
  while (std::getline(input, line)) {
    ++line_number;
    std::string compact = line;
    compact.erase(std::remove_if(compact.begin(), compact.end(),
                                [](unsigned char c) { return std::isspace(c); }), compact.end());
    if (compact.empty()) continue;
    const bool header = first_content && compact == "x,y,z,yaw";
    first_content = false;
    if (header) continue;
    const std::string error = "invalid four-column route CSV at line " + std::to_string(line_number);
    if (std::count(line.begin(), line.end(), ',') != 3) throw std::invalid_argument(error);
    std::istringstream fields(line);
    Pose point;
    for (int axis = 0; axis < 4; ++axis) {
      std::string field;
      if (!std::getline(fields, field, ',')) throw std::invalid_argument(error);
      std::istringstream value(field);
      if (!(value >> point[axis]) || !std::isfinite(point[axis])) throw std::invalid_argument(error);
      value >> std::ws;
      if (!value.eof()) throw std::invalid_argument(error);
    }
    route.push_back(point);
  }
  if (input.bad()) throw std::runtime_error("failed reading route CSV: " + path.string());
  if (route.size() < 2) throw std::invalid_argument("route CSV requires at least two poses");
  return route;
}

void writeCsv(std::ostream& csv, const char* phase, double route_time,
              bool control_tick, TerminalMode mode, const VehicleState& state,
              const Pose& goal, double requested_v, double requested_w,
              double integral_velocity = 0.0, int recovery_direction = 0) {
  const Pose current = pose(state);
  const double dx = goal.x() - current.x(), dy = goal.y() - current.y();
  const double c = std::cos(current[3]), s = std::sin(current[3]);
  const auto linear = bodyLinear(state), angular = bodyAngular(state);
  csv << phase << ',' << state.simulated_time << ',' << route_time << ','
      << (control_tick ? 1 : 0) << ',' << modeName(mode)
      << ',' << current.x() << ',' << current.y() << ',' << current.z() << ',' << current[3]
      << ',' << std::hypot(dx, dy) << ',' << wrap(goal[3] - current[3])
      << ',' << c * dx + s * dy << ',' << -s * dx + c * dy;
  for (const auto& vector : {linear, angular})
    csv << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
  csv << ',' << requested_v << ',' << requested_w
      << ',' << state.command_linear << ',' << state.command_angular
      << ',' << state.wheel_contacts << ',' << state.chassis_contacts;
  for (const auto& wheel : state.wheels)
    csv << ',' << wheel.angular_velocity << ',' << wheel.target_angular_velocity
        << ',' << (wheel.contact ? 1 : 0);
  csv << ',' << integral_velocity << ',' << recovery_direction << ',' << state.last_physics_step
      << ',' << state.applied_angular_command << ',' << state.angular_velocity_correction;
  for (const auto& wheel : state.wheels) {
    csv << ',' << wheel.joint_feedback_valid << ',' << wheel.joint_axis_torque
        << ',' << wheel.joint_axis_impulse << ',' << wheel.motor_impulse_limit
        << ',' << wheel.contact_diagnostics_valid << ',' << wheel.contact_points
        << ',' << wheel.normal_impulse_sum << ',' << wheel.tangent_impulse_magnitude_sum;
    for (const auto& vector : {wheel.normal_impulse_body, wheel.tangent_impulse_body,
                              wheel.contact_slip_body_mean})
      csv << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
    csv << ',' << wheel.contact_slip_speed_rms;
  }
  csv << '\n';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc > 6)
      throw std::invalid_argument("usage: smb_terminal_replay [output_directory [route.csv [integral_gain [legacy|polar|recovery|position [vehicle_yaw_feedback_gain]]]]]");
    const std::filesystem::path output = argc >= 2 ? argv[1] : "terminal_alignment_replay";
    const std::filesystem::path route_csv = argc >= 3 ? std::filesystem::absolute(argv[2]).lexically_normal() :
                                                       std::filesystem::path{};
    double integral_gain = 0.0;
    if (argc >= 4) {
      const std::string argument = argv[3];
      std::size_t consumed = 0;
      integral_gain = std::stod(argument, &consumed);
      if (!std::isfinite(integral_gain) || integral_gain < 0.0 ||
          argument.find_first_not_of(" \t\r\n", consumed) != std::string::npos)
        throw std::invalid_argument("integral_gain must be finite and nonnegative");
    }
    const std::string terminal_mode = argc >= 5 ? argv[4] : "legacy";
    if (terminal_mode != "legacy" && terminal_mode != "polar" &&
        terminal_mode != "recovery" && terminal_mode != "position")
      throw std::invalid_argument("terminal_mode must be legacy, polar, recovery or position");
    const bool experimental_polar = terminal_mode == "polar";
    const bool position_recovery = terminal_mode == "position";
    const bool latched_recovery = terminal_mode == "recovery" || position_recovery;
    if (experimental_polar && integral_gain != 0.0)
      throw std::invalid_argument("experimental polar mode requires integral_gain=0");
    std::filesystem::create_directories(output);
    std::ofstream csv(output / "trajectory.csv");
    if (!csv) throw std::runtime_error("cannot open trajectory.csv");
    csv << std::setprecision(17)
        << "phase,simulation_time_s,route_time_s,control_tick,mode,x_m,y_m,z_m,yaw_rad,"
           "distance_to_goal_m,yaw_error_rad,body_goal_x_m,body_goal_y_m,"
           "body_vx_mps,body_vy_mps,body_vz_mps,body_wx_radps,body_wy_radps,body_wz_radps,"
           "requested_v_mps,requested_w_radps,limited_v_mps,limited_w_radps,"
           "wheel_contacts,chassis_contacts,lf_velocity_radps,lf_target_radps,lf_contact,"
           "rf_velocity_radps,rf_target_radps,rf_contact,lh_velocity_radps,lh_target_radps,"
           "lh_contact,rh_velocity_radps,rh_target_radps,rh_contact,integral_velocity_mps,recovery_direction,last_physics_step_s,"
           "applied_angular_command_radps,angular_velocity_correction_radps";
    for (const char* wheel : {"lf", "rf", "lh", "rh"}) {
      for (const char* field : {"joint_feedback_valid", "joint_axis_torque_Nm", "joint_axis_impulse_Nms",
            "motor_impulse_limit_Nms", "contact_diagnostics_valid", "contact_points",
            "normal_impulse_sum_Ns", "tangent_impulse_magnitude_sum_Ns",
            "normal_impulse_body_x_Ns", "normal_impulse_body_y_Ns", "normal_impulse_body_z_Ns",
            "tangent_impulse_body_x_Ns", "tangent_impulse_body_y_Ns", "tangent_impulse_body_z_Ns",
            "slip_body_x_mps", "slip_body_y_mps", "slip_body_z_mps", "slip_speed_rms_mps"})
        csv << ',' << wheel << '_' << field;
    }
    csv << '\n';

    // Exact first local segment recorded by smb_autonomy/run8. Only initial
    // spawn height differs, as in check_smb_tracking.py --terminal-turn.
    const std::vector<Pose> route = argc >= 3 ? readRoute(route_csv) : std::vector<Pose>{
        Pose(-0.8389034867286682, 0.0016545632388442755, 0.2189998217821122,
             -0.0506260673246966),
        Pose(-1.0097428275770157, 0.1046982295880039, 0.18682535042036869,
             2.5988519635603526)};
    const Pose& goal = route.back();
    VehicleParams vehicle_params;
    if (argc >= 6) {
      const std::string argument = argv[5];
      std::size_t consumed = 0;
      vehicle_params.angular_velocity_feedback_gain = std::stod(argument, &consumed);
      if (!std::isfinite(vehicle_params.angular_velocity_feedback_gain) ||
          vehicle_params.angular_velocity_feedback_gain < 0.0 ||
          argument.find_first_not_of(" \t\r\n", consumed) != std::string::npos)
        throw std::invalid_argument("vehicle_yaw_feedback_gain must be finite and nonnegative");
    }
    vehicle_params.initial_position = {route.front().x(), route.front().y(), 0.26};
    vehicle_params.initial_orientation = Eigen::AngleAxisd(route.front()[3], Eigen::Vector3d::UnitZ());
    SmbVehicle vehicle(vehicle_params);
    for (int i = 0; i < kSettleSteps; ++i) {
      vehicle.step(kPhysicsStep, 0.0, 0.0);
      const auto state = vehicle.state();
      requireFinite(state);
      writeCsv(csv, "settling", (i + 1 - kSettleSteps) * kPhysicsStep,
               false, TerminalMode::Tracking, state, goal, 0.0, 0.0);
    }
    const VehicleState start = vehicle.state();

    // Match the actual ROS node defaults and the terminal-turn probe overrides.
    gbplanner_tracking::Parameters parameters;
    parameters.critical_angle = 0.52;
    parameters.dt = kControlStep;
    parameters.desired_vel = kRequestedSpeed;
    parameters.max_lin_vel = 0.5;
    parameters.max_ang_vel = 0.5;
    parameters.look_ahead_distance = 0.5;
    parameters.look_ahead_error_margin = 0.2;
    parameters.goal_reaching_threshold = kPositionTolerance;
    gbplanner_tracking::PurePursuitController controller(parameters);
    controller.setPath(route, kRequestedSpeed);
    gbplanner_tracking::TerminalPosePolicy policy(kPositionTolerance, kYawTolerance);
    gbplanner_tracking::TerminalAlignmentController terminal(integral_gain);
    terminal.reset();
    TerminalMode mode = TerminalMode::Tracking;
    VehicleState finish = start;
    double requested_v = 0.0, requested_w = 0.0;
    double elapsed = 0.0, first_alignment = -1.0;
    double maximum_requested_v = 0.0, maximum_requested_w = 0.0;
    double maximum_integral_velocity = 0.0;
    bool polar_active = false;
    int polar_direction = 0;
    double polar_activated_at = -1.0;
    double path_length = 0.0;
    int alignment_entries = 0, alignment_exits = 0, samples = 0, control_updates = 0;
    int chassis_contact_samples = 0, four_wheel_contact_samples = 0;
    std::size_t min_wheel_contacts = 4, max_chassis_contacts = 0;
    Eigen::Vector2d previous_xy = start.position.head<2>();

    // Sample before each physical step. At every 25th sample the newly issued
    // command and the state that produced it appear on the same CSV row.
    // Motor-limited commands are the result of the preceding physics step.
    for (int i = 0; i <= kBudgetSteps; ++i) {
      finish = vehicle.state();
      requireFinite(finish);
      const Pose current = pose(finish);
      elapsed = i * kPhysicsStep;
      const bool control_tick = i % kControlSubsteps == 0;
      if (control_tick) {
        ++control_updates;
        const double distance = (current.head<2>() - goal.head<2>()).norm();
        const double yaw_error = wrap(goal[3] - current[3]);
        const TerminalMode previous_mode = mode;
        mode = latched_recovery ? policy.update(current, goal) : policy.update(distance, yaw_error);
        const auto tracking = controller.step(current, policy.recoveryDirection());
        if (mode == TerminalMode::Aligning && previous_mode != mode) {
          ++alignment_entries;
          if (first_alignment < 0.0) first_alignment = elapsed;
          if (experimental_polar && !polar_active) {
            const double bearing = std::atan2(goal.y() - current.y(), goal.x() - current.x());
            polar_direction = std::cos(wrap(bearing - current[3])) >= 0.0 ? 1 : -1;
            polar_active = true;
            polar_activated_at = elapsed;
          }
        }
        if (previous_mode == TerminalMode::Aligning && mode == TerminalMode::Tracking)
          ++alignment_exits;
        if (previous_mode == TerminalMode::Aligning && mode != TerminalMode::Aligning)
          terminal.reset();
        requested_v = tracking.v;
        requested_w = tracking.w;
        if (mode == TerminalMode::Reached) {
          requested_v = 0.0;
          requested_w = 0.0;
        } else if (polar_active) {
          // Isolated replay experiment: latch direction once, retain the
          // original terminal policy and do not change the production law.
          if (distance <= 1.e-6) {
            requested_v = 0.0;
            requested_w = std::clamp(2.0 * yaw_error, -0.5, 0.5);
          } else {
            const double bearing = std::atan2(goal.y() - current.y(), goal.x() - current.x());
            const double offset = polar_direction < 0 ? std::acos(-1.0) : 0.0;
            const double alpha = wrap(bearing - (current[3] + offset));
            const double beta = wrap((goal[3] + offset) - bearing);
            requested_v = polar_direction * std::min(0.1, 1.0 * distance) * std::max(0.0, std::cos(alpha));
            requested_w = std::clamp(2.0 * alpha - 0.5 * beta, -0.5, 0.5);
          }
        } else if (mode == TerminalMode::Aligning) {
          const auto alignment = terminal.step(
              current, goal, bodyLinear(finish).x(), controller.speed(), parameters.max_ang_vel, kControlStep);
          requested_v = alignment.v;
          requested_w = alignment.w;
        } else if (position_recovery && policy.recoveryDirection()) {
          const auto correction = gbplanner_tracking::terminalPositionCommand(
              current, goal, *policy.recoveryDirection(), controller.speed(),
              parameters.max_ang_vel, parameters.critical_angle);
          requested_v = correction.v;
          requested_w = correction.w;
        }
      }
      ++samples;
      min_wheel_contacts = std::min(min_wheel_contacts, finish.wheel_contacts);
      max_chassis_contacts = std::max(max_chassis_contacts, finish.chassis_contacts);
      chassis_contact_samples += finish.chassis_contacts > 0 ? 1 : 0;
      four_wheel_contact_samples += finish.wheel_contacts == 4 ? 1 : 0;
      maximum_requested_v = std::max(maximum_requested_v, std::abs(requested_v));
      maximum_requested_w = std::max(maximum_requested_w, std::abs(requested_w));
      maximum_integral_velocity = std::max(maximum_integral_velocity, std::abs(terminal.integralVelocity()));
      path_length += (finish.position.head<2>() - previous_xy).norm();
      previous_xy = finish.position.head<2>();
      writeCsv(csv, "route", elapsed, control_tick, mode, finish, goal, requested_v, requested_w,
               terminal.integralVelocity(), policy.recoveryDirection().value_or(0));
      if (mode == TerminalMode::Reached || i == kBudgetSteps) break;
      vehicle.step(kPhysicsStep, requested_v, requested_w);
    }

    const Pose final_pose = pose(finish);
    const double final_distance = (final_pose.head<2>() - goal.head<2>()).norm();
    const double final_yaw_error = wrap(goal[3] - final_pose[3]);
    // No epsilon or widened exit band is allowed in the success criterion.
    // Both errors refer to this same physical chassis pose, never a command.
    const bool reached = mode == TerminalMode::Reached &&
        final_distance <= kPositionTolerance && std::abs(final_yaw_error) <= kYawTolerance;
    const bool contact_valid = chassis_contact_samples == 0;
    const bool passed = reached && contact_valid;
    std::ostringstream summary;
    summary << std::boolalpha << std::setprecision(17)
        << "{\n  \"case\": \"" << (argc >= 3 ? "csv_route" : "run8_terminal_turn") << "\","
           "\n  \"scope\": \"SmbVehicle Bullet motors/contact + pure pursuit + terminal pose policy"
        << (experimental_polar ? "; experimental latched polar feedback, replay only" : "") << "\","
           "\n  \"terminal_mode\": " << std::quoted(terminal_mode)
        << ",\n  \"experimental_polar\": " << experimental_polar
        << ",\n  \"latched_position_recovery\": " << latched_recovery
        << ",\n  \"independent_position_recovery\": " << position_recovery
        << ",\n  \"terminal_adaptation\": \""
        << (experimental_polar ? "experimental latched polar pose feedback" :
            position_recovery ? "latched bearing/distance position recovery; body-X position feedback with measured-velocity damping during yaw alignment" :
                                "body-X position feedback with measured-velocity damping")
        << (integral_gain > 0.0 ? " and explicit velocity integral" : "") << "\","
           "\n  \"adaptation_is_upstream_algorithm\": false,"
           "\n  \"trajectory\": \"trajectory.csv\","
           "\n  \"route_csv\": ";
    if (argc >= 3) summary << std::quoted(route_csv.string());
    else summary << "null";
    summary << ",\n  \"route_points\": " << route.size()
        << ",\n  \"initial_state_source\": \"fresh Bullet spawn at first route XY/yaw, z=0.26, followed by 1 s settling\","
           "\n  \"historical_wheel_velocities_restored\": false,"
           "\n  \"physics_step_s\": " << kPhysicsStep
        << ",\n  \"control_step_s\": " << kControlStep
        << ",\n  \"settle_duration_s\": " << kSettleSteps * kPhysicsStep
        << ",\n  \"control_budget_s\": " << kBudgetSteps * kPhysicsStep
        << ",\n  \"position_tolerance_m\": " << kPositionTolerance
        << ",\n  \"yaw_tolerance_rad\": " << kYawTolerance
        << ",\n  \"alignment_exit_distance_m\": " << policy.alignmentExitDistance()
        << ",\n  \"polar_configuration\": ";
    if (experimental_polar) {
      summary << "{\"rho_gain\": 1.0, \"alpha_gain\": 2.0, \"beta_gain\": -0.5,"
                 " \"linear_limit_mps\": 0.1, \"angular_limit_radps\": 0.5,"
                 " \"singularity_radius_m\": 1e-6, \"activated\": " << polar_active
              << ", \"locked_direction\": " << polar_direction << ", \"activated_at_s\": ";
      if (polar_active) summary << polar_activated_at;
      else summary << "null";
      summary << '}';
    } else summary << "null";
    summary
        << ",\n  \"terminal_position_gain_per_s\": 2,"
           "\n  \"terminal_velocity_damping\": 0.5,"
           "\n  \"terminal_velocity_integral_gain\": " << terminal.integralGain()
        << ",\n  \"final_integral_velocity_mps\": " << terminal.integralVelocity()
        << ",\n  \"maximum_absolute_integral_velocity_mps\": " << maximum_integral_velocity
        << ",\n  \"terminal_linear_limit_mps\": " << std::min(controller.speed(), 0.1)
        << ",\n  \"active_speed_mps\": " << controller.speed()
        << ",\n  \"critical_angle_rad\": " << parameters.critical_angle
        << ",\n  \"curvature_scaling_angle_rad\": " << parameters.curvature_scaling_angle
        << ",\n  \"lookahead_m\": " << parameters.look_ahead_distance
        << ",\n  \"lookahead_error_margin_m\": " << parameters.look_ahead_error_margin
        << ",\n  \"max_linear_velocity_mps\": " << parameters.max_lin_vel
        << ",\n  \"max_angular_velocity_radps\": " << parameters.max_ang_vel
        << ",\n  \"vehicle_parameters\": {\"wheel_friction\": " << vehicle_params.wheel_friction
        << ", \"ground_friction\": " << vehicle_params.ground_friction
        << ", \"rolling_friction\": " << vehicle_params.rolling_friction
        << ", \"spinning_friction\": " << vehicle_params.spinning_friction
        << ", \"max_wheel_torque_Nm\": " << vehicle_params.max_wheel_torque
        << ", \"wheel_radius_m\": " << vehicle_params.wheel_radius
        << ", \"wheel_track_m\": " << vehicle_params.wheel_track << "},"
        << "\n  \"vehicle_yaw_feedback_gain_per_s\": " << vehicle_params.angular_velocity_feedback_gain << ','
        << "\n  \"requested_route_xyz_yaw\": [";
    for (std::size_t i = 0; i < route.size(); ++i) {
      if (i) summary << ", ";
      poseJson(summary, route[i]);
    }
    summary << "],\n  \"start_xyz_yaw\": ";
    poseJson(summary, pose(start));
    summary << ",\n  \"final_xyz_yaw\": ";
    poseJson(summary, final_pose);
    summary << ",\n  \"final_mode\": \"" << modeName(mode) << "\","
        << "\n  \"elapsed_s\": " << elapsed
        << ",\n  \"first_alignment_s\": ";
    if (first_alignment < 0.0) summary << "null";
    else summary << first_alignment;
    summary << ",\n  \"alignment_entries\": " << alignment_entries
        << ",\n  \"alignment_exits_to_tracking\": " << alignment_exits
        << ",\n  \"control_updates\": " << control_updates
        << ",\n  \"route_samples\": " << samples
        << ",\n  \"route_path_length_xy_m\": " << path_length
        << ",\n  \"final_distance_m\": " << final_distance
        << ",\n  \"final_yaw_error_rad\": " << final_yaw_error
        << ",\n  \"maximum_requested_v_mps\": " << maximum_requested_v
        << ",\n  \"maximum_requested_w_radps\": " << maximum_requested_w
        << ",\n  \"min_wheel_contacts\": " << min_wheel_contacts
        << ",\n  \"four_wheel_contact_ratio\": " << double(four_wheel_contact_samples) / samples
        << ",\n  \"chassis_contact_samples\": " << chassis_contact_samples
        << ",\n  \"maximum_chassis_contacts\": " << max_chassis_contacts
        << ",\n  \"reached\": " << reached
        << ",\n  \"contact_valid\": " << contact_valid
        << ",\n  \"passed\": " << passed << "\n}\n";
    std::ofstream report(output / "summary.json");
    if (!report) throw std::runtime_error("cannot open summary.json");
    report << summary.str();
    report.flush();
    csv.flush();
    if (!report || !csv) throw std::runtime_error("failed writing replay outputs");
    std::cout << summary.str();
    return passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
