// Standalone fixed-command response probe. Link only to smb_vehicle/Bullet;
// no ROS node, planner, gain search, or chassis pose/velocity assignment.
#include "gbplanner_smb/smb_vehicle.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using gbplanner_smb::SmbVehicle;
using gbplanner_smb::VehicleParams;
using gbplanner_smb::VehicleState;
constexpr double kStep = 0.002;
constexpr int kSettleSteps = 500;

double yaw(const VehicleState& s) {
  const auto r = s.orientation.toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
Eigen::Vector3d bodyLinear(const VehicleState& s) {
  return s.orientation.conjugate() * s.linear_velocity;
}
Eigen::Vector3d bodyAngular(const VehicleState& s) {
  return s.orientation.conjugate() * s.angular_velocity;
}
void finite(const VehicleState& s) {
  if (!s.position.allFinite() || !s.orientation.coeffs().allFinite() ||
      !s.linear_velocity.allFinite() || !s.angular_velocity.allFinite() ||
      !std::isfinite(s.command_linear) || !std::isfinite(s.command_angular) ||
      !std::isfinite(s.applied_angular_command) ||
      !std::isfinite(s.angular_velocity_correction))
    throw std::runtime_error("nonfinite vehicle state");
  for (const auto& w : s.wheels)
    if (!std::isfinite(w.target_angular_velocity) || !std::isfinite(w.angular_velocity))
      throw std::runtime_error("nonfinite wheel state");
}

struct Range {
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  void add(double value) { minimum = std::min(minimum, value); maximum = std::max(maximum, value); }
};
struct Invariants {
  std::size_t samples = 0, wheel_values_checked = 0, legacy_mapping_mismatches = 0;
  std::size_t legacy_setpoint_mismatches = 0, applied_limit_violations = 0;
  double maximum_legacy_mapping_error = 0.0;

  void check(const VehicleState& s, const VehicleParams& p) {
    finite(s);
    ++samples;
    if (std::abs(s.applied_angular_command) > p.max_angular_velocity + 1e-12)
      ++applied_limit_violations;
    if (p.angular_velocity_feedback_gain != 0.0) return;
    if (s.applied_angular_command != s.command_angular || s.angular_velocity_correction != 0.0)
      ++legacy_setpoint_mismatches;
    for (std::size_t i = 0; i < s.wheels.size(); ++i) {
      const double side = i % 2 == 0 ? -1.0 : 1.0;
      // Keep the original source operation order; compare the stored double,
      // not Bullet's float motor/actual angular velocity.
      const double expected = (s.command_linear + side * s.command_angular * p.wheel_track * 0.5) /
                              p.wheel_radius;
      const double actual = s.wheels[i].target_angular_velocity;
      ++wheel_values_checked;
      if (actual != expected) ++legacy_mapping_mismatches;
      maximum_legacy_mapping_error = std::max(maximum_legacy_mapping_error, std::abs(actual - expected));
    }
  }
  bool passed() const {
    return legacy_mapping_mismatches == 0 && legacy_setpoint_mismatches == 0 &&
           applied_limit_violations == 0;
  }
};

struct Phase {
  const char* name;
  double v, w;
  int steps;
};
constexpr std::array<Phase, 4> kPhases{{
    {"positive_yaw", 0.1, 0.1, 4000}, {"brake_after_positive", 0.0, 0.0, 1000},
    {"negative_yaw", 0.1, -0.1, 4000}, {"brake_after_negative", 0.0, 0.0, 1000}}};

struct PhaseResult {
  Phase phase;
  VehicleState start, finish;
  double body_wz_integral = 0.0, yaw_change = 0.0;
  double limited_yaw_integral = 0.0, applied_yaw_integral = 0.0;
  double body_vy_integral = 0.0, absolute_body_vy_integral = 0.0;
  double tail_wz_integral = 0.0;
  Eigen::Vector2d nominal_xy = Eigen::Vector2d::Zero();
  double nominal_yaw = 0.0;
  Range applied, correction, limited_w;
  std::size_t four_contact_samples = 0, chassis_contact_samples = 0;
  std::size_t saturated_samples = 0, zero_limited_command_samples = 0;
  std::size_t nonzero_correction_after_zero_command = 0;
  std::size_t minimum_contacts = 4, maximum_chassis_contacts = 0;
};

// The nominal arc is driven by the limited requested twist, not the corrected
// wheel setpoint. Its endpoint error includes tracking/slip and makes the
// nonzero world-lateral displacement of a commanded turn distinguishable
// from lateral drift relative to that intended turn.
PhaseResult runPhase(SmbVehicle& vehicle, const VehicleParams& params,
                     const Phase& phase, Invariants& checks) {
  PhaseResult r{phase};
  r.start = vehicle.state();
  r.nominal_xy = r.start.position.head<2>();
  r.nominal_yaw = yaw(r.start);
  VehicleState previous = r.start;
  for (int i = 0; i < phase.steps; ++i) {
    vehicle.step(kStep, phase.v, phase.w);
    const VehicleState state = vehicle.state();
    checks.check(state, params);
    const double wz = 0.5 * (bodyAngular(previous).z() + bodyAngular(state).z());
    const double vy0 = bodyLinear(previous).y(), vy1 = bodyLinear(state).y();
    r.body_wz_integral += kStep * wz;
    r.yaw_change += wrap(yaw(state) - yaw(previous));
    r.body_vy_integral += kStep * 0.5 * (vy0 + vy1);
    r.absolute_body_vy_integral += kStep * 0.5 * (std::abs(vy0) + std::abs(vy1));
    r.limited_yaw_integral += kStep * state.command_angular;
    r.applied_yaw_integral += kStep * state.applied_angular_command;
    if (i >= phase.steps / 2) r.tail_wz_integral += kStep * wz;
    const double turn = state.command_angular * kStep;
    const double sinc = std::abs(turn) < 1e-12 ? 1.0 : std::sin(0.5 * turn) / (0.5 * turn);
    r.nominal_xy += state.command_linear * kStep * sinc *
                    Eigen::Vector2d(std::cos(r.nominal_yaw + 0.5 * turn),
                                    std::sin(r.nominal_yaw + 0.5 * turn));
    r.nominal_yaw += turn;
    r.applied.add(state.applied_angular_command);
    r.correction.add(state.angular_velocity_correction);
    r.limited_w.add(state.command_angular);
    if (std::abs(state.applied_angular_command) >= params.max_angular_velocity - 1e-12)
      ++r.saturated_samples;
    r.minimum_contacts = std::min(r.minimum_contacts, state.wheel_contacts);
    if (state.wheel_contacts == 4) ++r.four_contact_samples;
    r.maximum_chassis_contacts = std::max(r.maximum_chassis_contacts, state.chassis_contacts);
    if (state.chassis_contacts > 0) ++r.chassis_contact_samples;
    // Skip the request's finite deceleration ramp; once its limited yaw
    // command is exactly zero, a stop must not retain feedback actuation.
    if (phase.v == 0.0 && phase.w == 0.0 && state.command_angular == 0.0) {
      ++r.zero_limited_command_samples;
      if (state.angular_velocity_correction != 0.0 || state.applied_angular_command != 0.0)
        ++r.nonzero_correction_after_zero_command;
    }
    previous = state;
  }
  r.finish = previous;
  return r;
}

void vectorJson(std::ostream& out, const Eigen::Vector2d& v) { out << '[' << v.x() << ',' << v.y() << ']'; }
void rangeJson(std::ostream& out, const Range& r) { out << '[' << r.minimum << ',' << r.maximum << ']'; }
void poseJson(std::ostream& out, const VehicleState& s) {
  out << '[' << s.position.x() << ',' << s.position.y() << ',' << s.position.z() << ',' << yaw(s) << ']';
}
void phaseJson(std::ostream& out, const PhaseResult& r) {
  const double duration = r.phase.steps * kStep;
  const double c = std::cos(yaw(r.start)), s = std::sin(yaw(r.start));
  const Eigen::Vector2d displacement = (r.finish.position - r.start.position).head<2>();
  const Eigen::Vector2d error = r.finish.position.head<2>() - r.nominal_xy;
  const Eigen::Vector2d start_frame_displacement(c * displacement.x() + s * displacement.y(),
                                               -s * displacement.x() + c * displacement.y());
  const Eigen::Vector2d start_frame_error(c * error.x() + s * error.y(), -s * error.x() + c * error.y());
  out << "{\"name\":" << std::quoted(r.phase.name)
      << ",\"duration_s\":" << duration << ",\"samples\":" << r.phase.steps
      << ",\"requested_v_mps\":" << r.phase.v << ",\"requested_w_radps\":" << r.phase.w
      << ",\"mean_body_wz_radps\":" << r.body_wz_integral / duration
      << ",\"second_half_mean_body_wz_radps\":" << r.tail_wz_integral / (duration * 0.5)
      << ",\"requested_yaw_integral_rad\":" << r.phase.w * duration
      << ",\"limited_request_yaw_integral_rad\":" << r.limited_yaw_integral
      << ",\"applied_setpoint_yaw_integral_rad\":" << r.applied_yaw_integral
      << ",\"actual_body_wz_integral_rad\":" << r.body_wz_integral
      << ",\"actual_unwrapped_planar_yaw_change_rad\":" << r.yaw_change
      << ",\"body_lateral_velocity_integral_m\":" << r.body_vy_integral
      << ",\"absolute_body_lateral_velocity_integral_m\":" << r.absolute_body_vy_integral
      << ",\"start_pose_xyz_yaw\":"; poseJson(out, r.start);
  out << ",\"finish_pose_xyz_yaw\":"; poseJson(out, r.finish);
  out << ",\"world_xy_displacement_m\":"; vectorJson(out, displacement);
  out << ",\"start_frame_xy_displacement_m\":"; vectorJson(out, start_frame_displacement);
  out << ",\"limited_twist_nominal_endpoint_xy_m\":"; vectorJson(out, r.nominal_xy);
  out << ",\"world_xy_error_to_limited_twist_arc_m\":"; vectorJson(out, error);
  out << ",\"start_frame_xy_error_to_limited_twist_arc_m\":"; vectorJson(out, start_frame_error);
  out << ",\"limited_w_range_radps\":"; rangeJson(out, r.limited_w);
  out << ",\"applied_setpoint_range_radps\":"; rangeJson(out, r.applied);
  out << ",\"correction_range_radps\":"; rangeJson(out, r.correction);
  out << ",\"applied_limit_saturated_samples\":" << r.saturated_samples
      << ",\"minimum_wheel_contacts\":" << r.minimum_contacts
      << ",\"four_wheel_contact_samples\":" << r.four_contact_samples
      << ",\"chassis_contact_samples\":" << r.chassis_contact_samples
      << ",\"maximum_chassis_contacts\":" << r.maximum_chassis_contacts
      << ",\"zero_limited_command_samples\":" << r.zero_limited_command_samples
      << ",\"nonzero_feedback_after_zero_limited_command_samples\":" << r.nonzero_correction_after_zero_command
      << ",\"finish_feedback_correction_radps\":" << r.finish.angular_velocity_correction
      << ",\"finish_applied_setpoint_radps\":" << r.finish.applied_angular_command << '}';
}
}  // namespace

int main(int argc, char** argv) {
  try {
    std::filesystem::path output;
    if (argc == 3 && std::string(argv[1]) == "--output") output = argv[2];
    else if (argc != 1) throw std::invalid_argument("usage: smb_yaw_response [--output result.json]");
    std::ostringstream json;
    json << std::setprecision(17) << std::boolalpha
         << "{\"scope\":\"standalone Bullet core, no ROS or planner\","
            "\"physics_step_s\":" << kStep
         << ",\"settling_s\":" << kSettleSteps * kStep
         << ",\"protocol\":\"each gain has a fresh world; +yaw 8s, brake 2s, -yaw 8s, brake 2s\","
            "\"integral_method\":\"actual body velocities trapezoidal; limited/applied commands rectangular per solver step; planar yaw unwrapped from orientation\","
            "\"drift_reference\":\"world displacement includes the intended turn; endpoint error uses an ideal planar arc integrated from limited requested twist\","
            "\"runs\":[";
    bool all_passed = true, first = true;
    for (double gain : {0.0, 2.0}) {
      VehicleParams p;
      p.angular_velocity_feedback_gain = gain;
      SmbVehicle vehicle(p);
      Invariants checks;
      for (int i = 0; i < kSettleSteps; ++i) {
        vehicle.step(kStep, 0.0, 0.0);
        checks.check(vehicle.state(), p);
      }
      std::vector<PhaseResult> results;
      for (const auto& phase : kPhases) results.push_back(runPhase(vehicle, p, phase, checks));
      bool brake_reset = true;
      for (const auto& r : results)
        if (r.phase.w == 0.0)
          brake_reset = brake_reset && r.zero_limited_command_samples > 0 &&
              r.nonzero_correction_after_zero_command == 0 &&
              r.finish.angular_velocity_correction == 0.0 && r.finish.applied_angular_command == 0.0;
      all_passed = all_passed && checks.passed() && brake_reset;
      if (!first) json << ',';
      first = false;
      json << "{\"angular_velocity_feedback_gain_per_s\":" << gain
           << ",\"wheel_track_m\":" << p.wheel_track << ",\"wheel_radius_m\":" << p.wheel_radius
           << ",\"maximum_angular_setpoint_radps\":" << p.max_angular_velocity
           << ",\"max_wheel_torque_Nm\":" << p.max_wheel_torque
           << ",\"wheel_friction\":" << p.wheel_friction << ",\"ground_friction\":" << p.ground_friction
           << ",\"samples_including_settling\":" << checks.samples
           << ",\"legacy_mapping_values_checked\":" << checks.wheel_values_checked
           << ",\"legacy_mapping_exact_mismatches\":" << checks.legacy_mapping_mismatches
           << ",\"legacy_mapping_maximum_absolute_error_radps\":" << checks.maximum_legacy_mapping_error
           << ",\"legacy_setpoint_exact_mismatches\":" << checks.legacy_setpoint_mismatches
           << ",\"applied_setpoint_limit_violations\":" << checks.applied_limit_violations
           << ",\"brake_feedback_reset_passed\":" << brake_reset << ",\"phases\":[";
      for (std::size_t i = 0; i < results.size(); ++i) {
        if (i) json << ',';
        phaseJson(json, results[i]);
      }
      json << "]}";
    }
    json << "],\"invariants_passed\":" << all_passed << "}\n";
    if (!output.empty()) {
      if (!output.parent_path().empty()) std::filesystem::create_directories(output.parent_path());
      std::ofstream file(output);
      if (!file || !(file << json.str())) throw std::runtime_error("cannot write output JSON");
    }
    std::cout << json.str();
    return all_passed ? 0 : 2;
  } catch (const std::exception& e) {
    std::cerr << "smb_yaw_response: " << e.what() << '\n';
    return 1;
  }
}
