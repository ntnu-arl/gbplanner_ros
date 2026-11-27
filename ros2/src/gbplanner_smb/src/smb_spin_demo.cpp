#include "gbplanner_smb/smb_vehicle.hpp"

#include <algorithm>
#include <array>
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

namespace {
constexpr double kStep = 0.002;
constexpr int kSettleSteps = 500;
constexpr int kSpinSteps = 1500;
constexpr int kSteadyStartStep = 500;
constexpr std::array<const char*, 4> kWheelNames{"LF", "RF", "LH", "RH"};

double yaw(const VehicleState& state) {
  const auto rotation = state.orientation.toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double angleDifference(double to, double from) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

Eigen::Vector3d bodyLinear(const VehicleState& state) {
  return state.orientation.conjugate() * state.linear_velocity;
}

Eigen::Vector3d bodyAngular(const VehicleState& state) {
  return state.orientation.conjugate() * state.angular_velocity;
}

// VehicleState.position is the chassis COM. The complete assembly COM also
// includes the four wheel masses and must not be inferred from base_link alone.
Eigen::Vector3d assemblyCom(const VehicleState& state, const VehicleParams& params) {
  Eigen::Vector3d weighted = params.chassis_mass * state.position;
  for (const auto& wheel : state.wheels) weighted += params.wheel_mass * wheel.position;
  return weighted / (params.chassis_mass + 4.0 * params.wheel_mass);
}

void vectorJson(std::ostream& out, const Eigen::Vector3d& value) {
  out << '[' << value.x() << ", " << value.y() << ", " << value.z() << ']';
}

struct Statistics {
  int samples{0}, all_wheel_contact_samples{0}, chassis_contact_samples{0};
  std::size_t min_wheel_contacts{4}, max_chassis_contacts{0};
  double contact_sum{0.0};
  Eigen::Vector3d linear_sum{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_sum{Eigen::Vector3d::Zero()};
  std::array<double, 4> wheel_sum{}, target_sum{}, wheel_contact_sum{};

  void add(const VehicleState& state) {
    ++samples;
    linear_sum += bodyLinear(state);
    angular_sum += bodyAngular(state);
    contact_sum += state.wheel_contacts;
    min_wheel_contacts = std::min(min_wheel_contacts, state.wheel_contacts);
    max_chassis_contacts = std::max(max_chassis_contacts, state.chassis_contacts);
    if (state.wheel_contacts == 4) ++all_wheel_contact_samples;
    if (state.chassis_contacts > 0) ++chassis_contact_samples;
    for (std::size_t i = 0; i < state.wheels.size(); ++i) {
      wheel_sum[i] += state.wheels[i].angular_velocity;
      target_sum[i] += state.wheels[i].target_angular_velocity;
      wheel_contact_sum[i] += state.wheels[i].contact ? 1.0 : 0.0;
    }
  }
};

struct Trial {
  std::string name;
  double requested_w{0.0};
  VehicleState start, finish;
  Eigen::Vector3d assembly_start{Eigen::Vector3d::Zero()};
  Eigen::Vector3d assembly_finish{Eigen::Vector3d::Zero()};
  double chassis_path_xy{0.0}, chassis_path_3d{0.0};
  double assembly_path_xy{0.0}, assembly_path_3d{0.0}, yaw_change{0.0};
  Statistics whole, steady;
};

void writeCsv(std::ostream& csv, const std::string& trial, const char* phase,
              double phase_time, const VehicleState& state, const VehicleParams& params) {
  const auto com = assemblyCom(state, params);
  const auto linear = bodyLinear(state);
  const auto angular = bodyAngular(state);
  csv << trial << ',' << phase << ',' << phase_time << ',' << state.simulated_time;
  for (const auto& vector : {state.position, com, linear, angular})
    csv << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
  csv << ',' << yaw(state) << ',' << state.command_linear << ',' << state.command_angular;
  for (const auto& wheel : state.wheels)
    csv << ',' << wheel.angular_velocity << ',' << wheel.target_angular_velocity
        << ',' << (wheel.contact ? 1 : 0);
  csv << ',' << state.wheel_contacts << ',' << state.chassis_contacts << '\n';
}

Trial run(const VehicleParams& params, double requested_w, std::ostream* csv) {
  // A separate world for each sign avoids carrying velocity, contact manifolds
  // or wheel angles from one turn into its mirror experiment.
  SmbVehicle vehicle(params);
  Trial trial;
  trial.name = requested_w > 0.0 ? "spin_left" : "spin_right";
  trial.requested_w = requested_w;
  for (int i = 0; i < kSettleSteps; ++i) {
    vehicle.step(kStep, 0.0, 0.0);
    if (csv) writeCsv(*csv, trial.name, "settle", (i + 1) * kStep, vehicle.state(), params);
  }
  trial.start = vehicle.state();
  trial.assembly_start = assemblyCom(trial.start, params);
  auto previous = trial.start;
  Eigen::Vector3d previous_com = trial.assembly_start;
  if (csv) writeCsv(*csv, trial.name, "spin", 0.0, trial.start, params);
  for (int i = 0; i < kSpinSteps; ++i) {
    vehicle.step(kStep, 0.0, requested_w);
    const auto state = vehicle.state();
    if (!state.position.allFinite() || !state.orientation.coeffs().allFinite() ||
        !state.linear_velocity.allFinite() || !state.angular_velocity.allFinite())
      throw std::runtime_error("nonfinite rigid-body state in " + trial.name);
    const auto com = assemblyCom(state, params);
    const Eigen::Vector3d chassis_step = state.position - previous.position;
    const Eigen::Vector3d assembly_step = com - previous_com;
    trial.chassis_path_xy += chassis_step.head<2>().norm();
    trial.chassis_path_3d += chassis_step.norm();
    trial.assembly_path_xy += assembly_step.head<2>().norm();
    trial.assembly_path_3d += assembly_step.norm();
    trial.yaw_change += angleDifference(yaw(state), yaw(previous));
    trial.whole.add(state);
    // The same fixed 1--3 s window is reported for both directions. This is
    // an observation window, not a threshold for declaring the turn successful.
    if (i >= kSteadyStartStep) trial.steady.add(state);
    if (csv) writeCsv(*csv, trial.name, "spin", (i + 1) * kStep, state, params);
    previous = state;
    previous_com = com;
  }
  trial.finish = previous;
  trial.assembly_finish = previous_com;
  return trial;
}

void snapshotJson(std::ostream& out, const VehicleState& state,
                  const Eigen::Vector3d& com) {
  out << "{\n        \"chassis_com_world_m\": ";
  vectorJson(out, state.position);
  out << ",\n        \"assembly_com_world_m\": ";
  vectorJson(out, com);
  out << ",\n        \"yaw_rad\": " << yaw(state)
      << ",\n        \"body_linear_velocity_mps\": ";
  vectorJson(out, bodyLinear(state));
  out << ",\n        \"body_angular_velocity_radps\": ";
  vectorJson(out, bodyAngular(state));
  out << ",\n        \"limited_command_linear_mps\": " << state.command_linear
      << ",\n        \"limited_command_angular_radps\": " << state.command_angular
      << ",\n        \"wheel_contacts\": " << state.wheel_contacts
      << ",\n        \"chassis_contacts\": " << state.chassis_contacts
      << ",\n        \"wheels\": [\n";
  for (std::size_t i = 0; i < state.wheels.size(); ++i) {
    const auto& wheel = state.wheels[i];
    out << "          {\"name\": \"" << kWheelNames[i]
        << "\", \"velocity_radps\": " << wheel.angular_velocity
        << ", \"target_radps\": " << wheel.target_angular_velocity
        << ", \"contact\": " << (wheel.contact ? "true" : "false") << '}'
        << (i + 1 < state.wheels.size() ? ",\n" : "\n");
  }
  out << "        ]\n      }";
}

void statisticsJson(std::ostream& out, const Statistics& stats, double start_time) {
  const double count = stats.samples;
  out << "{\n        \"window_start_s\": " << start_time
      << ",\n        \"window_end_s\": " << kSpinSteps * kStep
      << ",\n        \"samples\": " << stats.samples
      << ",\n        \"mean_body_linear_velocity_mps\": ";
  vectorJson(out, stats.linear_sum / count);
  out << ",\n        \"mean_body_angular_velocity_radps\": ";
  vectorJson(out, stats.angular_sum / count);
  out << ",\n        \"mean_wheel_contacts\": " << stats.contact_sum / count
      << ",\n        \"min_wheel_contacts\": " << stats.min_wheel_contacts
      << ",\n        \"all_four_contact_ratio\": " << stats.all_wheel_contact_samples / count
      << ",\n        \"chassis_contact_samples\": " << stats.chassis_contact_samples
      << ",\n        \"max_chassis_contacts\": " << stats.max_chassis_contacts
      << ",\n        \"wheels\": [\n";
  for (std::size_t i = 0; i < kWheelNames.size(); ++i) {
    out << "          {\"name\": \"" << kWheelNames[i]
        << "\", \"mean_velocity_radps\": " << stats.wheel_sum[i] / count
        << ", \"mean_target_radps\": " << stats.target_sum[i] / count
        << ", \"mean_velocity_error_radps\": " << (stats.wheel_sum[i] - stats.target_sum[i]) / count
        << ", \"contact_ratio\": " << stats.wheel_contact_sum[i] / count << '}'
        << (i + 1 < kWheelNames.size() ? ",\n" : "\n");
  }
  out << "        ]\n      }";
}

void trialJson(std::ostream& out, const Trial& trial) {
  const Eigen::Vector3d chassis_change = trial.finish.position - trial.start.position;
  const Eigen::Vector3d assembly_change = trial.assembly_finish - trial.assembly_start;
  out << "    {\n      \"name\": \"" << trial.name << "\","
      << "\n      \"requested_linear_mps\": 0,"
      << "\n      \"requested_angular_radps\": " << trial.requested_w
      << ",\n      \"chassis_com_displacement_world_m\": ";
  vectorJson(out, chassis_change);
  out << ",\n      \"chassis_com_displacement_start_body_m\": ";
  vectorJson(out, trial.start.orientation.conjugate() * chassis_change);
  out << ",\n      \"chassis_com_xy_displacement_m\": " << chassis_change.head<2>().norm()
      << ",\n      \"chassis_com_xy_path_length_m\": " << trial.chassis_path_xy
      << ",\n      \"chassis_com_3d_path_length_m\": " << trial.chassis_path_3d
      << ",\n      \"assembly_com_displacement_world_m\": ";
  vectorJson(out, assembly_change);
  out << ",\n      \"assembly_com_xy_displacement_m\": " << assembly_change.head<2>().norm()
      << ",\n      \"assembly_com_xy_path_length_m\": " << trial.assembly_path_xy
      << ",\n      \"assembly_com_3d_path_length_m\": " << trial.assembly_path_3d
      << ",\n      \"unwrapped_yaw_change_rad\": " << trial.yaw_change
      << ",\n      \"start\": ";
  snapshotJson(out, trial.start, trial.assembly_start);
  out << ",\n      \"finish\": ";
  snapshotJson(out, trial.finish, trial.assembly_finish);
  out << ",\n      \"whole_spin\": ";
  statisticsJson(out, trial.whole, 0.0);
  out << ",\n      \"last_two_seconds\": ";
  statisticsJson(out, trial.steady, kSteadyStartStep * kStep);
  out << "\n    }";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc > 2) throw std::invalid_argument("usage: smb_spin_demo [output_directory]");
    std::filesystem::path output;
    std::ofstream csv;
    if (argc == 2) {
      output = argv[1];
      std::filesystem::create_directories(output);
      csv.open(output / "trajectory.csv");
      if (!csv) throw std::runtime_error("cannot open trajectory.csv");
      csv << std::setprecision(12)
          << "trial,phase,phase_time_s,simulation_time_s,chassis_x_m,chassis_y_m,chassis_z_m,"
             "assembly_com_x_m,assembly_com_y_m,assembly_com_z_m,body_vx_mps,body_vy_mps,"
             "body_vz_mps,body_wx_radps,body_wy_radps,body_wz_radps,yaw_rad,"
             "command_linear_mps,command_angular_radps,lf_velocity_radps,lf_target_radps,"
             "lf_contact,rf_velocity_radps,rf_target_radps,rf_contact,lh_velocity_radps,"
             "lh_target_radps,lh_contact,rh_velocity_radps,rh_target_radps,rh_contact,"
             "wheel_contacts,chassis_contacts\n";
    }
    VehicleParams params;
    params.initial_position = {0.0, 0.0, 0.26};
    std::vector<Trial> trials;
    for (double w : {0.5, -0.5}) trials.push_back(run(params, w, csv.is_open() ? &csv : nullptr));
    std::ostringstream summary;
    summary << std::setprecision(12)
        << "{\n  \"experiment\": \"independent flat-ground zero-linear-command spins\","
           "\n  \"diagnostic_only\": true,"
           "\n  \"physics_step_s\": " << kStep
        << ",\n  \"settle_duration_s\": " << kSettleSteps * kStep
        << ",\n  \"spin_duration_s\": " << kSpinSteps * kStep
        << ",\n  \"velocity_frame\": \"instantaneous chassis body frame\","
           "\n  \"chassis_mass_kg\": " << params.chassis_mass
        << ",\n  \"wheel_mass_kg\": " << params.wheel_mass
        << ",\n  \"wheel_radius_m\": " << params.wheel_radius
        << ",\n  \"command_wheel_track_m\": " << params.wheel_track
        << ",\n  \"geometric_wheel_track_m\": " << 2.0 * params.wheel_y
        << ",\n  \"motor_torque_limit_Nm\": " << params.max_wheel_torque
        << ",\n  \"trials\": [\n";
    for (std::size_t i = 0; i < trials.size(); ++i) {
      trialJson(summary, trials[i]);
      summary << (i + 1 < trials.size() ? ",\n" : "\n");
    }
    summary << "  ]\n}\n";
    std::cout << summary.str();
    if (!output.empty()) {
      std::ofstream report(output / "summary.json");
      if (!report) throw std::runtime_error("cannot open summary.json");
      report << summary.str();
      if (!report || !csv) throw std::runtime_error("failed writing spin diagnostic output");
    }
    // Successful execution is not a physical-accuracy pass criterion.
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
