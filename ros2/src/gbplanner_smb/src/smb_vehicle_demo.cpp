#include "gbplanner_smb/smb_vehicle.hpp"

#include <algorithm>
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
double yaw(const VehicleState& state) {
  const auto r = state.orientation.toRotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}
double angleDifference(double to, double from) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}
void settle(SmbVehicle& vehicle) {
  for (int i = 0; i < 500; ++i) vehicle.step(kStep, 0.0, 0.0);
}
struct Trial {
  std::string name;
  VehicleState start, finish;
  double distance{0.0}, maximum_tilt{0.0}, mean_contacts{0.0}, contact_ratio{0.0};
  int steps{0}, chassis_contact_steps{0};
  bool passed{false};
};
Trial run(SmbVehicle& vehicle, const std::string& name, double duration, double v,
          double w, std::ostream* csv) {
  Trial trial;
  trial.name = name;
  trial.start = vehicle.state();
  auto previous = trial.start.position;
  trial.steps = static_cast<int>(std::lround(duration / kStep));
  for (int i = 0; i < trial.steps; ++i) {
    vehicle.step(kStep, v, w);
    const auto state = vehicle.state();
    if (!state.position.allFinite() || !state.orientation.coeffs().allFinite() ||
        !state.linear_velocity.allFinite() || !state.angular_velocity.allFinite())
      throw std::runtime_error("nonfinite rigid-body state in " + name);
    trial.distance += (state.position - previous).norm();
    previous = state.position;
    const auto rotation = state.orientation.toRotationMatrix();
    trial.maximum_tilt = std::max(trial.maximum_tilt,
        std::acos(std::clamp(rotation(2, 2), -1.0, 1.0)));
    trial.mean_contacts += state.wheel_contacts;
    if (state.wheel_contacts >= 2) trial.contact_ratio += 1.0;
    if (state.chassis_contacts) ++trial.chassis_contact_steps;
    if (csv && (i % 10 == 0 || i + 1 == trial.steps)) {
      *csv << name << ',' << state.simulated_time << ',' << state.position.x() << ','
           << state.position.y() << ',' << state.position.z() << ',' << yaw(state) << ','
           << state.linear_velocity.x() << ',' << state.linear_velocity.y() << ','
           << state.linear_velocity.z() << ',' << state.angular_velocity.z() << ','
           << state.command_linear << ',' << state.command_angular;
      for (const auto& wheel : state.wheels)
        *csv << ',' << wheel.angular_velocity << ',' << wheel.target_angular_velocity;
      *csv << ',' << state.wheel_contacts << ',' << state.chassis_contacts << '\n';
    }
  }
  trial.mean_contacts /= trial.steps;
  trial.contact_ratio /= trial.steps;
  trial.finish = vehicle.state();
  return trial;
}
std::string json(const Trial& t) {
  std::ostringstream out;
  const auto change = t.finish.position - t.start.position;
  out << std::setprecision(10) << "{\"name\":\"" << t.name << "\",\"steps\":" << t.steps
      << ",\"displacement\":[" << change.x() << ',' << change.y() << ',' << change.z()
      << "],\"distance_m\":" << t.distance << ",\"yaw_change_rad\":"
      << angleDifference(yaw(t.finish), yaw(t.start)) << ",\"final_speed_mps\":"
      << t.finish.linear_velocity.norm() << ",\"final_yaw_rate_radps\":"
      << t.finish.angular_velocity.z() << ",\"mean_wheel_contacts\":" << t.mean_contacts
      << ",\"at_least_two_wheel_contact_ratio\":" << t.contact_ratio
      << ",\"chassis_contact_steps\":" << t.chassis_contact_steps
      << ",\"maximum_tilt_rad\":" << t.maximum_tilt << ",\"passed\":"
      << (t.passed ? "true" : "false") << '}';
  return out.str();
}
}  // namespace

int main(int argc, char** argv) {
  try {
    std::ofstream csv_file;
    std::filesystem::path output;
    if (argc > 1) {
      output = argv[1];
      std::filesystem::create_directories(output);
      csv_file.open(output / "trajectory.csv");
      if (!csv_file) throw std::runtime_error("cannot open trajectory.csv");
      csv_file << std::setprecision(10)
          << "trial,time,x,y,z,yaw,vx,vy,vz,wz,command_linear,command_angular,"
             "lf_velocity,lf_target,rf_velocity,rf_target,lh_velocity,lh_target,"
             "rh_velocity,rh_target,wheel_contacts,chassis_contacts\n";
    }
    std::ostream* csv = csv_file.is_open() ? &csv_file : nullptr;
    VehicleParams flat_params;
    flat_params.initial_position = {0.0, 0.0, 0.26};
    SmbVehicle forward(flat_params);
    settle(forward);
    std::vector<Trial> trials;
    auto straight = run(forward, "forward", 3.0, 0.5, 0.0, csv);
    straight.passed = straight.finish.position.x() - straight.start.position.x() > 0.8 &&
        straight.contact_ratio > 0.9 && straight.chassis_contact_steps == 0;
    trials.push_back(straight);
    auto brake = run(forward, "brake", 1.2, 0.0, 0.0, csv);
    brake.passed = brake.finish.linear_velocity.norm() < 0.08 && brake.distance < 0.25 &&
        brake.chassis_contact_steps == 0;
    trials.push_back(brake);
    for (const int sign : {1, -1}) {
      SmbVehicle turning(flat_params);
      settle(turning);
      auto turn = run(turning, sign > 0 ? "left_turn" : "right_turn", 2.5,
                       0.35, sign * 0.5, csv);
      turn.passed = sign * angleDifference(yaw(turn.finish), yaw(turn.start)) > 0.2 &&
          turn.contact_ratio > 0.9 && turn.chassis_contact_steps == 0;
      trials.push_back(turn);
    }
    SmbVehicle reversing(flat_params);
    settle(reversing);
    auto reverse = run(reversing, "reverse", 2.5, -0.4, 0.0, csv);
    reverse.passed = reverse.finish.position.x() - reverse.start.position.x() < -0.5 &&
        reverse.contact_ratio > 0.9 && reverse.chassis_contact_steps == 0;
    trials.push_back(reverse);

    constexpr double slope = 4.0 * 3.14159265358979323846 / 180.0;
    auto slope_params = flat_params;
    slope_params.ground_normal = {-std::sin(slope), 0.0, std::cos(slope)};
    slope_params.initial_orientation = Eigen::AngleAxisd(-slope, Eigen::Vector3d::UnitY());
    SmbVehicle uphill(slope_params);
    settle(uphill);
    auto climb = run(uphill, "uphill_four_degrees", 3.0, 0.4, 0.0, csv);
    const auto change = climb.finish.position - climb.start.position;
    climb.passed = change.x() > 0.65 && change.z() > 0.035 &&
        climb.contact_ratio > 0.9 && climb.chassis_contact_steps == 0;
    trials.push_back(climb);

    // Verify that the same static world supplies laser geometry. The rays
    // exclude the vehicle even when their origin is inside its chassis.
    forward.addBox({3.0, 0.0, 0.5}, {0.2, 1.0, 1.0});
    const auto wall = forward.raycast({0.0, 0.0, 0.5}, {4.0, 0.0, 0.5});
    const auto floor = forward.raycast({0.0, 0.0, 0.24}, {0.0, 0.0, -1.0});
    const auto sloped_floor = uphill.raycast({0.5, 0.0, 2.0}, {0.5, 0.0, -1.0});
    const bool rays_passed = wall.hit && std::abs(wall.range - 2.9) < 0.005 &&
        floor.hit && std::abs(floor.range - 0.24) < 0.005 && sloped_floor.hit &&
        std::abs(sloped_floor.point.z() - 0.5 * std::tan(slope)) < 0.005;
    const bool passed = rays_passed && std::all_of(trials.begin(), trials.end(),
                                                   [](const auto& t) { return t.passed; });
    std::ostringstream summary;
    summary << std::setprecision(10)
        << "{\n  \"physics\":\"Bullet rigid chassis + four cylinder wheels + revolute motors\","
           "\n  \"physics_step_s\":" << kStep
        << ",\n  \"vehicle_mass_kg\":60,\n  \"motor_torque_limit_Nm\":20,"
           "\n  \"motor_torque_is_adapter_assumption\":true,\n  \"trials\":[\n";
    for (std::size_t i = 0; i < trials.size(); ++i)
      summary << "    " << json(trials[i]) << (i + 1 < trials.size() ? ",\n" : "\n");
    summary << "  ],\n  \"static_raycast\":{\"wall_range_m\":" << wall.range
            << ",\"ground_range_m\":" << floor.range << ",\"slope_ground_z_m\":"
            << sloped_floor.point.z() << ",\"passed\":" << (rays_passed ? "true" : "false")
            << "},\n  \"passed\":" << (passed ? "true" : "false") << "\n}\n";
    std::cout << summary.str();
    if (!output.empty()) {
      std::ofstream report(output / "summary.json");
      if (!report) throw std::runtime_error("cannot open summary.json");
      report << summary.str();
    }
    return passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
