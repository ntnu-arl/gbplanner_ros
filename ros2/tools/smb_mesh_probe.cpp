// Standalone finite-surface check; link to smb_vehicle. No ROS node is started.
#include "gbplanner_smb/smb_vehicle.hpp"
#include "gbplanner_smb/triangle_mesh.hpp"

#include <algorithm>
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

using gbplanner_smb::RayHit;
using gbplanner_smb::SmbVehicle;
using gbplanner_smb::TriangleMesh;
using gbplanner_smb::VehicleParams;
using gbplanner_smb::VehicleState;

namespace {
constexpr double kStep = .002;
constexpr int kSettlingSteps = 500;
constexpr int kObservationSteps = 500;

void vectorJson(std::ostream& output, const Eigen::Vector3d& value) {
  output << '[' << value.x() << ',' << value.y() << ',' << value.z() << ']';
}

std::string rayJson(const RayHit& hit) {
  std::ostringstream out;
  out << std::setprecision(12) << std::boolalpha
      << "{\"hit\":" << hit.hit << ",\"range_m\":" << hit.range << ",\"point\":";
  vectorJson(out, hit.point);
  out << ",\"normal\":";
  vectorJson(out, hit.normal);
  out << '}';
  return out.str();
}

TriangleMesh syntheticMesh() {
  TriangleMesh mesh;
  // Upward floor faces cover only [-2,2] x [-2,2]. The two wall faces
  // face -X; no infinite plane is present underneath either surface.
  mesh.vertices = {{-2., -2., 0.}, {2., -2., 0.}, {2., 2., 0.}, {-2., 2., 0.},
                   {1.5, -1., 0.}, {1.5, 1., 0.}, {1.5, 1., 1.5}, {1.5, -1., 1.5}};
  mesh.triangles = {{{0, 1, 2}}, {{0, 2, 3}}, {{4, 6, 5}}, {{4, 7, 6}}};
  return mesh;
}

struct Report {
  std::string json;
  bool passed{false};
};

Report inspect(TriangleMesh mesh, const std::string& name,
               const std::filesystem::path& output, bool original_mine) {
  VehicleParams params;
  params.add_ground = false;
  params.initial_position = {0., 0., .5};
  const double mass = params.chassis_mass + 4. * params.wheel_mass;
  const std::size_t vertex_count = mesh.vertices.size(), triangle_count = mesh.triangles.size();
  SmbVehicle vehicle(params);
  vehicle.addTriangleMesh(mesh.vertices, mesh.triangles);
  // Release the caller's buffers before the first ray or physics step. The
  // collision shape must retain independent triangle storage for its lifetime.
  mesh = TriangleMesh{};

  const auto origin_floor = vehicle.raycast({0., 0., .5}, {0., 0., -1.});
  const double expected_floor_z = original_mine ? .159680 : 0.;
  const bool origin_floor_ok = origin_floor.hit &&
      std::abs(origin_floor.point.z() - expected_floor_z) < .001;
  RayHit floor, outside, wall, sensor_wall;
  bool finite_scene_ok = true;
  if (!original_mine) {
    floor = vehicle.raycast({.8, .3, 1.}, {.8, .3, -1.});
    outside = vehicle.raycast({2.5, 0., 1.}, {2.5, 0., -1.});
    wall = vehicle.raycast({.7, .4, .8}, {2., .4, .8});
    sensor_wall = vehicle.sensorRaycast({.7, .4, .8}, {2., .4, .8});
    finite_scene_ok = floor.hit && std::abs(floor.point.z()) < .001 &&
        !outside.hit && wall.hit && sensor_wall.hit &&
        std::abs(wall.point.x() - 1.5) < .001 &&
        std::abs(sensor_wall.range - wall.range) < .001;
  }

  std::ofstream csv;
  if (!output.empty()) {
    csv.open(output / (name + ".csv"));
    if (!csv) throw std::runtime_error("cannot open mesh probe trajectory");
    csv << std::setprecision(12)
        << "phase,time_s,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,wx_radps,wy_radps,wz_radps,"
           "wheel_contacts,chassis_contacts,upward_contact_impulse_Ns\n";
  }
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  double contact_sum = 0., upward_impulse = 0.;
  std::size_t min_contacts = 4, full_contact_samples = 0, chassis_contact_samples = 0;
  bool diagnostics_valid = true;
  VehicleState observed_start, finish;
  for (int i = 0; i < kSettlingSteps + kObservationSteps; ++i) {
    vehicle.step(kStep, 0., 0.);
    const auto state = vehicle.state();
    if (!state.position.allFinite() || !state.orientation.coeffs().allFinite() ||
        !state.linear_velocity.allFinite() || !state.angular_velocity.allFinite())
      throw std::runtime_error("nonfinite mesh-supported vehicle state");
    double impulse_z = 0.;
    for (const auto& wheel : state.wheels) {
      const Eigen::Vector3d impulse = state.orientation *
          (wheel.normal_impulse_body + wheel.tangent_impulse_body);
      impulse_z += impulse.z();
      if (i >= kSettlingSteps)
        diagnostics_valid = diagnostics_valid && wheel.contact_diagnostics_valid && impulse.allFinite();
    }
    if (state.chassis_contacts) ++chassis_contact_samples;
    if (i >= kSettlingSteps) {
      if (i == kSettlingSteps) observed_start = state;
      min_z = std::min(min_z, state.position.z());
      max_z = std::max(max_z, state.position.z());
      min_contacts = std::min(min_contacts, state.wheel_contacts);
      contact_sum += state.wheel_contacts;
      full_contact_samples += state.wheel_contacts == 4 ? 1 : 0;
      upward_impulse += impulse_z;
    }
    if (csv.is_open())
      csv << (i < kSettlingSteps ? "settling" : "support") << ',' << state.simulated_time
          << ',' << state.position.x() << ',' << state.position.y() << ',' << state.position.z()
          << ',' << state.linear_velocity.x() << ',' << state.linear_velocity.y() << ',' << state.linear_velocity.z()
          << ',' << state.angular_velocity.x() << ',' << state.angular_velocity.y() << ',' << state.angular_velocity.z()
          << ',' << state.wheel_contacts << ',' << state.chassis_contacts << ',' << impulse_z << '\n';
    finish = state;
  }
  const double observation_duration = kObservationSteps * kStep;
  const double mean_support_force = upward_impulse / observation_duration;
  const double weight = mass * params.gravity;
  const double force_relative_error = std::abs(mean_support_force - weight) / weight;
  // These are acceptance bounds, not assumed outputs. The report always
  // records the measured pose, contacts and force even when a bound fails.
  const bool stable_support = min_contacts >= 3 &&
      double(full_contact_samples) / kObservationSteps >= .95 &&
      chassis_contact_samples == 0 && max_z - min_z < .01 &&
      finish.linear_velocity.norm() < .03 && finish.angular_velocity.norm() < .05 &&
      diagnostics_valid && force_relative_error < .15;
  const double nominal_base_z = expected_floor_z + params.wheel_radius - params.wheel_z;
  // The synthetic floor is flat. For the original mine, the four tire
  // contacts sample different triangles; report the center-ray height rather
  // than imposing flat-plane kinematics on that terrain.
  const bool flat_height_ok = original_mine || std::abs(finish.position.z() - nominal_base_z) < .01;
  const Eigen::Vector3d beam_from = finish.position + Eigen::Vector3d(0., 0., .3);
  const Eigen::Vector3d beam_to = finish.position - Eigen::Vector3d(0., 0., 1.);
  const auto through_chassis = vehicle.raycast(beam_from, beam_to);
  const auto blocked_by_chassis = vehicle.sensorRaycast(beam_from, beam_to);
  const bool occlusion_ok = through_chassis.hit && !blocked_by_chassis.hit;
  const bool passed = origin_floor_ok && finite_scene_ok && stable_support && flat_height_ok && occlusion_ok;

  std::ostringstream report;
  report << std::setprecision(12) << std::boolalpha
      << "{\"name\":" << std::quoted(name)
      << ",\"geometry\":\"static btTriangleMesh + btBvhTriangleMeshShape\","
         "\"implicit_ground_plane\":false,\"input_buffers_released_before_use\":true"
      << ",\"vertices\":" << vertex_count << ",\"triangles\":" << triangle_count
      << ",\"vehicle_mass_kg\":" << mass << ",\"spawn_xyz\":[0,0,0.5]"
      << ",\"physics_step_s\":" << kStep << ",\"settling_duration_s\":" << kSettlingSteps * kStep
      << ",\"observation_duration_s\":" << observation_duration
      << ",\"origin_floor_ray\":" << rayJson(origin_floor)
      << ",\"expected_origin_floor_z_m\":" << expected_floor_z
      << ",\"origin_floor_passed\":" << origin_floor_ok;
  if (!original_mine)
    report << ",\"finite_surface_rays\":{\"floor\":" << rayJson(floor)
           << ",\"outside_floor\":" << rayJson(outside)
           << ",\"wall\":" << rayJson(wall) << ",\"sensor_wall\":" << rayJson(sensor_wall)
           << ",\"passed\":" << finite_scene_ok << '}';
  report << ",\"self_occlusion\":{\"scene_only_ray\":" << rayJson(through_chassis)
      << ",\"sensor_ray\":" << rayJson(blocked_by_chassis) << ",\"passed\":" << occlusion_ok << '}'
      << ",\"support\":{\"observed_start_xyz\":";
  vectorJson(report, observed_start.position);
  report << ",\"final_xyz\":";
  vectorJson(report, finish.position);
  report << ",\"final_linear_velocity\":";
  vectorJson(report, finish.linear_velocity);
  report << ",\"final_angular_velocity\":";
  vectorJson(report, finish.angular_velocity);
  report << ",\"nominal_base_z_from_center_ray_m\":" << nominal_base_z
      << ",\"min_base_z_m\":" << min_z << ",\"max_base_z_m\":" << max_z
      << ",\"min_wheel_contacts\":" << min_contacts
      << ",\"mean_wheel_contacts\":" << contact_sum / kObservationSteps
      << ",\"four_wheel_contact_ratio\":" << double(full_contact_samples) / kObservationSteps
      << ",\"chassis_contact_samples\":" << chassis_contact_samples
      << ",\"contact_diagnostics_valid\":" << diagnostics_valid
      << ",\"mean_upward_contact_force_N\":" << mean_support_force
      << ",\"vehicle_weight_N\":" << weight
      << ",\"force_relative_error\":" << force_relative_error
      << ",\"stable_support\":" << stable_support
      << ",\"flat_height_check_applies\":" << !original_mine
      << ",\"flat_height_passed\":" << flat_height_ok << '}'
      << ",\"passed\":" << passed << '}';
  if (csv.is_open()) {
    csv.flush();
    if (!csv) throw std::runtime_error("failed writing mesh probe trajectory");
  }
  return {report.str(), passed};
  // vehicle is destroyed here, before the caller reports success: its bodies
  // and BVH must release references before the owned triangle storage dies.
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc > 3)
      throw std::invalid_argument("usage: smb_mesh_probe [output_directory [edgar_collision.mesh]]");
    const std::filesystem::path output = argc >= 2 ? argv[1] : std::filesystem::path{};
    if (!output.empty()) std::filesystem::create_directories(output);
    std::vector<Report> reports;
    reports.push_back(inspect(syntheticMesh(), "finite_floor_and_wall", output, false));
    if (argc == 3)
      reports.push_back(inspect(gbplanner_smb::readTriangleMesh(argv[2]), "edgar_mine", output, true));
    const bool passed = std::all_of(reports.begin(), reports.end(), [](const auto& result) { return result.passed; });
    std::ostringstream summary;
    summary << "{\n  \"scope\":\"finite mesh ray geometry, self occlusion and real Bullet wheel support\",\n"
               "  \"scenes\":[\n";
    for (std::size_t i = 0; i < reports.size(); ++i)
      summary << "    " << reports[i].json << (i + 1 < reports.size() ? ",\n" : "\n");
    summary << "  ],\n  \"passed\":" << std::boolalpha << passed << "\n}\n";
    if (!output.empty()) {
      std::ofstream file(output / "summary.json");
      if (!file) throw std::runtime_error("cannot open mesh probe summary");
      file << summary.str();
      file.flush();
      if (!file) throw std::runtime_error("failed writing mesh probe summary");
    }
    std::cout << summary.str();
    return passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
