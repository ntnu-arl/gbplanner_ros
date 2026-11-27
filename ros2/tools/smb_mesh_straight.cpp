// Fixed-command mesh-contact reproduction. No ROS or chassis feedback input.
#include "gbplanner_smb/smb_vehicle.hpp"
#include "gbplanner_smb/triangle_mesh.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::invalid_argument("usage: smb_mesh_straight mesh output_directory");
    const auto mesh = gbplanner_smb::readTriangleMesh(argv[1]);
    const std::filesystem::path output = argv[2];
    std::filesystem::create_directories(output);
    gbplanner_smb::VehicleParams params;
    params.add_ground = false;
    params.initial_position = {0., 0., .5};
    gbplanner_smb::SmbVehicle vehicle(params);
    vehicle.addTriangleMesh(mesh.vertices, mesh.triangles);
    std::ofstream csv(output / "straight.csv");
    if (!csv) throw std::runtime_error("cannot create trajectory");
    csv << std::setprecision(12)
        << "time_s,command_v,command_w,x,y,z,yaw,wz,wheel_contacts,chassis_contacts,"
           "max_wheel_horizontal_normal_impulse,max_wheel_normal_impulse\n";
    double max_horizontal = 0., max_yaw = 0.;
    std::size_t chassis_contacts = 0;
    gbplanner_smb::VehicleState state;
    constexpr double dt = .002;
    for (int i = 0; i < 6500; ++i) {
      vehicle.step(dt, i < 500 ? 0. : .2, 0.);
      state = vehicle.state();
      const auto r = state.orientation.toRotationMatrix();
      const double yaw = std::atan2(r(1, 0), r(0, 0));
      double horizontal = 0., normal = 0.;
      for (const auto& wheel : state.wheels) {
        const Eigen::Vector3d world_impulse = state.orientation * wheel.normal_impulse_body;
        horizontal = std::max(horizontal, world_impulse.head<2>().norm());
        normal = std::max(normal, wheel.normal_impulse_sum);
      }
      if (i >= 500) {
        max_horizontal = std::max(max_horizontal, horizontal);
        max_yaw = std::max(max_yaw, std::abs(yaw));
        chassis_contacts += state.chassis_contacts;
      }
      csv << state.simulated_time << ',' << state.command_linear << ',' << state.command_angular
          << ',' << state.position.x() << ',' << state.position.y() << ',' << state.position.z()
          << ',' << yaw << ',' << state.body_angular_velocity.z() << ',' << state.wheel_contacts
          << ',' << state.chassis_contacts << ',' << horizontal << ',' << normal << '\n';
    }
    const auto r = state.orientation.toRotationMatrix();
    std::ofstream summary(output / "summary.json");
    summary << std::setprecision(12)
            << "{\"settling_s\":1,\"straight_command_s\":12,\"command_v_mps\":0.2,"
               "\"command_w_radps\":0,\"physics_step_s\":0.002,\"final_xyz\":["
            << state.position.x() << ',' << state.position.y() << ',' << state.position.z()
            << "],\"final_yaw_rad\":" << std::atan2(r(1, 0), r(0, 0))
            << ",\"max_abs_yaw_rad\":" << max_yaw
            << ",\"max_wheel_horizontal_normal_impulse_Ns\":" << max_horizontal
            << ",\"chassis_contacts\":" << chassis_contacts << "}\n";
    summary.close();
    csv.close();
    std::ifstream saved(output / "summary.json");
    std::cout << saved.rdbuf();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
