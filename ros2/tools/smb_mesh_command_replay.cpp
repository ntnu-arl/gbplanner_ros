// Replay measured command timings through only the native Bullet core.
#include "gbplanner_smb/smb_vehicle.hpp"
#include "gbplanner_smb/triangle_mesh.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

struct Command { double time, v, w; };
int main(int argc, char** argv) {
  try {
    if (argc != 5) throw std::invalid_argument("usage: smb_mesh_command_replay mesh commands.csv end_seconds output_directory");
    std::ifstream input(argv[2]);
    std::vector<Command> commands{{0, 0, 0}};
    std::string line;
    std::getline(input, line);
    while (std::getline(input, line)) {
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      Command c;
      if (!(row >> c.time >> c.v >> c.w)) throw std::runtime_error("invalid command row");
      commands.push_back(c);
    }
    const double end = std::stod(argv[3]);
    const std::filesystem::path output(argv[4]);
    std::filesystem::create_directories(output);
    const auto mesh = gbplanner_smb::readTriangleMesh(argv[1]);
    gbplanner_smb::VehicleParams params;
    params.add_ground = false;
    params.initial_position = {0, 0, .5};
    params.angular_velocity_feedback_gain = 2.;
    gbplanner_smb::SmbVehicle vehicle(params);
    vehicle.addTriangleMesh(mesh.vertices, mesh.triangles);
    std::ofstream csv(output / "states.csv");
    csv << std::setprecision(12) << "time_s,x,y,z,yaw,command_v,command_w,applied_w,body_wz";
    for (int k = 0; k < 4; ++k) csv << ",w" << k << "_normal_sum,w" << k << "_normal_z,w" << k << "_cancellation";
    csv << '\n';
    std::size_t ci = 0;
    double maximum_cancellation = 0;
    gbplanner_smb::VehicleState s;
    for (int i = 0; (i + 1) * .002 <= end + 1e-10; ++i) {
      while (ci + 1 < commands.size() && commands[ci + 1].time <= i * .002) ++ci;
      vehicle.step(.002, commands[ci].v, commands[ci].w);
      s = vehicle.state();
      const auto r = s.orientation.toRotationMatrix();
      csv << s.simulated_time << ',' << s.position.x() << ',' << s.position.y() << ',' << s.position.z()
          << ',' << std::atan2(r(1, 0), r(0, 0)) << ',' << s.command_linear << ',' << s.command_angular
          << ',' << s.applied_angular_command << ',' << s.body_angular_velocity.z();
      for (const auto& wheel : s.wheels) {
        const double cancellation = wheel.normal_impulse_sum - wheel.normal_impulse_body.norm();
        maximum_cancellation = std::max(maximum_cancellation, cancellation);
        csv << ',' << wheel.normal_impulse_sum << ',' << wheel.normal_impulse_body.z() << ',' << cancellation;
      }
      csv << '\n';
    }
    const auto r = s.orientation.toRotationMatrix();
    std::ofstream summary(output / "summary.json");
    summary << std::setprecision(12) << "{\"final_time_s\":" << s.simulated_time
            << ",\"final_xyz\":[" << s.position.x() << ',' << s.position.y() << ',' << s.position.z()
            << "],\"final_yaw\":" << std::atan2(r(1, 0), r(0, 0))
            << ",\"max_normal_impulse_cancellation_Ns\":" << maximum_cancellation << "}\n";
    summary.close();
    std::ifstream result(output / "summary.json");
    std::cout << result.rdbuf();
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
