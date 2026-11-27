// Focused cold-start pairing on a recorded route and the original triangle
// mesh. No ROS, mapping, action cancellation or historical contact-state replay.
#include "gbplanner_smb/smb_vehicle.hpp"
#include "gbplanner_smb/triangle_mesh.hpp"
#include "gbplanner_tracking/pure_pursuit.hpp"

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

using gbplanner_tracking::Pose;
using gbplanner_tracking::TerminalMode;

namespace {
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
Pose pose(const gbplanner_smb::VehicleState& state) {
  const auto r = state.orientation.toRotationMatrix();
  return {state.position.x(), state.position.y(), state.position.z(), std::atan2(r(1, 0), r(0, 0))};
}

std::vector<Pose> readRoute(const char* filename) {
  std::ifstream input(filename);
  if (!input) throw std::runtime_error("cannot open recorded route");
  std::string line;
  std::getline(input, line);
  if (line != "x,y,z,yaw") throw std::runtime_error("invalid route CSV header");
  std::vector<Pose> route;
  while (std::getline(input, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    Pose point;
    if (!(row >> point.x() >> point.y() >> point.z() >> point[3]) || !point.allFinite())
      throw std::runtime_error("invalid route row");
    route.push_back(point);
  }
  if (route.size() < 2) throw std::runtime_error("recorded route requires two poses");
  return route;
}

void checkCoreContract() {
  gbplanner_tracking::Parameters p;
  p.critical_angle = .52; p.goal_reaching_threshold = .04;
  p.look_ahead_distance = .5; p.max_ang_vel = .5;
  gbplanner_tracking::PurePursuitController pp(p);
  const Pose origin = Pose::Zero();
  pp.setPath({origin, Pose(0, 1, 0, 0)});
  const auto turn = pp.step(origin);
  if (!turn.rotation_only || turn.v != 0 || turn.w != .5)
    throw std::runtime_error("critical-angle turn flag/velocity mismatch");
  pp.setPath({origin, Pose(1, 0, 0, 0)});
  if (pp.step(origin).rotation_only || pp.step(Pose(1, 0, 0, 0)).rotation_only)
    throw std::runtime_error("rotation-only flag leaked to translation/reached");
  pp.reset();
  if (pp.step(origin).rotation_only) throw std::runtime_error("rotation-only flag leaked to empty path");
  gbplanner_tracking::RotationOnlyHold hold;
  if (hold.step(origin, .02, .2) != -.01) throw std::runtime_error("hold velocity damping mismatch");
  const Pose displaced(.03, .02, 0, .4);
  const auto terminal = gbplanner_tracking::terminalAlignmentCommand(displaced, origin, .04, .2, .5);
  if (std::abs(hold.step(displaced, .04, .2) - terminal.v) > 1e-14)
    throw std::runtime_error("hold differs from existing longitudinal terminal law");
  if ((hold.anchor().value() - origin.head<2>()).norm() != 0)
    throw std::runtime_error("hold did not retain episode anchor");
  hold.reset();
  if (hold.anchor() || hold.step(displaced, 0, .2) != 0)
    throw std::runtime_error("hold episode reset mismatch");
  if (hold.step(Pose(2, 2, 0, 0), 0, .03) != -.03)
    throw std::runtime_error("hold active-speed cap mismatch");

  // Recovery adaptation: an available ray into the original XY disk does not
  // require a bearing correction. These are geometric boundaries, not a new
  // success criterion; the policy still requires its original XY and yaw.
  const auto recovery = [&](const Pose& goal, int direction, double tolerance = 0.0,
                            double critical_angle = .52) {
    return gbplanner_tracking::terminalPositionCommand(origin, goal, direction,
        .2, .5, critical_angle, tolerance);
  };
  for (const int direction : {-1, 1}) {
    const Pose goal(direction * .062, .009, 0, 0);
    const auto prior = recovery(goal, direction);
    const auto straight = recovery(goal, direction, .04);
    if (prior.w == 0.0 || straight.w != 0.0 || straight.v != prior.v)
      throw std::runtime_error("recovery disk entry changed v or retained unnecessary yaw");
  }
  for (const double lateral : {-.041, -.04, .04, .041}) {
    const Pose goal(.1, lateral, 0, 0);
    const auto prior = recovery(goal, 1);
    const auto candidate = recovery(goal, 1, .04);
    if (candidate.v != prior.v || candidate.w != prior.w)
      throw std::runtime_error("recovery disk boundary relaxed");
  }
  const Pose small_disk(.062, .009, 0, 0);
  if (recovery(small_disk, 1, .008).w == 0.0 ||
      recovery(small_disk, 1, .01).w != 0.0)
    throw std::runtime_error("recovery ignored the caller's position tolerance");
  const Pose lateral_goal(0, .03, 0, 0);
  if (recovery(lateral_goal, 1, .04, 1.5707963267948966).w == 0.0)
    throw std::runtime_error("recovery enabled a ray with no positive forward projection");
  const Pose guarded_goal(.06, .038, 0, 0);
  const auto guarded = recovery(guarded_goal, 1, .04);
  if (guarded.v != 0.0 || guarded.w == 0.0)
    throw std::runtime_error("recovery bypassed the original critical-angle guard");
  const Pose boundary_goal(.1, .04, 0, 0);
  const double boundary_alpha = std::atan2(boundary_goal.y(), boundary_goal.x());
  const auto angle_boundary = recovery(boundary_goal, 1, .05, boundary_alpha);
  if (angle_boundary.v != 0.0 || angle_boundary.w == 0.0)
    throw std::runtime_error("recovery critical-angle boundary changed");
  for (const double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity()}) {
    bool rejected = false;
    try { recovery(small_disk, 1, invalid); }
    catch (const std::invalid_argument&) { rejected = true; }
    if (!rejected) throw std::runtime_error("invalid recovery tolerance accepted");
  }
}

void checkYawGovernorContract() {
  const Pose origin = Pose::Zero();
  for (const double x : {-.08, -.04, -.02, 0., .02, .04, .08}) {
    gbplanner_tracking::RotationOnlyHold old, governed;
    old.step(origin, 0., .2);
    const auto start = governed.governedStep(origin, 0., .2, -.5, .04);
    if (start.v != 0. || start.w != -.5 || start.yaw_scale != 1.)
      throw std::runtime_error("governor changed the zero-offset turn");
    const Pose current(x, .01, 0., 0.);
    const double expected_v = old.step(current, -.03, .2);
    const auto c = governed.governedStep(current, -.03, .2, -.5, .04);
    const double expected_scale = std::max(0., 1. - std::abs(x) / .04);
    if (c.v != expected_v || c.yaw_scale != expected_scale ||
        c.w != -.5 * expected_scale || c.body_anchor_x != -x)
      throw std::runtime_error("governor changed v or failed its symmetric distance boundary");
    governed.reset();
    if (governed.anchor() || governed.governedStep(current, 0., .2, .5, .04).w != .5)
      throw std::runtime_error("governor retained an old episode anchor");
  }
  gbplanner_tracking::RotationOnlyHold rotated;
  rotated.governedStep(origin, 0., .2, .5, .04);
  const auto c = rotated.governedStep(Pose(0., .02, 0., std::acos(-1.) / 2.),
                                     0., .2, .5, .04);
  if (std::abs(c.body_anchor_x + .02) > 1.e-14 || std::abs(c.w - .25) > 1.e-14)
    throw std::runtime_error("governor offset was not measured in body X");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 5) throw std::runtime_error("usage: smb_rotation_hold_replay mesh route.csv old|hold|hold_straight|hold_corners|hold_governed|hold_governed_terminal output_directory");
    const std::string treatment = argv[3];
    if (treatment != "old" && treatment != "hold" && treatment != "hold_straight" &&
        treatment != "hold_corners" && treatment != "hold_governed" &&
        treatment != "hold_governed_terminal")
      throw std::runtime_error("invalid treatment");
    const bool govern_hold = treatment == "hold_governed" || treatment == "hold_governed_terminal";
    const bool govern_terminal = treatment == "hold_governed_terminal";
    checkCoreContract();
    if (govern_hold) checkYawGovernorContract();
    const auto route = readRoute(argv[2]);
    const auto mesh = gbplanner_smb::readTriangleMesh(argv[1]);
    gbplanner_smb::VehicleParams vp;
    vp.add_ground = false;
    vp.initial_position = {route.front().x(), route.front().y(), .5};
    vp.initial_orientation = Eigen::AngleAxisd(route.front()[3], Eigen::Vector3d::UnitZ());
    vp.angular_velocity_feedback_gain = 2.;
    gbplanner_smb::SmbVehicle vehicle(vp);
    vehicle.addTriangleMesh(mesh.vertices, mesh.triangles);
    for (int i = 0; i < 500; ++i) vehicle.step(.002, 0, 0);

    gbplanner_tracking::Parameters p;
    p.critical_angle = .52; p.dt = .05; p.desired_vel = .20000000298023224;
    p.max_lin_vel = .5; p.max_ang_vel = .5;
    p.look_ahead_distance = .5; p.look_ahead_error_margin = .2;
    p.goal_reaching_threshold = .04;
    p.preserve_corners = treatment == "hold_corners" || govern_hold;
    gbplanner_tracking::PurePursuitController pp(p);
    pp.setPath(route, p.desired_vel);
    gbplanner_tracking::TerminalPosePolicy policy(.04, .1);
    gbplanner_tracking::TerminalAlignmentController terminal;
    gbplanner_tracking::RotationOnlyHold hold;
    std::filesystem::create_directories(argv[4]);
    std::ofstream csv(std::filesystem::path(argv[4]) / "trajectory.csv");
    if (!csv) throw std::runtime_error("cannot open trajectory output");
    csv << std::setprecision(17)
        << "time_s,control_tick,x,y,z,yaw,body_vx,body_vy,body_wz,requested_v,requested_w,"
           "mode,recovery_direction,rotation_only,corner_limited,pending_corners,hold_active,anchor_x,anchor_y,"
           "pp_w,governor_yaw_scale,anchor_body_x,anchor_body_y,endpoint_body_x,endpoint_body_y,"
           "wheel_contacts,chassis_contacts,carrot_x,carrot_y,"
           "limited_v,limited_w,applied_w,yaw_rate_correction,"
           "lf_target,lf_velocity,rf_target,rf_velocity,lh_target,lh_velocity,rh_target,rh_velocity\n";
    double v = 0, w = 0, governor_scale = 1.;
    TerminalMode mode = TerminalMode::Tracking;
    gbplanner_tracking::Command tracking;
    gbplanner_smb::VehicleState state;
    int reached_step = -1;
    int index = 0;
    // Same fixed 60 s physical budget for both cold starts. Monitoring is
    // offline and does not stop this replay at a route-tube violation, so the
    // resulting terminal convergence is separately observable.
    for (; index <= 30000; ++index) {
      state = vehicle.state();
      const Pose current = pose(state);
      if (!current.allFinite() || !state.body_linear_velocity.allFinite())
        throw std::runtime_error("nonfinite vehicle state");
      const bool tick = index % 25 == 0;
      if (tick) {
        governor_scale = 1.;
        if (pp.pendingCorners() > 0) {
          policy.reset();
          mode = TerminalMode::Tracking;
        } else mode = policy.update(current, route.back());
        tracking = pp.step(current, policy.recoveryDirection());
        v = tracking.v; w = tracking.w;
        if (mode == TerminalMode::Reached) {
          v = 0; w = 0; hold.reset(); terminal.reset(); reached_step = index;
        } else if (mode == TerminalMode::Aligning) {
          hold.reset();
          const auto command = terminal.step(current, route.back(), state.body_linear_velocity.x(),
                                             p.desired_vel, p.max_ang_vel, p.dt);
          v = command.v; w = command.w;
          if (govern_terminal) {
            governor_scale = gbplanner_tracking::longitudinalYawScale(
                current, route.back().head<2>(), p.goal_reaching_threshold);
            w *= governor_scale;
          }
        } else {
          terminal.reset();
          if (const auto direction = policy.recoveryDirection()) {
            hold.reset();
            const auto command = gbplanner_tracking::terminalPositionCommand(current, route.back(),
                *direction, p.desired_vel, p.max_ang_vel, p.critical_angle,
                (treatment == "hold_straight" || treatment == "hold_corners" ||
                 govern_hold) ? p.goal_reaching_threshold : 0.0);
            v = command.v; w = command.w;
          } else if (tracking.rotation_only && treatment != "old") {
            if (govern_hold) {
              const auto command = hold.governedStep(current, state.body_linear_velocity.x(),
                  p.desired_vel, w, p.goal_reaching_threshold);
              v = command.v; w = command.w; governor_scale = command.yaw_scale;
            } else v = hold.step(current, state.body_linear_velocity.x(), p.desired_vel);
          } else hold.reset();
        }
      }
      const auto& anchor = hold.anchor();
      Eigen::Vector2d anchor_offset = Eigen::Vector2d::Zero();
      if (anchor) anchor_offset = *anchor - current.head<2>();
      const double anchor_body_x = std::cos(current[3]) * anchor_offset.x() +
                                   std::sin(current[3]) * anchor_offset.y();
      const double anchor_body_y = -std::sin(current[3]) * anchor_offset.x() +
                                    std::cos(current[3]) * anchor_offset.y();
      const Eigen::Vector2d endpoint_offset = route.back().head<2>() - current.head<2>();
      const double endpoint_body_x = std::cos(current[3]) * endpoint_offset.x() +
                                     std::sin(current[3]) * endpoint_offset.y();
      const double endpoint_body_y = -std::sin(current[3]) * endpoint_offset.x() +
                                      std::cos(current[3]) * endpoint_offset.y();
      csv << index * .002 << ',' << tick << ',' << current.x() << ',' << current.y() << ','
          << current.z() << ',' << current[3] << ',' << state.body_linear_velocity.x() << ','
          << state.body_linear_velocity.y() << ',' << state.body_angular_velocity.z() << ','
          << v << ',' << w << ',' << static_cast<int>(mode) << ',' << policy.recoveryDirection().value_or(0)
          << ',' << tracking.rotation_only << ',' << tracking.corner_limited << ','
          << tracking.pending_corners << ',' << bool(anchor) << ','
          << (anchor ? anchor->x() : current.x()) << ',' << (anchor ? anchor->y() : current.y()) << ','
          << tracking.w << ',' << governor_scale << ',' << anchor_body_x << ',' << anchor_body_y << ','
          << endpoint_body_x << ',' << endpoint_body_y << ','
          << state.wheel_contacts << ',' << state.chassis_contacts << ','
          << tracking.carrot.x() << ',' << tracking.carrot.y() << ','
          << state.command_linear << ',' << state.command_angular << ','
          << state.applied_angular_command << ',' << state.angular_velocity_correction;
      for (const auto& wheel : state.wheels)
        csv << ',' << wheel.target_angular_velocity << ',' << wheel.angular_velocity;
      csv << '\n';
      if (reached_step >= 0 || index == 30000) break;
      vehicle.step(.002, v, w);
    }
    const Pose final = pose(state);
    std::ofstream result(std::filesystem::path(argv[4]) / "summary.json");
    result << std::setprecision(17) << "{\"treatment\":\"" << treatment << "\",\"reached\":"
           << (reached_step >= 0 ? "true" : "false") << ",\"elapsed_s\":" << index * .002
           << ",\"final_position_error_m\":" << (route.back().head<2>() - final.head<2>()).norm()
           << ",\"final_yaw_error_rad\":" << std::abs(wrap(route.back()[3] - final[3]))
           << ",\"core_contract_passed\":true}\n";
    std::cout << "completed " << treatment << " cold start in " << index * .002 << " simulated seconds\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
