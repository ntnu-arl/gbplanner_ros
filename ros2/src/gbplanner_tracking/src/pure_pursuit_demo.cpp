#include "gbplanner_tracking/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {
using gbplanner_tracking::Pose;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main() {
  try {
    gbplanner_tracking::Parameters parameters;
    parameters.critical_angle = 0.52;
    parameters.max_ang_vel = 0.5;
    parameters.max_lin_vel = 0.3;
    parameters.look_ahead_distance = 0.5;
    parameters.look_ahead_error_margin = 0.2;
    parameters.goal_reaching_threshold = 0.05;
    gbplanner_tracking::PurePursuitController controller(parameters);

    // An ordinary, unscaled upstream branch has an explicit numeric oracle.
    // Requested 0.8 m/s must also honor the 0.3 m/s configured ceiling.
    controller.setPath({Pose(2.0, 0.1, 0.0, 0.0)}, 0.8);
    const auto command = controller.step(Pose::Zero());
    const double source_radius = (2.0 * 2.0 + 0.1 * 0.1) / (2.0 * (0.1 + 0.01));
    const double source_w = 0.3 / source_radius;
    require(std::abs(command.v - 0.3) < 1e-12 &&
            std::abs(command.w - source_w) < 1e-12, "Source formula comparison failed");
    require((command.carrot - Pose(2.0, 0.1, 0.0, 0.0)).norm() < 1e-12,
            "Unexpected numeric-example carrot");

    // Native Pittsburgh exposed finite left/right commands on a straight
    // route. The optional geometric steering must have a continuous zero at
    // the centreline, while retaining source speed and turn-only decisions.
    auto geometric_parameters = parameters;
    geometric_parameters.steering_curvature = gbplanner_tracking::SteeringCurvature::SignedGeometric;
    gbplanner_tracking::PurePursuitController geometric(geometric_parameters);
    for (double x : {-.2, .2}) {
      geometric.setPath({Pose(x, 0., 0., 0.)}, .3);
      const auto straight = geometric.step(Pose::Zero());
      require(straight.v * x > 0. && straight.w == 0.,
              "Straight geometric steering still requests a turn");
      for (double y : {-1e-10, 1e-10}) {
        geometric.setPath({Pose(x, y, 0., 0.)}, .3);
        const auto tiny = geometric.step(Pose::Zero());
        require(std::abs(tiny.w) < 1e-8 && tiny.w * x * y > 0.,
                "Geometric steering is discontinuous or has the wrong reverse sign");
      }
    }
    geometric.setPath({Pose(.2, .02, 0., 0.)}, .3);
    const auto left = geometric.step(Pose::Zero());
    geometric.setPath({Pose(.2, -.02, 0., 0.)}, .3);
    const auto right = geometric.step(Pose::Zero());
    geometric.setPath({Pose(-.2, .02, 0., 0.)}, .3);
    const auto reverse = geometric.step(Pose::Zero());
    require(std::abs(left.v - right.v) < 1e-12 && std::abs(left.w + right.w) < 1e-12 &&
            std::abs(left.v + reverse.v) < 1e-12 && std::abs(left.w + reverse.w) < 1e-12,
            "Geometric steering changed mirrored or reverse geometry");
    controller.setPath({Pose(.2, .02, 0., 0.)}, .3);
    require(std::abs(controller.step(Pose::Zero()).v - left.v) < 1e-12,
            "Steering choice changed the existing speed law");
    for (double y : {-1., 1.}) {
      geometric.setPath({Pose(0., y, 0., 0.)}, .3);
      controller.setPath({Pose(0., y, 0., 0.)}, .3);
      const auto updated = geometric.step(Pose::Zero()), legacy = controller.step(Pose::Zero());
      require(updated.rotation_only && updated.v == legacy.v && updated.w == legacy.w,
              "Steering choice changed the original turn-before-translation branch");
    }
    const Pose steering_origin(.6, -.4, 0., .8);
    geometric.setPath({Pose(.6 + .2*std::cos(.8) - .02*std::sin(.8),
                            -.4 + .2*std::sin(.8) + .02*std::cos(.8), 0., .8)}, .3);
    const auto transformed = geometric.step(steering_origin);
    require(std::abs(transformed.v - left.v) < 1e-12 && std::abs(transformed.w - left.w) < 1e-12,
            "Geometric steering depends on the world-frame heading");
    geometric_parameters.max_ang_vel = .001;
    gbplanner_tracking::PurePursuitController angular_limited(geometric_parameters);
    angular_limited.setPath({Pose(.2, .02, 0., 0.)}, .3);
    require(std::abs(angular_limited.step(Pose::Zero()).w - .001) < 1e-12,
            "Geometric steering exceeded the configured angular limit");

    // A short kinematic loop exercises pruning, interpolated carrots and the
    // terminal slowdown. It represents commanded SE(2) motion, not dynamics.
    controller.setPath({Pose(0.0, 0.0, 0.0, 0.0), Pose(0.5, 0.0, 0.0, 0.0),
                        Pose(1.0, 0.2, 0.0, 0.0), Pose(1.5, 0.5, 0.0, 0.0),
                        Pose(2.0, 0.5, 0.0, 0.0)}, 0.3);
    Pose state = Pose::Zero();
    double max_v = 0.0, max_w = 0.0;
    std::size_t steps = 0;
    bool reached = false;
    for (; steps < 1800; ++steps) {
      const auto next = controller.step(state);
      require(std::isfinite(next.v) && std::isfinite(next.w), "Nonfinite closed-loop command");
      max_v = std::max(max_v, std::abs(next.v));
      max_w = std::max(max_w, std::abs(next.w));
      if (next.position_reached) {
        require(next.v == 0.0 && next.w == 0.0, "Terminal command is not zero");
        reached = true;
        break;
      }
      const double midpoint_yaw = state[3] + next.w * parameters.dt * 0.5;
      state.x() += next.v * std::cos(midpoint_yaw) * parameters.dt;
      state.y() += next.v * std::sin(midpoint_yaw) * parameters.dt;
      state[3] = std::atan2(std::sin(state[3] + next.w * parameters.dt),
                            std::cos(state[3] + next.w * parameters.dt));
    }
    const double error = (state.head<2>() - Eigen::Vector2d(2.0, 0.5)).norm();
    require(reached && error <= parameters.goal_reaching_threshold, "Closed loop did not reach endpoint");
    require(max_v <= parameters.max_lin_vel + 1e-12 &&
            max_w <= parameters.max_ang_vel + 1e-12, "Velocity ceiling exceeded");
    controller.reset();
    require(controller.empty(), "Reset did not clear path");

    // Recorded run8 distances crossed 0.04 m during terminal rotation. They
    // must not alternate heading alignment with position correction anymore.
    using gbplanner_tracking::TerminalMode;
    gbplanner_tracking::TerminalPosePolicy terminal(.04, .1);
    require(terminal.update(.03994510428, -3.074367938) == TerminalMode::Aligning,
            "Terminal rotation did not enter at the original position tolerance");
    for (double jitter : {.04009676031, .03965313651, .04028706063, .03996350212, .04008863181})
      require(terminal.update(jitter, -2.69) == TerminalMode::Aligning,
              "Recorded submillimeter jitter interrupted terminal rotation");
    require(terminal.update(.04008863181, .09) == TerminalMode::Tracking,
            "Aligned yaw outside the original position tolerance did not request correction");
    require(terminal.update(.0399, .11) == TerminalMode::Aligning,
            "Original yaw tolerance was weakened");
    require(terminal.update(.060001, .11) == TerminalMode::Tracking,
            "Large position drift did not leave terminal rotation");
    require(terminal.update(.0399, .09) == TerminalMode::Reached,
            "Strict simultaneous terminal conditions were not accepted");
    terminal.reset();
    require(terminal.update(.0401, -2.69) == TerminalMode::Tracking,
            "A new goal inherited the previous alignment state");

    // run7 position recovery became stuck at a lateral goal: body_x crossed
    // zero by millimetres and switched the chosen forward/reverse turn.
    parameters.goal_reaching_threshold = .04;
    gbplanner_tracking::PurePursuitController recovery(parameters);
    const Pose lateral_goal(.001, -.05, 0., 1.);
    recovery.setPath({lateral_goal}, .3);
    require(recovery.step(Pose::Zero()).w < 0. &&
            recovery.step(Pose(.002, 0., 0., 0.)).w > 0.,
            "Fixture no longer reproduces the unlatch direction ambiguity");
    for (int direction : {1, -1}) {
      recovery.setPath({lateral_goal}, .3);
      terminal.reset();
      const Pose entry(.001, -.02, 0., 0.);
      require(terminal.update(entry, lateral_goal) == TerminalMode::Aligning,
              "Recovery fixture did not start in alignment");
      const Pose exit(direction > 0 ? 0. : .002, .02, 0., 0.);
      require(terminal.update(exit, lateral_goal) == TerminalMode::Tracking &&
              terminal.recoveryDirection() == direction,
              "Recovery direction was not selected on the exit tick");
      for (double x : {0., .002, 0., .002, 0., .002}) {
        const Pose perturbed(x, 0., 0., 0.);
        require(terminal.update(perturbed, lateral_goal) == TerminalMode::Tracking &&
                terminal.recoveryDirection() == direction,
                "Millimetre perturbation changed recovery direction");
        const auto fixed = recovery.step(perturbed, terminal.recoveryDirection());
        require(fixed.v == 0. && fixed.w * direction < 0.,
                "Latched recovery reversed the in-place turn");
      }
      const Pose turned(0., 0., 0., direction > 0 ? -1.1 : 1.1);
      terminal.update(turned, lateral_goal);
      require(recovery.step(turned, terminal.recoveryDirection()).v * direction > 0.,
              "Latched recovery never enabled translation after rotation");
      require(terminal.update(entry, lateral_goal) == TerminalMode::Aligning &&
              !terminal.recoveryDirection(), "Re-entering alignment did not release direction");
      terminal.update(exit, lateral_goal);
      terminal.reset();
      require(!terminal.recoveryDirection(), "New goal inherited a recovery direction");
    }

    // Position recovery has its own bounded ROS 2 law. These analytic checks
    // cover geometry independently of the inherited pursuit curvature law.
    using gbplanner_tracking::terminalPositionCommand;
    const Pose origin = Pose::Zero();
    const auto close_forward = terminalPositionCommand(
        origin, Pose(.05, 0., 0., 2.7), 1, .3, .5, .52);
    const auto close_reverse = terminalPositionCommand(
        origin, Pose(-.05, 0., 0., -1.4), -1, .3, .5, .52);
    require(std::abs(close_forward.v - .1) < 1e-12 && std::abs(close_forward.w) < 1e-12 &&
            std::abs(close_reverse.v + .1) < 1e-12 && std::abs(close_reverse.w) < 1e-12,
            "Five-centimetre recovery retained pursuit's near-goal speed suppression");
    const auto proportional = terminalPositionCommand(origin, Pose(.02, 0., 0., 0.), 1, .3, .5, .52);
    const auto speed_limited = terminalPositionCommand(origin, Pose(.05, 0., 0., 0.), 1, .03, .5, .52);
    require(std::abs(proportional.v - .04) < 1e-12 && std::abs(speed_limited.v - .03) < 1e-12,
            "Recovery distance feedback or requested-speed limit changed");
    const Pose positive_bearing(.05 * std::cos(.2), .05 * std::sin(.2), 0., -2.);
    const Pose negative_bearing(positive_bearing.x(), -positive_bearing.y(), 0., 1.);
    const auto positive = terminalPositionCommand(origin, positive_bearing, 1, .3, .5, .52);
    const auto negative = terminalPositionCommand(origin, negative_bearing, 1, .3, .5, .52);
    const auto backward = terminalPositionCommand(
        origin, Pose(-positive_bearing.x(), -positive_bearing.y(), 0., 0.), -1, .3, .5, .52);
    require(std::abs(positive.v - .1 * std::cos(.2)) < 1e-12 &&
            std::abs(positive.w - .4) < 1e-12 &&
            std::abs(negative.v - positive.v) < 1e-12 &&
            std::abs(negative.w + positive.w) < 1e-12 &&
            std::abs(backward.v + positive.v) < 1e-12 &&
            std::abs(backward.w - positive.w) < 1e-12,
            "Mirrored or reverse recovery changed the endpoint geometry");
    const Pose rotated_origin(.4, -.7, 0., 1.1);
    const Pose rotated_goal(rotated_origin.x() + std::cos(1.1) * positive_bearing.x() -
                             std::sin(1.1) * positive_bearing.y(),
                            rotated_origin.y() + std::sin(1.1) * positive_bearing.x() +
                             std::cos(1.1) * positive_bearing.y(), 0., 2.8);
    const auto rotated = terminalPositionCommand(rotated_origin, rotated_goal, 1, .3, .5, .52);
    require(std::abs(rotated.v - positive.v) < 1e-12 && std::abs(rotated.w - positive.w) < 1e-12,
            "Recovery is not invariant under a common planar frame transform");
    for (int direction : {1, -1}) {
      const Pose side_goal(0., -.05, 0., 2.);
      const auto turn_only = terminalPositionCommand(origin, side_goal, direction, .3, .5, .52);
      const Pose facing(0., 0., 0., direction > 0 ? -1.1 : 1.1);
      const auto advance = terminalPositionCommand(facing, side_goal, direction, .3, .5, .52);
      require(turn_only.v == 0. && std::abs(turn_only.w + direction * .5) < 1e-12 &&
              advance.v * direction > .08 && advance.w * direction < 0.,
              "Recovery did not switch from a bounded turn to directed translation");
    }
    const auto coincident = terminalPositionCommand(origin, Pose(0., 0., 0., 2.), 1, .3, .5, .52);
    require(coincident.v == 0. && coincident.w == 0.,
            "Position recovery tried to control final yaw at a coincident endpoint");

    // The run8 final observation requested only about -0.00483 m/s under
    // pursuit. Position recovery must produce the analytic bounded command;
    // this says nothing yet about the subsequent Bullet contact response.
    const Pose recovery_goal(-2.697704124579147, 1.4852882487972754,
                             .22476257255151044, 2.4256560436547985);
    const Pose recorded_final(-2.638629913330078, 1.4740843772888184,
                              .2399999350309372, -.5291941599067994);
    const auto recorded_recovery = terminalPositionCommand(recorded_final, recovery_goal, -1, .3, .5, .52);
    require(std::abs(recorded_recovery.v + .1 * std::cos(.3417627306178706)) < 1e-12 &&
            std::abs(recorded_recovery.w - .5) < 1e-12,
            "Recorded close recovery pose did not produce its analytic command");

    // Start from run8's measured alignment exit, retaining its reverse choice.
    // Exact constant-Twist SE(2) integration verifies position-only progress;
    // the loop ends at 0.04 m and makes no claim about physical arrival or yaw.
    Pose recovery_state(-2.7391819953918457, 1.4536229372024536,
                        .23999959230422974, 2.330100401314385);
    const double recovery_initial_distance = (recovery_goal - recovery_state).head<2>().norm();
    double recovery_distance = recovery_initial_distance;
    bool recovery_turned = false, recovery_translated = false;
    std::size_t recovery_steps = 0;
    for (; recovery_steps < 200 && recovery_distance > .04; ++recovery_steps) {
      const auto velocity = terminalPositionCommand(recovery_state, recovery_goal, -1, .3, .5, .52);
      recovery_turned = recovery_turned || (velocity.v == 0. && std::abs(velocity.w) > .1);
      recovery_translated = recovery_translated || velocity.v < -1e-4;
      require(std::isfinite(velocity.v) && std::isfinite(velocity.w) &&
              velocity.v <= 0. && std::abs(velocity.v) <= .1 && std::abs(velocity.w) <= .5,
              "Position-only recovery exceeded its command limits");
      const double old_yaw = recovery_state[3], next_yaw = old_yaw + velocity.w * parameters.dt;
      if (std::abs(velocity.w) > 1e-12) {
        recovery_state.x() += velocity.v / velocity.w * (std::sin(next_yaw) - std::sin(old_yaw));
        recovery_state.y() += velocity.v / velocity.w * (std::cos(old_yaw) - std::cos(next_yaw));
      } else {
        recovery_state.x() += velocity.v * std::cos(old_yaw) * parameters.dt;
        recovery_state.y() += velocity.v * std::sin(old_yaw) * parameters.dt;
      }
      recovery_state[3] = std::atan2(std::sin(next_yaw), std::cos(next_yaw));
      const double next_distance = (recovery_goal - recovery_state).head<2>().norm();
      require(next_distance <= recovery_distance + 1e-12,
              "Position-only SE(2) recovery increased endpoint distance");
      recovery_distance = next_distance;
    }
    require(recovery_turned && recovery_translated && recovery_distance <= .04 &&
            recovery_distance < recovery_initial_distance,
            "Position-only SE(2) recovery did not return inside the original position tolerance");

    // Isolate hysteresis with the pursuit controller and yaw-only command law,
    // before the adapter's longitudinal terminal feedback. Start at run8's measured first-alignment pose;
    // inject one 1 mm outward displacement while turning, then integrate only
    // commanded SE(2) velocities. The complete current controller is exercised
    // with real contact dynamics by tools/smb_terminal_replay.cpp.
    parameters.goal_reaching_threshold = .04;
    gbplanner_tracking::PurePursuitController terminal_controller(parameters);
    const Pose endpoint(-1.0097428275770157, .1046982295880039, .18682535042036869, 2.5988519635603526);
    Pose terminal_state(-.9773166179656982, .08226337283849716, .23999963700771332, -.6077496891189952);
    terminal_controller.setPath({endpoint}, .3);
    terminal.reset();
    bool terminal_reached = false, aligned_outside = false, corrected_after_yaw = false;
    std::size_t terminal_steps = 0;
    double terminal_distance = 0., terminal_yaw_error = 0.;
    for (; terminal_steps < 600; ++terminal_steps) {
      if (terminal_steps == 3) {
        const Eigen::Vector2d outward = (terminal_state - endpoint).head<2>().normalized();
        terminal_state.head<2>() += .001 * outward;
      }
      const auto next = terminal_controller.step(terminal_state);
      terminal_distance = (endpoint - terminal_state).head<2>().norm();
      terminal_yaw_error = std::atan2(std::sin(endpoint[3] - terminal_state[3]),
                                     std::cos(endpoint[3] - terminal_state[3]));
      const auto mode = terminal.update(terminal_distance, terminal_yaw_error);
      if (mode == TerminalMode::Reached) { terminal_reached = true; break; }
      double v = next.v, w = next.w;
      if (mode == TerminalMode::Aligning) {
        aligned_outside = aligned_outside || terminal_distance > .04;
        v = 0.;w = std::clamp(2. * terminal_yaw_error, -parameters.max_ang_vel, parameters.max_ang_vel);
      } else if (terminal_distance > .04 && std::abs(terminal_yaw_error) <= .1) {
        corrected_after_yaw = true;
      }
      require(std::isfinite(v) && std::isfinite(w) && std::abs(v) <= parameters.max_lin_vel &&
              std::abs(w) <= parameters.max_ang_vel, "Invalid terminal-loop velocity");
      const double midpoint = terminal_state[3] + w * parameters.dt * .5;
      terminal_state.x() += v * std::cos(midpoint) * parameters.dt;
      terminal_state.y() += v * std::sin(midpoint) * parameters.dt;
      terminal_state[3] = std::atan2(std::sin(terminal_state[3] + w * parameters.dt),
                                   std::cos(terminal_state[3] + w * parameters.dt));
    }
    require(terminal_reached && terminal_distance <= .04 && std::abs(terminal_yaw_error) <= .1,
            "Terminal controller did not converge within both original tolerances");
    require(aligned_outside && corrected_after_yaw,
            "Terminal loop did not exercise drift, retained rotation and final position correction");
    std::cout << std::setprecision(12)
              << "{\"numeric_v\":" << command.v << ",\"numeric_w\":" << command.w
              << ",\"source_w\":" << source_w << ",\"closed_loop_steps\":" << steps
              << ",\"endpoint_error\":" << error << ",\"max_v\":" << max_v
              << ",\"max_w\":" << max_w << ",\"reached\":true"
              << ",\"terminal\":{\"model\":\"commanded_SE2_not_Bullet\",\"recorded_jitter_retained_alignment\":true"
              << ",\"steps\":" << terminal_steps << ",\"position_error\":" << terminal_distance
              << ",\"yaw_error\":" << terminal_yaw_error << ",\"alignment_exit_distance\":" << terminal.alignmentExitDistance()
              << ",\"strict_position_tolerance\":0.04,\"strict_yaw_tolerance\":0.1,\"corrected_after_yaw\":true}"
              << ",\"position_recovery\":{\"both_directions_latched\":true,\"millimetre_jitter_stable\":true,"
                 "\"translation_after_turn\":true,\"mode_and_goal_reset\":true}"
              << ",\"position_recovery_law\":{\"origin\":\"ROS2_terminal_adaptation\","
                 "\"model\":\"commanded_SE2_not_Bullet\",\"geometry_and_mirrors\":true,"
                 "\"frame_invariant\":true,\"near_goal_speed_analytic\":true,\"turn_then_translate\":true"
              << ",\"recorded_pose_requested_v_mps\":" << recorded_recovery.v
              << ",\"recorded_pose_requested_w_radps\":" << recorded_recovery.w
              << ",\"initial_distance_m\":" << recovery_initial_distance
              << ",\"final_distance_m\":" << recovery_distance
              << ",\"steps\":" << recovery_steps << ",\"position_only_recovered\":true}}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Pure-pursuit demo failed: " << error.what() << '\n';
    return 1;
  }
}
