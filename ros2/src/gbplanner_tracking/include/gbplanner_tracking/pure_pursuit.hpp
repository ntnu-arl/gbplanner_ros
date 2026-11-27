#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace gbplanner_tracking {

using Pose = Eigen::Vector4d;  // x, y, z, yaw, in the same fixed frame.

enum class SteeringCurvature { UpstreamRegularized, SignedGeometric };
const char* steeringCurvatureName(SteeringCurvature mode);

struct Parameters {
  double critical_angle = 0.7853981633974483;
  double curvature_scaling_angle = 0.7853981633974483;
  double dt = 0.05;
  double desired_vel = 0.5;
  double max_lin_vel = 0.5;
  // Upstream loads minimum velocities and acceleration limits but does not
  // apply them in computeCommandVelocity; keep that behavior in this port.
  double min_lin_vel = 0.0;
  double max_ang_vel = 0.75;
  double min_ang_vel = 0.0;
  double max_lin_acc = std::numeric_limits<double>::max();
  double max_lin_dec = std::numeric_limits<double>::max();
  double look_ahead_distance = 1.0;
  double look_ahead_error_margin = 0.25;
  double goal_reaching_threshold = 0.5;
  // Optional ROS 2 geometry adaptation. A lookahead selection must not consume
  // an unvisited sharp corner; false retains upstream path-consumption rules.
  bool preserve_corners = false;
  // ROS 2 steering adaptation: w=2*v*y/L^2 has no turn bias at y=0.
  // The upstream radius-based speed scaling and turn-before-drive branch stay
  // unchanged; this choice affects only the moving branch's angular command.
  SteeringCurvature steering_curvature = SteeringCurvature::UpstreamRegularized;
};

struct Command {
  double v = 0.0;
  double w = 0.0;
  Pose carrot = Pose::Zero();  // Carrot in the fixed frame, including segment yaw.
  std::size_t remaining = 0;
  // Planar distance from the current pose through the remaining carrot/path.
  double remaining_distance = 0.0;
  bool position_reached = false;
  // True only for the upstream critical-angle turn-before-translation branch.
  bool rotation_only = false;
  // True when the selectable route prefix ends at an unvisited corner.
  bool corner_limited = false;
  std::size_t pending_corners = 0;
};

enum class TerminalMode { Tracking, Aligning, Reached };

// Terminal rotation has a wider exit band than its entry band so a small
// wheel/contact displacement does not immediately cancel heading alignment.
// This affects mode selection only: success always requires both original
// position and yaw tolerances on the same observed pose.
class TerminalPosePolicy {
 public:
  TerminalPosePolicy(double position_tolerance, double yaw_tolerance);
  TerminalMode update(double distance, double yaw_error);
  // Pose-aware mode selection also latches forward/reverse intent for each
  // position-correction episode after heading alignment drifts out of bounds.
  TerminalMode update(const Pose& current, const Pose& goal);
  std::optional<int> recoveryDirection() const { return recovery_direction_; }
  void reset() { aligning_ = false; recovery_direction_.reset(); }
  double alignmentExitDistance() const { return 1.5 * position_tolerance_; }

 private:
  double position_tolerance_, yaw_tolerance_;
  bool aligning_ = false;
  std::optional<int> recovery_direction_;
};

struct TerminalAlignmentCommand {
  double v = 0.0;
  double w = 0.0;
};

// ROS 2 terminal-control adaptation, separate from the upstream pure-pursuit
// velocity law. Call only in Aligning mode, with measured chassis-body vx.
// Forward position feedback (2/s) and velocity damping (0.5) counter motion
// during a turn. Translation is capped at min(active_speed, 0.1 m/s); heading
// retains the existing 2/s proportional law. This does not choose the mode or
// relax either success tolerance, and does not control lateral velocity.
TerminalAlignmentCommand terminalAlignmentCommand(
    const Pose& current, const Pose& goal, double actual_body_vx,
    double active_speed, double max_ang_vel);

// ROS 2 position-only recovery adaptation, not an upstream pure-pursuit law.
// Call only while TerminalPosePolicy retains a recovery direction. The caller
// owns the original position/yaw tolerances and switches back to alignment.
// Turn toward the endpoint in the locked +/-1 travel direction before moving;
// inside critical_angle, use 2/s distance feedback capped at 0.1 m/s and the
// active speed, with cosine heading reduction. Goal yaw is not controlled here.
// Retain this overload and symbol for existing replay tools.
TerminalAlignmentCommand terminalPositionCommand(
    const Pose& current, const Pose& goal, int direction,
    double active_speed, double max_ang_vel, double critical_angle);
// With a positive position_tolerance, keep yaw fixed when the current travel
// ray already enters that terminal XY disk and the original critical-angle
// guard allows translation. This is a ROS 2 terminal recovery adaptation,
// not the upstream pursuit law. Zero retains the prior bearing command.
TerminalAlignmentCommand terminalPositionCommand(
    const Pose& current, const Pose& goal, int direction,
    double active_speed, double max_ang_vel, double critical_angle,
    double position_tolerance);

// ROS 2 tracking adaptation, separate from the upstream pursuit law. During
// one ordinary rotation_only episode, retain its initial XY and oppose only
// longitudinal slip with the same 2/s position and 0.5 velocity feedback used
// by terminal alignment. The step() caller preserves pursuit's w and clears this
// state on leaving that episode, terminal alignment/recovery, or a goal reset.
struct RotationOnlyHoldCommand {
  double v = 0.0;
  double w = 0.0;
  double yaw_scale = 1.0;
  double body_anchor_x = 0.0;
};

// Shared ordinary-turn/terminal-alignment governor. Return a factor in [0,1]
// that reaches zero when target body-X offset reaches position_tolerance.
// This changes requested yaw only; the caller retains its longitudinal law.
double longitudinalYawScale(const Pose& current, const Eigen::Vector2d& target,
                            double position_tolerance);

class RotationOnlyHold {
 public:
  double step(const Pose& current, double actual_body_vx, double active_speed);
  // Optional yaw governor: give longitudinal recovery priority as the anchor
  // offset approaches the existing position tolerance. The original step API
  // and its v are retained; this does not control lateral error or choose yaw.
  RotationOnlyHoldCommand governedStep(const Pose& current, double actual_body_vx,
      double active_speed, double angular_velocity, double position_tolerance);
  void reset() { anchor_.reset(); }
  const std::optional<Eigen::Vector2d>& anchor() const { return anchor_; }

 private:
  std::optional<Eigen::Vector2d> anchor_;
};

// Optional ROS 2 terminal velocity-integral adaptation. Gain zero returns the
// existing stateless command exactly. Call step only while Aligning, with the
// control timer's fixed dt, and reset on leaving that mode or ending a goal.
// Integration compensates longitudinal velocity error; it does not control
// lateral slip or guarantee convergence of both terminal pose errors.
class TerminalAlignmentController {
 public:
  explicit TerminalAlignmentController(double integral_gain = 0.0);
  TerminalAlignmentCommand step(const Pose& current, const Pose& goal,
      double actual_body_vx, double active_speed, double max_ang_vel, double dt);
  void reset() { integral_velocity_ = 0.0; }
  double integralGain() const { return integral_gain_; }
  double integralVelocity() const { return integral_velocity_; }

 private:
  double integral_gain_, integral_velocity_ = 0.0;
};

// Algorithm port of smb_path_tracker at 9aa33187d7375fd48dbd63e27ca5e55bb219f750,
// with the explicitly optional preserve_corners route-consumption adaptation.
// ROS, frame transforms, cancellation and terminal yaw commands belong to the node.
class PurePursuitController {
 public:
  explicit PurePursuitController(Parameters parameters = {});
  void setPath(const std::vector<Pose>& path,
               std::optional<double> speed = std::nullopt);
  // An explicit +/-1 direction is a terminal recovery adaptation. Ordinary
  // tracking keeps the upstream per-carrot forward/reverse selection.
  Command step(const Pose& current, std::optional<int> direction = std::nullopt);
  void reset();
  bool empty() const { return path_.empty(); }
  std::size_t pendingCorners() const { return corner_indices_.size() - next_corner_; }
  double speed() const { return speed_; }
  const Parameters& parameters() const { return parameters_; }

 private:
  Parameters parameters_;
  std::vector<Pose> path_;
  // Original indices survive measured-prefix pruning, including when the
  // closest sample is the corner itself and no incoming segment remains.
  std::vector<std::size_t> corner_indices_;
  std::size_t next_corner_ = 0;
  std::size_t path_begin_index_ = 0;
  double speed_ = 0.0;
};

}  // namespace gbplanner_tracking
