#include "gbplanner_tracking/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gbplanner_tracking {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-12;
double distance(const Pose& a, const Pose& b) {
  return (a.head<2>() - b.head<2>()).norm();
}
bool positive(double value) { return std::isfinite(value) && value > 0.0; }
bool nonnegative(double value) { return std::isfinite(value) && value >= 0.0; }
double longitudinalHold(const Pose& current, const Eigen::Vector2d& target,
                        double actual_body_vx, double active_speed) {
  const Eigen::Vector2d offset = target - current.head<2>();
  const double body_x = std::cos(current[3]) * offset.x() +
                        std::sin(current[3]) * offset.y();
  const double limit = std::min(active_speed, 0.1);
  return std::clamp(2.0 * body_x - 0.5 * actual_body_vx, -limit, limit);
}
}  // namespace

const char* steeringCurvatureName(SteeringCurvature mode) {
  switch (mode) {
    case SteeringCurvature::UpstreamRegularized: return "upstream_regularized";
    case SteeringCurvature::SignedGeometric: return "signed_geometric";
  }
  throw std::invalid_argument("Invalid steering curvature mode");
}

TerminalPosePolicy::TerminalPosePolicy(double position_tolerance, double yaw_tolerance)
    : position_tolerance_(position_tolerance), yaw_tolerance_(yaw_tolerance) {
  if (!positive(position_tolerance_) || !positive(alignmentExitDistance()) ||
      !positive(yaw_tolerance_) || yaw_tolerance_ > kPi)
    throw std::invalid_argument("Invalid terminal pose tolerances");
}

TerminalMode TerminalPosePolicy::update(double distance, double yaw_error) {
  if (!nonnegative(distance) || !std::isfinite(yaw_error))
    throw std::invalid_argument("Invalid terminal pose error");
  const bool position_reached = distance <= position_tolerance_;
  const bool yaw_reached = std::abs(yaw_error) <= yaw_tolerance_;
  if (position_reached && yaw_reached) {
    recovery_direction_.reset();
    return TerminalMode::Reached;
  }
  // Finish the rotation across small position jitter, but do not report
  // success there. Once heading is aligned, correct any remaining position
  // error; a larger displacement also returns to tracking immediately.
  if (aligning_ && (distance > alignmentExitDistance() || yaw_reached)) aligning_ = false;
  if (!aligning_ && position_reached) aligning_ = true;
  if (aligning_) recovery_direction_.reset();
  return aligning_ ? TerminalMode::Aligning : TerminalMode::Tracking;
}

TerminalMode TerminalPosePolicy::update(const Pose& current, const Pose& goal) {
  if (!current.allFinite() || !goal.allFinite())
    throw std::invalid_argument("Invalid terminal pose");
  const bool was_aligning = aligning_;
  const auto mode = update(distance(current, goal),
      std::atan2(std::sin(goal[3] - current[3]), std::cos(goal[3] - current[3])));
  if (was_aligning && mode == TerminalMode::Tracking) {
    const double body_x = std::cos(current[3]) * (goal.x() - current.x()) +
                          std::sin(current[3]) * (goal.y() - current.y());
    // Select once, then retain it while rotating toward a lateral goal. A
    // millimetre-scale change in body_x must not reverse the turn every tick.
    recovery_direction_ = body_x >= 0.0 ? 1 : -1;
  }
  return mode;
}

TerminalAlignmentCommand terminalAlignmentCommand(
    const Pose& current, const Pose& goal, double actual_body_vx,
    double active_speed, double max_ang_vel) {
  if (!current.allFinite() || !goal.allFinite() || !std::isfinite(actual_body_vx) ||
      !nonnegative(active_speed) || !positive(max_ang_vel))
    throw std::invalid_argument("Invalid terminal alignment input");
  const double yaw_error = std::atan2(std::sin(goal[3] - current[3]),
                                    std::cos(goal[3] - current[3]));
  TerminalAlignmentCommand command;
  // Longitudinal proportional feedback reduces the goal offset along the
  // currently actuated axis; measured-velocity damping opposes ongoing drift.
  // Lateral error remains subject to TerminalPosePolicy's tracking fallback.
  command.v = longitudinalHold(current, goal.head<2>(), actual_body_vx, active_speed);
  command.w = std::clamp(2.0 * yaw_error, -max_ang_vel, max_ang_vel);
  return command;
}

double RotationOnlyHold::step(const Pose& current, double actual_body_vx,
                              double active_speed) {
  if (!current.allFinite() || !std::isfinite(actual_body_vx) ||
      !nonnegative(active_speed))
    throw std::invalid_argument("Invalid rotation-only hold input");
  if (!anchor_) anchor_ = current.head<2>();
  return longitudinalHold(current, *anchor_, actual_body_vx, active_speed);
}

double longitudinalYawScale(const Pose& current, const Eigen::Vector2d& target,
                            double position_tolerance) {
  if (!current.allFinite() || !target.allFinite() || !positive(position_tolerance))
    throw std::invalid_argument("Invalid longitudinal yaw governor input");
  const Eigen::Vector2d offset = target - current.head<2>();
  const double body_x = std::cos(current[3]) * offset.x() +
                        std::sin(current[3]) * offset.y();
  return std::clamp(1.0 - std::abs(body_x) / position_tolerance, 0.0, 1.0);
}

RotationOnlyHoldCommand RotationOnlyHold::governedStep(
    const Pose& current, double actual_body_vx, double active_speed,
    double angular_velocity, double position_tolerance) {
  if (!std::isfinite(angular_velocity) || !positive(position_tolerance))
    throw std::invalid_argument("Invalid rotation-only yaw governor input");
  RotationOnlyHoldCommand command;
  command.v = step(current, actual_body_vx, active_speed);
  const Eigen::Vector2d offset = *anchor_ - current.head<2>();
  command.body_anchor_x = std::cos(current[3]) * offset.x() +
                          std::sin(current[3]) * offset.y();
  // A large differential-wheel demand can defeat the longitudinal hold in
  // skid steering. Reduce only the requested yaw, reaching zero at the caller's
  // existing tolerance; leave the original v feedback and episode anchor intact.
  command.yaw_scale = longitudinalYawScale(current, *anchor_, position_tolerance);
  command.w = angular_velocity * command.yaw_scale;
  return command;
}

TerminalAlignmentCommand terminalPositionCommand(
    const Pose& current, const Pose& goal, int direction,
    double active_speed, double max_ang_vel, double critical_angle) {
  return terminalPositionCommand(current, goal, direction,
      active_speed, max_ang_vel, critical_angle, 0.0);
}

TerminalAlignmentCommand terminalPositionCommand(
    const Pose& current, const Pose& goal, int direction,
    double active_speed, double max_ang_vel, double critical_angle,
    double position_tolerance) {
  if (!current.allFinite() || !goal.allFinite() ||
      (direction != 1 && direction != -1) || !nonnegative(active_speed) ||
      !positive(max_ang_vel) || !positive(critical_angle) || critical_angle > kPi / 2.0 ||
      !nonnegative(position_tolerance))
    throw std::invalid_argument("Invalid terminal position recovery input");
  const double dx = goal.x() - current.x(), dy = goal.y() - current.y();
  const double rho = std::hypot(dx, dy);
  TerminalAlignmentCommand command;
  // A coincident endpoint has no positional bearing. Final yaw remains the
  // caller's alignment task; this law never declares the goal reached.
  if (rho <= kEpsilon) return command;
  const double heading_error = std::atan2(dy, dx) - current[3] -
                               (direction < 0 ? kPi : 0.0);
  const double alpha = std::atan2(std::sin(heading_error), std::cos(heading_error));
  command.w = std::clamp(2.0 * alpha, -max_ang_vel, max_ang_vel);
  if (std::abs(alpha) >= critical_angle) return command;
  // Ordinary pursuit's approach*curvature product reduces close recovery to
  // millimetres per second. Use one bounded distance law for this mode only.
  command.v = direction * std::min({active_speed, 0.1, 2.0 * rho}) *
              std::max(0.0, std::cos(alpha));
  const double body_x = std::cos(current[3]) * dx + std::sin(current[3]) * dy;
  const double body_y = -std::sin(current[3]) * dx + std::cos(current[3]) * dy;
  // Recover entry into the existing terminal XY disk, rather than needlessly
  // rotating toward its centre. Along this travel ray the squared distance is
  // (body_x - direction*s)^2 + body_y^2. The original critical-angle guard
  // above still applies; translation magnitude and success tests are unchanged.
  if (direction * body_x > 0.0 && std::abs(body_y) < position_tolerance)
    command.w = 0.0;
  return command;
}

TerminalAlignmentController::TerminalAlignmentController(double integral_gain)
    : integral_gain_(integral_gain) {
  if (!nonnegative(integral_gain_))
    throw std::invalid_argument("Invalid terminal velocity integral gain");
}

TerminalAlignmentCommand TerminalAlignmentController::step(
    const Pose& current, const Pose& goal, double actual_body_vx,
    double active_speed, double max_ang_vel, double dt) {
  if (!positive(dt)) throw std::invalid_argument("Invalid terminal control step");
  auto command = terminalAlignmentCommand(current, goal, actual_body_vx,
                                           active_speed, max_ang_vel);
  if (integral_gain_ == 0.0) { reset(); return command; }
  const double dx = goal.x() - current.x(), dy = goal.y() - current.y();
  const double body_goal_x = std::cos(current[3]) * dx + std::sin(current[3]) * dy;
  const double linear_limit = std::min(active_speed, 0.1);
  integral_velocity_ = std::clamp(integral_velocity_, -linear_limit, linear_limit);
  // 2*ex - .5*vx = v_ref + .5*(v_ref-vx), with v_ref=(4/3)*ex.
  // Keeping this same proportional part makes I=0 the existing controller.
  const double proportional = 2.0 * body_goal_x - 0.5 * actual_body_vx;
  const double velocity_error = (2.0 / 1.5) * body_goal_x - actual_body_vx;
  const double unsaturated = proportional + integral_velocity_;
  // Freeze integration only when it would drive an already saturated output
  // further into saturation. An opposite error can always unwind the state.
  if (!((unsaturated >= linear_limit && velocity_error > 0.0) ||
        (unsaturated <= -linear_limit && velocity_error < 0.0))) {
    integral_velocity_ = std::clamp(integral_velocity_ + integral_gain_ * dt * velocity_error,
                                    -linear_limit, linear_limit);
  }
  command.v = std::clamp(proportional + integral_velocity_, -linear_limit, linear_limit);
  return command;  // The existing bounded yaw law is unchanged.
}

PurePursuitController::PurePursuitController(Parameters parameters)
    : parameters_(parameters) {
  steeringCurvatureName(parameters_.steering_curvature);
  if (!positive(parameters_.critical_angle) || parameters_.critical_angle > kPi / 2.0 ||
      !positive(parameters_.curvature_scaling_angle) ||
      parameters_.curvature_scaling_angle > kPi / 2.0 ||
      !positive(parameters_.dt) || !positive(parameters_.desired_vel) ||
      !positive(parameters_.max_lin_vel) || !positive(parameters_.max_ang_vel) ||
      !nonnegative(parameters_.min_lin_vel) || !nonnegative(parameters_.min_ang_vel) ||
      parameters_.min_lin_vel > parameters_.max_lin_vel ||
      parameters_.min_ang_vel > parameters_.max_ang_vel ||
      !positive(parameters_.max_lin_acc) || !positive(parameters_.max_lin_dec) ||
      !positive(parameters_.look_ahead_distance) ||
      !positive(parameters_.look_ahead_error_margin) ||
      !positive(parameters_.goal_reaching_threshold))
    throw std::invalid_argument("Invalid pure-pursuit parameters");
  speed_ = std::min(parameters_.desired_vel, parameters_.max_lin_vel);
}

void PurePursuitController::setPath(const std::vector<Pose>& path,
                                  std::optional<double> speed) {
  if (path.empty()) throw std::invalid_argument("Pure-pursuit path is empty");
  for (const auto& pose : path)
    if (!pose.allFinite()) throw std::invalid_argument("Nonfinite pure-pursuit pose");
  if (speed && (!std::isfinite(*speed) || *speed < 0.0))
    throw std::invalid_argument("Invalid pure-pursuit requested speed");
  path_ = path;
  corner_indices_.clear();
  next_corner_ = path_begin_index_ = 0;
  if (parameters_.preserve_corners) {
    // Interpolation repeats segment endpoints (sometimes at different yaw/Z).
    // Use one representative per consecutive XY plateau for tangent geometry,
    // retaining its original index so pruning cannot forget a locked corner.
    std::vector<std::size_t> distinct;
    distinct.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (!distinct.empty() && distance(path[i], path[distinct.back()]) <= kEpsilon)
        distinct.back() = i;
      else
        distinct.push_back(i);
    }
    for (std::size_t j = 1; j + 1 < distinct.size(); ++j) {
      const std::size_t i = distinct[j];
      const Eigen::Vector2d incoming = path[i].head<2>() - path[distinct[j - 1]].head<2>();
      const Eigen::Vector2d outgoing = path[distinct[j + 1]].head<2>() - path[i].head<2>();
      const double turn = std::abs(std::atan2(
          incoming.x() * outgoing.y() - incoming.y() * outgoing.x(), incoming.dot(outgoing)));
      if (turn >= parameters_.critical_angle) corner_indices_.push_back(i);
    }
  }
  speed_ = std::min(speed && *speed > 0.0 ? *speed : parameters_.desired_vel,
                    parameters_.max_lin_vel);
}

void PurePursuitController::reset() {
  path_.clear();
  corner_indices_.clear();
  next_corner_ = path_begin_index_ = 0;
}

Command PurePursuitController::step(const Pose& current, std::optional<int> direction) {
  if (!current.allFinite()) throw std::invalid_argument("Nonfinite current pose");
  if (direction && *direction != 1 && *direction != -1)
    throw std::invalid_argument("Travel direction must be +1 or -1");
  Command output;
  if (path_.empty()) return output;
  const double goal_distance = distance(current, path_.back());
  output.carrot = path_.back();
  output.remaining = path_.size();
  if (goal_distance <= parameters_.goal_reaching_threshold &&
      (!parameters_.preserve_corners || pendingCorners() == 0)) {
    output.position_reached = true;
    output.remaining_distance = goal_distance;
    return output;  // A terminal position always commands zero; node aligns yaw.
  }

  std::size_t selectable_end = path_.size() - 1;
  if (parameters_.preserve_corners) {
    // Only a measured entry into the original position disk releases a gate.
    // The gate queue is independent of the retained incoming samples.
    while (next_corner_ < corner_indices_.size() &&
        distance(current, path_[corner_indices_[next_corner_] - path_begin_index_]) <=
            parameters_.goal_reaching_threshold)
      ++next_corner_;
    output.pending_corners = corner_indices_.size() - next_corner_;
    if (output.pending_corners) {
      selectable_end = corner_indices_[next_corner_] - path_begin_index_;
      output.corner_limited = true;
    }
  }

  // Legacy keeps the upstream closest-waypoint and first-outside-lookahead
  // rules. The optional adaptation limits both searches to the locked corner.
  std::size_t closest = 0;
  double closest_distance = distance(current, path_.front());
  for (std::size_t i = 1; i <= selectable_end; ++i) {
    const double candidate = distance(current, path_[i]);
    if (candidate < closest_distance) { closest_distance = candidate; closest = i; }
  }
  path_.erase(path_.begin(), path_.begin() + closest);
  selectable_end -= closest;
  if (parameters_.preserve_corners) path_begin_index_ += closest;
  std::size_t carrot_index = selectable_end;
  for (std::size_t i = 0; i <= selectable_end; ++i)
    if (distance(current, path_[i]) >= parameters_.look_ahead_distance) {
      carrot_index = i;
      break;
    }
  Pose carrot = path_[carrot_index];
  if (carrot_index > 0 && distance(current, carrot) >=
      parameters_.look_ahead_distance + parameters_.look_ahead_error_margin) {
    const Eigen::Vector3d start = path_[carrot_index - 1].head<3>();
    const Eigen::Vector3d delta = carrot.head<3>() - start;
    const double length = delta.norm();
    // Upstream samples this segment at error-margin increments. Its local
    // carrot was uninitialized when no sample qualified; retain the endpoint.
    if (length > kEpsilon) {
      const Eigen::Vector3d direction = delta / length;
      for (double d = 0.0; d < length; d += parameters_.look_ahead_error_margin) {
        const Eigen::Vector3d candidate = start + direction * d;
        if ((candidate - current.head<3>()).norm() >= parameters_.look_ahead_distance) {
          carrot.head<3>() = candidate;
          break;
        }
      }
      carrot[3] = std::atan2(delta.y(), delta.x());
    }
  }

  if (!parameters_.preserve_corners) {
    // Legacy: retain the interpolated carrot and unconsumed suffix. Do not
    // repeatedly insert an identical first waypoint, growing the path forever.
    std::vector<Pose> remaining;
    remaining.reserve(path_.size() - carrot_index + 1);
    remaining.push_back(carrot);
    std::size_t suffix = carrot_index;
    if ((carrot.head<3>() - path_[carrot_index].head<3>()).squaredNorm() <= kEpsilon * kEpsilon)
      ++suffix;
    remaining.insert(remaining.end(), path_.begin() + suffix, path_.end());
    path_.swap(remaining);
  }
  // With corner preservation, selecting a carrot never consumes geometry.
  // Only the measured nearest-point pruning above advances the route prefix.
  output.carrot = carrot;
  output.remaining = path_.size();
  output.remaining_distance = distance(current, path_.front());
  for (std::size_t i = 1; i < path_.size(); ++i)
    output.remaining_distance += distance(path_[i - 1], path_[i]);

  // Equivalent to the planar fixed->robot TF used by the ROS 1 controller.
  const double dx = carrot.x() - current.x(), dy = carrot.y() - current.y();
  const double c = std::cos(current[3]), s = std::sin(current[3]);
  const double x = c * dx + s * dy, y = -s * dx + c * dy;
  const double planar_distance = std::hypot(x, y);
  if (planar_distance <= kEpsilon) return output;
  const double linear_sign = direction ? *direction : (x >= 0.0 ? 1.0 : -1.0);
  double delta_theta = std::atan2(y, x);
  if (linear_sign < 0.0) delta_theta += delta_theta > 0.0 ? -kPi : kPi;
  const double angular_sign = delta_theta >= 0.0 ? 1.0 : -1.0;
  // Apply both angle thresholds symmetrically to left and right turns.
  const double angle = std::abs(delta_theta);
  if (angle >= parameters_.critical_angle) {
    output.rotation_only = true;
    output.w = angular_sign * parameters_.max_ang_vel;
    return output;
  }

  const double radius = planar_distance * planar_distance / (2.0 * (std::abs(y) + 0.01));
  const double radius_min = planar_distance / (2.0 * std::sin(parameters_.curvature_scaling_angle));
  const double radius_max = planar_distance / (2.0 * std::sin(10.0 * kPi / 180.0));
  double scale = 1.0;
  if (angle >= parameters_.curvature_scaling_angle) {
    scale = std::clamp(radius / radius_min, 0.0, 1.0);
  } else if (radius_max > radius_min + kEpsilon) {
    const double candidate = (std::min(radius_max, radius) - radius_min) / (radius_max - radius_min);
    // The fixed +0.01 radius regularizer can make this factor negative close
    // to the goal. Never reverse direction or stall because of that artifact;
    // use the angular ratio there and apply the approach slowdown below.
    scale = candidate > 0.0 ? std::min(1.0, candidate) :
        std::clamp(radius / radius_min, 0.0, 1.0);
  } else {
    // Equal 10-degree scaling angles make the original denominator zero.
    scale = std::clamp(radius / radius_min, 0.0, 1.0);
  }
  // Bound the actual requested magnitude before curvature and approach scaling.
  // A returning route may end at the current pose while a corner is still
  // unvisited. Its endpoint distance must not stop motion toward that gate.
  // Keep the existing slowdown whenever the endpoint is farther away, rather
  // than treating the intermediate corner as a new stopping destination.
  const double approach_distance = output.corner_limited ?
      std::max(goal_distance, distance(current, path_[selectable_end])) : goal_distance;
  const double approach = std::min(1.0, approach_distance / parameters_.look_ahead_distance);
  output.v = linear_sign * speed_ * scale * approach;
  if (parameters_.steering_curvature == SteeringCurvature::SignedGeometric) {
    // Signed planar curvature is continuous through a straight carrot. The
    // upstream +0.01 radius term otherwise requests a finite turn even for
    // arbitrarily small lateral error, alternating sign as odometry jitters.
    // Signed v gives the correct yaw direction when pursuing in reverse.
    output.w = std::clamp(2.0 * output.v * y / (planar_distance * planar_distance),
                          -parameters_.max_ang_vel, parameters_.max_ang_vel);
  } else {
    // Retain the source radius/+0.01 law in the default mode, with w derived
    // from the final scaled v rather than the source's unscaled full speed.
    output.w = radius > kEpsilon ? angular_sign * std::min(parameters_.max_ang_vel,
        std::abs(output.v) / radius) : 0.0;
  }
  return output;
}

}  // namespace gbplanner_tracking
