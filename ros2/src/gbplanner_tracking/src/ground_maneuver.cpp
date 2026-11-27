#include "gbplanner_tracking/ground_maneuver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gbplanner_tracking {
namespace {
constexpr double pi = 3.14159265358979323846;
double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
double sign(double value) { return std::copysign(1., value); }
double toward(double current, double target, double increment) {
  return std::abs(target-current) <= increment ? target : current+std::copysign(increment,target-current);
}
double brakingRate(double remaining, double acceleration, double control_step) {
  // Ideal one-step hold plus constant-deceleration distance:
  // rate*control_step + rate^2/(2*acceleration) <= remaining.
  // This sampled-model margin is not a bound on physical actuator latency.
  const double distance = std::max(0.,remaining);
  const double hold = acceleration*control_step;
  return distance == 0. ? 0. :
      2.*acceleration*distance/(std::sqrt(hold*hold+2.*acceleration*distance)+hold);
}
bool finite(const ManeuverObservation& o) {
  return o.pose.allFinite() && std::isfinite(o.body_vx) && std::isfinite(o.body_vy) &&
         std::isfinite(o.yaw_rate) && std::isfinite(o.stamp_s);
}
bool samePose(const Pose& a, const Pose& b, double xy, double z, double yaw) {
  return (a-b).head<2>().norm() <= xy && std::abs(a[2]-b[2]) <= z &&
         std::abs(wrap(a[3]-b[3])) <= yaw;
}
}  // namespace

const char* maneuverStatusName(ManeuverStatus status) {
  switch (status) {
    case ManeuverStatus::Idle: return "idle";
    case ManeuverStatus::Running: return "running";
    case ManeuverStatus::Settling: return "settling";
    case ManeuverStatus::Braking: return "braking";
    case ManeuverStatus::Succeeded: return "succeeded";
    case ManeuverStatus::Cancelled: return "cancelled";
    case ManeuverStatus::Failed: return "failed";
  }
  return "invalid";
}

Pose maneuverReferenceEnd(const Pose& start, ManeuverKind kind, double amount, double offset) {
  if (!start.allFinite() || !std::isfinite(amount) || !std::isfinite(offset))
    throw std::invalid_argument("nonfinite_maneuver_reference");
  Pose end = start;
  if (kind == ManeuverKind::Straight) {
    end[0] += amount*std::cos(start[3]);
    end[1] += amount*std::sin(start[3]);
  } else if (kind == ManeuverKind::Pivot) {
    end[0] += offset*(std::cos(start[3])-std::cos(start[3]+amount));
    end[1] += offset*(std::sin(start[3])-std::sin(start[3]+amount));
    end[3] = wrap(start[3]+amount);
  } else throw std::invalid_argument("unknown_maneuver_kind");
  return end;
}

GroundManeuverController::GroundManeuverController(ManeuverParameters p) : parameters_(p) {
  for (double value : {p.max_speed,p.max_yaw_rate,p.max_acceleration,p.max_yaw_acceleration,
       p.position_tolerance,p.yaw_tolerance,p.segment_position_tolerance,p.segment_yaw_tolerance,
       p.height_tolerance,p.stop_speed_tolerance,p.stop_yaw_rate_tolerance,p.reference_tolerance,
       p.straight_heading_limit,p.start_position_tolerance,p.start_yaw_tolerance,
       p.longitudinal_gain,p.heading_gain,p.pivot_correction_speed,p.odometry_timeout,
       p.segment_timeout,p.total_timeout,p.stop_timeout,p.maximum_step_dt,p.max_reference_length})
    if (!std::isfinite(value) || value <= 0.) throw std::invalid_argument("invalid_maneuver_parameters");
  if (!std::isfinite(p.velocity_damping) || p.velocity_damping < 0. ||
      !std::isfinite(p.lateral_gain) || p.lateral_gain < 0. || p.settled_observations < 1 ||
      p.max_segments < 1 || p.yaw_tolerance > pi || p.straight_heading_limit > pi ||
      p.segment_position_tolerance > p.position_tolerance || p.segment_yaw_tolerance > p.yaw_tolerance ||
      p.stop_speed_tolerance >= p.max_speed || p.stop_yaw_rate_tolerance >= p.max_yaw_rate)
    throw std::invalid_argument("inconsistent_maneuver_parameters");
}

bool GroundManeuverController::active() const {
  return output_.status == ManeuverStatus::Running || output_.status == ManeuverStatus::Settling ||
         output_.status == ManeuverStatus::Braking;
}

bool GroundManeuverController::stationary(const ManeuverObservation& o) const {
  return std::hypot(o.body_vx,o.body_vy) <= parameters_.stop_speed_tolerance &&
         std::abs(o.yaw_rate) <= parameters_.stop_yaw_rate_tolerance;
}

double GroundManeuverController::scheduledRate(const ManeuverSegment& segment,
                                               double elapsed_s, bool pivot) const {
  if (plan_.reference_times.empty() || plan_.reference_forward_velocities.empty() ||
      plan_.reference_yaw_rates.empty() || segment.reference_end_index <= segment.reference_start_index)
    return std::numeric_limits<double>::quiet_NaN();
  const auto& times = plan_.reference_times;
  const auto& values = pivot ? plan_.reference_yaw_rates : plan_.reference_forward_velocities;
  const std::size_t first = segment.reference_start_index, last = segment.reference_end_index;
  if (last >= times.size() || last >= values.size() || !std::isfinite(elapsed_s))
    return std::numeric_limits<double>::quiet_NaN();
  const double target = times[first] + std::clamp(elapsed_s, 0., times[last]-times[first]);
  auto upper = std::upper_bound(times.begin()+static_cast<std::ptrdiff_t>(first),
                                times.begin()+static_cast<std::ptrdiff_t>(last)+1, target);
  std::size_t hi = static_cast<std::size_t>(std::distance(times.begin(), upper));
  if (hi <= first) return values[first];
  if (hi > last) return values[last];
  const std::size_t lo = hi-1;
  const double span = times[hi]-times[lo];
  const double fraction = span > 0. ? (target-times[lo])/span : 0.;
  return values[lo] + fraction*(values[hi]-values[lo]);
}

void GroundManeuverController::start(const ManeuverPlan& plan, const ManeuverObservation& o, double now) {
  if (active()) throw std::logic_error("maneuver_already_active");
  if (!plan.geometry_validated || plan.segments.empty() || plan.segments.size() > parameters_.max_segments ||
      !std::isfinite(plan.control_offset)) throw std::invalid_argument("unapproved_or_invalid_maneuver_plan");
  double length = 0.;
  std::size_t pass_through_total = 0;
  for (std::size_t i=0; i<plan.segments.size(); ++i) {
    const auto& segment = plan.segments[i];
    // Match the explicit transport contract: finite and nonzero, without a
    // second minimum-size threshold. Tiny planned segments still retain their
    // geometry and observed-stop boundary; they are not erased or enlarged.
    if (!segment.stop_at_end || !std::isfinite(segment.signed_amount) || segment.signed_amount==0. ||
        !segment.expected_start.allFinite() || !segment.expected_end.allFinite() ||
        (segment.kind == ManeuverKind::Pivot && std::abs(segment.signed_amount) > 2.*pi))
      throw std::invalid_argument("invalid_maneuver_segment");
    if (segment.kind != ManeuverKind::Straight && !segment.pass_through_distances.empty())
      throw std::invalid_argument("invalid_maneuver_pass_through");
    if (segment.pass_through_distances.size() > 64-pass_through_total)
      throw std::invalid_argument("maneuver_pass_through_limit");
    double previous_distance = 0.;
    for (double distance : segment.pass_through_distances) {
      if (!std::isfinite(distance) || distance <= previous_distance ||
          distance >= std::abs(segment.signed_amount))
        throw std::invalid_argument("invalid_maneuver_pass_through");
      previous_distance = distance;
    }
    pass_through_total += segment.pass_through_distances.size();
    const Pose expected = maneuverReferenceEnd(segment.expected_start,segment.kind,
                                               segment.signed_amount,plan.control_offset);
    if (!samePose(expected,segment.expected_end,1e-6,1e-6,1e-6) ||
        (i && !samePose(plan.segments[i-1].expected_end,segment.expected_start,1e-6,1e-6,1e-6)))
      throw std::invalid_argument("inconsistent_maneuver_geometry");
    length += std::abs(segment.signed_amount)*(segment.kind == ManeuverKind::Pivot ? std::abs(plan.control_offset) : 1.);
  }
  const bool timed = !plan.reference_times.empty() || !plan.reference_forward_velocities.empty() ||
                     !plan.reference_yaw_rates.empty();
  if (timed && (plan.reference_times.size()!=plan.reference_forward_velocities.size() ||
                plan.reference_times.size()!=plan.reference_yaw_rates.size() || plan.reference_times.size()<2))
    throw std::invalid_argument("invalid_maneuver_timed_reference");
  if (timed && (plan.reference_times.front()!=0. ||
      !std::is_sorted(plan.reference_times.begin(),plan.reference_times.end())))
    throw std::invalid_argument("invalid_maneuver_timed_reference");
  for (const auto& segment : plan.segments) {
    if (timed && (segment.reference_end_index >= plan.reference_times.size() ||
                  segment.reference_start_index >= segment.reference_end_index))
      throw std::invalid_argument("invalid_maneuver_reference_indices");
  }
  if (length > parameters_.max_reference_length || !finite(o) || !std::isfinite(now) || !stationary(o) ||
      !samePose(o.pose,plan.segments.front().expected_start,parameters_.start_position_tolerance,
                parameters_.height_tolerance,parameters_.start_yaw_tolerance))
    throw std::invalid_argument("maneuver_start_not_admissible");
  plan_ = plan;
  output_ = ManeuverCommand{};
  output_.pass_through_total = pass_through_total;
  output_.status = ManeuverStatus::Running;
  output_.reason = "approved_plan_started";
  last_observation_ = o;
  pass_through_source_stamp_ = o.stamp_s;
  started_at_ = last_step_at_ = last_observed_at_ = now;
  beginSegment(o,now);
}

void GroundManeuverController::beginSegment(const ManeuverObservation& o, double now) {
  segment_started_at_ = now;
  pivot_angle_ = wrap(o.pose[3]-plan_.segments[output_.segment_index].expected_start[3]);
  output_.segment_pass_through_total = plan_.segments[output_.segment_index].pass_through_distances.size();
  output_.segment_pass_through_consumed = 0;
  // Observations delivered while the previous primitive was active cannot
  // be replayed as visits in this newly activated primitive.
  pass_through_source_stamp_ = std::max(pass_through_source_stamp_,o.stamp_s);
  output_.terminal_capture_preparing = output_.terminal_capture_active = false;
  output_.terminal_capture_progress = 0.;
  terminal_capture_attempted_ = false;
  terminal_capture_settled_ = 0;
  output_.measured_brake_active = false;
  output_.measured_brake_stop_distance = output_.measured_brake_remaining = 0.;
  measured_brake_settled_ = 0;
  settled_ = 0;
  output_.status = ManeuverStatus::Running;
  output_.reason = "tracking_planned_primitive";
}

double GroundManeuverController::pivotTerminalTarget(const ManeuverObservation& o,
    double progress, double position_tolerance, double yaw_tolerance) const {
  const auto& segment = plan_.segments[output_.segment_index];
  const double direction = sign(segment.signed_amount), amount = std::abs(segment.signed_amount);
  const double offset = plan_.control_offset, radius = std::abs(offset);
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  if (radius == 0.) return unavailable;  // A zero-offset turn cannot translate the base.

  // With zero forward command, the local model keeps q=p+d*heading fixed.
  // Intersect its base-position circle with the ORIGINAL terminal XY disk.
  // This predicts a feedback target only; measured endpoint/stop checks still
  // own acceptance, including when contact dynamics do not keep q fixed.
  const double rx = o.pose[0]+offset*std::cos(o.pose[3])-segment.expected_end[0];
  const double ry = o.pose[1]+offset*std::sin(o.pose[3])-segment.expected_end[1];
  const double distance = std::hypot(rx,ry);
  double half_width = pi;
  if (distance == 0.) {
    if (radius > position_tolerance) return unavailable;
  } else {
    const double cosine = (distance*distance+radius*radius-position_tolerance*position_tolerance)
        /(2.*distance*radius);
    if (!std::isfinite(cosine) || cosine > 1.) return unavailable;
    half_width = cosine <= -1. ? pi : std::acos(cosine);
  }
  const double center = wrap(std::atan2(ry,rx)+(offset < 0. ? pi : 0.)-o.pose[3]);

  // Work in an angle difference from the measured heading to avoid a branch
  // cut at +/-pi. Independently retain unwrapped progress, actual yaw, and
  // same-direction reachability; an entire requested revolution cannot be
  // shortened because its wrapped endpoint matches the starting heading.
  const double min_travel = std::max(0.,amount-yaw_tolerance-progress);
  const double max_travel = amount+yaw_tolerance-progress;
  if (max_travel <= min_travel) return unavailable;
  double lower = std::min(direction*min_travel,direction*max_travel);
  double upper = std::max(direction*min_travel,direction*max_travel);
  const double goal_delta = wrap(segment.expected_end[3]-o.pose[3]);
  lower = std::max(lower,goal_delta-yaw_tolerance);
  upper = std::min(upper,goal_delta+yaw_tolerance);
  if (upper <= lower) return unavailable;
  const int first = static_cast<int>(std::ceil((lower-center-half_width)/(2.*pi)));
  const int last = static_cast<int>(std::floor((upper-center+half_width)/(2.*pi)));
  double selected_travel = std::numeric_limits<double>::infinity();
  for (int branch=first; branch<=last; ++branch) {
    const double a = std::max(lower,center+branch*2.*pi-half_width);
    const double b = std::min(upper,center+branch*2.*pi+half_width);
    if (b <= a) continue;  // A tangent point has no settling margin.
    const double travel = direction*(a+.5*(b-a));
    if (travel > 0. && travel < selected_travel) selected_travel = travel;
  }
  return std::isfinite(selected_travel) ? progress+selected_travel : unavailable;
}

void GroundManeuverController::observePassThrough(const ManeuverObservation& o, bool fresh) {
  if ((output_.status!=ManeuverStatus::Running && output_.status!=ManeuverStatus::Settling) ||
      output_.pass_through_missed || !fresh || !finite(o) || o.stamp_s<=pass_through_source_stamp_)
    return;
  pass_through_source_stamp_=o.stamp_s;
  const auto& segment=plan_.segments[output_.segment_index];
  if (segment.kind!=ManeuverKind::Straight) return;
  const auto& start=segment.expected_start;
  const double direction=sign(segment.signed_amount), amount=std::abs(segment.signed_amount);
  const double progress=direction*((o.pose[0]-start[0])*std::cos(start[3])+
                                   (o.pose[1]-start[1])*std::sin(start[3]));
  const Pose reference=maneuverReferenceEnd(start,ManeuverKind::Straight,
      direction*std::clamp(progress,0.,amount),plan_.control_offset);
  const bool on_reference=(o.pose-reference).head<3>().norm()<=parameters_.reference_tolerance &&
      std::abs(o.pose[2]-reference[2])<=parameters_.height_tolerance &&
      std::abs(wrap(start[3]-o.pose[3]))<=parameters_.straight_heading_limit &&
      progress>=-parameters_.position_tolerance && progress<=amount+parameters_.position_tolerance;
  // A source sample must both cross the next distance and enter its actual
  // pose neighborhood. No interpolation, nearest-point jump or time advance
  // substitutes for that observation, even at a higher sensor rate.
  while (output_.segment_pass_through_consumed<segment.pass_through_distances.size()) {
    const double distance=segment.pass_through_distances[output_.segment_pass_through_consumed];
    if (progress>distance+parameters_.segment_position_tolerance) {
      output_.pass_through_missed=true;
      output_.pass_through_miss_stamp_s=o.stamp_s;
      return;
    }
    if (progress<distance || !on_reference) return;
    const Pose checkpoint=maneuverReferenceEnd(start,ManeuverKind::Straight,
                                               direction*distance,plan_.control_offset);
    if (!samePose(o.pose,checkpoint,parameters_.segment_position_tolerance,
                  parameters_.height_tolerance,parameters_.segment_yaw_tolerance)) return;
    ++output_.segment_pass_through_consumed;
    ++output_.pass_through_consumed;
    output_.pass_through_last_consumed_stamp_s=o.stamp_s;
  }
}

bool GroundManeuverController::endpointReached(const ManeuverObservation& o) const {
  const bool last = output_.segment_index+1 == plan_.segments.size();
  return samePose(o.pose,plan_.segments[output_.segment_index].expected_end,
                  last ? parameters_.position_tolerance : parameters_.segment_position_tolerance,
                  parameters_.height_tolerance,
                  last ? parameters_.yaw_tolerance : parameters_.segment_yaw_tolerance);
}

void GroundManeuverController::limit(double v, double w, double dt) {
  v = std::clamp(v,-parameters_.max_speed,parameters_.max_speed);
  w = std::clamp(w,-parameters_.max_yaw_rate,parameters_.max_yaw_rate);
  output_.v = toward(output_.v,v,parameters_.max_acceleration*dt);
  output_.w = toward(output_.w,w,parameters_.max_yaw_acceleration*dt);
}

void GroundManeuverController::beginStop(ManeuverStatus outcome, const std::string& reason, double now) {
  if (!active() || output_.status == ManeuverStatus::Braking) return;
  stop_outcome_ = outcome;
  stop_started_at_ = now;
  output_.status = ManeuverStatus::Braking;
  output_.reason = reason;
  output_.measured_brake_active = false;
  measured_brake_settled_ = 0;
  settled_ = 0;
}

void GroundManeuverController::cancel(double now) {
  if (!std::isfinite(now)) throw std::invalid_argument("invalid_cancel_clock");
  beginStop(ManeuverStatus::Cancelled,"cancelled",std::max(now,last_step_at_));
}

void GroundManeuverController::abort(const std::string& reason, double now) {
  if (!std::isfinite(now) || reason.empty()) throw std::invalid_argument("invalid_abort_request");
  beginStop(ManeuverStatus::Failed,reason,std::max(now,last_step_at_));
}

void GroundManeuverController::braking(const ManeuverObservation& o, bool is_new, bool fresh,
                                       double now, double dt) {
  limit(0.,0.,dt);
  const bool zero_command = output_.v == 0. && output_.w == 0.;
  if (is_new) settled_ = fresh && stationary(o) && zero_command ? settled_+1 : 0;
  if (fresh && zero_command && settled_ >= parameters_.settled_observations) {
    output_.status = stop_outcome_;
    output_.stop_confirmed = true;
  } else if (zero_command && now-stop_started_at_ > parameters_.stop_timeout) {
    output_.status = ManeuverStatus::Failed;
    output_.reason += ":stop_unconfirmed";
  }
}

void GroundManeuverController::measuredBraking(const ManeuverObservation& o, bool is_new,
                                               bool fresh, double dt) {
  // A latched bounded stop, not a new primitive or a new motion clock. Keep
  // reducing commands even between fresh source samples; only new samples
  // can establish the existing stationary observation window.
  output_.status = ManeuverStatus::Running;
  output_.reason = "braking_measured_pivot_speed";
  limit(0.,0.,dt);
  const bool stopped = fresh && stationary(o) && output_.v == 0. && output_.w == 0.;
  if (is_new) measured_brake_settled_ = stopped ? measured_brake_settled_+1 : 0;
  if (fresh && measured_brake_settled_ >= parameters_.settled_observations) {
    output_.measured_brake_active = false;
    measured_brake_settled_ = 0;
    output_.reason = "measured_pivot_stop_observed";
  }
}

ManeuverCommand GroundManeuverController::step(const ManeuverObservation& incoming, double now) {
  if (!active()) return output_;
  if (!std::isfinite(now) || now < last_step_at_) {
    beginStop(ManeuverStatus::Failed,"steady_clock_invalid",last_step_at_);
    return output_;  // No elapsed time is invented for the output limiter.
  }
  const double dt = std::min(now-last_step_at_,parameters_.maximum_step_dt);
  last_step_at_ = now;
  const bool valid = finite(incoming);
  const bool is_new = valid && incoming.stamp_s > last_observation_.stamp_s;
  if (!valid) beginStop(ManeuverStatus::Failed,"invalid_odometry",now);
  else if (incoming.stamp_s < last_observation_.stamp_s)
    beginStop(ManeuverStatus::Failed,"odometry_clock_regressed",now);
  if (is_new) {
    if (output_.status != ManeuverStatus::Braking)
      pivot_angle_ += wrap(incoming.pose[3]-last_observation_.pose[3]);
    last_observation_ = incoming;
    last_observed_at_ = now;
  }
  const bool fresh = valid && now-last_observed_at_ <= parameters_.odometry_timeout;
  if (!fresh) beginStop(ManeuverStatus::Failed,"odometry_stale",now);
  if (output_.pass_through_missed)
    beginStop(ManeuverStatus::Failed,"pass_through_checkpoint_missed",now);
  if (output_.status == ManeuverStatus::Braking) {
    braking(last_observation_,is_new,fresh,now,dt);
    return output_;
  }
  if (now-started_at_ > parameters_.total_timeout || now-segment_started_at_ > parameters_.segment_timeout) {
    beginStop(ManeuverStatus::Failed,"primitive_timeout",now);
    braking(last_observation_,is_new,fresh,now,dt);
    return output_;
  }
  // Duplicate observations never advance phase, settling count or feedback.
  // An already latched brake still reduces its output on legal control time.
  if (!is_new) {
    if (output_.measured_brake_active) measuredBraking(last_observation_,false,fresh,dt);
    return output_;
  }
  const auto& o = last_observation_;
  const auto& segment = plan_.segments[output_.segment_index];
  const auto& start = segment.expected_start;
  const double direction = sign(segment.signed_amount);
  const double amount = std::abs(segment.signed_amount);
  const double hx = std::cos(start[3]), hy = std::sin(start[3]);
  const double dx = o.pose[0]-start[0], dy = o.pose[1]-start[1];
  const double progress = segment.kind == ManeuverKind::Straight ? direction*(dx*hx+dy*hy) : direction*pivot_angle_;
  const double remaining = amount-progress;
  const double timed_rate = scheduledRate(segment,now-segment_started_at_,segment.kind==ManeuverKind::Pivot);
  Pose reference = maneuverReferenceEnd(start,segment.kind,
      direction*std::clamp(progress,0.,amount),plan_.control_offset);
  output_.reference_error = (o.pose-reference).head<3>().norm();
  const double endpoint_band = segment.kind == ManeuverKind::Straight ? parameters_.position_tolerance : parameters_.yaw_tolerance;
  if (output_.reference_error > parameters_.reference_tolerance ||
      std::abs(o.pose[2]-reference[2]) > parameters_.height_tolerance ||
      (segment.kind == ManeuverKind::Straight && std::abs(wrap(start[3]-o.pose[3])) > parameters_.straight_heading_limit) ||
      progress < -endpoint_band || remaining < -endpoint_band) {
    beginStop(ManeuverStatus::Failed,"departed_approved_reference",now);
    braking(o,is_new,fresh,now,dt);
    return output_;
  }
  // Single-rate callers retain the same checks. A sample already handled by
  // the sensor callback is a duplicate for the checkpoint ledger only, not
  // for the control/settling observation above.
  observePassThrough(o,fresh);
  if (output_.pass_through_missed) {
    beginStop(ManeuverStatus::Failed,"pass_through_checkpoint_missed",now);
    braking(o,is_new,fresh,now,dt);
    return output_;
  }
  const bool all_checkpoints_observed =
      output_.segment_pass_through_consumed == segment.pass_through_distances.size();
  const bool last_segment = output_.segment_index+1 == plan_.segments.size();
  const double progress_tolerance = segment.kind == ManeuverKind::Straight
      ? (last_segment ? parameters_.position_tolerance : parameters_.segment_position_tolerance)
      : (last_segment ? parameters_.yaw_tolerance : parameters_.segment_yaw_tolerance);
  if (segment.kind == ManeuverKind::Pivot && !output_.terminal_capture_preparing &&
      !output_.measured_brake_active) {
    const double rate = std::max(0.,direction*o.yaw_rate);
    const double target_remaining = output_.terminal_capture_active
        ? output_.terminal_capture_progress-progress : remaining;
    // Apply the same one-step stopping model to the MEASURED angular rate.
    // Command slew limits alone do not bound a changed contact response.
    // This is a model-based trigger, not a physical stopping guarantee.
    const double stop_distance = rate*parameters_.maximum_step_dt+
        rate*rate/(2.*parameters_.max_yaw_acceleration);
    if (rate > parameters_.stop_yaw_rate_tolerance && stop_distance > std::max(0.,target_remaining)) {
      output_.measured_brake_active = true;
      ++output_.measured_brake_count;
      output_.measured_brake_stop_distance = stop_distance;
      output_.measured_brake_remaining = target_remaining;
      measured_brake_settled_ = 0;
      settled_ = 0;
    }
  }
  if (output_.measured_brake_active) {
    measuredBraking(o,true,fresh,dt);
    return output_;
  }
  // Pose yaw alone is periodic: a requested full turn must not succeed at
  // its starting pose. Signed observed progress must also finish the segment.
  if (all_checkpoints_observed && endpointReached(o) && std::abs(remaining) <= progress_tolerance) {
    output_.status = ManeuverStatus::Settling;
    output_.reason = "waiting_for_observed_stop";
    limit(0.,0.,dt);
    settled_ = stationary(o) && output_.v == 0. && output_.w == 0. ? settled_+1 : 0;
    if (settled_ >= parameters_.settled_observations) {
      if (output_.segment_index+1 == plan_.segments.size()) {
        output_.status = ManeuverStatus::Succeeded;
        output_.reason = "final_pose_and_stop_observed";
        output_.stop_confirmed = true;
      } else {
        ++output_.segment_index;
        beginSegment(o,now);
      }
    }
    return output_;
  }
  output_.status = ManeuverStatus::Running;
  output_.reason = "tracking_planned_primitive";
  settled_ = 0;
  double v = 0., w = 0.;
  if (segment.kind == ManeuverKind::Straight) {
    const double request = std::max(0.,parameters_.longitudinal_gain*remaining-
                                       parameters_.velocity_damping*direction*o.body_vx);
    const double braking_speed = brakingRate(remaining,parameters_.max_acceleration,
                                             parameters_.maximum_step_dt);
    const double feedback = std::min({parameters_.max_speed,request,braking_speed});
    const double profile = std::isfinite(timed_rate) ? std::abs(timed_rate) : 0.;
    // A time-profile lead/lag cannot override braking based on measured
    // remaining progress. Both contributions share the same output envelope.
    v = direction*std::min(braking_speed,
        std::max(feedback,std::min(parameters_.max_speed,profile)));
    const double cross_error = dx*hy-dy*hx;
    w = parameters_.heading_gain*wrap(start[3]-o.pose[3])+
        parameters_.lateral_gain*direction*cross_error;
  } else {
    const double position_tolerance = last_segment ? parameters_.position_tolerance
                                                   : parameters_.segment_position_tolerance;
    if (!terminal_capture_attempted_ && !output_.terminal_capture_preparing &&
        std::abs(remaining) <= progress_tolerance &&
        (o.pose-segment.expected_end).head<2>().norm() > position_tolerance &&
        std::isfinite(pivotTerminalTarget(o,progress,position_tolerance,progress_tolerance))) {
      output_.terminal_capture_preparing = true;
      terminal_capture_settled_ = 0;
    }
    if (output_.terminal_capture_preparing) {
      // Stop forward motion before freezing q; merely setting desired v to
      // zero while it is still braking would invalidate the circle model.
      output_.reason = "preparing_pivot_terminal_capture";
      limit(0.,0.,dt);
      terminal_capture_settled_ = stationary(o) && output_.v == 0. && output_.w == 0.
          ? terminal_capture_settled_+1 : 0;
      if (terminal_capture_settled_ >= parameters_.settled_observations) {
        output_.terminal_capture_preparing = false;
        terminal_capture_attempted_ = true;
        const double target = pivotTerminalTarget(o,progress,position_tolerance,progress_tolerance);
        output_.terminal_capture_active = std::isfinite(target);
        if (output_.terminal_capture_active) output_.terminal_capture_progress = target;
        output_.reason = output_.terminal_capture_active ? "pivot_terminal_capture_latched"
                                                        : "pivot_terminal_capture_unreachable";
      }
      return output_;
    }
    const double feedback_remaining = output_.terminal_capture_active
        ? output_.terminal_capture_progress-progress : remaining;
    const double request = std::max(0.,parameters_.heading_gain*feedback_remaining-
                                       parameters_.velocity_damping*direction*o.yaw_rate);
    const double braking_speed = brakingRate(feedback_remaining,parameters_.max_yaw_acceleration,
                                             parameters_.maximum_step_dt);
    const double feedback = std::min({parameters_.max_yaw_rate,request,braking_speed});
    const double profile = !output_.terminal_capture_active && std::isfinite(timed_rate)
        ? std::abs(timed_rate) : 0.;
    w = direction*std::min(braking_speed,
        std::max(feedback,std::min(parameters_.max_yaw_rate,profile)));
    // Follow the APPROVED moving base reference during a q-point pivot.
    // Longitudinal feedback can oppose measured slip, but it is not a
    // lateral actuator and does not replace the departure/endpoint checks.
    const double longitudinal_error = (reference[0]-o.pose[0])*std::cos(o.pose[3])+
                                      (reference[1]-o.pose[1])*std::sin(o.pose[3]);
    const double correction = std::min(parameters_.pivot_correction_speed,parameters_.max_speed);
    v = output_.terminal_capture_active ? 0. :
        std::clamp(parameters_.longitudinal_gain*longitudinal_error-
                   parameters_.velocity_damping*o.body_vx,-correction,correction);
    if (output_.terminal_capture_active) output_.reason = "tracking_pivot_terminal_capture";
  }
  limit(v,w,dt);  // Feedback cannot bypass either velocity or slew limits.
  return output_;
}

}  // namespace gbplanner_tracking
