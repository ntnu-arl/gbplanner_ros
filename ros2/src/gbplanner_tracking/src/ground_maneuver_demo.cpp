#include "gbplanner_tracking/ground_maneuver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
using namespace gbplanner_tracking;
constexpr double pi = 3.14159265358979323846;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
bool active(ManeuverStatus status) {
  return status == ManeuverStatus::Running || status == ManeuverStatus::Settling || status == ManeuverStatus::Braking;
}
Pose integrate(const Pose& p, double v, double w, double dt, double offset) {
  // Exact constant-twist offset model; the midpoint velocity approximates
  // each commanded acceleration ramp. This is NOT a native dynamics test.
  const double half = .5*w*dt, middle = p[3]+half;
  const double forward = v*dt*(half == 0. ? 1. : std::sin(half)/half);
  const double chord = 2.*offset*std::sin(half);
  return Pose(p[0]+forward*std::cos(middle)+chord*std::sin(middle),
              p[1]+forward*std::sin(middle)-chord*std::cos(middle), p[2],
              std::atan2(std::sin(p[3]+w*dt),std::cos(p[3]+w*dt)));
}
ManeuverSegment segment(const Pose& start, ManeuverKind kind, double amount, double offset) {
  return {kind,amount,start,maneuverReferenceEnd(start,kind,amount,offset),true,{}};
}
struct Row { ManeuverObservation observation; ManeuverCommand command; double control_time=0.; };
struct Run {
  std::vector<Row> rows, checkpoint_observations;
  ManeuverCommand result;
  double control_period=0., observation_period=0.;
  bool multirate=false;
};

Run run(const ManeuverPlan& plan, const ManeuverParameters& p, double dt, bool cancel_after_motion=false,
        const Eigen::Vector2d& initial_displacement=Eigen::Vector2d::Zero(), double initial_yaw_error=0.,
        double tail_yaw_gain=1.) {
  GroundManeuverController controller(p);
  ManeuverObservation o;
  o.pose = plan.segments.front().expected_start;
  o.pose.head<2>() += initial_displacement;
  o.pose[3] += initial_yaw_error;
  controller.start(plan,o,0.);
  ManeuverCommand previous;
  Run result;
  result.control_period=dt; result.observation_period=dt;
  bool cancelled = false, tail_gain_active = false;
  const std::size_t max_steps = static_cast<std::size_t>(std::ceil((p.total_timeout+p.stop_timeout+5.)/dt));
  for (std::size_t i=1; i<=max_steps; ++i) {
    o.stamp_s = i*dt;
    if (cancel_after_motion && !cancelled && std::abs(previous.v) >= .09) {
      controller.cancel(o.stamp_s);
      cancelled = true;
    }
    const auto command = controller.step(o,o.stamp_s);
    require(std::abs(command.v) <= p.max_speed+1e-12 && std::abs(command.w) <= p.max_yaw_rate+1e-12,
            "velocity limit violated");
    require(std::abs(command.v-previous.v) <= p.max_acceleration*dt+1e-12 &&
            std::abs(command.w-previous.w) <= p.max_yaw_acceleration*dt+1e-12,
            "acceleration limit violated");
    result.rows.push_back({o,command,o.stamp_s});
    result.result = command;
    if (!active(command.status)) break;
    // Optional, explicitly specified test plant: a single contact-response
    // step after the FIRST pivot has less than 0.30 rad left. Its condition
    // depends on measured geometry, never on controller state or version.
    if (tail_yaw_gain != 1. && !tail_gain_active) {
      const auto& first = plan.segments.front();
      require(first.kind == ManeuverKind::Pivot && std::abs(first.signed_amount) < pi,
              "gain-step fixture requires a first pivot shorter than pi");
      const double progress = std::copysign(1.,first.signed_amount)*
          std::atan2(std::sin(o.pose[3]-first.expected_start[3]),std::cos(o.pose[3]-first.expected_start[3]));
      tail_gain_active = std::abs(first.signed_amount)-progress < .30;
    }
    const double gain = tail_gain_active ? tail_yaw_gain : 1.;
    o.pose = integrate(o.pose,.5*(previous.v+command.v),gain*.5*(previous.w+command.w),dt,plan.control_offset);
    o.body_vx = command.v;
    o.body_vy = -plan.control_offset*gain*command.w;
    o.yaw_rate = gain*command.w;
    previous = command;
  }
  require(!active(result.result.status),"model driver exhausted bounded steps");
  return result;
}

Run runMultirate(const ManeuverPlan& plan, const ManeuverParameters& p,
                 double control_dt=.05, double observation_dt=.02) {
  // A separate ideal plant advances between sensor/control events under a
  // linear command ramp. Observations are sampled from that plant only at
  // sensor instants; no missing observation is reconstructed from two poses.
  // This remains a model driver, not a native/contact-dynamics measurement.
  require(std::isfinite(control_dt) && std::isfinite(observation_dt) &&
          control_dt>0. && control_dt<=p.maximum_step_dt && observation_dt>0.,
          "invalid multirate periods");
  const int64_t control_ns=std::llround(control_dt*1e9), observation_ns=std::llround(observation_dt*1e9);
  require(control_ns>0 && observation_ns>0,"multirate clock resolution");
  GroundManeuverController controller(p);
  ManeuverObservation observed;
  observed.pose=plan.segments.front().expected_start;
  controller.start(plan,observed,0.);
  Pose plant=observed.pose;
  ManeuverCommand ramp_from, ramp_to;
  double ramp_start=0., integrated_at=0.;
  int64_t next_control=control_ns, next_observation=observation_ns;
  const int64_t end_ns=std::llround((p.total_timeout+p.stop_timeout+5.)*1e9);
  Run result;
  result.control_period=control_dt; result.observation_period=observation_dt; result.multirate=true;
  while (std::min(next_control,next_observation)<=end_ns) {
    const int64_t event_ns=std::min(next_control,next_observation);
    const double now=event_ns*1e-9;
    const double left=std::clamp((integrated_at-ramp_start)/control_dt,0.,1.);
    const double right=std::clamp((now-ramp_start)/control_dt,0.,1.);
    const double v0=ramp_from.v+left*(ramp_to.v-ramp_from.v);
    const double v1=ramp_from.v+right*(ramp_to.v-ramp_from.v);
    const double w0=ramp_from.w+left*(ramp_to.w-ramp_from.w);
    const double w1=ramp_from.w+right*(ramp_to.w-ramp_from.w);
    plant=integrate(plant,.5*(v0+v1),.5*(w0+w1),now-integrated_at,plan.control_offset);
    integrated_at=now;
    if (event_ns==next_observation) {
      observed={plant,v1,-plan.control_offset*w1,w1,now};
      const auto before=controller.command();
      if (now<=p.total_timeout) controller.observePassThrough(observed,true);
      const auto after=controller.command();
      if (before.pass_through_consumed!=after.pass_through_consumed ||
          before.pass_through_missed!=after.pass_through_missed)
        result.checkpoint_observations.push_back({observed,after,ramp_start});
      next_observation+=observation_ns;
    }
    if (event_ns==next_control) {
      const auto command=controller.step(observed,now);
      require(std::abs(command.v)<=p.max_speed+1e-12 && std::abs(command.w)<=p.max_yaw_rate+1e-12,
              "multirate velocity limit violated");
      require(std::abs(command.v-ramp_to.v)<=p.max_acceleration*control_dt+1e-12 &&
              std::abs(command.w-ramp_to.w)<=p.max_yaw_acceleration*control_dt+1e-12,
              "multirate acceleration limit violated");
      result.rows.push_back({observed,command,now}); result.result=command;
      if (!active(command.status)) break;
      ramp_from=ramp_to; ramp_to=command; ramp_start=now;
      next_control+=control_ns;
    }
  }
  require(!active(result.result.status),"multirate driver exhausted bounded steps");
  return result;
}

void writePose(const Pose& pose) {
  std::cout << '[' << pose[0] << ',' << pose[1] << ',' << pose[2] << ',' << pose[3] << ']';
}

void writeRun(const Run& run, const ManeuverPlan& plan) {
  std::cout << std::setprecision(17)
            << "{\"model\":" << std::quoted(run.multirate?"offset_constant_twist_event_ramp":
                                                       "offset_constant_twist_midpoint_ramp")
            << ",\"native\":false,\"control_period_s\":" << run.control_period
            << ",\"observation_period_s\":" << run.observation_period
            << ",\"observation_source\":\"ideal_plant_samples_not_native_odometry\",\"offset\":"
            << plan.control_offset << ",\"status\":" << std::quoted(maneuverStatusName(run.result.status))
            << ",\"reason\":" << std::quoted(run.result.reason)
            << ",\"stop_confirmed\":" << (run.result.stop_confirmed ? "true" : "false")
            << ",\"pass_through_total\":" << run.result.pass_through_total
            << ",\"pass_through_consumed\":" << run.result.pass_through_consumed
            << ",\"goal\":";
  writePose(plan.segments.back().expected_end);
  std::cout << ",\"steps\":[";
  for (std::size_t i=0; i<run.rows.size(); ++i) {
    if (i) std::cout << ',';
    const auto& row = run.rows[i];
    std::cout << "{\"time\":" << row.observation.stamp_s << ",\"control_time_s\":" << row.control_time << ",\"pose\":";
    writePose(row.observation.pose);
    std::cout << ",\"body_vx\":" << row.observation.body_vx
              << ",\"body_vy\":" << row.observation.body_vy
              << ",\"yaw_rate\":" << row.observation.yaw_rate
              << ",\"command_v\":" << row.command.v << ",\"command_w\":" << row.command.w
              << ",\"segment\":" << row.command.segment_index
              << ",\"pass_through_total\":" << row.command.pass_through_total
              << ",\"pass_through_consumed\":" << row.command.pass_through_consumed
              << ",\"segment_pass_through_total\":" << row.command.segment_pass_through_total
              << ",\"segment_pass_through_consumed\":" << row.command.segment_pass_through_consumed
              << ",\"pass_through_last_consumed_stamp_s\":" << row.command.pass_through_last_consumed_stamp_s
              << ",\"pass_through_missed\":" << (row.command.pass_through_missed?"true":"false")
              << ",\"pass_through_miss_stamp_s\":" << row.command.pass_through_miss_stamp_s
              << ",\"status\":" << std::quoted(maneuverStatusName(row.command.status))
              << ",\"reference_error\":" << row.command.reference_error << '}';
  }
  std::cout << "],\"checkpoint_observations\":[";
  for (std::size_t i=0; i<run.checkpoint_observations.size(); ++i) {
    if (i) std::cout << ',';
    const auto& row=run.checkpoint_observations[i];
    std::cout << "{\"source_stamp_s\":" << row.observation.stamp_s << ",\"pose\":";
    writePose(row.observation.pose);
    std::cout << ",\"segment\":" << row.command.segment_index
              << ",\"pass_through_consumed\":" << row.command.pass_through_consumed
              << ",\"segment_pass_through_consumed\":" << row.command.segment_pass_through_consumed
              << ",\"pass_through_missed\":" << (row.command.pass_through_missed?"true":"false") << '}';
  }
  std::cout << "]}\n";
}

void terminalCaptureChecks() {
  // Controlled local-model counterexamples: the measured start is admitted
  // within the existing start band, but carries an independent lateral
  // displacement just outside the intermediate terminal band. No native
  // sensor record is altered or presented as a new physical measurement.
  ManeuverParameters p;
  p.max_speed = .3;
  p.total_timeout = 30.;
  p.max_reference_length = 5.;
  const double dt = .02;
  bool passed = true;
  std::cout << "{\"native\":false,\"scope\":\"offset-model terminal feedback\",\"cases\":[";
  bool first_case = true;
  auto exercise = [&](double direction, double offset, double displacement, bool expect_success) {
    const Pose start(.1,-.2,.4,direction*(pi-.003));
    const auto pivot = segment(start,ManeuverKind::Pivot,direction*.005,offset);
    const auto onward = segment(pivot.expected_end,ManeuverKind::Straight,.08,offset);
    const Eigen::Vector2d normal(-std::sin(pivot.expected_end[3]),std::cos(pivot.expected_end[3]));
    const double offset_sign = offset < 0. ? -1. : 1.;
    const auto trace = run(ManeuverPlan{{pivot,onward},offset,true},p,dt,false,
                           direction*offset_sign*displacement*normal);
    bool captured = false, prepared = false, transition = false;
    double capture_target = 0., boundary_xy = -1., boundary_yaw = -1.;
    for (const auto& row : trace.rows) {
      if (row.command.segment_index == 0) {
        require(direction*row.command.w >= -1e-12,"terminal capture reversed pivot direction");
        prepared = prepared || row.command.terminal_capture_preparing;
        if (row.command.terminal_capture_active) {
          if (captured) require(row.command.terminal_capture_progress == capture_target,
                                "terminal capture target was retuned while executing");
          captured = true;
          capture_target = row.command.terminal_capture_progress;
          require(capture_target >= std::abs(pivot.signed_amount)-p.segment_yaw_tolerance &&
                  capture_target <= std::abs(pivot.signed_amount)+p.segment_yaw_tolerance,
                  "terminal capture enlarged original progress band");
          require(row.command.v == 0.,"terminal capture moved its fixed-q model forward");
        }
      } else if (!transition) {
        transition = true;
        boundary_xy = (row.observation.pose-pivot.expected_end).head<2>().norm();
        boundary_yaw = std::abs(std::atan2(std::sin(row.observation.pose[3]-pivot.expected_end[3]),
                                         std::cos(row.observation.pose[3]-pivot.expected_end[3])));
        require(boundary_xy <= p.segment_position_tolerance && boundary_yaw <= p.segment_yaw_tolerance,
                "terminal capture bypassed original intermediate pose conditions");
        require(std::hypot(row.observation.body_vx,row.observation.body_vy) <= p.stop_speed_tolerance &&
                std::abs(row.observation.yaw_rate) <= p.stop_yaw_rate_tolerance,
                "terminal capture bypassed measured stop");
        require(!row.command.terminal_capture_active && !row.command.terminal_capture_preparing,
                "terminal capture leaked into the next primitive");
      }
    }
    const bool success = trace.result.status == ManeuverStatus::Succeeded && trace.result.stop_confirmed;
    const bool case_passed = expect_success ? success && captured && prepared && transition
                                            : !success && !captured && !transition && trace.result.stop_confirmed;
    passed = passed && case_passed;
    if (!first_case) std::cout << ',';
    first_case = false;
    std::cout << "{\"direction\":" << direction << ",\"offset\":" << offset
              << ",\"initial_lateral_error\":" << displacement << ",\"expected_success\":"
              << (expect_success?"true":"false") << ",\"status\":" << std::quoted(maneuverStatusName(trace.result.status))
              << ",\"elapsed_s\":" << trace.rows.back().observation.stamp_s
              << ",\"captured\":" << (captured?"true":"false") << ",\"target_progress\":" << capture_target
              << ",\"intermediate_xy_error\":" << boundary_xy << ",\"intermediate_yaw_error\":" << boundary_yaw
              << ",\"passed\":" << (case_passed?"true":"false") << '}';
  };
  for (double offset : {.2159,-.2159})
    for (double direction : {-1.,1.}) exercise(direction,offset,.011,true);
  exercise(-1.,.2159,.018,false);  // No same-direction terminal-circle intersection.
  exercise(1.,0.,.011,false);     // No translational authority from a zero-offset pivot.

  for (double direction : {-1.,1.}) {
    const auto revolution = segment(Pose::Zero(),ManeuverKind::Pivot,direction*2.*pi,.2159);
    GroundManeuverController controller(p);
    ManeuverObservation observation;
    controller.start(ManeuverPlan{{revolution},.2159,true},observation,0.);
    observation.stamp_s = dt;
    const auto command = controller.step(observation,dt);
    require(command.status == ManeuverStatus::Running && direction*command.w > 0. &&
            !command.terminal_capture_preparing && !command.terminal_capture_active,
            "wrapped full-turn endpoint entered terminal capture before cumulative progress");
  }
  std::cout << "],\"full_revolution_start_preserved\":true,\"passed\":" << (passed?"true":"false") << "}\n";
  require(passed,"terminal capture model case failed");
}

void brakingEnvelopeChecks() {
  // The reference is a regular, acceleration-limited trapezoid. The plant
  // begins 0.025 rad ahead in the commanded direction, inside the unchanged
  // 0.03 rad start band. Thus reference phase lags observed progress near
  // the endpoint. This is an explicit ideal-model perturbation, not replayed
  // or edited native odometry and not a claim of contact-dynamics validation.
  ManeuverParameters p;
  p.max_speed = .3;
  p.total_timeout = 30.;
  p.max_reference_length = 5.;
  const double dt = p.maximum_step_dt, offset = .2159, amount = .9, yaw_lead = .025;
  const double ramp = p.max_yaw_rate/p.max_yaw_acceleration;
  const double plateau = amount/p.max_yaw_rate-ramp;
  const double pivot_duration = 2.*ramp+plateau;
  const double straight_length = .08;
  const double straight_peak = std::sqrt(straight_length*p.max_acceleration);
  const double straight_ramp = straight_peak/p.max_acceleration;
  require(plateau > 0. && straight_peak <= p.max_speed && yaw_lead <= p.start_yaw_tolerance,
          "invalid braking envelope fixture");
  bool passed = true;
  std::cout << "{\"native\":false,\"scope\":\"20 Hz ideal model with admissible initial yaw lead\","
               "\"control_and_observation_period_s\":" << dt
            << ",\"initial_yaw_lead_rad\":" << yaw_lead << ",\"cases\":[";
  bool first = true;
  for (double direction : {-1.,1.}) {
    const Pose start(.1,-.2,.4,.3);
    auto pivot = segment(start,ManeuverKind::Pivot,direction*amount,offset);
    auto onward = segment(pivot.expected_end,ManeuverKind::Straight,straight_length,offset);
    pivot.reference_start_index = 0; pivot.reference_end_index = 3;
    onward.reference_start_index = 3; onward.reference_end_index = 5;
    ManeuverPlan plan{{pivot,onward},offset,true};
    plan.reference_times = {0.,ramp,ramp+plateau,pivot_duration,
                            pivot_duration+straight_ramp,pivot_duration+2.*straight_ramp};
    plan.reference_forward_velocities = {0.,0.,0.,0.,straight_peak,0.};
    plan.reference_yaw_rates = {0.,direction*p.max_yaw_rate,direction*p.max_yaw_rate,0.,0.,0.};
    const auto trace = run(plan,p,dt,false,Eigen::Vector2d::Zero(),direction*yaw_lead);
    bool boundary = false;
    double boundary_xy = -1., boundary_yaw = -1., first_settling_w = -1.;
    double max_progress = yaw_lead, max_command_braking_excess = 0.;
    for (const auto& row : trace.rows) {
      if (row.command.segment_index == 0) {
        const double progress = direction*std::atan2(std::sin(row.observation.pose[3]-start[3]),
                                                     std::cos(row.observation.pose[3]-start[3]));
        max_progress = std::max(max_progress,progress);
        require(direction*row.command.w >= -1e-12,"braking envelope reversed original pivot direction");
        if (row.command.status == ManeuverStatus::Settling && first_settling_w < 0.)
          first_settling_w = std::abs(row.command.w);
        // Diagnostic only: the final slew limiter cannot instantaneously
        // remove an inherited excess. Final pose/stop is the acceptance test.
        const double braking_rate = std::sqrt(2.*p.max_yaw_acceleration*std::max(0.,amount-progress));
        max_command_braking_excess = std::max(max_command_braking_excess,
                                             std::abs(row.command.w)-braking_rate);
      } else if (!boundary) {
        boundary = true;
        boundary_xy = (row.observation.pose-pivot.expected_end).head<2>().norm();
        boundary_yaw = std::abs(std::atan2(std::sin(row.observation.pose[3]-pivot.expected_end[3]),
                                         std::cos(row.observation.pose[3]-pivot.expected_end[3])));
        require(boundary_xy <= p.segment_position_tolerance && boundary_yaw <= p.segment_yaw_tolerance,
                "braking envelope bypassed intermediate pose");
        require(std::hypot(row.observation.body_vx,row.observation.body_vy) <= p.stop_speed_tolerance &&
                std::abs(row.observation.yaw_rate) <= p.stop_yaw_rate_tolerance,
                "braking envelope bypassed measured stop");
      }
    }
    const bool success = trace.result.status == ManeuverStatus::Succeeded && trace.result.stop_confirmed && boundary;
    passed = passed && success;
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"direction\":" << direction << ",\"status\":"
              << std::quoted(maneuverStatusName(trace.result.status)) << ",\"elapsed_s\":"
              << trace.rows.back().observation.stamp_s << ",\"first_settling_command_yaw_rate\":" << first_settling_w
              << ",\"max_observed_pivot_progress\":" << max_progress
              << ",\"max_command_classical_braking_excess\":" << max_command_braking_excess
              << ",\"intermediate_xy_error\":" << boundary_xy << ",\"intermediate_yaw_error\":" << boundary_yaw
              << ",\"passed\":" << (success?"true":"false") << '}';
  }
  std::cout << "],\"passed\":" << (passed?"true":"false") << "}\n";
  require(passed,"braking envelope model case failed");
}

bool measuredBrakeObservationChecks(const ManeuverParameters& p, double offset) {
  GroundManeuverController controller(p);
  const auto pivot = segment(Pose::Zero(),ManeuverKind::Pivot,.6,offset);
  ManeuverObservation o;
  controller.start(ManeuverPlan{{pivot},offset,true},o,0.);
  for (int i=1; i<=4; ++i) {
    o.stamp_s = i*.05;
    controller.step(o,o.stamp_s);
  }
  o.pose = maneuverReferenceEnd(Pose::Zero(),ManeuverKind::Pivot,.5,offset);
  o.yaw_rate = .4; o.body_vy = -offset*.4; o.stamp_s = .25;
  auto command = controller.step(o,.25);
  if (!command.measured_brake_active) return false;  // Old-core comparison.
  require(command.measured_brake_count == 1,"measured brake trigger count differs");
  double previous = command.w;
  for (int i=1; i<=6; ++i) {
    command = controller.step(o,.25+i*.025);  // Same source, legal control time.
    require(command.measured_brake_active && command.measured_brake_count == 1 &&
            command.w <= previous && previous-command.w <= p.max_yaw_acceleration*.025+1e-12,
            "duplicate observation interrupted bounded measured braking");
    previous = command.w;
  }
  require(command.w == 0.,"duplicate observation froze a nonzero brake command");
  o.yaw_rate = o.body_vy = 0.; o.stamp_s = .27;
  require(controller.step(o,.42).measured_brake_active,"one source sample released measured brake");
  require(controller.step(o,.44).measured_brake_active && controller.step(o,.46).measured_brake_active,
          "duplicate stationary observations released measured brake");
  o.stamp_s = .29;
  require(controller.step(o,.48).measured_brake_active,"two source samples released measured brake");
  o.stamp_s = .31;
  command = controller.step(o,.50);
  require(!command.measured_brake_active && command.measured_brake_count == 1 && command.segment_index == 0,
          "measured brake did not resume the same primitive after observed stop");
  o.stamp_s = .33; o.yaw_rate = .4; o.body_vy = -offset*.4;
  require(controller.step(o,.55).measured_brake_active,"second observed overspeed did not brake");
  controller.cancel(.55);
  require(!controller.command().measured_brake_active && controller.command().measured_brake_count == 2 &&
          controller.command().status == ManeuverStatus::Braking,"cancel did not supersede measured brake");
  return true;
}

void measuredBrakingChecks() {
  ManeuverParameters p;
  p.max_speed = .3; p.total_timeout = 30.; p.max_reference_length = 5.;
  const double dt = p.maximum_step_dt, offset = .2159, amount = 2.547175616;
  const double ramp = p.max_yaw_rate/p.max_yaw_acceleration;
  const double plateau = amount/p.max_yaw_rate-ramp, pivot_duration = 2.*ramp+plateau;
  const double straight_length = .08, straight_peak = std::sqrt(straight_length*p.max_acceleration);
  const double straight_ramp = straight_peak/p.max_acceleration;
  bool passed = true, first_case = true;
  std::cout << "{\"native\":false,\"scope\":\"20 Hz ideal offset plant with fixed tail gain step\","
            << "\"control_and_observation_period_s\":" << dt
            << ",\"gain_switch_remaining_rad\":0.30,\"cases\":[";
  for (double gain : {1.,1.28}) for (double direction : {-1.,1.}) {
    const Pose start(.1,-.2,.4,.7);
    auto pivot = segment(start,ManeuverKind::Pivot,direction*amount,offset);
    auto onward = segment(pivot.expected_end,ManeuverKind::Straight,straight_length,offset);
    pivot.reference_start_index = 0; pivot.reference_end_index = 3;
    onward.reference_start_index = 3; onward.reference_end_index = 5;
    ManeuverPlan plan{{pivot,onward},offset,true};
    plan.reference_times = {0.,ramp,ramp+plateau,pivot_duration,
                            pivot_duration+straight_ramp,pivot_duration+2.*straight_ramp};
    plan.reference_forward_velocities = {0.,0.,0.,0.,straight_peak,0.};
    plan.reference_yaw_rates = {0.,direction*p.max_yaw_rate,direction*p.max_yaw_rate,0.,0.,0.};
    const auto trace = run(plan,p,dt,false,Eigen::Vector2d::Zero(),0.,gain);
    bool boundary = false;
    double boundary_xy = -1., boundary_yaw = -1., max_progress = 0., peak_rate = 0.;
    unsigned brake_count = 0;
    double first_trigger_time = -1., first_trigger_remaining = -1.;
    for (const auto& row : trace.rows) {
      peak_rate = std::max(peak_rate,std::abs(row.observation.yaw_rate));
      if (row.command.measured_brake_count > brake_count) {
        require(row.command.measured_brake_count == brake_count+1,"measured brake count skipped an episode");
        brake_count = row.command.measured_brake_count;
        if (first_trigger_time < 0.) {
          first_trigger_time = row.observation.stamp_s;
          first_trigger_remaining = row.command.measured_brake_remaining;
        }
      }
      if (row.command.segment_index == 0) {
        const double progress = direction*std::atan2(std::sin(row.observation.pose[3]-start[3]),
                                                     std::cos(row.observation.pose[3]-start[3]));
        max_progress = std::max(max_progress,progress);
        require(direction*row.command.w >= -1e-12,"measured brake reversed original pivot");
      } else if (!boundary) {
        boundary = true;
        boundary_xy = (row.observation.pose-pivot.expected_end).head<2>().norm();
        boundary_yaw = std::abs(std::atan2(std::sin(row.observation.pose[3]-pivot.expected_end[3]),
                                         std::cos(row.observation.pose[3]-pivot.expected_end[3])));
        require(boundary_xy <= p.segment_position_tolerance && boundary_yaw <= p.segment_yaw_tolerance &&
                std::hypot(row.observation.body_vx,row.observation.body_vy) <= p.stop_speed_tolerance &&
                std::abs(row.observation.yaw_rate) <= p.stop_yaw_rate_tolerance,
                "measured brake bypassed original intermediate pose or stop");
        require(!row.command.measured_brake_active && row.command.measured_brake_remaining == 0. &&
                row.command.measured_brake_stop_distance == 0.,"measured brake leaked into next primitive");
      }
    }
    const bool success = trace.result.status == ManeuverStatus::Succeeded && trace.result.stop_confirmed && boundary;
    passed = passed && success;
    if (!first_case) std::cout << ',';
    first_case = false;
    std::cout << "{\"tail_gain\":" << gain << ",\"direction\":" << direction << ",\"status\":"
              << std::quoted(maneuverStatusName(trace.result.status)) << ",\"elapsed_s\":"
              << trace.rows.back().observation.stamp_s << ",\"measured_brake_count\":" << brake_count
              << ",\"first_trigger_s\":" << first_trigger_time << ",\"first_trigger_remaining\":" << first_trigger_remaining
              << ",\"peak_measured_yaw_rate\":" << peak_rate << ",\"max_observed_pivot_progress\":" << max_progress
              << ",\"intermediate_xy_error\":" << boundary_xy << ",\"intermediate_yaw_error\":" << boundary_yaw
              << ",\"passed\":" << (success?"true":"false") << '}';
  }
  const bool observation_checks = measuredBrakeObservationChecks(p,offset);
  for (double direction : {-1.,1.}) {
    GroundManeuverController controller(p);
    const auto full = segment(Pose::Zero(),ManeuverKind::Pivot,direction*2.*pi,offset);
    ManeuverObservation o;
    controller.start(ManeuverPlan{{full},offset,true},o,0.);
    o.stamp_s = dt; o.yaw_rate = direction*.4; o.body_vy = -offset*o.yaw_rate;
    const auto command = controller.step(o,dt);
    require(command.status == ManeuverStatus::Running && direction*command.w > 0. &&
            !command.measured_brake_active && command.measured_brake_count == 0,
            "measured brake treated full-turn wrapped yaw as final progress");
  }
  passed = passed && observation_checks;
  std::cout << "],\"duplicate_observation_and_cancel_checks\":" << (observation_checks?"true":"false")
            << ",\"full_revolution_start_preserved\":true,\"passed\":" << (passed?"true":"false") << "}\n";
  require(passed,"measured braking model case failed");
}

void fixture(const std::string& filename, bool multirate=false) {
  std::ifstream file(filename);
  if (!file) throw std::runtime_error("cannot open fixture");
  // Numeric, whitespace-separated; '#' starts an optional line comment.
  // Header: offset max_v max_w max_a max_alpha dt segment_count
  // Rows: kind(0=straight,1=pivot) amount start[x y z yaw] end[x y z yaw]
  // Optional suffix: crossings row_count, followed by rows of
  // segment_index(0-based) checkpoint_count positive_distance_1 ...;
  // an optional profile count then carries index time v w for the producer's
  // approved timing profile.
  std::ostringstream content;
  for (std::string line; std::getline(file,line);) content << line.substr(0,line.find('#')) << '\n';
  std::istringstream input(content.str());
  ManeuverPlan plan;
  ManeuverParameters p;
  // Match the saved preflight experiment's finite execution/reference budget.
  p.total_timeout = 30.;
  p.max_reference_length = 5.;
  double dt;
  std::size_t count;
  if (!(input >> plan.control_offset >> p.max_speed >> p.max_yaw_rate >> p.max_acceleration
              >> p.max_yaw_acceleration >> dt >> count) || !std::isfinite(dt) || dt <= 0. ||
      dt > p.maximum_step_dt || count < 1 || count > p.max_segments)
    throw std::runtime_error("invalid fixture header");
  for (std::size_t i=0; i<count; ++i) {
    int kind;
    ManeuverSegment s;
    if (!(input >> kind >> s.signed_amount) || (kind != 0 && kind != 1))
      throw std::runtime_error("invalid fixture primitive");
    s.kind = kind == 0 ? ManeuverKind::Straight : ManeuverKind::Pivot;
    for (int j=0; j<4; ++j) if (!(input >> s.expected_start[j])) throw std::runtime_error("invalid start pose");
    for (int j=0; j<4; ++j) if (!(input >> s.expected_end[j])) throw std::runtime_error("invalid end pose");
    plan.segments.push_back(s);
  }
  std::string excess;
  while (input >> excess) {
    if (excess=="crossings") {
      std::size_t rows, total=0;
      if (!(input >> rows) || rows>count) throw std::runtime_error("invalid fixture crossings header");
      std::vector<bool> seen(count,false);
      for (std::size_t i=0; i<rows; ++i) {
        std::size_t index, points;
        if (!(input >> index >> points) || index>=count || seen[index] || points==0 || points>64-total)
          throw std::runtime_error("invalid fixture crossings row");
        seen[index]=true; total+=points;
        auto& distances=plan.segments[index].pass_through_distances;
        for (std::size_t j=0; j<points; ++j) {
          double distance;
          if (!(input >> distance)) throw std::runtime_error("invalid fixture crossing distance");
          distances.push_back(distance);
        }
      }
    } else if (excess=="segment_indices") {
      std::size_t rows;
      if (!(input >> rows) || rows!=count) throw std::runtime_error("invalid fixture segment index header");
      for (std::size_t i=0; i<rows; ++i) {
        std::size_t index, first, last;
        if (!(input >> index >> first >> last) || index>=count || first>=last)
          throw std::runtime_error("invalid fixture segment index row");
        plan.segments[index].reference_start_index=first;
        plan.segments[index].reference_end_index=last;
      }
    } else if (excess=="profile") {
      std::size_t profile_size;
      if (!(input >> profile_size) || profile_size < 2 || profile_size > 20000)
        throw std::runtime_error("invalid fixture profile header");
      plan.reference_times.resize(profile_size);
      plan.reference_forward_velocities.resize(profile_size);
      plan.reference_yaw_rates.resize(profile_size);
      for (std::size_t i=0; i<profile_size; ++i) {
        std::size_t index;
        if (!(input >> index >> plan.reference_times[i] >> plan.reference_forward_velocities[i] >>
              plan.reference_yaw_rates[i]) || index != i)
          throw std::runtime_error("invalid fixture profile row");
      }
    } else throw std::runtime_error("unexpected fixture content");
  }
  // The fixture producer owns real map approval; this standalone model
  // runner exercises the core, and emits no claim of external validation.
  plan.geometry_validated = true;
  if (multirate) {
    require(std::abs(dt-.05)<1e-12,"multirate fixture requires 0.05 second control period");
    writeRun(runMultirate(plan,p,dt,.02),plan);
  } else writeRun(run(plan,p,dt),plan);
}

void passThroughChecks(const ManeuverParameters& p, double offset, double dt) {
  for (double direction : {-1.,1.}) {
    auto straight=segment(Pose(.1,-.2,.4,.3),ManeuverKind::Straight,direction*.35,offset);
    straight.pass_through_distances={.10,.25};
    const auto trace=run(ManeuverPlan{{straight},offset,true},p,dt);
    require(trace.result.status==ManeuverStatus::Succeeded && trace.result.stop_confirmed &&
            trace.result.pass_through_total==2 && trace.result.pass_through_consumed==2,
            "signed pass-through route did not complete all checkpoints");
    std::size_t previous=0;
    for (const auto& row : trace.rows) {
      require(row.command.pass_through_consumed>=previous,"pass-through count regressed");
      if (row.command.pass_through_consumed!=previous)
        require(row.command.status==ManeuverStatus::Running && std::abs(row.command.v)>p.stop_speed_tolerance,
                "interior checkpoint added a stopped boundary");
      previous=row.command.pass_through_consumed;
    }
  }
  // Keep cumulative evidence, but reset only the current segment's cursor.
  auto first=segment(Pose::Zero(),ManeuverKind::Straight,.3,offset);
  auto second=segment(first.expected_end,ManeuverKind::Straight,.3,offset);
  first.pass_through_distances={.15}; second.pass_through_distances={.15};
  const auto two=run(ManeuverPlan{{first,second},offset,true},p,dt);
  bool boundary_seen=false;
  for (const auto& row : two.rows)
    if (row.command.segment_index==1 && row.command.segment_pass_through_consumed==0) {
      boundary_seen=true;
      require(row.command.pass_through_consumed==1 && row.command.segment_pass_through_total==1,
              "new segment lost cumulative checkpoint evidence");
    }
  require(boundary_seen && two.result.status==ManeuverStatus::Succeeded &&
          two.result.pass_through_consumed==2,"pass-through segment cursor did not reset");

  auto near=segment(Pose::Zero(),ManeuverKind::Straight,1.,offset);
  near.pass_through_distances={.005,.2};
  GroundManeuverController observed(p);
  ManeuverObservation o;
  observed.start(ManeuverPlan{{near},offset,true},o,0.);
  o.stamp_s=dt;
  require(observed.step(o,dt).pass_through_consumed==0,"start proximity consumed a future checkpoint");
  o.pose[0]=.0051; o.stamp_s=2.*dt;
  require(observed.step(o,2.*dt).pass_through_consumed==1,"new crossed checkpoint was not observed");
  o.pose[0]=.2;
  require(observed.step(o,3.*dt).pass_through_consumed==1,"duplicate source consumed a checkpoint");
  o.pose[3]=.03; o.stamp_s=4.*dt;
  require(observed.step(o,4.*dt).pass_through_consumed==1,"checkpoint accepted the wrong heading");
  o.pose[3]=0.; o.pose[1]=.015; o.stamp_s=5.*dt;
  require(observed.step(o,5.*dt).pass_through_consumed==1,"checkpoint accepted lateral error outside its band");
  o.pose[1]=0.; o.stamp_s=6.*dt;
  require(observed.step(o,6.*dt).pass_through_consumed==2,"fresh checkpoint pose was not consumed");

  // The final endpoint's wider tolerance cannot bypass a pending near-end
  // checkpoint. Feedback continues until that ordered obligation is observed.
  auto tail=segment(Pose::Zero(),ManeuverKind::Straight,1.,offset);
  tail.pass_through_distances={.98};
  GroundManeuverController endpoint(p);
  o=ManeuverObservation{};
  endpoint.start(ManeuverPlan{{tail},offset,true},o,0.);
  o.pose[0]=.965; o.stamp_s=dt;
  const auto pending=endpoint.step(o,dt);
  require(pending.status==ManeuverStatus::Running && pending.v>0. && pending.pass_through_consumed==0,
          "endpoint tolerance bypassed an unobserved checkpoint");
  o.pose[0]=.985;
  ManeuverCommand arrived;
  for (unsigned i=2; i<=p.settled_observations+2; ++i) {
    o.stamp_s=i*dt; arrived=endpoint.step(o,o.stamp_s);
  }
  require(arrived.status==ManeuverStatus::Succeeded && arrived.pass_through_consumed==1 && arrived.stop_confirmed,
          "final stop did not retain the consumed checkpoint");

  // A jump over the next tolerance band is failure, never nearest-point or
  // time-based advancement. Existing slew-limited braking still owns output.
  auto missed=segment(Pose::Zero(),ManeuverKind::Straight,1.,offset);
  missed.pass_through_distances={.2};
  GroundManeuverController missing(p);
  o=ManeuverObservation{};
  missing.start(ManeuverPlan{{missed},offset,true},o,0.);
  ManeuverCommand last;
  for (unsigned i=1; i<=5; ++i) { o.stamp_s=i*dt; last=missing.step(o,o.stamp_s); }
  o.pose[0]=.212; o.stamp_s=6.*dt;
  auto failure=missing.step(o,o.stamp_s);
  require(failure.status==ManeuverStatus::Braking && failure.reason=="pass_through_checkpoint_missed" &&
          failure.pass_through_consumed==0 && failure.v<last.v &&
          last.v-failure.v<=p.max_acceleration*dt+1e-12,"missed checkpoint bypassed bounded braking");
  for (unsigned i=7; i<50 && active(failure.status); ++i) {
    o.stamp_s=i*dt; failure=missing.step(o,o.stamp_s);
  }
  require(failure.status==ManeuverStatus::Failed && failure.stop_confirmed && failure.pass_through_consumed==0,
          "missed checkpoint did not terminate with measured stop");

  const auto rejected=[&](const ManeuverPlan& plan) {
    GroundManeuverController admission(p);
    ManeuverObservation initial;
    initial.pose=plan.segments.front().expected_start;
    try { admission.start(plan,initial,0.); }
    catch (const std::invalid_argument&) { return true; }
    return false;
  };
  for (const std::vector<double>& distances : std::vector<std::vector<double>>{
       {0.},{-.1},{1.},{.2,.2},{.3,.2},{std::numeric_limits<double>::quiet_NaN()},
       {std::numeric_limits<double>::infinity()}}) {
    auto invalid=segment(Pose::Zero(),ManeuverKind::Straight,1.,offset);
    invalid.pass_through_distances=distances;
    require(rejected(ManeuverPlan{{invalid},offset,true}),"invalid checkpoint distances were admitted");
  }
  auto pivot=segment(Pose::Zero(),ManeuverKind::Pivot,.5,offset);
  pivot.pass_through_distances={.2};
  require(rejected(ManeuverPlan{{pivot},offset,true}),"pivot checkpoint was admitted");
  auto full=segment(Pose::Zero(),ManeuverKind::Straight,1.,offset);
  for (unsigned i=1; i<=64; ++i) full.pass_through_distances.push_back(i*.01);
  require(!rejected(ManeuverPlan{{full},offset,true}),"64 valid checkpoints were refused");
  auto extra=segment(full.expected_end,ManeuverKind::Straight,.3,offset);
  extra.pass_through_distances={.15};
  require(rejected(ManeuverPlan{{full,extra},offset,true}),"total checkpoint cap was applied only per segment");
}

void passThroughObservationChecks(const ManeuverParameters& p, double offset) {
  auto straight=segment(Pose::Zero(),ManeuverKind::Straight,1.,offset);
  straight.pass_through_distances={.2,.6};
  ManeuverPlan plan{{straight},offset,true};
  GroundManeuverController controller(p);
  ManeuverObservation o;
  controller.start(plan,o,0.);
  o.pose[0]=.199; o.stamp_s=.02;
  controller.observePassThrough(o,true);
  require(controller.command().pass_through_consumed==0,"sensor proximity counted as crossing");
  o.pose[0]=.205; o.stamp_s=.04;
  controller.observePassThrough(o,false);
  require(controller.command().pass_through_consumed==0,"stale sensor observation consumed a checkpoint");
  auto invalid=o; invalid.body_vy=std::numeric_limits<double>::quiet_NaN();
  controller.observePassThrough(invalid,true);
  require(controller.command().pass_through_consumed==0,"invalid sensor observation consumed a checkpoint");
  const auto before=controller.command();
  controller.observePassThrough(o,true);
  const auto observed=controller.command();
  require(observed.pass_through_consumed==1 && observed.pass_through_last_consumed_stamp_s==.04 &&
          observed.v==before.v && observed.w==before.w && observed.status==before.status &&
          observed.segment_index==before.segment_index,"sensor callback changed control state or lost crossing");
  const auto controlled=controller.step(o,.05);
  require(controlled.pass_through_consumed==1 && std::abs(controlled.v-p.max_acceleration*.05)<1e-12,
          "sensor callback advanced the control clock or fallback double-counted");
  o.pose[0]=.605;
  controller.observePassThrough(o,true);
  o.stamp_s=.03; controller.observePassThrough(o,true);
  require(controller.command().pass_through_consumed==1,"duplicate or reversed sensor stamp was consumed");
  o.stamp_s=.06; controller.observePassThrough(o,true);
  require(controller.command().pass_through_consumed==2,"next real sensor observation was discarded");
  controller.cancel(.1);
  for (unsigned i=1; i<30 && active(controller.command().status); ++i) {
    o.stamp_s=.1+i*.02; controller.step(o,o.stamp_s);
  }
  require(controller.command().status==ManeuverStatus::Cancelled,"sensor test cancellation failed");
  o=ManeuverObservation{};
  controller.start(plan,o,0.);
  require(controller.command().pass_through_consumed==0 && !controller.command().pass_through_missed &&
          controller.command().pass_through_last_consumed_stamp_s==0.,"new action inherited old crossing ledger");
  o.pose[0]=.205; o.stamp_s=.02; controller.observePassThrough(o,true);
  require(controller.command().pass_through_consumed==1,"new action retained old source-stamp barrier");

  GroundManeuverController missed(p);
  o=ManeuverObservation{};
  missed.start(plan,o,0.);
  o.stamp_s=.02; const auto moving=missed.step(o,.05);
  o.pose[0]=.212; o.stamp_s=.04;
  missed.observePassThrough(o,true);
  const auto pending=missed.command();
  require(pending.pass_through_missed && pending.pass_through_miss_stamp_s==.04 &&
          pending.v==moving.v && pending.w==moving.w && pending.status==moving.status,
          "sensor miss commanded braking instead of recording pending failure");
  // A later good pose cannot erase an already observed missed obligation.
  o.pose[0]=.205; o.stamp_s=.06; missed.observePassThrough(o,true);
  require(missed.command().pass_through_consumed==0,"later observation erased a pending checkpoint miss");
  const auto braking=missed.step(o,.1);
  require(braking.status==ManeuverStatus::Braking && braking.reason=="pass_through_checkpoint_missed",
          "control timer ignored pending sensor miss");

  auto first=segment(Pose::Zero(),ManeuverKind::Straight,.3,offset);
  auto second=segment(first.expected_end,ManeuverKind::Straight,.3,offset);
  second.pass_through_distances={.005};
  GroundManeuverController phase(p);
  o=ManeuverObservation{};
  phase.start(ManeuverPlan{{first,second},offset,true},o,0.);
  for (unsigned i=1; i<=p.settled_observations; ++i) {
    o.pose[0]=.305; o.stamp_s=i*.02;
    phase.observePassThrough(o,true); phase.step(o,o.stamp_s);
  }
  require(phase.command().segment_index==1 && phase.command().pass_through_consumed==0,
          "prior-primitive sensor observation leaked into new primitive");
  o.pose[0]=.3051; phase.observePassThrough(o,true);
  require(phase.command().pass_through_consumed==0,"phase boundary replay consumed a checkpoint");
  o.stamp_s+=.02; phase.observePassThrough(o,true);
  require(phase.command().pass_through_consumed==1,"post-boundary source observation was not consumed");

  ManeuverParameters native_rates=p;
  native_rates.max_speed=.3;
  for (double direction : {-1.,1.}) {
    auto route=segment(Pose::Zero(),ManeuverKind::Straight,direction,offset);
    route.pass_through_distances={.2,.4,.6};
    const auto trace=runMultirate(ManeuverPlan{{route},offset,true},native_rates,.05,.02);
    require(trace.result.status==ManeuverStatus::Succeeded && trace.result.pass_through_consumed==3,
            "50 Hz observation / 20 Hz control model missed a straight checkpoint");
    bool between_control_ticks=false;
    for (const auto& event : trace.checkpoint_observations) {
      const double sensor_tick=event.observation.stamp_s/.02, control_tick=event.observation.stamp_s/.05;
      require(std::abs(sensor_tick-std::round(sensor_tick))<1e-8,"checkpoint evidence invented a sensor sample");
      if (std::abs(control_tick-std::round(control_tick))>1e-8) between_control_ticks=true;
    }
    require(between_control_ticks,"multirate test did not exercise an inter-control observation");
  }
}

void checks() {
  ManeuverParameters p;
  const double offset = .17, dt = .02;
  for (double direction : {-1.,1.}) {
    ManeuverPlan plan{{segment(Pose(.1,-.2,.4,.3),ManeuverKind::Straight,direction*.35,offset)},offset,true};
    const auto trace = run(plan,p,dt);
    require(trace.result.status == ManeuverStatus::Succeeded && trace.result.stop_confirmed,"signed straight did not settle");
    require(trace.result.pass_through_total==0 && trace.result.pass_through_consumed==0,
            "empty checkpoint route changed legacy behavior");
    for (const auto& row : trace.rows) require(direction*row.command.v >= -1e-12,"straight changed travel direction");
  }
  ManeuverPlan pivot{{segment(Pose(.1,-.2,.4,.3),ManeuverKind::Pivot,-pi/3.,offset)},offset,true};
  const auto trace = run(pivot,p,dt);
  require(trace.result.status == ManeuverStatus::Succeeded,"planned pivot did not settle");
  const auto initial = pivot.segments.front().expected_start;
  const Eigen::Vector2d q(initial[0]+offset*std::cos(initial[3]),initial[1]+offset*std::sin(initial[3]));
  for (const auto& row : trace.rows) {
    const auto& x = row.observation.pose;
    const Eigen::Vector2d observed_q(x[0]+offset*std::cos(x[3]),x[1]+offset*std::sin(x[3]));
    require((observed_q-q).norm() < 1e-10,"pivot held base instead of following offset reference");
    require(row.command.w <= 1e-12,"pivot changed its declared turn direction");
  }
  require((trace.rows.back().observation.pose-initial).head<2>().norm() > .1,"pivot unexpectedly left base fixed");
  ManeuverPlan cancellable{{segment(Pose::Zero(),ManeuverKind::Straight,1.,offset)},offset,true};
  const auto cancelled = run(cancellable,p,dt,true);
  require(cancelled.result.status == ManeuverStatus::Cancelled && cancelled.result.stop_confirmed,
          "cancellation did not confirm bounded braking");
  bool braking_seen = false;
  double previous_v = 0.;
  for (const auto& row : cancelled.rows) {
    if (row.command.status == ManeuverStatus::Braking) {
      if (braking_seen) require(std::abs(row.command.v) <= std::abs(previous_v)+1e-12,"cancel accelerated again");
      braking_seen = true;
    }
    previous_v = row.command.v;
  }
  require(braking_seen,"cancellation skipped the braking state");
  passThroughChecks(p,offset,dt);
  passThroughObservationChecks(p,offset);
  std::cout << "{\"status\":\"passed\",\"checks\":[\"signed_straight\",\"planned_offset_pivot\","
               "\"cancel_with_limited_braking\",\"ordered_pass_through_signed_straight\","
               "\"pass_through_segment_cursor\",\"pass_through_new_observation_and_pose\","
               "\"pass_through_required_before_final_stop\",\"missed_pass_through_braking\","
               "\"pass_through_contract_validation\",\"pass_through_sensor_freshness_and_clock_separation\","
               "\"pass_through_action_and_phase_isolation\",\"pass_through_sensor_miss_latch\","
               "\"pass_through_50hz_observation_20hz_control_model\"],\"native\":false}\n";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 1) checks();
    else if (argc == 2 && std::string(argv[1]) == "--terminal-capture-model") terminalCaptureChecks();
    else if (argc == 2 && std::string(argv[1]) == "--braking-envelope-model") brakingEnvelopeChecks();
    else if (argc == 2 && std::string(argv[1]) == "--measured-braking-model") measuredBrakingChecks();
    else if (argc == 3 && std::string(argv[1]) == "--fixture") fixture(argv[2]);
    else if (argc == 3 && std::string(argv[1]) == "--fixture-multirate") fixture(argv[2],true);
    else throw std::runtime_error("usage: ground_maneuver_demo [--terminal-capture-model|--braking-envelope-model|--measured-braking-model|--fixture numeric_file|--fixture-multirate numeric_file]");
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
