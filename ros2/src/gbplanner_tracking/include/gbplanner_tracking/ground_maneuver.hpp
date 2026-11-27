#pragma once

#include "gbplanner_tracking/pure_pursuit.hpp"

#include <limits>
#include <string>
#include <vector>

namespace gbplanner_tracking {

enum class ManeuverKind { Straight, Pivot };
enum class ManeuverStatus { Idle, Running, Settling, Braking, Succeeded, Cancelled, Failed };
const char* maneuverStatusName(ManeuverStatus status);

struct ManeuverSegment {
  ManeuverKind kind = ManeuverKind::Straight;
  // Finite and nonzero; no positive minimum magnitude. Tiny segments keep
  // their approved geometry and observed-stop boundary. Sign is binding.
  double signed_amount = 0.;  // Straight metres or pivot radians.
  Pose expected_start = Pose::Zero();
  Pose expected_end = Pose::Zero();
  bool stop_at_end = true;    // This first core rejects continuous boundaries.
  // Ordered positive distances in the declared straight direction from
  // expected_start. Interior observations are required, without an added stop.
  // Pivots cannot carry these checkpoints; both stopped anchors remain intact.
  std::vector<double> pass_through_distances;
  // Indices into the optional timed reference arrays in ManeuverPlan.
  std::size_t reference_start_index = 0, reference_end_index = 0;
};

struct ManeuverPlan {
  std::vector<ManeuverSegment> segments;
  double control_offset = std::numeric_limits<double>::quiet_NaN();
  // Caller attests full geometry and original route-progress validation.
  // This library checks consistency, not collision/support/map validity.
  bool geometry_validated = false;
  // The producer's approved velocity profile. Empty means use the legacy
  // feedback law (standalone fixtures); populated plans use it as bounded
  // feed-forward while retaining measured-error and limit checks.
  std::vector<double> reference_times, reference_forward_velocities, reference_yaw_rates;
};

struct ManeuverObservation {
  Pose pose = Pose::Zero();  // Measured base_link, never inferred from the model.
  double body_vx = 0.;
  double body_vy = 0.;
  double yaw_rate = 0.;
  double stamp_s = 0.;      // Source clock: strictly increasing new observations.
};

struct ManeuverParameters {
  double max_speed = .15, max_yaw_rate = .4;
  double max_acceleration = .3, max_yaw_acceleration = .6;
  double position_tolerance = .04, yaw_tolerance = .1;
  double segment_position_tolerance = .01, segment_yaw_tolerance = .02;
  double height_tolerance = .04;
  double stop_speed_tolerance = .02, stop_yaw_rate_tolerance = .03;
  unsigned settled_observations = 3;
  double reference_tolerance = .08, straight_heading_limit = .35;
  double start_position_tolerance = .02, start_yaw_tolerance = .03;
  double longitudinal_gain = 1.5, heading_gain = 2., lateral_gain = 1.;
  double velocity_damping = .25, pivot_correction_speed = .05;
  double odometry_timeout = .25, segment_timeout = 30., total_timeout = 120.;
  double stop_timeout = 2., maximum_step_dt = .05;
  double max_reference_length = 10.;
  std::size_t max_segments = 64;
};

struct ManeuverCommand {
  double v = 0., w = 0.;  // Body-forward and yaw commands; no ROS publication.
  ManeuverStatus status = ManeuverStatus::Idle;
  std::size_t segment_index = 0;
  std::size_t pass_through_total = 0, pass_through_consumed = 0;
  std::size_t segment_pass_through_total = 0, segment_pass_through_consumed = 0;
  double pass_through_last_consumed_stamp_s = 0.;
  bool pass_through_missed = false;
  double pass_through_miss_stamp_s = 0.;
  double reference_error = 0.;
  // A one-shot, same-direction pivot correction inside the ORIGINAL endpoint
  // bands. The target is positive signed-direction cumulative angular
  // progress, not wrapped yaw; it never replaces the planned signed amount.
  bool terminal_capture_preparing = false, terminal_capture_active = false;
  double terminal_capture_progress = 0.;
  // Nonterminal measured-yaw braking episodes. Count is cumulative within
  // the goal; distance/remaining describe this segment's last trigger (zero
  // until it has one), and reset when the next segment starts.
  bool measured_brake_active = false;
  unsigned measured_brake_count = 0;
  double measured_brake_stop_distance = 0., measured_brake_remaining = 0.;
  bool stop_confirmed = false;
  std::string reason = "idle";
};

// Analytic REFERENCE endpoint for q = p + offset*heading. It is not an
// estimate of real motion and does not modify an observation.
Pose maneuverReferenceEnd(const Pose& start, ManeuverKind kind,
                          double signed_amount, double control_offset);

// Standalone closed-loop execution core. No ROS, map mutation or implicit
// recovery paths. The caller owns exclusive command output and must retain
// the original PathProgress/live geometry checks, calling abort on failure.
// A constant offset defines only the approved reference; all feedback,
// segment transitions and success use actual base observations.
class GroundManeuverController {
 public:
  explicit GroundManeuverController(ManeuverParameters parameters = {});
  // Throws before admission for inconsistent/unapproved plans, a moving or
  // mismatched start, or an already active controller. Clock is monotonic
  // caller time, independent of the observation's source clock.
  void start(const ManeuverPlan& plan, const ManeuverObservation& observation,
             double steady_now);
  // Feed every real source observation independently of the control timer.
  // Freshness is supplied by the transport's clock/age gate: source and
  // controller clocks may differ. Only the current primitive's ordered
  // checkpoint ledger changes; missed points are latched for step() to brake.
  // This never changes commands, phase, settling evidence or control clocks.
  void observePassThrough(const ManeuverObservation& observation, bool fresh);
  ManeuverCommand step(const ManeuverObservation& observation, double steady_now);
  void cancel(double steady_now);
  void abort(const std::string& reason, double steady_now);
  const ManeuverCommand& command() const { return output_; }
  const ManeuverParameters& parameters() const { return parameters_; }

 private:
  bool active() const;
  bool stationary(const ManeuverObservation& observation) const;
  bool endpointReached(const ManeuverObservation& observation) const;
  double scheduledRate(const ManeuverSegment& segment, double elapsed_s, bool pivot) const;
  double pivotTerminalTarget(const ManeuverObservation& observation, double progress,
                             double position_tolerance, double yaw_tolerance) const;
  void beginSegment(const ManeuverObservation& observation, double steady_now);
  void beginStop(ManeuverStatus outcome, const std::string& reason, double steady_now);
  void limit(double desired_v, double desired_w, double dt);
  void braking(const ManeuverObservation& observation, bool new_observation,
               bool fresh, double now, double dt);
  void measuredBraking(const ManeuverObservation& observation, bool new_observation,
                       bool fresh, double dt);

  ManeuverParameters parameters_;
  ManeuverPlan plan_;
  ManeuverCommand output_;
  ManeuverObservation last_observation_;
  ManeuverStatus stop_outcome_ = ManeuverStatus::Failed;
  double started_at_ = 0., segment_started_at_ = 0., last_step_at_ = 0.;
  double last_observed_at_ = 0., stop_started_at_ = 0.;
  double pivot_angle_ = 0.;  // Unwrapped observed yaw relative to planned start.
  double pass_through_source_stamp_ = 0.;
  bool terminal_capture_attempted_ = false;
  unsigned terminal_capture_settled_ = 0;
  unsigned measured_brake_settled_ = 0;
  unsigned settled_ = 0;
};

}  // namespace gbplanner_tracking
