#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <memory>
#include <vector>

namespace gbplanner_smb {

// Dimensions, masses and controller limits follow the retained SMB xacro/YAML.
// Contact settings and the finite motor torque are Bullet-adapter parameters.
struct VehicleParams {
  double chassis_mass{40.0};
  Eigen::Vector3d chassis_size{0.605, 0.410, 0.220};
  double wheel_mass{5.0};
  double wheel_radius{0.19};
  double wheel_width{0.185};
  double front_wheel_x{0.2159};
  double rear_wheel_x{-0.21};
  double wheel_y{0.3275};
  double wheel_z{-0.05};
  double wheel_track{0.6604};  // Controller separation, distinct from geometry.
  double max_linear_velocity{1.5};
  double max_angular_velocity{1.0};
  double max_linear_acceleration{3.0};
  double max_angular_acceleration{2.0};
  // Optional body-Z angular-rate integral feedback, in 1/s. Zero retains the
  // original open-loop differential wheel mapping exactly. The correction
  // acts through wheel motors and the resulting command obeys the yaw limit.
  double angular_velocity_feedback_gain{0.0};
  double max_wheel_torque{20.0};  // Adapter assumption; no upstream effort limit.
  double wheel_friction{1.0};
  double ground_friction{1.0};
  double rolling_friction{0.001};
  double spinning_friction{0.001};
  double gravity{9.81};
  Eigen::Vector3d initial_position{-2.0, 0.0, 0.26};
  Eigen::Quaterniond initial_orientation{Eigen::Quaterniond::Identity()};
  bool add_ground{true};
  Eigen::Vector3d ground_normal{0.0, 0.0, 1.0};
  // Plane equation: ground_normal.normalized().dot(point) = ground_offset.
  double ground_offset{0.0};
};

struct WheelState {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  // Positive angular velocity is about chassis +Y, hence forward rolling.
  double angle{0.0};
  double angular_velocity{0.0};
  double target_angular_velocity{0.0};
  bool contact{false};
  // Last solver step's total joint feedback on the wheel (body A), projected
  // onto its +Y hinge axis at the start of that step. This includes constraint
  // reactions; it is not a separately observed motor torque/impulse.
  bool joint_feedback_valid{false};
  double joint_axis_torque{0.0};       // N m.
  double joint_axis_impulse{0.0};      // N m s = torque * last physics step.
  double motor_impulse_limit{0.0};     // Configured limit, not measured effort.
  // Contacts use the existing distance <= 0.002 m criterion. Impulse vectors
  // sum impulse ON this wheel in the current chassis body frame; scalar
  // magnitudes sum per point before cancellation. Tangential impulse includes
  // the enabled sliding-friction rows, excluding rolling/spinning torque.
  bool contact_diagnostics_valid{false};
  std::size_t contact_points{0};
  double normal_impulse_sum{0.0};
  double tangent_impulse_magnitude_sum{0.0};
  Eigen::Vector3d normal_impulse_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tangent_impulse_body{Eigen::Vector3d::Zero()};
  // Post-step wheel-minus-scene point velocity, with its normal component
  // removed, at the manifold's local contact anchors. Unweighted mean vector
  // and RMS speed over the same contact points; both zero without contacts.
  Eigen::Vector3d contact_slip_body_mean{Eigen::Vector3d::Zero()};
  double contact_slip_speed_rms{0.0};
};

struct VehicleState {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  // Both velocities are expressed in the world frame.
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  std::array<WheelState, 4> wheels{};  // LF, RF, LH, RH.
  std::size_t wheel_contacts{0};  // Number of wheels touching static geometry.
  std::size_t chassis_contacts{0};  // Number of penetrating contact points.
  double simulated_time{0.0};
  double command_linear{0.0};  // Velocity/acceleration-limited motor command.
  double command_angular{0.0};
  // Last motor setpoint expressed as a differential-drive yaw rate, rad/s.
  // command_angular remains the rate/acceleration-limited external command;
  // their difference is the bounded wheel-motor integral correction.
  double applied_angular_command{0.0};
  double angular_velocity_correction{0.0};
  Eigen::Vector3d body_linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_angular_velocity{Eigen::Vector3d::Zero()};
  double last_physics_step{0.0};  // Seconds; zero before the first solver step.
};

struct RayHit {
  bool hit{false};
  double fraction{1.0};
  double range{0.0};
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
};

// A chassis, four solid cylinder wheels, four revolute motor constraints,
// gravity and Coulomb contacts. Commands only drive joint motors: the adapter
// never assigns a chassis pose or velocity after initial construction.
class SmbVehicle {
 public:
  explicit SmbVehicle(const VehicleParams& params = VehicleParams{});
  ~SmbVehicle();
  SmbVehicle(const SmbVehicle&) = delete;
  SmbVehicle& operator=(const SmbVehicle&) = delete;

  // One exact physics step, 0 < dt <= 0.01 s. Caller handles command timeout.
  void step(double dt, double linear_velocity, double angular_velocity);
  VehicleState state() const;
  void addBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
              const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity());
  // Copy a finite triangle surface in world coordinates into the static
  // scene. No implicit floor or thickness is added. Triangles must have valid
  // distinct indices and remain nondegenerate in Bullet's scalar precision.
  // The mesh is owned by this vehicle and cannot be given a dynamic mass.
  void addTriangleMesh(const std::vector<Eigen::Vector3d>& vertices,
                       const std::vector<std::array<int, 3>>& triangles);
  // Static scene only: chassis and wheels are excluded from sensor returns.
  RayHit raycast(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
  // A sensor beam stops at the nearest scene OR vehicle surface. A vehicle
  // hit is self-occlusion and yields no environment return (hit=false), rather
  // than allowing the beam to continue through the chassis or a wheel.
  RayHit sensorRaycast(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gbplanner_smb
