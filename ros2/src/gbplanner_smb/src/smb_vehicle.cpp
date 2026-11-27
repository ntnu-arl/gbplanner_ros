#include "gbplanner_smb/smb_vehicle.hpp"
#include "mesh_contacts.hpp"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace gbplanner_smb {
namespace {
constexpr short kScene = 1;
constexpr short kVehicle = 2;
btVector3 bullet(const Eigen::Vector3d& v) {
  return {btScalar(v.x()), btScalar(v.y()), btScalar(v.z())};
}
Eigen::Vector3d eigen(const btVector3& v) { return {v.x(), v.y(), v.z()}; }
btQuaternion bullet(const Eigen::Quaterniond& q) {
  return {btScalar(q.x()), btScalar(q.y()), btScalar(q.z()), btScalar(q.w())};
}
Eigen::Quaterniond eigen(const btQuaternion& q) { return {q.w(), q.x(), q.y(), q.z()}; }
double approach(double from, double to, double maximum_change) {
  return from + std::clamp(to - from, -maximum_change, maximum_change);
}
void requirePositive(double value, const char* field) {
  if (!std::isfinite(value) || value <= 0.0) throw std::invalid_argument(field);
}
void requireNonnegative(double value, const char* field) {
  if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument(field);
}
void requirePose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
    throw std::invalid_argument("SMB pose must be finite, with a nonzero quaternion");
}
void clearFeedback(btJointFeedback& feedback) {
  feedback.m_appliedForceBodyA.setZero();
  feedback.m_appliedTorqueBodyA.setZero();
  feedback.m_appliedForceBodyB.setZero();
  feedback.m_appliedTorqueBodyB.setZero();
}
}  // namespace

struct SmbVehicle::Impl {
  explicit Impl(const VehicleParams& requested) : params(requested) {
    requirePositive(params.chassis_mass, "chassis_mass");
    if (!params.chassis_size.allFinite() || (params.chassis_size.array() <= 0.0).any())
      throw std::invalid_argument("chassis_size");
    requirePositive(params.wheel_mass, "wheel_mass");
    requirePositive(params.wheel_radius, "wheel_radius");
    requirePositive(params.wheel_width, "wheel_width");
    requirePositive(params.wheel_y, "wheel_y");
    requirePositive(params.wheel_track, "wheel_track");
    requirePositive(params.max_wheel_torque, "max_wheel_torque");
    requirePositive(params.max_linear_velocity, "max_linear_velocity");
    requirePositive(params.max_angular_velocity, "max_angular_velocity");
    requirePositive(params.max_linear_acceleration, "max_linear_acceleration");
    requirePositive(params.max_angular_acceleration, "max_angular_acceleration");
    requireNonnegative(params.angular_velocity_feedback_gain, "angular_velocity_feedback_gain");
    requirePositive(params.gravity, "gravity");
    requireNonnegative(params.wheel_friction, "wheel_friction");
    requireNonnegative(params.ground_friction, "ground_friction");
    requireNonnegative(params.rolling_friction, "rolling_friction");
    requireNonnegative(params.spinning_friction, "spinning_friction");
    if (!std::isfinite(params.front_wheel_x) || !std::isfinite(params.rear_wheel_x) ||
        !std::isfinite(params.wheel_z) || params.front_wheel_x <= params.rear_wheel_x)
      throw std::invalid_argument("wheel axle coordinates");
    requirePose(params.initial_position, params.initial_orientation);
    if (!params.ground_normal.allFinite() || params.ground_normal.norm() < 1e-9 ||
        !std::isfinite(params.ground_offset)) throw std::invalid_argument("ground plane");
    params.initial_orientation.normalize();
    params.ground_normal.normalize();

    collision = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher = std::make_unique<btCollisionDispatcher>(collision.get());
    // Per-world processing avoids replacing Bullet's process-global contact
    // callback, so other vehicles/worlds retain their own callback lifetime.
    dispatcher->setNearCallback(detail::meshNearCallback);
    broadphase = std::make_unique<btDbvtBroadphase>();
    solver = std::make_unique<btSequentialImpulseConstraintSolver>();
    world = std::make_unique<btDiscreteDynamicsWorld>(dispatcher.get(), broadphase.get(),
                                                     solver.get(), collision.get());
    world->setGravity(btVector3(0, 0, -params.gravity));
    world->getSolverInfo().m_numIterations = 80;
    world->getSolverInfo().m_splitImpulse = true;
    if (params.add_ground) {
      btTransform pose;
      pose.setIdentity();
      auto shape = std::make_unique<btStaticPlaneShape>(bullet(params.ground_normal),
                                                      btScalar(params.ground_offset));
      auto* ground = addBody(std::move(shape), pose, 0.0, Eigen::Vector3d::Zero(), kScene);
      ground->setFriction(params.ground_friction);
    }

    btTransform pose;
    pose.setIdentity();
    pose.setOrigin(bullet(params.initial_position));
    pose.setRotation(bullet(params.initial_orientation));
    auto chassis_shape = std::make_unique<btBoxShape>(bullet(params.chassis_size * 0.5));
    chassis_shape->setMargin(0.001);
    const auto& d = params.chassis_size;
    // Analytic box inertia about the chassis centre; no sensor mass is added.
    const Eigen::Vector3d inertia = params.chassis_mass / 12.0 *
        Eigen::Vector3d(d.y() * d.y() + d.z() * d.z(),
                        d.x() * d.x() + d.z() * d.z(),
                        d.x() * d.x() + d.y() * d.y());
    chassis = addBody(std::move(chassis_shape), pose, params.chassis_mass, inertia, kVehicle);
    chassis->setFriction(params.wheel_friction);
    chassis->setActivationState(DISABLE_DEACTIVATION);
    chassis->setCcdMotionThreshold(0.02);
    chassis->setCcdSweptSphereRadius(0.05);

    for (std::size_t i = 0; i < wheels.size(); ++i) {
      const bool left = (i % 2 == 0);
      const Eigen::Vector3d offset(i < 2 ? params.front_wheel_x : params.rear_wheel_x,
                                    left ? params.wheel_y : -params.wheel_y, params.wheel_z);
      btTransform wheel_pose = pose;
      wheel_pose.setOrigin(pose * bullet(offset));
      // btCylinderShape's native symmetry axis is Y, matching the SMB joint.
      auto shape = std::make_unique<btCylinderShape>(
          btVector3(params.wheel_radius, params.wheel_width * 0.5, params.wheel_radius));
      shape->setMargin(0.001);
      const double axial = params.wheel_mass * params.wheel_radius * params.wheel_radius * 0.5;
      const double radial = params.wheel_mass / 12.0 *
          (3.0 * params.wheel_radius * params.wheel_radius + params.wheel_width * params.wheel_width);
      wheels[i] = addBody(std::move(shape), wheel_pose, params.wheel_mass,
                          Eigen::Vector3d(radial, axial, radial), kVehicle);
      wheels[i]->setFriction(params.wheel_friction);
      wheels[i]->setRollingFriction(params.rolling_friction);
      wheels[i]->setSpinningFriction(params.spinning_friction);
      wheels[i]->setActivationState(DISABLE_DEACTIVATION);
      wheels[i]->setCcdMotionThreshold(0.02);
      wheels[i]->setCcdSweptSphereRadius(0.05);
      // Wheel is body A: Bullet's A-minus-B motor velocity is then the wheel
      // velocity relative to the chassis. Both left and right axes are +Y.
      auto hinge = std::make_unique<btHingeConstraint>(*wheels[i], *chassis,
          btVector3(0, 0, 0), bullet(offset), btVector3(0, 1, 0), btVector3(0, 1, 0), false);
      hinge->setLimit(1.0, -1.0);  // Continuous rotation, no artificial hard stop.
      clearFeedback(joint_feedback[i]);
      // Bullet 3.06 writeBackJoints checks this pointer directly. It only
      // accumulates solved rows into feedback; needsFeedback/getAppliedImpulse
      // are not used, and no constraint rows or solver settings are changed.
      hinge->setJointFeedback(&joint_feedback[i]);
      world->addConstraint(hinge.get(), true);
      hinges.push_back(std::move(hinge));
    }
  }

  ~Impl() {
    for (auto& hinge : hinges) world->removeConstraint(hinge.get());
    for (auto& body : bodies) world->removeRigidBody(body.get());
  }

  btRigidBody* addBody(std::unique_ptr<btCollisionShape> shape, const btTransform& pose,
                       double mass, const Eigen::Vector3d& inertia, short group) {
    // Allocate owning slots before registering the raw body pointer in Bullet.
    // After addRigidBody, moving the unique_ptrs into these slots cannot throw.
    shapes.reserve(shapes.size() + 1);
    motions.reserve(motions.size() + 1);
    bodies.reserve(bodies.size() + 1);
    auto motion = std::make_unique<btDefaultMotionState>(pose);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motion.get(), shape.get(), bullet(inertia));
    auto body = std::make_unique<btRigidBody>(info);
    body->setRestitution(0.0);
    auto* result = body.get();
    // Only vehicle/scene contacts: no wheel/chassis or other self contacts.
    world->addRigidBody(result, group, group == kVehicle ? kScene : kScene | kVehicle);
    shapes.push_back(std::move(shape));
    motions.push_back(std::move(motion));
    bodies.push_back(std::move(body));
    return result;
  }

  VehicleParams params;
  std::unique_ptr<btDefaultCollisionConfiguration> collision;
  std::unique_ptr<btCollisionDispatcher> dispatcher;
  std::unique_ptr<btBroadphaseInterface> broadphase;
  std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
  std::unique_ptr<btDiscreteDynamicsWorld> world;
  // A Bvh shape references its mesh and edge map without owning either.
  // Reverse member destruction releases bodies, shapes, maps, then meshes; the
  // destructor above first unregisters all bodies while the world is alive.
  std::vector<std::unique_ptr<btTriangleMesh>> triangle_meshes;
  std::vector<std::unique_ptr<btTriangleInfoMap>> triangle_info_maps;
  std::vector<std::unique_ptr<btCollisionShape>> shapes;
  std::vector<std::unique_ptr<btDefaultMotionState>> motions;
  std::vector<std::unique_ptr<btRigidBody>> bodies;
  std::vector<std::unique_ptr<btHingeConstraint>> hinges;
  btRigidBody* chassis{nullptr};
  std::array<btRigidBody*, 4> wheels{};
  std::array<double, 4> angles{};
  std::array<double, 4> targets{};
  std::array<btJointFeedback, 4> joint_feedback;
  std::array<btVector3, 4> step_hinge_axes;
  double linear_command{0.0}, angular_command{0.0}, simulated_time{0.0};
  double applied_angular_command{0.0}, angular_velocity_correction{0.0};
  double last_physics_step{0.0};
};

SmbVehicle::SmbVehicle(const VehicleParams& params) : impl_(std::make_unique<Impl>(params)) {}
SmbVehicle::~SmbVehicle() = default;

void SmbVehicle::step(double dt, double linear_velocity, double angular_velocity) {
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.01 ||
      !std::isfinite(linear_velocity) || !std::isfinite(angular_velocity))
    throw std::invalid_argument("SMB step needs finite commands and 0 < dt <= 0.01");
  auto& s = *impl_;
  const auto& p = s.params;
  s.linear_command = approach(s.linear_command,
      std::clamp(linear_velocity, -p.max_linear_velocity, p.max_linear_velocity),
      p.max_linear_acceleration * dt);
  const double previous_angular_command = s.angular_command;
  s.angular_command = approach(s.angular_command,
      std::clamp(angular_velocity, -p.max_angular_velocity, p.max_angular_velocity),
      p.max_angular_acceleration * dt);
  // Four-wheel skid steering can follow joint rates while losing chassis
  // yaw to contact slip. Integrate the actual body-Z rate error into the
  // differential motor target. No chassis velocity or contact setting is
  // assigned. Skip the computation at gain zero to retain the original
  // mapping bit for bit. Clear on stop/reversal rather than carrying an
  // accumulated positive-turn correction into a negative-turn manoeuvre.
  const bool angular_reversal = previous_angular_command != 0.0 &&
      s.angular_command != 0.0 &&
      ((previous_angular_command > 0.0) != (s.angular_command > 0.0));
  if (p.angular_velocity_feedback_gain == 0.0 || s.angular_command == 0.0 || angular_reversal) {
    s.angular_velocity_correction = 0.0;
  } else {
    const btVector3 body_angular_velocity =
        s.chassis->getWorldTransform().getBasis().transpose() * s.chassis->getAngularVelocity();
    const double error = s.angular_command - body_angular_velocity.z();
    const double increment = p.angular_velocity_feedback_gain * error * dt;
    const double current = s.angular_command + s.angular_velocity_correction;
    // Conditional anti-windup: integrate at a limit only when the error
    // drives back into its range. Project the accepted increment to the
    // remaining command headroom, so the integral cannot wind up beyond it.
    if ((current < p.max_angular_velocity || increment < 0.0) &&
        (current > -p.max_angular_velocity || increment > 0.0))
      s.angular_velocity_correction += increment;
    s.angular_velocity_correction = std::clamp(s.angular_velocity_correction,
        -p.max_angular_velocity - s.angular_command,
        p.max_angular_velocity - s.angular_command);
  }
  s.applied_angular_command = std::clamp(s.angular_command + s.angular_velocity_correction,
      -p.max_angular_velocity, p.max_angular_velocity);
  for (std::size_t i = 0; i < s.wheels.size(); ++i) {
    const double side = i % 2 == 0 ? -1.0 : 1.0;
    s.targets[i] = (s.linear_command + side * s.applied_angular_command * p.wheel_track * 0.5) / p.wheel_radius;
    // Bullet accepts a per-step angular impulse, so the torque limit scales
    // with dt. This is the only actuation applied to the five dynamic bodies.
    s.hinges[i]->enableAngularMotor(true, s.targets[i], p.max_wheel_torque * dt);
    clearFeedback(s.joint_feedback[i]);
    s.step_hinge_axes[i] = s.wheels[i]->getWorldTransform().getBasis().getColumn(1);
  }
  s.world->stepSimulation(dt, 0, dt);
  s.last_physics_step = dt;
  const btVector3 axle = s.chassis->getWorldTransform().getBasis().getColumn(1);
  for (std::size_t i = 0; i < s.wheels.size(); ++i)
    s.angles[i] += dt * axle.dot(s.wheels[i]->getAngularVelocity() - s.chassis->getAngularVelocity());
  s.simulated_time += dt;
}

VehicleState SmbVehicle::state() const {
  const auto& s = *impl_;
  VehicleState result;
  const auto& pose = s.chassis->getWorldTransform();
  result.position = eigen(pose.getOrigin());
  result.orientation = eigen(pose.getRotation());
  result.linear_velocity = eigen(s.chassis->getLinearVelocity());
  result.angular_velocity = eigen(s.chassis->getAngularVelocity());
  result.body_linear_velocity = result.orientation.conjugate() * result.linear_velocity;
  result.body_angular_velocity = result.orientation.conjugate() * result.angular_velocity;
  result.last_physics_step = s.last_physics_step;
  result.simulated_time = s.simulated_time;
  result.command_linear = s.linear_command;
  result.command_angular = s.angular_command;
  result.applied_angular_command = s.applied_angular_command;
  result.angular_velocity_correction = s.angular_velocity_correction;
  const btVector3 axle = pose.getBasis().getColumn(1);
  for (std::size_t i = 0; i < s.wheels.size(); ++i) {
    auto& wheel = result.wheels[i];
    const auto& wheel_pose = s.wheels[i]->getWorldTransform();
    wheel.position = eigen(wheel_pose.getOrigin());
    wheel.orientation = eigen(wheel_pose.getRotation());
    wheel.angle = s.angles[i];
    wheel.angular_velocity = axle.dot(s.wheels[i]->getAngularVelocity() - s.chassis->getAngularVelocity());
    wheel.target_angular_velocity = s.targets[i];
    if (s.last_physics_step > 0.0) {
      const double torque = s.step_hinge_axes[i].dot(s.joint_feedback[i].m_appliedTorqueBodyA);
      const double impulse = torque * s.last_physics_step;
      const double limit = s.params.max_wheel_torque * s.last_physics_step;
      wheel.joint_feedback_valid = std::isfinite(torque) && std::isfinite(impulse) && std::isfinite(limit);
      if (wheel.joint_feedback_valid) {
        wheel.joint_axis_torque = torque;
        wheel.joint_axis_impulse = impulse;
        wheel.motor_impulse_limit = limit;
      }
      wheel.contact_diagnostics_valid = true;
    }
  }
  const btMatrix3x3 world_to_body = pose.getBasis().transpose();
  const bool two_friction_directions =
      (s.world->getSolverInfo().m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) != 0;
  for (int i = 0; i < s.dispatcher->getNumManifolds(); ++i) {
    const auto* manifold = s.dispatcher->getManifoldByIndexInternal(i);
    for (int j = 0; j < manifold->getNumContacts(); ++j) {
      // A small separation tolerance also counts persistent resting contacts.
      const auto& contact = manifold->getContactPoint(j);
      if (contact.getDistance() > 0.002) continue;
      for (std::size_t k = 0; k < s.wheels.size(); ++k) {
        const bool wheel_is_a = manifold->getBody0() == s.wheels[k];
        if (!wheel_is_a && manifold->getBody1() != s.wheels[k]) continue;
        auto& wheel = result.wheels[k];
        wheel.contact = true;
        ++wheel.contact_points;
        const auto* other = btRigidBody::upcast(wheel_is_a ? manifold->getBody1() : manifold->getBody0());
        const btVector3& wheel_local = wheel_is_a ? contact.m_localPointA : contact.m_localPointB;
        const btVector3& other_local = wheel_is_a ? contact.m_localPointB : contact.m_localPointA;
        const btVector3 wheel_arm = s.wheels[k]->getWorldTransform().getBasis() * wheel_local;
        btVector3 relative_velocity = s.wheels[k]->getVelocityInLocalPoint(wheel_arm);
        if (other) relative_velocity -= other->getVelocityInLocalPoint(other->getWorldTransform().getBasis() * other_local);
        const btVector3& normal = contact.m_normalWorldOnB;
        const double normal_squared = normal.length2();
        if (!eigen(relative_velocity).allFinite() || !eigen(normal).allFinite() ||
            !std::isfinite(normal_squared) || normal_squared <= 0.0) {
          wheel.contact_diagnostics_valid = false;
          continue;
        }
        const btVector3 slip = relative_velocity - normal * (relative_velocity.dot(normal) / normal_squared);
        const double normal_impulse = contact.getAppliedImpulse();
        btVector3 tangent_impulse(0, 0, 0);
        // Unused friction directions may be uninitialized. Only read a
        // direction if its enabled row actually wrote a nonzero impulse.
        if (contact.m_appliedImpulseLateral1 != 0.0)
          tangent_impulse += contact.m_lateralFrictionDir1 * contact.m_appliedImpulseLateral1;
        if (two_friction_directions && contact.m_appliedImpulseLateral2 != 0.0)
          tangent_impulse += contact.m_lateralFrictionDir2 * contact.m_appliedImpulseLateral2;
        const double side = wheel_is_a ? 1.0 : -1.0;
        const Eigen::Vector3d normal_body = eigen(world_to_body * (normal * (side * normal_impulse)));
        const Eigen::Vector3d tangent_body = eigen(world_to_body * (tangent_impulse * side));
        const Eigen::Vector3d slip_body = eigen(world_to_body * slip);
        const double tangent_magnitude = tangent_impulse.length();
        const double slip_squared = slip_body.squaredNorm();
        if (!std::isfinite(normal_impulse) || !std::isfinite(tangent_magnitude) ||
            !std::isfinite(slip_squared) || !normal_body.allFinite() ||
            !tangent_body.allFinite() || !slip_body.allFinite()) {
          wheel.contact_diagnostics_valid = false;
          continue;
        }
        wheel.normal_impulse_sum += normal_impulse;
        wheel.tangent_impulse_magnitude_sum += tangent_magnitude;
        wheel.normal_impulse_body += normal_body;
        wheel.tangent_impulse_body += tangent_body;
        wheel.contact_slip_body_mean += slip_body;
        wheel.contact_slip_speed_rms += slip_squared;
      }
      if (contact.getDistance() < 0.0 &&
          (manifold->getBody0() == s.chassis || manifold->getBody1() == s.chassis))
        ++result.chassis_contacts;
    }
  }
  for (auto& wheel : result.wheels) {
    if (wheel.contact) ++result.wheel_contacts;
    if (wheel.contact_points) {
      wheel.contact_slip_body_mean /= static_cast<double>(wheel.contact_points);
      wheel.contact_slip_speed_rms = std::sqrt(wheel.contact_slip_speed_rms / wheel.contact_points);
    }
  }
  return result;
}

void SmbVehicle::addBox(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
                         const Eigen::Quaterniond& orientation) {
  requirePose(center, orientation);
  if (!size.allFinite() || (size.array() <= 0.0).any()) throw std::invalid_argument("static box size");
  btTransform pose;
  pose.setIdentity();
  pose.setOrigin(bullet(center));
  pose.setRotation(bullet(orientation.normalized()));
  auto shape = std::make_unique<btBoxShape>(bullet(size * 0.5));
  shape->setMargin(0.001);
  auto* body = impl_->addBody(std::move(shape), pose, 0.0, Eigen::Vector3d::Zero(), kScene);
  body->setFriction(impl_->params.ground_friction);
}

void SmbVehicle::addTriangleMesh(const std::vector<Eigen::Vector3d>& vertices,
                               const std::vector<std::array<int, 3>>& triangles) {
  if (vertices.size() < 3 || triangles.empty())
    throw std::invalid_argument("static mesh needs vertices and triangles");
  // addTriangle stores three vertices per face; Bullet's mesh indices are int.
  if (vertices.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      triangles.size() > static_cast<std::size_t>(std::numeric_limits<int>::max() / 3))
    throw std::invalid_argument("static mesh exceeds Bullet index capacity");
  std::vector<btVector3> converted;
  converted.reserve(vertices.size());
  for (const auto& vertex : vertices) {
    if (!vertex.allFinite() ||
        (vertex.array().abs() > static_cast<double>(std::numeric_limits<btScalar>::max())).any())
      throw std::invalid_argument("static mesh vertex is not finite in Bullet precision");
    converted.push_back(bullet(vertex));
  }
  // Validate the complete surface before changing the world. Converting first
  // also catches faces that collapse when double input becomes btScalar.
  for (const auto& face : triangles) {
    for (int index : face)
      if (index < 0 || static_cast<std::size_t>(index) >= converted.size())
        throw std::invalid_argument("static mesh triangle index out of range");
    if (face[0] == face[1] || face[1] == face[2] || face[2] == face[0])
      throw std::invalid_argument("static mesh triangle has repeated indices");
    const btVector3 normal = (converted[face[1]] - converted[face[0]]).cross(
                             converted[face[2]] - converted[face[0]]);
    const double area_squared = normal.length2();
    if (!std::isfinite(area_squared) || area_squared <= 0.0)
      throw std::invalid_argument("static mesh triangle is degenerate in Bullet precision");
  }
  auto mesh = std::make_unique<btTriangleMesh>();
  for (const auto& face : triangles)
    mesh->addTriangle(converted[face[0]], converted[face[1]], converted[face[2]], false);
  auto shape = std::make_unique<btBvhTriangleMeshShape>(mesh.get(), true);
  shape->setMargin(0.001);
  auto edge_info = std::make_unique<detail::MeshEdgeInfoMap>();
  edge_info->setGeometry(converted, triangles);
  btGenerateInternalEdgeInfo(shape.get(), edge_info.get());
  btTransform pose;
  pose.setIdentity();  // Input vertices already include the world transform.
  impl_->triangle_meshes.reserve(impl_->triangle_meshes.size() + 1);
  impl_->triangle_info_maps.reserve(impl_->triangle_info_maps.size() + 1);
  auto* body = impl_->addBody(std::move(shape), pose, 0.0, Eigen::Vector3d::Zero(), kScene);
  body->setFriction(impl_->params.ground_friction);
  // The reserved slot takes ownership without allocating after registration.
  impl_->triangle_meshes.push_back(std::move(mesh));
  impl_->triangle_info_maps.push_back(std::move(edge_info));
}

RayHit SmbVehicle::raycast(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
  if (!from.allFinite() || !to.allFinite() || (to - from).norm() < 1e-9)
    throw std::invalid_argument("ray endpoints");
  const auto start = bullet(from), end = bullet(to);
  btCollisionWorld::ClosestRayResultCallback callback(start, end);
  callback.m_collisionFilterGroup = kScene;
  callback.m_collisionFilterMask = kScene;
  impl_->world->rayTest(start, end, callback);
  RayHit result;
  result.hit = callback.hasHit();
  if (result.hit) {
    result.fraction = callback.m_closestHitFraction;
    result.point = eigen(callback.m_hitPointWorld);
    result.normal = eigen(callback.m_hitNormalWorld);
  } else {
    result.point = to;
  }
  result.range = (to - from).norm() * result.fraction;
  return result;
}

RayHit SmbVehicle::sensorRaycast(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
  if (!from.allFinite() || !to.allFinite() || (to - from).norm() < 1e-9)
    throw std::invalid_argument("sensor ray endpoints");
  const auto start = bullet(from), end = bullet(to);
  btCollisionWorld::ClosestRayResultCallback callback(start, end);
  // A query in kScene passes both the scene's (kScene|kVehicle) mask and the
  // vehicle's kScene mask. Include BOTH groups so self geometry can occlude.
  callback.m_collisionFilterGroup = kScene;
  callback.m_collisionFilterMask = kScene | kVehicle;
  impl_->world->rayTest(start, end, callback);
  bool self_hit = callback.m_collisionObject == impl_->chassis;
  for (const auto* wheel : impl_->wheels)
    self_hit = self_hit || callback.m_collisionObject == wheel;

  RayHit result;
  result.hit = callback.hasHit() && !self_hit;
  if (result.hit) {
    result.fraction = callback.m_closestHitFraction;
    result.point = eigen(callback.m_hitPointWorld);
    result.normal = eigen(callback.m_hitNormalWorld);
  } else {
    // Self returns are filtered from the environment cloud; they must not
    // become either a floor observation or a free-space ray through the body.
    result.point = to;
  }
  result.range = (to - from).norm() * result.fraction;
  return result;
}

}  // namespace gbplanner_smb
