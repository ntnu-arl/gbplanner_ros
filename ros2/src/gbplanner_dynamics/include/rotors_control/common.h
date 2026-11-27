#pragma once
// ROS-independent types replacing ROS 1/mav_msgs transport only.
// Allocation matrix extracted unchanged from RotorS common.h (Apache-2.0).
#include <Eigen/Dense>
#include <vector>
#include <cassert>
#include <iostream>
#include <cmath>
#include "rotors_control/parameters.h"
namespace mav_msgs {
struct EigenTrajectoryPoint {
  Eigen::Vector3d position_W{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_W{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_W{Eigen::Vector3d::Zero()};
  double yaw{0.},yaw_rate{0.};
  double getYaw() const{return yaw;}
  double getYawRate() const{return yaw_rate;}
};
}
namespace rotors_control {
struct EigenOdometry {
 Eigen::Vector3d position{Eigen::Vector3d::Zero()};
 Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
 Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
 Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};
inline void calculateAllocationMatrix(const RotorConfiguration& rotor_configuration,
                                      Eigen::Matrix4Xd* allocation_matrix) {
  assert(allocation_matrix != nullptr);
  allocation_matrix->resize(4, rotor_configuration.rotors.size());
  unsigned int i = 0;
  for (const Rotor& rotor : rotor_configuration.rotors) {
    // Set first row of allocation matrix.
    (*allocation_matrix)(0, i) = sin(rotor.angle) * rotor.arm_length
                                 * rotor.rotor_force_constant;
    // Set second row of allocation matrix.
    (*allocation_matrix)(1, i) = -cos(rotor.angle) * rotor.arm_length
                                 * rotor.rotor_force_constant;
    // Set third row of allocation matrix.
    (*allocation_matrix)(2, i) = -rotor.direction * rotor.rotor_force_constant
                                 * rotor.rotor_moment_constant;
    // Set forth row of allocation matrix.
    (*allocation_matrix)(3, i) = rotor.rotor_force_constant;
    ++i;
  }
  Eigen::FullPivLU<Eigen::Matrix4Xd> lu(*allocation_matrix);
  // Setting the threshold for when pivots of the rank calculation should be considered nonzero.
  lu.setThreshold(1e-9);
  int rank = lu.rank();
  if (rank < 4) {
    std::cout << "The rank of the allocation matrix is " << lu.rank()
              << ", it should have rank 4, to have a fully controllable system,"
              << " check your configuration." << std::endl;
  }

}

inline void vectorFromSkewMatrix(Eigen::Matrix3d& matrix,Eigen::Vector3d* vector){
 *vector<<matrix(2,1),matrix(0,2),matrix(1,0);
}
}
