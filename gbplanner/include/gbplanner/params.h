
#ifndef PARAMS_H_
#define PARAMS_H_

#include <stdio.h>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

// Macros for timing, private use only
typedef ros::Time TIMER;
#define START_TIMER(x) (x = ros::Time::now())
#define GET_ELAPSED_TIME(x) (float)((ros::Time::now() - x).toSec())

// Macros for output parameter loading info
#define ROSPARAM_INFO(msg) std::cout << msg << std::endl

#define ROSPARAM_ERROR(param_name)                                            \
  std::cout << "\033[31m"                                                     \
            << "[ERROR][File: " << __FILE__ << "] [Line: " << __LINE__ << "]" \
            << "\nParam is not set: " << param_name << "\033[0m\n"            \
            << std::endl

#define ROSPARAM_WARN(param_name, default_val)                               \
  std::cout << "\033[33m"                                                    \
            << "[WARN][File: " << __FILE__ << "] [Line: " << __LINE__ << "]" \
            << "\nParam is not set: " << param_name                          \
            << ". Default value is: " << default_val << "\033[0m\n"          \
            << std::endl

// Macros for output planning related info
#define PLANNER_INFO(msg) std::cout << msg << std::endl

#define PLANNER_ERROR(msg)                                                    \
  std::cout << "\033[31m"                                                     \
            << "[ERROR][File: " << __FILE__ << "] [Line: " << __LINE__ << "]" \
            << "\n[Planner-Error]: " << msg << "\033[0m\n"                    \
            << std::endl

#define PLANNER_WARN(msg)                                                    \
  std::cout << "\033[33m"                                                    \
            << "[WARN][File: " << __FILE__ << "] [Line: " << __LINE__ << "]" \
            << "\n[Planner-Warn] is not set: " << param_name                 \
            << ". Default value is: " << msg << "\033[0m\n"                  \
            << std::endl

namespace explorer {

typedef Eigen::Vector4d StateVec; // x,y,z,yaw

/* Coodinate notations:
 * W: Global coordinate (World)
 * S: Sensor coordinate
 * B: Robot's body center coordinate
 */
enum SensorType { kCamera = 0, kLidar = 1 };
struct SensorParamsBase {
  SensorType type;
  double max_range;             // Maximum range for volumetric gain calc.
  Eigen::Vector3d center_offset;// Wrt the body center.
  Eigen::Vector3d rotations;    // Wrt the body in ZYX order (body to sensor).
  Eigen::Vector2d fov;          // FoV angles in [horizontal, vertical].
  Eigen::Vector2d resolution;   // Resolution angles in [H x V].
  std::string frame_id;         // Unique frame id for the sensor
  bool loadParams(std::string ns);

  // Check if a position is inside the sensor's FOV at a state (both in W).
  bool isInsideFOV(StateVec &state, Eigen::Vector3d &pos);
  // Get all endpoints from ray casting models in W.
  void getFrustumEndpoints(StateVec &state, std::vector<Eigen::Vector3d> &ep);
  // Get all edges in W.
  void getFrustumEdges(StateVec &state, std::vector<Eigen::Vector3d> &edges);
  // Check if this is potential frontier.
  bool isFrontier(double num_unknown_voxels_normalized);
  // Convert a set of points from B to S.
  void convertBodyToSensor(pcl::PointCloud<pcl::PointXYZ>::Ptr ep,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr ep_s);

 private:
  // Fixed rotation matrices computed at the initilization step.
  Eigen::Matrix3d rot_B2S;
  Eigen::Matrix3d rot_S2B;

  // These are to support volumetric gain calc with the camera model.
  // Model is approximated as a frustum, precomputed at the initialization step.
  // TopLeft, TopRight, BottomRight, BottomLeft.
  Eigen::Matrix<double, 3, 4> edge_points;    // Sensor coordinate, normalized.
  Eigen::Matrix<double, 3, 4> edge_points_B;  // Body coordinate, normalized.
  // Place all 4 {top, right, bottom, left} vector into a matrix.
  Eigen::Matrix<double, 3, 4> normal_vectors;
  std::vector<Eigen::Vector3d> frustum_endpoints;    // Sensor coordinate.
  std::vector<Eigen::Vector3d> frustum_endpoints_B;  // Body coordinate.

  double num_voxels_full_fov;
  double frontier_percentage_threshold;
};

struct SensorParams {
  std::vector<std::string> sensor_list;
  std::unordered_map<std::string, SensorParamsBase> sensor;
  bool loadParams(std::string ns);
};

/* Different collision checking options for safety purposes.
 * This could be use in some service calls, so set value
 * explicitly to avoid ambiguity.
 */
enum BoundModeType {
  kExtendedBound = 0,  // Use full extension to actual size. (default)
  kRelaxedBound = 1,   // Use relaxed extension.
  kMinBound = 2,       // Use minimum bound allowed.
  kExactBound = 3,     // Use its exact size.
  kNoBound = 4,        // Consider a robot as a point.
};

/* Two types of robot: aerial and legged.
 * However, in this code, only aerial robot is supported.
 */
enum RobotType {kAerialRobot = 0, kLeggedRobot};

/* Settings related to the dimensions of the robot.
 * They are mainly used for collision checking, model the robot as a cuboid.
 */
struct RobotParams {
  RobotType type;
  // Actual size of the robot: length(x) x width(y) x height(z).
  Eigen::Vector3d size;
  // Minimum extension required for collision checking.
  Eigen::Vector3d size_extension_min;
  /* Desired extension for collision checking for safety purpose.
   * Must be at least larger than min_size_extension.
   */
  Eigen::Vector3d size_extension;
  /* Offset from the cuboid center to odometry center.
   * Cuboid center = state + center_offset;
   */
  Eigen::Vector3d center_offset;
  /* If cannot find any solution with maximum bound,
   *  relax (reduce) the bound then search again
   * relax_ratio * size_extension_min + (1-relax_ratio) * size_extension
   */
  double relax_ratio;
  // Bound mode for collision checking.
  BoundModeType bound_mode;
  // Extension for safety improvement step.
  Eigen::Vector3d safety_extension;

  // Utilities.
  void setBoundMode(BoundModeType bmode);
  // Compute the planning size according to the bound mode setting.
  void getPlanningSize(Eigen::Vector3d &psize);

  bool loadParams(std::string ns);
};

enum BoundedSpaceType { kCuboid = 0, kSphere };
/* Bounded exploration space settings.
 */
struct BoundedSpaceParams {
  BoundedSpaceType type;
  Eigen::Vector3d min_val;       // [x,y,z]
  Eigen::Vector3d max_val;
  Eigen::Vector3d min_extension; // (min_val + min_extension): for exp gain.
  Eigen::Vector3d max_extension; // (max_val + max_extension): for exp gain.
  Eigen::Vector3d rotations;     // [Y, P, R] wrt W->B coordinate.
  double radius;                 // for Sphere: space for vertex sampling.
  double radius_extension;       // (radius + radius_extension): for exp gain.

  bool loadParams(std::string ns);
  void setCenter(StateVec &state, bool use_extension);
  bool isInsideSpace(Eigen::Vector3d &pos);

 private:
  Eigen::Vector3d root_pos;
  Eigen::Matrix3d rot_B2W;
  Eigen::Vector3d min_val_total;
  Eigen::Vector3d max_val_total;
  double radius_total;
};

/* Different exploration modes depending on the environment.
*/
enum PlanningModeType {
  kBasicExploration = 0,     // Bare-bone functionalities (default).
  kNarrowEnvExploration = 1, // Exploration in narrow environment.
  kVerticalExploration = 2   // Exploration in multi-level settings.
};

/* Search mode using random graph or random tree
*/
enum RRModeType {
  kGraph = 0,  // Graph based search (default).
  kTree        // Tree based search,
};

/* Parameters for the planner
*/
struct PlanningParams {
  PlanningModeType type;
  RRModeType rr_mode;
  std::vector<std::string> exp_sensor_list;
  double v_max;
  double v_homing_max;
  double yaw_rate_max;
  // Allow to modify the yaw to be tangent with the path direction.
  bool yaw_tangent_correction;
  bool use_ray_model_for_volumetric_gain;
  double free_voxel_gain;
  double occupied_voxel_gain;
  double unknown_voxel_gain;
  double edge_length_min;
  double edge_length_max;
  double edge_overshoot;
  double num_vertices_max;
  double num_edges_max;
  double num_loops_cutoff;
  double num_loops_max;
  double nearest_range;
  double nearest_range_min;
  double nearest_range_max;
  bool use_current_state;
  double path_length_penalty;
  double path_direction_penalty;
  double traverse_length_max;
  double traverse_time_max;
  double augment_free_voxels_time;
  bool augment_free_frustum_en;
  bool adjust_local_sampling_direction;
  bool z_sample_from_ground;
  bool free_frustum_before_planning;
  bool auto_homing_enable;
  double time_budget_limit;
  bool geofence_checking_enable;
  bool homing_backward;
  bool planning_backward;
  bool safety_aware_enable;
  bool path_safety_enhance_enable;
  bool freespace_cloud_enable;
  std::string global_frame_id;

  /* Voxel size to compute exploration gain. Equal to map voxel size or bigger
   * to save computation.
   * This only works with the deprecated computeVolumetricGain function.
   */
  double exp_gain_voxel_size;

  bool loadParams(std::string ns);
  void setPlanningMode(PlanningModeType pmode);
};

/* Use the constant velocity model to estimate the time to execute any path.
*/
struct RobotDynamicsParams {
  double v_max;
  double v_homing_max;
  double yaw_rate_max;
  bool loadParams(std::string ns);
};

}  // namespace explorer

#endif