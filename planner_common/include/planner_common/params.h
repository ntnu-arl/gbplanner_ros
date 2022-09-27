
#ifndef PARAMS_H_
#define PARAMS_H_

#include <stdio.h>

#include <iostream>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

typedef ros::Time TIMER;
#define START_TIMER(x) (x = ros::Time::now())
#define GET_ELAPSED_TIME(x) (float)((ros::Time::now() - x).toSec())

enum Verbosity { SILENT = 0, PLANNER_STATUS = 1, ERROR = 2, WARN = 3, INFO = 4, DEBUG = 5 };

#define global_verbosity Verbosity::ERROR
#define param_verbosity Verbosity::SILENT

#define ROSPARAM_ERROR(param_name)                                         \
  ({                                                                       \
    if (param_verbosity >= Verbosity::ERROR) {                     \
      std::cout << "\033[31m"                                              \
                << "[ERROR][File: " << __FILE__ << "] [Line: " << __LINE__ \
                << "]"                                                     \
                << "\nParam is not set: " << param_name << "\033[0m\n"     \
                << std::endl;                                              \
    }                                                                      \
  })

#define ROSPARAM_WARN(param_name, default_val)                            \
  ({                                                                      \
    if (param_verbosity >= Verbosity::WARN) {                  \
      std::cout << "\033[33m"                                             \
                << "[WARN][File: " << __FILE__ << "] [Line: " << __LINE__ \
                << "]"                                                    \
                << "\nParam is not set: " << param_name                   \
                << ". Default value is: " << default_val << "\033[0m\n"   \
                << std::endl;                                             \
    }                                                                     \
  })

#define ROSPARAM_INFO(msg)                                                \
  ({                                                                      \
    if (param_verbosity >= Verbosity::INFO) {                     \
      std::cout << "\033[32m"                                             \
                << "[INFO][File: " << __FILE__ << "] [Line: " << __LINE__ \
                << "]"                                                    \
                << "\n"                                                   \
                << msg << "\033[0m\n"                                     \
                << std::endl;                                             \
    }                                                                     \
  })

// #define ROSPARAM_ERROR(param_name)
//   std::cout << "\033[31m"
//             << "[ERROR][File: " << __FILE__ << "] [Line: " << __LINE__ << "]"
//             << "\nParam is not set: " << param_name << "\033[0m\n"
//             << std::endl

// #define ROSPARAM_WARN(param_name, default_val)
//   std::cout << "\033[33m"
//             << "[WARN][File: " << __FILE__ << "] [Line: " << __LINE__ << "]"
//             << "\nParam is not set: " << param_name
//             << ". Default value is: " << default_val << "\033[0m\n"
//             << std::endl

//
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

// State of the robot used for planning (x,y,z,yaw)
typedef Eigen::Vector4d StateVec;

enum ProjectedEdgeStatus {
  kAdmissible = 0,
  kSteep,
  kOccipied,
  kUnknown,
  kHanging
};

enum SensorType { kCamera = 0, kLidar = 1 };
enum CameraType { kFixed = 0, kRotating, kZoom, kRotatingZoom };

enum struct PlannerTriggerModeType {
  kManual = 0,  // Manually trigger the control interface each time.
  kAuto = 1     // Automatic exploration.
};

struct SensorParamsBase {
  SensorType type;
  CameraType camera_type;
  double min_range;  // Minimum range for map annotation (used for zoom camera
                     // only).
  double max_range;  // Maximum range for volumetric gain.
  Eigen::Vector3d center_offset;  // Offset from the body center (odometry).
  Eigen::Vector3d rotations;      // Body to sensor; [Y, P, R] (rad).
  Eigen::Vector2d fov;            // [Horizontal, Vertical] angles (rad).
  Eigen::Vector2d resolution;  // Resolution in rad [H x V] for volumetric gain.
  std::string frame_id;        // Frame id of the sensor
  std::string callback_topic;  // Topic on which sensor data is being published
  std::string focal_lenght_topic;  // Topic on which Camera's focal lenght is
                                   // published (Only for zoom_camera).
  int height;                      // Number of rays in vertical direction
  int width;                       // Number of rays in horizontal direction
  int heightRemoval;  // Number of pixel which are not seen on each side in
                      // vertical direction.
  int widthRemoval;   // Number of pixel which are not seen on each side in
                      // horizontal direction.
  bool loadParams(std::string ns);

  // Check if this state is inside sensor's FOV.
  // pos in (W).
  bool isInsideFOV(StateVec& state, Eigen::Vector3d& pos);
  // Get all endpoints from ray casting models in (W).
  void getFrustumEndpoints(StateVec& state, std::vector<Eigen::Vector3d>& ep);
  void getFrustumEndpoints(StateVec& state, std::vector<Eigen::Vector3d>& ep,
                           float darkness_range);

  void updateFrustumEndpoints();
  // Get all edges in (W).
  void getFrustumEdges(StateVec& state, std::vector<Eigen::Vector3d>& edges);
  // Check if this is potential frontier given sensor FOV.
  // Number of unknow voxels perceived, scaled to map resolution (per meter).
  bool isFrontier(double num_unknown_voxels_normalized);
  // Convert points from body frame of robot to frame of this sensor
  void convertBodyToSensor(pcl::PointCloud<pcl::PointXYZ>::Ptr ep,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr ep_s);

 private:
  // Rotation matrix
  Eigen::Matrix3d rot_B2S;
  Eigen::Matrix3d rot_S2B;

  // These are to support camera model, approximate as a pyramid.
  // TopLeft, TopRight, BottomRight, BottomLeft.
  Eigen::Matrix<double, 3, 4> edge_points;    // Sensor coordinate, normalized.
  Eigen::Matrix<double, 3, 4> edge_points_B;  // Body coordinate, normalized.
  // These are to support camera model.
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

// Set the size of the robot with different protection level based on robot's
// configuration. This could be use in some service calls, so set value
// explicitly to avoid ambiguity.
enum BoundModeType {
  kExtendedBound = 0,  // Use extension to actual size. (default)
  kRelaxedBound = 1,   // Use relaxed extension.
  kMinBound = 2,       // Use minimum bound allowed.
  kExactBound = 3,     // Use its exact size.
  kNoBound = 4,        // Consider a robot as a point.
};
// Represent the robot as a cuboid.
enum RobotType { kAerialRobot = 0, kGroundRobot };
struct RobotParams {
  // Set the robot type here.
  RobotType type;
  // Actual size of the robot: length(x) x width(y) x height(z) in meters.
  Eigen::Vector3d size;
  // Minimum extension required for the planner.
  Eigen::Vector3d size_extension_min;
  // Recommend this extension to be used in the planner.
  // size_extension must be at least larger than min_size_extension.
  Eigen::Vector3d size_extension;
  // Offset from the cuboid center to odometry center.
  // Cuboid center = state + center_offset;
  Eigen::Vector3d center_offset;
  // Ratio to compute relaxed extension [0,1].
  //  relax_ratio * size_extension_min + (1-relax_ratio) * size_extension
  double relax_ratio;
  // Bound mode for planning.
  BoundModeType bound_mode;
  // Safety extension
  Eigen::Vector3d safety_extension;

  // Utilities.
  void setBoundMode(BoundModeType bmode);
  // Compute the planning size according to the bound mode setting.
  void getPlanningSize(Eigen::Vector3d& psize);

  bool loadParams(std::string ns);
};

enum BoundedSpaceType { kCuboid = 0, kSphere };
struct BoundedSpaceParams {
  BoundedSpaceType type;
  Eigen::Vector3d min_val;        // [x,y,z] (m): vertex sampling space.
  Eigen::Vector3d max_val;        // [x,y,z] (m)
  Eigen::Vector3d min_extension;  // (min + min_extension): exploration gain.
  Eigen::Vector3d max_extension;  // (max + max_extension): exploration gain.
  Eigen::Vector3d rotations;      // [Y, P, R] wrt W->B coordinate.
  double radius;                  // for Sphere: space for vertex sampling.
  double radius_extension;        // (radius + radius_extension): for
                                  // exploration gain.
  bool loadParams(std::string ns);

  void setCenter(StateVec& state, bool use_extension);
  void setCenter(Eigen::Vector3d& root, bool use_extension);
  Eigen::Vector3d getCenter() { return root_pos; }
  Eigen::Matrix3d getRotationMatrix() { return rot_B2W; }
  void setBound(Eigen::Vector3d& min_val_in, Eigen::Vector3d& max_val_in);
  void setRotation(Eigen::Vector3d& rotations_in);
  bool isInsideSpace(Eigen::Vector3d& pos);

 private:
  Eigen::Vector3d root_pos;
  Eigen::Matrix3d rot_B2W;
  Eigen::Vector3d min_val_total;
  Eigen::Vector3d max_val_total;
  double radius_total;
};

// Operation modes depending on the environment.
enum PlanningModeType {
  kBasicExploration = 0,      // Bare-bone functionalities.
  kNarrowEnvExploration = 1,  // Exploration in narrow environment.
  kAdaptiveExploration = 2    // Exploration adapting to the environment
};

enum RRModeType {
  kGraph = 0,  // Graph based search (default).
  kTree        // Tree based search,
};

struct PlanningParams {
  // Common
  std::string global_frame_id;
  bool freespace_cloud_enable;
  // Robot dynamics
  double v_max;
  double v_homing_max;
  double yaw_rate_max;
  bool yaw_tangent_correction;
  // Graph building
  PlanningModeType type;
  RRModeType rr_mode;
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
  bool geofence_checking_enable;
  // Ground robot specific
  double max_ground_height;
  double robot_height;
  double max_inclination;
  bool interpolate_projection_distance;
  // Clearing free space before planning (depricated)
  double augment_free_voxels_time;
  bool augment_free_frustum_en;
  bool free_frustum_before_planning;
  // Exploration gain calculation
  std::vector<std::string> exp_sensor_list;
  std::vector<std::string> no_gain_zones_list;
  double exp_gain_voxel_size;
  bool use_ray_model_for_volumetric_gain;
  double free_voxel_gain;
  double occupied_voxel_gain;
  double unknown_voxel_gain;
  double path_length_penalty;
  double path_direction_penalty;
  double hanging_vertex_penalty;
  bool leafs_only_for_volumetric_gain;
  bool cluster_vertices_for_gain;
  double clustering_radius;
  double ray_cast_step_size_multiplier;
  bool nonuniform_ray_cast;
  // Best path generation and improvement for safety
  double traverse_length_max;
  double traverse_time_max;
  bool planning_backward;
  bool path_safety_enhance_enable;
  double path_interpolation_distance;
  // Global planner
  double relaxed_corridor_multiplier;
  bool auto_global_planner_enable;
  bool go_home_if_fully_explored;
  bool auto_homing_enable;
  bool homing_backward;
  double time_budget_limit;
  bool auto_landing_enable;
  double time_budget_before_landing;
  double max_negative_inclination;

  bool loadParams(std::string ns);
  void setPlanningMode(PlanningModeType pmode);
};

struct RobotDynamicsParams {
  double v_max;
  double v_homing_max;
  double yaw_rate_max;
  bool loadParams(std::string ns);
};

struct DarpaGateParams {
  DarpaGateParams() {
    enable = false;
    load_from_darpa_frame = true;
    darpa_frame_offset << 0.0, 0.0, 0.0;
    world_frame_id = "world";
    gate_center_frame_id = "darpa";
    center << 4.0, 0.0, 1.0;
    heading = 0.0;
    center_search_radius = 0.5;
    center_search_step = 0.05;
    line_search_length = 5.0;
    line_search_range = M_PI / 18.0;  // 10 deg
    line_search_step = M_PI / 180.0;  // 1 deg
  }

  bool enable;
  double line_length;
  bool load_from_darpa_frame;
  Eigen::Vector3d darpa_frame_offset;
  std::string world_frame_id;
  std::string gate_center_frame_id;
  Eigen::Vector3d center;
  double heading;
  double center_search_radius;
  double center_search_step;
  double line_search_length;
  double line_search_range;
  double line_search_step;

  bool loadParams(std::string ns);
};

#endif
