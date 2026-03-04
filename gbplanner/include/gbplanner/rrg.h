#ifndef RRG_H_
#define RRG_H_

#include <cmath>
#include <fstream>
#include <iostream>
#include <list>
#include <numeric>
#include <unordered_map>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <kdtree/kdtree.h>
#include <pcl/common/distances.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "adaptive_obb/adaptive_obb.h"
#include "gbplanner/gbplanner_rviz.h"
#include "planner_common/geofence_manager.h"
#include "planner_common/graph.h"
#include "planner_common/graph_base.h"
#include "planner_common/graph_manager.h"
#include "planner_common/params.h"
#include "planner_common/random_sampler.h"
#include "planner_common/trajectory.h"
#include "planner_common/utils.h"
#include "map_manager/map_manager.h"
#include "planner_msgs/PlanningBound.h"
#include "planner_msgs/PlanningMode.h"
#include "planner_msgs/planner_dynamic_global_bound.h"
#include "planner_msgs/planner_srv.h"
#include "planner_semantic_msgs/SemanticPoint.h"
#include "planner_msgs/OpeningDetection.h"
#include "planner_msgs/MultipleOpeningDetections.h"
#include "planner_msgs/planner_opening_approval.h"
#include "planner_msgs/planner_set_planning_mode.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

// #include "opening_detector/opening_detector.hpp"
// Publish all gbplanner rviz topics or not.
#define FULL_PLANNER_VIZ 1

static const double max_difference_waypoint_to_graph = 15.0;

// namespace explorer {

// Keep tracking state of the robot every T seconds.
class RobotStateHistory {
 public:
  RobotStateHistory();
  void addState(StateVec* s);
  bool getNearestState(const StateVec* state, StateVec** s_res);
  bool getNearestStateInRange(const StateVec* state, double range,
                              StateVec** s_res);
  bool getNearestStates(const StateVec* state, double range,
                        std::vector<StateVec*>* s_res);
  void reset();
  std::vector<StateVec*> state_hist_;

 private:
  // Kd-tree for nearest neigbor lookup.
  kdtree* kd_tree_;
};

struct Opening {
  int id;
  geometry_msgs::Pose pose;
  bool active = true;
  std::vector<geometry_msgs::Pose> through_path;
  int num_tries = 0;
  bool exists = true;
};

enum OpeningTraversalMode {
  kNone = 0,
  kGoingTo,
  kPathCheck,
  kPassingThrough
};

enum OpeningTraversalStatus
{
  OK = 0,
  CANT_CONNECT,
  OPENING_DOUBLE_CHECK_FAILED,
  NO_OPENINGS
};

enum OpeningApproval {
  kWaiting = 0,
  kApproved,
  kRejected,
  kReEvaluate
};

class Rrg {
 public:
  enum GraphStatus {
    OK = 0,                // Everything is OK as expected.
    ERR_KDTREE,            // Could not search nearest neigbors from kdtree.
    ERR_NO_FEASIBLE_PATH,  // Could not find any path.
    NO_GAIN,               // No non-zero gain found.
    NOT_OK,                // Any other errors.
    CONSEC_LOW_GAIN        // Local exploration exhausted
  };

  enum LocalPlannerStatus {
    L_OK = 0,                // Everything is OK as expected.
    L_ERR,                   // Any error.
    L_EXHAUSTED,             // Local exploration exhausted
    L_TIME_LIMIT_REACHED,
    L_STUCK
  };

  enum GlobalPlannerStatus {
    G_OK = 0,               // Ok
    G_ERR,
    G_HOMING
  };
 

  Rrg(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  // If the external map manager is to be passed
  Rrg(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      MapManager* map_manager);

  // Initialize the graph to start a new planning session.
  void reset();

  // Clear out old vertices from previous session.
  void clear();

  // Sample points and construct a graph.
  GraphStatus batchGraph();
  GraphStatus buildGraph();
  GraphStatus buildGridGraph(StateVec state, Eigen::Vector3d robot_size,
                             Eigen::Vector3d grid_min, Eigen::Vector3d grid_max,
                             Eigen::Vector3d grid_res, double heading);

  // Connect the new vertex to the graph using collision free edges
  void expandGraph2(std::shared_ptr<GraphManager> graph_manager,
                      std::vector<Vertex>& samples, ExpandGraphReport& rep,
                      bool allow_short_edge=false);
  void expandGraphEdgesBatch(std::shared_ptr<GraphManager> graph_manager,
                        std::vector<Vertex> sampled_vertices);
  void expandGraph(std::shared_ptr<GraphManager> graph_manager,
                   StateVec& new_state, ExpandGraphReport& rep,
                   bool allow_short_edge = false);
  void expandGraph(std::shared_ptr<GraphManager> graph_manager,
                   Vertex& new_vertex, ExpandGraphReport& rep,
                   bool allow_short_edge = false);

  // Add edges only from this vertex.
  void expandGraphEdges(std::shared_ptr<GraphManager> graph_manager,
                        Vertex* new_vertex, ExpandGraphReport& rep);

  // Add egdes without collision checking
  void expandGraphEdgesBlindly(std::shared_ptr<GraphManager> graph_manager,
                               Vertex* new_vertex, double radius,
                               ExpandGraphReport& rep);

  // Compute exploration gain for each vertex.
  void computeExplorationGain(bool only_leaf_vertices = false);
  void computeExplorationGain(bool only_leaf_vertices = false,
                              bool clustering = false);

  MapManager* getMapManager() {
    return map_manager_;
  }

  // Evaluate gains of all vertices and find the best path.
  GraphStatus evaluateGraph();

  // Evaluate path for local navigation
  LocalPlannerStatus evaluateLocalNavigationPath();

  // Search a path to connect two arbitrary states in the whole map.
  // Build a graph and find Dijkstra shortest path.
  bool search(geometry_msgs::Pose source_pose, geometry_msgs::Pose target_pose,
              bool use_current_state,
              std::vector<geometry_msgs::Pose>& path_ret);
  ConnectStatus findPathToConnect(StateVec& source, StateVec& target,
                                  std::shared_ptr<GraphManager> graph_manager,
                                  RandomSamplingParams& params,
                                  int& final_target_id,
                                  std::vector<geometry_msgs::Pose>& path_ret);

  std::vector<geometry_msgs::Pose> runGlobalPlanner(int vertex_id,
                                                    bool not_check_frontier,
                                                    bool ignore_time,
                                                    int &status);
  // In case the global planner was to be retrigered while executing the global
  // path
  std::vector<geometry_msgs::Pose> reRunGlobalPlanner(int &status);
  std::vector<geometry_msgs::Pose> calculateGlobalPath();

  // Remove edges violating geofences
  void cleanViolatedEdgesInGraph(std::shared_ptr<GraphManager> graph_manager);

  // Some utilities
  void addGeofenceAreas(const geometry_msgs::PolygonStamped& polygon_msgs);
  void clearUntraversableZones();
  void setState(StateVec& state);
  void setBoundMode(BoundModeType bmode);
  void setRootMode(bool plan_ahead);
  void setGlobalFrame(std::string frame_id);
  bool loadParams(bool);
  void initializeParams();
  void initializeAttributes();
  void resetMissionTimer() {
    if (!landing_engaged_) rostime_start_ = ros::Time::now();
  }

  void setExplorationAndInspectionBounds(BoundedSpaceParams exp_bounds, BoundedSpaceParams insp_bounds) {
    global_space_params_.min_val = exp_bounds.min_val;
    global_space_params_.max_val = exp_bounds.max_val;
    inspection_bound_.min_val = insp_bounds.min_val;
    inspection_bound_.max_val = insp_bounds.max_val;
    ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Exploration and inspection bounds changed");
  }

  bool modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
                  Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                  Eigen::Vector3d& p1_mod);

  bool improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                       std::vector<geometry_msgs::Pose>& path_mod, bool);

  // Return the best path
  std::vector<geometry_msgs::Pose> getBestPath(std::string tgt_frame,
                                               int& status);
  std::vector<geometry_msgs::Pose> getBestPathSimplified();

  bool homingRequired(std::vector<geometry_msgs::Pose> &homing_path);
  std::vector<geometry_msgs::Pose> searchHomingPath(std::string tgt_frame,
                                                    const StateVec& cur_state);
  std::vector<geometry_msgs::Pose> getHomingPath(std::string tgt_frame);
  std::vector<geometry_msgs::Pose> getGlobalPath(
      geometry_msgs::PoseStamped& waypoint);

  std::vector<geometry_msgs::Pose> getOpeningTraversalPath();
  std::vector<geometry_msgs::Pose> getOpeningTraversalPath(OpeningTraversalMode mode, OpeningTraversalStatus &status);
  void setNextCompartmentCenter(Eigen::Vector3d &center);
  void setNextCompartmentIndex(int ind) {next_compartment_index_ = ind;}

  // Set current position as homing.
  bool setHomingPos();

  void setRootStateForPlanning(const geometry_msgs::Pose& root_pose);

  void setTimeRemaining(double t) { current_battery_time_remaining_ = t; }

  std::vector<geometry_msgs::Pose> searchPathToPassGate();
  bool searchPathThroughCenterPoint(const StateVec& current_state,
                                    const Eigen::Vector3d& center,
                                    const double& heading,
                                    Eigen::Vector3d& robot_size,
                                    std::vector<geometry_msgs::Pose>& path);

  bool isPathCollisionFree(const std::vector<geometry_msgs::Pose>& path,
                           const Eigen::Vector3d& robot_size);

  bool setGlobalBound(planner_msgs::PlanningBound& bound,
                      bool reset_to_default = false);
  bool setGlobalBound(
      planner_msgs::planner_dynamic_global_bound::Request bound);
  void getGlobalBound(planner_msgs::PlanningBound& bound);
  void getGlobalBoundParams(BoundedSpaceParams& global_space) {
    global_space = global_space_params_;
  }

  void setGeofenceManager(std::shared_ptr<GeofenceManager> geofence_manager);

  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params);
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params,
                       const BoundedSpaceParams& local_space_params);

  void setLocalNavGoal(Eigen::Vector3d goal) {
    local_navigation_goal_ = goal;
    local_navigation_goal_set_ = true;
    local_goal_distance_reached_ = std::numeric_limits<double>::max();
    local_goal_progress_fail_iters_ = 0;
  }

  std::vector<geometry_msgs::Pose> getInspectionPath();
  std::vector<geometry_msgs::Pose> getInspectionPathBasic();

  void generateCostMatrix(std::vector<int> nodes, std::vector<std::vector<int>> &cost_matrix, std::map<int, ShortestPathsReport> &path_rep_map);
  double calculateTourCost(const std::vector<int>& tour, const std::vector<std::vector<double>>& costMatrix);
  void twoOptSwap(std::vector<int>& tour, int i, int k);
  std::vector<int> solveTSP(const std::vector<std::vector<double>>& costMatrix);
  std::vector<int> doTSP(std::vector<int> &nodes, std::vector<std::vector<int>> &cost_matrix);
  std::vector<geometry_msgs::Pose> connectTSPOrder(std::vector<int> &tsp_nodes, std::map<int, ShortestPathsReport> &path_rep_map, std::shared_ptr<GraphManager> graph);
  std::vector<geometry_msgs::Pose> connectTSPOrderWithSubvertices(std::vector<std::pair<int, std::vector<int>>> &tsp_nodes, std::map<int, ShortestPathsReport> &path_rep_map, std::shared_ptr<GraphManager> graph);

  void generateGridSamples(std::vector<int> &viewpoint_ids);
  void generateGridSamplesBasic(std::vector<int> &viewpoint_ids);

  bool loadGraph(const std::string& path) {
    global_graph_->loadGraph(path);
    visualization_->visualizeGlobalGraph(global_graph_);
    return true;
  }

  bool saveGraph(const std::string& path) {
    visualization_->visualizeGlobalGraph(global_graph_);
    global_graph_->saveGraph(path);
    return true;
  }

  void setPlannerTriggerMode(PlannerTriggerModeType& trig_mode) {
    planner_trigger_mode_ = trig_mode;
    if (planner_trigger_mode_ == PlannerTriggerModeType::kAuto) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Planner Trigger Mode set to kAuto.");
    } else if (planner_trigger_mode_ == PlannerTriggerModeType::kManual) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Planner Trigger Mode set to kManual.");
    }
  }

  bool openingTraversalOngoing() {
    if(opening_traversal_mode_ == OpeningTraversalMode::kNone) {
      return false;
    }
    else {
      return true;
    }
  }

  bool autoOpeningPathApproval() {
    return planning_params_.auto_opening_path_approval;
  }

  void getBestPitchAngles(StateVec state, std::vector<std::pair<StateVec, VolumetricGain>> &out_states);
  void getBestViewpointAngles(StateVec state, std::vector<std::pair<StateVec, VolumetricGain>> &out_states);

 private:
  bool sampleRandomState(StateVec& state);
  bool sampleVertex(Vertex& vertex);
  bool sampleVertex(RandomSampler& random_sampler, StateVec& root_state,
                    Vertex& vertex);
  double projectSample(Eigen::Vector3d& sample,
                       VoxelStatus& voxel_status);
  double projectSampleEleMap(Eigen::Vector3d& sample,
                       VoxelStatus& voxel_status);
  ProjectedEdgeStatus getProjectedEdgeStatus(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
      std::vector<Eigen::Vector3d>& projected_edge, bool);
  ProjectedEdgeStatus getProjectedEdgeStatusEleMap(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& box_size, bool stop_at_unknown_voxel,
      std::vector<Eigen::Vector3d>& projected_edge, bool);
  // Correct the heading of each vertex in the Dijkstra shortest paths in the
  // local graph to follow the tangent of each segment.
  void correctYaw();
  // Compute volumetric gain by counting each voxel in the sensor frustum
  void computeVolumetricGain(StateVec& state, VolumetricGain& vgain,
                             bool vis_en = false);
  // Compute volumetric gain by casting rays inside the sensor frustum
  void computeVolumetricGainRayModel(StateVec& state, VolumetricGain& vgain,
                                     bool vis_en = false,
                                     bool iterative = false);

  void computeVolumetricGainRayModelNoBound(StateVec& state,
                                            VolumetricGain& vgain);

  void computeInspectionGainRayModel(Vertex* vert);

  void evaluateShortestPaths();

  // Add frontiers from the local graph to the global graph
  void addFrontiers(int best_vertex_id);

  void semanticsCallback(const planner_semantic_msgs::SemanticPoint& semantic);

  std::string world_frame_ = "world";

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformListener* listener_;

  ros::Publisher time_log_pub_;
  ros::Publisher pci_reset_pub_;
  ros::Publisher local_free_map_pub_;
  ros::Publisher path_pub_;
  ros::Publisher free_cloud_pub_;
  ros::Publisher entry_point_pub_;
  ros::Publisher local_target_pub_;

  ros::Subscriber semantics_subscriber_;
  ros::Subscriber stop_srv_subscriber_;
  ros::Subscriber opening_detection_sub_;
  ros::Subscriber query_pt_sub_;
  ros::Subscriber cam_pitch_sub_;
  ros::Subscriber ele_map_sub_;

  ros::ServiceClient pci_homing_;
  ros::ServiceClient landing_srv_client_;
  ros::ServiceServer reset_timer_srv_;
  ros::ServiceServer pass_opening_srv_;
  ros::ServiceServer approve_passing_srv_;
  ros::ServiceServer reset_map_srv_;
  ros::ServiceServer query_srv_;
  ros::ServiceServer remove_geofence_srv_;

  bool resetTimerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool getOpeningPathCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool approvePassingCallback(planner_msgs::planner_opening_approval::Request &req, planner_msgs::planner_opening_approval::Response &res);
  bool resetMapCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool queryCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool removeGeofenceCallback(planner_msgs::planner_set_planning_mode::Request &req, planner_msgs::planner_set_planning_mode::Response &res);

  void stopMsgCallback(const std_msgs::Bool& msg);
  void queryPtCallback(const geometry_msgs::PoseStamped& pose);
  void camPitchCallback(const sensor_msgs::JointState &state);
  void eleMapCallback(const grid_map_msgs::GridMap& msg);

  void openingDetectionCallback(const planner_msgs::MultipleOpeningDetections &detections);

  // Graphs.
  std::shared_ptr<GraphManager> local_graph_;
  std::shared_ptr<GraphManager> projected_graph_;
  ShortestPathsReport local_graph_rep_;  // shortest path to root vertex
  std::shared_ptr<GraphManager> global_graph_;
  ShortestPathsReport global_graph_rep_;  // shortest path to root vertex
  std::vector<std::vector<double>> edge_inclinations_;

  StateVec query_vec_;

  Eigen::Vector3d local_navigation_goal_;
  bool local_navigation_goal_set_ = false;
  double local_goal_distance_reached_ = std::numeric_limits<double>::max();
  int local_goal_progress_fail_iters_ = 0;

  // Add a collision-free path to the graph.
  bool addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                         const std::vector<Vertex*>& vertices);
  bool addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                         const std::vector<geometry_msgs::Pose>& path);
  bool connectStateToGraph(std::shared_ptr<GraphManager> graph,
                           StateVec& cur_state, Vertex*& v_added,
                           double dist_ignore_collision_check);
  double getTimeElapsed();
  double getTimeRemained();
  bool isRemainingTimeSufficient(const double& time_cost, double& time_spare);

  PlannerTriggerModeType planner_trigger_mode_;

  // Current exploring direction.
  double exploring_direction_;
  const double kTimerPeriod = 0.25;
  ros::Timer periodic_timer_;
  void timerCallback(const ros::TimerEvent& event);
  int dir_change_count_ = 0;

  std::queue<StateVec> robot_state_queue_;

  const double kGlobalGraphUpdateTimerPeriod = 0.5;
  const double kGlobalGraphFrontierAdditionTimerPeriod = 1.0;
  const double kGlobalGraphUpdateTimeBudget = 0.1;
  ros::Timer global_graph_update_timer_;
  void expandGlobalGraphTimerCallback(const ros::TimerEvent& event);
  ros::Timer global_graph_frontier_addition_timer_;
  void expandGlobalGraphFrontierAdditionTimerCallback(
      const ros::TimerEvent& event);
  ros::Timer camera_annotation_timer_;
  void cameraAnnotationTimerCallback(const ros::TimerEvent& event);

  const double kFreePointCloudUpdatePeriod = 0.5;
  ros::Timer free_cloud_pub_timer_;
  void freePointCloudtimerCallback(const ros::TimerEvent& event);

  const int backtracking_queue_max_size = 500;
  std::queue<StateVec> robot_backtracking_queue_;
  Vertex* robot_backtracking_prev_;

  // Openings detected so far
  std::map<int, std::shared_ptr<Opening>> detected_openings_;

  // Compare 2 angles within a threshold (positive).
  bool compareAngles(double dir_angle_a, double dir_angle_b, double thres);
  bool comparePathWithDirectionApprioximately(
      const std::vector<geometry_msgs::Pose>& path, double yaw);

  bool reconnectPathBlindly(std::vector<geometry_msgs::Pose>& ref_path,
                            std::vector<geometry_msgs::Pose>& mod_path);

  std::vector<int> performShortestPathsClustering(
      const std::shared_ptr<GraphManager> graph_manager,
      const ShortestPathsReport& graph_rep, std::vector<Vertex*>& vertices,
      double dist_threshold = 1.0, double principle_path_min_length = 1.0,
      bool refinement_enable = true);

  void printShortestPath(int id);

  inline void truncateYaw(double& x) {
    if (x > M_PI)
      x -= 2 * M_PI;
    else if (x < -M_PI)
      x += 2 * M_PI;
  }

  inline void offsetZAxis(StateVec& state, bool down = false) {
    if (robot_params_.type == RobotType::kGroundRobot) {
      if (down)
        state[2] -= random_sampler_.getZOffset();
      else
        state[2] += random_sampler_.getZOffset();
    }
  }

  inline double getDistance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2) {
    Eigen::Vector3d v1(p1.position.x, p1.position.y, p1.position.z);
    Eigen::Vector3d v2(p2.position.x, p2.position.y, p2.position.z);
    return (v2 - v1).norm();
  }

  inline double getDistance(const StateVec &v1, const geometry_msgs::Pose &p2) {
    Eigen::Vector3d v2(p2.position.x, p2.position.y, p2.position.z);
    return (v2 - v1.head(3)).norm();
  }

  void convertStateToPoseMsg(const StateVec& state, geometry_msgs::Pose& pose) {
    tf::Quaternion quat;
    quat.setEuler(0.0, state[4], state[3]);
    tf::Vector3 origin(state[0], state[1], state[2]);
    tf::Pose poseTF(quat, origin);
    tf::poseTFToMsg(poseTF, pose);
  }

  void convertPoseMsgToState(const geometry_msgs::Pose& pose, StateVec& state) {
    state[0] = pose.position.x;
    state[1] = pose.position.y;
    state[2] = pose.position.z;
    
    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;

    auto euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    state[3] = euler[0];
    state[4] = euler[1];
  }

  void convertPointToEigen(const geometry_msgs::Point& point,
                           Eigen::Vector3d& vec) {
    vec(0) = point.x;
    vec(1) = point.y;
    vec(2) = point.z;
  }

  void publishTimings(std::shared_ptr<SampleStatistic> stat) {
    std_msgs::Float32MultiArray time_log;
    time_log.data.push_back(stat->build_graph_time);
    time_log.data.push_back(stat->compute_exp_gain_time);
    time_log.data.push_back(stat->shortest_path_time);
    time_log.data.push_back(stat->evaluate_graph_time);
    time_log_pub_.publish(time_log);
  }

  // Function to perform a **single** convolution pass
  inline Eigen::MatrixXi sphericalConvolution(const Eigen::MatrixXi& A, const Eigen::MatrixXi& kernel) {
      int rows = A.rows(), cols = A.cols();
      int kRows = kernel.rows(), kCols = kernel.cols();
      int kHalfRows = kRows / 2, kHalfCols = kCols / 2;

      Eigen::MatrixXi B = Eigen::MatrixXi::Zero(rows, cols);

      // Perform convolution with cylindrical wrapping
      for (int i = 0; i < rows; i++) {
          for (int j = 0; j < cols; j++) {
              int sum = 0;
              for (int ki = -kHalfRows; ki <= kHalfRows; ki++) {
                  for (int kj = -kHalfCols; kj <= kHalfCols; kj++) {
                      int ai = (i + ki + rows) % rows;
                      int aj = (j + kj + cols) % cols;
                      sum += A(ai, aj) * kernel(ki + kHalfRows, kj + kHalfCols);
                  }
              }
              B(i, j) = sum;
          }
      }
      return B;
  }

  // **Efficiently update convolution matrix B locally**
  inline void updateConvolution(Eigen::MatrixXi& B, const Eigen::MatrixXi& A, const Eigen::MatrixXi& kernel, 
                        const std::vector<std::pair<int, int>>& affectedIndices) {
      int rows = A.rows(), cols = A.cols();
      int kRows = kernel.rows(), kCols = kernel.cols();
      int kHalfRows = kRows / 2, kHalfCols = kCols / 2;

      for (auto [i, j] : affectedIndices) {
          for (int ki = -kHalfRows; ki <= kHalfRows; ki++) {
              for (int kj = -kHalfCols; kj <= kHalfCols; kj++) {
                  int bi = (i - ki + rows) % rows;
                  int bj = (j - kj + cols) % cols;
                  
                  // Recompute only affected elements
                  int newSum = 0;
                  for (int ki2 = -kHalfRows; ki2 <= kHalfRows; ki2++) {
                      for (int kj2 = -kHalfCols; kj2 <= kHalfCols; kj2++) {
                          int ai = (bi + ki2 + rows) % rows;
                          int aj = (bj + kj2 + cols) % cols;
                          newSum += A(ai, aj) * kernel(ki2 + kHalfRows, kj2 + kHalfCols);
                      }
                  }
                  B(bi, bj) = newSum;
              }
          }
      }
  }

  // For visualization.
  Visualization* visualization_;

  // Params required for planning.
  SensorParams sensor_params_;
  SensorParams camera_annotation_params_;
  SensorParams free_frustum_params_;
  RobotParams robot_params_;
  BoundedSpaceParams local_space_params_;
  BoundedSpaceParams global_space_params_;
  std::vector<BoundedSpaceParams> no_gain_zones_;
  bool use_no_gain_space_ = true;
  PlanningParams planning_params_;
  RandomSampler random_sampler_;  // x,y,z,yaw: for exploration purpose
  std::vector<RandomSamplerBase::RandomDistributionType> init_pdf_type_; // = RandomSamplerBase::RandomDistributionType::kUniform; // In case of kNormalUnifrom
  RandomSampler random_sampler_to_search_;  // for searching feasible path
                                            // connecting two points in space
  BoundedSpaceParams local_search_params_;
  RandomSampler random_sampler_adaptive_;  // for adapting the sampling space to
                                           // the surrounding environment
  BoundedSpaceParams local_adaptive_params_;
  Eigen::Vector3d adaptive_orig_min_val_, adaptive_orig_max_val_;
  RobotDynamicsParams robot_dynamics_params_;
  DarpaGateParams darpa_gate_params_;
  // Used to store the default global bounding box that is loaded from the
  // config file
  BoundingBoxType global_bound_;

  BoundedSpaceParams inspection_bound_;

  MapManager* map_manager_;

  std::shared_ptr<GeofenceManager> geofence_manager_;

  grid_map::GridMap ele_map_;

  AdaptiveObb* adaptive_obb_;

  // std::shared_ptr<OpeningDetector> opening_detector_;
  OpeningTraversalMode opening_traversal_mode_;
  int opening_under_execution_ = -1;
  OpeningApproval opening_passing_approved_ = OpeningApproval::kWaiting;
  Eigen::Vector3d next_compartment_;
  int next_compartment_index_ = -1;

  // Mission time tracking
  ros::Time rostime_start_;
  double current_battery_time_remaining_;

  Vertex* root_vertex_;
  Vertex* best_vertex_;

  // Current state of the robot, updated from odometry.
  StateVec current_state_;
  StateVec state_for_planning_;
  double cam_pitch_;

  // Precompute params for planner.
  Eigen::Vector3d robot_box_size_;
  int planning_num_vertices_max_;
  int planning_num_edges_max_;

  // Planner timing statistices
  std::shared_ptr<SampleStatistic> stat_, stat_chrono_;

  //
  int planner_trigger_count_;

  // Temprary variable for timing purpose.
  ros::Time ttime;

  bool odometry_ready;

  // State variables for the planner
  int num_low_gain_iters_;
  bool auto_global_planner_trig_;  // When true, global planner will be triggered

  bool global_exploration_ongoing_;
  int current_global_vertex_id_;
  bool local_exploration_ongoing_;

  bool homing_engaged_ = false;
  bool landing_engaged_ = false;

  //
  bool add_frontiers_to_global_graph_;

  // Save a spare set of states from odometry for homing purpose.
  StateVec last_state_marker_;
  StateVec last_state_marker_global_;

  //
  std::shared_ptr<RobotStateHistory> robot_state_hist_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> obs_pcl_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> feasible_corridor_pcl_;
};

// }  // namespace explorer

#endif
