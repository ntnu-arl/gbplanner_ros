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
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "adaptive_obb/adaptive_obb.h"
#include "gbplanner/gbplanner_rviz.h"
#include "planner_common/geofence_manager.h"
#include "planner_common/graph.h"
#include "planner_common/graph_base.h"
#include "planner_common/graph_manager.h"
#include "planner_common/map_manager.h"
#include "planner_common/map_manager_voxblox_impl.h"
#include "planner_common/params.h"
#include "planner_common/random_sampler.h"
#include "planner_common/trajectory.h"
#include "planner_msgs/PlanningBound.h"
#include "planner_msgs/PlanningMode.h"
#include "planner_msgs/planner_dynamic_global_bound.h"
#include "planner_msgs/planner_srv.h"
#include "planner_semantic_msgs/SemanticPoint.h"

// Publish all gbplanner rviz topics or not.
#define FULL_PLANNER_VIZ 1

static const double max_difference_waypoint_to_graph = 15.0;

namespace explorer {

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

class Rrg {
 public:
  enum GraphStatus {
    OK = 0,                // Everything is OK as expected.
    ERR_KDTREE,            // Could not search nearest neigbors from kdtree.
    ERR_NO_FEASIBLE_PATH,  // Could not find any path.
    NO_GAIN,               // No non-zero gain found.
    NOT_OK,                // Any other errors.
  };

  Rrg(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  // If the external map manager is to be passed
  Rrg(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
      MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
          map_manager);

  // Initialize the graph to start a new planning session.
  void reset();

  // Clear out old vertices from previous session.
  void clear();

  // Sample points and construct a graph.
  GraphStatus buildGraph();
  GraphStatus buildGridGraph(StateVec state, Eigen::Vector3d robot_size,
                             Eigen::Vector3d grid_min, Eigen::Vector3d grid_max,
                             Eigen::Vector3d grid_res, double heading);

  // Connect the new vertex to the graph using collision free edges
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

  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
  getMapManager() {
    return map_manager_;
  }

  // Evaluate gains of all vertices and find the best path.
  GraphStatus evaluateGraph();

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
                                                    bool ignore_time);
  // In case the global planner was to be retrigered while executing the global
  // path
  std::vector<geometry_msgs::Pose> reRunGlobalPlanner();
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

  bool modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
                  Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                  Eigen::Vector3d& p1_mod);

  bool improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                       std::vector<geometry_msgs::Pose>& path_mod, bool);

  // Return the best path
  std::vector<geometry_msgs::Pose> getBestPath(std::string tgt_frame,
                                               int& status);

  std::vector<geometry_msgs::Pose> searchHomingPath(std::string tgt_frame,
                                                    const StateVec& cur_state);
  std::vector<geometry_msgs::Pose> getHomingPath(std::string tgt_frame);
  std::vector<geometry_msgs::Pose> getGlobalPath(
      geometry_msgs::PoseStamped& waypoint);

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

  void setGeofenceManager(std::shared_ptr<GeofenceManager> geofence_manager);

  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params);
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params,
                       const BoundedSpaceParams& local_space_params);

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

 private:
  bool sampleRandomState(StateVec& state);
  bool sampleVertex(Vertex& vertex);
  bool sampleVertex(RandomSampler& random_sampler, StateVec& root_state,
                    Vertex& vertex);
  double projectSample(Eigen::Vector3d& sample,
                       MapManager::VoxelStatus& voxel_status);
  ProjectedEdgeStatus getProjectedEdgeStatus(
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

  void evaluateShortestPaths();

  // Add frontiers from the local graph to the global graph
  void addFrontiers(int best_vertex_id);

  void semanticsCallback(const planner_semantic_msgs::SemanticPoint& semantic);

  std::string world_frame_ = "world";

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformListener* listener_;

  ros::Publisher free_cloud_pub_;
  ros::Publisher time_log_pub_;
  ros::Publisher pci_reset_pub_;

  ros::Subscriber semantics_subscriber_;
  ros::Subscriber stop_srv_subscriber_;

  ros::ServiceClient pci_homing_;
  ros::ServiceClient landing_srv_client_;

  void stopMsgCallback(const std_msgs::Bool& msg);

  // Graphs.
  std::shared_ptr<GraphManager> local_graph_;
  std::shared_ptr<GraphManager> projected_graph_;
  ShortestPathsReport local_graph_rep_;  // shortest path to root vertex
  std::shared_ptr<GraphManager> global_graph_;
  ShortestPathsReport global_graph_rep_;  // shortest path to root vertex
  std::vector<std::vector<double>> edge_inclinations_;

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

  std::queue<StateVec> robot_state_queue_;

  const double kFreePointCloudUpdatePeriod = 0.5;
  ros::Timer free_pointcloud_update_timer_;
  void freePointCloudtimerCallback(const ros::TimerEvent& event);

  const double kGlobalGraphUpdateTimerPeriod = 0.5;
  const double kGlobalGraphFrontierAdditionTimerPeriod = 1.0;
  const double kGlobalGraphUpdateTimeBudget = 0.1;
  ros::Timer global_graph_update_timer_;
  void expandGlobalGraphTimerCallback(const ros::TimerEvent& event);
  ros::Timer global_graph_frontier_addition_timer_;
  void expandGlobalGraphFrontierAdditionTimerCallback(
      const ros::TimerEvent& event);

  const int backtracking_queue_max_size = 500;
  std::queue<StateVec> robot_backtracking_queue_;
  Vertex* robot_backtracking_prev_;

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

  void convertStateToPoseMsg(const StateVec& state, geometry_msgs::Pose& pose) {
    pose.position.x = state[0];
    pose.position.y = state[1];
    pose.position.z = state[2];
    double yawhalf = state[3] * 0.5;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = sin(yawhalf);
    pose.orientation.w = cos(yawhalf);
  }

  void convertPoseMsgToState(const geometry_msgs::Pose& pose, StateVec& state) {
    state[0] = pose.position.x;
    state[1] = pose.position.y;
    state[2] = pose.position.z;
    state[3] = tf::getYaw(pose.orientation);
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

  // For visualization.
  Visualization* visualization_;

  // Params required for planning.
  SensorParams sensor_params_;
  SensorParams free_frustum_params_;
  RobotParams robot_params_;
  BoundedSpaceParams local_space_params_;
  BoundedSpaceParams global_space_params_;
  std::vector<BoundedSpaceParams> no_gain_zones_;
  bool use_no_gain_space_ = true;
  PlanningParams planning_params_;
  RandomSampler random_sampler_;  // x,y,z,yaw: for exploration purpose
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

#ifdef USE_OCTOMAP
  MapManagerOctomap* map_manager_;
#else
  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
      map_manager_;
#endif

  std::shared_ptr<GeofenceManager> geofence_manager_;

  AdaptiveObb* adaptive_obb_;

  // Mission time tracking
  ros::Time rostime_start_;
  double current_battery_time_remaining_;

  Vertex* root_vertex_;
  Vertex* best_vertex_;

  // Current state of the robot, updated from odometry.
  StateVec current_state_;
  StateVec state_for_planning_;

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
  bool
      auto_global_planner_trig_;  // When true, global planner will be triggered

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

}  // namespace explorer

#endif
