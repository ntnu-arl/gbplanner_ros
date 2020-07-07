#ifndef RRG_H_
#define RRG_H_

#include <fstream>
#include <iostream>
#include <unordered_map>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <kdtree/kdtree.h>

#include "gbplanner/gbplanner_rviz.h"
#include "gbplanner/geofence_manager.h"
#include "gbplanner/graph.h"
#include "gbplanner/graph_manager.h"
#include "gbplanner/map_manager.h"
#include "gbplanner/params.h"
#include "gbplanner/random_sampler.h"
#include "gbplanner/rrg_base.h"
#include "gbplanner/trajectory.h"
#ifdef USE_OCTOMAP
#include "gbplanner/map_manager_octomap_impl.h"
#else
#include "gbplanner/map_manager_voxblox_impl.h"
#endif

#include "planner_msgs/PlanningBound.h"
#include "planner_msgs/PlanningMode.h"
#include "planner_msgs/planner_srv.h"

namespace explorer {
namespace gbplanner {

/* Keep tracking state of the robot every T seconds.
 * This is useful for the global planner
 */
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
    OK = 0,                // Everything is OK.
    ERR_KDTREE,            // Could not search nearest neigbors from kdtree.
    ERR_NO_FEASIBLE_PATH,  // Could not find any path.
    NO_GAIN,               // No non-zero gain found.
    NOT_OK,                // Any other errors.
  };

  Rrg(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Initialize the graph to start a new planning session.
  void reset();

  // Clear out old vertices from previous session.
  void clear();

  // Sample points and construct a graph.
  GraphStatus buildGraph();

  // Check collision free and extend graph with this new vertex.
  void expandGraph(std::shared_ptr<GraphManager> graph_manager,
                   StateVec& new_state, ExpandGraphReport& rep,
                   bool allow_short_edge = false);

  // Expand the tree for RRT* algorithm.
  void expandTreeStar(std::shared_ptr<GraphManager> graph_manager,
                      StateVec& new_state, ExpandGraphReport& rep);

  // Add edges only from this vertex with collision checking.
  void expandGraphEdges(std::shared_ptr<GraphManager> graph_manager,
                        Vertex* new_vertex, ExpandGraphReport& rep);

  // Add egdes without collision checking
  void expandGraphEdgesBlindly(std::shared_ptr<GraphManager> graph_manager,
                               Vertex* new_vertex, double radius,
                               ExpandGraphReport& rep);

  // Compute exploration gain for each vertex in the graph/tree.
  void computeExplorationGain(bool only_leaf_vertices = false);

  // Evaluate gains of all vertices and find the best path[s].
  GraphStatus evaluateGraph();

  // Search for a feasible path to go from a src to a tgt.
  bool search(geometry_msgs::Pose source_pose, geometry_msgs::Pose target_pose,
              bool use_current_state,
              std::vector<geometry_msgs::Pose>& path_ret);

  /* A naive approach (build a graph and find shortest path)
   * to search for a path to connect two arbitrary states in the map.
   * This is not a mature feature, for testing only.
   */
  ConnectStatus findPathToConnect(StateVec& source, StateVec& target,
                                  std::shared_ptr<GraphManager> graph_manager,
                                  RandomSamplingParams& params,
                                  int& final_target_id,
                                  std::vector<geometry_msgs::Pose>& path_ret);

  // Trigger global planner.
  std::vector<geometry_msgs::Pose> runGlobalPlanner(int vertex_id,
                                                    bool not_check_frontier,
                                                    bool ignore_time);

  // Modify P0-P1 to center of the free corridor given surrounding obstacle PCL.
  bool modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
                  Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                  Eigen::Vector3d& p1_mod);

  // Improve any collision free path.
  bool improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                       std::vector<geometry_msgs::Pose>& path_mod);

  // Return the best path calculated from the planner.
  std::vector<geometry_msgs::Pose> getBestPath(std::string tgt_frame,
                                               int& status);

  // Search for the homing path from the current state.
  std::vector<geometry_msgs::Pose> searchHomingPath(std::string tgt_frame,
                                                    const StateVec& cur_state);

  // Get the homing path calculated by the planner.
  std::vector<geometry_msgs::Pose> getHomingPath(std::string tgt_frame);

  // Set current position as homing location.
  bool setHomingPos();

  // Set the root state for local planning (not necessary the current state).
  void setRootStateForPlanning(const geometry_msgs::Pose& root_pose);

  // Set the remaining flight time estimated from battery's status.
  void setTimeRemaining(double t) { current_battery_time_remaining_ = t; }

  // Allow to change the global bound online.
  bool setGlobalBound(planner_msgs::PlanningBound& bound,
                      bool reset_to_default = false);

  // Return the current global bound.
  void getGlobalBound(planner_msgs::PlanningBound& bound);

  // Set the Exploration mode.
  bool setExpMode(planner_msgs::PlanningMode& exp_mode);

  // Load a global graph (e.g. from previous mission).
  bool loadGraph(const std::string& path);

  // Save the global graph from the current mission (e.g. to be re-used).
  bool saveGraph(const std::string& path);

  // Add geofence areas to limit the exploration space.
  void addGeofenceAreas(const geometry_msgs::PolygonStamped& polygon_msgs);

  // Set the current robot's state (e.g. from odometry msg).
  void setState(StateVec& state);

  // Set the bound mode for collision checking.
  void setBoundMode(explorer::BoundModeType bmode);

  // Set the global frame name if it is not "world" (e.g. map).
  void setGlobalFrame(std::string frame_id);

  // Load parameters from the yaml file for the planner.
  bool loadParams();

  // Initialize some variables for later use right after loading parameters.
  void initializeParams();

 private:
  // Randomly sample a vertex from a predefined PDF, used for the local planner.
  bool sampleVertex(StateVec& state);

  // Sample a vertex from a sampler, for others rather than the local planner.
  bool sampleVertex(RandomSampler& random_sampler, StateVec& root_state,
                    StateVec& sampled_state);

  // Volumetric gain calculation using ray casting model.
  void computeVolumetricGainRayModel(StateVec& state, VolumetricGain& vgain,
                                     bool vis_en = false);

  // Volumetric gain calculation without bound checking.
  void computeVolumetricGainRayModelNoBound(StateVec& state,
                                            VolumetricGain& vgain);

  // Volumetric gain calculation, slower than computeVolumetricGainRayModel.
  void computeVolumetricGain(StateVec& state, VolumetricGain& vgain,
                             bool vis_en = false);

  // Correct heading yaw angle following the tangent of each segment.
  void correctYaw();

  // Add frontiers to the global graph.
  void addFrontiers(int best_vertex_id);

  // Add a collision-free path to a graph.
  bool addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                         const std::vector<Vertex*>& vertices);

  // Another instance to add a collision-free path to a graph.
  bool addRefPathToGraph(const std::shared_ptr<GraphManager> graph_manager,
                         const std::vector<geometry_msgs::Pose>& path);

  // Try to add this state to a graph.
  bool connectStateToGraph(std::shared_ptr<GraphManager> graph,
                           StateVec& cur_state, Vertex*& v_added,
                           double dist_ignore_collision_check);

  // Time passed from the beginning of the misison.
  double getTimeElapsed();

  // Time reamining for the mission.
  double getTimeRemained();

  // Compare if still have enough time.
  bool isRemainingTimeSufficient(const double& time_cost, double& time_spare);

  // Current exploring direction for directionality bias.
  double exploring_direction_;

  // Timer callbacks to call the graph update for the global graph.
  void timerCallback(const ros::TimerEvent& event);
  void expandGlobalGraphTimerCallback(const ros::TimerEvent& event);

  // Publish a PCL to free the space.
  void freePointCloudtimerCallback(const ros::TimerEvent& event);

  // Utilities.
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

///--------------------
  // ROS node handle either public or private options.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Local graph.
  std::shared_ptr<GraphManager> local_graph_;

  // A report/result after a planning step from the local planner.
  ShortestPathsReport local_graph_rep_;

  // Global graph for global planner.
  std::shared_ptr<GraphManager> global_graph_;

  // Report from the global planner.
  ShortestPathsReport global_graph_rep_;
  std::queue<StateVec> robot_state_queue_;

  // For RVIZ visualization.
  Visualization* visualization_;

  // Params required for planning.
  SensorParams sensor_params_;
  SensorParams free_frustum_params_;
  RobotParams robot_params_;
  BoundedSpaceParams local_space_params_;
  BoundedSpaceParams global_space_params_;
  PlanningParams planning_params_;

  // Random samplers for planning.
  RandomSampler random_sampler_;
  RandomSampler random_sampler_to_search_;  // for the search function.
  BoundedSpaceParams local_search_params_;
  RandomSampler random_sampler_vertical_;   // for vertical exploration mode.
  BoundedSpaceParams local_vertical_params_;
  RobotDynamicsParams robot_dynamics_params_;
  BoundingBoxType global_bound_;

  // Map interface either Octomap or Voxblox.
#ifdef USE_OCTOMAP
  MapManagerOctomap* map_manager_;
#else
  MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
      map_manager_;
#endif

  // Geofence areas manager.
  std::shared_ptr<GeofenceManager> geofence_manager_;

  // Flight time tracking
  ros::Time rostime_start_;
  double current_battery_time_remaining_;

  // Special vertices.
  Vertex* root_vertex_;
  Vertex* best_vertex_;

  // Current state of the robot, updated from odometry.
  StateVec current_state_;
  StateVec state_for_planning_;

  // Current waypoint
  StateVec current_waypoint_;

  // Precompute params for planner.
  Eigen::Vector3d robot_box_size_;
  int planning_num_vertices_max_;
  int planning_num_edges_max_;

  // Statistic from the random sampling.
  std::shared_ptr<SampleStatistic> stat_;

  // Number of the planning steps triggered during the mission.
  int planner_trigger_time_;

  // Temporary variable for timing purpose.
  ros::Time ttime;

  // Received the robot's state from the localization module, ready to plan.
  bool odometry_ready;

  // Save a spare set of states from odometry for homing purpose.
  StateVec last_state_marker_;
  StateVec last_state_marker_global_;

  // Robot's history for global planning purposes.
  std::shared_ptr<RobotStateHistory> robot_state_hist_;
  const int backtracking_queue_max_size = 500;
  std::queue<StateVec> robot_backtracking_queue_;
  Vertex* robot_backtracking_prev_;

  // To visualize the feasible corridors in the path refinement step.
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> feasible_corridor_pcl_;

  // Timer for global graph update.
  const double kTimerPeriod = 0.25;
  ros::Timer periodic_timer_;
  const double kGlobalGraphUpdateTimerPeriod = 0.5;
  const double kGlobalGraphUpdateTimeBudget = 0.1;
  ros::Timer global_graph_update_timer_;

  /* This is a hacky way to augment a free PCL to the map to clear out free
   * space for planning in special cases like vertical exploration.
   */
  ros::Publisher free_cloud_pub_;
  const double kFreePointCloudUpdatePeriod = 0.5;
  ros::Timer free_pointcloud_update_timer_;
};

}  // namespace gbplanner
}  // namespace explorer

#endif
