#ifndef GBPLANNER_RVIZ_H_
#define GBPLANNER_RVIZ_H_

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PolygonStamped.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "planner_common/geofence_manager.h"
#include "planner_common/graph.h"
#include "planner_common/graph_base.h"
#include "planner_common/graph_manager.h"
#include "planner_common/map_manager.h"
#include "planner_common/params.h"
#include "planner_common/random_sampler.h"
#include "planner_common/trajectory.h"

namespace explorer {

class Visualization {
 public:
  Visualization(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  // Visualize planning workspace including global and local bounds.
  void visualizeWorkspace(StateVec& state, BoundedSpaceParams& global_ws,
                          BoundedSpaceParams& local_ws);
  // Visualize no gain zones
  void visualizeNoGainZones(std::vector<BoundedSpaceParams>& no_gain_zones);
  // Visualize a graph including its vertices, egdes, and heading angles.
  void visualizeGraph(const std::shared_ptr<GraphManager> graph_manager);
  // Visualize a graph including its vertices, egdes, and heading angles.
  // Used to visualize the graph projected on ground for ground robots
  void visualizeProjectedGraph(
      const std::shared_ptr<GraphManager> graph_manager);
  // Visualize a subset of failed edges to understand why the planning failed.
  void visualizeFailedEdges(std::shared_ptr<SampleStatistic> ss);
  // Visualize bounding box of the robot.
  void visualizeRobotState(StateVec& state, RobotParams& robot_params);
  // Visualize FOV of all sensors using.
  void visualizeSensorFOV(StateVec& state, SensorParams& sensor_params);
  // Visualize shortest paths in a graph computed from Dijkstra algorithm.
  void visualizeShortestPaths(const std::shared_ptr<GraphManager> graph_manager,
                              const ShortestPathsReport& graph_rep);
  // Visualize paths that have edges with negative slope above a threshold
  void visualizeNegativePaths(const std::vector<int>& ids,
                              const std::shared_ptr<GraphManager> graph_manager,
                              const ShortestPathsReport& graph_rep);
  // Visualize edges with negative slope above a threshold
  void visualizeNegativePaths(const std::vector<Eigen::Vector3d>& edge_vertices,
                              const std::shared_ptr<GraphManager> graph_manager,
                              const ShortestPathsReport& graph_rep);

  // Visualize best path[s].
  void visualizeBestPaths(const std::shared_ptr<GraphManager> graph_manager,
                          const ShortestPathsReport& graph_rep, int n,
                          int best_vertex_id);
  // Visualize executing path.
  void visualizeRefPath(const std::vector<geometry_msgs::Pose>& path);
  // Visualize executing path. Option to change color (0=red, 1=green, 2=blue)
  void visualizeRefPath(const std::vector<geometry_msgs::Pose>& path,
                        int color);
  // Visualize volumetric gain.
  void visualizeVolumetricGain(
      Eigen::Vector3d& bound_min, Eigen::Vector3d& bound_max,
      std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>>& voxels,
      double voxel_size);
  // Visualize sampled points.
  void visualizeSampler(RandomSampler& random_sampler);
  // Visualize rays from a state
  void visualizeRays(const StateVec state,
                     const std::vector<Eigen::Vector3d> ray_endpoints);
  // Visualize global graph.
  void visualizeGlobalGraph(const std::shared_ptr<GraphManager> graph_manager);
  // Visualize homing path.
  void visualizeHomingPath(const std::shared_ptr<GraphManager> graph_manager,
                           const ShortestPathsReport& graph_rep,
                           int current_id);

  // Visualize clusters of Dijkstra paths from local planner graph.
  void visualizeClusteredPaths(
      const std::shared_ptr<GraphManager> graph_manager,
      const ShortestPathsReport& graph_rep,
      const std::vector<Vertex*>& vertices,
      const std::vector<int>& cluster_ids);

  void visualizeRobotStateHistory(const std::vector<StateVec*> state_hist);
  // Visualize global planner paths. Path from current location to best frontier
  // and frontier to home location
  void visualizeGlobalPaths(const std::shared_ptr<GraphManager> graph_manager,
                            std::vector<int>& to_frontier_ids,
                            std::vector<int>& to_home_ids);
  void visualizeGeofence(
      const std::shared_ptr<GeofenceManager> geofence_manager);

  // Visualize any pointcloud. Used to visualize the freespace corridor used in
  // path improvement step
  void visualizePCL(const pcl::PointCloud<pcl::PointXYZ>* pcl);

  // Visualize path from root vertex of the graph given by graph_manager to
  // vertex with id=vertex_id
  void visualizePath(const std::shared_ptr<GraphManager> graph_manager,
                     const ShortestPathsReport& graph_rep, int vertex_id);
  void visualizeHyperplanes(Eigen::Vector3d& center,
                            std::vector<Eigen::Vector3d>& hyperplane_list,
                            std::vector<Eigen::Vector3d>& tangent_point_list);
  void visualizeModPath(const std::vector<geometry_msgs::Pose>& path);
  void visualizeBlindModPath(const std::vector<geometry_msgs::Pose>& path);

  // Set the fixed frame of the mission for visualization
  void setGlobalFrame(std::string frame_id) { world_frame_id = frame_id; }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher planning_workspace_pub_;
  ros::Publisher no_gain_zone_pub_;
  ros::Publisher robot_state_pub_;
  ros::Publisher planning_graph_pub_;
  ros::Publisher planning_projected_graph_pub_;
  ros::Publisher planning_failed_pub_;
  ros::Publisher sampler_pub_;
  ros::Publisher geofence_pub_;
  ros::Publisher sensor_fov_pub_;
  ros::Publisher volumetric_gain_pub_;
  ros::Publisher rays_pub_;
  ros::Publisher negative_edges_pub_;
  ros::Publisher shortest_paths_pub_;
  ros::Publisher best_planning_path_pub_;
  ros::Publisher ref_path_pub_;
  ros::Publisher ref_path_color_pub_;
  ros::Publisher mod_path_pub_;
  ros::Publisher blind_mod_path_pub_;
  ros::Publisher planning_global_graph_pub_;
  ros::Publisher planning_homing_pub_;
  ros::Publisher planning_global_pub_;
  ros::Publisher hyperplanes_pub_;
  ros::Publisher clustered_paths_pub_;
  ros::Publisher state_history_pub_;
  ros::Publisher pcl_pub_;
  ros::Publisher path_pub_;

  std::string world_frame_id = "world";
  // 0 = infinite
  const double ws_lifetime = 0;
  const double graph_lifetime = 0.0;
  const double shortest_paths_lifetime = 0.0;
  const double robot_lifetime = 0;
  const double sampler_lifetime = 0;
  const double ray_lifetime = 0;

  int best_path_id_;
  bool getHeatMapColor(float value, float& red, float& green, float& blue);
};

}  // namespace explorer

#endif
