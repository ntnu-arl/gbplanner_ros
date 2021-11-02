#include "gbplanner/gbplanner_rviz.h"

#include <tf/transform_datatypes.h>

namespace explorer {

Visualization::Visualization(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private) {
  nh_ = nh;
  nh_private_ = nh_private;
  planning_workspace_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "vis/planning_workspace", 10);
  no_gain_zone_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/no_gain_zone", 10);
  planning_graph_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/planning_graph", 10);
  planning_projected_graph_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(
          "vis/planning_projected_graph", 10);
  planning_failed_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/planning_failed", 10);
  shortest_paths_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/shortest_paths", 10);
  robot_state_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/robot_state", 100);
  sensor_fov_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/sensor_fov", 100);
  best_planning_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "vis/best_planning_paths", 10);
  volumetric_gain_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "vis/volumetric_gains", 10);
  sampler_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/sampler", 10);
  rays_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/ray_casting", 10);
  ref_path_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/ref_path", 10);
  ref_path_color_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/imp_ref_path", 10);
  planning_global_graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "vis/planning_global_graph", 10);
  planning_homing_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "vis/planning_homing_path", 10);
  planning_global_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "vis/planning_global_path", 10);
  clustered_paths_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "vis/shortest_path_clustering", 10);
  state_history_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/state_history", 10);
  geofence_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/geofence", 10);
  negative_edges_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/negative_edges", 10);

  hyperplanes_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/hyperplanes", 10);
  mod_path_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/mod_path", 10);
  blind_mod_path_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/blind_mod_path", 10);

  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("vis/occupied_pcl", 10);
  path_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vis/alternate_path", 10);
  best_path_id_ = 0;
}

void Visualization::visualizeWorkspace(StateVec& state,
                                       BoundedSpaceParams& global_ws,
                                       BoundedSpaceParams& local_ws) {
  if (planning_workspace_pub_.getNumSubscribers() < 1) return;
  visualization_msgs::MarkerArray marker_array;

  // Global ws
  visualization_msgs::Marker global_ws_marker;
  global_ws_marker.header.stamp = ros::Time::now();
  global_ws_marker.header.seq = 0;
  global_ws_marker.header.frame_id = world_frame_id;
  global_ws_marker.id = 0;
  global_ws_marker.ns = "global";
  global_ws_marker.action = visualization_msgs::Marker::ADD;
  if (global_ws.type == BoundedSpaceType::kCuboid) {
    global_ws_marker.type = visualization_msgs::Marker::CUBE;
    global_ws_marker.pose.position.x =
        global_ws.getCenter()(0) +
        0.5 * (global_ws.min_val(0) + global_ws.max_val(0));
    global_ws_marker.pose.position.y =
        global_ws.getCenter()(1) +
        0.5 * (global_ws.min_val(1) + global_ws.max_val(1));
    global_ws_marker.pose.position.z =
        global_ws.getCenter()(2) +
        0.5 * (global_ws.min_val(2) + global_ws.max_val(2));
    global_ws_marker.scale.x = global_ws.max_val[0] - global_ws.min_val[0];
    global_ws_marker.scale.y = global_ws.max_val[1] - global_ws.min_val[1];
    global_ws_marker.scale.z = global_ws.max_val[2] - global_ws.min_val[2];
  } else if (global_ws.type == BoundedSpaceType::kSphere) {
    global_ws_marker.type = visualization_msgs::Marker::SPHERE;
    global_ws_marker.pose.position.x = 0;
    global_ws_marker.pose.position.y = 0;
    global_ws_marker.pose.position.z = 0;
    global_ws_marker.scale.x = global_ws.radius * 2;
    global_ws_marker.scale.y = global_ws.radius * 2;
    global_ws_marker.scale.z = global_ws.radius * 2;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Not supported.");
    return;
  }
  Eigen::Matrix3d g_rot;
  g_rot = Eigen::AngleAxisd(global_ws.rotations[0], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(global_ws.rotations[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(global_ws.rotations[2], Eigen::Vector3d::UnitX());
  Eigen::Quaterniond g_quat(g_rot);
  global_ws_marker.pose.orientation.x = g_quat.x();
  global_ws_marker.pose.orientation.y = g_quat.y();
  global_ws_marker.pose.orientation.z = g_quat.z();
  global_ws_marker.pose.orientation.w = g_quat.w();
  global_ws_marker.color.r = 200.0 / 255.0;
  global_ws_marker.color.g = 100.0 / 255.0;
  global_ws_marker.color.b = 0.0;
  global_ws_marker.color.a = 0.25;
  global_ws_marker.lifetime = ros::Duration(ws_lifetime);
  global_ws_marker.frame_locked = false;
  marker_array.markers.push_back(global_ws_marker);

  // Local ws
  visualization_msgs::Marker local_ws_marker;
  local_ws_marker.header.stamp = ros::Time::now();
  local_ws_marker.header.seq = 0;
  local_ws_marker.header.frame_id = world_frame_id;
  local_ws_marker.ns = "local";
  local_ws_marker.id = 0;
  local_ws_marker.action = visualization_msgs::Marker::ADD;

  Eigen::Matrix3d rot;
  rot = Eigen::AngleAxisd(local_ws.rotations[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(local_ws.rotations[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(local_ws.rotations[2], Eigen::Vector3d::UnitX());
  Eigen::Matrix3d rot_inv = rot.inverse();
  Eigen::Quaterniond quatd(rot_inv);
  local_ws_marker.pose.orientation.x = quatd.x();
  local_ws_marker.pose.orientation.y = quatd.y();
  local_ws_marker.pose.orientation.z = quatd.z();
  local_ws_marker.pose.orientation.w = quatd.w();

  Eigen::Vector3d offset =
      0.5 * rot_inv * (local_ws.min_val + local_ws.max_val);

  if (local_ws.type == BoundedSpaceType::kCuboid) {
    local_ws_marker.type = visualization_msgs::Marker::CUBE;
    local_ws_marker.pose.position.x = state[0] + offset[0];
    local_ws_marker.pose.position.y = state[1] + offset[1];
    local_ws_marker.pose.position.z = state[2] + offset[2];
    local_ws_marker.scale.x = local_ws.max_val[0] - local_ws.min_val[0];
    local_ws_marker.scale.y = local_ws.max_val[1] - local_ws.min_val[1];
    local_ws_marker.scale.z = local_ws.max_val[2] - local_ws.min_val[2];
  } else if (local_ws.type == BoundedSpaceType::kSphere) {
    local_ws_marker.type = visualization_msgs::Marker::SPHERE;
    local_ws_marker.pose.position.x = state[0];
    local_ws_marker.pose.position.y = state[1];
    local_ws_marker.pose.position.z = state[2];
    local_ws_marker.scale.x = local_ws.radius * 2;
    local_ws_marker.scale.y = local_ws.radius * 2;
    local_ws_marker.scale.z = local_ws.radius * 2;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Not supported.");
    return;
  }

  local_ws_marker.color.r = 255.0 / 255.0;
  local_ws_marker.color.g = 100.0 / 255.0;
  local_ws_marker.color.b = 255.0 / 255.0;
  local_ws_marker.color.a = 0.25;
  local_ws_marker.lifetime = ros::Duration(ws_lifetime);
  local_ws_marker.frame_locked = false;
  marker_array.markers.push_back(local_ws_marker);

  planning_workspace_pub_.publish(marker_array);
}

void Visualization::visualizeNoGainZones(
    std::vector<BoundedSpaceParams>& no_gain_zones) {
  if (no_gain_zone_pub_.getNumSubscribers() < 1) return;
  visualization_msgs::MarkerArray marker_array;

  for (int i = 0; i < no_gain_zones.size(); ++i) {
    BoundedSpaceParams no_gain_zone = no_gain_zones[i];
    visualization_msgs::Marker no_gain_zone_marker;
    no_gain_zone_marker.header.stamp = ros::Time::now();
    no_gain_zone_marker.header.seq = 0;
    no_gain_zone_marker.header.frame_id = world_frame_id;
    no_gain_zone_marker.id = 0;
    no_gain_zone_marker.ns = "no_gain_zone_" + std::to_string(i);
    no_gain_zone_marker.action = visualization_msgs::Marker::ADD;
    if (no_gain_zone.type == BoundedSpaceType::kCuboid) {
      no_gain_zone_marker.type = visualization_msgs::Marker::CUBE;
      no_gain_zone_marker.pose.position.x =
          no_gain_zone.getCenter()(0) +
          0.5 * (no_gain_zone.min_val(0) + no_gain_zone.max_val(0));
      no_gain_zone_marker.pose.position.y =
          no_gain_zone.getCenter()(1) +
          0.5 * (no_gain_zone.min_val(1) + no_gain_zone.max_val(1));
      no_gain_zone_marker.pose.position.z =
          no_gain_zone.getCenter()(2) +
          0.5 * (no_gain_zone.min_val(2) + no_gain_zone.max_val(2));
      no_gain_zone_marker.scale.x =
          no_gain_zone.max_val[0] - no_gain_zone.min_val[0];
      no_gain_zone_marker.scale.y =
          no_gain_zone.max_val[1] - no_gain_zone.min_val[1];
      no_gain_zone_marker.scale.z =
          no_gain_zone.max_val[2] - no_gain_zone.min_val[2];
    } else if (no_gain_zone.type == BoundedSpaceType::kSphere) {
      no_gain_zone_marker.type = visualization_msgs::Marker::SPHERE;
      no_gain_zone_marker.pose.position.x = 0;
      no_gain_zone_marker.pose.position.y = 0;
      no_gain_zone_marker.pose.position.z = 0;
      no_gain_zone_marker.scale.x = no_gain_zone.radius * 2;
      no_gain_zone_marker.scale.y = no_gain_zone.radius * 2;
      no_gain_zone_marker.scale.z = no_gain_zone.radius * 2;
    } else {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Not supported.");
      return;
    }
    Eigen::Matrix3d g_rot;
    g_rot =
        Eigen::AngleAxisd(no_gain_zone.rotations[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(no_gain_zone.rotations[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(no_gain_zone.rotations[2], Eigen::Vector3d::UnitX());
    Eigen::Quaterniond g_quat(g_rot);
    no_gain_zone_marker.pose.orientation.x = g_quat.x();
    no_gain_zone_marker.pose.orientation.y = g_quat.y();
    no_gain_zone_marker.pose.orientation.z = g_quat.z();
    no_gain_zone_marker.pose.orientation.w = g_quat.w();
    no_gain_zone_marker.color.r = 255.0 / 255.0;
    no_gain_zone_marker.color.g = 0.0 / 255.0;
    no_gain_zone_marker.color.b = 0.0;
    no_gain_zone_marker.color.a = 0.25;
    no_gain_zone_marker.lifetime = ros::Duration(ws_lifetime);
    no_gain_zone_marker.frame_locked = false;
    marker_array.markers.push_back(no_gain_zone_marker);
  }

  no_gain_zone_pub_.publish(marker_array);
}

void Visualization::visualizeGraph(
    const std::shared_ptr<GraphManager> graph_manager) {
  std::shared_ptr<Graph> g = graph_manager->graph_;
  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  if (graph_manager->getNumVertices() == 0) return;
  if (planning_graph_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 125.0 / 255.0;
  vertex_marker.color.g = 42.0 / 255.0;
  vertex_marker.color.b = 104.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  // Plot all hanging vertices
  visualization_msgs::Marker hanging_vertex_marker;
  hanging_vertex_marker.header.stamp = ros::Time::now();
  hanging_vertex_marker.header.seq = 0;
  hanging_vertex_marker.header.frame_id = world_frame_id;
  hanging_vertex_marker.id = 0;
  hanging_vertex_marker.ns = "hanging_vertices";
  hanging_vertex_marker.action = visualization_msgs::Marker::ADD;
  hanging_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  hanging_vertex_marker.scale.x = 0.4;
  hanging_vertex_marker.scale.y = 0.4;
  hanging_vertex_marker.scale.z = 0.4;
  hanging_vertex_marker.color.r = 255.0 / 255.0;
  hanging_vertex_marker.color.g = 0.0 / 255.0;
  hanging_vertex_marker.color.b = 255.0 / 255.0;
  hanging_vertex_marker.color.a = 1.0;
  hanging_vertex_marker.lifetime = ros::Duration(graph_lifetime);
  hanging_vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi_h;
  g->getVertexIterator(vi_h);
  for (Graph::GraphType::vertex_iterator it = vi_h.first; it != vi_h.second;
       ++it) {
    int id = g->getVertexProperty(it);
    if (v_map[id]->is_hanging) {
      // if(true){
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      hanging_vertex_marker.points.push_back(p1);
    }
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "[Vis]: Num hanging verts: %d", (int)hanging_vertex_marker.points.size());
  marker_array.markers.push_back(hanging_vertex_marker);

  // // Plot all headings
  // int marker_id = 0;
  // for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second;
  // ++it) {
  //   visualization_msgs::Marker marker;
  //   marker.header.stamp = ros::Time::now();
  //   marker.header.seq = 0;
  //   marker.header.frame_id = world_frame_id;
  //   marker.ns = "heading";
  //   marker.action = visualization_msgs::Marker::ADD;
  //   marker.type = visualization_msgs::Marker::ARROW;
  //   marker.scale.x = 0.5;   // length of the arrow
  //   marker.scale.y = 0.15;  // arrow width
  //   marker.scale.z = 0.15;  // arrow height
  //   marker.color.r = 200.0 / 255.0;
  //   marker.color.g = 50.0 / 255.0;
  //   marker.color.b = 0.0;
  //   marker.color.a = 0.3;
  //   marker.lifetime = ros::Duration(graph_lifetime);
  //   marker.frame_locked = false;
  //   int id = g->getVertexProperty(it);
  //   marker.pose.position.x = v_map[id]->state[0];
  //   marker.pose.position.y = v_map[id]->state[1];
  //   marker.pose.position.z = v_map[id]->state[2];
  //   tf::Quaternion quat;
  //   quat.setRPY(0.0, 0.0, v_map[id]->state[3]);
  //   marker.pose.orientation.x = quat.x();
  //   marker.pose.orientation.y = quat.y();
  //   marker.pose.orientation.z = quat.z();
  //   marker.pose.orientation.w = quat.w();
  //   marker.id = marker_id++;
  //   marker_array.markers.push_back(marker);
  // }

  planning_graph_pub_.publish(marker_array);
}

void Visualization::visualizeProjectedGraph(
    const std::shared_ptr<GraphManager> graph_manager) {
  std::shared_ptr<Graph> g = graph_manager->graph_;
  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  if (graph_manager->getNumVertices() == 0) return;
  if (planning_projected_graph_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 125.0 / 255.0;
  vertex_marker.color.g = 42.0 / 255.0;
  vertex_marker.color.b = 104.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  // // Plot all headings
  // int marker_id = 0;
  // for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second;
  // ++it) {
  //   visualization_msgs::Marker marker;
  //   marker.header.stamp = ros::Time::now();
  //   marker.header.seq = 0;
  //   marker.header.frame_id = world_frame_id;
  //   marker.ns = "heading";
  //   marker.action = visualization_msgs::Marker::ADD;
  //   marker.type = visualization_msgs::Marker::ARROW;
  //   marker.scale.x = 0.5;   // length of the arrow
  //   marker.scale.y = 0.15;  // arrow width
  //   marker.scale.z = 0.15;  // arrow height
  //   marker.color.r = 200.0 / 255.0;
  //   marker.color.g = 50.0 / 255.0;
  //   marker.color.b = 0.0;
  //   marker.color.a = 0.3;
  //   marker.lifetime = ros::Duration(graph_lifetime);
  //   marker.frame_locked = false;
  //   int id = g->getVertexProperty(it);
  //   marker.pose.position.x = v_map[id]->state[0];
  //   marker.pose.position.y = v_map[id]->state[1];
  //   marker.pose.position.z = v_map[id]->state[2];
  //   tf::Quaternion quat;
  //   quat.setRPY(0.0, 0.0, v_map[id]->state[3]);
  //   marker.pose.orientation.x = quat.x();
  //   marker.pose.orientation.y = quat.y();
  //   marker.pose.orientation.z = quat.z();
  //   marker.pose.orientation.w = quat.w();
  //   marker.id = marker_id++;
  //   marker_array.markers.push_back(marker);
  // }

  planning_projected_graph_pub_.publish(marker_array);
}

void Visualization::visualizeGlobalGraph(
    const std::shared_ptr<GraphManager> graph_manager) {
  std::shared_ptr<Graph> g = graph_manager->graph_;
  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  if (graph_manager->getNumVertices() == 0) return;
  if (planning_global_graph_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all edges using line (fast)
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.1;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(graph_lifetime);
  edge_marker.frame_locked = false;

  std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
      ei;
  g->getEdgeIterator(ei);
  for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it) {
    int src_id, tgt_id;
    double weight;
    std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[src_id]->state[0];
    p1.y = v_map[src_id]->state[1];
    p1.z = v_map[src_id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[tgt_id]->state[0];
    p2.y = v_map[tgt_id]->state[1];
    p2.z = v_map[tgt_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // // Plot all edges using arrows (slow)
  // int marker_idd = 0;
  // std::pair<Graph::GraphType::edge_iterator, Graph::GraphType::edge_iterator>
  //     ei;
  // g->getEdgeIterator(ei);
  // for (Graph::GraphType::edge_iterator it = ei.first; it != ei.second; ++it)
  // {
  //   visualization_msgs::Marker marker;
  //   marker.header.stamp = ros::Time::now();
  //   marker.header.seq = 0;
  //   marker.header.frame_id = world_frame_id;
  //   marker.ns = "edges1";
  //   marker.action = visualization_msgs::Marker::ADD;
  //   marker.type = visualization_msgs::Marker::ARROW;
  //   marker.scale.x = 0.1;
  //   marker.scale.y = 0.0;
  //   marker.scale.z = 0.1;
  //   marker.color.r = 200.0 / 255.0;
  //   marker.color.g = 50.0 / 255.0;
  //   marker.color.b = 0.0;
  //   marker.color.a = 1.0;
  //   marker.lifetime = ros::Duration(graph_lifetime);
  //   marker.frame_locked = false;
  //   marker.id = marker_idd++;

  //   int src_id, tgt_id;
  //   double weight;
  //   std::tie(src_id, tgt_id, weight) = g->getEdgeProperty(it);
  //   geometry_msgs::Point p1;
  //   p1.x = v_map[src_id]->state[0];
  //   p1.y = v_map[src_id]->state[1];
  //   p1.z = v_map[src_id]->state[2];
  //   geometry_msgs::Point p2;
  //   p2.x = v_map[tgt_id]->state[0];
  //   p2.y = v_map[tgt_id]->state[1];
  //   p2.z = v_map[tgt_id]->state[2];
  //   marker.points.push_back(p1);
  //   marker.points.push_back(p2);
  //   marker_array.markers.push_back(marker);
  // }

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 53.0 / 255.0;
  vertex_marker.color.g = 49.0 / 255.0;
  vertex_marker.color.b = 119.0 / 255.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(graph_lifetime);
  vertex_marker.frame_locked = false;

  std::pair<Graph::GraphType::vertex_iterator,
            Graph::GraphType::vertex_iterator>
      vi;
  g->getVertexIterator(vi);
  for (Graph::GraphType::vertex_iterator it = vi.first; it != vi.second; ++it) {
    int id = g->getVertexProperty(it);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  visualization_msgs::Marker frontier_marker;
  frontier_marker.header.stamp = ros::Time::now();
  frontier_marker.header.seq = 0;
  frontier_marker.header.frame_id = world_frame_id;
  frontier_marker.id = 0;
  frontier_marker.ns = "frontier";
  frontier_marker.action = visualization_msgs::Marker::ADD;
  frontier_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  frontier_marker.scale.x = 0.5;
  frontier_marker.scale.y = 0.5;
  frontier_marker.scale.z = 0.5;
  frontier_marker.color.r = 1.0;
  frontier_marker.color.g = 0.0;
  frontier_marker.color.b = 0.0;
  frontier_marker.color.a = 1.0;
  frontier_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  frontier_marker.frame_locked = false;
  int num_vertices = graph_manager->getNumVertices();
  int marker_id = 0;
  for (int id = 0; id < num_vertices; ++id) {
    if (v_map[id]->type == VertexType::kFrontier) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      frontier_marker.points.push_back(p1);

      // Gain
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      marker.header.seq = 0;
      marker.header.frame_id = world_frame_id;
      marker.ns = "gain";
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 2.0;  // text height
      marker.color.r = 18.0 / 255.0;
      marker.color.g = 15.0 / 255.0;
      marker.color.b = 196.0 / 255.0;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration(shortest_paths_lifetime);
      marker.frame_locked = false;
      marker.pose.position.x = v_map[id]->state[0];
      marker.pose.position.y = v_map[id]->state[1];
      marker.pose.position.z = v_map[id]->state[2] + 0.1;
      std::string text_display = std::to_string(v_map[id]->id);

      marker.text = text_display;
      marker.id = marker_id++;
      marker_array.markers.push_back(marker);
    }
  }
  marker_array.markers.push_back(frontier_marker);

  // Semantics
  visualization_msgs::Marker semantic_marker_stairs;
  semantic_marker_stairs.header.stamp = ros::Time::now();
  semantic_marker_stairs.header.seq = 0;
  semantic_marker_stairs.header.frame_id = world_frame_id;
  semantic_marker_stairs.id = 0;
  semantic_marker_stairs.ns = "stairs";
  semantic_marker_stairs.action = visualization_msgs::Marker::ADD;
  semantic_marker_stairs.type = visualization_msgs::Marker::CUBE_LIST;
  semantic_marker_stairs.scale.x = 0.7;
  semantic_marker_stairs.scale.y = 0.7;
  semantic_marker_stairs.scale.z = 0.7;
  semantic_marker_stairs.color.r = 1.0;
  semantic_marker_stairs.color.g = 1.0;
  semantic_marker_stairs.color.b = 0.0;
  semantic_marker_stairs.color.a = 1.0;
  semantic_marker_stairs.lifetime = ros::Duration(shortest_paths_lifetime);
  semantic_marker_stairs.frame_locked = false;
  int num_vertices_stairs = graph_manager->getNumVertices();
  // std::cout << "l11" << std::endl;
  for (int id = 0; id < num_vertices_stairs; ++id) {
    if (v_map[id]->semantic_class.value ==
        planner_semantic_msgs::SemanticClass::kStaircase) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      semantic_marker_stairs.points.push_back(p1);
    }
  }
  marker_array.markers.push_back(semantic_marker_stairs);

  visualization_msgs::Marker semantic_marker_door;
  semantic_marker_door.header.stamp = ros::Time::now();
  semantic_marker_door.header.seq = 0;
  semantic_marker_door.header.frame_id = world_frame_id;
  semantic_marker_door.id = 0;
  semantic_marker_door.ns = "door";
  semantic_marker_door.action = visualization_msgs::Marker::ADD;
  semantic_marker_door.type = visualization_msgs::Marker::CUBE_LIST;
  semantic_marker_door.scale.x = 0.7;
  semantic_marker_door.scale.y = 0.7;
  semantic_marker_door.scale.z = 0.7;
  semantic_marker_door.color.r = 1.0;
  semantic_marker_door.color.g = 0.0;
  semantic_marker_door.color.b = 1.0;
  semantic_marker_door.color.a = 1.0;
  semantic_marker_door.lifetime = ros::Duration(shortest_paths_lifetime);
  semantic_marker_door.frame_locked = false;
  int num_vertices_door = graph_manager->getNumVertices();
  // std::cout << "l11" << std::endl;
  for (int id = 0; id < num_vertices_door; ++id) {
    if (v_map[id]->semantic_class.value ==
        planner_semantic_msgs::SemanticClass::kDoor) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      semantic_marker_door.points.push_back(p1);
    }
  }
  marker_array.markers.push_back(semantic_marker_door);
  // visualization_msgs::Marker visited_marker;
  // visited_marker.header.stamp = ros::Time::now();
  // visited_marker.header.seq = 0;
  // visited_marker.header.frame_id = world_frame_id;
  // visited_marker.id = 0;
  // visited_marker.ns = "visited";
  // visited_marker.action = visualization_msgs::Marker::ADD;
  // visited_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  // visited_marker.scale.x = 0.5;
  // visited_marker.scale.y = 0.5;
  // visited_marker.scale.z = 0.5;
  // visited_marker.color.r = 0.0;
  // visited_marker.color.g = 1.0;
  // visited_marker.color.b = 0.0;
  // visited_marker.color.a = 1.0;
  // visited_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  // visited_marker.frame_locked = false;
  // marker_id = 0;
  // for (int id = 0; id < num_vertices; ++id) {
  //   if (v_map[id]->type == VertexType::kVisited) {
  //     geometry_msgs::Point p1;
  //     p1.x = v_map[id]->state[0];
  //     p1.y = v_map[id]->state[1];
  //     p1.z = v_map[id]->state[2];
  //     visited_marker.points.push_back(p1);
  //   }
  // }
  // marker_array.markers.push_back(visited_marker);

  // visualization_msgs::Marker unvisited_marker;
  // unvisited_marker.header.stamp = ros::Time::now();
  // unvisited_marker.header.seq = 0;
  // unvisited_marker.header.frame_id = world_frame_id;
  // unvisited_marker.id = 0;
  // unvisited_marker.ns = "unvisited";
  // unvisited_marker.action = visualization_msgs::Marker::ADD;
  // unvisited_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  // unvisited_marker.scale.x = 0.5;
  // unvisited_marker.scale.y = 0.5;
  // unvisited_marker.scale.z = 0.5;
  // unvisited_marker.color.r = 0.0;
  // unvisited_marker.color.g = 0.0;
  // unvisited_marker.color.b = 1.0;
  // unvisited_marker.color.a = 1.0;
  // unvisited_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  // unvisited_marker.frame_locked = false;
  // marker_id = 0;
  // for (int id = 0; id < num_vertices; ++id) {
  //   if (v_map[id]->type == VertexType::kUnvisited) {
  //     geometry_msgs::Point p1;
  //     p1.x = v_map[id]->state[0];
  //     p1.y = v_map[id]->state[1];
  //     p1.z = v_map[id]->state[2];
  //     unvisited_marker.points.push_back(p1);
  //   }
  // }
  // marker_array.markers.push_back(unvisited_marker);

  planning_global_graph_pub_.publish(marker_array);
}

void Visualization::visualizeFailedEdges(std::shared_ptr<SampleStatistic> ss) {
  if (planning_failed_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all edges
  visualization_msgs::Marker edge_fail_marker;
  edge_fail_marker.header.stamp = ros::Time::now();
  edge_fail_marker.header.seq = 0;
  edge_fail_marker.header.frame_id = world_frame_id;
  edge_fail_marker.id = 0;
  edge_fail_marker.ns = "failed_edges";
  edge_fail_marker.action = visualization_msgs::Marker::ADD;
  edge_fail_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_fail_marker.scale.x = 0.1;
  edge_fail_marker.color.r = 200.0 / 255.0;
  edge_fail_marker.color.g = 0.0;
  edge_fail_marker.color.b = 0.0;
  edge_fail_marker.color.a = 1.0;
  edge_fail_marker.lifetime = ros::Duration(graph_lifetime);
  edge_fail_marker.frame_locked = false;

  int limit_count = 0;
  for (auto it = ss->edges_fail.begin();
       ((it != ss->edges_fail.end()) && (limit_count++ < 100)); ++it) {
    std::vector<double> edge = *it;
    geometry_msgs::Point p1;
    p1.x = edge[0];
    p1.y = edge[1];
    p1.z = edge[2];
    geometry_msgs::Point p2;
    p2.x = edge[3];
    p2.y = edge[4];
    p2.z = edge[5];
    edge_fail_marker.points.push_back(p1);
    edge_fail_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_fail_marker);
  planning_failed_pub_.publish(marker_array);
}

void Visualization::visualizeRobotState(StateVec& state,
                                        RobotParams& robot_params) {
  // @TODO: not complete yet, have to incoporate offset
  if (robot_state_pub_.getNumSubscribers() < 1) return;
  visualization_msgs::MarkerArray marker_array;

  // True size
  visualization_msgs::Marker robot_size_marker;
  robot_size_marker.header.stamp = ros::Time::now();
  robot_size_marker.header.seq = 0;
  robot_size_marker.header.frame_id = world_frame_id;
  robot_size_marker.id = 0;
  robot_size_marker.ns = "size";
  robot_size_marker.action = visualization_msgs::Marker::ADD;
  robot_size_marker.type = visualization_msgs::Marker::CUBE;
  robot_size_marker.color.r = 1.0;
  robot_size_marker.color.g = 100.0 / 255.0;
  robot_size_marker.color.b = 100.0 / 255.0;
  robot_size_marker.color.a = 0.5;
  robot_size_marker.scale.x = robot_params.size[0];
  robot_size_marker.scale.y = robot_params.size[1];
  robot_size_marker.scale.z = robot_params.size[2];
  robot_size_marker.pose.position.x = state[0] + robot_params.center_offset[0];
  robot_size_marker.pose.position.y = state[1] + robot_params.center_offset[1];
  robot_size_marker.pose.position.z = state[2] + robot_params.center_offset[2];
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, state[3]);
  robot_size_marker.pose.orientation.x = quat.x();
  robot_size_marker.pose.orientation.y = quat.y();
  robot_size_marker.pose.orientation.z = quat.z();
  robot_size_marker.pose.orientation.w = quat.w();
  robot_size_marker.lifetime = ros::Duration(robot_lifetime);
  robot_size_marker.frame_locked = false;
  marker_array.markers.push_back(robot_size_marker);

  // Extension size-aligned with heading
  visualization_msgs::Marker robot_esize_marker;
  robot_esize_marker.header.stamp = ros::Time::now();
  robot_esize_marker.header.seq = 0;
  robot_esize_marker.header.frame_id = world_frame_id;
  robot_esize_marker.id = 0;
  robot_esize_marker.ns = "extension_aligned";
  robot_esize_marker.action = visualization_msgs::Marker::ADD;
  robot_esize_marker.type = visualization_msgs::Marker::CUBE;
  robot_esize_marker.color.r = 1.0;
  robot_esize_marker.color.g = 50.0 / 255.0;
  robot_esize_marker.color.b = 50.0 / 255.0;
  robot_esize_marker.color.a = 0.5;
  Eigen::Vector3d ext_size;
  robot_params.getPlanningSize(ext_size);
  robot_esize_marker.scale.x = ext_size[0];
  robot_esize_marker.scale.y = ext_size[1];
  robot_esize_marker.scale.z = ext_size[2];
  robot_esize_marker.pose.position.x = state[0] + robot_params.center_offset[0];
  robot_esize_marker.pose.position.y = state[1] + robot_params.center_offset[1];
  robot_esize_marker.pose.position.z = state[2] + robot_params.center_offset[2];
  robot_esize_marker.pose.orientation.x = quat.x();
  robot_esize_marker.pose.orientation.y = quat.y();
  robot_esize_marker.pose.orientation.z = quat.z();
  robot_esize_marker.pose.orientation.w = quat.w();
  robot_esize_marker.lifetime = ros::Duration(robot_lifetime);
  robot_esize_marker.frame_locked = false;
  marker_array.markers.push_back(robot_esize_marker);

  // Extension size in XYZ World.
  visualization_msgs::Marker robot_esize_xyz_marker;
  robot_esize_xyz_marker.header.stamp = ros::Time::now();
  robot_esize_xyz_marker.header.seq = 0;
  robot_esize_xyz_marker.header.frame_id = world_frame_id;
  robot_esize_xyz_marker.id = 0;
  robot_esize_xyz_marker.ns = "extension";
  robot_esize_xyz_marker.action = visualization_msgs::Marker::ADD;
  robot_esize_xyz_marker.type = visualization_msgs::Marker::CUBE;
  robot_esize_xyz_marker.color.r = 0.0;
  robot_esize_xyz_marker.color.g = 200.0 / 255.0;
  robot_esize_xyz_marker.color.b = 0.0;
  robot_esize_xyz_marker.color.a = 0.5;
  robot_params.getPlanningSize(ext_size);
  robot_esize_xyz_marker.scale.x = ext_size[0];
  robot_esize_xyz_marker.scale.y = ext_size[1];
  robot_esize_xyz_marker.scale.z = ext_size[2];
  robot_esize_xyz_marker.pose.position.x =
      state[0] + robot_params.center_offset[0];
  robot_esize_xyz_marker.pose.position.y =
      state[1] + robot_params.center_offset[1];
  robot_esize_xyz_marker.pose.position.z =
      state[2] + robot_params.center_offset[2];
  robot_esize_xyz_marker.pose.orientation.x = 0.0;
  robot_esize_xyz_marker.pose.orientation.y = 0.0;
  robot_esize_xyz_marker.pose.orientation.z = 0.0;
  robot_esize_xyz_marker.pose.orientation.w = 1.0;
  robot_esize_xyz_marker.lifetime = ros::Duration(robot_lifetime);
  robot_esize_xyz_marker.frame_locked = false;
  marker_array.markers.push_back(robot_esize_xyz_marker);

  // Center and heading.
  visualization_msgs::Marker robot_ori_marker;
  robot_ori_marker.header.stamp = ros::Time::now();
  robot_ori_marker.header.seq = 0;
  robot_ori_marker.header.frame_id = world_frame_id;
  robot_ori_marker.id = 0;
  robot_ori_marker.ns = "heading";
  robot_ori_marker.action = visualization_msgs::Marker::ADD;
  robot_ori_marker.type = visualization_msgs::Marker::ARROW;
  robot_ori_marker.color.r = 1.0;
  robot_ori_marker.color.g = 10.0 / 255.0;
  robot_ori_marker.color.b = 10.0 / 255.0;
  robot_ori_marker.color.a = 1.0;
  robot_ori_marker.scale.x = 2.0;
  robot_ori_marker.scale.y = 0.1;
  robot_ori_marker.scale.z = 0.1;
  robot_ori_marker.pose.position.x = state[0];
  robot_ori_marker.pose.position.y = state[1];
  robot_ori_marker.pose.position.z = state[2];
  robot_ori_marker.pose.orientation.x = quat.x();
  robot_ori_marker.pose.orientation.y = quat.y();
  robot_ori_marker.pose.orientation.z = quat.z();
  robot_ori_marker.pose.orientation.w = quat.w();
  robot_ori_marker.lifetime = ros::Duration(robot_lifetime);
  robot_ori_marker.frame_locked = false;
  marker_array.markers.push_back(robot_ori_marker);

  robot_state_pub_.publish(marker_array);
}

void Visualization::visualizeSensorFOV(StateVec& state,
                                       SensorParams& sensor_params) {
  if (sensor_fov_pub_.getNumSubscribers() < 1) return;
  if (sensor_params.sensor_list.size() == 0) return;

  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < sensor_params.sensor_list.size(); ++i) {
    std::string sensor_name = sensor_params.sensor_list[i];
    SensorParamsBase* sb = &(sensor_params.sensor[sensor_name]);
    if (sb->type == SensorType::kCamera) {
      // Visualize its frustum using lines precomputed.
      visualization_msgs::Marker line_marker;
      line_marker.header.stamp = ros::Time::now();
      line_marker.header.seq = 0;
      line_marker.header.frame_id = world_frame_id;
      line_marker.ns = sensor_name;
      line_marker.id = 0;
      line_marker.action = visualization_msgs::Marker::ADD;
      line_marker.type = visualization_msgs::Marker::LINE_LIST;
      line_marker.scale.x = 0.1;
      line_marker.color.r = 0.0;
      line_marker.color.g = 100.0 / 255.0;
      line_marker.color.b = 1.0;
      line_marker.color.a = 1.0;
      line_marker.lifetime = ros::Duration(robot_lifetime);
      line_marker.frame_locked = false;

      // Rotate the edges from body to world coordinate.
      std::vector<Eigen::Vector3d> edge_points_w;
      sb->getFrustumEdges(state, edge_points_w);
      // Add lines
      for (int j = 0; j < 4; ++j) {
        geometry_msgs::Point p1;
        p1.x = state[0];
        p1.y = state[1];
        p1.z = state[2];
        geometry_msgs::Point p2;
        p2.x = edge_points_w[j][0];
        p2.y = edge_points_w[j][1];
        p2.z = edge_points_w[j][2];
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
      }
      for (int j = 0; j < 3; ++j) {
        geometry_msgs::Point p1;
        p1.x = edge_points_w[j][0];
        p1.y = edge_points_w[j][1];
        p1.z = edge_points_w[j][2];
        geometry_msgs::Point p2;
        p2.x = edge_points_w[j + 1][0];
        p2.y = edge_points_w[j + 1][1];
        p2.z = edge_points_w[j + 1][2];
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
      }
      {
        geometry_msgs::Point p1;
        p1.x = edge_points_w[3][0];
        p1.y = edge_points_w[3][1];
        p1.z = edge_points_w[3][2];
        geometry_msgs::Point p2;
        p2.x = edge_points_w[0][0];
        p2.y = edge_points_w[0][1];
        p2.z = edge_points_w[0][2];
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
      }
      marker_array.markers.push_back(line_marker);
    } else if (sb->type == SensorType::kLidar) {
      // Visualize as a cylinder (not correct but for debug purpose).
      visualization_msgs::Marker cyl_marker;
      cyl_marker.header.stamp = ros::Time::now();
      cyl_marker.header.seq = 0;
      cyl_marker.header.frame_id = world_frame_id;
      cyl_marker.ns = sensor_name;
      cyl_marker.action = visualization_msgs::Marker::ADD;
      cyl_marker.type = visualization_msgs::Marker::CYLINDER;
      cyl_marker.scale.x = sb->max_range * 2;  // x-diameter
      cyl_marker.scale.y = sb->max_range * 2;  // y-diameter
      cyl_marker.scale.z = sb->max_range * sin(sb->fov[1] / 2) * 2;  // height
      cyl_marker.pose.position.x = state[0];
      cyl_marker.pose.position.y = state[1];
      cyl_marker.pose.position.z = state[2];
      Eigen::Matrix3d rot_W2B;
      rot_W2B = Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
      Eigen::Matrix3d rot_B2S;  // body to sensor from yaml setting.
      rot_B2S = Eigen::AngleAxisd(sb->rotations[0], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(sb->rotations[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(sb->rotations[2], Eigen::Vector3d::UnitX());
      Eigen::Matrix3d rot_W2S =
          rot_W2B * rot_B2S;  // Convert point from sensor to body
      Eigen::Quaterniond quat(rot_W2S);
      cyl_marker.pose.orientation.x = quat.x();
      cyl_marker.pose.orientation.y = quat.y();
      cyl_marker.pose.orientation.z = quat.z();
      cyl_marker.pose.orientation.w = quat.w();
      cyl_marker.color.r = 0.0;
      cyl_marker.color.g = 100.0 / 255.0;
      cyl_marker.color.b = 1.0;
      cyl_marker.color.a = 0.5;
      cyl_marker.lifetime = ros::Duration(robot_lifetime);
      cyl_marker.frame_locked = false;
      marker_array.markers.push_back(cyl_marker);
    }
  }
  sensor_fov_pub_.publish(marker_array);
}

void Visualization::visualizeNegativePaths(
    const std::vector<int>& ids,
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep) {
  std::shared_ptr<Graph> g = graph_manager->graph_;
  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  if (graph_manager->getNumVertices() == 0) return;
  // if (shortest_paths_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;
  // Plot all egdes.
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "negative_edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.2;
  edge_marker.color.r = 1.0;
  edge_marker.color.g = 0.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  for (int id : ids) {
    int parent_id = graph_manager->getParentIDFromShortestPath(id, graph_rep);
    while (parent_id != 0) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map[parent_id]->state[0];
      p2.y = v_map[parent_id]->state[1];
      p2.z = v_map[parent_id]->state[2];
      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
      id = parent_id;
      parent_id =
          graph_manager->getParentIDFromShortestPath(parent_id, graph_rep);
    }
  }
  marker_array.markers.push_back(edge_marker);

  negative_edges_pub_.publish(marker_array);
}

void Visualization::visualizeNegativePaths(
    const std::vector<Eigen::Vector3d>& edge_vertices,
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep) {
  std::shared_ptr<Graph> g = graph_manager->graph_;

  if (graph_manager->getNumVertices() == 0) return;
  // if (shortest_paths_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;
  // Plot all egdes.
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "negative_edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.2;
  edge_marker.color.r = 1.0;
  edge_marker.color.g = 0.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  for (int i = 0; i < edge_vertices.size(); i += 2) {
    geometry_msgs::Point p1;
    p1.x = edge_vertices[i][0];
    p1.y = edge_vertices[i][1];
    p1.z = edge_vertices[i][2];
    geometry_msgs::Point p2;
    p2.x = edge_vertices[i + 1][0];
    p2.y = edge_vertices[i + 1][1];
    p2.z = edge_vertices[i + 1][2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  negative_edges_pub_.publish(marker_array);
}

void Visualization::visualizeShortestPaths(
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep) {
  std::shared_ptr<Graph> g = graph_manager->graph_;
  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  if (graph_manager->getNumVertices() == 0) return;
  if (shortest_paths_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;
  int num_vertices = graph_manager->getNumVertices();

  // Plot all egdes.
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "shortest_edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.1;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 0.0;
  edge_marker.color.b = 0.9;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  for (int id = 0; id < num_vertices; ++id) {
    int parent_id = graph_manager->getParentIDFromShortestPath(id, graph_rep);
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    geometry_msgs::Point p2;
    p2.x = v_map[parent_id]->state[0];
    p2.y = v_map[parent_id]->state[1];
    p2.z = v_map[parent_id]->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices.
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.color.r = 200.0 / 255.0;
  vertex_marker.color.g = 100.0 / 255.0;
  vertex_marker.color.b = 0.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  vertex_marker.frame_locked = false;

  for (int id = 0; id < num_vertices; ++id) {
    geometry_msgs::Point p1;
    p1.x = v_map[id]->state[0];
    p1.y = v_map[id]->state[1];
    p1.z = v_map[id]->state[2];
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  // Plot all leave.
  visualization_msgs::Marker leaf_vertex_marker;
  leaf_vertex_marker.header.stamp = ros::Time::now();
  leaf_vertex_marker.header.seq = 0;
  leaf_vertex_marker.header.frame_id = world_frame_id;
  leaf_vertex_marker.id = 0;
  leaf_vertex_marker.ns = "leaf_vertices";
  leaf_vertex_marker.action = visualization_msgs::Marker::ADD;
  leaf_vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  leaf_vertex_marker.scale.x = 0.4;
  leaf_vertex_marker.scale.y = 0.4;
  leaf_vertex_marker.scale.z = 0.4;
  leaf_vertex_marker.color.r = 200.0 / 255.0;
  leaf_vertex_marker.color.g = 50 / 255.0;
  leaf_vertex_marker.color.b = 0.0;
  leaf_vertex_marker.color.a = 1.0;
  leaf_vertex_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  leaf_vertex_marker.frame_locked = false;
  std::vector<Vertex*> leaf_vertices;
  for (int id = 0; id < num_vertices; ++id) {
    if (v_map[id]->is_leaf_vertex) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      leaf_vertex_marker.points.push_back(p1);
      leaf_vertices.push_back(v_map[id]);
    }
  }
  marker_array.markers.push_back(leaf_vertex_marker);

  // Plot all headings.
  int marker_id = 0;
  for (int id = 0; id < num_vertices; ++id) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = world_frame_id;
    marker.ns = "heading";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.5;   // length of the arrow
    marker.scale.y = 0.15;  // arrow width
    marker.scale.z = 0.15;  // arrow height
    marker.color.r = 200.0 / 255.0;
    marker.color.g = 50.0 / 255.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(shortest_paths_lifetime);
    marker.frame_locked = false;
    marker.pose.position.x = v_map[id]->state[0];
    marker.pose.position.y = v_map[id]->state[1];
    marker.pose.position.z = v_map[id]->state[2];
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, v_map[id]->state[3]);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);
  }

  // Plot all gains
  marker_id = 0;
  for (int id = 0; id < num_vertices; ++id) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = world_frame_id;
    marker.ns = "gain";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 0.15;  // text height
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(shortest_paths_lifetime);
    marker.frame_locked = false;
    marker.pose.position.x = v_map[id]->state[0];
    marker.pose.position.y = v_map[id]->state[1];
    marker.pose.position.z = v_map[id]->state[2] + 0.1;
    // Show vertex gains.
    std::string text_display =
        std::to_string(v_map[id]->id) + "," + std::to_string(v_map[id]->dm) +
        "," + std::to_string(v_map[id]->vol_gain.accumulative_gain) + "," +
        std::to_string(v_map[id]->vol_gain.gain) + "," +
        std::to_string(v_map[id]->vol_gain.num_unknown_voxels) + "," +
        std::to_string(v_map[id]->vol_gain.num_free_voxels) + "," +
        std::to_string(v_map[id]->vol_gain.num_occupied_voxels);

    marker.text = text_display;
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);
  }

  // Sort the leaf vertices.
  // Then visualize top 10.
  std::sort(leaf_vertices.begin(), leaf_vertices.end(),
            [](const Vertex* a, const Vertex* b) {
              return a->vol_gain.num_unknown_voxels >
                     b->vol_gain.num_unknown_voxels;
            });
  visualization_msgs::Marker best_leaf_marker;
  best_leaf_marker.header.stamp = ros::Time::now();
  best_leaf_marker.header.seq = 0;
  best_leaf_marker.header.frame_id = world_frame_id;
  best_leaf_marker.id = 0;
  best_leaf_marker.ns = "best_leaf_vertices";
  best_leaf_marker.action = visualization_msgs::Marker::ADD;
  best_leaf_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  best_leaf_marker.scale.x = 0.4;
  best_leaf_marker.scale.y = 0.4;
  best_leaf_marker.scale.z = 0.4;
  best_leaf_marker.color.r = 0.0;
  best_leaf_marker.color.g = 1.0;
  best_leaf_marker.color.b = 0.0;
  best_leaf_marker.color.a = 1.0;
  best_leaf_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  best_leaf_marker.frame_locked = false;
  if (leaf_vertices.size() > 10) {
    for (int ind = 0; ind < 10; ++ind) {
      geometry_msgs::Point p1;
      p1.x = leaf_vertices[ind]->state[0];
      p1.y = leaf_vertices[ind]->state[1];
      p1.z = leaf_vertices[ind]->state[2];
      best_leaf_marker.points.push_back(p1);
    }
    marker_array.markers.push_back(best_leaf_marker);
  }

  // Mark all potential frontiers.
  visualization_msgs::Marker frontier_marker;
  frontier_marker.header.stamp = ros::Time::now();
  frontier_marker.header.seq = 0;
  frontier_marker.header.frame_id = world_frame_id;
  frontier_marker.id = 0;
  frontier_marker.ns = "frontiers";
  frontier_marker.action = visualization_msgs::Marker::ADD;
  frontier_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  frontier_marker.scale.x = 0.5;
  frontier_marker.scale.y = 0.5;
  frontier_marker.scale.z = 0.5;
  frontier_marker.color.r = 1.0;
  frontier_marker.color.g = 0.0;
  frontier_marker.color.b = 0.0;
  frontier_marker.color.a = 1.0;
  frontier_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  frontier_marker.frame_locked = false;
  num_vertices = graph_manager->getNumVertices();
  for (int id = 0; id < num_vertices; ++id) {
    if (v_map[id]->type == VertexType::kFrontier) {
      geometry_msgs::Point p1;
      p1.x = v_map[id]->state[0];
      p1.y = v_map[id]->state[1];
      p1.z = v_map[id]->state[2];
      frontier_marker.points.push_back(p1);
    }
  }
  marker_array.markers.push_back(frontier_marker);

  shortest_paths_pub_.publish(marker_array);
}

void Visualization::visualizeClusteredPaths(
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep, const std::vector<Vertex*>& vertices,
    const std::vector<int>& cluster_ids) {
  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  if (graph_manager->getNumVertices() == 0) return;
  if (clustered_paths_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;
  for (int cind = 0; cind < cluster_ids.size(); ++cind) {
    // Visualize one cluster.
    // Plot all egdes.
    visualization_msgs::Marker edge_marker;
    edge_marker.header.stamp = ros::Time::now();
    edge_marker.header.seq = 0;
    edge_marker.header.frame_id = world_frame_id;
    edge_marker.id = 0;
    edge_marker.ns = "cluster" + std::to_string(cind);
    edge_marker.action = visualization_msgs::Marker::ADD;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.scale.x = 0.1;
    float r, g, b;
    getHeatMapColor((float)cind / (float)cluster_ids.size(), r, g, b);
    edge_marker.color.r = r;
    edge_marker.color.g = g;
    edge_marker.color.b = b;
    edge_marker.color.a = 1.0;
    edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
    edge_marker.frame_locked = false;

    for (int cl = 0; cl < vertices.size(); ++cl) {
      if (vertices[cl]->cluster_id == cluster_ids[cind]) {
        std::vector<int> id_best_path;
        graph_manager->getShortestPath(vertices[cl]->id, graph_rep, false,
                                       id_best_path);
        if (id_best_path.size() > 1) {
          for (int i = 0; i < (id_best_path.size() - 1); ++i) {
            geometry_msgs::Point p1;
            p1.x = v_map[id_best_path[i]]->state[0];
            p1.y = v_map[id_best_path[i]]->state[1];
            p1.z = v_map[id_best_path[i]]->state[2];
            geometry_msgs::Point p2;
            p2.x = v_map[id_best_path[i + 1]]->state[0];
            p2.y = v_map[id_best_path[i + 1]]->state[1];
            p2.z = v_map[id_best_path[i + 1]]->state[2];
            edge_marker.points.push_back(p1);
            edge_marker.points.push_back(p2);
          }
        }
        marker_array.markers.push_back(edge_marker);
      }
    }
  }

  clustered_paths_pub_.publish(marker_array);
}

bool Visualization::getHeatMapColor(float value, float& red, float& green,
                                    float& blue) {
  // const int NUM_COLORS = 9;
  // static float color[NUM_COLORS][3] = {
  //     {255, 255, 204}, {255, 237, 160}, {254, 217, 118},
  //     {254, 178, 76},  {253, 141, 60},  {252, 78, 42},
  //     {227, 26, 28},   {189, 0, 38},    {128, 0, 38}};  // 0-255 scale

  const int NUM_COLORS = 6;
  static float color[NUM_COLORS][3] = {
      {254, 178, 76}, {253, 141, 60}, {252, 78, 42},
      {227, 26, 28},  {189, 0, 38},   {128, 0, 38}};  // 0-255 scale

  // const int NUM_COLORS = 4;
  // static float color[NUM_COLORS][3] = {
  //   /*{0,0,1}, // blue*/
  //   {0,1,1}, // cyan
  //   {0,1,0}, //green
  //   {1,1,0}, // yellow
  //   {1,0,0}  //red
  // };

  int idx1;  // |-- Our desired color will be between these two indexes in
             // "color".
  int idx2;  // |
  float fractBetween =
      0;  // Fraction between "idx1" and "idx2" where our value is.

  if (value <= 0)
    idx1 = idx2 = 0;  // accounts for an input <=0
  else if (value >= 1)
    idx1 = idx2 = NUM_COLORS - 1;  // accounts for an input >=0
  else {
    value = value * (NUM_COLORS - 1);  // Will multiply value by 3.
    idx1 = floor(value);  // Our desired color will be after this index.
    idx2 = idx1 + 1;      // ... and before this index (inclusive).
    fractBetween =
        value - (float)idx1;  // Distance between the two indexes (0-1).
  }
  red = (color[idx2][0] - color[idx1][0]) * fractBetween + color[idx1][0];
  green = (color[idx2][1] - color[idx1][1]) * fractBetween + color[idx1][1];
  blue = (color[idx2][2] - color[idx1][2]) * fractBetween + color[idx1][2];
  red /= 255.0;
  green /= 255.0;
  blue /= 255.0;
  return true;
}

void Visualization::visualizeHomingPath(
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep, int current_id) {
  if (planning_homing_pub_.getNumSubscribers() < 1) return;

  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  visualization_msgs::MarkerArray marker_array;

  // Plot the best one first.
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "best_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.4;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 1.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  std::vector<int> id_best_path;
  graph_manager->getShortestPath(current_id, graph_rep, false, id_best_path);
  if (id_best_path.size() > 1) {
    for (int i = 0; i < (id_best_path.size() - 1); ++i) {
      geometry_msgs::Point p1;
      p1.x = v_map[id_best_path[i]]->state[0];
      p1.y = v_map[id_best_path[i]]->state[1];
      p1.z = v_map[id_best_path[i]]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map[id_best_path[i + 1]]->state[0];
      p2.y = v_map[id_best_path[i + 1]]->state[1];
      p2.z = v_map[id_best_path[i + 1]]->state[2];
      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(edge_marker);
  planning_homing_pub_.publish(marker_array);
}

void Visualization::visualizeGlobalPaths(
    const std::shared_ptr<GraphManager> graph_manager,
    std::vector<int>& to_frontier_ids, std::vector<int>& to_home_ids) {
  if (planning_global_pub_.getNumSubscribers() < 1) return;

  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  visualization_msgs::MarkerArray marker_array;

  // Plot the current to frontier path first.
  visualization_msgs::Marker frontier_marker;
  frontier_marker.header.stamp = ros::Time::now();
  frontier_marker.header.seq = 0;
  frontier_marker.header.frame_id = world_frame_id;
  frontier_marker.id = 0;
  frontier_marker.ns = "current2frontier";
  frontier_marker.action = visualization_msgs::Marker::ADD;
  frontier_marker.type = visualization_msgs::Marker::LINE_LIST;
  frontier_marker.scale.x = 0.4;
  frontier_marker.color.r = 0.0;
  frontier_marker.color.g = 1.0;
  frontier_marker.color.b = 0.0;
  frontier_marker.color.a = 1.0;
  frontier_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  frontier_marker.frame_locked = false;

  if (to_frontier_ids.size() > 1) {
    for (int i = 0; i < (to_frontier_ids.size() - 1); ++i) {
      geometry_msgs::Point p1;
      p1.x = v_map[to_frontier_ids[i]]->state[0];
      p1.y = v_map[to_frontier_ids[i]]->state[1];
      p1.z = v_map[to_frontier_ids[i]]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map[to_frontier_ids[i + 1]]->state[0];
      p2.y = v_map[to_frontier_ids[i + 1]]->state[1];
      p2.z = v_map[to_frontier_ids[i + 1]]->state[2];
      frontier_marker.points.push_back(p1);
      frontier_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(frontier_marker);

  // Plot the current to frontier path first.
  visualization_msgs::Marker home_marker;
  home_marker.header.stamp = ros::Time::now();
  home_marker.header.seq = 0;
  home_marker.header.frame_id = world_frame_id;
  home_marker.id = 0;
  home_marker.ns = "frontier2home";
  home_marker.action = visualization_msgs::Marker::ADD;
  home_marker.type = visualization_msgs::Marker::LINE_LIST;
  home_marker.scale.x = 0.4;
  home_marker.color.r = 0.5;
  home_marker.color.g = 0.0;
  home_marker.color.b = 0.0;
  home_marker.color.a = 1.0;
  home_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  home_marker.frame_locked = false;

  if (to_home_ids.size() > 1) {
    for (int i = 0; i < (to_home_ids.size() - 1); ++i) {
      geometry_msgs::Point p1;
      p1.x = v_map[to_home_ids[i]]->state[0];
      p1.y = v_map[to_home_ids[i]]->state[1];
      p1.z = v_map[to_home_ids[i]]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map[to_home_ids[i + 1]]->state[0];
      p2.y = v_map[to_home_ids[i + 1]]->state[1];
      p2.z = v_map[to_home_ids[i + 1]]->state[2];
      home_marker.points.push_back(p1);
      home_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(home_marker);
  planning_global_pub_.publish(marker_array);
}

void Visualization::visualizePath(
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep, int vertex_id) {
  if (path_pub_.getNumSubscribers() < 1) return;

  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  visualization_msgs::MarkerArray marker_array;
  // Plot the best one first.
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "best_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.4;
  edge_marker.color.r = 244.0 / 255.0;
  edge_marker.color.g = 66.0 / 255.0;
  edge_marker.color.b = 226.0 / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  std::vector<int> id_best_path;
  graph_manager->getShortestPath(vertex_id, graph_rep, false, id_best_path);
  if (id_best_path.size() > 1) {
    for (int i = 0; i < (id_best_path.size() - 1); ++i) {
      geometry_msgs::Point p1;
      p1.x = v_map[id_best_path[i]]->state[0];
      p1.y = v_map[id_best_path[i]]->state[1];
      p1.z = v_map[id_best_path[i]]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map[id_best_path[i + 1]]->state[0];
      p2.y = v_map[id_best_path[i + 1]]->state[1];
      p2.z = v_map[id_best_path[i + 1]]->state[2];
      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices.
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.5;
  vertex_marker.scale.y = 0.5;
  vertex_marker.scale.z = 0.5;
  vertex_marker.color.r = 200.0 / 255.0;
  vertex_marker.color.g = 100.0 / 255.0;
  vertex_marker.color.b = 0.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < id_best_path.size(); ++i) {
    geometry_msgs::Point p1;
    p1.x = v_map[id_best_path[i]]->state[0];
    p1.y = v_map[id_best_path[i]]->state[1];
    p1.z = v_map[id_best_path[i]]->state[2];
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  path_pub_.publish(marker_array);
}

void Visualization::visualizeBestPaths(
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep, int n, int best_vertex_id) {
  if (best_planning_path_pub_.getNumSubscribers() < 1) return;

  std::unordered_map<int, Vertex*>& v_map = graph_manager->vertices_map_;

  visualization_msgs::MarkerArray marker_array;

  // Plot the best one first.
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "best_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.20;
  edge_marker.color.r = 11.0 / 255.0;
  edge_marker.color.g = 114.0 / 255.0;
  edge_marker.color.b = 27.0 / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  std::vector<int> id_best_path;
  graph_manager->getShortestPath(best_vertex_id, graph_rep, false,
                                 id_best_path);
  if (id_best_path.size() > 1) {
    for (int i = 0; i < (id_best_path.size() - 1); ++i) {
      geometry_msgs::Point p1;
      p1.x = v_map[id_best_path[i]]->state[0];
      p1.y = v_map[id_best_path[i]]->state[1];
      p1.z = v_map[id_best_path[i]]->state[2];
      geometry_msgs::Point p2;
      p2.x = v_map[id_best_path[i + 1]]->state[0];
      p2.y = v_map[id_best_path[i + 1]]->state[1];
      p2.z = v_map[id_best_path[i + 1]]->state[2];
      edge_marker.points.push_back(p1);
      edge_marker.points.push_back(p2);
    }
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices.
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.25;
  vertex_marker.scale.y = 0.25;
  vertex_marker.scale.z = 0.25;
  vertex_marker.color.r = 200.0 / 255.0;
  vertex_marker.color.g = 100.0 / 255.0;
  vertex_marker.color.b = 0.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < id_best_path.size(); ++i) {
    geometry_msgs::Point p1;
    p1.x = v_map[id_best_path[i]]->state[0];
    p1.y = v_map[id_best_path[i]]->state[1];
    p1.z = v_map[id_best_path[i]]->state[2];
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  // // Plot its headings.
  // int marker_id = 0;
  // int id;
  // for (int i = 0; i < id_best_path.size(); ++i) {
  //   id = id_best_path[i];
  //   visualization_msgs::Marker marker;
  //   marker.header.stamp = ros::Time::now();
  //   marker.header.seq = 0;
  //   marker.header.frame_id = world_frame_id;
  //   marker.ns = "heading";
  //   marker.action = visualization_msgs::Marker::ADD;
  //   marker.type = visualization_msgs::Marker::ARROW;
  //   marker.scale.x = 0.6;   // length of the arrow
  //   marker.scale.y = 0.15;  // arrow width
  //   marker.scale.z = 0.15;  // arrow height
  //   marker.color.r = 200.0 / 255.0;
  //   marker.color.g = 50.0 / 255.0;
  //   marker.color.b = 0.0;
  //   marker.color.a = 1.0;
  //   marker.lifetime = ros::Duration(shortest_paths_lifetime);
  //   marker.frame_locked = false;
  //   marker.pose.position.x = v_map[id]->state[0];
  //   marker.pose.position.y = v_map[id]->state[1];
  //   marker.pose.position.z = v_map[id]->state[2];
  //   tf::Quaternion quat;
  //   quat.setRPY(0.0, 0.0, v_map[id]->state[3]);
  //   marker.pose.orientation.x = quat.x();
  //   marker.pose.orientation.y = quat.y();
  //   marker.pose.orientation.z = quat.z();
  //   marker.pose.orientation.w = quat.w();
  //   marker.id = marker_id++;
  //   marker_array.markers.push_back(marker);
  // }

  // Plot the set of best paths in terms of accumulative gain.
  // std::vector<Vertex*> all_leaf_vertices;
  // for (auto &vm: v_map) {
  //   if (vm.second->is_leaf_vertex) all_leaf_vertices.push_back(vm.second);
  // }
  // std::sort(all_leaf_vertices.begin(), all_leaf_vertices.end(), [](const
  // Vertex *a, const Vertex *b) {
  //   return a->vol_gain.accumulative_gain > b->vol_gain.accumulative_gain;
  // });

  // visualization_msgs::Marker nextbest_edge_marker;
  // nextbest_edge_marker.header.stamp = ros::Time::now();
  // nextbest_edge_marker.header.seq = 0;
  // nextbest_edge_marker.header.frame_id = world_frame_id;
  // nextbest_edge_marker.id = 0;
  // nextbest_edge_marker.ns = "next_best_paths";
  // nextbest_edge_marker.action = visualization_msgs::Marker::ADD;
  // nextbest_edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  // nextbest_edge_marker.scale.x = 0.1;
  // nextbest_edge_marker.color.g = 0.0;
  // nextbest_edge_marker.color.b = 150.0 / 255.0;
  // nextbest_edge_marker.color.a = 1.0;
  // nextbest_edge_marker.color.a = 1.0;
  // nextbest_edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  // nextbest_edge_marker.frame_locked = false;

  // int NBestPaths = n-1;
  // if (NBestPaths < 0) NBestPaths = 0;
  // int n_visualize = (NBestPaths <= all_leaf_vertices.size()) ? (NBestPaths) :
  // all_leaf_vertices.size(); for (int i = 0; i < n_visualize; ++i) {
  //   std::vector<int> id_list;
  //   graph_manager->getShortestPath(all_leaf_vertices[i]->id, graph_rep,
  //   false, id_list); if (id_list.size() > 1) {
  //     for (int j=0; j < (id_list.size()-1); ++j) {
  //       geometry_msgs::Point p1;
  //       p1.x = v_map[id_list[j]]->state[0];
  //       p1.y = v_map[id_list[j]]->state[1];
  //       p1.z = v_map[id_list[j]]->state[2];
  //       geometry_msgs::Point p2;
  //       p2.x = v_map[id_list[j+1]]->state[0];
  //       p2.y = v_map[id_list[j+1]]->state[1];
  //       p2.z = v_map[id_list[j+1]]->state[2];
  //       nextbest_edge_marker.points.push_back(p1);
  //       nextbest_edge_marker.points.push_back(p2);
  //     }
  //   }
  // }
  // marker_array.markers.push_back(nextbest_edge_marker);
  best_planning_path_pub_.publish(marker_array);
}

void Visualization::visualizeRefPath(
    const std::vector<geometry_msgs::Pose>& path) {
  if (ref_path_pub_.getNumSubscribers() < 1) return;

  if (path.empty()) return;

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "ref_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.25;
  edge_marker.color.r = 244.0 / 255.0;
  edge_marker.color.g = 66.0 / 255.0;
  edge_marker.color.b = 226.0 / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    geometry_msgs::Point p2;
    p2.x = path[i + 1].position.x;
    p2.y = path[i + 1].position.y;
    p2.z = path[i + 1].position.z;
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices.
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.25;
  vertex_marker.scale.y = 0.25;
  vertex_marker.scale.z = 0.25;
  vertex_marker.color.r = 200.0 / 255.0;
  vertex_marker.color.g = 100.0 / 255.0;
  vertex_marker.color.b = 0.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  ref_path_pub_.publish(marker_array);
}

void Visualization::visualizeRefPath(
    const std::vector<geometry_msgs::Pose>& path, int color) {
  if (ref_path_color_pub_.getNumSubscribers() < 1) return;

  if (path.empty()) return;

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "ref_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.25;
  edge_marker.color.r = (color == 0 ? 255.0 : 0.0) / 255.0;
  edge_marker.color.g = (color == 1 ? 255.0 : 0.0) / 255.0;
  edge_marker.color.b = (color == 2 ? 255.0 : 0.0) / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    geometry_msgs::Point p2;
    p2.x = path[i + 1].position.x;
    p2.y = path[i + 1].position.y;
    p2.z = path[i + 1].position.z;
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices.
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.25;
  vertex_marker.scale.y = 0.25;
  vertex_marker.scale.z = 0.25;
  vertex_marker.color.r = 200.0 / 255.0;
  vertex_marker.color.g = 100.0 / 255.0;
  vertex_marker.color.b = 0.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);

  ref_path_color_pub_.publish(marker_array);
}

void Visualization::visualizeVolumetricGain(
    Eigen::Vector3d& bound_min, Eigen::Vector3d& bound_max,
    std::vector<std::pair<Eigen::Vector3d, MapManager::VoxelStatus>>& voxels,
    double voxel_size) {
  if (volumetric_gain_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot bounding area.
  visualization_msgs::Marker bound_marker;
  bound_marker.header.stamp = ros::Time::now();
  bound_marker.header.seq = 0;
  bound_marker.header.frame_id = world_frame_id;
  bound_marker.id = 0;
  bound_marker.ns = "bound";
  bound_marker.action = visualization_msgs::Marker::ADD;
  bound_marker.type = visualization_msgs::Marker::CUBE;
  bound_marker.pose.position.x = 0.5 * (bound_min[0] + bound_max[0]);
  bound_marker.pose.position.y = 0.5 * (bound_min[1] + bound_max[1]);
  bound_marker.pose.position.z = 0.5 * (bound_min[2] + bound_max[2]);
  bound_marker.scale.x = bound_max[0] - bound_min[0];
  bound_marker.scale.y = bound_max[1] - bound_min[1];
  bound_marker.scale.z = bound_max[2] - bound_min[2];
  tf::Quaternion quat;
  quat.setEuler(0, 0, 0);
  bound_marker.pose.orientation.x = quat.x();
  bound_marker.pose.orientation.y = quat.y();
  bound_marker.pose.orientation.z = quat.z();
  bound_marker.pose.orientation.w = quat.w();
  bound_marker.color.r = 200.0 / 255.0;
  bound_marker.color.g = 100.0 / 255.0;
  bound_marker.color.b = 0.0;
  bound_marker.color.a = 0.25;
  bound_marker.lifetime = ros::Duration(ws_lifetime);
  bound_marker.frame_locked = false;
  marker_array.markers.push_back(bound_marker);

  // Plot unknow voxels.
  visualization_msgs::Marker unknown_voxel_marker;
  unknown_voxel_marker.header.stamp = ros::Time::now();
  unknown_voxel_marker.header.seq = 0;
  unknown_voxel_marker.header.frame_id = world_frame_id;
  unknown_voxel_marker.id = 0;
  unknown_voxel_marker.ns = "unknown_voxels";
  unknown_voxel_marker.action = visualization_msgs::Marker::ADD;
  unknown_voxel_marker.type = visualization_msgs::Marker::CUBE_LIST;
  unknown_voxel_marker.scale.x = voxel_size;
  unknown_voxel_marker.scale.y = voxel_size;
  unknown_voxel_marker.scale.z = voxel_size;
  unknown_voxel_marker.color.r = 1.0;
  unknown_voxel_marker.color.g = 0.0;
  unknown_voxel_marker.color.b = 0.0;
  unknown_voxel_marker.color.a = 0.8;
  unknown_voxel_marker.lifetime = ros::Duration(graph_lifetime);
  unknown_voxel_marker.frame_locked = false;

  visualization_msgs::Marker free_voxel_marker;
  free_voxel_marker.header.stamp = ros::Time::now();
  free_voxel_marker.header.seq = 0;
  free_voxel_marker.header.frame_id = world_frame_id;
  free_voxel_marker.id = 0;
  free_voxel_marker.ns = "free_voxels";
  free_voxel_marker.action = visualization_msgs::Marker::ADD;
  free_voxel_marker.type = visualization_msgs::Marker::CUBE_LIST;
  free_voxel_marker.scale.x = voxel_size;
  free_voxel_marker.scale.y = voxel_size;
  free_voxel_marker.scale.z = voxel_size;
  free_voxel_marker.color.r = 0.0;
  free_voxel_marker.color.g = 1.0;
  free_voxel_marker.color.b = 0.0;
  free_voxel_marker.color.a = 0.8;
  free_voxel_marker.lifetime = ros::Duration(graph_lifetime);
  free_voxel_marker.frame_locked = false;

  visualization_msgs::Marker occupied_voxel_marker;
  occupied_voxel_marker.header.stamp = ros::Time::now();
  occupied_voxel_marker.header.seq = 0;
  occupied_voxel_marker.header.frame_id = world_frame_id;
  occupied_voxel_marker.id = 0;
  occupied_voxel_marker.ns = "occupied_voxels";
  occupied_voxel_marker.action = visualization_msgs::Marker::ADD;
  occupied_voxel_marker.type = visualization_msgs::Marker::CUBE_LIST;
  occupied_voxel_marker.scale.x = voxel_size;
  occupied_voxel_marker.scale.y = voxel_size;
  occupied_voxel_marker.scale.z = voxel_size;
  occupied_voxel_marker.color.r = 0.0;
  occupied_voxel_marker.color.g = 0.0;
  occupied_voxel_marker.color.b = 1.0;
  occupied_voxel_marker.color.a = 0.4;
  occupied_voxel_marker.lifetime = ros::Duration(graph_lifetime);
  occupied_voxel_marker.frame_locked = false;

  for (auto& v : voxels) {
    geometry_msgs::Point p;
    p.x = v.first[0];
    p.y = v.first[1];
    p.z = v.first[2];
    if (v.second == MapManager::VoxelStatus::kUnknown) {
      unknown_voxel_marker.points.push_back(p);
    } else if (v.second == MapManager::VoxelStatus::kFree) {
      free_voxel_marker.points.push_back(p);
    } else if (v.second == MapManager::VoxelStatus::kOccupied) {
      occupied_voxel_marker.points.push_back(p);
    } else {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Unsupported voxel type.");
    }
  }
  marker_array.markers.push_back(unknown_voxel_marker);
  marker_array.markers.push_back(free_voxel_marker);
  marker_array.markers.push_back(occupied_voxel_marker);

  volumetric_gain_pub_.publish(marker_array);
}

void Visualization::visualizeSampler(RandomSampler& random_sampler) {
  if (sampler_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all vertices.
  visualization_msgs::Marker samples_marker;
  samples_marker.header.stamp = ros::Time::now();
  samples_marker.header.seq = 0;
  samples_marker.header.frame_id = world_frame_id;
  samples_marker.id = 0;
  samples_marker.ns = "valid_samples";
  samples_marker.action = visualization_msgs::Marker::ADD;
  samples_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  samples_marker.scale.x = 0.2;
  samples_marker.scale.y = 0.2;
  samples_marker.scale.z = 0.2;
  samples_marker.color.r = 0.0;
  samples_marker.color.g = 255.0 / 255.0;
  samples_marker.color.b = 0.0;
  samples_marker.color.a = 1.0;
  samples_marker.lifetime = ros::Duration(sampler_lifetime);
  samples_marker.frame_locked = false;

  for (auto& p : *(random_sampler.getSamples(true))) {
    geometry_msgs::Point p1;
    p1.x = p.x();
    p1.y = p.y();
    p1.z = p.z();
    samples_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(samples_marker);

  visualization_msgs::Marker invalid_samples_marker;
  invalid_samples_marker.header.stamp = ros::Time::now();
  invalid_samples_marker.header.seq = 0;
  invalid_samples_marker.header.frame_id = world_frame_id;
  invalid_samples_marker.id = 0;
  invalid_samples_marker.ns = "invalid_samples";
  invalid_samples_marker.action = visualization_msgs::Marker::ADD;
  invalid_samples_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  invalid_samples_marker.scale.x = 0.2;
  invalid_samples_marker.scale.y = 0.2;
  invalid_samples_marker.scale.z = 0.2;
  invalid_samples_marker.color.r = 244.0 / 255.0;
  invalid_samples_marker.color.g = 229.0 / 255.0;
  invalid_samples_marker.color.b = 66.0 / 255.0;
  invalid_samples_marker.color.a = 1.0;
  invalid_samples_marker.lifetime = ros::Duration(sampler_lifetime);
  invalid_samples_marker.frame_locked = false;

  for (auto& p : *(random_sampler.getSamples(false))) {
    geometry_msgs::Point p1;
    p1.x = p.x();
    p1.y = p.y();
    p1.z = p.z();
    invalid_samples_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(invalid_samples_marker);

  sampler_pub_.publish(marker_array);
}

void Visualization::visualizeRays(
    const StateVec state, const std::vector<Eigen::Vector3d> ray_endpoints) {
  if (rays_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all rays.
  visualization_msgs::Marker ray_marker;
  ray_marker.header.stamp = ros::Time::now();
  ray_marker.header.seq = 0;
  ray_marker.header.frame_id = world_frame_id;
  ray_marker.id = 0;
  ray_marker.ns = "rays";
  ray_marker.action = visualization_msgs::Marker::ADD;
  ray_marker.type = visualization_msgs::Marker::LINE_LIST;
  ray_marker.scale.x = 0.2;
  ray_marker.color.r = 100.0 / 255.0;
  ray_marker.color.g = 0.0;
  ray_marker.color.b = 0.0;
  ray_marker.color.a = 0.8;
  ray_marker.lifetime = ros::Duration(ray_lifetime);
  ray_marker.frame_locked = false;

  geometry_msgs::Point p0;
  p0.x = state[0];
  p0.y = state[1];
  p0.z = state[2];
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Visualize: %d rays", (int)ray_endpoints.size());
  for (auto& ray : ray_endpoints) {
    geometry_msgs::Point p1;
    p1.x = ray[0];
    p1.y = ray[1];
    p1.z = ray[2];
    ray_marker.points.push_back(p0);
    ray_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(ray_marker);
  rays_pub_.publish(marker_array);
}

void Visualization::visualizeRobotStateHistory(
    const std::vector<StateVec*> state_hist) {
  if (state_history_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  // Plot all vertices.
  visualization_msgs::Marker state_marker;
  state_marker.header.stamp = ros::Time::now();
  state_marker.header.seq = 0;
  state_marker.header.frame_id = world_frame_id;
  state_marker.id = 0;
  state_marker.ns = "state";
  state_marker.action = visualization_msgs::Marker::ADD;
  state_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  state_marker.scale.x = 0.2;
  state_marker.scale.y = 0.2;
  state_marker.scale.z = 0.2;
  state_marker.color.r = 0.0;
  state_marker.color.g = 1.0;
  state_marker.color.b = 0.0;
  state_marker.color.a = 1.0;
  state_marker.lifetime = ros::Duration(sampler_lifetime);
  state_marker.frame_locked = false;

  for (auto& p : state_hist) {
    geometry_msgs::Point p1;
    p1.x = p->x();
    p1.y = p->y();
    p1.z = p->z();
    state_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(state_marker);

  // Plot bubble around it.
  visualization_msgs::Marker state_range_marker;
  state_range_marker.header.stamp = ros::Time::now();
  state_range_marker.header.seq = 0;
  state_range_marker.header.frame_id = world_frame_id;
  state_range_marker.id = 0;
  state_range_marker.ns = "state_range";
  state_range_marker.action = visualization_msgs::Marker::ADD;
  state_range_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  state_range_marker.scale.x = 6.0;
  state_range_marker.scale.y = 6.0;
  state_range_marker.scale.z = 6.0;
  state_range_marker.color.r = 0.0;
  state_range_marker.color.g = 1.0;
  state_range_marker.color.b = 0.0;
  state_range_marker.color.a = 0.5;
  state_range_marker.lifetime = ros::Duration(sampler_lifetime);
  state_range_marker.frame_locked = false;

  for (auto& p : state_hist) {
    geometry_msgs::Point p1;
    p1.x = p->x();
    p1.y = p->y();
    p1.z = p->z();
    state_range_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(state_range_marker);

  state_history_pub_.publish(marker_array);
}

void Visualization::visualizeGeofence(
    const std::shared_ptr<GeofenceManager> geofence_manager) {
  if (geofence_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;
  std::vector<GeofenceArea> geofence_list;
  geofence_manager->getAllGeofenceAreas(geofence_list);
  int marker_id = 0;
  const double kZFixed = 1.0;
  for (auto& geof : geofence_list) {
    visualization_msgs::Marker obs_marker;
    obs_marker.header.stamp = ros::Time::now();
    obs_marker.header.seq = 0;
    obs_marker.header.frame_id = world_frame_id;
    obs_marker.ns = "geofence";
    obs_marker.id = marker_id++;
    obs_marker.action = visualization_msgs::Marker::ADD;
    obs_marker.type = visualization_msgs::Marker::LINE_STRIP;
    obs_marker.scale.x = 0.15;
    obs_marker.color.r = 245.0 / 255;
    obs_marker.color.g = 20.0 / 255;
    obs_marker.color.b = 12.0 / 255;
    obs_marker.color.a = 1.0;
    obs_marker.lifetime = ros::Duration(sampler_lifetime);
    obs_marker.frame_locked = false;

    std::vector<Eigen::Vector2d> nodes;
    geof.polygon.getVertices(nodes);
    geometry_msgs::Point p;
    for (auto no = nodes.begin(); no != nodes.end(); ++no) {
      p.x = no->x();
      p.y = no->y();
      p.z = kZFixed;
      obs_marker.points.push_back(p);
    }
    p.x = nodes[0].x();
    p.y = nodes[0].y();
    p.z = kZFixed;
    obs_marker.points.push_back(p);

    marker_array.markers.push_back(obs_marker);
  }

  // Text
  marker_id = 0;
  for (auto& geof : geofence_list) {
    // ID Status
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = world_frame_id;
    marker.ns = "ID";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 1.0;  // text height
    marker.color.r = 245.0 / 255;
    marker.color.g = 20.0 / 255;
    marker.color.b = 12.0 / 255;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(sampler_lifetime);
    marker.frame_locked = false;
    Eigen::Vector2d geof_center;
    geof.polygon.getCenter(geof_center);
    marker.pose.position.x = geof_center.x();
    marker.pose.position.y = geof_center.y();
    marker.pose.position.z = kZFixed;
    std::string text_display = std::to_string(geof.id);
    marker.text = text_display;
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);
  }

  geofence_pub_.publish(marker_array);
}

void Visualization::visualizePCL(const pcl::PointCloud<pcl::PointXYZ>* pcl) {
  if (pcl_pub_.getNumSubscribers() < 1) return;

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GBP VIS]: pcl size: %d", (int)(pcl->points.size()));
  if (pcl->points.size() <= 0) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[GBP VIS]: Empty PCL");
    return;
  }

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*pcl, output);
  output.header.frame_id = world_frame_id;
  output.header.stamp = ros::Time::now();
  pcl_pub_.publish(output);
}

void Visualization::visualizeHyperplanes(
    Eigen::Vector3d& center, std::vector<Eigen::Vector3d>& hyperplane_list,
    std::vector<Eigen::Vector3d>& tangent_point_list) {
  static int markers_id = 0;

  ++markers_id;

  if (hyperplanes_pub_.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker proj_points;
  proj_points.header.stamp = ros::Time::now();
  proj_points.header.seq = 0;
  proj_points.header.frame_id = world_frame_id;
  proj_points.id = markers_id;
  proj_points.ns = "planes";
  proj_points.action = visualization_msgs::Marker::ADD;
  proj_points.type = visualization_msgs::Marker::CUBE_LIST;
  proj_points.scale.x = 0.5;
  proj_points.scale.y = 0.5;
  proj_points.scale.z = 0.5;
  proj_points.color.r = 0.0;
  proj_points.color.g = 0.0;
  proj_points.color.b = 1.0;
  proj_points.color.a = 0.8;
  proj_points.lifetime = ros::Duration(graph_lifetime);
  proj_points.frame_locked = false;

  int marker_id = 0;
  for (int i = 0; i < hyperplane_list.size(); ++i) {
    // Get projection point
    // double t0 = -(hyperplane_list[i].dot(center) - 1) /
    // (hyperplane_list[i].squaredNorm()); Eigen::Vector3d x_proj = center +
    // hyperplane_list[i] * t0; geometry_msgs::Point p; p.x = x_proj[0]; p.y =
    // x_proj[1]; p.z = x_proj[2]; proj_points.points.push_back(p);

    geometry_msgs::Point p;
    p.x = tangent_point_list[i][0];
    p.y = tangent_point_list[i][1];
    p.z = tangent_point_list[i][2];
    proj_points.points.push_back(p);

    Eigen::Quaternion<double> quat_W2S;
    Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
    quat_W2S.setFromTwoVectors(x_axis, hyperplane_list[i].normalized());
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = world_frame_id;
    marker.ns = "normal";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 1.0;   // length of the arrow
    marker.scale.y = 0.15;  // arrow width
    marker.scale.z = 0.15;  // arrow height
    marker.color.r = 200.0 / 255.0;
    marker.color.g = 50.0 / 255.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(graph_lifetime);
    marker.frame_locked = false;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = p.z;
    marker.pose.orientation.x = quat_W2S.x();
    marker.pose.orientation.y = quat_W2S.y();
    marker.pose.orientation.z = quat_W2S.z();
    marker.pose.orientation.w = quat_W2S.w();
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);

    visualization_msgs::Marker marker_plane;
    marker_plane.header.stamp = ros::Time::now();
    marker_plane.header.seq = 0;
    marker_plane.header.frame_id = world_frame_id;
    marker_plane.ns = "plane";
    marker_plane.action = visualization_msgs::Marker::ADD;
    marker_plane.type = visualization_msgs::Marker::CUBE;
    marker_plane.scale.x = 0.1;  // length of the arrow
    marker_plane.scale.y = 5.0;  // arrow width
    marker_plane.scale.z = 5.0;  // arrow height
    marker_plane.color.r = 200.0 / 255.0;
    marker_plane.color.g = 50.0 / 255.0;
    marker_plane.color.b = 0.0;
    marker_plane.color.a = 0.5;
    marker_plane.lifetime = ros::Duration(graph_lifetime);
    marker_plane.frame_locked = false;
    marker_plane.pose.position.x = p.x;
    marker_plane.pose.position.y = p.y;
    marker_plane.pose.position.z = p.z;
    marker_plane.pose.orientation.x = quat_W2S.x();
    marker_plane.pose.orientation.y = quat_W2S.y();
    marker_plane.pose.orientation.z = quat_W2S.z();
    marker_plane.pose.orientation.w = quat_W2S.w();
    marker_plane.id = marker_id++;
    marker_array.markers.push_back(marker_plane);
  }
  marker_array.markers.push_back(proj_points);

  visualization_msgs::Marker tangent_points;
  tangent_points.header.stamp = ros::Time::now();
  tangent_points.header.seq = 0;
  tangent_points.header.frame_id = world_frame_id;
  tangent_points.id = 0;
  tangent_points.ns = "points";
  tangent_points.action = visualization_msgs::Marker::ADD;
  tangent_points.type = visualization_msgs::Marker::CUBE_LIST;
  tangent_points.scale.x = 0.5;
  tangent_points.scale.y = 0.5;
  tangent_points.scale.z = 0.5;
  tangent_points.color.r = 1.0;
  tangent_points.color.g = 0.0;
  tangent_points.color.b = 0.0;
  tangent_points.color.a = 0.8;
  tangent_points.lifetime = ros::Duration(graph_lifetime);
  tangent_points.frame_locked = false;

  for (int i = 0; i < tangent_point_list.size(); ++i) {
    geometry_msgs::Point p;
    p.x = tangent_point_list[i][0];
    p.y = tangent_point_list[i][1];
    p.z = tangent_point_list[i][2];
    tangent_points.points.push_back(p);
  }

  marker_array.markers.push_back(tangent_points);

  hyperplanes_pub_.publish(marker_array);
}

void Visualization::visualizeModPath(
    const std::vector<geometry_msgs::Pose>& path) {
  if (mod_path_pub_.getNumSubscribers() < 1) return;

  if (path.empty()) return;

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "mod_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.25;
  edge_marker.color.r = 244.0 / 255.0;
  edge_marker.color.g = 217 / 255.0;
  edge_marker.color.b = 66 / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    geometry_msgs::Point p2;
    p2.x = path[i + 1].position.x;
    p2.y = path[i + 1].position.y;
    p2.z = path[i + 1].position.z;
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);
  mod_path_pub_.publish(marker_array);
}

void Visualization::visualizeBlindModPath(
    const std::vector<geometry_msgs::Pose>& path) {
  if (blind_mod_path_pub_.getNumSubscribers() < 1) return;

  if (path.empty()) return;
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = world_frame_id;
  edge_marker.id = 0;
  edge_marker.ns = "blind_mod_path";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.25;
  edge_marker.color.r = 0.0;
  edge_marker.color.g = 0.0;
  edge_marker.color.b = 226.0 / 255.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  edge_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    geometry_msgs::Point p2;
    p2.x = path[i + 1].position.x;
    p2.y = path[i + 1].position.y;
    p2.z = path[i + 1].position.z;
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  marker_array.markers.push_back(edge_marker);

  // Plot all vertices.
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = world_frame_id;
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.25;
  vertex_marker.scale.y = 0.25;
  vertex_marker.scale.z = 0.25;
  vertex_marker.color.r = 200.0 / 255.0;
  vertex_marker.color.g = 100.0 / 255.0;
  vertex_marker.color.b = 0.0;
  vertex_marker.color.a = 1.0;
  vertex_marker.lifetime = ros::Duration(shortest_paths_lifetime);
  vertex_marker.frame_locked = false;

  for (int i = 0; i < (path.size() - 1); ++i) {
    geometry_msgs::Point p1;
    p1.x = path[i].position.x;
    p1.y = path[i].position.y;
    p1.z = path[i].position.z;
    vertex_marker.points.push_back(p1);
  }
  marker_array.markers.push_back(vertex_marker);
  blind_mod_path_pub_.publish(marker_array);
}

}  // namespace explorer
