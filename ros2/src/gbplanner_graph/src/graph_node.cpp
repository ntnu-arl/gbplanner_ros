#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <planner_msgs/srv/planner_search.hpp>
#include <planner_msgs/srv/planner_homing.hpp>
#include <planner_msgs/srv/planner_set_homing_pos.hpp>
#include <planner_msgs/srv/planner_string_trigger.hpp>
#include "planner_common/graph_manager.h"

// Transport for the migrated graph subsystem. Edges must already be
// collision-checked by a mapping/planning module; this node does not build RRGs.
class GraphNode : public rclcpp::Node {
 public:
  GraphNode() : Node("gbplanner_graph") {
    frame_ = declare_parameter<std::string>("frame_id", "map");
    snap_radius_ = declare_parameter<double>("snap_radius", 0.5);
    if (frame_.empty() || !std::isfinite(snap_radius_) || snap_radius_ <= 0)
      throw std::invalid_argument("frame_id and snap_radius must be valid");
    auto durable = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", durable);
    graph_sub_ = create_subscription<planner_msgs::msg::Graph>("graph", durable,
      [this](planner_msgs::msg::Graph::ConstSharedPtr message) {
        try {
          if (message->header.frame_id != frame_) throw std::invalid_argument("Graph frame mismatch");
          graph_.convertMsgToGraph(*message);
          RCLCPP_INFO(get_logger(), "Graph received: %d vertices, %d edges",
                      graph_.getNumVertices(), graph_.getNumEdges());
        } catch (const std::exception& error) { RCLCPP_WARN(get_logger(), "%s", error.what()); }
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        if (message->header.frame_id == frame_ && validPose(message->pose.pose)) {
          current_ = message->pose.pose; have_current_ = true;
        }
      });
    using Search = planner_msgs::srv::PlannerSearch;
    search_ = create_service<Search>("planner_search",
      [this](const std::shared_ptr<Search::Request> request, std::shared_ptr<Search::Response> response) {
        // A graph-only stage cannot enforce the original robot bounding modes.
        // Require an explicit no-map-check request instead of silently ignoring it.
        if (request->header.frame_id != frame_ || request->bound_mode != Search::Request::K_NO_BOUND ||
            (request->use_current_state && !have_current_)) return;
        response->success = search(request->use_current_state ? current_ : request->source,
                                   request->target, response->path);
        if (response->success) publish(response->path);
      });
    using Homing = planner_msgs::srv::PlannerHoming;
    homing_ = create_service<Homing>("planner_homing",
      [this](const std::shared_ptr<Homing::Request> request, std::shared_ptr<Homing::Response> response) {
        if (request->header.frame_id == frame_ && have_current_ && have_home_ &&
            search(current_, home_, response->path)) publish(response->path);
      });
    using SetHome = planner_msgs::srv::PlannerSetHomingPos;
    set_home_ = create_service<SetHome>("planner_set_homing_pos",
      [this](const std::shared_ptr<SetHome::Request>, std::shared_ptr<SetHome::Response> response) {
        response->success = have_current_;
        if (have_current_) { home_ = current_; have_home_ = true; }
      });
    using File = planner_msgs::srv::PlannerStringTrigger;
    save_ = create_service<File>("graph_save",
      [this](const std::shared_ptr<File::Request> request, std::shared_ptr<File::Response> response) {
        try { graph_.saveGraph(request->message); response->success = true; }
        catch (const std::exception& e) { RCLCPP_WARN(get_logger(), "%s", e.what()); }
      });
    load_ = create_service<File>("graph_load",
      [this](const std::shared_ptr<File::Request> request, std::shared_ptr<File::Response> response) {
        try { graph_.loadGraph(request->message); response->success = true; }
        catch (const std::exception& e) { RCLCPP_WARN(get_logger(), "%s", e.what()); }
      });
    RCLCPP_INFO(get_logger(), "GBPlanner graph subsystem ready; frame=%s", frame_.c_str());
  }
 private:
  static bool validPose(const geometry_msgs::msg::Pose& p) {
    const auto& q = p.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
           std::isfinite(p.position.z) && std::isfinite(norm) && norm > 1e-12;
  }
  bool search(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to,
              std::vector<geometry_msgs::msg::Pose>& path) {
    path.clear();
    if (!validPose(from) || !validPose(to)) return false;
    StateVec a(from.position.x, from.position.y, from.position.z, 0);
    StateVec b(to.position.x, to.position.y, to.position.z, 0);
    Vertex *source = nullptr, *target = nullptr;
    if (!graph_.getNearestVertexInRange(&a, snap_radius_, &source) ||
        !graph_.getNearestVertexInRange(&b, snap_radius_, &target)) return false;
    ShortestPathsReport report;
    if (!graph_.findShortestPaths(source->id, report)) return false;
    std::vector<Vertex*> vertices;
    graph_.getShortestPath(target->id, report, true, vertices);
    for (const auto* v : vertices) {
      geometry_msgs::msg::Pose p;
      p.position.x = v->state.x(); p.position.y = v->state.y(); p.position.z = v->state.z();
      p.orientation.z = std::sin(v->state[3] / 2); p.orientation.w = std::cos(v->state[3] / 2);
      path.push_back(p);
    }
    return !path.empty();
  }
  void publish(const std::vector<geometry_msgs::msg::Pose>& poses) {
    nav_msgs::msg::Path path; path.header.frame_id = frame_; path.header.stamp = now();
    for (const auto& pose : poses) { geometry_msgs::msg::PoseStamped p; p.header = path.header; p.pose = pose; path.poses.push_back(p); }
    path_pub_->publish(path);
  }
  GraphManager graph_;
  std::string frame_; double snap_radius_;
  geometry_msgs::msg::Pose current_, home_;
  bool have_current_ = false, have_home_ = false;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<planner_msgs::msg::Graph>::SharedPtr graph_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<planner_msgs::srv::PlannerSearch>::SharedPtr search_;
  rclcpp::Service<planner_msgs::srv::PlannerHoming>::SharedPtr homing_;
  rclcpp::Service<planner_msgs::srv::PlannerSetHomingPos>::SharedPtr set_home_;
  rclcpp::Service<planner_msgs::srv::PlannerStringTrigger>::SharedPtr save_, load_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphNode>());
  rclcpp::shutdown();
  return 0;
}
