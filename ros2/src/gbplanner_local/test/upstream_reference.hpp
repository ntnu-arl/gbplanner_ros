#pragma once
#include "gbplanner_local/local_planner.hpp"

namespace upstream_reference {
enum class RobotType { kAerialRobot };
enum class RRModeType { kGraph, kTree };
struct MapManager {
  enum class VoxelStatus { kUnknown, kOccupied, kFree };
  std::shared_ptr<const gbplanner2::MapQuery> map;
  VoxelStatus getPathStatus(const Eigen::Vector3d& a,const Eigen::Vector3d& b,
                            const Eigen::Vector3d& size,bool) {
    auto status=map->path(a,b,size);
    if(status==gbplanner2::VoxelStatus::Free)return VoxelStatus::kFree;
    if(status==gbplanner2::VoxelStatus::Occupied)return VoxelStatus::kOccupied;
    return VoxelStatus::kUnknown;
  }
};
class ReferenceRrg {
 public:
  ReferenceRrg(std::shared_ptr<const gbplanner2::MapQuery> map,const gbplanner2::Parameters& p) {
    map_manager_=std::make_shared<MapManager>();map_manager_->map=map;
    planning_params_={p.edge_max,p.edge_min,p.overshoot,p.neighbor_radius,p.neighbor_min,p.neighbor_max,
                     p.graph_mode?RRModeType::kGraph:RRModeType::kTree};
    robot_params_.center_offset=p.center_offset;robot_box_size_=p.robot_size;
  }
  void expandGraph(std::shared_ptr<GraphManager> graph_manager,StateVec& new_state,
                   ExpandGraphReport& rep,bool allow_short_edge=false);
  void expandVertex(std::shared_ptr<GraphManager> graph_manager,Vertex& new_vertex,
                   ExpandGraphReport& rep,bool allow_short_edge=false);
 private:
  struct PlanningParams {
    double edge_length_max,edge_length_min,edge_overshoot,nearest_range,nearest_range_min,nearest_range_max;
    RRModeType rr_mode;
  } planning_params_;
  struct RobotParams {RobotType type{RobotType::kAerialRobot};Eigen::Vector3d center_offset;} robot_params_;
  struct Statistics {int num_edges_fail{0};std::vector<std::vector<double>> edges_fail;};
  std::shared_ptr<Statistics> stat_{std::make_shared<Statistics>()};
  std::shared_ptr<MapManager> map_manager_;
  Eigen::Vector3d robot_box_size_;
};
#include "upstream_expand.inc"
#include "upstream_global_expand.inc"
}
