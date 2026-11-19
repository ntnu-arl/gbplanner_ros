#pragma once

#include <ros1_compat/node_handle.h>
#include "gbplanner/gbplanner.h"
#include "gbplanner/gbplanner_bt_nodes.h"

class GbplannerRos
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer planner_service_;
  ros::ServiceServer planner_homing_service_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<Gbplanner> gbplanner_;
public:
  GbplannerRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  bool plannerServiceCallback(planner_msgs::srv::PlannerSrv::Request& req,
                              planner_msgs::srv::PlannerSrv::Response& res);
  bool plannerHomingServiceCallback(planner_msgs::srv::PlannerHoming::Request& req,
                              planner_msgs::srv::PlannerHoming::Response& res);

  void registerTree();

};
