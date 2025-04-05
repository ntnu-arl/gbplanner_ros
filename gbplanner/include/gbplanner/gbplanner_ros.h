#pragma once

#include "gbplanner/gbplanner.h"
#include "gbplanner/gbplanner_bt_nodes.h"

class GbplannerRos
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer planner_service_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<Gbplanner> gbplanner_;
public:
  GbplannerRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  bool plannerServiceCallback(planner_msgs::planner_srv::Request& req,
                              planner_msgs::planner_srv::Response& res);

  void registerTree();

};
