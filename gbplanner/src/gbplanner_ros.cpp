#include "gbplanner/gbplanner_ros.h"

GbplannerRos::GbplannerRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  :nh_(nh), nh_private_(nh_private)
{
  planner_service_ = nh_.advertiseService(
      "gbplanner_ros", &GbplannerRos::plannerServiceCallback, this);
  
  gbplanner_.reset(new Gbplanner(nh_, nh_private_));

  registerTree();
}

bool GbplannerRos::plannerServiceCallback(planner_msgs::planner_srv::Request& req,
                              planner_msgs::planner_srv::Response& res)
{
  gbplanner_->setPlannerSrvReq(req);
  
  tree_.tickOnce();

  gbplanner_->getPlannerSrvRes(res);
  std::cout << "Sending response to PCI: " << res.status << std::endl;

  return true;
}

void GbplannerRos::registerTree()
{
  factory_.registerNodeType<LocalExploration>("LocalExploration", gbplanner_);
  factory_.registerNodeType<GlobalExploration>("GlobalExploration", gbplanner_);
  factory_.registerNodeType<LocalExpExhaustedCheck>("LocalExpExhaustedCheck", gbplanner_);
  factory_.registerNodeType<GlobalExpExhaustedCheck>("GlobalExpExhaustedCheck", gbplanner_);
  factory_.registerNodeType<Inspection>("Inspection", gbplanner_);
  factory_.registerNodeType<Homing>("Homing", gbplanner_);
  factory_.registerNodeType<HomingCheck>("HomingCheck", gbplanner_);
  factory_.registerNodeType<OPENINGPhase1>("OPENINGPhase1", gbplanner_);
  factory_.registerNodeType<OPENINGPhaseCheck>("OPENINGPhaseCheck", gbplanner_);
  factory_.registerNodeType<OPENINGPhase2>("OPENINGPhase2", gbplanner_);
  factory_.registerNodeType<LocalExpExhaustedReset>("LocalExpExhaustedReset", gbplanner_);
  factory_.registerNodeType<Idle>("Idle", gbplanner_);
  factory_.registerNodeType<OPENINGP1FailCheck>("OPENINGP1FailCheck", gbplanner_);
  factory_.registerNodeType<SetNextCompartment>("SetNextCompartment", gbplanner_);
  factory_.registerNodeType<AllCompartmentsInspectedCheck>("AllCompartmentsInspectedCheck", gbplanner_);
  
  std::string tree_path = ros::package::getPath("gbplanner") + "/config/bt_xml/main_tree.xml";
  if(!ros::param::get(ros::this_node::getName() + "/behavior_tree_path", tree_path))
  {
    tree_path = ros::package::getPath("gbplanner") + "/config/bt_xml/main_tree.xml";
  }
  std::string trial_tree_path;
  ros::param::get("~tree_path", trial_tree_path);

  ROS_ERROR_STREAM("Tree path: " << tree_path << " Trial Tree Path: " << trial_tree_path);
  
  factory_.registerBehaviorTreeFromFile(tree_path);
  tree_ = factory_.createTree("MainTree");
	std::cout << "Behavior Tree built" << std::endl;

  std::string xml_models = BT::writeTreeNodesModelXML(factory_);

  std::cout << "TreeNodesMode: " << xml_models << std::endl;

  // BT::Groot2Publisher publisher(tree_);
}