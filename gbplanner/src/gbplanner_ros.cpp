#include "gbplanner/gbplanner_ros.h"

GbplannerRos::GbplannerRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  :nh_(nh), nh_private_(nh_private)
{
  planner_service_ = nh_.advertiseService(
      "gbplanner_ros", &GbplannerRos::plannerServiceCallback, this);
  
  planner_homing_service_ = nh_.advertiseService(
      "gbplanner_ros_homing", &GbplannerRos::plannerHomingServiceCallback, this);
  
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

bool GbplannerRos::plannerHomingServiceCallback(planner_msgs::planner_homing::Request& req,
                              planner_msgs::planner_homing::Response& res)
{
  ROS_WARN("Homing through BT");
  tree_.haltTree();
  gbplanner_->requestHomingOverride();

  // planner_msgs::planner_srv::Request req_planner;
  // planner_msgs::planner_srv::Response res_planner;
  // req_planner.header = req.header;
  // req_planner.bound_mode = planner_msgs::BoundMode::kExtendedBound;

  // gbplanner_->setPlannerSrvReq(req_planner);
  // tree_.tickOnce();
  // gbplanner_->getPlannerSrvRes(res_planner);

  // res.path = res_planner.path;

  return true;
}

void GbplannerRos::registerTree()
{
  factory_.registerNodeType<LocalExploration>("LocalExploration", gbplanner_);
  factory_.registerNodeType<GlobalExploration>("GlobalExploration", gbplanner_);
  factory_.registerNodeType<LocalExpExhaustedCheck>("LocalExpExhaustedCheck", gbplanner_);
  factory_.registerNodeType<GlobalExpExhaustedCheck>("GlobalExpExhaustedCheck", gbplanner_);
  factory_.registerNodeType<Inspection>("Inspection", gbplanner_);
  factory_.registerNodeType<CompartmentTransition>("CompartmentTransition", gbplanner_);
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
  factory_.registerNodeType<LocalNavigation>("LocalNavigation", gbplanner_);
  factory_.registerNodeType<LocalNavigationExhaustedCheck>("LocalNavigationExhaustedCheck", gbplanner_);
  factory_.registerNodeType<LocalNavigationExhaustedReset>("LocalNavigationExhaustedReset", gbplanner_);
  factory_.registerNodeType<CalculateHomingPath>("CalculateHomingPath", gbplanner_);
  factory_.registerNodeType<UpdateHomingGoal>("UpdateHomingGoal", gbplanner_);
  factory_.registerNodeType<SwitchToLocalNavigation>("SwitchToLocalNavigation", gbplanner_);
  factory_.registerNodeType<CalculateGlobalPath>("CalculateGlobalPath", gbplanner_);
  factory_.registerNodeType<UpdateGlobalGoal>("UpdateGlobalGoal", gbplanner_);

  std::string tree_path = ros::package::getPath("gbplanner") + "/config/bt_xml/main_tree.xml";
  if(!ros::param::get(ros::this_node::getName() + "/behavior_tree_path", tree_path))
  {
    tree_path = ros::package::getPath("gbplanner") + "/config/bt_xml/main_tree.xml";
  }
  std::string trial_tree_path;
  ros::param::get("~tree_path", trial_tree_path);

  ROS_WARN_STREAM("Tree path: " << tree_path << " Trial Tree Path: " << trial_tree_path);
  
  factory_.registerBehaviorTreeFromFile(tree_path);
  tree_ = factory_.createTree("MainTree");
	std::cout << "Behavior Tree built" << std::endl;

  std::string xml_models = BT::writeTreeNodesModelXML(factory_);

  std::cout << "TreeNodesMode: " << xml_models << std::endl;

  // BT::Groot2Publisher publisher(tree_);
}
