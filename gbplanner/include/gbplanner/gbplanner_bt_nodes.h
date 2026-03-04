#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "gbplanner/gbplanner.h"

/*
Returns:
  SUCCESS: Always
  The flag gbplanner_->bt_states_.local_exp_exhausted is set to true when local exploration is exhausted
*/
class LocalExploration : public BT::StatefulActionNode
{
public:
  LocalExploration(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
    :StatefulActionNode(name, config), gbplanner_(std::move(gbplanner))
  {}

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_exp_count_ = 0;
  int max_exp_tries_ = 3;
};

/*
Returns:
  SUCCESS: If gbplanner_->bt_states_.local_exp_exhausted == true
  FAILURE: If gbplanner_->bt_states_.local_exp_exhausted == false
*/
class LocalExpExhaustedCheck : public BT::SyncActionNode
{
public:
  LocalExpExhaustedCheck(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};

/*
Returns:
  SUCCESS: Always
*/
class LocalExpExhaustedReset : public BT::SyncActionNode
{
public:
  LocalExpExhaustedReset(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};


/*
Returns:
  SUCCESS: If gbplanner_->bt_states_.operation_mode == 1
  FAILURE: If gbplanner_->bt_states_.operation_mode == 0
*/
class SwitchToLocalNavigation : public BT::SyncActionNode
{
public:
  SwitchToLocalNavigation(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};


/*
Returns:
  SUCCESS: Always
  The flag gbplanner_->bt_states_.local_navigation_complete is set to true when waypoint is reached
*/
class LocalNavigation : public BT::StatefulActionNode
{
public:
  LocalNavigation(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
    :StatefulActionNode(name, config), gbplanner_(std::move(gbplanner))
  {}

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_nav_count_ = 0;
  int max_nav_tries_ = 3;
};

/*
Returns:
  SUCCESS: If gbplanner_->bt_states_.local_navigation_complete == true
  FAILURE: If gbplanner_->bt_states_.local_navigation_complete == false
*/
class LocalNavigationExhaustedCheck : public BT::SyncActionNode
{
public:
  LocalNavigationExhaustedCheck(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};

/*
Returns:
  SUCCESS: Always
*/
class LocalNavigationExhaustedReset : public BT::SyncActionNode
{
public:
  LocalNavigationExhaustedReset(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};


/*
Returns:
  SUCCESS: Successfully found global repositioning or homing path, failed tries > max failed tries
  FAILURE: Unable to find a path (returns an error)
*/
class GlobalExploration : public BT::StatefulActionNode
{
public:
  GlobalExploration(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
    :StatefulActionNode(name, config), gbplanner_(std::move(gbplanner))
  {}

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_exp_count_ = 0;
  int max_global_planner_tries_ = 3;
};

/*
Returns:
  SUCCESS: No frontier exists in global graph
  FAILURE: Frontier exists in global graph
*/
class GlobalExpExhaustedCheck : public BT::SyncActionNode
{
public:
  GlobalExpExhaustedCheck(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};


/*
Returns:
  SUCCESS: Found a path successfully, failed attempts > max attempts
  FAILURE: Frontier exists in global graph
*/
class CalculateGlobalPath : public BT::SyncActionNode
{
public:
  CalculateGlobalPath(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_global_planner_count_ = 0;
  int max_global_planner_tries_ = 3;
};

/*
Returns:
  SUCCESS: Global path not fully executed
  FAILURE: Path fully executed
*/
class UpdateGlobalGoal : public BT::SyncActionNode
{
public:
  UpdateGlobalGoal(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_global_planner_count_ = 0;
  int max_global_planner_tries_ = 3;
};


/*
onStart():
 Returns:
  SUCCESS: Failed tries > max failed tries
  FAILURE: Error
  RUNNING: Inspection path calculated successfully
onRunning():
 Does nothing
 Returns:
  SUCCESS: Always
*/
class Inspection : public BT::StatefulActionNode
{
public:
  Inspection(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
    :StatefulActionNode(name, config), gbplanner_(std::move(gbplanner))
  {}

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_inspection_count_ = 0;
  int max_inspection_tries_ = 3;
};



class CompartmentTransition : public BT::StatefulActionNode
{
public:
  CompartmentTransition(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
    :StatefulActionNode(name, config), gbplanner_(std::move(gbplanner))
  {}

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_compartment_transition_count_ = 0;
  int max_compartment_transition_tries_ = 3;
};



/*
Returns:
  SUCCESS: Found a path successfully, failed attempts > max attempts
  FAILURE: Frontier exists in global graph
*/
class Homing : public BT::SyncActionNode
{
public:
  Homing(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_homing_count_ = 0;
  int max_homing_tries_ = 3;
};


/*
Returns:
  SUCCESS: Found a path successfully, failed attempts > max attempts
  FAILURE: Frontier exists in global graph
*/
class CalculateHomingPath : public BT::SyncActionNode
{
public:
  CalculateHomingPath(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_homing_count_ = 0;
  int max_homing_tries_ = 3;
};


/*
Returns:
  SUCCESS: Homing path not fully executed
  FAILURE: Path fully executed
*/
class UpdateHomingGoal : public BT::SyncActionNode
{
public:
  UpdateHomingGoal(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_homing_count_ = 0;
  int max_homing_tries_ = 3;
};


/*
Checks if homing is needed using set criteria, e.g., mission time is exhausted
Returns:
  SUCCESS: If homing is needed (according to the Gbplanner::homingRequired() function)
  FAILURE: Otherwise
*/
class HomingCheck : public BT::SyncActionNode
{
public:
  HomingCheck(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};

/*
onStart():
 Returns:
  SUCCESS: No openings. The parameter gbplanner_->bt_states_.opening_phase1_failed is set to true
  FAILURE: Can't find connecting path
  RUNNING: Phase 1 path calculated successfully
onRunning():
 Performs OPENINGCheck
 Returns:
  SUCCESS: If check successfull
  FAILURE: Otherwise
*/
class OPENINGPhase1 : public BT::StatefulActionNode
{
public:
  OPENINGPhase1(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
    :StatefulActionNode(name, config), gbplanner_(std::move(gbplanner))
  {}

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<Gbplanner> gbplanner_;
  int failed_opening_phase1_count_ = 0;
  int max_opening_phase1_tries_ = 3;
};

/*
Returns:
  SUCCESS: If gbplanner_->bt_states_.opening_phase1_failed == true
  FAILURE: Otherwise
*/
class OPENINGP1FailCheck : public BT::SyncActionNode
{
public:
  OPENINGP1FailCheck(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};

/*
NOT USED ANYMORE
*/
class OPENINGPhaseCheck : public BT::StatefulActionNode
{
public:
  OPENINGPhaseCheck(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
    :StatefulActionNode(name, config), gbplanner_(std::move(gbplanner))
  {}

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};

/*
Returns:
  SUCCESS: Path calculated successfully. 
  FAILURE: Otherwise. THIS SHOULD NEVER HAPPEN
*/
class OPENINGPhase2 : public BT::StatefulActionNode
{
public:
  OPENINGPhase2(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
    :StatefulActionNode(name, config), gbplanner_(std::move(gbplanner))
  {}

  static BT::PortsList providedPorts() 
  {
    return {BT::OutputPort<int>("mode")};
  }

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};

/*
Returns:
  SUCCESS: If next compartment is set successfully
  FAILURE: If all compartments are visited
*/
class SetNextCompartment : public BT::SyncActionNode
{
public:
  SetNextCompartment(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};

/*
Returns:
  SUCCESS: If all compartments are visited
  FAILURE: Otherwise
*/
class AllCompartmentsInspectedCheck : public BT::SyncActionNode
{
public:
  AllCompartmentsInspectedCheck(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};

/*
Sends empty path to PCI with path type kManualPath => No more planning iterations are triggered
Returns:
  FAILURE: Always => To halt the tree
*/
class Idle : public BT::SyncActionNode
{
public:
  Idle(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Gbplanner> gbplanner)
        :SyncActionNode(name, config), gbplanner_(std::move(gbplanner)) {}

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() 
  {
    return {};
  }

private:
  std::shared_ptr<Gbplanner> gbplanner_;
};
