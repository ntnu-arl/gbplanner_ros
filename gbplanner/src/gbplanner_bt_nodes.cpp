#include "gbplanner/gbplanner_bt_nodes.h"

/***************** LocalExploration **********************/
BT::NodeStatus LocalExploration::onStart()
{
  std::cout << "[Local Exploration] Triggered." << std::endl;
  Rrg::LocalPlannerStatus status = gbplanner_->getExplorationPath();
  if(status == Rrg::LocalPlannerStatus::L_EXHAUSTED)
  {
    gbplanner_->bt_states_.local_exp_exhausted = true;
  }
  else if(status == Rrg::LocalPlannerStatus::L_TIME_LIMIT_REACHED)
  {
    gbplanner_->bt_states_.homing_triggered = true;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LocalExploration::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void LocalExploration::onHalted()
{
  std::cout << "[Local Exploration] Halted" << std::endl;
}
/*******************************************************/

/***************** LocalExpExhaustedCheck **************/
BT::NodeStatus LocalExpExhaustedCheck::tick()
{
  if(gbplanner_->bt_states_.local_exp_exhausted)
  {
    ROS_WARN("Local Exp Exhausted");
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}
/*******************************************************/

/***************** LocalExpExhaustedReset **************/
BT::NodeStatus LocalExpExhaustedReset::tick()
{
  gbplanner_->bt_states_.local_exp_exhausted = false;
  gbplanner_->clearResPath();

  return BT::NodeStatus::SUCCESS;
}
/*******************************************************/


/***************** LocalNavigation **********************/
BT::NodeStatus LocalNavigation::onStart()
{
  std::cout << "[Local Exploration] Triggered." << std::endl;
  Rrg::LocalPlannerStatus status = gbplanner_->getLocalNavigationPath();
  if(status == Rrg::LocalPlannerStatus::L_EXHAUSTED)
  {
    gbplanner_->bt_states_.local_navigation_complete = true;
  }
  else if(status == Rrg::LocalPlannerStatus::L_STUCK)
  {
    gbplanner_->bt_states_.local_navigation_stuck = true;
  }
  else if(status == Rrg::LocalPlannerStatus::L_TIME_LIMIT_REACHED)
  {
    gbplanner_->bt_states_.homing_triggered = true;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LocalNavigation::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void LocalNavigation::onHalted()
{
  std::cout << "[Local Exploration] Halted" << std::endl;
}
/*******************************************************/

/***************** LocalNavigationExhaustedCheck **************/
BT::NodeStatus LocalNavigationExhaustedCheck::tick()
{
  if(gbplanner_->bt_states_.local_navigation_complete)
  {
    ROS_WARN("Local Exp Exhausted");
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}
/*******************************************************/

/***************** LocalNavigationExhaustedReset **************/
BT::NodeStatus LocalNavigationExhaustedReset::tick()
{
  gbplanner_->bt_states_.local_navigation_complete = false;
  gbplanner_->clearResPath();

  return BT::NodeStatus::SUCCESS;
}
/*******************************************************/


/***************** GlobalExploration **********************/
BT::NodeStatus GlobalExploration::onStart()
{
  std::cout << "[Global Exploration] Triggered." << std::endl;
  gbplanner_->bt_states_.local_exp_exhausted = false;
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_exp_count_, 2);  // TODO: Set the max bound number through param

  Rrg::GlobalPlannerStatus status = gbplanner_->getGlobalExplorationPath();
  if(status == Rrg::GlobalPlannerStatus::G_ERR)
  {
    ++failed_exp_count_;
    if(failed_exp_count_ > max_global_planner_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else if(status == Rrg::GlobalPlannerStatus::G_HOMING)
  {
    ROS_WARN("[GE]: Homing Triggered");
    gbplanner_->bt_states_.homing_triggered = true;
    failed_exp_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    failed_exp_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus GlobalExploration::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void GlobalExploration::onHalted()
{
  std::cout << "[Global Exploration] Halted" << std::endl;
}
/*******************************************************/

/***************** GlobalExpExhaustedCheck **************/
BT::NodeStatus GlobalExpExhaustedCheck::tick()
{
  ROS_WARN("[GlobalExpExhaustedCheck]");
  // if(gbplanner_->bt_states_.global_exp_exhausted)
  if(gbplanner_->checkGlobalExplorationStatus())
  {
    ROS_WARN("Global Exp Exhausted");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
/*******************************************************/


/***************** Inspection **********************/
BT::NodeStatus Inspection::onStart()
{
  ROS_INFO("[Inspection] Triggered.");
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_inspection_count_, 2);  // TODO: Set the max bound number through param

  bool success = gbplanner_->getInspectionPath();
  if(!success)
  {
    ++failed_inspection_count_;
    if(failed_inspection_count_ > max_inspection_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    failed_inspection_count_ = 0;
    return BT::NodeStatus::RUNNING;
  }
}

BT::NodeStatus Inspection::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void Inspection::onHalted()
{
  std::cout << "[Inspection] Halted" << std::endl;
}
/*******************************************************/


/***************** Homing **************/
BT::NodeStatus Homing::tick()
{
  ROS_WARN("[Homing node triggered]");
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_homing_count_, 2);  // TODO: Set the max bound number through param

  bool success = gbplanner_->getHomingPath();
  if(!success)
  {
    ROS_WARN("Homing failed");
    ++failed_homing_count_;
    if(failed_homing_count_ > max_homing_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_WARN("Homing Succeeded");
    failed_homing_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}
/*******************************************************/


/***************** HomingCheck **************/
BT::NodeStatus HomingCheck::tick()
{
  if(gbplanner_->bt_states_.homing_required)
  {
    ROS_WARN("Homing needed");
    return BT::NodeStatus::SUCCESS;
  }
  
  bool homing_reqd = gbplanner_->homingRequired();
  if(homing_reqd)
  {
    ROS_WARN("Homing needed");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_WARN("Homing NOT needed");
    return BT::NodeStatus::FAILURE;
  }
}
/*******************************************************/


/***************** CalculateHomingPath **************/
BT::NodeStatus CalculateHomingPath::tick()
{
  ROS_WARN("[CalculateHomingPath node triggered]");
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_homing_count_, 2);  // TODO: Set the max bound number through param

  bool success = gbplanner_->calculateHomingPath();
  if(!success)
  {
    ROS_WARN("Homing failed");
    ++failed_homing_count_;
    if(failed_homing_count_ > max_homing_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_WARN("Calculated Homing Path Successfully");
    failed_homing_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}
/*******************************************************/


/***************** UpdateHomingGoal **************/
BT::NodeStatus UpdateHomingGoal::tick()
{
  ROS_WARN("[UpdateHomingGoal node triggered]");

  bool success = gbplanner_->updateHomingGoal();
  if(!success)
  {
    ROS_WARN("Homing Goal Update: Completed");
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    ROS_WARN("Homing Goal Update: Continuing");
    return BT::NodeStatus::SUCCESS;
  }
}




/***************** OPENINGPhase1 **********************/
BT::NodeStatus OPENINGPhase1::onStart()
{
  ROS_INFO("[OPENINGPhase1] Triggered.");
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_opening_phase1_count_, 2);  // TODO: Set the max bound number through param

  OpeningTraversalMode mode = OpeningTraversalMode::kGoingTo;
  OpeningTraversalStatus status;

  gbplanner_->getOpeningTraversalPath(mode, status);
  if(status == OpeningTraversalStatus::CANT_CONNECT)
  {
    ++failed_opening_phase1_count_;
    return BT::NodeStatus::FAILURE;
  }
  else if(status == OpeningTraversalStatus::OK)
  {
    failed_opening_phase1_count_ = 0;
    gbplanner_->bt_states_.opening_phase1_failed = false;
    return BT::NodeStatus::RUNNING;
  }
  else if(status == OpeningTraversalStatus::NO_OPENINGS)
  {
    gbplanner_->bt_states_.opening_phase1_failed = true;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus OPENINGPhase1::onRunning()
{
  OpeningTraversalMode mode = OpeningTraversalMode::kPathCheck;
  OpeningTraversalStatus status;

  gbplanner_->getOpeningTraversalPath(mode, status);

  if(status == OpeningTraversalStatus::OK)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

void OPENINGPhase1::onHalted()
{
  std::cout << "[OPENINGPhase1] Halted" << std::endl;
}
/*******************************************************/


/***************** OPENINGP1FailCheck **************/
BT::NodeStatus OPENINGP1FailCheck::tick()
{
  if(gbplanner_->bt_states_.opening_phase1_failed)
  {
    ROS_WARN("OPENING Phase1 Failed");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
/*******************************************************/


/***************** OPENINGPhaseCheck **********************/
BT::NodeStatus OPENINGPhaseCheck::onStart()
{
  ROS_INFO("[OPENINGPhaseCheck] Triggered.");

  OpeningTraversalMode mode = OpeningTraversalMode::kPathCheck;
  OpeningTraversalStatus status;

  gbplanner_->getOpeningTraversalPath(mode, status);
  if(status == OpeningTraversalStatus::OK)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus OPENINGPhaseCheck::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void OPENINGPhaseCheck::onHalted()
{
  std::cout << "[OPENINGPhaseCheck] Halted" << std::endl;
}
/*******************************************************/


/***************** OPENINGPhase2 **********************/
BT::NodeStatus OPENINGPhase2::onStart()
{
  ROS_INFO("[OPENINGPhase2] Triggered.");

  OpeningTraversalMode mode = OpeningTraversalMode::kPassingThrough;
  OpeningTraversalStatus status;

  gbplanner_->getOpeningTraversalPath(mode, status);
  if(status == OpeningTraversalStatus::OK)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus OPENINGPhase2::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void OPENINGPhase2::onHalted()
{
  std::cout << "[OPENINGPhase2] Halted" << std::endl;
}
/*******************************************************/


/***************** SetNextCompartment **************/
BT::NodeStatus SetNextCompartment::tick()
{
  if(gbplanner_->transitionCompartment())
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
/*******************************************************/

/***************** AllCompartmentsInspectedCheck **************/
BT::NodeStatus AllCompartmentsInspectedCheck::tick()
{
  if(gbplanner_->allCompartmentsInspected())
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
/*******************************************************/


/***************** Idle **************/
BT::NodeStatus Idle::tick()
{
  gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;  
  gbplanner_->out_srv_res_.path.clear();
  return BT::NodeStatus::FAILURE;
}
/*******************************************************/

