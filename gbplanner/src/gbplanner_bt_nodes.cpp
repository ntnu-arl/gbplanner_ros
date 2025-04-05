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
    failed_homing_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}
/*******************************************************/


/***************** HomingCheck **************/
BT::NodeStatus HomingCheck::tick()
{
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


/***************** MHPhase1 **********************/
BT::NodeStatus MHPhase1::onStart()
{
  ROS_INFO("[MHPhase1] Triggered.");
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_mh_phase1_count_, 2);  // TODO: Set the max bound number through param

  ManholeTraversalMode mode = ManholeTraversalMode::kGoingTo;
  ManholeTraversalStatus status;

  gbplanner_->getManholeTraversalPath(mode, status);
  if(status == ManholeTraversalStatus::CANT_CONNECT)
  {
    ++failed_mh_phase1_count_;
    return BT::NodeStatus::FAILURE;
  }
  else if(status == ManholeTraversalStatus::OK)
  {
    failed_mh_phase1_count_ = 0;
    gbplanner_->bt_states_.mh_phase1_failed = false;
    return BT::NodeStatus::RUNNING;
  }
  else if(status == ManholeTraversalStatus::NO_MANHOLES)
  {
    gbplanner_->bt_states_.mh_phase1_failed = true;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MHPhase1::onRunning()
{
  ManholeTraversalMode mode = ManholeTraversalMode::kPathCheck;
  ManholeTraversalStatus status;

  gbplanner_->getManholeTraversalPath(mode, status);

  if(status == ManholeTraversalStatus::OK)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

void MHPhase1::onHalted()
{
  std::cout << "[MHPhase1] Halted" << std::endl;
}
/*******************************************************/


/***************** MHP1FailCheck **************/
BT::NodeStatus MHP1FailCheck::tick()
{
  if(gbplanner_->bt_states_.mh_phase1_failed)
  {
    ROS_WARN("MH Phase1 Failed");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
/*******************************************************/


/***************** MHPhaseCheck **********************/
BT::NodeStatus MHPhaseCheck::onStart()
{
  ROS_INFO("[MHPhaseCheck] Triggered.");

  ManholeTraversalMode mode = ManholeTraversalMode::kPathCheck;
  ManholeTraversalStatus status;

  gbplanner_->getManholeTraversalPath(mode, status);
  if(status == ManholeTraversalStatus::OK)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MHPhaseCheck::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void MHPhaseCheck::onHalted()
{
  std::cout << "[MHPhaseCheck] Halted" << std::endl;
}
/*******************************************************/


/***************** MHPhase2 **********************/
BT::NodeStatus MHPhase2::onStart()
{
  ROS_INFO("[MHPhase2] Triggered.");

  ManholeTraversalMode mode = ManholeTraversalMode::kPassingThrough;
  ManholeTraversalStatus status;

  gbplanner_->getManholeTraversalPath(mode, status);
  if(status == ManholeTraversalStatus::OK)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus MHPhase2::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void MHPhase2::onHalted()
{
  std::cout << "[MHPhase2] Halted" << std::endl;
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

