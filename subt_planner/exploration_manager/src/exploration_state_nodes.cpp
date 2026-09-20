/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include "subt_planner/exploration_state_nodes.h"
#include "subt_planner/terminal_view.h"

BT::NodeStatus LocalExploration::onStart()
{
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kLocal, "Local Exploration", "Triggered."));
  LocalExplorationPlanner::LocalPlannerStatus status = subt_planner_->getExplorationPath();
  
  if(status == LocalExplorationPlanner::LocalPlannerStatus::L_EXHAUSTED)
  {
    subt_planner_->bt_states_.local_exp_exhausted = true;
  }
  else if(status == LocalExplorationPlanner::LocalPlannerStatus::L_TIME_LIMIT_REACHED)
  {
    subt_planner_->bt_states_.homing_triggered = true;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LocalExploration::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void LocalExploration::onHalted()
{
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kLocal, "Local Exploration", "Halted."));
}

BT::NodeStatus LocalExpExhaustedCheck::tick()
{
  if(subt_planner_->bt_states_.local_exp_exhausted)
  {
    ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kLocal, "Local Exploration", "Local graph exhausted; requesting global repositioning."));
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus LocalExpExhaustedReset::tick()
{
  subt_planner_->bt_states_.local_exp_exhausted = false;
  subt_planner_->clearResPath();

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SwitchToLocalNavigation::tick()
{
  if(subt_planner_->bt_states_.operation_mode)
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kLocal, "Local Navigation", "Using direct local navigation mode."));
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus LocalNavigation::onStart()
{
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kLocal, "Local Navigation", "Triggered."));
  LocalExplorationPlanner::LocalPlannerStatus status = subt_planner_->getLocalNavigationPath();
  
  if(status == LocalExplorationPlanner::LocalPlannerStatus::L_EXHAUSTED)
  {
    subt_planner_->bt_states_.local_navigation_complete = true;
  }
  else if(status == LocalExplorationPlanner::LocalPlannerStatus::L_STUCK)
  {
    subt_planner_->bt_states_.local_navigation_stuck = true;
  }
  else if(status == LocalExplorationPlanner::LocalPlannerStatus::L_TIME_LIMIT_REACHED)
  {
    subt_planner_->bt_states_.homing_triggered = true;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LocalNavigation::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void LocalNavigation::onHalted()
{
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kLocal, "Local Navigation", "Halted."));
}

BT::NodeStatus LocalNavigationExhaustedCheck::tick()
{
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kLocal, "Local Navigation", "Checking completion."));
  if(subt_planner_->bt_states_.local_navigation_complete)
  {
    ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kLocal, "Local Navigation", "Navigation target completed."));
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus LocalNavigationExhaustedReset::tick()
{
  subt_planner_->bt_states_.local_navigation_complete = false;
  subt_planner_->bt_states_.local_navigation_stuck = false;

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GlobalExploration::onStart()
{
  ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "Triggered."));
  subt_planner_->bt_states_.local_exp_exhausted = false;
  subt_planner_->in_srv_req_.bound_mode = std::min(failed_exp_count_, 2);

  LocalExplorationPlanner::GlobalPlannerStatus status = subt_planner_->getGlobalExplorationPath();
  
  if(status == LocalExplorationPlanner::GlobalPlannerStatus::G_ERR)
  {
    ++failed_exp_count_;
    if(failed_exp_count_ > max_global_planner_tries_)
    {
      subt_planner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else if(status == LocalExplorationPlanner::GlobalPlannerStatus::G_HOMING)
  {
    ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Triggered by global planner."));
    subt_planner_->bt_states_.homing_triggered = true;
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
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "Halted."));
}

BT::NodeStatus GlobalExpExhaustedCheck::tick()
{
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "Checking remaining frontiers."));
  if(subt_planner_->checkGlobalExplorationStatus())
  {
    ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "No valid frontier remains."));
    return BT::NodeStatus::SUCCESS;
  }
  subt_planner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CalculateGlobalPath::tick()
{
  ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "Computing topological path."));
  subt_planner_->bt_states_.local_exp_exhausted = false;
  subt_planner_->in_srv_req_.bound_mode = std::min(failed_global_planner_count_, 2);

  bool success = subt_planner_->calculateGlobalPath();
  if(!success)
  {
    ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "Path computation failed."));
    ++failed_global_planner_count_;
    if(failed_global_planner_count_ > max_global_planner_tries_)
    {
      subt_planner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "Path computed successfully."));
    failed_global_planner_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus UpdateGlobalGoal::tick()
{
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "Updating active goal."));

  bool success = subt_planner_->updateGlobalGoal();
  if(!success)
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "Goal reached."));
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kGlobal, "Global Repositioning", "Goal tracking continues."));
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus Inspection::onStart()
{
  ROS_INFO("[Inspection] Triggered.");
  subt_planner_->in_srv_req_.bound_mode = std::min(failed_inspection_count_, 2);

  bool success = subt_planner_->getInspectionPath();
  if(!success)
  {
    ++failed_inspection_count_;
    if(failed_inspection_count_ > max_inspection_tries_)
    {
      subt_planner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
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

BT::NodeStatus CompartmentTransition::onStart()
{
  ROS_INFO("[CompartmentTransition] Triggered.");
  subt_planner_->in_srv_req_.bound_mode = std::min(failed_compartment_transition_count_, 2);

  bool success = subt_planner_->getCompartmentTransitionPath();
  ROS_WARN("Compartment transition returned %d", success);
  if(!success)
  {
    ++failed_compartment_transition_count_;
    if(failed_compartment_transition_count_ > max_compartment_transition_tries_)
    {
      subt_planner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    failed_compartment_transition_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus CompartmentTransition::onRunning()
{
  ROS_INFO("[CompartmentTransition] Running.");
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void CompartmentTransition::onHalted()
{
  std::cout << "[CompartmentTransition] Halted" << std::endl;
}

BT::NodeStatus Homing::tick()
{
  ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Triggered."));
  subt_planner_->in_srv_req_.bound_mode = std::min(failed_homing_count_, 2);

  bool success = subt_planner_->getHomingPath();
  if(!success)
  {
    ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Path request failed."));
    ++failed_homing_count_;
    if(failed_homing_count_ > max_homing_tries_)
    {
      subt_planner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Succeeded."));
    failed_homing_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus HomingCheck::tick()
{
  if(subt_planner_->bt_states_.homing_required)
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Required by mission state."));
    return BT::NodeStatus::SUCCESS;
  }
  
  bool homing_reqd = subt_planner_->homingRequired();
  if(homing_reqd)
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Required by planner status."));
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Not required."));
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus CalculateHomingPath::tick()
{
  ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Computing return path."));
  subt_planner_->in_srv_req_.bound_mode = std::min(failed_homing_count_, 2);

  bool success = subt_planner_->calculateHomingPath();
  if(!success)
  {
    ROS_WARN_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Path request failed."));
    ++failed_homing_count_;
    if(failed_homing_count_ > max_homing_tries_)
    {
      subt_planner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Return path computed."));
    failed_homing_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus UpdateHomingGoal::tick()
{
  ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Updating return goal."));

  bool success = subt_planner_->updateHomingGoal();
  if(!success)
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Return goal reached."));
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    ROS_INFO_STREAM(subt_terminal::tag(subt_terminal::Channel::kSafety, "Homing", "Return goal tracking continues."));
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus OPENINGPhase1::onStart()
{
  ROS_INFO("[OPENINGPhase1] Triggered.");
  subt_planner_->in_srv_req_.bound_mode = std::min(failed_opening_phase1_count_, 2);

  OpeningTraversalMode mode = OpeningTraversalMode::kGoingTo;
  OpeningTraversalStatus status;

  subt_planner_->getOpeningTraversalPath(mode, status);
  if(status == OpeningTraversalStatus::CANT_CONNECT)
  {
    ++failed_opening_phase1_count_;
    return BT::NodeStatus::FAILURE;
  }
  else if(status == OpeningTraversalStatus::OK)
  {
    failed_opening_phase1_count_ = 0;
    subt_planner_->bt_states_.opening_phase1_failed = false;
    return BT::NodeStatus::RUNNING;
  }
  else if(status == OpeningTraversalStatus::NO_OPENINGS)
  {
    subt_planner_->bt_states_.opening_phase1_failed = true;
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

  subt_planner_->getOpeningTraversalPath(mode, status);

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

BT::NodeStatus OPENINGP1FailCheck::tick()
{
  if(subt_planner_->bt_states_.opening_phase1_failed)
  {
    ROS_WARN("OPENING Phase1 Failed");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus OPENINGPhaseCheck::onStart()
{
  ROS_INFO("[OPENINGPhaseCheck] Triggered.");

  OpeningTraversalMode mode = OpeningTraversalMode::kPathCheck;
  OpeningTraversalStatus status;

  subt_planner_->getOpeningTraversalPath(mode, status);
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

BT::NodeStatus OPENINGPhase2::onStart()
{
  ROS_INFO("[OPENINGPhase2] Triggered.");

  OpeningTraversalMode mode = OpeningTraversalMode::kPassingThrough;
  OpeningTraversalStatus status;

  subt_planner_->getOpeningTraversalPath(mode, status);
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

BT::NodeStatus SetNextCompartment::tick()
{
  if(subt_planner_->transitionCompartment())
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus AllCompartmentsInspectedCheck::tick()
{
  if(subt_planner_->allCompartmentsInspected())
  {
    ROS_WARN("[AllCompartmentsInspectedCheck]");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus Idle::tick()
{
  subt_planner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;  
  subt_planner_->out_srv_res_.path.clear();
  return BT::NodeStatus::FAILURE;
}
