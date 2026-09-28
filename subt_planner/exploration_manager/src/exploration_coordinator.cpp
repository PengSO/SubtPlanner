#include "subt_planner/exploration_coordinator.h"

#include <nav_msgs/Path.h>


SubtPlanner::SubtPlanner(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  planner_status_ = SubtPlanner::PlannerStatus::NOT_READY;

  local_explorer_ = new LocalExplorationPlanner(nh, nh_private);
  if (!(local_explorer_->loadParams(false))) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Could not load all required parameters. Shutdown ROS node.");
    ros::shutdown();
  }

  initializeAttributes();
}

SubtPlanner::SubtPlanner(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private,
                     VoxelMap* voxel_map)
    : nh_(nh), nh_private_(nh_private) {
  
  planner_status_ = SubtPlanner::PlannerStatus::NOT_READY;
  local_explorer_ = new LocalExplorationPlanner(nh, nh_private, voxel_map);

  if (!(local_explorer_->loadParams(true))) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Could not load all required parameters. Shutdown ROS node.");
    ros::shutdown();
  }

  initializeAttributes();
}

void SubtPlanner::initializeAttributes() {
  planner_service_ = nh_.advertiseService(
      "subt_planner", &SubtPlanner::plannerServiceCallback, this);
  global_planner_service_ = nh_.advertiseService(
      "subt_planner/global", &SubtPlanner::globalPlannerServiceCallback, this);
  planner_homing_service_ = nh_.advertiseService(
      "subt_planner/homing", &SubtPlanner::homingServiceCallback, this);
  planner_set_homing_pos_service_ =
      nh_.advertiseService("subt_planner/set_homing_pos",
                           &SubtPlanner::setHomingPosServiceCallback, this);
  planner_search_service_ = nh_.advertiseService(
      "subt_planner/search", &SubtPlanner::plannerSearchServiceCallback, this);
  planner_passing_gate_service_ = nh_.advertiseService(
      "subt_planner/passing_gate", &SubtPlanner::passingGateCallback, this);
  planner_set_global_bound_service_ = nh_.advertiseService(
      "subt_planner/set_global_bound", &SubtPlanner::setGlobalBound, this);
  planner_set_dynamic_global_bound_service_ =
      nh_.advertiseService("subt_planner/set_dynamic_global_bound",
                           &SubtPlanner::setDynamicGlobalBound, this);
  planner_clear_exclusion_zones_service_ =
      nh_.advertiseService("subt_planner/clear_exclusion_zones",
                           &SubtPlanner::clearExclusionZones, this);
  planner_load_graph_service_ = nh_.advertiseService(
      "subt_planner/load_graph", &SubtPlanner::plannerLoadGraphCallback, this);
  planner_save_graph_service_ = nh_.advertiseService(
      "subt_planner/save_graph", &SubtPlanner::plannerSaveGraphCallback, this);
  planner_goto_wp_service_ =
      nh_.advertiseService("subt_planner/go_to_waypoint",
                           &SubtPlanner::plannerGotoWaypointCallback, this);
  planner_enable_exclusion_zone_subscriber_service_ =
      nh_.advertiseService(
          "subt_planner/enable_exclusion_zone_subscriber",
          &SubtPlanner::plannerEnableExclusionZoneSubscriberCallback,
          this);
  planner_set_planning_trigger_mode_service_ = nh_.advertiseService(
      "subt_planner/set_planning_trigger_mode",
      &SubtPlanner::plannerSetPlanningTriggerModeCallback, this);
      
  force_compartment_transition_service_ = nh_.advertiseService(
      "subt_planner/force_compartment_transition",
      &SubtPlanner::forceCompartmentChangeServiceCallback, this);
  
  pose_subscriber_ = nh_.subscribe("pose", 100, &SubtPlanner::poseCallback, this);
  pose_stamped_subscriber_ =
      nh_.subscribe("pose_stamped", 100, &SubtPlanner::poseStampedCallback, this);
  odometry_subscriber_ =
      nh_.subscribe("odometry", 100, &SubtPlanner::odometryCallback, this);
  robot_status_subcriber_ =
      nh_.subscribe("/robot_status", 1, &SubtPlanner::robotStatusCallback, this);
  exclusion_zone_subscriber_ =
      nh_.subscribe("/subt_planner/exclusion_zone_polygon", 100,
                    &SubtPlanner::exclusionZoneCallback, this);

  stop_srv_subscriber_ =
      nh_.subscribe("planner_control_interface/stop_request", 5, &SubtPlanner::stopMsgCallback, this);

  global_planner_local_goal_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("subt_planner/homing_local_goal", 10);

  std::string ns = ros::this_node::getName();
  planning_params_.loadParams(ns + "/PlanningParams");

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Compartment Centers:");
  if(global_verbosity >= Verbosity::DEBUG) {
    for(auto c : planning_params_.compartment_centers) {
      std::cout << c.transpose() << std::endl;
    }
  }
  
  planner_mode_ = PlannerMode::kExploration;
  

  BoundedSpaceParams exploration_bounds;
  local_explorer_->getGlobalBoundParams(exploration_bounds);
  local_explorer_->setExplorationBounds(exploration_bounds);
}

bool SubtPlanner::forceCompartmentChangeServiceCallback(
              std_srvs::Trigger::Request& req,
              std_srvs::Trigger::Response& res) {
  planner_mode_ = PlannerMode::kCompartmentChange;
  return true;
}

bool SubtPlanner::plannerGotoWaypointCallback(
    planner_msgs::planner_go_to_waypoint::Request& req,
    planner_msgs::planner_go_to_waypoint::Response& res) {
  res.path.clear();
  res.path = local_explorer_->getGlobalPath(req.waypoint);
  return true;
}

bool SubtPlanner::plannerEnableExclusionZoneSubscriberCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  if (static_cast<bool>(request.data)) {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "SubtPlanner enables exclusion-zone checking");
    exclusion_zone_subscriber_ =
        nh_.subscribe("/subt_planner/exclusion_zone_polygon", 100,
                      &SubtPlanner::exclusionZoneCallback, this);
  } else {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "SubtPlanner disables exclusion-zone checking");
    exclusion_zone_subscriber_.shutdown();
  }
  response.success = static_cast<unsigned char>(true);
  return true;
}

bool SubtPlanner::plannerLoadGraphCallback(
    planner_msgs::planner_string_trigger::Request& req,
    planner_msgs::planner_string_trigger::Response& res) {
  res.success = local_explorer_->loadGraph(req.message);
  return true;
}

bool SubtPlanner::plannerSaveGraphCallback(
    planner_msgs::planner_string_trigger::Request& req,
    planner_msgs::planner_string_trigger::Response& res) {
  res.success = local_explorer_->saveGraph(req.message);
  return true;
}

bool SubtPlanner::setGlobalBound(
    planner_msgs::planner_set_global_bound::Request& req,
    planner_msgs::planner_set_global_bound::Response& res) {
  if (!req.get_current_bound)
    res.success = local_explorer_->setGlobalBound(req.bound, req.reset_to_default);
  else
    res.success = true;

  local_explorer_->getGlobalBound(res.bound_ret);
  return true;
}

bool SubtPlanner::setDynamicGlobalBound(
    planner_msgs::planner_dynamic_global_bound::Request& req,
    planner_msgs::planner_dynamic_global_bound::Response& res) {
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Updating exploration bound");
  res.success = local_explorer_->setGlobalBound(req);
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Exploration bound update returned");

  return true;
}

void SubtPlanner::setGeofenceManager(
    std::shared_ptr<GeofenceManager> geofence_manager) {
  local_explorer_->setGeofenceManager(geofence_manager);
}

void SubtPlanner::setSharedParams(const RobotParams& robot_params,
                                const BoundedSpaceParams& global_space_params) {
  local_explorer_->setSharedParams(robot_params, global_space_params);
}

void SubtPlanner::setSharedParams(const RobotParams& robot_params,
                                const BoundedSpaceParams& global_space_params,
                                const BoundedSpaceParams& local_space_params) {
  local_explorer_->setSharedParams(robot_params, global_space_params, local_space_params);
}

bool SubtPlanner::passingGateCallback(
    planner_msgs::planner_request_path::Request& req,
    planner_msgs::planner_request_path::Response& res) {
  res.path = local_explorer_->searchPathToPassGate();
  res.bound.mode = res.bound.kExtendedBound;
  return true;
}

bool SubtPlanner::plannerServiceCallback(
    planner_msgs::planner_srv::Request& req,
    planner_msgs::planner_srv::Response& res) {

  res.is_global_fallback_path = false;

  local_explorer_->reset();
  
  if(planning_params_.exploration_only) {
    if(planning_params_.enable_opening_traversal) {
      if(exploration_counter_ >= planning_params_.max_exploration_iterations) {
        opening_traversal_requested_ = true;
        exploration_counter_ = 0;
        ROS_WARN("Exploration Iterations Complete, switching to: %d", (int)planner_mode_);
      }
      return getExplorationPath(req, res);
    }
    else {
      return getExplorationPath(req, res);
    }
  }

  ROS_WARN("Current Planner Mode: %d", (int)planner_mode_);

  bool success = true;
  std::vector<geometry_msgs::Pose> empty_path;
  switch (planner_mode_) {
    case PlannerMode::kExploration:
    {
      if(exploration_counter_ >= planning_params_.max_exploration_iterations) {
        res.path = empty_path;
        res.status = planner_msgs::planner_srv::Response::kForward;
        planner_mode_ = PlannerMode::kCompartmentChange;
        success = true;
        exploration_counter_ = 0;
        ROS_WARN("Exploration Iterations Complete, switching to: %d", (int)planner_mode_);
        break;
      }
      success = getExplorationPath(req, res);
      if(success)
      {
        ++exploration_counter_;
        ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Exploration Counter: %d", exploration_counter_);
        if(res.status != planner_msgs::planner_srv::Response::kForward) {
          res.path = empty_path;
          res.status = planner_msgs::planner_srv::Response::kForward;
          planner_mode_ = PlannerMode::kCompartmentChange;
          success = true;
          exploration_counter_ = 0;
          ROS_WARN("Exploration Complete, switching to: %d", (int)planner_mode_);
        }
      }
      else
      {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Exploration Failed trying again:");
      }
      
      break;
    }

    case PlannerMode::kExplorationComplete:
    {
      res.path = empty_path;
      res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      planner_mode_ = PlannerMode::kCompartmentChange;
      ROS_WARN("Switching to: %d", (int)planner_mode_);
      break;
    }

    case PlannerMode::kCompartmentChange:
    {
      if(compartment_counter_ >= planning_params_.compartment_centers.size()-1) {
        res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
        success = true;
        compartment_change_tries_ = 0;
        ROS_WARN("All compartments explored");
        if(planning_params_.auto_homing_enable)
          res.path = local_explorer_->getHomingPath("world");
        else
          res.path = empty_path;
        break;
      }

      bool search_success = getCompartmentTransitionPath(req, res);
      
      if(search_success) {
        compartment_change_tries_ = 0;
        success = true;
      }
      else {
        success = true;
        ++compartment_change_tries_;
        ROS_WARN_COND(global_verbosity>=Verbosity::WARN, "Path search failed. Try %d", compartment_change_tries_);
        if(compartment_change_tries_ <= max_compartment_change_tries_) {
          --compartment_counter_;
        }
      }
      local_explorer_->reset();
      
      break;
    }
    
    default:
      success = getExplorationPath(req, res);
      break;
  }

  return success;
}

LocalExplorationPlanner::LocalPlannerStatus SubtPlanner::getExplorationPath()
{
  const bool success = getExplorationPath(in_srv_req_, out_srv_res_);
  if (!success) {
    return LocalExplorationPlanner::LocalPlannerStatus::L_ERR;
  }
  if (out_srv_res_.status != planner_msgs::planner_srv::Response::kForward ||
      out_srv_res_.is_global_fallback_path) {
    return LocalExplorationPlanner::LocalPlannerStatus::L_EXHAUSTED;
  }
  return LocalExplorationPlanner::LocalPlannerStatus::L_OK;
}

bool SubtPlanner::getExplorationPath(planner_msgs::planner_srv::Request& req,
                                     planner_msgs::planner_srv::Response& res)
{
  local_explorer_->setGlobalFrame(req.header.frame_id);
  local_explorer_->setBoundMode(static_cast<BoundModeType>(req.bound_mode));
  local_explorer_->setRootStateForPlanning(req.root_pose);

  res.path.clear();
  res.is_global_fallback_path = false;
  if (getPlannerStatus() == SubtPlanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    res.status = planner_msgs::planner_srv::Response::kForward;
    return false;
  }

  local_explorer_->reset();

  LocalExplorationPlanner::GraphStatus build_status =
      LocalExplorationPlanner::GraphStatus::NOT_OK;
  if (planning_params_.graph_building_mode == GraphBuildingModeType::kBasic) {
    build_status = local_explorer_->buildGraph();
  } else if (planning_params_.graph_building_mode == GraphBuildingModeType::kBatch) {
    build_status = local_explorer_->batchGraph();
  }

  if (build_status != LocalExplorationPlanner::GraphStatus::OK) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "[PLANNER_ERROR] Local graph build failed with status %d",
                  static_cast<int>(build_status));
    if (build_status == LocalExplorationPlanner::GraphStatus::ERR_NO_FEASIBLE_PATH &&
        planning_params_.auto_global_planner_enable) {
      int global_status;
      res.path = local_explorer_->runGlobalPlanner(0, false, false, global_status);
      res.status = global_status;
      res.is_global_fallback_path = !res.path.empty() && global_status >= 0;
      return !res.path.empty() ||
             global_status == planner_msgs::planner_srv::Response::kHoming;
    }
    res.status = planner_msgs::planner_srv::Response::kForward;
    return false;
  }

  const LocalExplorationPlanner::GraphStatus eval_status =
      local_explorer_->evaluateGraph();
  if (eval_status == LocalExplorationPlanner::GraphStatus::OK) {
    res.path = local_explorer_->getBestPath(req.header.frame_id, res.status);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                  "[SUBTPLANNER] Regular local exploration path status: %d",
                  res.status);
    return true;
  }

  if (planning_params_.auto_global_planner_enable &&
      (eval_status == LocalExplorationPlanner::GraphStatus::CONSEC_LOW_GAIN ||
       eval_status == LocalExplorationPlanner::GraphStatus::NO_GAIN)) {
    int global_status;
    res.path = local_explorer_->runGlobalPlanner(0, false, false, global_status);
    res.status = global_status;
    res.is_global_fallback_path = !res.path.empty() && global_status >= 0;
    return !res.path.empty() ||
           global_status == planner_msgs::planner_srv::Response::kHoming;
  }

  res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
  return true;
}

LocalExplorationPlanner::GlobalPlannerStatus SubtPlanner::getGlobalExplorationPath()
{
  local_explorer_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));

  int status;
  out_srv_res_.path = local_explorer_->runGlobalPlanner(0, false, false, status);
  out_srv_res_.status = status;
  out_srv_res_.is_global_fallback_path = !out_srv_res_.path.empty() && status >= 0;
  if(status == planner_msgs::planner_srv::Response::kHoming)
  {
    bt_states_.global_exp_exhausted = true;
    return LocalExplorationPlanner::GlobalPlannerStatus::G_HOMING;
  }

  if(out_srv_res_.path.empty())
  {
    return LocalExplorationPlanner::GlobalPlannerStatus::G_ERR;
  }
  else
  {
    if(status == planner_msgs::planner_srv::Response::kHoming)
    {
      return LocalExplorationPlanner::GlobalPlannerStatus::G_HOMING;
    }
  }
  return LocalExplorationPlanner::GlobalPlannerStatus::G_OK;
}

bool SubtPlanner::checkGlobalExplorationStatus()
{
  int status;
  std::vector<geometry_msgs::Pose> path = local_explorer_->runGlobalPlanner(0, false, false, status);
  if(status == planner_msgs::planner_srv::Response::kHoming)
  {
    bt_states_.global_exp_exhausted = true;
    return true;
  }

  return false;
}

bool SubtPlanner::transitionCompartment()
{
  return getCompartmentTransitionPath();
}

bool SubtPlanner::allCompartmentsInspected()
{
  return compartment_counter_ >=
         static_cast<int>(planning_params_.compartment_centers.size()) - 1;
}

void SubtPlanner::getOpeningTraversalPath(OpeningTraversalMode mode, OpeningTraversalStatus &status)
{
  local_explorer_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.path = local_explorer_->getOpeningTraversalPath(mode, status);
  out_srv_res_.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
  if(status != OpeningTraversalStatus::OK)
  {
    out_srv_res_.path.clear();
  }
}

bool SubtPlanner::getCompartmentTransitionPath() {
  local_explorer_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));

  std::vector<geometry_msgs::Pose> empty_path;

  BoundedSpaceParams og_global_bb;
  local_explorer_->getGlobalBoundParams(og_global_bb);
  BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
  Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d max_extension = planning_params_.compartment_dimensions.max_extension + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
  Eigen::Vector3d min_extension = planning_params_.compartment_dimensions.min_extension + planning_params_.compartment_centers[compartment_counter_];
  translated_bound.setBound(min_val, max_val);
  BoundedSpaceParams extended_bound = planning_params_.compartment_dimensions;
  max_val = planning_params_.compartment_dimensions.max_val * 2.0;
  max_extension = planning_params_.compartment_dimensions.max_extension * 2.0;
  min_val = planning_params_.compartment_dimensions.min_val * 2.0;
  min_extension = planning_params_.compartment_dimensions.min_extension * 2.0;
  max_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
  max_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
  min_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
  min_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
  extended_bound.setBound(min_val, max_val);
  local_explorer_->setExplorationBounds(extended_bound);
  local_explorer_->reset();

  geometry_msgs::Pose current_pose;
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, current_state_[3]);
  tf::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
  tf::Pose poseTF(quat, origin);
  tf::poseTFToMsg(poseTF, current_pose);

  geometry_msgs::Pose target_pose;
  quat.setEuler(0.0, 0.0, 0.0);
  origin = tf::Vector3(planning_params_.compartment_centers[compartment_counter_][0], planning_params_.compartment_centers[compartment_counter_][1], planning_params_.compartment_centers[compartment_counter_][2]);
  tf::Pose poseTF_target(quat, origin);
  tf::poseTFToMsg(poseTF_target, target_pose);

  std::vector<geometry_msgs::Pose> connecting_path;
  bool search_success = local_explorer_->search(current_pose, target_pose, true, connecting_path);
  if(search_success) {
    local_explorer_->setExplorationBounds(translated_bound);
    out_srv_res_.path = connecting_path;
    out_srv_res_.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
  }
  else {
    out_srv_res_.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
    out_srv_res_.path = empty_path;
  }

  return search_success;
}

bool SubtPlanner::getCompartmentTransitionPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res) {
  std::vector<geometry_msgs::Pose> empty_path;

  if(planning_params_.enable_opening_traversal) {
    local_explorer_->setNextCompartmentCenter(planning_params_.compartment_centers[compartment_counter_+1]);
    local_explorer_->setNextCompartmentIndex(compartment_counter_+1);
    res.path = local_explorer_->getOpeningTraversalPath();
    res.is_global_fallback_path = !res.path.empty();
    if(local_explorer_->autoOpeningPathApproval()) {
      res.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
    }
    else {
      res.status = planner_msgs::planner_srv::Response::kManualCustomPath;
    }

    if(!local_explorer_->openingTraversalOngoing()) {
      planner_mode_ = PlannerMode::kExploration;
      ++compartment_counter_;
      BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
      Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
      Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
      translated_bound.setBound(min_val, max_val);
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
      local_explorer_->setExplorationBounds(translated_bound);
    }
    else {
      planner_mode_ = PlannerMode::kCompartmentChange;
    }

    bool success = false;
    if(res.path.empty()) {
      if(local_explorer_->openingTraversalOngoing()) {
        success = true;
      }
    }
    else {
      success = true;
    }

    return success;
  }
  else {
    ++compartment_counter_;
    BoundedSpaceParams translated_bound = planning_params_.compartment_dimensions;
    Eigen::Vector3d max_val = planning_params_.compartment_dimensions.max_val + planning_params_.compartment_centers[compartment_counter_];
    Eigen::Vector3d max_extension = planning_params_.compartment_dimensions.max_extension + planning_params_.compartment_centers[compartment_counter_];
    Eigen::Vector3d min_val = planning_params_.compartment_dimensions.min_val + planning_params_.compartment_centers[compartment_counter_];
    Eigen::Vector3d min_extension = planning_params_.compartment_dimensions.min_extension + planning_params_.compartment_centers[compartment_counter_];
    translated_bound.setBound(min_val, max_val);
    BoundedSpaceParams extended_bound = planning_params_.compartment_dimensions;
    max_val = planning_params_.compartment_dimensions.max_val * 2.0;
    max_extension = planning_params_.compartment_dimensions.max_extension * 2.0;
    min_val = planning_params_.compartment_dimensions.min_val * 2.0;
    min_extension = planning_params_.compartment_dimensions.min_extension * 2.0;
    max_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    max_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    min_val += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    min_extension += (planning_params_.compartment_centers[compartment_counter_] + planning_params_.compartment_centers[compartment_counter_-1]) / 2.0;
    extended_bound.setBound(min_val, max_val);
    local_explorer_->setExplorationBounds(extended_bound);
    local_explorer_->reset();

    geometry_msgs::Pose current_pose;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, current_state_[3]);
    tf::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
    tf::Pose poseTF(quat, origin);
    tf::poseTFToMsg(poseTF, current_pose);

    geometry_msgs::Pose target_pose;
    quat.setEuler(0.0, 0.0, 0.0);
    origin = tf::Vector3(planning_params_.compartment_centers[compartment_counter_][0], planning_params_.compartment_centers[compartment_counter_][1], planning_params_.compartment_centers[compartment_counter_][2]);
    tf::Pose poseTF_target(quat, origin);
    tf::poseTFToMsg(poseTF_target, target_pose);

    std::vector<geometry_msgs::Pose> connecting_path;
    bool search_success = local_explorer_->search(current_pose, target_pose, true, connecting_path);
    if(search_success) {
      local_explorer_->setExplorationBounds(translated_bound);
      res.path = connecting_path;
      res.is_global_fallback_path = !res.path.empty();
      res.status = planner_msgs::planner_srv::Response::kForward;
      planner_mode_ = PlannerMode::kExploration;
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Compartment counter: %d", compartment_counter_);
    }
    else {
      res.status = planner_msgs::planner_srv::Response::kForward;
      res.path = empty_path;
    }
    return search_success;
  }
}

bool SubtPlanner::homingRequired()
{
  local_explorer_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.status = planner_msgs::planner_srv::Response::kHoming;
  return local_explorer_->homingRequired(out_srv_res_.path);
}

bool SubtPlanner::getHomingPath()
{
  if (getPlannerStatus() == SubtPlanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    out_srv_res_.status = planner_msgs::planner_srv::Response::kForward;
    return false;
  }

  local_explorer_->setBoundMode(static_cast<BoundModeType>(in_srv_req_.bound_mode));
  out_srv_res_.path = local_explorer_->getHomingPath(in_srv_req_.header.frame_id);
  if(out_srv_res_.path.empty())
  {
    out_srv_res_.status = planner_msgs::planner_srv::Response::kForward;
    return false;  
  }

  out_srv_res_.status = planner_msgs::planner_srv::Response::kHoming;
  return true;
}

bool SubtPlanner::homingServiceCallback(
    planner_msgs::planner_homing::Request& req,
    planner_msgs::planner_homing::Response& res) {
  ROS_WARN("Homing through direct service call");
  res.path.clear();
  if (getPlannerStatus() == SubtPlanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return false;
  }
  res.path = local_explorer_->getHomingPath(req.header.frame_id);
  return true;
}

bool SubtPlanner::globalPlannerServiceCallback(
    planner_msgs::planner_global::Request& req,
    planner_msgs::planner_global::Response& res) {
  res.path.clear();
  if (getPlannerStatus() == SubtPlanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return false;
  }
  int status;
  res.path =
      local_explorer_->runGlobalPlanner(req.id, req.not_check_frontier, req.ignore_time, status);
  return true;
}

bool SubtPlanner::setHomingPosServiceCallback(
    planner_msgs::planner_set_homing_pos::Request& req,
    planner_msgs::planner_set_homing_pos::Response& res) {
  if (getPlannerStatus() == SubtPlanner::PlannerStatus::NOT_READY) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "The planner is not ready.");
    return false;
  }
  res.success = local_explorer_->setHomingPos();
  return true;
}

bool SubtPlanner::plannerSearchServiceCallback(
    planner_msgs::planner_search::Request& req,
    planner_msgs::planner_search::Response& res) {
  local_explorer_->setBoundMode(static_cast<BoundModeType>(req.bound_mode));
  res.success =
      local_explorer_->search(req.source, req.target, req.use_current_state, res.path);
  return true;
}

bool SubtPlanner::plannerSetPlanningTriggerModeCallback(
    planner_msgs::planner_set_planning_mode::Request& request,
    planner_msgs::planner_set_planning_mode::Response& response) {
  PlannerTriggerModeType in_trig_mode;
  if (request.planning_mode == request.kAuto)
    in_trig_mode = PlannerTriggerModeType::kAuto;
  else if (request.planning_mode == request.kManual)
    in_trig_mode = PlannerTriggerModeType::kManual;
  local_explorer_->setPlannerTriggerMode(in_trig_mode);
  response.success = true;
  return true;
}

bool SubtPlanner::clearExclusionZones(std_srvs::Trigger::Request& req,
                                      std_srvs::Trigger::Response& res) {
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Clearing exclusion zones");
  local_explorer_->clearExclusionZones();
  res.success = true;
  return true;
}

void SubtPlanner::exclusionZoneCallback(
    const geometry_msgs::PolygonStamped& polygon_msgs) {
  if (!polygon_msgs.polygon.points.empty()) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Detected exclusion zone");
    local_explorer_->addGeofenceAreas(polygon_msgs);
  }
}

void SubtPlanner::setExclusionZonePolygon(
    const geometry_msgs::PolygonStamped& polygon_msgs) {
  std::cout << "Exclusion-zone polygon size: "
            << polygon_msgs.polygon.points.size() << std::endl;
  if (!polygon_msgs.polygon.points.empty()) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Detected exclusion zone");
    local_explorer_->addGeofenceAreas(polygon_msgs);
  }
}

void SubtPlanner::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped& pose) {
  processPose(pose.pose.pose);
}

void SubtPlanner::poseStampedCallback(const geometry_msgs::PoseStamped& pose) {
  processPose(pose.pose);
}

void SubtPlanner::processPose(const geometry_msgs::Pose& pose) {
  StateVec state;
  state[0] = pose.position.x;
  state[1] = pose.position.y;
  state[2] = pose.position.z;
  state[3] = tf::getYaw(pose.orientation);
  local_explorer_->setState(state);
  current_state_ = state;
}

void SubtPlanner::odometryCallback(const nav_msgs::Odometry& odo) {
  StateVec state;
  state[0] = odo.pose.pose.position.x;
  state[1] = odo.pose.pose.position.y;
  state[2] = odo.pose.pose.position.z;
  state[3] = tf::getYaw(odo.pose.pose.orientation);
  local_explorer_->setState(state);
  current_state_ = state;
}

void SubtPlanner::robotStatusCallback(const planner_msgs::RobotStatus& status) {
  local_explorer_->setTimeRemaining(status.time_remaining);
}

void SubtPlanner::stopMsgCallback(const std_msgs::Bool& msg)
{
  bt_states_.homing_required = false;
}

SubtPlanner::PlannerStatus SubtPlanner::getPlannerStatus() {

  if (planner_status_ == SubtPlanner::PlannerStatus::READY)
    return SubtPlanner::PlannerStatus::READY;

  return SubtPlanner::PlannerStatus::READY;
}

// }  // namespace explorer
