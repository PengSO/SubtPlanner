#include "local_exploration/local_exploration_planner.h"

#include <algorithm>
#include <random>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

RobotStateHistory::RobotStateHistory() {
  kd_tree_ = NULL;
  reset();
}

void RobotStateHistory::reset() {
  if (kd_tree_) kd_free(kd_tree_);
  kd_tree_ = kd_create(3);
}

void RobotStateHistory::addState(StateVec* s) {
  kd_insert3(kd_tree_, s->x(), s->y(), s->z(), s);
  state_hist_.push_back(s);
}

bool RobotStateHistory::getNearestState(const StateVec* state,
                                        StateVec** s_res) {
  if (state_hist_.empty()) return false;
  kdres* nearest = kd_nearest3(kd_tree_, state->x(), state->y(), state->z());
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return false;
  }
  *s_res = static_cast<StateVec*>(kd_res_item_data(nearest));
  kd_res_free(nearest);
  return true;
}

bool RobotStateHistory::getNearestStates(const StateVec* state, double range,
                                         std::vector<StateVec*>* s_res) {
  if (state_hist_.empty()) return false;
  kdres* neighbors =
      kd_nearest_range3(kd_tree_, state->x(), state->y(), state->z(), range);
  const int neighbors_size = kd_res_size(neighbors);
  if (neighbors_size <= 0) {
    kd_res_free(neighbors);
    return false;
  }

  s_res->clear();
  for (int i = 0; i < neighbors_size; ++i) {
    StateVec* new_neighbor = static_cast<StateVec*>(kd_res_item_data(neighbors));
    s_res->push_back(new_neighbor);
    if (kd_res_next(neighbors) <= 0) break;
  }
  kd_res_free(neighbors);
  return true;
}

bool RobotStateHistory::getNearestStateInRange(const StateVec* state,
                                               double range, StateVec** s_res) {
  if (!getNearestState(state, s_res)) return false;

  const Eigen::Vector3d dist(state->x() - (*s_res)->x(),
                             state->y() - (*s_res)->y(),
                             state->z() - (*s_res)->z());
  return dist.norm() <= range;
}

LocalExplorationPlanner::LocalExplorationPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  
  voxel_map_ = new VoxelMap(nh_, nh_private_);

  repositioning_region_ = new RepositioningRegion(voxel_map_);

  initializeAttributes();
}
LocalExplorationPlanner::LocalExplorationPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
         VoxelMap* voxel_map)
    : nh_(nh), nh_private_(nh_private),
      voxel_map_(voxel_map) {
  
  repositioning_region_ = new RepositioningRegion(voxel_map_);

  initializeAttributes();
}

void LocalExplorationPlanner::initializeAttributes() {
  visualization_ = new MissionVisuals(nh_, nh_private_);
  geofence_manager_.reset(new GeofenceManager());

  global_graph_.reset(new CognitiveGraph());                       
  local_graph_.reset(new CognitiveGraph());

  robot_state_hist_.reset(new RobotStateHistory());

  stat_.reset(new SampleStatistic());
  stat_chrono_.reset(new SampleStatistic());

  planner_trigger_count_ = 0;
  current_battery_time_remaining_ = std::numeric_limits<double>::max();
  rostime_start_ = ros::Time::now();
  add_frontiers_to_global_graph_ = false;
  exploring_direction_ = 0.0;

  odometry_ready = false;
  last_state_marker_ << 0, 0, 0, 0, 0;
  last_state_marker_global_ << 0, 0, 0, 0, 0;
  robot_backtracking_prev_ = NULL;

  planner_trigger_mode_ = PlannerTriggerModeType::kManual;

  num_low_gain_iters_ = 0;
  auto_global_planner_trig_ = false;
  dead_zone_escape_pending_ = false;
  latest_dead_zone_vertex_id_ = -1;
  dead_zone_vertex_ids_.clear();
  branch_repositioning_point_time_ema_ = -1.0;
  branch_repositioning_region_time_ema_ = -1.0;
  branch_repositioning_cache_ready_ = false;
  global_exploration_ongoing_ = false;
  current_global_vertex_id_ = 0;
  local_exploration_ongoing_ = false;

  Eigen::Vector3d zero_vec = Eigen::Vector3d::Zero();
  inspection_bound_.setCenter(zero_vec, true);
  inspection_bound_.setRotation(zero_vec);

  pci_reset_pub_ = nh_.advertise<std_msgs::Bool>("planner_control_interface/msg/reset", 10);
  local_free_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("subt_planner/local_free_cloud", 10);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/subt_planner_path", 10);
  free_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("freespace_pointcloud", 10);
  entry_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("subt_planner/entry_point_viz", 10);
  local_target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("subt_planner/local_target_viz", 10);
  time_log_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("subt_planner/local_exploration/time_log", 10);
  ablation_log_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("subt_planner/local_exploration/ablation_log", 10);

  periodic_timer_ = nh_.createTimer(ros::Duration(kTimerPeriod), &LocalExplorationPlanner::timerCallback, this);
  topological_map_update_timer_ = nh_.createTimer(ros::Duration(kTopologicalMapUpdateTimerPeriod), &LocalExplorationPlanner::expandTopologicalMapTimerCallback, this);
  topological_frontier_addition_timer_ = nh_.createTimer(ros::Duration(kTopologicalFrontierAdditionTimerPeriod), &LocalExplorationPlanner::expandTopologicalFrontierAdditionTimerCallback, this);
  camera_annotation_timer_ = nh_.createTimer(ros::Duration(0.1), &LocalExplorationPlanner::cameraAnnotationTimerCallback, this);

  semantics_subscriber_ = nh_.subscribe("semantic_location", 100, &LocalExplorationPlanner::semanticsCallback, this);
  stop_srv_subscriber_ = nh_.subscribe("planner_control_interface/stop_request", 100, &LocalExplorationPlanner::stopMsgCallback, this);
  opening_detection_sub_ = nh_.subscribe("opening_detections", 100, &LocalExplorationPlanner::openingDetectionCallback, this);
  query_pt_sub_ = nh_.subscribe("query_point", 1, &LocalExplorationPlanner::queryPtCallback, this);
  cam_pitch_sub_ = nh_.subscribe("cam_pitch", 1, &LocalExplorationPlanner::camPitchCallback, this);

  pci_homing_ = nh_.serviceClient<std_srvs::Trigger>("planner_control_interface/std_srvs/homing_trigger");
  landing_srv_client_ = nh_.serviceClient<std_srvs::Empty>("land_srv");

  reset_timer_srv_ = nh_.advertiseService("subt_planner/reset_timer", &LocalExplorationPlanner::resetTimerCallback, this);
  pass_opening_srv_ = nh_.advertiseService("get_opening_traversal_path", &LocalExplorationPlanner::getOpeningPathCallback, this);
  approve_passing_srv_ = nh_.advertiseService("approve_opening_traversal", &LocalExplorationPlanner::approvePassingCallback, this);
  reset_map_srv_ = nh_.advertiseService("reset_map", &LocalExplorationPlanner::resetMapCallback, this);
  query_srv_ = nh_.advertiseService("query_srv", &LocalExplorationPlanner::queryCallback, this);
  remove_geofence_srv_ = nh_.advertiseService("remove_geofence", &LocalExplorationPlanner::removeGeofenceCallback, this);

  listener_ = new tf::TransformListener();
  next_compartment_ << std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::max();
}

void LocalExplorationPlanner::reset() {
  if (add_frontiers_to_global_graph_) {
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Reset: Adding frontiers to topological map");
    add_frontiers_to_global_graph_ = false;
    addFrontiers(0);  // id given as 0 because it is not used
  }
  if (local_graph_ != NULL) local_graph_->reset();
  local_graph_.reset(new CognitiveGraph());
  local_graph_rep_.reset();

  auto_global_planner_trig_ = false;
  stat_.reset(new SampleStatistic());
  stat_chrono_.reset(new SampleStatistic());
  StateVec root_state;
  if (planning_params_.use_current_state)
    root_state = current_state_;
  else {
    root_state = state_for_planning_;
  }
  stat_->init(root_state);
  stat_chrono_->init(root_state);
  root_vertex_ = new Vertex(local_graph_->generateVertexID(), root_state);
  local_graph_->addVertex(root_vertex_);

  if ((planner_trigger_count_ == 0) && (global_graph_->getNumVertices() == 0)) {
    Vertex* g_root_vertex =
        new Vertex(global_graph_->generateVertexID(), root_state);
    global_graph_->addVertex(g_root_vertex);
  }
  VoxelStatus voxel_state = voxel_map_->getBoxStatus(
      Eigen::Vector3d(root_state[0], root_state[1], root_state[2]) +
          robot_params_.center_offset,
      robot_box_size_, true);
  if (VoxelStatus::kFree != voxel_state) {
    switch (voxel_state) {
      case VoxelStatus::kFree:
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Current box is Free.");
        break;
      case VoxelStatus::kOccupied:
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Current box contains Occupied voxels.");
        break;
      case VoxelStatus::kUnknown:
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Current box contains Unknown voxels.");
        break;
    }
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Starting position is not clear--> clear space around the robot.");
    const double startup_reach = std::max(
        planning_params_.edge_length_min,
        std::min(planning_params_.edge_length_max,
                 std::max(planning_params_.nearest_range_min,
                          planning_params_.nearest_range)));
    augmentFreeStartupArea(root_state, startup_reach);
  }
  voxel_map_->setRobotRadius(robot_box_size_.maxCoeff() / 2.0);
  voxel_map_->setBoxCheckMethod(planning_params_.box_check_method);
  voxel_map_->setLineCheckMethod(planning_params_.line_check_method);
  if (planning_params_.free_frustum_before_planning) {
    voxel_map_->augmentFreeFrustum();
  }
  visualization_->visualizeRobotState(root_vertex_->state, robot_params_);
  visualization_->visualizeSensorFOV(root_vertex_->state, sensor_params_);

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_adaptive_params_);
  } else {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_space_params_);
  }
  visualization_->visualizeNoGainZones(no_gain_zones_);
}

bool LocalExplorationPlanner::resetMapCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  voxel_map_->resetMap();
  res.success = true;
  return true;
}

bool LocalExplorationPlanner::queryCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  std::shared_ptr<CognitiveGraph> graph;
  graph.reset(new CognitiveGraph());
  Vertex* root_vertex = new Vertex(graph->generateVertexID(), current_state_);
  graph->addVertex(root_vertex);

  std::vector<StateVec> all_viewpoints;
  all_viewpoints.push_back(current_state_);

  for(double x=1.0; x<=2.0; x+=1)
  {
    for(double y=1.0; y<=2.0; y+=1)
    {
      StateVec s;
      s << x,y,0.0,0.0,0.0;
      s += current_state_;
      truncateAngle(s(3));
      Vertex* v = new Vertex(graph->generateVertexID(), s);
      for(int i=0; i<3; ++i)
      {
        StateVec subs = s;
        subs(3) = y + i*M_PI/3.0;
        truncateAngle(subs(3));
        Vertex* subv = new Vertex(i, subs);
        v->orientation_sub_vertices.push_back(subv);
        all_viewpoints.push_back(subs);
      }
      graph->addVertex(v);
    }
  }

  for(auto v : graph->vertices_map_)
  {
    for(auto nv : graph->vertices_map_)
    {
      if(v.second->id != nv.second->id)
      {
        graph->addEdge(v.second, nv.second,
          (v.second->state.head(3) - nv.second->state.head(3)).norm());
      }
    }
  }

  visualization_->visualizeViewpoints(all_viewpoints);

  std::map<int, ShortestPathsReport> path_rep_map;
  for(int i=0; i<graph->vertices_map_.size(); ++i) {
    ShortestPathsReport rep;
    graph->findShortestPaths(i, rep);
    path_rep_map[i] = rep;
  }

  std::vector<int> tsp_order_vert;
  std::vector<std::pair<int, std::vector<int>>> tsp_order;
  std::vector<int> empty_vec_int;
  tsp_order.push_back(std::make_pair(root_vertex->id, empty_vec_int));
  for(int i=1; i<graph->vertices_map_.size(); ++i)
  {
    tsp_order_vert.push_back(i);
    std::vector<int> subv_ids;
    for(auto subv : graph->vertices_map_[i]->orientation_sub_vertices)
    {
      subv_ids.push_back(subv->id);
    }
    tsp_order.push_back(std::make_pair(i, subv_ids));
  }
  std::vector<geometry_msgs::Pose> out_path = connectTSPOrderWithSubvertices(tsp_order, path_rep_map, graph);
  nav_msgs::Path out_path_vis;
  out_path_vis.header.frame_id = world_frame_;
  for (auto p : out_path) {
    geometry_msgs::PoseStamped ps;
    ps.pose = p;
    out_path_vis.poses.push_back(ps);
  }
  path_pub_.publish(out_path_vis);

  visualization_->visualizeRefPath(out_path);

  return true;
}

void LocalExplorationPlanner::clear() {}

void LocalExplorationPlanner::openingDetectionCallback(const planner_msgs::MultipleOpeningDetections &detections) {
	std::map<int, std::shared_ptr<Opening>> remaining_detected_openings;
  
  for(auto det : detections.multiple_detections)
	{
    auto itr = detected_openings_.find(det.id);
    if(itr != detected_openings_.end()) 
    {
      itr->second->pose = det.pose;
      remaining_detected_openings[det.id] = itr->second;
    }
    else
    {
      std::shared_ptr<Opening> new_opening;
      new_opening.reset(new Opening());
      new_opening->id = det.id;
      new_opening->pose = det.pose;
      detected_openings_[det.id] = new_opening;
      remaining_detected_openings[det.id] = new_opening;
      ROS_INFO("\033[1;32m[Local Exploration]\033[0m New opening detected: %d", det.id);
    }
	}

  detected_openings_.clear();

  for(auto det : remaining_detected_openings)
  {
    detected_openings_[det.first] = det.second;
  }

}

void LocalExplorationPlanner::stopMsgCallback(const std_msgs::Bool& msg) {
  global_exploration_ongoing_ = false;
  auto_global_planner_trig_ = false;
  opening_traversal_mode_ = OpeningTraversalMode::kNone;
}

void LocalExplorationPlanner::camPitchCallback(const sensor_msgs::JointState &state)
{
  cam_pitch_ = state.position[0];
}

void LocalExplorationPlanner::queryPtCallback(const geometry_msgs::PoseStamped& pose)
{
  StateVec query_state;
  convert(pose.pose, query_vec_);
}

bool LocalExplorationPlanner::loadParams(bool shared_params) {
  std::string ns = ros::this_node::getName();
  if (!sensor_params_.loadParams(ns + "/SensorParams")) return false;

  if (!planning_params_.loadParams(ns + "/PlanningParams")) return false;
  world_frame_ = planning_params_.global_frame_id;
  if (planning_params_.annotate_map_with_camera ||
      planning_params_.use_camera_gain || planning_params_.inspection_planning) {
    camera_annotation_params_.loadParams(ns + "/CameraAnnotationParams");
  }

  if (planning_params_.free_frustum_before_planning ||
      planning_params_.freespace_cloud_enable) {
    if (!free_frustum_params_.loadParams(ns + "/FreeFrustumParams")) {
      ROS_WARN_COND(param_verbosity >= Verbosity::WARN,
                    "No setting for FreeFrustumParams.");
      planning_params_.freespace_cloud_enable = false;
      planning_params_.free_frustum_before_planning = false;
    }
  }

  if(!planning_params_.yaw_tangent_correction) {
    ROS_WARN_COND(param_verbosity >= Verbosity::ERROR, "[PlanningParams] yaw_tangent_correction is false, disabling path_safety_enhance_enable.");
    planning_params_.path_safety_enhance_enable = false;
  }

  voxel_map_->setRaycastingParams(
      planning_params_.nonuniform_ray_cast,
      planning_params_.ray_cast_step_size_multiplier);
  if (planning_params_.auto_landing_enable) {
    planning_params_.go_home_if_fully_explored = false;
    planning_params_.auto_homing_enable = false;
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "NoGainZones size: %zu", planning_params_.no_gain_zones_list.size());
  if(global_verbosity >= Verbosity::DEBUG) {
    for (int i=0; i<planning_params_.no_gain_zones_list.size();++i)  std::cout << planning_params_.no_gain_zones_list[i] << std::endl;
  }
  if (planning_params_.no_gain_zones_list.size() <= 0) {
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "No NoGainZones.");
    use_no_gain_space_ = false;
  } else {
    for (auto& zone : planning_params_.no_gain_zones_list) {
      BoundedSpaceParams ngz;
      if (!ngz.loadParams(ns + "/NoGainZones/" + zone)) {
        continue;
      }
      no_gain_zones_.push_back(ngz);
    }
    if (no_gain_zones_.size() <= 0) {
      use_no_gain_space_ = false;
    }
  }

  if (!shared_params) {
    if (!robot_params_.loadParams(ns + "/RobotParams")) return false;
    if (!global_space_params_.loadParams(ns + "/BoundedSpaceParams/Global"))
      return false;
  }

  if (!local_space_params_.loadParams(ns + "/BoundedSpaceParams/Local"))
    return false;

  if (!local_search_params_.loadParams(ns + "/BoundedSpaceParams/LocalSearch"))
    return false;
  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    if (!local_adaptive_params_.loadParams(
            ns + "/BoundedSpaceParams/LocalAdaptiveExp")) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                    "No setting for adaptive exploration mode.");
      local_adaptive_params_ = local_space_params_;
    }
  } else {
    local_adaptive_params_ = local_space_params_;
  }
  if (!repositioning_region_->loadParams(ns + "/RepositioningRegionParams")) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "No setting for global repositioning region model.");
  }
  adaptive_orig_min_val_ = local_adaptive_params_.min_val;
  adaptive_orig_max_val_ = local_adaptive_params_.max_val;
  if (!random_sampler_.loadParams(ns +
                                  "/RandomSamplerParams/SamplerForExploration"))
    return false;
  if (!random_sampler_to_search_.loadParams(
          ns + "/RandomSamplerParams/SamplerForSearching"))
    return false;
  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    if (!random_sampler_adaptive_.loadParams(
            ns + "/RandomSamplerParams/SamplerForAdaptiveExp")) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                    "No setting for adaptive exploration sampler.");
    }
  }

  if (ros::param::has(ns + "/RobotDynamics")) {
    robot_dynamics_params_.loadParams(ns + "/RobotDynamics");
  } else {
    robot_dynamics_params_.v_max = planning_params_.v_max;
    robot_dynamics_params_.v_homing_max = planning_params_.v_homing_max;
    robot_dynamics_params_.yaw_rate_max = planning_params_.yaw_rate_max;
  }

  if (planning_params_.geofence_checking_enable &&
      !geofence_manager_->loadParams(ns + "/GeofenceParams")) {
    ROS_WARN_COND(param_verbosity >= Verbosity::WARN,
                  "No GeofenceParams block; disabling geofence checking.");
    planning_params_.geofence_checking_enable = false;
  }
  if (ros::param::has(ns + "/DarpaGateParams")) {
    darpa_gate_params_.loadParams(ns + "/DarpaGateParams");
  }
  initializeParams();
  return true;
}

void LocalExplorationPlanner::setGeofenceManager(
    std::shared_ptr<GeofenceManager> geofence_manager) {
  geofence_manager_ = geofence_manager;
}

void LocalExplorationPlanner::setSharedParams(const RobotParams& robot_params,
                          const BoundedSpaceParams& global_space_params) {
  robot_params_ = robot_params;
  robot_params_.getPlanningSize(robot_box_size_);

  global_space_params_ = global_space_params;
}

void LocalExplorationPlanner::setSharedParams(const RobotParams& robot_params,
                          const BoundedSpaceParams& global_space_params,
                          const BoundedSpaceParams& local_space_params) {
  robot_params_ = robot_params;
  robot_params_.getPlanningSize(robot_box_size_);

  global_space_params_ = global_space_params;
  local_space_params_ = local_space_params;
}

void LocalExplorationPlanner::initializeParams() {
  if (planning_params_.type == PlanningModeType::kAdaptiveExploration){
    for(int i=0;i<4;i++){
      if (random_sampler_adaptive_.getInitPDF()[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
        random_sampler_adaptive_.setPDF(RandomSamplerBase::RandomDistributionType::kNormal,i); 
      }
    }
  }else{
    for(int i=0;i<4;i++){
      if (random_sampler_.getInitPDF()[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
        random_sampler_.setPDF(RandomSamplerBase::RandomDistributionType::kNormal,i); 
      }
    }
  }
  random_sampler_.setParams(global_space_params_, local_space_params_);
  random_sampler_to_search_.setParams(global_space_params_,
                                      local_search_params_);
  robot_params_.getPlanningSize(robot_box_size_);
  planning_num_vertices_max_ = planning_params_.num_vertices_max;
  planning_num_edges_max_ = planning_params_.num_edges_max;
  global_bound_.setDefault(global_space_params_.min_val,
                           global_space_params_.max_val);
}

bool LocalExplorationPlanner::setGlobalBound(planner_msgs::PlanningBound& bound,
                         bool reset_to_default) {
  if (!reset_to_default) {
    if ((current_state_.x() + robot_params_.center_offset.x() <
         bound.min_val.x + 0.5 * robot_box_size_.x()) ||
        (current_state_.y() + robot_params_.center_offset.y() <
         bound.min_val.y + 0.5 * robot_box_size_.y()) ||
        (current_state_.z() + robot_params_.center_offset.z() <
         bound.min_val.z + 0.5 * robot_box_size_.z()) ||
        (current_state_.x() + robot_params_.center_offset.x() >
         bound.max_val.x - 0.5 * robot_box_size_.x()) ||
        (current_state_.y() + robot_params_.center_offset.y() >
         bound.max_val.y - 0.5 * robot_box_size_.y()) ||
        (current_state_.z() + robot_params_.center_offset.z() >
         bound.max_val.z - 0.5 * robot_box_size_.z())) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
          "[GlobalBound] Failed to change since robot's position is outside "
          "the global bound.");
      return false;
    }

    Eigen::Vector3d v_min, v_max;
    global_bound_.get(v_min, v_max);
    v_min.x() = bound.min_val.x;
    v_min.y() = bound.min_val.y;
    if (bound.use_z_val) v_min.z() = bound.min_val.z;
    v_max.x() = bound.max_val.x;
    v_max.y() = bound.max_val.y;
    if (!bound.use_z_val) v_max.z() = bound.max_val.z;
    global_bound_.set(v_min, v_max);
    global_space_params_.min_val = v_min;
    global_space_params_.max_val = v_max;
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
        "[GlobalBound] Changed successfully: Min [%f, %f, %f], Max [%f, %f, "
        "%f]",
        v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
  } else {
    global_bound_.reset();
    Eigen::Vector3d v_min, v_max;
    global_bound_.get(v_min, v_max);
    global_space_params_.min_val = v_min;
    global_space_params_.max_val = v_max;
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
        "[GlobalBound] Reset to default: Min [%f, %f, %f], Max [%f, %f, %f]",
        v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
  }
  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_adaptive_params_);
  } else {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_space_params_);
  }
  return true;
}

bool LocalExplorationPlanner::setGlobalBound(
    planner_msgs::planner_dynamic_global_bound::Request bound) {
  if (bound.reset_to_default) {
    global_bound_.reset();
    Eigen::Vector3d v_min, v_max;
    global_bound_.get(v_min, v_max);
    global_space_params_.min_val = v_min;
    global_space_params_.max_val = v_max;
    Eigen::Vector3d zero_vector = Eigen::Vector3d::Zero();
    Eigen::Vector3d center = 0.5 * (v_max - v_min);
    global_space_params_.setRotation(zero_vector);
    global_space_params_.setCenter(center, false);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
        "[GlobalBound] Reset to default: Min [%f, %f, %f], Max [%f, %f, %f]",
        v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "world frame: %s, %s", world_frame_.c_str(), planning_params_.global_frame_id.c_str());
    tf::StampedTransform darpa_to_world_transform;
    try {
      listener_->lookupTransform(world_frame_, bound.header.frame_id,
                                 ros::Time(0), darpa_to_world_transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "%s", ex.what());
    }
    tf::Vector3 center_tf;
    tf::pointMsgToTF(bound.center, center_tf);
    tf::Vector3 center_trans_tf = darpa_to_world_transform * center_tf;
    geometry_msgs::Point center_trans_point;
    tf::pointTFToMsg(center_trans_tf, center_trans_point);
    Eigen::Vector3d center;
    convertPointToEigen(center_trans_point, center);
    tf::Vector3 left_tf;
    tf::pointMsgToTF(bound.left, left_tf);
    tf::Vector3 left_trans_tf = darpa_to_world_transform * left_tf;
    geometry_msgs::Point left_trans_point;
    tf::pointTFToMsg(left_trans_tf, left_trans_point);
    Eigen::Vector3d left;
    convertPointToEigen(left_trans_point, left);
    tf::Vector3 up_tf;
    tf::pointMsgToTF(bound.up, up_tf);
    tf::Vector3 up_trans_tf = darpa_to_world_transform * up_tf;
    geometry_msgs::Point up_trans_point;
    tf::pointTFToMsg(up_trans_tf, up_trans_point);
    Eigen::Vector3d up;
    convertPointToEigen(up_trans_point, up);
    tf::Vector3 front_tf;
    tf::pointMsgToTF(bound.front, front_tf);
    tf::Vector3 front_trans_tf = darpa_to_world_transform * front_tf;
    geometry_msgs::Point front_trans_point;
    tf::pointTFToMsg(front_trans_tf, front_trans_point);
    Eigen::Vector3d front;
    convertPointToEigen(front_trans_point, front);

    Eigen::Vector3d dir1 = front - center;
    Eigen::Vector3d dir2 = left - center;
    Eigen::Vector3d dir3 = up - center;

    Eigen::Vector3d rotations;
    rotations(0) = atan2(dir1(1), dir1(0));       // Yaw
    rotations(1) = asin(-dir1(2) / dir1.norm());  // Pitch
    rotations(2) = asin(dir2(2) / dir2.norm());   // Roll

    Eigen::Vector3d min_val, max_val;
    min_val = -0.5 * Eigen::Vector3d(dir1.norm(), dir2.norm(), dir3.norm());
    max_val = 0.5 * Eigen::Vector3d(dir1.norm(), dir2.norm(), dir3.norm());

    global_space_params_.setRotation(rotations);
    Eigen::Vector3d cuboid_center =
        center + global_space_params_.getRotationMatrix().inverse() *
                     (0.5 * (max_val - min_val));
    global_space_params_.setCenter(cuboid_center, false);
    global_space_params_.setBound(min_val, max_val);
  }

  StateVec local_bb_root;
  if (root_vertex_ != NULL) {
    local_bb_root = root_vertex_->state;
  } else {
    local_bb_root = current_state_;
  }
  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    visualization_->visualizeWorkspace(local_bb_root, global_space_params_,
                                       local_adaptive_params_);
  } else {
    visualization_->visualizeWorkspace(local_bb_root, global_space_params_,
                                       local_space_params_);
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "MissionVisuals done");
  return true;
}

void LocalExplorationPlanner::getGlobalBound(planner_msgs::PlanningBound& bound) {
  global_bound_.get(bound.min_val, bound.max_val);
}

void LocalExplorationPlanner::setState(StateVec& state) {
  if (!odometry_ready) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Received the first odometry, reset the map");
    voxel_map_->resetMap();
  }
  current_state_ = state;
  odometry_ready = true;
  if (planner_trigger_count_ < planning_params_.augment_free_voxels_time) {
    const double startup_reach = std::max(
        planning_params_.edge_length_min,
        std::min(planning_params_.edge_length_max,
                 std::max(planning_params_.nearest_range_min,
                          planning_params_.nearest_range)));
    augmentFreeStartupArea(current_state_, startup_reach);
  }
  if (robot_backtracking_queue_.size()) {
    if (robot_backtracking_queue_.size() >= backtracking_queue_max_size) {
      robot_backtracking_queue_.pop();
    }
    robot_backtracking_queue_.emplace(current_state_);
  } else {
    robot_backtracking_queue_.emplace(state);
  }
}

void LocalExplorationPlanner::freePointCloudtimerCallback(const ros::TimerEvent& event) {
  if (!planning_params_.freespace_cloud_enable) return;
  if(!odometry_ready) return;

  auto t1 = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud_body(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<Eigen::Vector3d> multiray_endpoints_body;
  for (auto sensor_name : free_frustum_params_.sensor_list) {
    StateVec state;
    state[0] = current_state_[0];
    state[1] = current_state_[1];
    state[2] = current_state_[2];
    state[3] = current_state_[3];
    free_frustum_params_.sensor[sensor_name].getFrustumEndpoints(
        state, multiray_endpoints_body);
    std::vector<Eigen::Vector3d> multiray_endpoints;
    voxel_map_->getFreeSpacePointCloud(multiray_endpoints_body, state,
                                         free_cloud_body);
    pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    free_frustum_params_.sensor[sensor_name].convertBodyToSensor(
        free_cloud_body, free_cloud);

    sensor_msgs::PointCloud2 out_cloud;
    pcl::toROSMsg(*free_cloud.get(), out_cloud);
    out_cloud.header.frame_id =
        free_frustum_params_.sensor[sensor_name].frame_id;
    out_cloud.header.stamp = ros::Time::now();
    free_cloud_pub_.publish(out_cloud);
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = t2 - t1;
}

void LocalExplorationPlanner::timerCallback(const ros::TimerEvent& event) {
  if (rostime_start_.toSec() == 0) rostime_start_ = ros::Time::now();
  ros::Time tcbtime;
  START_TIMER(tcbtime);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;

  
  if (!odometry_ready) {
    if (global_verbosity >= Verbosity::WARN) {
      ROS_WARN_THROTTLE(2.0, "Planner is waiting for odometry");
    }
    return;
  }

  if (landing_engaged_) {
    return;
  }

  if (planning_params_.auto_landing_enable && !landing_engaged_) {
    double time_elapsed = 0.0;
    if ((ros::Time::now()).toSec() != 0.0) {
      if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
      time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
    }
    double time_budget_remaining =
        planning_params_.time_budget_before_landing - time_elapsed;
    if (time_budget_remaining <= 0.0) {
      if (!local_exploration_ongoing_) {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "RAN OUT OF TIME BUDGET --> LANDING.");
        landing_engaged_ = true;
        std_msgs::Bool stop_msg;
        stop_msg.data = true;
        std_srvs::Empty empty_srv;
        landing_srv_client_.call(empty_srv);
        pci_reset_pub_.publish(stop_msg);
      }
      return;
    }
  }

  constexpr int kQueueMaxSize = 10;
  constexpr double kAlpha = 0.3;
  constexpr double kMinDist = 0.75;
  
  if (robot_state_queue_.size()) {
    StateVec& last_state = robot_state_queue_.back();
    Eigen::Vector3d cur_dir(current_state_[0] - last_state[0],
                            current_state_[1] - last_state[1],
                            0.0);  // ignore changes in z-axis
    if (cur_dir.norm() >= kMinDist) {
      double yaw = atan2(cur_dir[1], cur_dir[0]);
      double dyaw = yaw - exploring_direction_;
      truncateYaw(dyaw);
      exploring_direction_ = exploring_direction_ + (1 - kAlpha) * dyaw;
      truncateYaw(exploring_direction_);
      
      if (robot_state_queue_.size() >= kQueueMaxSize) {
        robot_state_queue_.pop();
      }
      robot_state_queue_.emplace(current_state_);
    }
  } else {
    robot_state_queue_.emplace(current_state_);
  }

  bool enforce_vertex_from_odometry = true;
  double kOdoEnforceLength = planning_params_.global_graph_odom_dist;
  auto t1_s = std::chrono::high_resolution_clock::now();
  auto t2_s = t1_s;
  double expand_graph_cumulative_time = 0;
  
  if (enforce_vertex_from_odometry) {
    while (robot_backtracking_queue_.size()) {
      StateVec bt_state = robot_backtracking_queue_.front();
      robot_backtracking_queue_.pop();
      
      if (robot_backtracking_prev_ == NULL)
        global_graph_->getNearestVertexInRange(&bt_state, kOdoEnforceLength, &robot_backtracking_prev_);
      
      if (robot_backtracking_prev_) {
        Eigen::Vector3d cur_dir(
            bt_state[0] - robot_backtracking_prev_->state[0],
            bt_state[1] - robot_backtracking_prev_->state[1],
            bt_state[2] - robot_backtracking_prev_->state[2]);
        double dir_norm = cur_dir.norm();
        
        if (dir_norm >= kOdoEnforceLength) {
          Vertex* new_vertex =
              new Vertex(global_graph_->generateVertexID(), bt_state);
          new_vertex->parent = robot_backtracking_prev_;
          new_vertex->distance = robot_backtracking_prev_->distance + dir_norm;

          
          global_graph_->addVertex(new_vertex);
          const double kEdgeWeightExtended = 1.0;
          global_graph_->addEdge(new_vertex, robot_backtracking_prev_,
                                 dir_norm * kEdgeWeightExtended);
          robot_backtracking_prev_ = new_vertex;
          
          ExpandGraphReport rep;
          auto t1_s_2 = std::chrono::high_resolution_clock::now();
          auto t2_s_2 = t1_s_2;
          
          double og_edge_length_max = planning_params_.edge_length_max;
          planning_params_.edge_length_max = planning_params_.global_graph_odom_connect_radius;
          planning_params_.nearest_range_max = planning_params_.global_graph_odom_connect_radius;
          planning_params_.nearest_range = planning_params_.global_graph_odom_connect_radius;
          
          expandGraphEdges(global_graph_, new_vertex, rep);
          maybeAddShortcutEdges(global_graph_, new_vertex);
          
          planning_params_.edge_length_max = og_edge_length_max;
          planning_params_.nearest_range_max = og_edge_length_max;
          planning_params_.nearest_range = og_edge_length_max;
          
          t2_s_2 = std::chrono::high_resolution_clock::now();
          expand_graph_cumulative_time += std::chrono::duration<double, std::milli>(t2_s_2 - t1_s_2).count();
        }
      }
    }
  }
  t2_s = std::chrono::high_resolution_clock::now();
  
  Eigen::Vector3d cur_dir(current_state_[0] - last_state_marker_[0],
                          current_state_[1] - last_state_marker_[1],
                          current_state_[2] - last_state_marker_[2]);

  t1_s = std::chrono::high_resolution_clock::now();
  constexpr double kMinLength = 1.0;
  Eigen::Vector3d cur_dir1(current_state_[0] - last_state_marker_global_[0],
                           current_state_[1] - last_state_marker_global_[1],
                           current_state_[2] - last_state_marker_global_[2]);
  
  if (cur_dir1.norm() >= kMinLength) {
    constexpr double kUpdateRadius = 3.0;
    StateVec state_add;
    state_add << current_state_[0], current_state_[1], current_state_[2],
        current_state_[3], 0.0;
    robot_state_hist_->addState(&state_add);
    
    const bool apply_eventE1 = true;
    if (apply_eventE1) {
      global_graph_->updateVertexTypeInRange(current_state_,
                                             kUpdateRadius);  // E1
    }

    last_state_marker_global_ = current_state_;
  }
  t2_s = std::chrono::high_resolution_clock::now();

  t2 = std::chrono::high_resolution_clock::now();
}

void LocalExplorationPlanner::setBoundMode(BoundModeType bmode) {
  constexpr double kNumVerticesRatio = 1.3;
  constexpr double kNumEdgesRatio = 1.3;
  robot_params_.setBoundMode(bmode);
  robot_params_.getPlanningSize(robot_box_size_);

  switch (bmode) {
    case BoundModeType::kExtendedBound:
      planning_num_vertices_max_ = planning_params_.num_vertices_max;
      planning_num_edges_max_ = planning_params_.num_edges_max;
      break;
    case BoundModeType::kRelaxedBound:
      planning_num_vertices_max_ =
          (int)((double)planning_params_.num_vertices_max * kNumVerticesRatio);
      planning_num_edges_max_ =
          (int)((double)planning_params_.num_edges_max * kNumEdgesRatio);
      break;
    case BoundModeType::kMinBound:
      planning_num_vertices_max_ =
          (int)((double)planning_params_.num_vertices_max * kNumVerticesRatio *
                kNumVerticesRatio);
      planning_num_edges_max_ = (int)((double)planning_params_.num_edges_max *
                                      kNumEdgesRatio * kNumEdgesRatio);
      break;
  }
}

bool LocalExplorationPlanner::compareAngles(double dir_angle_a, double dir_angle_b, double thres) {
  double dyaw = dir_angle_a - dir_angle_b;
  if (dyaw > M_PI)
    dyaw -= 2 * M_PI;
  else if (dyaw < -M_PI)
    dyaw += 2 * M_PI;

  if (std::abs(dyaw) <= thres) {
    return true;
  } else {
    return false;
  }
}

bool LocalExplorationPlanner::comparePathWithDirectionApprioximately(
    const std::vector<geometry_msgs::Pose>& path, double yaw) {
  const double kMinSegmentLen = 2.0;
  const double kYawThres = 0.5 * M_PI;

  if (path.size() <= 1) return true;
  Eigen::Vector3d root_pos(path[0].position.x, path[0].position.y,
                           path[0].position.z);
  double path_yaw = 0;
  for (int i = 1; i < path.size(); ++i) {
    Eigen::Vector3d dir_vec;
    dir_vec << path[i].position.x - root_pos.x(),
        path[i].position.y - root_pos.y(),
        0.0;  // ignore z
    if (dir_vec.norm() > kMinSegmentLen) {
      path_yaw = std::atan2(dir_vec.y(), dir_vec.x());
      break;
    }
  }

  double dyaw = path_yaw - yaw;
  if (dyaw > M_PI)
    dyaw -= 2 * M_PI;
  else if (dyaw < -M_PI)
    dyaw += 2 * M_PI;

  if (std::abs(dyaw) <= kYawThres) {
    return true;
  } else {
    return false;
  }
}

std::vector<int> LocalExplorationPlanner::performShortestPathsClustering(
    const std::shared_ptr<CognitiveGraph> graph_manager,
    const ShortestPathsReport& graph_rep, std::vector<Vertex*>& vertices,
    double dist_threshold, double principle_path_min_length,
    bool refinement_enable) {
  std::sort(vertices.begin(), vertices.end(),
            [&graph_manager, &graph_rep](const Vertex* a, const Vertex* b) {
              return graph_manager->getShortestDistance(a->id, graph_rep) >
                     graph_manager->getShortestDistance(b->id, graph_rep);
            });

  std::vector<std::vector<Eigen::Vector3d>> cluster_paths;
  std::vector<int> cluster_ids;
  for (int i = 0; i < vertices.size(); ++i) {
    std::vector<Eigen::Vector3d> path_cur;
    graph_manager->getShortestPath(vertices[i]->id, graph_rep, true, path_cur);
    bool found_a_neigbor = false;
    for (int j = 0; j < cluster_paths.size(); ++j) {
      if (Trajectory::compareTwoTrajectories(path_cur, cluster_paths[j],
                                             dist_threshold)) {
        vertices[i]->cluster_id = cluster_ids[j];
        found_a_neigbor = true;
        break;
      }
    }
    if (!found_a_neigbor) {
      cluster_paths.emplace_back(path_cur);
      cluster_ids.push_back(vertices[i]->id);
      vertices[i]->cluster_id = vertices[i]->id;
    }
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Cluster %d paths into %d clusters.", (int)vertices.size(),
           (int)cluster_paths.size());
  if (refinement_enable) {
    std::vector<std::vector<Eigen::Vector3d>> cluster_paths_refine;
    std::vector<int> cluster_ids_refine;
    for (int j = 0; j < cluster_paths.size(); ++j) {
      double path_len = Trajectory::getPathLength(cluster_paths[j]);
      if (path_len >= principle_path_min_length) {
        cluster_paths_refine.push_back(cluster_paths[j]);
        cluster_ids_refine.push_back(cluster_ids[j]);
      }
    }
    for (int i = 0; i < vertices.size(); ++i) {
      std::vector<Eigen::Vector3d> path_cur;
      graph_manager->getShortestPath(vertices[i]->id, graph_rep, true,
                                     path_cur);
      double dist_min = std::numeric_limits<double>::infinity();
      for (int j = 0; j < cluster_paths_refine.size(); ++j) {
        double dist_score = Trajectory::computeDistanceBetweenTwoTrajectories(
            path_cur, cluster_paths_refine[j]);
        if (dist_min > dist_score) {
          dist_min = dist_score;
          vertices[i]->cluster_id = cluster_ids_refine[j];
        }
      }
    }
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Clustering with refinement %d paths into %d clusters.",
             (int)vertices.size(), (int)cluster_paths_refine.size());
    return cluster_ids_refine;
  } else {
    return cluster_ids;
  }
}

double LocalExplorationPlanner::estimatePathClearance(const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& end) const {
  const Eigen::Vector3d edge = end - start;
  const double edge_length = edge.norm();
  if (edge_length <= kSmallNorm) {
    return getClearanceScore(start);
  }

  const double step =
      std::max(voxel_map_->getResolution(), 0.25 * robot_box_size_.norm());
  double min_clearance = std::numeric_limits<double>::infinity();
  for (double dist = 0.0; dist <= edge_length; dist += step) {
    Eigen::Vector3d point = start + (dist / edge_length) * edge;
    double clearance =
        voxel_map_->getPointDistance(point + robot_params_.center_offset);
    if (clearance < 0.0) {
      return clearance;
    }
    min_clearance = std::min(min_clearance, clearance);
  }
  return min_clearance;
}

void LocalExplorationPlanner::maybeAddShortcutEdges(
    const std::shared_ptr<CognitiveGraph> graph_manager, Vertex* new_vertex) {
  if (!planning_params_.shortcut_edges_enable || graph_manager == NULL ||
      new_vertex == NULL) {
    return;
  }
  if (graph_manager != global_graph_) {
    return;
  }

  std::vector<Vertex*> nearby_vertices;
  if (!graph_manager->getNearestVertices(&new_vertex->state,
                                         planning_params_.shortcut_edge_radius,
                                         &nearby_vertices)) {
    return;
  }

  ShortestPathsReport shortest_paths;
  if (!graph_manager->findShortestPaths(new_vertex->id, shortest_paths)) {
    return;
  }

  for (Vertex* candidate : nearby_vertices) {
    if (candidate == NULL || candidate->id == new_vertex->id) {
      continue;
    }
    if (std::find(new_vertex->neighbors.begin(), new_vertex->neighbors.end(),
                  candidate->id) != new_vertex->neighbors.end()) {
      continue;
    }

    const Eigen::Vector3d start = new_vertex->state.head(3);
    const Eigen::Vector3d end = candidate->state.head(3);
    const double direct_distance = (end - start).norm();
    if (direct_distance <=
        std::max(planning_params_.global_graph_odom_connect_radius,
                 planning_params_.edge_length_max)) {
      continue;
    }

    const double graph_distance =
        graph_manager->getShortestDistance(candidate->id, shortest_paths);
    if (graph_distance <= 0.0 ||
        graph_distance <
            direct_distance * planning_params_.shortcut_detour_ratio) {
      continue;
    }

    if (VoxelStatus::kFree !=
        voxel_map_->getPathStatus(start, end, robot_box_size_, true)) {
      continue;
    }

    const double min_clearance = estimatePathClearance(start, end);
    if (min_clearance >= 0.0 &&
        min_clearance < planning_params_.shortcut_min_clearance) {
      continue;
    }

    graph_manager->addEdge(new_vertex, candidate, direct_distance);
  }
}

bool LocalExplorationPlanner::anchorStateInCognitiveMap(std::shared_ptr<CognitiveGraph> graph,
                              StateVec& cur_state, Vertex*& v_added,
                              double dist_ignore_collision_check) {
  Vertex* nearest_vertex = NULL;
  if (!graph->getNearestVertex(&cur_state, &nearest_vertex)) return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();
  bool connect_state_to_graph = true;
  const double kDelta = 0.05;
  if (direction_norm <= kDelta) {
    ExpandGraphReport rep;
    expandGraphEdges(graph, nearest_vertex, rep);
    maybeAddShortcutEdges(graph, nearest_vertex);
    v_added = nearest_vertex;
  } else if (direction_norm <= std::max(dist_ignore_collision_check,
                                        planning_params_.edge_length_min)) {
    Vertex* new_vertex = new Vertex(graph->generateVertexID(), cur_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    graph->addVertex(new_vertex);
    graph->addEdge(new_vertex, nearest_vertex, direction_norm);
    ExpandGraphReport rep;
    expandGraphEdges(graph, new_vertex, rep);
    maybeAddShortcutEdges(graph, new_vertex);
    v_added = new_vertex;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Connecting current state to the topological graph.");
    ExpandGraphReport rep;
    Vertex new_vertex(-1, cur_state);
    expandGraph(graph, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Current state connected.");
      maybeAddShortcutEdges(graph, rep.vertex_added);
      v_added = rep.vertex_added;
    } else {
      connect_state_to_graph = false;
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Failed to connect current state:");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "kErrorShortEdge.");
          break;
      }
    }
  }
  return connect_state_to_graph;
}

double LocalExplorationPlanner::getTimeElapsed() {
  double time_elapsed = 0.0;
  if ((ros::Time::now()).toSec() != 0.0) {
    if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
    time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
  }
  return time_elapsed;
}

double LocalExplorationPlanner::getTimeRemained() {
  double time_budget_remaining =
      planning_params_.time_budget_limit - getTimeElapsed();
  return std::min(time_budget_remaining, current_battery_time_remaining_);
}

bool LocalExplorationPlanner::isRemainingTimeSufficient(const double& time_cost,
                                    double& time_spare) {
  const double kTimeDelta = 20;  // magic number, extra safety
  time_spare = getTimeRemained() - time_cost;
  if (time_spare < kTimeDelta) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "REACHED TIME LIMIT: BE CAREFUL.");
    return false;
  }
  return true;
}

bool LocalExplorationPlanner::getOpeningPathCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	std::vector<geometry_msgs::Pose> path = getOpeningTraversalPath();
	
	if(!path.empty()) {
		res.success = true;
	}
	return true;
}

bool LocalExplorationPlanner::approvePassingCallback(planner_msgs::planner_opening_approval::Request &req, planner_msgs::planner_opening_approval::Response &res) {
  if(req.approval == planner_msgs::planner_opening_approval::Request::kApproved) {
    opening_passing_approved_ = OpeningApproval::kApproved;
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Opening Passing Approved");
  }
  else if(req.approval == planner_msgs::planner_opening_approval::Request::kRejected) {
    opening_passing_approved_ = OpeningApproval::kRejected;
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Opening Passing Rejected");
  }
  else if(req.approval == planner_msgs::planner_opening_approval::Request::kReEvaluate) {
    opening_passing_approved_ = OpeningApproval::kReEvaluate;
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Opening Passing to be Re-evaluated");
  }
  
  return true;
}

void LocalExplorationPlanner::setNextCompartmentCenter(Eigen::Vector3d &center)
{
  next_compartment_ = center;
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::getOpeningTraversalPath(OpeningTraversalMode mode, OpeningTraversalStatus &status) {
	std::vector<geometry_msgs::Pose> through_path;

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "OPENING Mode: %d", mode);

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Opening under execution: %d", opening_under_execution_);

  bool opening_still_exists = true;
  if(mode == OpeningTraversalMode::kPathCheck)
  {
    auto itr = detected_openings_.find(opening_under_execution_);
    if(itr == detected_openings_.end())
    {
      opening_still_exists = false;
    }
  }

  if(mode == OpeningTraversalMode::kPathCheck)
  {
    if(opening_still_exists) 
    {
      status = OpeningTraversalStatus::OK;
      return through_path;
    }
    else // If after second detection, the opening is found to be a false detection, reject it and go to the next closest
    {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "This OPENING [%d] does not exist", opening_under_execution_);
      status = OpeningTraversalStatus::OPENING_DOUBLE_CHECK_FAILED;
      return through_path;
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Detections updated");


	if(detected_openings_.empty()) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Openings at all");
    status = OpeningTraversalStatus::NO_OPENINGS;
		return through_path;
	}

	std::shared_ptr<Opening> best_opening;
	double closest_distance = std::numeric_limits<double>::max();
  bool found = false;
  if(mode == OpeningTraversalMode::kGoingTo) {
    for(auto it : detected_openings_) {
      std::shared_ptr<Opening> current_opening = it.second;
      if(!current_opening->active || current_opening->num_tries >= planning_params_.max_opening_attempts) {
        continue;
      }
      
      if(planning_params_.exploration_only)  // If exploration only, go to the closest opening
      {
        double opening_robot_dist = (current_state_.head(3) 
              - Eigen::Vector3d(current_opening->pose.position.x, 
                                current_opening->pose.position.y, 
                                current_opening->pose.position.z)).norm();
        if(opening_robot_dist < closest_distance) {
          closest_distance = opening_robot_dist;
          best_opening = current_opening;
          found = true;
        }
      }
      else // If exploration + inspection, find the opening that leads to the next compartment
      {
        double opening_robot_dist = (planning_params_.compartment_centers[next_compartment_index_-1] - Eigen::Vector3d(current_opening->pose.position.x, current_opening->pose.position.y, current_opening->pose.position.z)).norm();
        Eigen::Vector3d compartment_dim = planning_params_.compartment_dimensions.max_val - planning_params_.compartment_dimensions.min_val;
        if(opening_robot_dist > compartment_dim.norm()/2.0)
        {
          continue;
        }
        double dist;
        if(next_compartment_.x() < std::numeric_limits<double>::max())
          dist = (next_compartment_ - Eigen::Vector3d(current_opening->pose.position.x, current_opening->pose.position.y, current_opening->pose.position.z)).norm();
        else
          dist = (current_state_.head(3) - Eigen::Vector3d(current_opening->pose.position.x, current_opening->pose.position.y, current_opening->pose.position.z)).norm();
        if(dist < closest_distance) {
          closest_distance = dist;
          best_opening = current_opening;
          found = true;
        }
      }
    }
    if(found)
      opening_under_execution_ = best_opening->id;
  }
  else {
    best_opening = detected_openings_[opening_under_execution_];
    found = true;
  }

  if(!found) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Appropriate Opening Found");
    status = OpeningTraversalStatus::NO_OPENINGS;
    return through_path;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Best opening found: %d", opening_under_execution_);
  std::vector<geometry_msgs::Pose> empty_path;
  

  if(global_verbosity >= Verbosity::DEBUG) {
    std::cout << best_opening->pose.position.x << " " << best_opening->pose.position.y << " " << best_opening->pose.position.z << " | "
              << best_opening->pose.orientation.x << " " << best_opening->pose.orientation.y << " " << best_opening->pose.orientation.z << " " << best_opening->pose.orientation.w << std::endl;
  }
  
  double direction = tf::getYaw(best_opening->pose.orientation);
  geometry_msgs::Pose p0;
  p0.position.x = best_opening->pose.position.x - planning_params_.opening_traversal_path_edge_length * std::cos(direction);
  p0.position.y = best_opening->pose.position.y - planning_params_.opening_traversal_path_edge_length * std::sin(direction);
  p0.position.z = best_opening->pose.position.z;
  p0.orientation = best_opening->pose.orientation;
  
  geometry_msgs::Pose p1;
  p1.position.x = best_opening->pose.position.x + planning_params_.opening_traversal_path_edge_length * std::cos(direction);
  p1.position.y = best_opening->pose.position.y + planning_params_.opening_traversal_path_edge_length * std::sin(direction);
  p1.position.z = best_opening->pose.position.z;
  p1.orientation = best_opening->pose.orientation;

  geometry_msgs::Pose first_pose;
  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
    first_pose = p0;
  }
  else {
    first_pose = p1;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path direction set");

  geometry_msgs::Quaternion corrected_quat = best_opening->pose.orientation;

  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
      tf::Quaternion quat;
      double corrected_yaw = std::atan2(p1.position.y - p0.position.y, p1.position.x - p0.position.x);
      quat.setEuler(0.0, 0.0, corrected_yaw);
      tf::quaternionTFToMsg(quat, corrected_quat);
      tf::quaternionTFToMsg(quat, p0.orientation);
      tf::quaternionTFToMsg(quat, best_opening->pose.orientation);
      tf::quaternionTFToMsg(quat, p1.orientation);
  }
  else {
    tf::Quaternion quat;
    double corrected_yaw = std::atan2(p0.position.y - p1.position.y, p0.position.x - p1.position.x);
    quat.setEuler(0.0, 0.0, corrected_yaw);
    tf::quaternionTFToMsg(quat, corrected_quat);
    tf::quaternionTFToMsg(quat, p0.orientation);
    tf::quaternionTFToMsg(quat, best_opening->pose.orientation);
    tf::quaternionTFToMsg(quat, p1.orientation);
  }

  tf::Quaternion corrected_quat_tf;
  tf::quaternionMsgToTF(corrected_quat, corrected_quat_tf);
  double corrected_yaw = tf::getYaw(corrected_quat_tf);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Corrected quat: %f", corrected_yaw);

  if(mode == OpeningTraversalMode::kPassingThrough) {
    if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
      p0.orientation = corrected_quat;
      best_opening->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p0);
      through_path.push_back(best_opening->pose);
      through_path.push_back(p1);

    }
    else {
      p0.orientation = corrected_quat;
      best_opening->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p1);
      through_path.push_back(best_opening->pose);
      through_path.push_back(p0);
    }

  }
  else {
    first_pose.orientation = corrected_quat;
    first_pose.position.z += planning_params_.opening_alignment_z_offset;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 1");

  geometry_msgs::Pose current_pose;
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, current_state_[3]);
  tf::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
  tf::Pose poseTF(quat, origin);
  tf::poseTFToMsg(poseTF, current_pose);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 2");

  std::vector<geometry_msgs::Pose> connecting_path;
  if(mode != OpeningTraversalMode::kPassingThrough)
  {
    bool success = search(current_pose, first_pose, false, connecting_path);
    if(success) {
      connecting_path.push_back(first_pose);
      connecting_path.back().orientation = corrected_quat;
      through_path.insert(through_path.begin(), connecting_path.begin(), connecting_path.end());
    }
    else {
      ROS_ERROR_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Connecting path not found");
      ++best_opening->num_tries;
      status = OpeningTraversalStatus::CANT_CONNECT;
      return empty_path;
    }
  }

  linearlyInterpolateYaw(through_path);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 3");

  visualization_->visualizeOpeningTraversalPath(through_path);

  if(mode == OpeningTraversalMode::kPathCheck) {
    status = OpeningTraversalStatus::OK;
    return empty_path;
  } 
  else if(mode == OpeningTraversalMode::kPassingThrough) {
    for(int i=0; i<through_path.size(); ++i) {
      through_path[i].orientation = corrected_quat;
    }
    through_path.insert(through_path.begin(), current_pose);
    best_opening->active = false;
    status = OpeningTraversalStatus::OK;
    return through_path;
  }
  else {
    status = OpeningTraversalStatus::OK;
    return through_path;
  }
}


std::vector<geometry_msgs::Pose> LocalExplorationPlanner::getOpeningTraversalPath() {
	std::vector<geometry_msgs::Pose> through_path;


  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "OPENING Mode: %d", opening_traversal_mode_);
  
  if(opening_traversal_mode_ == OpeningTraversalMode::kNone) {
    opening_traversal_mode_ = OpeningTraversalMode::kGoingTo;
  }
  else if(opening_traversal_mode_ == OpeningTraversalMode::kGoingTo) {

    if(planning_params_.auto_opening_path_approval) {
    	opening_traversal_mode_ = OpeningTraversalMode::kPassingThrough;
    }
    else{
    	opening_traversal_mode_ = OpeningTraversalMode::kPathCheck;
    }
  }
  else if(opening_traversal_mode_ == OpeningTraversalMode::kPathCheck) {
    if(planning_params_.auto_opening_path_approval) {
      opening_passing_approved_ = OpeningApproval::kApproved;
    }
    if(opening_passing_approved_ == OpeningApproval::kApproved) {
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Opening Traversal Approved");
      opening_passing_approved_ = OpeningApproval::kWaiting;
      opening_traversal_mode_ = OpeningTraversalMode::kPassingThrough;
    }
    else if(opening_passing_approved_ == OpeningApproval::kRejected) {
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Opening Traversal Rejected");
      opening_traversal_mode_ = OpeningTraversalMode::kNone;
      detected_openings_[opening_under_execution_]->active = false;
      return through_path;
    }
    else if(opening_passing_approved_ == OpeningApproval::kReEvaluate) {
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Opening Traversal ReEvaluate");
      opening_traversal_mode_ = OpeningTraversalMode::kPathCheck;
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "OPENING Mode: %d", opening_traversal_mode_);
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Opening under execution: %d", opening_under_execution_);

  if(opening_traversal_mode_ == OpeningTraversalMode::kPathCheck)
  {
    auto itr = detected_openings_.find(opening_under_execution_);
    if(itr == detected_openings_.end())
    {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "This OPENING [%d] does not exist", opening_under_execution_);
      opening_traversal_mode_ = OpeningTraversalMode::kGoingTo;
    }
  }


	if(detected_openings_.empty()) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Openings at all");
		return through_path;
	}

	std::shared_ptr<Opening> best_opening;
	double closest_distance = std::numeric_limits<double>::max();
  bool found = false;
  if(opening_traversal_mode_ != OpeningTraversalMode::kPassingThrough && opening_traversal_mode_ != OpeningTraversalMode::kPathCheck) {
    for(auto it : detected_openings_) {
      std::shared_ptr<Opening> current_opening = it.second;
      if(!current_opening->active) {
        continue;
      }
      
      double opening_robot_dist = (planning_params_.compartment_centers[next_compartment_index_-1] - Eigen::Vector3d(current_opening->pose.position.x, current_opening->pose.position.y, current_opening->pose.position.z)).norm();
      Eigen::Vector3d compartment_dim = planning_params_.compartment_dimensions.max_val - planning_params_.compartment_dimensions.min_val;
      if(opening_robot_dist > compartment_dim.norm()/2.0)
      {
        continue;
      }
      double dist;
      if(next_compartment_.x() < std::numeric_limits<double>::max())
        dist = (next_compartment_ - Eigen::Vector3d(current_opening->pose.position.x, current_opening->pose.position.y, current_opening->pose.position.z)).norm();
      else
        dist = (current_state_.head(3) - Eigen::Vector3d(current_opening->pose.position.x, current_opening->pose.position.y, current_opening->pose.position.z)).norm();
      if(dist < closest_distance) {
        closest_distance = dist;
        best_opening = current_opening;
        found = true;
      }
    }
    if(found)
      opening_under_execution_ = best_opening->id;
  }
  else {
    best_opening = detected_openings_[opening_under_execution_];
    found = true;
  }

  if(!found) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "No Opening Found");
    opening_traversal_mode_ = OpeningTraversalMode::kNone;
    return through_path;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Best opening found: %d", opening_under_execution_);
  std::vector<geometry_msgs::Pose> empty_path;

  if(global_verbosity >= Verbosity::DEBUG) {
    std::cout << best_opening->pose.position.x << " " << best_opening->pose.position.y << " " << best_opening->pose.position.z << " | "
              << best_opening->pose.orientation.x << " " << best_opening->pose.orientation.y << " " << best_opening->pose.orientation.z << " " << best_opening->pose.orientation.w << std::endl;
  }
  
  double direction = tf::getYaw(best_opening->pose.orientation);
  geometry_msgs::Pose p0;
  p0.position.x = best_opening->pose.position.x - planning_params_.opening_traversal_path_edge_length * std::cos(direction);
  p0.position.y = best_opening->pose.position.y - planning_params_.opening_traversal_path_edge_length * std::sin(direction);
  p0.position.z = best_opening->pose.position.z;
  p0.orientation = best_opening->pose.orientation;
  
  geometry_msgs::Pose p1;
  p1.position.x = best_opening->pose.position.x + planning_params_.opening_traversal_path_edge_length * std::cos(direction);
  p1.position.y = best_opening->pose.position.y + planning_params_.opening_traversal_path_edge_length * std::sin(direction);
  p1.position.z = best_opening->pose.position.z;
  p1.orientation = best_opening->pose.orientation;
  

  geometry_msgs::Pose first_pose;
  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
    first_pose = p0;
  }
  else {
    first_pose = p1;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path direction set");
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "OPENING Mode (later): %d", opening_traversal_mode_);

  geometry_msgs::Quaternion corrected_quat = best_opening->pose.orientation;

  if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {
      tf::Quaternion quat;
      double corrected_yaw = std::atan2(p1.position.y - p0.position.y, p1.position.x - p0.position.x);
      quat.setEuler(0.0, 0.0, corrected_yaw);
      tf::quaternionTFToMsg(quat, corrected_quat);
      tf::quaternionTFToMsg(quat, p0.orientation);
      tf::quaternionTFToMsg(quat, best_opening->pose.orientation);
      tf::quaternionTFToMsg(quat, p1.orientation);
  }
  else {
    tf::Quaternion quat;
    double corrected_yaw = std::atan2(p0.position.y - p1.position.y, p0.position.x - p1.position.x);
    quat.setEuler(0.0, 0.0, corrected_yaw);
    tf::quaternionTFToMsg(quat, corrected_quat);
    tf::quaternionTFToMsg(quat, p0.orientation);
    tf::quaternionTFToMsg(quat, best_opening->pose.orientation);
    tf::quaternionTFToMsg(quat, p1.orientation);
  }

  tf::Quaternion corrected_quat_tf;
  tf::quaternionMsgToTF(corrected_quat, corrected_quat_tf);
  double corrected_yaw = tf::getYaw(corrected_quat_tf);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Corrected quat: %f", corrected_yaw);

  if(opening_traversal_mode_ != OpeningTraversalMode::kGoingTo) {
    if(getDistance(current_state_, p0) < getDistance(current_state_, p1)) {

      p0.orientation = corrected_quat;
      best_opening->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p0);
      through_path.push_back(best_opening->pose);
      through_path.push_back(p1);

    }
    else {
      p0.orientation = corrected_quat;
      best_opening->pose.orientation = corrected_quat;
      p1.orientation = corrected_quat;
      through_path.push_back(p1);
      through_path.push_back(best_opening->pose);
      through_path.push_back(p0);
    }


  }
  else {
    first_pose.orientation = corrected_quat;
    first_pose.position.z += planning_params_.opening_alignment_z_offset;
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 1");

  geometry_msgs::Pose current_pose;
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, current_state_[3]);
  tf::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
  tf::Pose poseTF(quat, origin);
  tf::poseTFToMsg(poseTF, current_pose);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 2");

  std::vector<geometry_msgs::Pose> connecting_path;
  bool success = search(current_pose, first_pose, false, connecting_path);
  if(success) {
    connecting_path.push_back(first_pose);
    connecting_path.back().orientation = corrected_quat;
    through_path.insert(through_path.begin(), connecting_path.begin(), connecting_path.end());
  }
  else {
    ROS_ERROR_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Connecting path not found");
    return empty_path;
  }

  linearlyInterpolateYaw(through_path);

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Path set 3");

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "OPENING Mode (even later): %d", opening_traversal_mode_);

  visualization_->visualizeOpeningTraversalPath(through_path);

  if(opening_traversal_mode_ == OpeningTraversalMode::kPathCheck) {
    return empty_path;
  } 
  else if(opening_traversal_mode_ == OpeningTraversalMode::kPassingThrough) {
    for(int i=0; i<through_path.size(); ++i) {
      through_path[i].orientation = corrected_quat;
    }
    best_opening->active = false;

    opening_traversal_mode_ = OpeningTraversalMode::kNone;
    return through_path;
  }
  else {
  return through_path;
  }

}

bool LocalExplorationPlanner::removeGeofenceCallback(planner_msgs::planner_set_planning_mode::Request &req, planner_msgs::planner_set_planning_mode::Response &res)
{
  geofence_manager_->removeGeofenceAreaWithID(req.planning_mode);

  visualization_->visualizeGeofence(geofence_manager_);
  return true;
}

void LocalExplorationPlanner::addGeofenceAreas(const geometry_msgs::PolygonStamped& polygon_msgs) {
  if ((planning_params_.geofence_checking_enable)) {
    std::cout << "Receieved geofence: " << std::endl;
    for(auto pt : polygon_msgs.polygon.points)
    {
      std::cout << "  " << pt.x << " " << pt.y << " " << pt.z << std::endl;
    }

    if (!polygon_msgs.header.frame_id.compare(
            planning_params_.global_frame_id)) {
      geofence_manager_->addGeofenceArea(polygon_msgs.polygon);
    } else {
      geometry_msgs::Polygon polygon;
      tf::StampedTransform tf_to_global;
      tf::TransformListener listener;
      try {
        listener.waitForTransform(planning_params_.global_frame_id,
                                  polygon_msgs.header.frame_id, ros::Time(0),
                                  ros::Duration(0.1));  // this should be fast.
        listener.lookupTransform(planning_params_.global_frame_id,
                                 polygon_msgs.header.frame_id, ros::Time(0),
                                 tf_to_global);
        for (int i = 0; i < polygon_msgs.polygon.points.size(); ++i) {
          tf::Vector3 poly_in_global;
          poly_in_global.setValue(polygon_msgs.polygon.points[i].x,
                                  polygon_msgs.polygon.points[i].y,
                                  polygon_msgs.polygon.points[i].z);
          poly_in_global = tf_to_global * poly_in_global;
          geometry_msgs::Point32 p32;
          p32.x = poly_in_global.x();
          p32.y = poly_in_global.y();
          p32.z = poly_in_global.z();
          polygon.points.push_back(p32);
        }
        geofence_manager_->addGeofenceArea(polygon);
      } catch (tf::TransformException ex) {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
            "Could not look up TF from polygon frame [%s] to the global frame "
            "[%s].",
            polygon_msgs.header.frame_id.c_str(),
            planning_params_.global_frame_id.c_str());
      }
    }
    visualization_->visualizeGeofence(geofence_manager_);

    Eigen::Vector3d centroid;
    double geofence_rad = 0.0;
    for(auto pt : polygon_msgs.polygon.points)
    {
      Eigen::Vector3d p_vec(pt.x, pt.y, pt.z);
      centroid += p_vec;
    }
    centroid /= polygon_msgs.polygon.points.size();
    for(auto pt : polygon_msgs.polygon.points)
    {
      Eigen::Vector3d p_vec(pt.x, pt.y, pt.z);
      double dist = (p_vec - centroid).norm();
      if(dist > geofence_rad)
      {
        geofence_rad = dist;
      }
    }

    std::vector<Vertex*> nbs;
    StateVec geofence_state;
    geofence_state << centroid, 0.0;
    global_graph_->getNearestVertices(&geofence_state, geofence_rad, &nbs);
    for(int j=0; j<nbs.size(); ++j)
    {
      Vertex *v = nbs[j];
      if(GeofenceManager::CoordinateStatus::kViolated ==
        geofence_manager_->getBoxStatus(
            Eigen::Vector2d(v->state[0] + robot_params_.center_offset[0],
                            v->state[1] + robot_params_.center_offset[1]),
            Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))
      {
        std::vector<int> neighbor_ids;
        for(int i=0; i<v->neighbors.size(); ++i)
        {
          neighbor_ids.push_back(v->neighbors[i]);
        }
        for(int i=0; i<neighbor_ids.size(); ++i)
        {
          int u = neighbor_ids[i];
          global_graph_->removeEdge(v, global_graph_->getVertex(u));
        }
      }
    }
  }
}

void LocalExplorationPlanner::clearExclusionZones() {
  geofence_manager_.reset(new GeofenceManager());
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cleared all exclusion zones.");
}

void LocalExplorationPlanner::setGlobalFrame(std::string frame_id) {
  if (!frame_id.empty()) visualization_->setGlobalFrame(frame_id);
}

bool LocalExplorationPlanner::isPathCollisionFree(const std::vector<geometry_msgs::Pose>& path,
                              const Eigen::Vector3d& robot_size) {
  if (path.empty()) return true;  // nothing to check

  Eigen::Vector3d voxel(path[0].position.x, path[0].position.y,
                        path[0].position.z);
  if (path.size() == 1) {
    if (VoxelStatus::kFree ==
        voxel_map_->getBoxStatus(voxel, robot_size, true)) {
      return true;
    } else {
      return false;
    }
  }

  for (int i = 0; i < (path.size() - 1); ++i) {
    Eigen::Vector3d start_point(path[i].position.x, path[i].position.y,
                                path[i].position.z);
    Eigen::Vector3d end_point(path[i + 1].position.x, path[i + 1].position.y,
                              path[i + 1].position.z);
    if (VoxelStatus::kFree !=
        voxel_map_->getPathStatus(start_point, end_point, robot_size, true)) {
      return false;
    }
  }
  return true;
}

bool LocalExplorationPlanner::searchPathThroughCenterPoint(const StateVec& current_state,
                                       const Eigen::Vector3d& center,
                                       const double& heading,
                                       Eigen::Vector3d& robot_size,
                                       std::vector<geometry_msgs::Pose>& path) {
  path.clear();
  double L_line = darpa_gate_params_.line_search_length;
  double rotation_step = darpa_gate_params_.line_search_step;
  int n_lines = (int)(darpa_gate_params_.line_search_range / rotation_step);
  geometry_msgs::Pose current_pose;
  convertStateToPoseMsg(current_state, current_pose);
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose end_pose;

  Eigen::Vector3d start_point;
  Eigen::Vector3d end_point;

  for (int i = 0; i < n_lines; ++i) {
    double heading_sample = heading + i * rotation_step;
    Eigen::Vector3d u_vec(std::cos(heading_sample), std::sin(heading_sample),
                          0);
    start_point = center - u_vec * 0.5 * L_line;
    end_point = center + u_vec * 0.5 * L_line;
    StateVec start_state, end_state;
    start_state << start_point.x(), start_point.y(), start_point.z(), 0.0, 0.0;
    end_state << end_point.x(), end_point.y(), end_point.z(), 0.0, 0.0;
    convertStateToPoseMsg(start_state, start_pose);
    convertStateToPoseMsg(end_state, end_pose);
    path.clear();
    path.push_back(current_pose);
    path.push_back(start_pose);
    path.push_back(end_pose);
    if (isPathCollisionFree(path, robot_size)) {
      if(global_verbosity >= Verbosity::INFO) {
        std::cout << "Best voxel to go: " << center << std::endl;
        std::cout << "With robot size: " << robot_size << std::endl;
      }
      return true;
    }

    if (i > 0) {
      heading_sample = heading - i * rotation_step;
      u_vec << std::cos(heading_sample), std::sin(heading_sample), 0;
      start_point = center - u_vec * 0.5 * L_line;
      end_point = center + u_vec * 0.5 * L_line;
      start_state << start_point.x(), start_point.y(), start_point.z(), 0.0, 0.0;
      end_state << end_point.x(), end_point.y(), end_point.z(), 0.0, 0.0;
      convertStateToPoseMsg(start_state, start_pose);
      convertStateToPoseMsg(end_state, end_pose);
      path.clear();
      path.push_back(current_pose);
      path.push_back(start_pose);
      path.push_back(end_pose);
      if (isPathCollisionFree(path, robot_size)) {
        if(global_verbosity >= Verbosity::INFO) {
          std::cout << "Best voxel to go: " << center << std::endl;
          std::cout << "With robot size: " << robot_size << std::endl;
        }
        return true;
      }
    }
  }

  return false;
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::searchPathToPassGate() {
  std::vector<geometry_msgs::Pose> ret_path;
  if (!darpa_gate_params_.enable) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Not allow to run search to pass the gate");
    return ret_path;
  }
  Eigen::Vector3d gate_center = darpa_gate_params_.center;
  double gate_heading = darpa_gate_params_.heading;
  if (darpa_gate_params_.load_from_darpa_frame) {
    tf::StampedTransform tfW2G;
    tf::TransformListener listener;
    try {
      listener.waitForTransform(darpa_gate_params_.world_frame_id,
                                darpa_gate_params_.gate_center_frame_id,
                                ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform(darpa_gate_params_.world_frame_id,
                               darpa_gate_params_.gate_center_frame_id,
                               ros::Time(0), tfW2G);
      tf::Vector3 vec = tfW2G.getOrigin();
      gate_center << vec.x(), vec.y(), vec.z();
      if(global_verbosity >= Verbosity::INFO) {
        std::cout << "Darpa gate offset: "
                  << darpa_gate_params_.darpa_frame_offset << std::endl;
      }
      gate_center = gate_center + darpa_gate_params_.darpa_frame_offset;
      if(global_verbosity >= Verbosity::INFO) {
        std::cout << "Darpa gate center: " << gate_center << std::endl;
      }
    } catch (tf::TransformException ex) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "%s", ex.what());
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Could not look up TF for gate center.");
      return ret_path;
    }
  }

  double search_radius = darpa_gate_params_.center_search_radius;
  double search_step = darpa_gate_params_.center_search_step;
  int loop_max = (int)(search_radius / search_step);
  Eigen::Vector3d best_voxel;
  Eigen::Vector3d bbx_size;
  bool stop = false;
  for (int bound_level = 0; (!stop) && (bound_level < 3); ++bound_level) {
    robot_params_.setBoundMode((BoundModeType)bound_level);
    robot_params_.getPlanningSize(robot_box_size_);
    bbx_size = robot_box_size_;
    for (double loop = 0; (!stop) && (loop < loop_max); ++loop) {
      if (loop == 0) {
        Eigen::Vector3d voxel;
        voxel = gate_center;
        if ((VoxelStatus::kFree ==
             voxel_map_->getBoxStatus(voxel, bbx_size, true)) &&
            searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                         bbx_size, ret_path)) {
          stop = true;
          break;
        }
      } else {
        for (int i = -loop + 1; i < loop; ++i) {
          Eigen::Vector3d voxel;
          voxel.x() = gate_center.x();
          voxel.y() = gate_center.y() + loop * search_step;
          voxel.z() = gate_center.z() + i * search_step;
          if ((VoxelStatus::kFree ==
               voxel_map_->getBoxStatus(voxel, bbx_size, true)) &&
              searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                           bbx_size, ret_path)) {
            stop = true;
            break;
          }
          voxel.x() = gate_center.x();
          voxel.y() = gate_center.y() - loop * search_step;
          voxel.z() = gate_center.z() + i * search_step;
          if ((VoxelStatus::kFree ==
               voxel_map_->getBoxStatus(voxel, bbx_size, true)) &&
              searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                           bbx_size, ret_path)) {
            stop = true;
            break;
          }
        }
        for (int i = -loop; i <= loop; ++i) {
          Eigen::Vector3d voxel;
          voxel.x() = gate_center.x();
          voxel.y() = gate_center.y() + i * search_step;
          voxel.z() = gate_center.z() + loop * search_step;
          if ((VoxelStatus::kFree ==
               voxel_map_->getBoxStatus(voxel, bbx_size, true)) &&
              searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                           bbx_size, ret_path)) {
            stop = true;
            break;
          }
          voxel.x() = gate_center.x();
          voxel.y() = gate_center.y() + i * search_step;
          voxel.z() = gate_center.z() - loop * search_step;
          if ((VoxelStatus::kFree ==
               voxel_map_->getBoxStatus(voxel, bbx_size, true)) &&
              searchPathThroughCenterPoint(current_state_, voxel, gate_heading,
                                           bbx_size, ret_path)) {
            stop = true;
            break;
          }
        }
      }
    }
  }

  if (stop) {
    setHomingPos();
    if (global_graph_->getNumVertices()) {
      std::vector<Vertex*> vertices_to_add;
      for (int i = 0; i < ret_path.size(); ++i) {
        StateVec st;
        st << ret_path[i].position.x, ret_path[i].position.y,
                    ret_path[i].position.z, 0, 0;
        Vertex* ver =
            new Vertex(i, st);  // temporary id to generate vertex list.
        ver->is_leaf_vertex = false;
        vertices_to_add.push_back(ver);
      }
      commitVerifiedPathToCognitiveMap(global_graph_, vertices_to_add);
      visualization_->visualizeTopologicalGraph(global_graph_);
    }
    visualization_->visualizeRefPath(ret_path);
  } else {
    ret_path.clear();
  }

  if (ret_path.empty()) ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Could not find path to go through.");
  return ret_path;
}

void LocalExplorationPlanner::cameraAnnotationTimerCallback(const ros::TimerEvent& event) {
  if(!planning_params_.annotate_map_with_camera) return;

  for(auto sensor : camera_annotation_params_.sensor_list) {
    std::vector<Eigen::Vector3d> multiray_endpoints;
    camera_annotation_params_.sensor[sensor].rotations[1] = cam_pitch_;
    camera_annotation_params_.sensor[sensor].updateFrustumEndpoints();
    camera_annotation_params_.sensor[sensor].getFrustumEndpoints(current_state_, multiray_endpoints);
    Eigen::Vector3d current_pos = current_state_.head(3);
    voxel_map_->annotateCameraVoxels(current_pos, multiray_endpoints);
  }
}
