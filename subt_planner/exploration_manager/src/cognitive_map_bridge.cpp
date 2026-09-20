#include "local_exploration/local_exploration_planner.h"

#include <algorithm>
#include <random>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

bool LocalExplorationPlanner::setHomingPos() {
  if (global_graph_->getNumVertices() == 0) {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Global graph is empty: add current state as homing position.");
    Vertex* g_root_vertex =
        new Vertex(global_graph_->generateVertexID(), current_state_);
    global_graph_->addVertex(g_root_vertex);
    return true;
  } else {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Global graph is not empty, can not set current state as homing.");
    return false;
  }
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::searchHomingPath(
    std::string tgt_frame, const StateVec& current_state) {
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();

  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Topological graph is empty; no homing path can be searched.");
    return ret_path;
  }

  StateVec cur_state;
  cur_state << current_state[0], current_state[1], current_state[2],
      current_state[3], current_state_[4];
  Vertex* nearest_vertex = NULL;
  if (!global_graph_->getNearestVertex(&cur_state, &nearest_vertex))
    return ret_path;
  if (nearest_vertex == NULL) return ret_path;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();

  Vertex* link_vertex = NULL;
  const double kRadiusLimit = 1.0;
  bool connect_state_to_graph = true;
  if (direction_norm <= kRadiusLimit) {
    Vertex* new_vertex =
        new Vertex(global_graph_->generateVertexID(), cur_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    global_graph_->addVertex(new_vertex);
    global_graph_->addEdge(new_vertex, nearest_vertex, direction_norm);
    ExpandGraphReport rep;
    expandGraphEdges(global_graph_, new_vertex, rep);
    link_vertex = new_vertex;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Connecting current state to the topological graph.");
    ExpandGraphReport rep;
    expandGraph(global_graph_, cur_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Current state connected.");
      link_vertex = rep.vertex_added;
    } else {
      connect_state_to_graph = false;
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Failed to connect current state:");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorShortEdge.");
          break;
        default:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorUnknown.");
          break;
      }
      ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "\033[1;34m[Cognitive Map]\033[0m Failed to find a topological path.");
    }
  }

  if (connect_state_to_graph) {
    if (!global_graph_->findShortestPaths(global_graph_rep_)) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "\033[1;34m[Cognitive Map]\033[0m Failed to solve shortest paths.");
      return ret_path;
    }
    std::vector<int> homing_path_id;
    global_graph_->getShortestPath(link_vertex->id, global_graph_rep_, false,
                                   homing_path_id);
    if (homing_path_id.empty() || homing_path_id.back() != 0) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "\033[1;34m[Cognitive Map]\033[0m Could not find a return-home path.");
      return ret_path;
    }
    int homing_path_id_size = homing_path_id.size();
    for (int i = 0; i < homing_path_id_size; ++i) {
      StateVec state = global_graph_->getVertex(homing_path_id[i])->state;
      tf::Quaternion quat;
      Eigen::Matrix3d rot_eigen;
      rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
      rot_eigen = rot_eigen * Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY());
      Eigen::Quaterniond q_eigen(rot_eigen);
      quat.setX(q_eigen.x());
      quat.setY(q_eigen.y());
      quat.setZ(q_eigen.z());
      quat.setW(q_eigen.w());
      tf::Vector3 origin(state[0], state[1], state[2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      ret_path.push_back(pose);
    }
    bool is_similar = comparePathWithDirectionApprioximately(
        ret_path, tf::getYaw(ret_path[0].orientation));
    for (int i = 0; i < (ret_path.size() - 1); ++i) {
      Eigen::Vector3d vec;
      if ((!planning_params_.homing_backward) || (is_similar)) {
        vec << ret_path[i + 1].position.x - ret_path[i].position.x,
            ret_path[i + 1].position.y - ret_path[i].position.y,
            ret_path[i + 1].position.z - ret_path[i].position.z;
      } else if (planning_params_.homing_backward) {
        vec << ret_path[i].position.x - ret_path[i + 1].position.x,
            ret_path[i].position.y - ret_path[i + 1].position.y,
            ret_path[i].position.z - ret_path[i + 1].position.z;
      }
      double yaw = std::atan2(vec[1], vec[0]);
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      ret_path[i + 1].orientation.x = quat.x();
      ret_path[i + 1].orientation.y = quat.y();
      ret_path[i + 1].orientation.z = quat.z();
      ret_path[i + 1].orientation.w = quat.w();
    }
    visualization_->visualizeHomingPath(global_graph_, global_graph_rep_,
                                        link_vertex->id);
  }
  visualization_->visualizeTopologicalGraph(global_graph_);

  return ret_path;
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::getGlobalPath(
    geometry_msgs::PoseStamped& waypoint) {
  std::vector<geometry_msgs::Pose> ret_path;
  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Topological graph is empty; no homing path can be searched.");
    return ret_path;
  }

  StateVec cur_state;
  cur_state << current_state_[0], current_state_[1], current_state_[2],
      current_state_[3], current_state_[4];

  StateVec wp;
  wp << waypoint.pose.position.x, waypoint.pose.position.y,
      waypoint.pose.position.z;

  Vertex* wp_nearest_vertex;
  if (!global_graph_->getNearestVertex(&wp, &wp_nearest_vertex)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot find any nearby vertex to reposition.");
    return ret_path;
  } else if (wp_nearest_vertex == NULL) {
    return ret_path;
  } else {
    Eigen::Vector3d diff(wp_nearest_vertex->state.x() - wp.x(),
                         wp_nearest_vertex->state.y() - wp.y(),
                         wp_nearest_vertex->state.z() - wp.z());
    if (diff.norm() > max_difference_waypoint_to_graph) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
          "Waypoint is too far from the topological map (distance is '%.2f'; max "
          "allowed is '%.2f'). Choose a closer waypoint.",
          diff.norm(), max_difference_waypoint_to_graph);
      return ret_path;
    }
  }

  Vertex* nearest_vertex = NULL;
  if (!global_graph_->getNearestVertex(&cur_state, &nearest_vertex))
    return ret_path;
  if (nearest_vertex == NULL) return ret_path;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(cur_state[0] - origin[0], cur_state[1] - origin[1],
                            cur_state[2] - origin[2]);
  double direction_norm = direction.norm();

  Vertex* link_vertex = NULL;
  const double kRadiusLimit = 1.0;
  bool connect_state_to_graph = true;
  if (direction_norm <= kRadiusLimit) {
    Vertex* new_vertex =
        new Vertex(global_graph_->generateVertexID(), cur_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    global_graph_->addVertex(new_vertex);
    global_graph_->addEdge(new_vertex, nearest_vertex, direction_norm);
    ExpandGraphReport rep;
    expandGraphEdges(global_graph_, new_vertex, rep);
    link_vertex = new_vertex;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Connecting current state to the topological graph.");
    ExpandGraphReport rep;
    expandGraph(global_graph_, cur_state, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Current state connected.");
      link_vertex = rep.vertex_added;
    } else {
      connect_state_to_graph = false;
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Failed to connect current state:");
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorShortEdge.");
          break;
        default:
          ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "kErrorUnknown.");
          break;
      }
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Failed to find a topological path.");
    }
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Finding a path from current[%d] to vertex[%d].", link_vertex->id,
           wp_nearest_vertex->id);

  if (connect_state_to_graph) {
    if (!global_graph_->findShortestPaths(link_vertex->id, global_graph_rep_)) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "\033[1;34m[Cognitive Map]\033[0m Failed to solve shortest paths.");
      return ret_path;
    }
    std::vector<int> global_path_id;
    global_graph_->getShortestPath(wp_nearest_vertex->id, global_graph_rep_,
                                   true, global_path_id);
    if (global_path_id.empty()) {
      ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "\033[1;34m[Cognitive Map]\033[0m Could not find a return-home path.");
      return ret_path;
    }
    int global_path_id_size = global_path_id.size();
    for (int i = 0; i < global_path_id_size; ++i) {
      StateVec state = global_graph_->getVertex(global_path_id[i])->state;
      tf::Quaternion quat;
      Eigen::Matrix3d rot_eigen;
      rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
      rot_eigen = rot_eigen * Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY());
      Eigen::Quaterniond q_eigen(rot_eigen);
      quat.setX(q_eigen.x());
      quat.setY(q_eigen.y());
      quat.setZ(q_eigen.z());
      quat.setW(q_eigen.w());
      tf::Vector3 origin(state[0], state[1], state[2]);
      tf::Pose poseTF(quat, origin);
      geometry_msgs::Pose pose;
      tf::poseTFToMsg(poseTF, pose);
      ret_path.push_back(pose);
    }
    if (planning_params_.yaw_tangent_correction) {
      bool is_similar = comparePathWithDirectionApprioximately(
          ret_path, tf::getYaw(ret_path[0].orientation));
      for (int i = 0; i < (ret_path.size() - 1); ++i) {
        Eigen::Vector3d vec;
        if ((!planning_params_.homing_backward) || (is_similar)) {
          vec << ret_path[i + 1].position.x - ret_path[i].position.x,
              ret_path[i + 1].position.y - ret_path[i].position.y,
              ret_path[i + 1].position.z - ret_path[i].position.z;
        } else if (planning_params_.homing_backward) {
          vec << ret_path[i].position.x - ret_path[i + 1].position.x,
              ret_path[i].position.y - ret_path[i + 1].position.y,
              ret_path[i].position.z - ret_path[i + 1].position.z;
        }
        double yaw = std::atan2(vec[1], vec[0]);
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        ret_path[i + 1].orientation.x = quat.x();
        ret_path[i + 1].orientation.y = quat.y();
        ret_path[i + 1].orientation.z = quat.z();
        ret_path[i + 1].orientation.w = quat.w();
      }
    }
  }
  visualization_->visualizeTopologicalGraph(global_graph_);
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret_path, mod_path, true)) {
      ret_path = mod_path;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Computed an alternate homing path in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }
  const double kInterpolationDistance =
      planning_params_.path_interpolation_distance;
  std::vector<geometry_msgs::Pose> interp_path;
  if (Trajectory::interpolatePath(ret_path, kInterpolationDistance,
                                  interp_path)) {
    ret_path = interp_path;
  }
  visualization_->visualizeRefPath(ret_path);
  return ret_path;
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::getHomingPath(std::string tgt_frame) {
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path = searchHomingPath(tgt_frame, current_state_);
  if (ret_path.size() < 1) return ret_path;
  tf::Quaternion quat;
  Eigen::Matrix3d rot_eigen;
  rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(current_state_[3], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  rot_eigen = rot_eigen * Eigen::AngleAxisd(current_state_[4], Eigen::Vector3d::UnitY());
  Eigen::Quaterniond q_eigen(rot_eigen);
  ret_path[0].orientation.x = q_eigen.x();
  ret_path[0].orientation.y = q_eigen.y();
  ret_path[0].orientation.z = q_eigen.z();
  ret_path[0].orientation.w = q_eigen.w();

  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret_path, mod_path, true)) {
      ret_path = mod_path;
      bool is_similar = comparePathWithDirectionApprioximately(
          ret_path, tf::getYaw(ret_path[0].orientation));
      for (int i = 0; i < (ret_path.size() - 1); ++i) {
        Eigen::Vector3d vec;
        if ((!planning_params_.homing_backward) || (is_similar)) {
          vec << ret_path[i + 1].position.x - ret_path[i].position.x,
              ret_path[i + 1].position.y - ret_path[i].position.y,
              ret_path[i + 1].position.z - ret_path[i].position.z;
        } else if (planning_params_.homing_backward) {
          vec << ret_path[i].position.x - ret_path[i + 1].position.x,
              ret_path[i].position.y - ret_path[i + 1].position.y,
              ret_path[i].position.z - ret_path[i + 1].position.z;
        }
        double yaw = std::atan2(vec[1], vec[0]);
        tf::Quaternion quat;
        quat.setEuler(0.0, 0.0, yaw);
        ret_path[i + 1].orientation.x = quat.x();
        ret_path[i + 1].orientation.y = quat.y();
        ret_path[i + 1].orientation.z = quat.z();
        ret_path[i + 1].orientation.w = quat.w();
      }
    }

    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Computed an alternate homing path in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }
  const double kInterpolationDistance =
      planning_params_.path_interpolation_distance;
  std::vector<geometry_msgs::Pose> interp_path;
  if (Trajectory::interpolatePath(ret_path, kInterpolationDistance,
                                  interp_path)) {
    ret_path = interp_path;
  }

  visualization_->visualizeRefPath(ret_path);

  return ret_path;
}
bool LocalExplorationPlanner::homingRequired(std::vector<geometry_msgs::Pose> &homing_path)
{
  homing_path.clear();
  double time_elapsed = 0.0;
  if ((ros::Time::now()).toSec() != 0.0) {
    if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
    time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
  }
  double time_budget_remaining =
      planning_params_.time_budget_limit - time_elapsed;
  double time_remaining =
      std::min(time_budget_remaining, current_battery_time_remaining_);
  homing_path = searchHomingPath(world_frame_, current_state_);
  if (!homing_path.empty()) {
    double homing_len = Trajectory::getPathLength(homing_path);
    double time_to_home = homing_len / planning_params_.v_homing_max;
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Time to home: %f; Time remaining: %f", time_to_home,
              time_remaining);

    const double kTimeDelta = 20;
    if (time_to_home > time_remaining - kTimeDelta) {
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "REACHED TIME LIMIT: HOMING ENGAGED.");
      if (planning_params_.path_safety_enhance_enable) {
        std::vector<geometry_msgs::Pose> mod_path;
        if (improveFreePath(homing_path, mod_path, true)) {
          homing_path = mod_path;
        }
      }

      const double kInterpolationDistance =
          planning_params_.path_interpolation_distance;
      std::vector<geometry_msgs::Pose> interp_path;
      if (Trajectory::interpolatePath(homing_path, kInterpolationDistance,
                                      interp_path)) {
        homing_path = interp_path;
      }

      visualization_->visualizeRefPath(homing_path);
      homing_engaged_ = true;
      return true;
    }
    else
      return false;  // Homing not required
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot find a path to return home from here.");
    return false;  // Homing might be needed but can't find path
  }
  return false;
}
void LocalExplorationPlanner::addFrontiers(int best_vertex_id) {

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Global graph: %d vertices, %d edges.",
           global_graph_->getNumVertices(), global_graph_->getNumEdges());
  bool update_global_frontiers = true;
  if (update_global_frontiers) {
    std::vector<Vertex*> global_frontiers;
    int num_vertices = global_graph_->getNumVertices();
    for (int id = 0; id < num_vertices; ++id) {
      if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
        global_frontiers.push_back(global_graph_->getVertex(id));
      }
    }
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Have %d frontiers from the topological map.",
             (int)global_frontiers.size());
    for (auto& v : global_frontiers) {
      computeVolumetricGainRayModelNoBound(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier) v->type = VertexType::kUnvisited;
    }
  }
  std::vector<Vertex*> leaf_vertices;
  local_graph_->getLeafVertices(leaf_vertices);
  std::vector<Vertex*> frontier_vertices;
  if (planning_params_.add_only_frontiers_to_global_graph) {
    for (const auto& entry : local_graph_->vertices_map_) {
      Vertex* v = entry.second;
      if (v != NULL && v->type == VertexType::kFrontier && !v->is_hanging) {
        frontier_vertices.push_back(v);
      }
    }
  } else {
    for (auto& v : leaf_vertices) {
      if (v != NULL) {
        frontier_vertices.push_back(v);
      }
    }
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Got %d leaf vertices from the updated exploration graph.",
           (int)leaf_vertices.size());
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Got %d frontiers from the updated exploration graph.",
           (int)frontier_vertices.size());
  std::vector<int> cluster_ids = performShortestPathsClustering(
      local_graph_, local_graph_rep_, frontier_vertices);
  visualization_->visualizeClusteredPaths(local_graph_, local_graph_rep_,
                                          frontier_vertices, cluster_ids);
  double kRangeCheck;
  double kUpdateRadius;
  if(planning_params_.add_only_frontiers_to_global_graph) {
    kRangeCheck = 0.5;
    kUpdateRadius = 2.0;
  }
  else {
    kRangeCheck = 0.5;
    kUpdateRadius = 2.0;
  }
  for (int i = 0; i < cluster_ids.size(); ++i) {
    Vertex* nearest_vertex = NULL;
    if (!global_graph_->getNearestVertexInRange(
            &(local_graph_->getVertex(cluster_ids[i])->state), kRangeCheck,
            &nearest_vertex)) {
      StateVec* nearest_state = NULL;
      if (!robot_state_hist_->getNearestStateInRange(
              &(local_graph_->getVertex(cluster_ids[i])->state), kUpdateRadius,
              &nearest_state)) {
        std::vector<Vertex*> path;
        local_graph_->getShortestPath(cluster_ids[i], local_graph_rep_, true,
                                      path);
        for (auto pa = path.begin(); pa != (path.end() - 1); ++pa) {
          (*pa)->type = VertexType::kUnvisited;
        }
        commitVerifiedPathToCognitiveMap(global_graph_, path);
      }
    }
  }
  addOpeningFrontierSeeds();
  visualization_->visualizeRobotStateHistory(robot_state_hist_->state_hist_);
}

int LocalExplorationPlanner::addOpeningFrontierSeeds() {
  if (!planning_params_.opening_frontier_seed_enable ||
      local_graph_ == NULL || global_graph_ == NULL ||
      root_vertex_ == NULL) {
    return 0;
  }

  struct SeedCandidate {
    double score;
    double angle;
    std::vector<Vertex*> path;
    StateVec state;
    VolumetricGain vol_gain;
  };

  std::vector<Vertex*> anchors;
  std::vector<Vertex*> leaf_vertices;
  local_graph_->getLeafVertices(leaf_vertices);
  anchors.push_back(root_vertex_);

  auto maybe_add_anchor = [&](Vertex* vertex) {
    if (vertex == NULL || vertex->is_hanging) {
      return;
    }
    const double distance =
        (vertex->state.head(3) - root_vertex_->state.head(3)).norm();
    if (distance > planning_params_.opening_frontier_seed_search_radius) {
      return;
    }
    if (std::find(anchors.begin(), anchors.end(), vertex) == anchors.end()) {
      anchors.push_back(vertex);
    }
  };

  for (Vertex* vertex : leaf_vertices) {
    maybe_add_anchor(vertex);
  }
  const size_t max_anchor_count =
      std::max<size_t>(8, planning_params_.opening_frontier_seed_max_num * 4);
  if (anchors.size() < max_anchor_count) {
    for (const auto& entry : local_graph_->vertices_map_) {
      maybe_add_anchor(entry.second);
      if (anchors.size() >= max_anchor_count) {
        break;
      }
    }
  }

  const int angle_bins =
      std::max(8, planning_params_.opening_frontier_seed_angle_bins);
  const double step =
      std::max(voxel_map_->getResolution(), planning_params_.edge_length_min);
  const double min_free =
      std::max(step, planning_params_.opening_frontier_seed_min_free_length);
  const double unknown_depth =
      std::max(step, planning_params_.opening_frontier_seed_unknown_depth);
  const double min_clearance =
      std::max(0.0, planning_params_.opening_frontier_seed_min_clearance);
  const std::vector<double> pitch_angles = {0.0, 0.30, -0.30};

  BoundedSpaceParams local_sampling_space = local_space_params_;
  Eigen::Vector3d local_sampling_center = root_vertex_->state.head(3);
  local_sampling_space.setCenter(local_sampling_center, false);
  std::vector<SeedCandidate> candidates;
  int rejected_free = 0;
  int rejected_unknown = 0;
  int rejected_global_duplicate = 0;
  int rejected_history_duplicate = 0;
  int rejected_gain_refresh = 0;
  int rejected_angle = 0;
  int rejected_connect = 0;

  for (Vertex* anchor : anchors) {
    std::vector<Vertex*> root_to_anchor;
    local_graph_->getShortestPath(anchor->id, local_graph_rep_, true,
                                  root_to_anchor);
    if (root_to_anchor.empty()) {
      continue;
    }

    const Eigen::Vector3d anchor_pos = anchor->state.head(3);
    for (int i = 0; i < angle_bins; ++i) {
      const double yaw = -M_PI + 2.0 * M_PI * static_cast<double>(i) /
                                      static_cast<double>(angle_bins);
      for (double pitch : pitch_angles) {
        const double cp = std::cos(pitch);
        const Eigen::Vector3d dir(cp * std::cos(yaw),
                                  cp * std::sin(yaw),
                                  std::sin(pitch));
        Eigen::Vector3d prev = anchor_pos + robot_params_.center_offset;
        Eigen::Vector3d last_free = anchor_pos;
        bool free_prefix_ok = true;
        double free_length = 0.0;

        for (double dist = step; dist <= min_free; dist += step) {
          Eigen::Vector3d sample = anchor_pos + dist * dir;
          if (!local_sampling_space.isInsideSpace(sample) ||
              !global_space_params_.isInsideSpace(sample)) {
            free_prefix_ok = false;
            break;
          }
          const Eigen::Vector3d point =
              sample + robot_params_.center_offset;
          if (voxel_map_->getPathStatus(prev, point, robot_box_size_,
                                          true) != VoxelStatus::kFree ||
              voxel_map_->getBoxStatus(point, robot_box_size_, true) !=
                  VoxelStatus::kFree ||
              voxel_map_->getPointDistance(point) < min_clearance) {
            free_prefix_ok = false;
            break;
          }
          prev = point;
          last_free = sample;
          free_length = dist;
        }
        if (!free_prefix_ok || free_length + kSmallNorm < min_free) {
          ++rejected_free;
          continue;
        }

        int unknown_count = 0;
        for (double dist = min_free + step;
             dist <= min_free + unknown_depth; dist += step) {
          Eigen::Vector3d sample = anchor_pos + dist * dir;
          if (!local_sampling_space.isInsideSpace(sample) ||
              !global_space_params_.isInsideSpace(sample)) {
            break;
          }
          if (voxel_map_->getVoxelStatus(sample) == VoxelStatus::kUnknown) {
            ++unknown_count;
          }
        }
        if (unknown_count <= 0) {
          ++rejected_unknown;
          continue;
        }

        StateVec seed_state = anchor->state;
        seed_state.head(3) = last_free;
        seed_state[3] = yaw;
        seed_state[4] = pitch;

        const double duplicate_radius =
            std::max(1.5 * voxel_map_->getResolution(),
                     std::min(0.45, 0.4 * min_free));
        Vertex* duplicate_vertex = NULL;
        if (global_graph_->getNearestVertexInRange(
                &seed_state, duplicate_radius, &duplicate_vertex)) {
          ++rejected_global_duplicate;
          continue;
        }
        StateVec* hist_state = NULL;
        const double history_duplicate_radius =
            std::max(1.5 * voxel_map_->getResolution(),
                     std::min(0.35, 0.3 * min_free));
        if (robot_state_hist_->getNearestStateInRange(
                &seed_state, history_duplicate_radius, &hist_state)) {
          ++rejected_history_duplicate;
          continue;
        }

        VolumetricGain seed_gain;
        computeVolumetricGainRayModelNoBound(seed_state, seed_gain);
        if (!seed_gain.is_frontier || seed_gain.gain <= 0.0) {
          ++rejected_gain_refresh;
          continue;
        }

        SeedCandidate candidate;
        candidate.score = seed_gain.gain + unknown_count + 0.1 * free_length;
        candidate.angle = std::atan2((last_free - root_vertex_->state.head(3)).y(),
                                     (last_free - root_vertex_->state.head(3)).x());
        candidate.path = root_to_anchor;
        candidate.state = seed_state;
        candidate.vol_gain = seed_gain;
        candidates.push_back(candidate);
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const SeedCandidate& lhs, const SeedCandidate& rhs) {
              return lhs.score > rhs.score;
            });

  auto angle_distance = [](double a, double b) {
    double d = a - b;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return std::abs(d);
  };

  std::vector<double> selected_angles;
  int added = 0;
  for (const SeedCandidate& candidate : candidates) {
    bool too_close = false;
    for (double selected_angle : selected_angles) {
      if (angle_distance(candidate.angle, selected_angle) <
          planning_params_.opening_frontier_seed_min_angle_sep) {
        too_close = true;
        break;
      }
    }
    if (too_close) {
      ++rejected_angle;
      continue;
    }

    Vertex seed_vertex(-1, candidate.state);
    seed_vertex.type = VertexType::kFrontier;
    seed_vertex.vol_gain = candidate.vol_gain;

    std::vector<Vertex*> path = candidate.path;
    path.push_back(&seed_vertex);
    if (commitVerifiedPathToCognitiveMap(global_graph_, path)) {
      ++added;
      selected_angles.push_back(candidate.angle);
    } else {
      ++rejected_connect;
    }
    if (added >= planning_params_.opening_frontier_seed_max_num) {
      break;
    }
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN && added > 0,
                "[OpeningFrontierSeed] Added %d sparse opening frontier seeds.",
                added);
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG ||
                    planning_params_.opening_frontier_seed_enable,
                "[OpeningFrontierSeed] anchors=%d candidates=%d added=%d "
                "reject_free=%d reject_unknown=%d reject_global_dup=%d "
                "reject_history_dup=%d reject_gain_refresh=%d "
                "reject_angle=%d reject_connect=%d",
                static_cast<int>(anchors.size()),
                static_cast<int>(candidates.size()), added, rejected_free,
                rejected_unknown, rejected_global_duplicate,
                rejected_history_duplicate, rejected_gain_refresh,
                rejected_angle, rejected_connect);
  return added;
}

bool LocalExplorationPlanner::resetTimerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    resetMissionTimer();
    ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "Mission time reset.");
    res.success = true;
    return true;
  }

void LocalExplorationPlanner::expandTopologicalFrontierAdditionTimerCallback(
    const ros::TimerEvent& event) {
  if (add_frontiers_to_global_graph_) {
    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = t1;

    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Timer: Adding frontiers to topological map");
    add_frontiers_to_global_graph_ = false;
    addFrontiers(0);  // id given as 0 because it is not used
    
    t2 = std::chrono::high_resolution_clock::now();
    double chrono_time = std::chrono::duration<double, std::milli>(t2 - t1).count();
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Add frontiers time: %f", chrono_time);
  }
}

void LocalExplorationPlanner::expandTopologicalMapTimerCallback(const ros::TimerEvent& event) {
  //

  ros::Time time_lim;
  START_TIMER(time_lim);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;

  if (planner_trigger_count_ == 0) return;

  bool update_global_frontiers = false;
  if (update_global_frontiers) {
    std::vector<Vertex*> global_frontiers;
    int num_vertices = global_graph_->getNumVertices();
    for (int id = 0; id < num_vertices; ++id) {
      if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
        global_frontiers.push_back(global_graph_->getVertex(id));
      }
    }
    for (auto& v : global_frontiers) {
      computeVolumetricGainRayModel(v->state, v->vol_gain);
      if (!v->vol_gain.is_frontier) v->type = VertexType::kUnvisited;
    }
  }

  std::vector<Vertex*> unvisited_vertices;
  int global_graph_size = global_graph_->getNumVertices();
  for (int id = 0; id < global_graph_size; ++id) {
    if (global_graph_->getVertex(id)->type == VertexType::kUnvisited) {
      unvisited_vertices.push_back(global_graph_->getVertex(id));
    }
  }
  if (unvisited_vertices.empty()) return;

  const double kLocalBoxRadius = 10;
  const double kLocalBoxRadiusSq = kLocalBoxRadius * kLocalBoxRadius;
  std::vector<Eigen::Vector3d> cluster_centroids;
  std::vector<Vertex*> unvisited_vertices_remain;
  while (true) {
    unvisited_vertices_remain.clear();
    int ind = rand() % (unvisited_vertices.size());
    Eigen::Vector3d cluster_center(0, 0, 0);
    int num_vertices_in_cluster = 0;
    for (int i = 0; i < unvisited_vertices.size(); ++i) {
      Eigen::Vector3d dist(
          unvisited_vertices[i]->state.x() - unvisited_vertices[ind]->state.x(),
          unvisited_vertices[i]->state.y() - unvisited_vertices[ind]->state.y(),
          unvisited_vertices[i]->state.z() -
              unvisited_vertices[ind]->state.z());
      if (dist.squaredNorm() <= kLocalBoxRadiusSq) {
        cluster_center =
            cluster_center + Eigen::Vector3d(unvisited_vertices[i]->state.x(),
                                             unvisited_vertices[i]->state.y(),
                                             unvisited_vertices[i]->state.z());
        ++num_vertices_in_cluster;
      } else {
        unvisited_vertices_remain.push_back(unvisited_vertices[i]);
      }
    }
    cluster_center = cluster_center / num_vertices_in_cluster;
    cluster_centroids.push_back(cluster_center);
    unvisited_vertices = unvisited_vertices_remain;
    if (unvisited_vertices.empty()) break;
  }
  double time_elapsed = 0;
  int loop_count = 0, loop_count_success = 0;
  int num_vertices = 1;
  int num_edges = 0;
  while (time_elapsed < kTopologicalMapUpdateTimeBudget) {
    time_elapsed = GET_ELAPSED_TIME(time_lim);
    ++loop_count;
    for (int i = 0; i < cluster_centroids.size(); ++i) {
      StateVec centroid_state;
      centroid_state << cluster_centroids[i].x(),
                        cluster_centroids[i].y(),
                        cluster_centroids[i].z(), 0.0, 0.0;
      Vertex new_vertex(-1, StateVec::Zero());
      if (!sampleVertex(random_sampler_, centroid_state, new_vertex)) continue;
      if (new_vertex.is_hanging) continue;
      const double kSparseRadius = 5.0;              // m
      const double kOverlappedFrontierRadius = 5.0;  // m
      std::vector<StateVec*> s_res;
      robot_state_hist_->getNearestStates(&new_vertex.state, kSparseRadius,
                                          &s_res);
      if (s_res.size()) continue;
      std::vector<Vertex*> v_res;
      global_graph_->getNearestVertices(&new_vertex.state, kSparseRadius,
                                        &v_res);
      if (v_res.size()) continue;
      std::vector<Vertex*> f_res;
      global_graph_->getNearestVertices(&new_vertex.state,
                                        kOverlappedFrontierRadius, &f_res);
      bool frontier_existed = false;
      for (auto v : f_res) {
        if (v->type == VertexType::kFrontier) {
          frontier_existed = true;
          break;
        }
      }
      if (frontier_existed) continue;

      loop_count_success++;
      ExpandGraphReport rep;
      expandGraph(global_graph_, new_vertex, rep);
      if (rep.status == ExpandGraphStatus::kSuccess) {
        computeVolumetricGainRayModel(rep.vertex_added->state,
                                      rep.vertex_added->vol_gain, false);
        if (rep.vertex_added->vol_gain.is_frontier)
          rep.vertex_added->type = VertexType::kFrontier;
        num_vertices += rep.num_vertices_added;
        num_edges += rep.num_edges_added;
      }
    }
  }

  time_elapsed = GET_ELAPSED_TIME(time_lim);
  t2 = std::chrono::high_resolution_clock::now();
}

void LocalExplorationPlanner::semanticsCallback(
    const planner_semantic_msgs::SemanticPoint& semantic) {
  StateVec new_state;
  new_state << semantic.point.x, semantic.point.y, semantic.point.z, 0.0, 0.0;
  Eigen::Vector3d sem(new_state[0], new_state[1], new_state[2]);

  if (VoxelStatus::kFree !=
      voxel_map_->getBoxStatus(sem + robot_params_.center_offset,
                                 robot_box_size_, true)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[SEMANTICS]: Marker state not free.");
    return;
  }

  Vertex* temp_nearest_vertex;
  if (!global_graph_->getNearestVertex(&new_state, &temp_nearest_vertex)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[SEMANTICS]: No nearest vertex found.");
    return;
  }
  std::vector<Vertex*> nearest_vertices;
  Vertex* nearest_vertex;
  Eigen::Vector3d nv(temp_nearest_vertex->state[0],
                     temp_nearest_vertex->state[1],
                     temp_nearest_vertex->state[2]);
  if (!global_graph_->getNearestVertices(&new_state, 2.0 * ((sem - nv).norm()),
                                         &nearest_vertices)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[SEMANTICS]: No nearest vertex found.");
    return;
  }

  std::vector<geometry_msgs::Pose> path_ret;
  bool path_status = false;
  const int kMaxNumTrials = 5;
  for (int i = 0; (i < nearest_vertices.size()) && (i < kMaxNumTrials); ++i) {
    nearest_vertex = nearest_vertices[i];
    geometry_msgs::Pose start, end;
    end.position.x = semantic.point.x;
    end.position.y = semantic.point.y;
    end.position.z = semantic.point.z;
    end.orientation.x = 0.0;
    end.orientation.y = 0.0;
    end.orientation.z = 0.0;
    end.orientation.w = 1.0;

    start.position.x = nearest_vertex->state[0];
    start.position.y = nearest_vertex->state[1];
    start.position.z = nearest_vertex->state[2];
    start.orientation.x = 0.0;
    start.orientation.y = 0.0;
    start.orientation.z = 0.0;
    start.orientation.w = 1.0;

    path_status = search(start, end, false, path_ret);

    if (path_status) break;
  }

  if (!path_status) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[SEMANTICS]: Cannot connect to nearest node");
    return;
  } else {
    std::vector<Vertex*> semantic_path;
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, 
        "[SEMANTICS]: Found a path to the semantic point with %d vertices.",
        (int)path_ret.size());
    if (path_ret.size() > 2) {
      for (int i = 1; i < path_ret.size(); ++i) {
        StateVec next_state;
        next_state << path_ret[i].position.x, path_ret[i].position.y,
                      path_ret[i].position.z, 0.0, 0.0;
        Vertex* vert = new Vertex(i, next_state);
        if (i == path_ret.size() - 1) {
          vert->semantic_class.value = semantic.type.value;
          vert->type = VertexType::kFrontier;
          vert->is_leaf_vertex = true;
        } else {
          vert->semantic_class.value =
              planner_semantic_msgs::SemanticClass::kNone;
        }
        semantic_path.push_back(vert);
      }
    } else {
      StateVec next_state;
      next_state << path_ret[1].position.x, path_ret[1].position.y,
                    path_ret[1].position.z, 0.0, 0.0;
      Vertex* vert = new Vertex(global_graph_->generateVertexID(), next_state);
      vert->semantic_class.value = semantic.type.value;
      vert->type = VertexType::kFrontier;
      vert->is_leaf_vertex = true;
      global_graph_->addVertex(vert);
      Eigen::Vector3d tgt_pos(vert->state[0], vert->state[1], vert->state[2]);
      Eigen::Vector3d src_pos(nearest_vertex->state[0],
                              nearest_vertex->state[1],
                              nearest_vertex->state[2]);
      global_graph_->addEdge(nearest_vertex, vert, (tgt_pos - src_pos).norm());
    }
  }
  visualization_->visualizeTopologicalGraph(global_graph_);
}

bool LocalExplorationPlanner::search(geometry_msgs::Pose source_pose,
                 geometry_msgs::Pose target_pose, bool use_current_state,
                 std::vector<geometry_msgs::Pose>& path_ret) {
  StateVec source;
  if (use_current_state)
    source = current_state_;
  else
    convertPoseMsgToState(source_pose, source);
  StateVec target;
  convertPoseMsgToState(target_pose, target);
  std::shared_ptr<CognitiveGraph> graph_search(new CognitiveGraph());
  RandomSamplingParams sampling_params;
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Start searching ...");
  int final_target_id;

  sampling_params.reached_target_radius = 1.0;
  sampling_params.num_vertices_max = 800;
  ConnectStatus status = findPathToConnect(
      source, target, graph_search, sampling_params, final_target_id, path_ret);
  visualization_->visualizeGraph(graph_search);
  visualization_->visualizeSampler(random_sampler_to_search_);
  if (status == ConnectStatus::kSuccess)
    return true;
  else
    return false;
}

ConnectStatus LocalExplorationPlanner::findPathToConnect(
    StateVec& source, StateVec& target,
    std::shared_ptr<CognitiveGraph> graph_manager, RandomSamplingParams& params,
    int& final_target_id, std::vector<geometry_msgs::Pose>& path_ret) {
  ConnectStatus status;
  path_ret.clear();
  graph_manager->reset();

  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Search a path from src [%f,%f,%f] to tgt [%f,%f,%f]", source[0],
           source[1], source[2], target[0], target[1], target[2]);
  VoxelStatus voxel_state;
  bool try_straight_path = true;
  if (try_straight_path) {
    Eigen::Vector3d src_pos(source[0], source[1], source[2]);
    Eigen::Vector3d tgt_pos(target[0], target[1], target[2]);
    voxel_state = voxel_map_->getPathStatus(
        src_pos + robot_params_.center_offset,
        tgt_pos + robot_params_.center_offset, robot_box_size_, false);
    if (voxel_state == VoxelStatus::kFree) {
      ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Try straight path...");
      Vertex* source_vertex =
          new Vertex(graph_manager->generateVertexID(), source);
      graph_manager->addVertex(source_vertex);
      Vertex* target_vertex =
          new Vertex(graph_manager->generateVertexID(), target);
      graph_manager->addVertex(target_vertex);
      graph_manager->addEdge(source_vertex, target_vertex,
                             (tgt_pos - src_pos).norm());
      final_target_id = target_vertex->id;

      geometry_msgs::Pose source_pose;
      convertStateToPoseMsg(source, source_pose);
      path_ret.push_back(source_pose);
      geometry_msgs::Pose target_pose;
      convertStateToPoseMsg(target, target_pose);
      path_ret.push_back(target_pose);
      Eigen::Vector3d vec(path_ret[1].position.x - path_ret[0].position.x,
                          path_ret[1].position.y - path_ret[0].position.y,
                          path_ret[1].position.z - path_ret[0].position.z);
      double yaw = std::atan2(vec[1], vec[0]);
      tf::Quaternion quat;
      Eigen::Matrix3d rot_eigen;
      rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
      rot_eigen = rot_eigen * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY());
      Eigen::Quaterniond q_eigen(rot_eigen);
      path_ret[1].orientation.x = q_eigen.x();
      path_ret[1].orientation.y = q_eigen.y();
      path_ret[1].orientation.z = q_eigen.z();
      path_ret[1].orientation.w = q_eigen.w();

      status = ConnectStatus::kSuccess;
      return status;
    }
  }
  if (params.check_collision_at_source) {
    voxel_state = voxel_map_->getBoxStatus(
        Eigen::Vector3d(source[0], source[1], source[2]) +
            robot_params_.center_offset,
        robot_box_size_, true);
    if (VoxelStatus::kFree != voxel_state) {
      switch (voxel_state) {
        case VoxelStatus::kOccupied:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Source position contains Occupied voxels --> Stop.");
          break;
        case VoxelStatus::kUnknown:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Source position contains Unknown voxels  --> Stop.");
          break;
        case VoxelStatus::kFree:
          break;
      }
      status = ConnectStatus::kErrorCollisionAtSource;
      return status;
    }
  }
  Vertex* source_vertex = new Vertex(graph_manager->generateVertexID(), source);
  graph_manager->addVertex(source_vertex);
  bool reached_target = false;
  int num_paths_to_target = 0;
  std::vector<Vertex*> target_neigbors;
  int loop_count = 0;
  int num_vertices = 0;
  int num_edges = 0;
  random_sampler_to_search_.reset();
  bool stop_sampling = false;
  while (!stop_sampling) {
    Vertex new_vertex(-1, StateVec::Zero());
    if (!sampleVertex(random_sampler_to_search_, source, new_vertex)) continue;
    ExpandGraphReport rep;
    expandGraph(graph_manager, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
      Eigen::Vector3d radius_vec(new_vertex.state[0] - target[0],
                                 new_vertex.state[1] - target[1],
                                 new_vertex.state[2] - target[2]);
      if (radius_vec.norm() < params.reached_target_radius) {
        target_neigbors.push_back(rep.vertex_added);
        reached_target = true;
        ++num_paths_to_target;
        if (num_paths_to_target > params.num_paths_to_target_max)
          stop_sampling = true;
      }
    }
    if ((loop_count >= params.num_loops_cutoff) &&
        (graph_manager->getNumVertices() <= 1)) {
      stop_sampling = true;
    }

    if ((loop_count++ > params.num_loops_max) ||
        (num_vertices > params.num_vertices_max) ||
        (num_edges > params.num_edges_max))
      stop_sampling = true;
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Built a graph with %d vertices and %d edges.",
           graph_manager->getNumVertices(), graph_manager->getNumEdges());
  bool added_target = false;
  Vertex* target_vertex = NULL;
  if (reached_target) {
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Reached target.");
    voxel_state = voxel_map_->getBoxStatus(
        Eigen::Vector3d(target[0], target[1], target[2]) +
            robot_params_.center_offset,
        robot_box_size_, true);
    if (voxel_state == VoxelStatus::kFree) {
      ExpandGraphReport rep;
      expandGraph(graph_manager, target, rep);
      if (rep.status == ExpandGraphStatus::kSuccess) {
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Added target to the graph successfully.");
        num_vertices += rep.num_vertices_added;
        num_edges += rep.num_edges_added;
        added_target = true;
        target_vertex = rep.vertex_added;
      } else {
        ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Cannot expand the graph to connect to the target.");
      }
    } else {
      ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Target is not free, failed to add to the graph.");
    }
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "ConnectStatus::kErrorNoFeasiblePath");
    status = ConnectStatus::kErrorNoFeasiblePath;
    return status;
  }
  ShortestPathsReport graph_rep;
  graph_manager->findShortestPaths(graph_rep);
  if (!added_target) {
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Sorting best path.");
    std::sort(target_neigbors.begin(), target_neigbors.end(),
              [&graph_manager, &graph_rep](const Vertex* a, const Vertex* b) {
                return graph_manager->getShortestDistance(a->id, graph_rep) <
                       graph_manager->getShortestDistance(b->id, graph_rep);
              });
    target_vertex = target_neigbors[0];
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Get shortest path [%d] from %d path.", target_vertex->id,
           (int)target_neigbors.size());
  std::vector<int> path_id_list;
  graph_manager->getShortestPath(target_vertex->id, graph_rep, false,
                                 path_id_list);
  final_target_id = target_vertex->id;
  while (!path_id_list.empty()) {
    geometry_msgs::Pose pose;
    int id = path_id_list.back();
    path_id_list.pop_back();
    convertStateToPoseMsg(graph_manager->getVertex(id)->state, pose);
    path_ret.push_back(pose);
  }
  if (planning_params_.yaw_tangent_correction) {
    for (int i = 0; i < (path_ret.size() - 1); ++i) {
      Eigen::Vector3d vec(path_ret[i + 1].position.x - path_ret[i].position.x,
                          path_ret[i + 1].position.y - path_ret[i].position.y,
                          path_ret[i + 1].position.z - path_ret[i].position.z);
      double yaw = std::atan2(vec[1], vec[0]);
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      path_ret[i + 1].orientation.x = quat.x();
      path_ret[i + 1].orientation.y = quat.y();
      path_ret[i + 1].orientation.z = quat.z();
      path_ret[i + 1].orientation.w = quat.w();
    }
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Finish searching.");
  status = ConnectStatus::kSuccess;
  visualization_->visualizeBestPaths(graph_manager, graph_rep, 0,
                                     final_target_id);
  return status;
}



bool LocalExplorationPlanner::commitVerifiedPathToCognitiveMap(const std::shared_ptr<CognitiveGraph> graph_manager,
                            const std::vector<Vertex*>& vertices) {
  if (vertices.size() <= 0) return false;

  StateVec first_state;
  first_state << vertices[0]->state[0], vertices[0]->state[1],
      vertices[0]->state[2], vertices[0]->state[3], vertices[0]->state[4];
  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&first_state, &nearest_vertex))
    return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(first_state[0] - origin[0],
                            first_state[1] - origin[1],
                            first_state[2] - origin[2]);
  double direction_norm = direction.norm();
  Vertex* parent_vertex = NULL;
  const double kDeltaLimit = 0.1;
  const double kRadiusLimit = 0.5;
  if (direction_norm <= kDeltaLimit) {
    parent_vertex = nearest_vertex;
  } else if (direction_norm <=
             std::max(kRadiusLimit, planning_params_.edge_length_min)) {
    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), first_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    parent_vertex = new_vertex;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Connecting current state to the topological graph.");
    ExpandGraphReport rep;
    Vertex new_vertex(-1, first_state);
    expandGraph(graph_manager, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Current state connected.");
      parent_vertex = rep.vertex_added;
    } else {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Failed to connect current state to the topological graph.");
      return false;
    }
  }
  std::vector<Vertex*> vertex_list;
  vertex_list.push_back(parent_vertex);
  for (int i = 1; i < vertices.size(); ++i) {
    if (vertices[i]->is_hanging) {
      break;
    }
    StateVec new_state;
    new_state << vertices[i]->state[0], vertices[i]->state[1],
        vertices[i]->state[2], vertices[i]->state[3], vertices[i]->state[4];
    Eigen::Vector3d origin(parent_vertex->state[0], parent_vertex->state[1],
                           parent_vertex->state[2]);
    Eigen::Vector3d direction(new_state[0] - origin[0],
                              new_state[1] - origin[1],
                              new_state[2] - origin[2]);
    double direction_norm = direction.norm();

    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), new_state);
    new_vertex->type = vertices[i]->type;
    new_vertex->vol_gain = vertices[i]->vol_gain;
    new_vertex->parent = parent_vertex;
    new_vertex->distance = parent_vertex->distance + direction_norm;
    parent_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, parent_vertex, direction_norm);
    vertex_list.push_back(new_vertex);
    parent_vertex = new_vertex;
  }
  int n_vertices = 0;
  int n_edges = 0;
  for (int i = 0; i < vertex_list.size(); ++i) {
    int num_vertices_added = 0;
    int num_edges_added = 0;
    ExpandGraphReport rep;
    expandGraphEdges(graph_manager, vertex_list[i], rep);
    maybeAddShortcutEdges(graph_manager, vertex_list[i]);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      n_vertices += num_vertices_added;
      n_edges += num_edges_added;
    } else {
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot add this vertex: kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot add this vertex: kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot add this vertex: kErrorShortEdge.");
          break;
      }
    }
  }

  const bool path_intp_add = true;
  const double intp_len = 1.0;  // m
  if (path_intp_add) {
    for (int i = 0; i < (vertex_list.size() - 1); ++i) {
      Eigen::Vector3d start_vertex(vertex_list[i]->state.x(),
                                   vertex_list[i]->state.y(),
                                   vertex_list[i]->state.z());
      Eigen::Vector3d end_vertex(vertex_list[i + 1]->state.x(),
                                 vertex_list[i + 1]->state.y(),
                                 vertex_list[i + 1]->state.z());
      Eigen::Vector3d edge_vec = end_vertex - start_vertex;
      double edge_length = edge_vec.norm();
      if (edge_length <= intp_len) continue;
      edge_vec.normalize();
      int n_intp = (int)std::ceil(edge_length / intp_len);  // segments
      Vertex* prev_vertex = vertex_list[i];
      double acc_len = 0;
      for (int j = 1; j < n_intp; ++j) {
        Eigen::Vector3d new_v;
        new_v = start_vertex + j * intp_len * edge_vec;
        StateVec new_state;
        new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3], vertex_list[i]->state[4];
        Vertex* new_vertex =
            new Vertex(graph_manager->generateVertexID(), new_state);
        graph_manager->addVertex(new_vertex);
        graph_manager->addEdge(new_vertex, prev_vertex, intp_len);
        prev_vertex = new_vertex;
        acc_len += intp_len;
      }
      double last_edge_len = edge_length - acc_len;
      graph_manager->addEdge(prev_vertex, vertex_list[i + 1], last_edge_len);
    }
  }

  return true;
}

bool LocalExplorationPlanner::commitVerifiedPathToCognitiveMap(const std::shared_ptr<CognitiveGraph> graph_manager,
                            const std::vector<geometry_msgs::Pose>& path) {
  if (path.size() <= 0) return false;

  StateVec first_state;
  first_state << path[0].position.x, path[0].position.y, path[0].position.z,
      0.0, 0.0;
  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&first_state, &nearest_vertex))
    return false;
  if (nearest_vertex == NULL) return false;
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(first_state[0] - origin[0],
                            first_state[1] - origin[1],
                            first_state[2] - origin[2]);
  double direction_norm = direction.norm();
  Vertex* parent_vertex = NULL;
  const double kDeltaLimit = 0.1;
  const double kRadiusLimit = 0.5;
  if (direction_norm <= kDeltaLimit) {
    parent_vertex = nearest_vertex;
  } else if (direction_norm <=
             std::max(kRadiusLimit, planning_params_.edge_length_min)) {
    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), first_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    parent_vertex = new_vertex;
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Connecting current state to the topological graph.");
    ExpandGraphReport rep;
    Vertex new_vertex(-1, first_state);
    expandGraph(graph_manager, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Current state connected.");
      parent_vertex = rep.vertex_added;
    } else {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Failed to connect current state to the topological graph.");
      return false;
    }
  }
  std::vector<Vertex*> vertex_list;
  vertex_list.push_back(parent_vertex);
  for (int i = 1; i < path.size(); ++i) {
    StateVec new_state;
    new_state << path[i].position.x, path[i].position.y, path[i].position.z,
        0.0, 0.0;
    Eigen::Vector3d origin(parent_vertex->state[0], parent_vertex->state[1],
                           parent_vertex->state[2]);
    Eigen::Vector3d direction(new_state[0] - origin[0],
                              new_state[1] - origin[1],
                              new_state[2] - origin[2]);
    double direction_norm = direction.norm();

    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), new_state);
    new_vertex->parent = parent_vertex;
    new_vertex->distance = parent_vertex->distance + direction_norm;
    parent_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    graph_manager->addEdge(new_vertex, parent_vertex, direction_norm);
    vertex_list.push_back(new_vertex);
    parent_vertex = new_vertex;
  }
  int n_vertices = 0;
  int n_edges = 0;
  for (int i = 0; i < vertex_list.size(); ++i) {
    int num_vertices_added = 0;
    int num_edges_added = 0;
    ExpandGraphReport rep;
    expandGraphEdges(graph_manager, vertex_list[i], rep);
    maybeAddShortcutEdges(graph_manager, vertex_list[i]);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      n_vertices += num_vertices_added;
      n_edges += num_edges_added;
    } else {
      switch (rep.status) {
        case ExpandGraphStatus::kErrorKdTree:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot add this vertex: kErrorKdTree.");
          break;
        case ExpandGraphStatus::kErrorCollisionEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot add this vertex: kErrorCollisionEdge.");
          break;
        case ExpandGraphStatus::kErrorShortEdge:
          ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot add this vertex: kErrorShortEdge.");
          break;
      }
    }
  }

  const bool path_intp_add = true;
  const double intp_len = 1.0;  // m
  if (path_intp_add) {
    for (int i = 0; i < (vertex_list.size() - 1); ++i) {
      Eigen::Vector3d start_vertex(vertex_list[i]->state.x(),
                                   vertex_list[i]->state.y(),
                                   vertex_list[i]->state.z());
      Eigen::Vector3d end_vertex(vertex_list[i + 1]->state.x(),
                                 vertex_list[i + 1]->state.y(),
                                 vertex_list[i + 1]->state.z());
      Eigen::Vector3d edge_vec = end_vertex - start_vertex;
      double edge_length = edge_vec.norm();
      if (edge_length <= intp_len) continue;
      edge_vec.normalize();
      int n_intp = (int)std::ceil(edge_length / intp_len);  // segments
      Vertex* prev_vertex = vertex_list[i];
      double acc_len = 0;
      for (int j = 1; j < n_intp; ++j) {
        Eigen::Vector3d new_v;
        new_v = start_vertex + j * intp_len * edge_vec;
        StateVec new_state;
        new_state << new_v[0], new_v[1], new_v[2], vertex_list[i]->state[3], vertex_list[i]->state[4];
        Vertex* new_vertex =
            new Vertex(graph_manager->generateVertexID(), new_state);
        graph_manager->addVertex(new_vertex);
        graph_manager->addEdge(new_vertex, prev_vertex, intp_len);
        prev_vertex = new_vertex;
        acc_len += intp_len;
      }
      double last_edge_len = edge_length - acc_len;
      graph_manager->addEdge(prev_vertex, vertex_list[i + 1], last_edge_len);
    }
  }

  return true;
}
