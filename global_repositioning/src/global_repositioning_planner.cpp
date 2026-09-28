
#include "global_repositioning/global_repositioning_planner.h"

#include "voxel_map/voxel_map.h"

#include "global_repositioning/branch_repositioning.h"
#include "planner_common/terminal_log.h"
#include "planner_common/trajectory.h"
#include "planner_common/utils.h"
#include "planner_msgs/planner_srv.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <stdexcept>
#include <sstream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <random>
#include <tf/transform_listener.h>

#define SQ(x) (x * x)

namespace global_repositioning {
namespace {

double normalizeYaw(double yaw) {
  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
}

geometry_msgs::Pose stateToPose(const StateVec& state, bool planning_backward) {
  const double yaw = planning_backward ? normalizeYaw(state[3] + M_PI)
                                       : state[3];
  Eigen::Matrix3d rot_eigen =
      (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  rot_eigen = rot_eigen * Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY());

  const Eigen::Quaterniond q_eigen(rot_eigen);
  tf::Quaternion quat;
  quat.setX(q_eigen.x());
  quat.setY(q_eigen.y());
  quat.setZ(q_eigen.z());
  quat.setW(q_eigen.w());

  tf::Vector3 origin(state[0], state[1], state[2]);
  tf::Pose pose_tf(quat, origin);
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(pose_tf, pose);
  return pose;
}

bool buildPosePathFromGraphIds(const std::shared_ptr<CognitiveGraph>& graph,
                               const std::vector<int>& path_ids,
                               bool planning_backward,
                               std::vector<geometry_msgs::Pose>& path) {
  path.clear();
  if (graph == NULL || path_ids.empty()) {
    return false;
  }

  path.reserve(path_ids.size());
  for (int vertex_id : path_ids) {
    if (vertex_id < 0 || vertex_id >= graph->getNumVertices()) {
      path.clear();
      return false;
    }
    Vertex* vertex = graph->getVertex(vertex_id);
    if (vertex == NULL) {
      path.clear();
      return false;
    }
    path.push_back(stateToPose(vertex->state, planning_backward));
  }
  return !path.empty();
}

void applyYawTangentCorrection(bool planning_backward,
                               std::vector<geometry_msgs::Pose>& path) {
  if (path.size() < 2) {
    return;
  }

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Eigen::Vector3d vec(path[i + 1].position.x - path[i].position.x,
                              path[i + 1].position.y - path[i].position.y,
                              path[i + 1].position.z - path[i].position.z);
    const double yaw =
        planning_backward ? std::atan2(-vec.y(), -vec.x())
                          : std::atan2(vec.y(), vec.x());
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, yaw);
    path[i + 1].orientation.x = quat.x();
    path[i + 1].orientation.y = quat.y();
    path[i + 1].orientation.z = quat.z();
    path[i + 1].orientation.w = quat.w();
  }
}

}  // namespace
}  // namespace global_repositioning

global_repositioning::GlobalRepositioningPlanner::GlobalRepositioningPlanner(
    GlobalRepositioningContext context)
    : context_(std::move(context)),
      planning_params_(*context_.planning_params),
      robot_params_(*context_.robot_params),
      voxel_map_(context_.voxel_map),
      global_graph_(*context_.global_graph),
      global_graph_rep_(*context_.global_graph_report),
      current_state_(*context_.current_state),
      robot_box_size_(*context_.robot_box_size),
      global_repositioning_state_(*context_.state),
      num_low_gain_iters_(*context_.num_low_gain_iters),
      auto_global_planner_trig_(*context_.auto_global_planner_triggered),
      global_exploration_ongoing_(*context_.global_exploration_ongoing),
      current_global_vertex_id_(*context_.current_global_vertex_id),
      homing_engaged_(*context_.homing_engaged),
      world_frame_(*context_.world_frame) {
  if (context_.planning_params == nullptr || context_.robot_params == nullptr ||
      context_.global_graph == nullptr ||
      context_.global_graph_report == nullptr ||
      context_.current_state == nullptr || context_.robot_box_size == nullptr ||
      context_.state == nullptr || context_.num_low_gain_iters == nullptr ||
      context_.auto_global_planner_triggered == nullptr ||
      context_.global_exploration_ongoing == nullptr ||
      context_.current_global_vertex_id == nullptr ||
      context_.homing_engaged == nullptr || context_.world_frame == nullptr) {
    throw std::invalid_argument(
        "GlobalRepositioningPlanner received an incomplete context.");
  }
}

bool global_repositioning::GlobalRepositioningPlanner::isDeadZoneCandidate(bool frontier_exists, double best_gain,
                              int num_leaf_vertices) const {
  if (!planning_params_.dead_zone_escape_enable) {
    return false;
  }
  if (frontier_exists && best_gain > 0.0) {
    return false;
  }
  if (num_low_gain_iters_ < planning_params_.dead_zone_min_low_gain_iters) {
    return false;
  }

  bool weak_topology = true;
  Vertex* nearest_global_vertex = NULL;
  if (global_graph_ != NULL &&
      global_graph_->getNearestVertexInRange(
          &current_state_, planning_params_.dead_zone_region_radius,
          &nearest_global_vertex)) {
    weak_topology =
        global_repositioning::getTopologicalMapDegree(global_graph_,
                                                       nearest_global_vertex) <=
        planning_params_.dead_zone_max_branch_degree;
  }

  const Eigen::Vector3d start =
      current_state_.head(3) + robot_params_.center_offset;
  const Eigen::Vector3d forward(std::cos(current_state_[3]),
                                std::sin(current_state_[3]), 0.0);
  const Eigen::Vector3d end =
      start + forward * planning_params_.dead_zone_front_clearance;
  const VoxelStatus forward_status =
      voxel_map_->getPathStatus(start, end, robot_box_size_, true);
  const bool forward_blocked = forward_status != VoxelStatus::kFree;
  const bool locally_terminal =
      num_leaf_vertices <= std::max(1, planning_params_.dead_zone_max_branch_degree + 1);

  return weak_topology && (forward_blocked || locally_terminal);
}

void global_repositioning::GlobalRepositioningPlanner::markDeadZoneRegion(Vertex* center_vertex) {
  if (!planning_params_.dead_zone_escape_enable || center_vertex == NULL) {
    return;
  }

  std::vector<Vertex*> nearby_vertices;
  global_graph_->getNearestVertices(&center_vertex->state,
                                    planning_params_.dead_zone_region_radius,
                                    &nearby_vertices);
  nearby_vertices.push_back(center_vertex);

  for (Vertex* vertex : nearby_vertices) {
    if (vertex == NULL) {
      continue;
    }
    if (vertex->type == VertexType::kFrontier && vertex != center_vertex) {
      continue;
    }
    vertex->type = VertexType::kDeadZone;
    if (std::find(global_repositioning_state_.dead_zone.vertex_ids.begin(), global_repositioning_state_.dead_zone.vertex_ids.end(),
                  vertex->id) == global_repositioning_state_.dead_zone.vertex_ids.end()) {
      global_repositioning_state_.dead_zone.vertex_ids.push_back(vertex->id);
    }
  }

  global_repositioning_state_.dead_zone.latest_vertex_id = center_vertex->id;
  ROS_WARN_STREAM_COND(
      global_verbosity >= Verbosity::WARN,
      planner_log::tag(
          planner_log::Channel::kSafety, "Dead-Zone",
          "Marked " +
              std::to_string(global_repositioning_state_.dead_zone.vertex_ids.size()) +
              " cognitive-map vertices around node " +
              std::to_string(center_vertex->id) +
              " as dead-zone labels."));
}

double global_repositioning::GlobalRepositioningPlanner::getDeadZoneDistance(const Eigen::Vector3d& position) const {
  if (!planning_params_.dead_zone_escape_enable ||
      global_repositioning_state_.dead_zone.vertex_ids.empty() || global_graph_ == NULL) {
    return std::numeric_limits<double>::infinity();
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (int vertex_id : global_repositioning_state_.dead_zone.vertex_ids) {
    if (vertex_id < 0 || vertex_id >= global_graph_->getNumVertices()) {
      continue;
    }
    Vertex* dead_vertex = global_graph_->getVertex(vertex_id);
    if (dead_vertex == NULL) {
      continue;
    }
    min_distance =
        std::min(min_distance, (dead_vertex->state.head(3) - position).norm());
  }
  return min_distance;
}

double global_repositioning::GlobalRepositioningPlanner::getDeadZonePenalty(const Eigen::Vector3d& position) const {
  const double distance = getDeadZoneDistance(position);
  if (!std::isfinite(distance)) {
    return 1.0;
  }
  return 1.0 - std::exp(-planning_params_.dead_zone_penalty_lambda * distance);
}

bool global_repositioning::GlobalRepositioningPlanner::tryPrependDeadZoneEscapeManeuver(
    std::vector<geometry_msgs::Pose>& path) const {
  if (!planning_params_.dead_zone_escape_enable ||
      global_repositioning_state_.dead_zone.latest_vertex_id < 0 || path.size() < 2) {
    return false;
  }

  if (!global_repositioning::prependDeadZoneEscapeManeuver(
          current_state_, robot_params_.center_offset, robot_box_size_,
          voxel_map_, planning_params_.dead_zone_turn_radius,
          planning_params_.dead_zone_turn_step,
          planning_params_.dead_zone_turn_clearance, path)) {
    ROS_WARN_STREAM_COND(
        global_verbosity >= Verbosity::WARN,
        planner_log::tag(planner_log::Channel::kSafety, "Dead-Zone",
                         "U-shaped dead-end escape is infeasible; using "
                         "topological backtracking path."));
    return false;
  }

  ROS_WARN_STREAM_COND(
      global_verbosity >= Verbosity::WARN,
      planner_log::tag(planner_log::Channel::kSafety, "Dead-Zone",
                       "Prepended U-shaped dead-end escape before global "
                       "repositioning."));
  return true;
}

bool global_repositioning::GlobalRepositioningPlanner::refreshGlobalFrontiers(
    const std::vector<Vertex*>& frontiers_to_check,
    std::vector<Vertex*>& verified_frontiers) {
  verified_frontiers.clear();
  verified_frontiers.reserve(frontiers_to_check.size());
  int checked_count = 0;
  int expired_count = 0;
  int zero_gain_count = 0;
  for (Vertex* frontier : frontiers_to_check) {
    if (frontier == NULL || frontier->type != VertexType::kFrontier) {
      continue;
    }
    ++checked_count;
    computeVolumetricGainRayModelNoBound(frontier->state, frontier->vol_gain);
    if (!frontier->vol_gain.is_frontier) {
      frontier->type = VertexType::kUnvisited;
      ++expired_count;
    } else if (frontier->vol_gain.gain <= 0.0) {
      ++zero_gain_count;
      verified_frontiers.push_back(frontier);
    } else {
      verified_frontiers.push_back(frontier);
    }
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG ||
                    planning_params_.opening_frontier_seed_enable,
                "[TopologicalRepositioning] checked=%d verified=%d expired=%d "
                "zero_gain=%d",
                checked_count, static_cast<int>(verified_frontiers.size()),
                expired_count, zero_gain_count);
  return !verified_frontiers.empty();
}

bool global_repositioning::GlobalRepositioningPlanner::evaluatePointGlobalFrontiers(
    const std::vector<Vertex*>& frontiers_to_evaluate,
    const ShortestPathsReport& frontier_graph_rep, bool ignore_time,
    const StateVec& cur_state, Vertex*& best_frontier, double& best_gain,
    int& evaluated_frontier_count, double& point_total_time) {
  best_frontier = NULL;
  best_gain = -1.0;
  evaluated_frontier_count = 0;
  Timer point_timer;

  std::vector<Vertex*> verified_frontiers;
  refreshGlobalFrontiers(frontiers_to_evaluate, verified_frontiers);
  const double point_update_time = point_timer.endTimer();

  std::vector<BranchFrontierCandidate> frontier_candidates;
  global_repositioning::FrontierEvaluationOptions options;
  options.ignore_time = ignore_time;
  options.auto_homing_enable = planning_params_.auto_homing_enable;
  options.select_closest_frontier = planning_params_.select_closest_frontier;
  options.v_homing_max = planning_params_.v_homing_max;
  options.region_count = planning_params_.branch_repositioning_region_count;
  if (!global_repositioning::collectFrontierCandidates(
          verified_frontiers, global_graph_, frontier_graph_rep,
          global_graph_rep_, cur_state, options,
          [this](double time_cost, double& time_spare) {
            return isRemainingTimeSufficient(time_cost, time_spare);
          },
          [this](const Eigen::Vector3d& source,
                 const Eigen::Vector3d& target) {
            return getZoneTravelFactor(source, target);
          },
          [this](const Eigen::Vector3d& position) {
            return getDeadZonePenalty(position);
          },
          frontier_candidates)) {
    point_total_time = point_update_time;
    return false;
  }
  evaluated_frontier_count = static_cast<int>(frontier_candidates.size());

  Timer point_selection_timer;
  for (const auto& candidate : frontier_candidates) {
    if (candidate.exp_gain > best_gain) {
      best_gain = candidate.exp_gain;
      best_frontier = candidate.frontier;
    }
  }
  global_repositioning::applyTargetHysteresis(
      frontier_candidates, planning_params_.global_frontier_stickiness_enable,
      global_exploration_ongoing_, current_global_vertex_id_,
      planning_params_.global_frontier_stickiness_ratio, global_graph_,
      best_frontier, best_gain);
  point_total_time = point_update_time + point_selection_timer.endTimer();
  return best_frontier != NULL && best_gain >= 0.0;
}

void global_repositioning::GlobalRepositioningPlanner::updateAdaptiveGlobalFrontierCountStats(int frontier_count) {
  global_repositioning::updateFrontierCountStats(
      frontier_count, planning_params_.branch_repositioning_memory_alpha,
      global_repositioning_state_.branch);
}

void global_repositioning::GlobalRepositioningPlanner::updateAdaptiveBranchRegionSuccess(bool success) {
  global_repositioning::updateBranchRegionSuccess(
      success, planning_params_.branch_repositioning_memory_alpha,
      global_repositioning_state_.branch);
}

BranchTargetSelectionMode global_repositioning::GlobalRepositioningPlanner::chooseBranchTargetSelectionMode(
    const std::vector<Vertex*>& global_frontiers,
    const ShortestPathsReport& frontier_graph_rep) const {
  if (!planning_params_.branch_repositioning_selection_enable ||
      !global_repositioning_state_.branch.cache_ready || global_frontiers.empty()) {
    return BranchTargetSelectionMode::kPoint;
  }

  const double compactness =
      global_repositioning::estimateBranchRegionCompactness(
          global_graph_, global_frontiers, global_graph_rep_);
  global_repositioning::BranchSelectionWeights weights;
  weights.frontier_scale =
      planning_params_.branch_repositioning_frontier_scale_weight;
  weights.compactness = planning_params_.branch_repositioning_compactness_weight;
  weights.success = planning_params_.branch_repositioning_success_weight;
  weights.time = planning_params_.branch_repositioning_time_weight;
  const global_repositioning::BranchSelectionScore score =
      global_repositioning::scoreBranchTargetSelection(
          static_cast<int>(global_frontiers.size()), compactness,
          global_repositioning_state_.branch, weights);

  ROS_WARN_STREAM_COND(
      global_verbosity >= Verbosity::WARN,
      planner_log::tag(
          planner_log::Channel::kGlobal, "Global Repositioning",
          "Adaptive branch-aware mode selection: frontier_nodes=" +
              std::to_string(static_cast<int>(global_frontiers.size())) +
              ", frontier_ema=" +
              planner_log::num(global_repositioning_state_.branch.frontier_count_ema) +
              ", compactness=" + planner_log::num(score.compactness) +
              ", region_success=" + planner_log::num(score.reliability) +
              ", point_score=" + planner_log::num(score.point_score) +
              ", region_score=" + planner_log::num(score.region_score)));

  return score.mode;
}

bool global_repositioning::GlobalRepositioningPlanner::selectGlobalFrontierTarget(
    const std::vector<Vertex*>& global_frontiers,
    const ShortestPathsReport& frontier_graph_rep, bool ignore_time,
    const StateVec& cur_state, Vertex*& best_frontier, double& best_gain,
    int& evaluated_frontier_count, bool& final_branch_region_mode) {
  best_frontier = NULL;
  best_gain = -1.0;
  evaluated_frontier_count = 0;
  final_branch_region_mode = false;

  const BranchTargetSelectionMode selection_mode =
      chooseBranchTargetSelectionMode(global_frontiers, frontier_graph_rep);
  updateAdaptiveGlobalFrontierCountStats(
      static_cast<int>(global_frontiers.size()));

  if (selection_mode == BranchTargetSelectionMode::kRegion) {
    std::vector<Vertex*> preselected_frontiers;
    Timer preselection_timer;
    global_repositioning::FrontierEvaluationOptions options;
    options.ignore_time = ignore_time;
    options.auto_homing_enable = planning_params_.auto_homing_enable;
    options.select_closest_frontier = planning_params_.select_closest_frontier;
    options.v_homing_max = planning_params_.v_homing_max;
    options.region_count = planning_params_.branch_repositioning_region_count;
    options.constrain_to_nearest_branch_entry =
        planning_params_.branch_entry_distance_constraint_enable;
    options.branch_entry_distance_tolerance =
        planning_params_.branch_entry_distance_tolerance;
    const bool preselected =
        global_repositioning::preselectFrontiersByBranchRegion(
            global_frontiers, global_graph_, frontier_graph_rep,
            global_graph_rep_, cur_state, options,
            [this](double time_cost, double& time_spare) {
              return isRemainingTimeSufficient(time_cost, time_spare);
            },
            [this](const Eigen::Vector3d& source,
                   const Eigen::Vector3d& target) {
              return getZoneTravelFactor(source, target);
            },
            [this](const Eigen::Vector3d& position) {
              return getDeadZonePenalty(position);
            },
            preselected_frontiers);
    const double preselection_time = preselection_timer.endTimer();
    if (preselected) {
      ROS_WARN_STREAM(planner_log::tag(
          planner_log::Channel::kGlobal, "Global Repositioning",
          "Using branch-aware frontier filtering: selected " +
              std::to_string(static_cast<int>(preselected_frontiers.size())) +
              " / " +
              std::to_string(static_cast<int>(global_frontiers.size())) +
              " frontier nodes."));

      Timer update_timer;
      std::vector<Vertex*> verified_frontiers;
      refreshGlobalFrontiers(preselected_frontiers, verified_frontiers);
      const double point_update_time = update_timer.endTimer();
      if (!verified_frontiers.empty()) {
        std::vector<BranchFrontierCandidate> frontier_candidates;
        if (global_repositioning::collectFrontierCandidates(
                verified_frontiers, global_graph_, frontier_graph_rep,
                global_graph_rep_, cur_state, options,
                [this](double time_cost, double& time_spare) {
                  return isRemainingTimeSufficient(time_cost, time_spare);
                },
                [this](const Eigen::Vector3d& source,
                       const Eigen::Vector3d& target) {
                  return getZoneTravelFactor(source, target);
                },
                [this](const Eigen::Vector3d& position) {
                  return getDeadZonePenalty(position);
                },
                frontier_candidates)) {
          Vertex* region_frontier = NULL;
          double region_gain = -1.0;
          Timer region_selection_timer;
          if (global_repositioning::selectFrontierByBranchRegion(
                  frontier_candidates, global_graph_, frontier_graph_rep,
                  global_graph_rep_,
                  planning_params_.branch_repositioning_region_count,
                  planning_params_.branch_entry_distance_constraint_enable,
                  planning_params_.branch_entry_distance_tolerance,
                  [this](const Eigen::Vector3d& position) {
                    return getDeadZonePenalty(position);
                  },
                  region_frontier, region_gain)) {
            const double region_selection_time =
                region_selection_timer.endTimer();
            best_frontier = region_frontier;
            best_gain = region_gain;
            evaluated_frontier_count =
                static_cast<int>(frontier_candidates.size());
            final_branch_region_mode = true;
            global_repositioning::applyTargetHysteresis(
                frontier_candidates,
                planning_params_.global_frontier_stickiness_enable,
                global_exploration_ongoing_, current_global_vertex_id_,
                planning_params_.global_frontier_stickiness_ratio,
                global_graph_, best_frontier, best_gain);
            visualizeBranchEntry(global_repositioning::findBranchRegionEntry(
                global_graph_, best_frontier, global_graph_rep_));
            const double region_total_time =
                std::max(0.0, preselection_time) + point_update_time +
                std::max(0.0, region_selection_time);
            updateAdaptiveGlobalSelectionTimings(-1.0, region_total_time);
            updateAdaptiveBranchRegionSuccess(true);
            return best_frontier != NULL && best_gain >= 0.0;
          }
        }
      }
    }

    ROS_WARN_STREAM_COND(
        global_verbosity >= Verbosity::WARN,
        planner_log::tag(
            planner_log::Channel::kGlobal, "Global Repositioning",
            "Branch-aware filtering unavailable; falling back to point-level "
            "frontier utility evaluation."));
    updateAdaptiveBranchRegionSuccess(false);
  }

  ROS_WARN_STREAM(planner_log::tag(
      planner_log::Channel::kGlobal, "Global Repositioning",
      "Using point-level frontier utility evaluation."));
  double point_total_time = -1.0;
  const bool point_success = evaluatePointGlobalFrontiers(
      global_frontiers, frontier_graph_rep, ignore_time, cur_state,
      best_frontier, best_gain, evaluated_frontier_count, point_total_time);
  if (point_success) {
    global_repositioning_state_.branch.cache_ready = true;
    updateAdaptiveGlobalSelectionTimings(point_total_time, -1.0);
  }
  return point_success;
}

bool global_repositioning::GlobalRepositioningPlanner::validateManualGlobalTarget(
    int vertex_id, const ShortestPathsReport& frontier_graph_rep,
    bool ignore_time, Vertex*& best_frontier, double& best_gain) {
  best_frontier = NULL;
  best_gain = -1.0;

  ROS_WARN_STREAM_COND(
      global_verbosity >= Verbosity::WARN,
      planner_log::tag(planner_log::Channel::kGlobal,
                       "Global Repositioning", "Manual mode."));

  std::vector<int> current_to_target_path_id;
  std::vector<int> target_to_home_path_id;
  global_graph_->getShortestPath(vertex_id, frontier_graph_rep, true,
                                 current_to_target_path_id);
  global_graph_->getShortestPath(vertex_id, global_graph_rep_, false,
                                 target_to_home_path_id);

  const double current_to_target_distance =
      global_graph_->getShortestDistance(vertex_id, frontier_graph_rep);
  const double target_to_home_distance =
      global_graph_->getShortestDistance(vertex_id, global_graph_rep_);
  const double time_to_target =
      current_to_target_distance / planning_params_.v_homing_max;
  const double time_to_home =
      target_to_home_distance / planning_params_.v_homing_max;

  double time_cost = 0.0;
  if (!ignore_time) {
    ROS_INFO_STREAM_COND(
        global_verbosity >= Verbosity::INFO,
        planner_log::tag(planner_log::Channel::kGlobal,
                         "Global Repositioning",
                         "Time remaining: " +
                             planner_log::num(getTimeRemained()) + " s"));
    time_cost += time_to_target;
    ROS_INFO_STREAM_COND(
        global_verbosity >= Verbosity::INFO,
        planner_log::tag(planner_log::Channel::kGlobal,
                         "Global Repositioning",
                         "Time to vertex " + std::to_string(vertex_id) +
                             ": " + planner_log::num(time_to_target) + " s"));
    if (planning_params_.auto_homing_enable) {
      time_cost += time_to_home;
      ROS_INFO_STREAM_COND(
          global_verbosity >= Verbosity::INFO,
          planner_log::tag(planner_log::Channel::kGlobal,
                           "Global Repositioning",
                           "Time to home: " +
                               planner_log::num(time_to_home) + " s"));
    }
  }

  double time_spare = 0.0;
  if (!isRemainingTimeSufficient(time_cost, time_spare)) {
    ROS_WARN_STREAM_COND(
        global_verbosity >= Verbosity::WARN,
        planner_log::tag(planner_log::Channel::kGlobal,
                         "Global Repositioning",
                         "Not enough time to go to vertex " +
                             std::to_string(vertex_id) + "."));
    ROS_WARN_STREAM_COND(
        global_verbosity >= Verbosity::WARN,
        planner_log::tag(planner_log::Channel::kGlobal,
                         "Global Repositioning",
                         "Consider another vertex ID or set ignore_time=true."));
    return false;
  }

  best_frontier = global_graph_->getVertex(vertex_id);
  best_gain = 1.0;
  return best_frontier != NULL;
}

void global_repositioning::GlobalRepositioningPlanner::updateAdaptiveGlobalSelectionTimings(double point_time,
                                               double region_time) {
  global_repositioning::updateSelectionTimings(
      point_time, region_time, planning_params_.branch_repositioning_memory_alpha,
      global_repositioning_state_.branch);
}

std::vector<geometry_msgs::Pose> global_repositioning::GlobalRepositioningPlanner::replanCurrent(int &status) {
  return plan(current_global_vertex_id_, true, true, status);
}

std::vector<geometry_msgs::Pose> global_repositioning::GlobalRepositioningPlanner::plan(int vertex_id,
                                                       bool not_check_frontier,
                                                       bool ignore_time,
                                                       int &status) {
  // @not_check_frontier: just check if it is feasible (collision-free + time)
  // @ignore_time: don't consider time budget.
  const auto global_timing_start = std::chrono::high_resolution_clock::now();
  auto stage_timing_start = global_timing_start;
  double frontier_update_time_ms = 0.0;
  double graph_connection_time_ms = 0.0;
  double graph_search_time_ms = 0.0;
  double target_selection_time_ms = 0.0;
  double path_generation_time_ms = 0.0;
  double dead_zone_escape_time_ms = -1.0;
  double safety_enhancement_time_ms = 0.0;
  const bool is_global_frontier_fallback = (vertex_id == 0 && !not_check_frontier);
  ROS_INFO_STREAM(planner_log::tag(
      planner_log::Channel::kGlobal, "Global Repositioning",
      "Cognitive-map-guided repositioning started: target_vertex=" +
          std::to_string(vertex_id) +
          ", frontier_check=" + planner_log::boolText(!not_check_frontier) +
          ", time_check=" + planner_log::boolText(!ignore_time)));

  visualizeTopologicalGraph(global_graph_);

  // Check whether the topological map still has frontier vertices.
  // Get the list of current frontiers.
  //
  ROS_INFO_STREAM(planner_log::stage(
      planner_log::Channel::kGlobal, "Global Repositioning",
      "Update global topological graph and frontier candidates"));
  if (vertex_id) {
    ros::Duration(3.0).sleep();  // sleep to unblock the thread to get and
                                 // update all latest pose update.
    ros::spinOnce();
  }
  
  
  status = planner_msgs::planner_srv::Response::kRepositioning;

  if(!vertex_id && planning_params_.enable_opening_traversal && planning_params_.exploration_only)  // Don't do opening traversal if asked to go to a specific vertex
  {
    status = -1;
		return getOpeningTraversalPath();	
  }

  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();
  // Check if the global planner exists
  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN_STREAM_COND(
        global_verbosity >= Verbosity::WARN,
        planner_log::tag(planner_log::Channel::kMap, "Cognitive Map",
                         "Global topological graph is empty; no frontier can be selected."));
    return ret_path;
  }
  // Check if the vertex id exists
  if ((vertex_id < 0) || (vertex_id >= global_graph_->getNumVertices())) {
    ROS_WARN_STREAM_COND(
        global_verbosity >= Verbosity::WARN,
        planner_log::tag(
            planner_log::Channel::kMap, "Cognitive Map",
            "Requested topological vertex is invalid; valid id range is [0, " +
                std::to_string(global_graph_->getNumVertices() - 1) + "]."));
    return ret_path;
  }
  // Check if the time endurance is still available.
  if (!ignore_time) {
    if (getTimeRemained() <= 0.0) {
      ROS_WARN_STREAM_COND(
          global_verbosity >= Verbosity::WARN,
          planner_log::tag(planner_log::Channel::kGlobal,
                           "Global Repositioning",
                           "Ran out of time; stopping global repositioning."));
      return ret_path;
    }
  }
  // Check if exists any frontiers in the graph. Frontier gains are refreshed
  // later, after the current state is connected and branch-region preselection
  // has enough graph-distance context.
  std::vector<Vertex*> global_frontiers;
  int num_vertices = global_graph_->getNumVertices();
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG,
                "Collect current global frontiers.");
  global_frontiers.clear();
  for (int id = 0; id < num_vertices; ++id) {
    if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
      global_frontiers.push_back(global_graph_->getVertex(id));
    }
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG,
                "Currently have %d frontier-labeled vertices in the topological map.",
                (int)global_frontiers.size());
  frontier_update_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::high_resolution_clock::now() - stage_timing_start)
          .count();
  if ((!not_check_frontier) && (global_frontiers.size() <= 0)) {
    ROS_WARN_STREAM_COND(
        global_verbosity >= Verbosity::PLANNER_STATUS,
        planner_log::tag(planner_log::Channel::kGlobal,
                         "Global Repositioning",
                         "No valid frontier remains in the cognitive map."));
    // Keep exploring or go home if no more frontiers
    if (planning_params_.go_home_if_fully_explored) {
      ROS_WARN_STREAM_COND(
          global_verbosity >= Verbosity::PLANNER_STATUS,
          planner_log::tag(planner_log::Channel::kGlobal,
                           "Global Repositioning",
                           "Switching to return-home path."));
      ret_path = getHomingPath(world_frame_);
      homing_engaged_ = true;
    } else {
      num_low_gain_iters_ = 0;
    }
    status = planner_msgs::planner_srv::Response::kHoming;
    return ret_path;
  }
  // Insert the current state into the topological map when feasible.
  ROS_INFO_STREAM(planner_log::stage(
      planner_log::Channel::kMap, "Cognitive Map",
      "Connect current UAV state to topological graph"));
  stage_timing_start = std::chrono::high_resolution_clock::now();
  StateVec cur_state;
  cur_state << current_state_[0], current_state_[1], current_state_[2],
      current_state_[3], current_state_[4];
  cur_state[2] += planning_params_.topological_anchor_z_offset;
  Vertex* link_vertex = NULL;
  const double kRadiusLimit = 1.5;  // 0.5
  bool connected_to_graph =
      anchorStateInCognitiveMap(global_graph_, cur_state, link_vertex, kRadiusLimit);

  if (!connected_to_graph) {
    ROS_WARN_STREAM_COND(
        global_verbosity >= Verbosity::WARN,
        planner_log::tag(planner_log::Channel::kMap, "Cognitive Map",
                         "Failed to insert current UAV state into the global topological graph."));
    return ret_path;
  }
  graph_connection_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::high_resolution_clock::now() - stage_timing_start)
          .count();

  ROS_INFO_STREAM(planner_log::stage(
      planner_log::Channel::kMap, "Cognitive Map",
      "Current state connected; search graph paths"));
  stage_timing_start = std::chrono::high_resolution_clock::now();
  // Get Dijsktra path from home to all.
  if (!global_graph_->findShortestPaths(global_graph_rep_)) {
    ROS_ERROR_STREAM_COND(
        global_verbosity >= Verbosity::ERROR,
        planner_log::tag(planner_log::Channel::kMap, "Cognitive Map",
                         "Failed to compute home-rooted topological distances."));
    return ret_path;
  }
  // Get Dijsktra path from current to all.
  ShortestPathsReport frontier_graph_rep;
  if (!global_graph_->findShortestPaths(link_vertex->id, frontier_graph_rep)) {
    ROS_ERROR_STREAM_COND(
        global_verbosity >= Verbosity::ERROR,
        planner_log::tag(planner_log::Channel::kMap, "Cognitive Map",
                         "Failed to compute query-rooted topological distances."));
    return ret_path;
  }
  graph_search_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::high_resolution_clock::now() - stage_timing_start)
          .count();
  // Check if the planner should find the best vertex automatically or manually
  double best_gain = -1.0;
  Vertex* best_frontier = NULL;

  stage_timing_start = std::chrono::high_resolution_clock::now();
  if (vertex_id) {
    if (!validateManualGlobalTarget(vertex_id, frontier_graph_rep,
                                    ignore_time, best_frontier, best_gain)) {
      return ret_path;
    }
  } else {
    // Auto mode
    ROS_INFO_STREAM(planner_log::stage(
        planner_log::Channel::kGlobal, "Global Repositioning",
        "Topology-guided branch-aware frontier selection"));
    bool final_branch_region_mode = false;
    int evaluated_frontier_count = 0;
    if (!selectGlobalFrontierTarget(global_frontiers, frontier_graph_rep,
                                    ignore_time, cur_state, best_frontier,
                                    best_gain, evaluated_frontier_count,
                                    final_branch_region_mode)) {
      ROS_INFO_STREAM_COND(
          global_verbosity >= Verbosity::INFO,
          planner_log::tag(
              planner_log::Channel::kGlobal, "Global Repositioning",
              "No feasible frontier passes utility and feasibility checks."));
      return ret_path;
    }
    if (final_branch_region_mode) {
      ROS_INFO_STREAM(planner_log::stage(
          planner_log::Channel::kGlobal, "Global Repositioning",
          "Target mode: branch-aware region, evaluated=" +
              std::to_string(evaluated_frontier_count) + "/" +
              std::to_string(global_frontiers.size()) +
              ", selected_frontier=" +
              std::to_string(best_frontier != NULL ? best_frontier->id : -1) +
              ", gain=" + planner_log::num(best_gain)));
    } else {
      ROS_INFO_STREAM(planner_log::stage(
          planner_log::Channel::kGlobal, "Global Repositioning",
          "Target mode: point-level frontier, evaluated=" +
              std::to_string(evaluated_frontier_count) + "/" +
              std::to_string(global_frontiers.size()) +
              ", selected_frontier=" +
              std::to_string(best_frontier != NULL ? best_frontier->id : -1) +
              ", gain=" + planner_log::num(best_gain)));
    }
  }
  target_selection_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::high_resolution_clock::now() - stage_timing_start)
          .count();

  std::vector<int> current_to_frontier_path_id;
  std::vector<int> frontier_to_home_path_id;
  stage_timing_start = std::chrono::high_resolution_clock::now();
  if (best_gain >= 0) {
    ROS_INFO_STREAM(planner_log::stage(
        planner_log::Channel::kGlobal, "Global Repositioning",
        "Generate topological repositioning path to frontier " +
            std::to_string(best_frontier->id)));

    if (auto_global_planner_trig_) {
      current_global_vertex_id_ = best_frontier->id;
      global_exploration_ongoing_ = true;
    }

    global_graph_->getShortestPath(best_frontier->id, frontier_graph_rep, true,
                                   current_to_frontier_path_id);
    global_graph_->getShortestPath(best_frontier->id, global_graph_rep_, false,
                                   frontier_to_home_path_id);
    if (!buildPosePathFromGraphIds(
            global_graph_, current_to_frontier_path_id,
            planning_params_.planning_backward, ret_path)) {
      ROS_WARN_STREAM_COND(
          global_verbosity >= Verbosity::WARN,
          planner_log::tag(planner_log::Channel::kGlobal,
                           "Global Repositioning",
                           "Failed to convert graph path to poses."));
      return ret_path;
    }
  } else {
    ROS_WARN_STREAM_COND(
        global_verbosity >= Verbosity::WARN,
        planner_log::tag(planner_log::Channel::kGlobal,
                         "Global Repositioning",
                         "No positive-utility frontier was selected; "
                         "return-home fallback may be required."));
    return ret_path;
  }

  if (global_repositioning_state_.dead_zone.escape_pending) {
    ROS_INFO_STREAM(planner_log::stage(
        planner_log::Channel::kSafety, "Dead-Zone",
        "Prepend U-shaped dead-end escape maneuver to repositioning path"));
    const auto dead_zone_escape_start = std::chrono::high_resolution_clock::now();
    tryPrependDeadZoneEscapeManeuver(ret_path);
    dead_zone_escape_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - dead_zone_escape_start)
            .count();
    global_repositioning_state_.dead_zone.escape_pending = false;
  }

  if (planning_params_.yaw_tangent_correction) {
    applyYawTangentCorrection(
        planning_params_.planning_backward, ret_path);
  }
  path_generation_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::high_resolution_clock::now() - stage_timing_start)
          .count();
  if (dead_zone_escape_time_ms > 0.0) {
    path_generation_time_ms =
        std::max(0.0, path_generation_time_ms - dead_zone_escape_time_ms);
  }

  // Modify path if required
  if (planning_params_.path_safety_enhance_enable) {
    const auto safety_timing_start = std::chrono::high_resolution_clock::now();
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret_path, mod_path, true)) {
      ret_path = mod_path;
    }
    safety_enhancement_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - safety_timing_start)
            .count();
    ROS_INFO_STREAM_COND(
        global_verbosity >= Verbosity::INFO,
        planner_log::tag(
            planner_log::Channel::kGlobal, "Global Repositioning",
            "Safety-enhanced global path computed: time=" +
                planner_log::num(safety_enhancement_time_ms) + " ms"));
    visualizeModPath(mod_path);
  }

  visualizeGlobalPaths(
      global_graph_, current_to_frontier_path_id, frontier_to_home_path_id);

  // Path interpolation:
  const double kInterpolationDistance =
      planning_params_.path_interpolation_distance;
  std::vector<geometry_msgs::Pose> interp_path;
  if (Trajectory::interpolatePath(ret_path, kInterpolationDistance,
                                  interp_path)) {
    ret_path = interp_path;
  }

  const double total_global_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::high_resolution_clock::now() - global_timing_start)
          .count();
  std::ostringstream timing;
  constexpr int kTimingNameWidth = 34;
  constexpr int kTimingValueWidth = 12;
  auto write_border = [&]() {
    timing << "  +" << std::string(kTimingNameWidth + 2, '-')
           << "+" << std::string(kTimingValueWidth + 2, '-') << "+\n";
  };
  auto write_row = [&](const std::string& name, double value) {
    timing << "  | " << std::left << std::setw(kTimingNameWidth) << name
           << " | " << std::right << std::setw(kTimingValueWidth) << value
           << " |\n";
  };
  timing << "\n"
         << planner_log::paint(planner_log::Channel::kMission,
                               "Global Repositioning Timing")
         << "\n"
         << std::fixed << std::setprecision(3);
  write_border();
  timing << "  | " << std::left << std::setw(kTimingNameWidth) << "Stage"
         << " | " << std::right << std::setw(kTimingValueWidth) << "Time (ms)"
         << " |\n";
  write_border();
  write_row("Frontier candidate update", frontier_update_time_ms);
  write_row("Current-state graph connection", graph_connection_time_ms);
  write_row("Topological shortest paths", graph_search_time_ms);
  write_row("Branch/frontier selection", target_selection_time_ms);
  write_row("Repositioning path generation", path_generation_time_ms);
  if (dead_zone_escape_time_ms > 0.0) {
    write_row("U-shaped dead-zone escape", dead_zone_escape_time_ms);
  }
  if (safety_enhancement_time_ms > 0.001) {
    write_row("Safety enhancement", safety_enhancement_time_ms);
  }
  write_border();
  write_row("Total global repositioning", total_global_time_ms);
  write_border();
  ROS_INFO_STREAM(timing.str());

  accumulateGlobalFallbackPathLength(ret_path);
  if (is_global_frontier_fallback) {
    accumulateGlobalFrontierFallbackMetrics(ret_path);
  }
  publishAblationMetrics();
  visualizeRefPath(ret_path);
  return ret_path;
}

void global_repositioning::GlobalRepositioningPlanner::visualizeTopologicalGraph(
    const std::shared_ptr<CognitiveGraph>& graph) {
  if (context_.callbacks.visualize_topological_graph) {
    context_.callbacks.visualize_topological_graph(graph);
  }
}

void global_repositioning::GlobalRepositioningPlanner::visualizeModPath(
    const std::vector<geometry_msgs::Pose>& path) {
  if (context_.callbacks.visualize_modified_path) {
    context_.callbacks.visualize_modified_path(path);
  }
}

void global_repositioning::GlobalRepositioningPlanner::visualizeGlobalPaths(
    const std::shared_ptr<CognitiveGraph>& graph,
    const std::vector<int>& current_to_frontier_path,
    const std::vector<int>& frontier_to_home_path) {
  if (context_.callbacks.visualize_global_paths) {
    context_.callbacks.visualize_global_paths(
        graph, current_to_frontier_path, frontier_to_home_path);
  }
}

void global_repositioning::GlobalRepositioningPlanner::visualizeRefPath(
    const std::vector<geometry_msgs::Pose>& path) {
  if (context_.callbacks.visualize_reference_path) {
    context_.callbacks.visualize_reference_path(path);
  }
}

void global_repositioning::GlobalRepositioningPlanner::visualizeBranchEntry(
    int entry_vertex_id) {
  if (context_.callbacks.visualize_branch_entry) {
    context_.callbacks.visualize_branch_entry(global_graph_, entry_vertex_id);
  }
}

void global_repositioning::GlobalRepositioningPlanner::
    computeVolumetricGainRayModelNoBound(StateVec& state,
                                         VolumetricGain& gain) {
  if (context_.callbacks.compute_frontier_gain) {
    context_.callbacks.compute_frontier_gain(state, gain);
  }
}

bool global_repositioning::GlobalRepositioningPlanner::
    isRemainingTimeSufficient(double time_cost, double& time_spare) {
  if (!context_.callbacks.is_remaining_time_sufficient) {
    time_spare = 1.0;
    return true;
  }
  return context_.callbacks.is_remaining_time_sufficient(time_cost, time_spare);
}

double global_repositioning::GlobalRepositioningPlanner::getTimeRemained()
    const {
  if (!context_.callbacks.get_time_remaining) {
    return std::numeric_limits<double>::infinity();
  }
  return context_.callbacks.get_time_remaining();
}

double global_repositioning::GlobalRepositioningPlanner::getZoneTravelFactor(
    const Eigen::Vector3d& source, const Eigen::Vector3d& target) const {
  if (!context_.callbacks.get_zone_travel_factor) {
    return 1.0;
  }
  return context_.callbacks.get_zone_travel_factor(source, target);
}

bool global_repositioning::GlobalRepositioningPlanner::anchorStateInCognitiveMap(
    std::shared_ptr<CognitiveGraph> graph, StateVec& state,
    Vertex*& added_vertex, double radius_limit) {
  if (!context_.callbacks.anchor_state_in_cognitive_map) {
    return false;
  }
  return context_.callbacks.anchor_state_in_cognitive_map(
      graph, state, added_vertex, radius_limit);
}

std::vector<geometry_msgs::Pose>
global_repositioning::GlobalRepositioningPlanner::getHomingPath(
    const std::string& frame) {
  if (!context_.callbacks.get_homing_path) {
    return {};
  }
  return context_.callbacks.get_homing_path(frame);
}

std::vector<geometry_msgs::Pose>
global_repositioning::GlobalRepositioningPlanner::getOpeningTraversalPath() {
  if (!context_.callbacks.get_opening_traversal_path) {
    return {};
  }
  return context_.callbacks.get_opening_traversal_path();
}

bool global_repositioning::GlobalRepositioningPlanner::improveFreePath(
    const std::vector<geometry_msgs::Pose>& path,
    std::vector<geometry_msgs::Pose>& modified_path, bool allow_shortcut) {
  if (!context_.callbacks.improve_free_path) {
    return false;
  }
  return context_.callbacks.improve_free_path(
      path, modified_path, allow_shortcut);
}

void global_repositioning::GlobalRepositioningPlanner::
    accumulateGlobalFallbackPathLength(
        const std::vector<geometry_msgs::Pose>& path) {
  if (context_.callbacks.accumulate_global_fallback_path_length) {
    context_.callbacks.accumulate_global_fallback_path_length(path);
  }
}

void global_repositioning::GlobalRepositioningPlanner::
    accumulateGlobalFrontierFallbackMetrics(
        const std::vector<geometry_msgs::Pose>& path) {
  if (context_.callbacks.accumulate_global_frontier_fallback_metrics) {
    context_.callbacks.accumulate_global_frontier_fallback_metrics(path);
  }
}

void global_repositioning::GlobalRepositioningPlanner::
    publishAblationMetrics() {
  if (context_.callbacks.publish_ablation_metrics) {
    context_.callbacks.publish_ablation_metrics();
  }
}
