
#include "local_exploration/local_exploration_planner.h"

#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <random>
#include <tf/transform_listener.h>

#define SQ(x) (x * x)

bool LocalExplorationPlanner::isDeadZoneCandidate(bool frontier_exists, double best_gain,
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
        getTopologicalMapDegree(nearest_global_vertex) <=
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

int LocalExplorationPlanner::getTopologicalMapDegree(const Vertex* vertex) const {
  if (vertex == NULL || global_graph_ == NULL) {
    return 0;
  }
  const auto edge_it = global_graph_->edge_map_.find(vertex->id);
  if (edge_it == global_graph_->edge_map_.end()) {
    return 0;
  }
  return static_cast<int>(edge_it->second.size());
}

void LocalExplorationPlanner::markDeadZoneRegion(Vertex* center_vertex) {
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
    if (std::find(dead_zone_vertex_ids_.begin(), dead_zone_vertex_ids_.end(),
                  vertex->id) == dead_zone_vertex_ids_.end()) {
      dead_zone_vertex_ids_.push_back(vertex->id);
    }
  }

  latest_dead_zone_vertex_id_ = center_vertex->id;
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "\033[1;35m[Dead-Zone Recovery]\033[0m Marked %zu graph vertices around node %d as dead zone.",
                dead_zone_vertex_ids_.size(), center_vertex->id);
}

double LocalExplorationPlanner::getDeadZoneDistance(const Eigen::Vector3d& position) const {
  if (!planning_params_.dead_zone_escape_enable ||
      dead_zone_vertex_ids_.empty() || global_graph_ == NULL) {
    return std::numeric_limits<double>::infinity();
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (int vertex_id : dead_zone_vertex_ids_) {
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

double LocalExplorationPlanner::getDeadZonePenalty(const Eigen::Vector3d& position) const {
  const double distance = getDeadZoneDistance(position);
  if (!std::isfinite(distance)) {
    return 1.0;
  }
  return 1.0 - std::exp(-planning_params_.dead_zone_penalty_lambda * distance);
}

bool LocalExplorationPlanner::buildUShapedEscapeManeuver(
    double lateral_sign, const std::vector<geometry_msgs::Pose>& follow_path,
    std::vector<geometry_msgs::Pose>& escape_path) const {
  escape_path.clear();
  if (follow_path.size() < 2) {
    return false;
  }

  const double radius = planning_params_.dead_zone_turn_radius;
  const double step = std::max(0.1, planning_params_.dead_zone_turn_step);
  const double safe_clearance = planning_params_.dead_zone_turn_clearance;
  const Eigen::Vector3d origin = current_state_.head(3);
  const Eigen::Vector3d forward(std::cos(current_state_[3]),
                                std::sin(current_state_[3]), 0.0);
  const Eigen::Vector3d left(-forward.y(), forward.x(), 0.0);
  const Eigen::Vector3d lateral = lateral_sign * left;
  const Eigen::Vector3d center = origin + lateral * radius;

  std::vector<Eigen::Vector3d> points;
  const int samples = std::max(6, static_cast<int>(std::ceil(M_PI * radius / step)));
  points.reserve(samples + 1);
  for (int i = 0; i <= samples; ++i) {
    const double theta = M_PI * static_cast<double>(i) / samples;
    points.push_back(center + (-lateral * std::cos(theta) +
                               forward * std::sin(theta)) * radius);
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3d check_point = points[i] + robot_params_.center_offset;
    const double clearance = voxel_map_->getPointDistance(check_point);
    if (clearance < safe_clearance) {
      return false;
    }
    if (i > 0 &&
        voxel_map_->getPathStatus(points[i - 1], points[i],
                                    robot_box_size_, true) != VoxelStatus::kFree) {
      return false;
    }
  }

  const Eigen::Vector3d follow_next(follow_path[1].position.x,
                                    follow_path[1].position.y,
                                    follow_path[1].position.z);
  if (voxel_map_->getPathStatus(points.back(), follow_next,
                                  robot_box_size_, true) != VoxelStatus::kFree) {
    return false;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const double theta = M_PI * static_cast<double>(i) / samples;
    const double yaw = current_state_[3] + theta;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, std::atan2(std::sin(yaw), std::cos(yaw)));
    geometry_msgs::Pose pose;
    pose.position.x = points[i].x();
    pose.position.y = points[i].y();
    pose.position.z = points[i].z();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    escape_path.push_back(pose);
  }

  return true;
}

bool LocalExplorationPlanner::tryPrependDeadZoneEscapeManeuver(
    std::vector<geometry_msgs::Pose>& path) const {
  if (!planning_params_.dead_zone_escape_enable ||
      latest_dead_zone_vertex_id_ < 0 || path.size() < 2) {
    return false;
  }

  std::vector<geometry_msgs::Pose> escape_path;
  if (!buildUShapedEscapeManeuver(1.0, path, escape_path) &&
      !buildUShapedEscapeManeuver(-1.0, path, escape_path)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "\033[1;35m[Dead-Zone Recovery]\033[0m U-shaped escape is not feasible; using graph backtracking path.");
    return false;
  }

  std::vector<geometry_msgs::Pose> merged_path;
  merged_path.reserve(escape_path.size() + path.size());
  merged_path.insert(merged_path.end(), escape_path.begin(), escape_path.end());
  merged_path.insert(merged_path.end(), path.begin() + 1, path.end());
  path.swap(merged_path);
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "\033[1;35m[Dead-Zone Recovery]\033[0m Prepended a U-shaped escape maneuver before global revisit.");
  return true;
}

bool LocalExplorationPlanner::refreshGlobalFrontiers(
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

bool LocalExplorationPlanner::evaluatePointGlobalFrontiers(
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
  if (!collectBranchFrontierCandidates(verified_frontiers, frontier_graph_rep,
                                       ignore_time, cur_state,
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
  applyGlobalTargetHysteresis(frontier_candidates, best_frontier, best_gain);
  point_total_time = point_update_time + point_selection_timer.endTimer();
  return best_frontier != NULL && best_gain >= 0.0;
}

double LocalExplorationPlanner::estimateBranchRegionCompactness(
    const std::vector<Vertex*>& global_frontiers,
    const ShortestPathsReport& home_graph_rep) const {
  if (global_frontiers.empty()) {
    return 0.0;
  }
  std::unordered_set<int> region_entries;
  int valid_frontier_count = 0;
  for (Vertex* frontier : global_frontiers) {
    if (frontier == NULL || frontier->type != VertexType::kFrontier) {
      continue;
    }
    const int entry_id = findBranchRegionEntry(frontier, home_graph_rep);
    if (entry_id < 0) {
      continue;
    }
    region_entries.insert(entry_id);
    ++valid_frontier_count;
  }
  if (valid_frontier_count <= 0) {
    return 0.0;
  }
  const double compactness =
      1.0 - static_cast<double>(region_entries.size()) /
                static_cast<double>(valid_frontier_count);
  return std::max(0.0, std::min(1.0, compactness));
}

void LocalExplorationPlanner::updateAdaptiveGlobalFrontierCountStats(int frontier_count) {
  const double alpha = std::max(
      0.0, std::min(1.0, planning_params_.branch_repositioning_memory_alpha));
  const double count = std::max(0, frontier_count);
  const double count_sq = count * count;
  if (branch_repositioning_frontier_count_ema_ < 0.0 ||
      !std::isfinite(branch_repositioning_frontier_count_ema_)) {
    branch_repositioning_frontier_count_ema_ = count;
    branch_repositioning_frontier_count_sq_ema_ = count_sq;
    return;
  }
  branch_repositioning_frontier_count_ema_ =
      alpha * count + (1.0 - alpha) * branch_repositioning_frontier_count_ema_;
  branch_repositioning_frontier_count_sq_ema_ =
      alpha * count_sq +
      (1.0 - alpha) * branch_repositioning_frontier_count_sq_ema_;
}

void LocalExplorationPlanner::updateAdaptiveBranchRegionSuccess(bool success) {
  const double alpha = std::max(
      0.0, std::min(1.0, planning_params_.branch_repositioning_memory_alpha));
  const double sample = success ? 1.0 : 0.0;
  branch_repositioning_success_ema_ =
      alpha * sample + (1.0 - alpha) * branch_repositioning_success_ema_;
}

LocalExplorationPlanner::BranchTargetSelectionMode LocalExplorationPlanner::chooseBranchTargetSelectionMode(
    const std::vector<Vertex*>& global_frontiers,
    const ShortestPathsReport& frontier_graph_rep) const {
  if (!planning_params_.branch_repositioning_selection_enable ||
      !branch_repositioning_cache_ready_ || global_frontiers.empty()) {
    return BranchTargetSelectionMode::kPoint;
  }

  const double count = static_cast<double>(global_frontiers.size());
  double scale_pressure = 0.5;
  if (branch_repositioning_frontier_count_ema_ >= 0.0 &&
      branch_repositioning_frontier_count_sq_ema_ >= 0.0) {
    const double variance =
        std::max(1.0, branch_repositioning_frontier_count_sq_ema_ -
                          branch_repositioning_frontier_count_ema_ *
                              branch_repositioning_frontier_count_ema_);
    const double z_score =
        (count - branch_repositioning_frontier_count_ema_) / std::sqrt(variance);
    scale_pressure = 1.0 / (1.0 + std::exp(-z_score));
  }

  const double compactness =
      estimateBranchRegionCompactness(global_frontiers, global_graph_rep_);
  double time_advantage = 0.0;
  if (branch_repositioning_point_time_ema_ >= 0.0 &&
      branch_repositioning_region_time_ema_ >= 0.0 &&
      std::isfinite(branch_repositioning_point_time_ema_) &&
      std::isfinite(branch_repositioning_region_time_ema_)) {
    const double normalizer =
        std::max(1e-3, std::max(branch_repositioning_point_time_ema_,
                                branch_repositioning_region_time_ema_));
    time_advantage = std::tanh((branch_repositioning_point_time_ema_ -
                                branch_repositioning_region_time_ema_) /
                               normalizer);
  }

  const double reliability =
      std::max(0.0, std::min(1.0, branch_repositioning_success_ema_));
  const double region_score =
      planning_params_.branch_repositioning_frontier_scale_weight * scale_pressure +
      planning_params_.branch_repositioning_compactness_weight * compactness +
      planning_params_.branch_repositioning_success_weight *
          (reliability - 0.5) +
      planning_params_.branch_repositioning_time_weight * time_advantage;
  const double point_score =
      planning_params_.branch_repositioning_frontier_scale_weight *
          (1.0 - scale_pressure) +
      0.5 * planning_params_.branch_repositioning_compactness_weight *
          (1.0 - compactness) -
      planning_params_.branch_repositioning_time_weight * time_advantage;

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "\033[1;33m[Global Repositioning]\033[0m Adaptive pre-selection: frontiers=%d "
                "frontier_ema=%.2f compactness=%.3f region_success=%.3f "
                "point_score=%.3f region_score=%.3f",
                static_cast<int>(global_frontiers.size()),
                branch_repositioning_frontier_count_ema_, compactness, reliability,
                point_score, region_score);

  return region_score > point_score ? BranchTargetSelectionMode::kRegion
                                    : BranchTargetSelectionMode::kPoint;
}

bool LocalExplorationPlanner::selectGlobalFrontierTarget(
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
    double preselection_time = -1.0;
    const bool preselected = preselectFrontiersWithBranchRegions(
        global_frontiers, frontier_graph_rep, ignore_time, cur_state,
        preselected_frontiers, preselection_time);
    if (preselected) {
      ROS_WARN("\033[1;33m[Global Repositioning]\033[0m Using branch region planning: "
               "prefilter selected %d / %d frontier-labeled vertices.",
               static_cast<int>(preselected_frontiers.size()),
               static_cast<int>(global_frontiers.size()));

      Timer update_timer;
      std::vector<Vertex*> verified_frontiers;
      refreshGlobalFrontiers(preselected_frontiers, verified_frontiers);
      const double point_update_time = update_timer.endTimer();
      if (!verified_frontiers.empty()) {
        std::vector<BranchFrontierCandidate> frontier_candidates;
        if (collectBranchFrontierCandidates(verified_frontiers,
                                            frontier_graph_rep, ignore_time,
                                            cur_state, frontier_candidates)) {
          Vertex* region_frontier = NULL;
          double region_gain = -1.0;
          double region_selection_time = -1.0;
          if (selectFrontierWithBranchRegions(
                  frontier_candidates, frontier_graph_rep, region_frontier,
                  region_gain, region_selection_time)) {
            best_frontier = region_frontier;
            best_gain = region_gain;
            evaluated_frontier_count =
                static_cast<int>(frontier_candidates.size());
            final_branch_region_mode = true;
            applyGlobalTargetHysteresis(frontier_candidates, best_frontier,
                                        best_gain);
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

    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "\033[1;33m[Global Repositioning]\033[0m Branch region pre-selection failed; "
                  "falling back to full point-level frontier planning.");
    updateAdaptiveBranchRegionSuccess(false);
  }

  ROS_WARN("\033[1;33m[Global Repositioning]\033[0m Using frontier planning.");
  double point_total_time = -1.0;
  const bool point_success = evaluatePointGlobalFrontiers(
      global_frontiers, frontier_graph_rep, ignore_time, cur_state,
      best_frontier, best_gain, evaluated_frontier_count, point_total_time);
  if (point_success) {
    branch_repositioning_cache_ready_ = true;
    updateAdaptiveGlobalSelectionTimings(point_total_time, -1.0);
  }
  return point_success;
}

bool LocalExplorationPlanner::preselectFrontiersWithBranchRegions(
    const std::vector<Vertex*>& global_frontiers,
    const ShortestPathsReport& frontier_graph_rep, bool ignore_time,
    const StateVec& cur_state, std::vector<Vertex*>& selected_frontiers,
    double& estimated_region_time) {
  estimated_region_time = 0.0;
  selected_frontiers.clear();
  if (!planning_params_.branch_repositioning_selection_enable ||
      !branch_repositioning_cache_ready_ || global_frontiers.empty()) {
    return false;
  }

  Timer region_timer;
  std::unordered_map<int, BranchRegionCandidate> region_map;
  std::vector<BranchFrontierCandidate> cached_candidates;
  cached_candidates.reserve(global_frontiers.size());
  for (Vertex* frontier : global_frontiers) {
    if (frontier == NULL || frontier->type != VertexType::kFrontier) {
      continue;
    }
    const int entry_id = findBranchRegionEntry(frontier, global_graph_rep_);
    if (entry_id < 0) {
      continue;
    }
    const double current_to_frontier_distance =
        global_graph_->getShortestDistance(frontier->id, frontier_graph_rep);
    const double frontier_to_home_distance =
        global_graph_->getShortestDistance(frontier->id, global_graph_rep_);
    if (!std::isfinite(current_to_frontier_distance) ||
        !std::isfinite(frontier_to_home_distance) ||
        current_to_frontier_distance < 0.0 ||
        frontier_to_home_distance < 0.0) {
      continue;
    }

    double time_spare = 1.0;
    if (!ignore_time) {
      const double time_cost =
          current_to_frontier_distance / planning_params_.v_homing_max +
          (planning_params_.auto_homing_enable
               ? frontier_to_home_distance / planning_params_.v_homing_max
               : 0.0);
      if (!isRemainingTimeSufficient(time_cost, time_spare)) {
        continue;
      }
    }

    BranchFrontierCandidate cached_candidate;
    cached_candidate.frontier = frontier;
    cached_candidate.current_to_frontier_distance =
        current_to_frontier_distance;
    cached_candidate.frontier_to_home_distance = frontier_to_home_distance;
    cached_candidate.time_spare = time_spare;
    const double cached_gain =
        planning_params_.select_closest_frontier
            ? 1.0
            : std::max(0.0, frontier->vol_gain.gain);
    cached_candidate.exp_gain =
        cached_gain * std::exp(-0.01 * current_to_frontier_distance) *
        getZoneTravelFactor(cur_state.head(3), frontier->state.head(3)) *
        getDeadZonePenalty(frontier->state.head(3)) * time_spare;
    if (cached_candidate.exp_gain <= 0.0) {
      continue;
    }
    cached_candidates.push_back(cached_candidate);
    BranchRegionCandidate& region = region_map[entry_id];
    region.entry_vertex_id = entry_id;
    region.frontiers.push_back(&cached_candidates.back());
  }

  std::vector<BranchRegionCandidate> regions;
  regions.reserve(region_map.size());
  const double kRegionDistancePenalty = 0.01;
  const double kRegionSupplementGainRatio = 0.25;
  for (auto& entry : region_map) {
    BranchRegionCandidate& region = entry.second;
    double max_gain = 0.0;
    double gain_sum = 0.0;
    for (const BranchFrontierCandidate* frontier_candidate :
         region.frontiers) {
      if (frontier_candidate == NULL) {
        continue;
      }
      const double gain = std::max(0.0, frontier_candidate->exp_gain);
      max_gain = std::max(max_gain, gain);
      gain_sum += gain;
    }
    region.aggregated_gain =
        max_gain +
        kRegionSupplementGainRatio * std::max(0.0, gain_sum - max_gain);
    region.current_to_entry_distance =
        global_graph_->getShortestDistance(region.entry_vertex_id,
                                           frontier_graph_rep);
    Vertex* entry_vertex = global_graph_->getVertex(region.entry_vertex_id);
    if (entry_vertex == NULL ||
        !std::isfinite(region.current_to_entry_distance)) {
      continue;
    }
    region.score =
        region.aggregated_gain *
        std::exp(-kRegionDistancePenalty * region.current_to_entry_distance) *
        getDeadZonePenalty(entry_vertex->state.head(3));
    regions.push_back(region);
  }

  std::sort(regions.begin(), regions.end(),
            [](const BranchRegionCandidate& lhs,
               const BranchRegionCandidate& rhs) {
              return lhs.score > rhs.score;
            });

  if (regions.empty()) {
    estimated_region_time = region_timer.endTimer();
    return false;
  }

  const int num_selected_regions = std::max(
      1, std::min(static_cast<int>(regions.size()),
                  planning_params_.branch_repositioning_region_count));
  std::unordered_set<int> selected_ids;
  for (int i = 0; i < num_selected_regions; ++i) {
    for (const BranchFrontierCandidate* frontier_candidate :
         regions[i].frontiers) {
      if (frontier_candidate != NULL && frontier_candidate->frontier != NULL &&
          selected_ids.insert(frontier_candidate->frontier->id).second) {
        selected_frontiers.push_back(frontier_candidate->frontier);
      }
    }
  }

  estimated_region_time = region_timer.endTimer();
  return !selected_frontiers.empty();
}

bool LocalExplorationPlanner::collectBranchFrontierCandidates(
    const std::vector<Vertex*>& global_frontiers,
    const ShortestPathsReport& frontier_graph_rep, bool ignore_time,
    const StateVec& cur_state,
    std::vector<BranchFrontierCandidate>& candidates) {
  const double kGDistancePenalty = 0.01;
  candidates.clear();
  candidates.reserve(global_frontiers.size());

  for (Vertex* frontier : global_frontiers) {
    if (frontier == NULL) {
      continue;
    }

    BranchFrontierCandidate candidate;
    candidate.frontier = frontier;
    candidate.current_to_frontier_distance =
        global_graph_->getShortestDistance(frontier->id, frontier_graph_rep);
    candidate.frontier_to_home_distance =
        global_graph_->getShortestDistance(frontier->id, global_graph_rep_);

    if (!std::isfinite(candidate.current_to_frontier_distance) ||
        !std::isfinite(candidate.frontier_to_home_distance) ||
        candidate.current_to_frontier_distance < 0.0 ||
        candidate.frontier_to_home_distance < 0.0) {
      continue;
    }

    candidate.time_to_target =
        candidate.current_to_frontier_distance / planning_params_.v_homing_max;
    candidate.time_to_home =
        candidate.frontier_to_home_distance / planning_params_.v_homing_max;
    candidate.time_cost =
        candidate.time_to_target +
        (planning_params_.auto_homing_enable ? candidate.time_to_home : 0.0);

    if (!ignore_time) {
      candidate.time_spare = 0.0;
      if (!isRemainingTimeSufficient(candidate.time_cost,
                                     candidate.time_spare)) {
        continue;
      }
    }

    if (planning_params_.select_closest_frontier) {
      candidate.exp_gain =
          exp(-kGDistancePenalty * candidate.current_to_frontier_distance);
    } else {
      candidate.exp_gain =
          frontier->vol_gain.gain *
          exp(-kGDistancePenalty * candidate.current_to_frontier_distance);
    }
    candidate.exp_gain *=
        getZoneTravelFactor(cur_state.head(3), frontier->state.head(3));
    candidate.exp_gain *= getDeadZonePenalty(frontier->state.head(3));
    if (!ignore_time) {
      candidate.exp_gain *= candidate.time_spare;
    }

    candidates.push_back(candidate);
  }

  return !candidates.empty();
}

int LocalExplorationPlanner::findBranchRegionEntry(
    Vertex* frontier, const ShortestPathsReport& home_graph_rep) const {
  if (frontier == NULL || global_graph_ == NULL || !home_graph_rep.status) {
    return -1;
  }
  if (frontier->id < 0 ||
      frontier->id >= static_cast<int>(home_graph_rep.parent_id_map.size())) {
    return -1;
  }

  int current_id = frontier->id;
  int last_valid_id = current_id;
  std::unordered_set<int> visited;
  while (current_id >= 0 &&
         current_id < static_cast<int>(home_graph_rep.parent_id_map.size()) &&
         visited.insert(current_id).second) {
    Vertex* vertex = global_graph_->getVertex(current_id);
    if (vertex == NULL) {
      break;
    }
    if (getTopologicalMapDegree(vertex) >= 3) {
      return current_id;
    }
    last_valid_id = current_id;
    if (current_id == home_graph_rep.source_id) {
      return current_id;
    }
    const int parent_id = home_graph_rep.parent_id_map.at(current_id);
    if (parent_id == current_id) {
      break;
    }
    current_id = parent_id;
  }
  return last_valid_id;
}

bool LocalExplorationPlanner::selectFrontierWithBranchRegions(
    const std::vector<BranchFrontierCandidate>& candidates,
    const ShortestPathsReport& frontier_graph_rep, Vertex*& best_frontier,
    double& best_gain, double& estimated_region_time) {
  estimated_region_time = 0.0;
  best_frontier = NULL;
  best_gain = -1.0;
  if (!planning_params_.branch_repositioning_selection_enable ||
      candidates.empty()) {
    return false;
  }

  Timer region_timer;
  std::unordered_map<int, BranchRegionCandidate> region_map;
  for (const auto& candidate : candidates) {
    if (candidate.frontier == NULL || candidate.exp_gain <= 0.0) {
      continue;
    }
    const int entry_id =
        findBranchRegionEntry(candidate.frontier, global_graph_rep_);
    if (entry_id < 0) {
      continue;
    }
    BranchRegionCandidate& region = region_map[entry_id];
    region.entry_vertex_id = entry_id;
    region.frontiers.push_back(&candidate);
  }

  if (region_map.empty()) {
    estimated_region_time = region_timer.endTimer();
    return false;
  }

  std::vector<BranchRegionCandidate> regions;
  regions.reserve(region_map.size());
  const double kRegionDistancePenalty = 0.01;
  const double kRegionSupplementGainRatio = 0.25;
  for (auto& entry : region_map) {
    BranchRegionCandidate& region = entry.second;
    double max_gain = 0.0;
    double gain_sum = 0.0;
    for (const BranchFrontierCandidate* frontier_candidate :
         region.frontiers) {
      if (frontier_candidate == NULL) {
        continue;
      }
      const double gain = std::max(0.0, frontier_candidate->exp_gain);
      max_gain = std::max(max_gain, gain);
      gain_sum += gain;
    }
    region.aggregated_gain =
        max_gain +
        kRegionSupplementGainRatio * std::max(0.0, gain_sum - max_gain);
    region.current_to_entry_distance =
        global_graph_->getShortestDistance(region.entry_vertex_id,
                                           frontier_graph_rep);
    region.entry_to_home_distance =
        global_graph_->getShortestDistance(region.entry_vertex_id,
                                           global_graph_rep_);
    if (!std::isfinite(region.current_to_entry_distance) ||
        !std::isfinite(region.entry_to_home_distance)) {
      continue;
    }
    region.score =
        region.aggregated_gain *
        std::exp(-kRegionDistancePenalty * region.current_to_entry_distance) *
        getDeadZonePenalty(
            global_graph_->getVertex(region.entry_vertex_id)->state.head(3));
    regions.push_back(region);
  }

  if (regions.empty()) {
    estimated_region_time = region_timer.endTimer();
    return false;
  }

  std::sort(regions.begin(), regions.end(),
            [](const BranchRegionCandidate& lhs,
               const BranchRegionCandidate& rhs) {
              return lhs.score > rhs.score;
            });

  const int num_selected_regions = std::max(
      1, std::min(static_cast<int>(regions.size()),
                  planning_params_.branch_repositioning_region_count));
  for (int i = 0; i < num_selected_regions; ++i) {
    for (const BranchFrontierCandidate* frontier_candidate :
         regions[i].frontiers) {
      if (frontier_candidate != NULL &&
          frontier_candidate->exp_gain > best_gain) {
        best_gain = frontier_candidate->exp_gain;
        best_frontier = frontier_candidate->frontier;
      }
    }
  }
  estimated_region_time = region_timer.endTimer();
  return best_frontier != NULL;
}

void LocalExplorationPlanner::updateAdaptiveGlobalSelectionTimings(double point_time,
                                               double region_time) {
  const double alpha = std::max(
      0.0, std::min(1.0, planning_params_.branch_repositioning_memory_alpha));
  auto update_ema = [alpha](double sample, double& ema) {
    if (sample < 0.0 || !std::isfinite(sample)) {
      return;
    }
    if (ema < 0.0 || !std::isfinite(ema)) {
      ema = sample;
    } else {
      ema = alpha * sample + (1.0 - alpha) * ema;
    }
  };
  update_ema(point_time, branch_repositioning_point_time_ema_);
  update_ema(region_time, branch_repositioning_region_time_ema_);
}

bool LocalExplorationPlanner::shouldUseBranchRegionSelection(
    double point_time, double region_time, double point_gain,
    double region_gain) const {
  if (!planning_params_.branch_repositioning_selection_enable ||
      region_gain <= 0.0 || point_gain <= 0.0) {
    return false;
  }
  const double point_est =
      branch_repositioning_point_time_ema_ >= 0.0 ? branch_repositioning_point_time_ema_
                                             : point_time;
  const double region_est =
      branch_repositioning_region_time_ema_ >= 0.0 ? branch_repositioning_region_time_ema_
                                              : region_time;
  if (!std::isfinite(point_est) || !std::isfinite(region_est)) {
    return false;
  }
  if (region_gain < point_gain) {
    return false;
  }
  return region_est < point_est;
}

void LocalExplorationPlanner::applyGlobalTargetHysteresis(
    const std::vector<BranchFrontierCandidate>& candidates,
    Vertex*& best_frontier, double& best_gain) const {
  if (!planning_params_.global_frontier_stickiness_enable ||
      !global_exploration_ongoing_ || best_frontier == NULL) {
    return;
  }
  if (current_global_vertex_id_ < 0 ||
      current_global_vertex_id_ >= global_graph_->getNumVertices()) {
    return;
  }

  const BranchFrontierCandidate* current_target = NULL;
  for (const auto& candidate : candidates) {
    if (candidate.frontier != NULL &&
        candidate.frontier->id == current_global_vertex_id_) {
      current_target = &candidate;
      break;
    }
  }

  if (current_target == NULL || current_target->exp_gain <= 0.0 ||
      best_frontier->id == current_target->frontier->id) {
    return;
  }

  if (best_gain <= current_target->exp_gain *
                       planning_params_.global_frontier_stickiness_ratio) {
    best_frontier = current_target->frontier;
    best_gain = current_target->exp_gain;
  }
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::calculateGlobalPath()
{
  last_global_path_failed_no_frontier_ = false;
  ROS_WARN("\033[1;33m[Global Repositioning]\033[0m ===== START GLOBAL PLANNING =====");
  visualization_->visualizeTopologicalGraph(global_graph_);
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();

  // Check if the global planner exists
  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Graph is empty, nothing to search.");
    return ret_path;
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
  if ((global_frontiers.size() <= 0)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "No frontier exists");
    last_global_path_failed_no_frontier_ = true;
    return ret_path;
  }

  // Insert the current state into the topological map when feasible.
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Trying to add new vertex from current position.");
  StateVec cur_state;
  cur_state << current_state_[0], current_state_[1], current_state_[2],
      current_state_[3], current_state_[4];
  cur_state[2] += planning_params_.topological_anchor_z_offset;
  Vertex* link_vertex = NULL;
  const double kRadiusLimit = 1.5;  // 0.5
  bool connected_to_graph =
      anchorStateInCognitiveMap(global_graph_, cur_state, link_vertex, kRadiusLimit);

  if (!connected_to_graph) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot add the state to the topological map.");
    return ret_path;
  }
  if (dead_zone_escape_pending_) {
    markDeadZoneRegion(link_vertex);
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
      "Added current state to the graph. Start searching for the global path "
      "now.");
  // Get Dijsktra path from home to all.
  if (!global_graph_->findShortestPaths(global_graph_rep_)) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "\033[1;34m[Cognitive Map]\033[0m Failed to find shortest path.");
    return ret_path;
  }
  // Get Dijsktra path from current to all.
  ShortestPathsReport frontier_graph_rep;
  if (!global_graph_->findShortestPaths(link_vertex->id, frontier_graph_rep)) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "\033[1;34m[Cognitive Map]\033[0m Failed to find shortest path.");
    return ret_path;
  }
  // Check if the planner should find the best vertex automatically or manually
  double best_gain = -1.0;
  Vertex* best_frontier = NULL;

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                "\033[1;33m[Global Repositioning]\033[0m Auto target selection mode.");
  bool final_branch_region_mode = false;
  int evaluated_frontier_count = 0;
  if (!selectGlobalFrontierTarget(global_frontiers, frontier_graph_rep, false,
                                  cur_state, best_frontier, best_gain,
                                  evaluated_frontier_count,
                                  final_branch_region_mode)) {
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, 
        "No feasible frontier exists --> Call HOMING instead if fully "
        "explored.");
    return ret_path;
  }
  if (final_branch_region_mode) {
    ROS_WARN("\033[1;33m[Global Repositioning]\033[0m FINAL TARGET MODE: branch region "
             "planning, evaluated_frontiers=%d/%d, selected_frontier=%d, "
             "selected_gain=%.3f",
             evaluated_frontier_count, (int)global_frontiers.size(),
             best_frontier != NULL ? best_frontier->id : -1, best_gain);
  } else {
    ROS_WARN("\033[1;33m[Global Repositioning]\033[0m FINAL TARGET MODE: frontier "
             "planning, evaluated_frontiers=%d/%d, selected_frontier=%d, "
             "selected_gain=%.3f",
             evaluated_frontier_count, (int)global_frontiers.size(),
             best_frontier != NULL ? best_frontier->id : -1, best_gain);
  }
  
  std::vector<int> current_to_frontier_path_id;
  std::vector<int> frontier_to_home_path_id;
  if (best_gain >= 0) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Found the best frontier to go is: %d", best_frontier->id);

    // if (auto_global_planner_trig_) {
    //   current_global_vertex_id_ = best_frontier->id;
    //   global_exploration_ongoing_ = true;
    // }

    global_graph_->getShortestPath(best_frontier->id, frontier_graph_rep, true,
                                   current_to_frontier_path_id);
    global_graph_->getShortestPath(best_frontier->id, global_graph_rep_, false,
                                   frontier_to_home_path_id);
    int current_to_frontier_path_id_size = current_to_frontier_path_id.size();
    for (int i = 0; i < current_to_frontier_path_id_size; ++i) {
      StateVec state =
          global_graph_->getVertex(current_to_frontier_path_id[i])->state;
      tf::Quaternion quat;
      // quat.setEuler(0.0, state[4], state[3]);
      Eigen::Matrix3d rot_eigen;
      if(planning_params_.planning_backward)
      {
        double new_yaw = state[3] + M_PI;
        if(new_yaw > M_PI)
          new_yaw -= 2.0 * M_PI;
        if(new_yaw < -M_PI)
          new_yaw += 2.0 * M_PI;
        rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(new_yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());  
      }
      else
      {
        rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
      }
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
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
        "Could not find any positive gain (Should not happen) --> Try "
        "HOMING.");
    return ret_path;
  }

  // Set the heading angle tangent with the moving direction,
  // from the second waypoint; the first waypoint keeps the same direction.
  if (planning_params_.yaw_tangent_correction) {
    for (int i = 0; i < (ret_path.size() - 1); ++i) {
      Eigen::Vector3d vec(ret_path[i + 1].position.x - ret_path[i].position.x,
                          ret_path[i + 1].position.y - ret_path[i].position.y,
                          ret_path[i + 1].position.z - ret_path[i].position.z);
      double yaw;
      if(planning_params_.planning_backward)
      {
        yaw = std::atan2(-vec[1], -vec[0]);
      }
      else {
        yaw = std::atan2(vec[1], vec[0]);
      }
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      ret_path[i + 1].orientation.x = quat.x();
      ret_path[i + 1].orientation.y = quat.y();
      ret_path[i + 1].orientation.z = quat.z();
      ret_path[i + 1].orientation.w = quat.w();
    }
  }

  // Modify path if required
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret_path, mod_path, true)) {
      ret_path = mod_path;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Compute an alternate path for homing in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  visualization_->visualizeGlobalPaths(
      global_graph_, current_to_frontier_path_id, frontier_to_home_path_id);

  double dtime = GET_ELAPSED_TIME(ttime);
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "runGlobalPlanner costs: %f (s)", dtime);

  // Path interpolation:
  const double kInterpolationDistance =
      planning_params_.path_interpolation_distance;
  std::vector<geometry_msgs::Pose> interp_path;
  if (Trajectory::interpolatePath(ret_path, kInterpolationDistance,
                                  interp_path)) {
    ret_path = interp_path;
  }

  accumulateGlobalFallbackPathLength(ret_path);
  publishAblationMetrics();
  visualization_->visualizeRefPath(ret_path);
  return ret_path;
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::reRunGlobalPlanner(int &status) {
  return runGlobalPlanner(current_global_vertex_id_, true, true, status);
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::runGlobalPlanner(int vertex_id,
                                                       bool not_check_frontier,
                                                       bool ignore_time,
                                                       int &status) {
  // @not_check_frontier: just check if it is feasible (collision-free + time)
  // @ignore_time: don't consider time budget.
  const bool is_global_frontier_fallback = (vertex_id == 0 && !not_check_frontier);
  ROS_WARN("\033[1;33m[Global Repositioning]\033[0m ===== START GLOBAL PLANNING ===== "
           "vertex_id=%d not_check_frontier=%d ignore_time=%d",
           vertex_id, static_cast<int>(not_check_frontier),
           static_cast<int>(ignore_time));

  visualization_->visualizeTopologicalGraph(global_graph_);

  // Check whether the topological map still has frontier vertices.
  // Get the list of current frontiers.
  //
  ROS_INFO_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "\033[1;33m[Global Repositioning]\033[0m Triggered.");
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

  START_TIMER(ttime);
  std::vector<geometry_msgs::Pose> ret_path;
  ret_path.clear();
  // Check if the global planner exists
  if (global_graph_->getNumVertices() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;34m[Cognitive Map]\033[0m Graph is empty, nothing to search.");
    return ret_path;
  }
  // Check if the vertex id exists
  if ((vertex_id < 0) || (vertex_id >= global_graph_->getNumVertices())) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
        "\033[1;34m[Cognitive Map]\033[0m Vertex ID doesn't exist, plz consider IDs in the range "
        "[0-%d].",
        global_graph_->getNumVertices() - 1);
    return ret_path;
  }
  // Check if the time endurance is still available.
  if (!ignore_time) {
    if (getTimeRemained() <= 0.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;33m[Global Repositioning]\033[0m RAN OUT OF TIME --> STOP HERE.");
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
  if ((!not_check_frontier) && (global_frontiers.size() <= 0)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "No frontier exists");
    // Keep exploring or go home if no more frontiers
    if (planning_params_.go_home_if_fully_explored) {
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, " --> Calling HOMING instead.");
      ret_path = getHomingPath(world_frame_);
      homing_engaged_ = true;
    } else {
      num_low_gain_iters_ = 0;
    }
    status = planner_msgs::planner_srv::Response::kHoming;
    return ret_path;
  }
  // Insert the current state into the topological map when feasible.
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Trying to add new vertex from current position.");
  StateVec cur_state;
  cur_state << current_state_[0], current_state_[1], current_state_[2],
      current_state_[3], current_state_[4];
  cur_state[2] += planning_params_.topological_anchor_z_offset;
  Vertex* link_vertex = NULL;
  const double kRadiusLimit = 1.5;  // 0.5
  bool connected_to_graph =
      anchorStateInCognitiveMap(global_graph_, cur_state, link_vertex, kRadiusLimit);

  if (!connected_to_graph) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot add the state to the topological map.");
    return ret_path;
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
      "Added current state to the graph. Start searching for the global path "
      "now.");
  // Get Dijsktra path from home to all.
  if (!global_graph_->findShortestPaths(global_graph_rep_)) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "\033[1;34m[Cognitive Map]\033[0m Failed to find shortest path.");
    return ret_path;
  }
  // Get Dijsktra path from current to all.
  ShortestPathsReport frontier_graph_rep;
  if (!global_graph_->findShortestPaths(link_vertex->id, frontier_graph_rep)) {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "\033[1;34m[Cognitive Map]\033[0m Failed to find shortest path.");
    return ret_path;
  }
  // Check if the planner should find the best vertex automatically or manually
  double best_gain = -1.0;
  Vertex* best_frontier = NULL;

  if (vertex_id) {
    // Manual mode
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;33m[Global Repositioning]\033[0m Manual Mode");
    // Just need to check if it is feasible
    std::vector<int> current_to_target_path_id;
    std::vector<int> target_to_home_path_id;
    double current_to_target_distance;
    double target_to_home_distance;
    global_graph_->getShortestPath(vertex_id, frontier_graph_rep, true,
                                   current_to_target_path_id);
    global_graph_->getShortestPath(vertex_id, global_graph_rep_, false,
                                   target_to_home_path_id);
    current_to_target_distance =
        global_graph_->getShortestDistance(vertex_id, frontier_graph_rep);
    target_to_home_distance =
        global_graph_->getShortestDistance(vertex_id, global_graph_rep_);
    // Estimate time
    double time_remaining = getTimeRemained();
    double time_to_target =
        current_to_target_distance / planning_params_.v_homing_max;
    double time_to_home =
        target_to_home_distance / planning_params_.v_homing_max;
    double time_cost = 0;
    if (!ignore_time) {
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "\033[1;33m[Global Repositioning]\033[0m Time remaining: %f (sec)", time_remaining);
      time_cost += time_to_target;
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "\033[1;33m[Global Repositioning]\033[0m Time to [%3d]: %f (sec)", vertex_id, time_to_target);
      if (planning_params_.auto_homing_enable) {
        time_cost += time_to_home;
        ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "\033[1;33m[Global Repositioning]\033[0m Time to home  : %f (sec)", time_to_home);
      }
    }
    double time_spare = 0;
    if (!isRemainingTimeSufficient(time_cost, time_spare)) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;33m[Global Repositioning]\033[0m Not enough time to go the vertex [%d]", vertex_id);
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
          "\033[1;33m[Global Repositioning]\033[0m Consider change to another ID or set ignore_time to True");
      return ret_path;
    }
    best_frontier = global_graph_->getVertex(vertex_id);
    best_gain = 1.0;
  } else {
    // Auto mode
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "\033[1;33m[Global Repositioning]\033[0m Auto mode");
    bool final_branch_region_mode = false;
    int evaluated_frontier_count = 0;
    if (!selectGlobalFrontierTarget(global_frontiers, frontier_graph_rep,
                                    ignore_time, cur_state, best_frontier,
                                    best_gain, evaluated_frontier_count,
                                    final_branch_region_mode)) {
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, 
          "No feasible frontier exists --> Call HOMING instead if fully "
          "explored.");
      return ret_path;
    }
    if (final_branch_region_mode) {
      ROS_WARN("\033[1;33m[Global Repositioning]\033[0m FINAL TARGET MODE: branch region "
               "planning, evaluated_frontiers=%d/%d, selected_frontier=%d, "
               "selected_gain=%.3f",
               evaluated_frontier_count, (int)global_frontiers.size(),
               best_frontier != NULL ? best_frontier->id : -1, best_gain);
    } else {
      ROS_WARN("\033[1;33m[Global Repositioning]\033[0m FINAL TARGET MODE: frontier "
               "planning, evaluated_frontiers=%d/%d, selected_frontier=%d, "
               "selected_gain=%.3f",
               evaluated_frontier_count, (int)global_frontiers.size(),
               best_frontier != NULL ? best_frontier->id : -1, best_gain);
    }
  }

  std::vector<int> current_to_frontier_path_id;
  std::vector<int> frontier_to_home_path_id;
  if (best_gain >= 0) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Found the best frontier to go is: %d", best_frontier->id);

    if (auto_global_planner_trig_) {
      current_global_vertex_id_ = best_frontier->id;
      global_exploration_ongoing_ = true;
    }

    global_graph_->getShortestPath(best_frontier->id, frontier_graph_rep, true,
                                   current_to_frontier_path_id);
    global_graph_->getShortestPath(best_frontier->id, global_graph_rep_, false,
                                   frontier_to_home_path_id);
    int current_to_frontier_path_id_size = current_to_frontier_path_id.size();
    for (int i = 0; i < current_to_frontier_path_id_size; ++i) {
      StateVec state =
          global_graph_->getVertex(current_to_frontier_path_id[i])->state;
      tf::Quaternion quat;
      // quat.setEuler(0.0, state[4], state[3]);
      Eigen::Matrix3d rot_eigen;
      if(planning_params_.planning_backward)
      {
        double new_yaw = state[3] + M_PI;
        if(new_yaw > M_PI)
          new_yaw -= 2.0 * M_PI;
        if(new_yaw < -M_PI)
          new_yaw += 2.0 * M_PI;
        rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(new_yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());  
      }
      else
      {
        rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
      }
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
  } else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, 
        "Could not find any positive gain (Should not happen) --> Try with "
        "HOMING.");
    return ret_path;
  }

  if (dead_zone_escape_pending_) {
    tryPrependDeadZoneEscapeManeuver(ret_path);
    dead_zone_escape_pending_ = false;
  }

  // Set the heading angle tangent with the moving direction,
  // from the second waypoint; the first waypoint keeps the same direction.
  if (planning_params_.yaw_tangent_correction) {
    for (int i = 0; i < (ret_path.size() - 1); ++i) {
      Eigen::Vector3d vec(ret_path[i + 1].position.x - ret_path[i].position.x,
                          ret_path[i + 1].position.y - ret_path[i].position.y,
                          ret_path[i + 1].position.z - ret_path[i].position.z);
      double yaw;
      if(planning_params_.planning_backward)
      {
        yaw = std::atan2(-vec[1], -vec[0]);
      }
      else {
        yaw = std::atan2(vec[1], vec[0]);
      }
      tf::Quaternion quat;
      quat.setEuler(0.0, 0.0, yaw);
      ret_path[i + 1].orientation.x = quat.x();
      ret_path[i + 1].orientation.y = quat.y();
      ret_path[i + 1].orientation.z = quat.z();
      ret_path[i + 1].orientation.w = quat.w();
    }
  }

  // Modify path if required
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret_path, mod_path, true)) {
      ret_path = mod_path;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Compute an alternate path for homing in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  visualization_->visualizeGlobalPaths(
      global_graph_, current_to_frontier_path_id, frontier_to_home_path_id);

  double dtime = GET_ELAPSED_TIME(ttime);
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "runGlobalPlanner costs: %f (s)", dtime);

  // Path interpolation:
  const double kInterpolationDistance =
      planning_params_.path_interpolation_distance;
  std::vector<geometry_msgs::Pose> interp_path;
  if (Trajectory::interpolatePath(ret_path, kInterpolationDistance,
                                  interp_path)) {
    ret_path = interp_path;
  }

  accumulateGlobalFallbackPathLength(ret_path);
  if (is_global_frontier_fallback) {
    accumulateGlobalFrontierFallbackMetrics(ret_path);
  }
  publishAblationMetrics();
  visualization_->visualizeRefPath(ret_path);
  return ret_path;
}
