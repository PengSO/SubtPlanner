#include "global_repositioning/branch_repositioning.h"

#include "voxel_map/voxel_map.h"

#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace global_repositioning {

namespace {

constexpr double kFrontierDistancePenalty = 0.01;
constexpr double kRegionDistancePenalty = 0.01;
constexpr double kRegionSupplementGainRatio = 0.25;

double clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

void updateEma(double sample, double alpha, double& ema) {
  if (sample < 0.0 || !std::isfinite(sample)) {
    return;
  }
  if (ema < 0.0 || !std::isfinite(ema)) {
    ema = sample;
  } else {
    ema = alpha * sample + (1.0 - alpha) * ema;
  }
}

}  // namespace

void updateFrontierCountStats(int frontier_count, double memory_alpha,
                              BranchRepositioningAdaptiveState& state) {
  const double alpha = clamp01(memory_alpha);
  const double count = std::max(0, frontier_count);
  const double count_sq = count * count;
  if (state.frontier_count_ema < 0.0 ||
      !std::isfinite(state.frontier_count_ema)) {
    state.frontier_count_ema = count;
    state.frontier_count_sq_ema = count_sq;
    return;
  }
  state.frontier_count_ema =
      alpha * count + (1.0 - alpha) * state.frontier_count_ema;
  state.frontier_count_sq_ema =
      alpha * count_sq + (1.0 - alpha) * state.frontier_count_sq_ema;
}

void updateBranchRegionSuccess(bool success, double memory_alpha,
                               BranchRepositioningAdaptiveState& state) {
  const double alpha = clamp01(memory_alpha);
  const double sample = success ? 1.0 : 0.0;
  state.success_ema = alpha * sample + (1.0 - alpha) * state.success_ema;
}

void updateSelectionTimings(double point_time, double region_time,
                            double memory_alpha,
                            BranchRepositioningAdaptiveState& state) {
  const double alpha = clamp01(memory_alpha);
  updateEma(point_time, alpha, state.point_time_ema);
  updateEma(region_time, alpha, state.region_time_ema);
}

BranchSelectionScore scoreBranchTargetSelection(
    int frontier_count, double compactness,
    const BranchRepositioningAdaptiveState& state,
    const BranchSelectionWeights& weights) {
  BranchSelectionScore score;
  score.compactness = clamp01(compactness);

  const double count = static_cast<double>(std::max(0, frontier_count));
  if (state.frontier_count_ema >= 0.0 && state.frontier_count_sq_ema >= 0.0) {
    const double variance =
        std::max(1.0, state.frontier_count_sq_ema -
                          state.frontier_count_ema * state.frontier_count_ema);
    const double z_score =
        (count - state.frontier_count_ema) / std::sqrt(variance);
    score.scale_pressure = 1.0 / (1.0 + std::exp(-z_score));
  }

  if (state.point_time_ema >= 0.0 && state.region_time_ema >= 0.0 &&
      std::isfinite(state.point_time_ema) &&
      std::isfinite(state.region_time_ema)) {
    const double normalizer =
        std::max(1e-3, std::max(state.point_time_ema, state.region_time_ema));
    score.time_advantage =
        std::tanh((state.point_time_ema - state.region_time_ema) / normalizer);
  }

  score.reliability = clamp01(state.success_ema);
  score.region_score =
      weights.frontier_scale * score.scale_pressure +
      weights.compactness * score.compactness +
      weights.success * (score.reliability - 0.5) +
      weights.time * score.time_advantage;
  score.point_score =
      weights.frontier_scale * (1.0 - score.scale_pressure) +
      0.5 * weights.compactness * (1.0 - score.compactness) -
      weights.time * score.time_advantage;
  score.mode = score.region_score > score.point_score
                   ? BranchTargetSelectionMode::kRegion
                   : BranchTargetSelectionMode::kPoint;
  return score;
}

int getTopologicalMapDegree(const std::shared_ptr<CognitiveGraph>& graph,
                            const Vertex* vertex) {
  if (vertex == NULL || graph == NULL) {
    return 0;
  }
  const auto edge_it = graph->edge_map_.find(vertex->id);
  if (edge_it == graph->edge_map_.end()) {
    return 0;
  }
  return static_cast<int>(edge_it->second.size());
}

int findBranchRegionEntry(const std::shared_ptr<CognitiveGraph>& graph,
                          Vertex* frontier,
                          const ShortestPathsReport& home_graph_rep) {
  if (frontier == NULL || graph == NULL || !home_graph_rep.status) {
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
    Vertex* vertex = graph->getVertex(current_id);
    if (vertex == NULL) {
      break;
    }
    if (getTopologicalMapDegree(graph, vertex) >= 3) {
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

double estimateBranchRegionCompactness(
    const std::shared_ptr<CognitiveGraph>& graph,
    const std::vector<Vertex*>& global_frontiers,
    const ShortestPathsReport& home_graph_rep) {
  if (global_frontiers.empty()) {
    return 0.0;
  }

  std::unordered_set<int> region_entries;
  int valid_frontier_count = 0;
  for (Vertex* frontier : global_frontiers) {
    if (frontier == NULL || frontier->type != VertexType::kFrontier) {
      continue;
    }
    const int entry_id = findBranchRegionEntry(graph, frontier, home_graph_rep);
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

double scoreFrontierCandidate(double frontier_gain,
                              double current_to_frontier_distance,
                              double zone_factor,
                              double dead_zone_factor,
                              double time_spare,
                              bool select_closest_frontier,
                              bool ignore_time) {
  const double base_gain =
      select_closest_frontier ? 1.0 : std::max(0.0, frontier_gain);
  double score = base_gain *
                 std::exp(-kFrontierDistancePenalty *
                          current_to_frontier_distance) *
                 zone_factor * dead_zone_factor;
  if (!ignore_time) {
    score *= time_spare;
  }
  return score;
}

double aggregateRegionGain(
    const std::vector<const BranchFrontierCandidate*>& frontiers) {
  double max_gain = 0.0;
  double gain_sum = 0.0;
  for (const BranchFrontierCandidate* frontier_candidate : frontiers) {
    if (frontier_candidate == NULL) {
      continue;
    }
    const double gain = std::max(0.0, frontier_candidate->exp_gain);
    max_gain = std::max(max_gain, gain);
    gain_sum += gain;
  }
  return max_gain +
         kRegionSupplementGainRatio * std::max(0.0, gain_sum - max_gain);
}

double scoreRegionCandidate(double aggregated_gain,
                            double current_to_entry_distance,
                            double dead_zone_factor) {
  return aggregated_gain *
         std::exp(-kRegionDistancePenalty * current_to_entry_distance) *
         dead_zone_factor;
}

bool buildScoredBranchRegions(
    const std::vector<BranchFrontierCandidate>& candidates,
    const std::shared_ptr<CognitiveGraph>& graph,
    const ShortestPathsReport& frontier_graph_rep,
    const ShortestPathsReport& home_graph_rep,
    bool require_home_distance,
    const std::function<double(const Eigen::Vector3d&)>& dead_zone_factor,
    std::vector<BranchRegionCandidate>& regions) {
  regions.clear();
  if (graph == NULL || candidates.empty()) {
    return false;
  }

  std::unordered_map<int, BranchRegionCandidate> region_map;
  for (const auto& candidate : candidates) {
    if (candidate.frontier == NULL || candidate.exp_gain <= 0.0) {
      continue;
    }
    const int entry_id =
        findBranchRegionEntry(graph, candidate.frontier, home_graph_rep);
    if (entry_id < 0) {
      continue;
    }
    BranchRegionCandidate& region = region_map[entry_id];
    region.entry_vertex_id = entry_id;
    region.frontiers.push_back(&candidate);
  }

  if (region_map.empty()) {
    return false;
  }

  regions.reserve(region_map.size());
  for (auto& entry : region_map) {
    BranchRegionCandidate& region = entry.second;
    region.aggregated_gain = aggregateRegionGain(region.frontiers);
    region.current_to_entry_distance =
        graph->getShortestDistance(region.entry_vertex_id, frontier_graph_rep);
    if (require_home_distance) {
      region.entry_to_home_distance =
          graph->getShortestDistance(region.entry_vertex_id, home_graph_rep);
    }

    Vertex* entry_vertex = graph->getVertex(region.entry_vertex_id);
    if (entry_vertex == NULL ||
        !std::isfinite(region.current_to_entry_distance) ||
        (require_home_distance &&
         !std::isfinite(region.entry_to_home_distance))) {
      continue;
    }

    region.score =
        scoreRegionCandidate(region.aggregated_gain,
                             region.current_to_entry_distance,
                             dead_zone_factor(entry_vertex->state.head(3)));
    regions.push_back(region);
  }

  std::sort(regions.begin(), regions.end(),
            [](const BranchRegionCandidate& lhs,
               const BranchRegionCandidate& rhs) {
              return lhs.score > rhs.score;
            });
  return !regions.empty();
}

void applyNearestBranchEntryConstraint(
    bool enabled,
    double tolerance,
    std::vector<BranchRegionCandidate>& regions) {
  if (!enabled || regions.empty()) {
    return;
  }

  double nearest_entry_distance = std::numeric_limits<double>::infinity();
  for (const BranchRegionCandidate& region : regions) {
    if (std::isfinite(region.current_to_entry_distance)) {
      nearest_entry_distance =
          std::min(nearest_entry_distance, region.current_to_entry_distance);
    }
  }
  if (!std::isfinite(nearest_entry_distance)) {
    return;
  }

  const double max_allowed_distance =
      nearest_entry_distance + std::max(0.0, tolerance);
  regions.erase(
      std::remove_if(regions.begin(), regions.end(),
                     [max_allowed_distance](const BranchRegionCandidate& region) {
                       return !std::isfinite(region.current_to_entry_distance) ||
                              region.current_to_entry_distance >
                                  max_allowed_distance;
                     }),
      regions.end());
}

bool collectFrontierCandidates(
    const std::vector<Vertex*>& global_frontiers,
    const std::shared_ptr<CognitiveGraph>& graph,
    const ShortestPathsReport& frontier_graph_rep,
    const ShortestPathsReport& home_graph_rep,
    const StateVec& cur_state,
    const FrontierEvaluationOptions& options,
    const RemainingTimeEvaluator& is_remaining_time_sufficient,
    const ZoneTravelFactorEvaluator& zone_travel_factor,
    const PositionPenaltyEvaluator& dead_zone_penalty,
    std::vector<BranchFrontierCandidate>& candidates) {
  candidates.clear();
  candidates.reserve(global_frontiers.size());
  if (graph == NULL) {
    return false;
  }

  for (Vertex* frontier : global_frontiers) {
    if (frontier == NULL) {
      continue;
    }

    BranchFrontierCandidate candidate;
    candidate.frontier = frontier;
    candidate.current_to_frontier_distance =
        graph->getShortestDistance(frontier->id, frontier_graph_rep);
    candidate.frontier_to_home_distance =
        graph->getShortestDistance(frontier->id, home_graph_rep);

    if (!std::isfinite(candidate.current_to_frontier_distance) ||
        !std::isfinite(candidate.frontier_to_home_distance) ||
        candidate.current_to_frontier_distance < 0.0 ||
        candidate.frontier_to_home_distance < 0.0) {
      continue;
    }

    candidate.time_to_target =
        candidate.current_to_frontier_distance / options.v_homing_max;
    candidate.time_to_home =
        candidate.frontier_to_home_distance / options.v_homing_max;
    candidate.time_cost =
        candidate.time_to_target +
        (options.auto_homing_enable ? candidate.time_to_home : 0.0);

    if (!options.ignore_time) {
      candidate.time_spare = 0.0;
      if (!is_remaining_time_sufficient(candidate.time_cost,
                                        candidate.time_spare)) {
        continue;
      }
    }

    candidate.exp_gain =
        scoreFrontierCandidate(
            frontier->vol_gain.gain, candidate.current_to_frontier_distance,
            zone_travel_factor(cur_state.head(3), frontier->state.head(3)),
            dead_zone_penalty(frontier->state.head(3)),
            candidate.time_spare, options.select_closest_frontier,
            options.ignore_time);

    candidates.push_back(candidate);
  }

  return !candidates.empty();
}

bool preselectFrontiersByBranchRegion(
    const std::vector<Vertex*>& global_frontiers,
    const std::shared_ptr<CognitiveGraph>& graph,
    const ShortestPathsReport& frontier_graph_rep,
    const ShortestPathsReport& home_graph_rep,
    const StateVec& cur_state,
    const FrontierEvaluationOptions& options,
    const RemainingTimeEvaluator& is_remaining_time_sufficient,
    const ZoneTravelFactorEvaluator& zone_travel_factor,
    const PositionPenaltyEvaluator& dead_zone_penalty,
    std::vector<Vertex*>& selected_frontiers) {
  selected_frontiers.clear();
  if (global_frontiers.empty()) {
    return false;
  }

  std::vector<BranchFrontierCandidate> cached_candidates;
  if (!collectFrontierCandidates(global_frontiers, graph, frontier_graph_rep,
                                 home_graph_rep, cur_state, options,
                                 is_remaining_time_sufficient,
                                 zone_travel_factor, dead_zone_penalty,
                                 cached_candidates)) {
    return false;
  }

  cached_candidates.erase(
      std::remove_if(cached_candidates.begin(), cached_candidates.end(),
                     [](const BranchFrontierCandidate& candidate) {
                       return candidate.frontier == NULL ||
                              candidate.frontier->type !=
                                  VertexType::kFrontier ||
                              candidate.exp_gain <= 0.0;
                     }),
      cached_candidates.end());

  std::vector<BranchRegionCandidate> regions;
  if (!buildScoredBranchRegions(cached_candidates, graph, frontier_graph_rep,
                                home_graph_rep, false, dead_zone_penalty,
                                regions)) {
    return false;
  }
  applyNearestBranchEntryConstraint(
      options.constrain_to_nearest_branch_entry,
      options.branch_entry_distance_tolerance, regions);
  if (regions.empty()) {
    return false;
  }

  const int num_selected_regions = std::max(
      1, std::min(static_cast<int>(regions.size()), options.region_count));
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

  return !selected_frontiers.empty();
}

bool selectFrontierByBranchRegion(
    const std::vector<BranchFrontierCandidate>& candidates,
    const std::shared_ptr<CognitiveGraph>& graph,
    const ShortestPathsReport& frontier_graph_rep,
    const ShortestPathsReport& home_graph_rep,
    int region_count,
    bool constrain_to_nearest_branch_entry,
    double branch_entry_distance_tolerance,
    const PositionPenaltyEvaluator& dead_zone_penalty,
    Vertex*& best_frontier,
    double& best_gain) {
  best_frontier = NULL;
  best_gain = -1.0;
  if (candidates.empty()) {
    return false;
  }

  std::vector<BranchRegionCandidate> regions;
  if (!buildScoredBranchRegions(candidates, graph, frontier_graph_rep,
                                home_graph_rep, true, dead_zone_penalty,
                                regions)) {
    return false;
  }
  applyNearestBranchEntryConstraint(constrain_to_nearest_branch_entry,
                                    branch_entry_distance_tolerance, regions);
  if (regions.empty()) {
    return false;
  }

  const int num_selected_regions =
      std::max(1, std::min(static_cast<int>(regions.size()), region_count));
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
  return best_frontier != NULL;
}

void applyTargetHysteresis(
    const std::vector<BranchFrontierCandidate>& candidates,
    bool enabled,
    bool global_exploration_ongoing,
    int current_global_vertex_id,
    double stickiness_ratio,
    const std::shared_ptr<CognitiveGraph>& graph,
    Vertex*& best_frontier,
    double& best_gain) {
  if (!enabled || !global_exploration_ongoing || best_frontier == NULL ||
      graph == NULL) {
    return;
  }
  if (current_global_vertex_id < 0 ||
      current_global_vertex_id >= graph->getNumVertices()) {
    return;
  }

  const BranchFrontierCandidate* current_target = NULL;
  for (const auto& candidate : candidates) {
    if (candidate.frontier != NULL &&
        candidate.frontier->id == current_global_vertex_id) {
      current_target = &candidate;
      break;
    }
  }

  if (current_target == NULL || current_target->exp_gain <= 0.0 ||
      best_frontier->id == current_target->frontier->id) {
    return;
  }

  if (best_gain <= current_target->exp_gain * stickiness_ratio) {
    best_frontier = current_target->frontier;
    best_gain = current_target->exp_gain;
  }
}

bool buildUShapedEscapeManeuver(
    const StateVec& current_state,
    const Eigen::Vector3d& robot_center_offset,
    const Eigen::Vector3d& robot_box_size,
    VoxelMap* voxel_map,
    double turn_radius,
    double turn_step,
    double turn_clearance,
    double lateral_sign,
    const std::vector<geometry_msgs::Pose>& follow_path,
    std::vector<geometry_msgs::Pose>& escape_path) {
  escape_path.clear();
  if (voxel_map == NULL || follow_path.size() < 2) {
    return false;
  }

  const double radius = turn_radius;
  const double step = std::max(0.1, turn_step);
  const double safe_clearance = turn_clearance;
  const Eigen::Vector3d origin = current_state.head(3);
  const Eigen::Vector3d forward(std::cos(current_state[3]),
                                std::sin(current_state[3]), 0.0);
  const Eigen::Vector3d left(-forward.y(), forward.x(), 0.0);
  const Eigen::Vector3d lateral = lateral_sign * left;
  const Eigen::Vector3d center = origin + lateral * radius;

  std::vector<Eigen::Vector3d> points;
  const int samples =
      std::max(6, static_cast<int>(std::ceil(M_PI * radius / step)));
  points.reserve(samples + 1);
  for (int i = 0; i <= samples; ++i) {
    const double theta = M_PI * static_cast<double>(i) / samples;
    points.push_back(center + (-lateral * std::cos(theta) +
                               forward * std::sin(theta)) *
                                  radius);
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3d check_point = points[i] + robot_center_offset;
    const double clearance = voxel_map->getPointDistance(check_point);
    if (clearance < safe_clearance) {
      return false;
    }
    if (i > 0 &&
        voxel_map->getPathStatus(points[i - 1], points[i],
                                 robot_box_size, true) != VoxelStatus::kFree) {
      return false;
    }
  }

  const Eigen::Vector3d follow_next(follow_path[1].position.x,
                                    follow_path[1].position.y,
                                    follow_path[1].position.z);
  if (voxel_map->getPathStatus(points.back(), follow_next, robot_box_size,
                               true) != VoxelStatus::kFree) {
    return false;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const double theta = M_PI * static_cast<double>(i) / samples;
    const double yaw = current_state[3] + theta;
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

bool prependDeadZoneEscapeManeuver(
    const StateVec& current_state,
    const Eigen::Vector3d& robot_center_offset,
    const Eigen::Vector3d& robot_box_size,
    VoxelMap* voxel_map,
    double turn_radius,
    double turn_step,
    double turn_clearance,
    std::vector<geometry_msgs::Pose>& path) {
  if (path.size() < 2) {
    return false;
  }

  std::vector<geometry_msgs::Pose> escape_path;
  if (!buildUShapedEscapeManeuver(current_state, robot_center_offset,
                                  robot_box_size, voxel_map, turn_radius,
                                  turn_step, turn_clearance, 1.0, path,
                                  escape_path) &&
      !buildUShapedEscapeManeuver(current_state, robot_center_offset,
                                  robot_box_size, voxel_map, turn_radius,
                                  turn_step, turn_clearance, -1.0, path,
                                  escape_path)) {
    return false;
  }

  std::vector<geometry_msgs::Pose> merged_path;
  merged_path.reserve(escape_path.size() + path.size());
  merged_path.insert(merged_path.end(), escape_path.begin(), escape_path.end());
  merged_path.insert(merged_path.end(), path.begin() + 1, path.end());
  path.swap(merged_path);
  return true;
}

}  // namespace global_repositioning
