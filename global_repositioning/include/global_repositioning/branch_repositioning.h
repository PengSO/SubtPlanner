#pragma once

#include "cognitive_graph_model/cognitive_graph.h"
#include "cognitive_graph_model/graph_types.h"

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>

#include <functional>
#include <memory>
#include <vector>

class VoxelMap;

struct BranchFrontierCandidate {
  Vertex* frontier = NULL;
  double current_to_frontier_distance = 0.0;
  double frontier_to_home_distance = 0.0;
  double time_to_target = 0.0;
  double time_to_home = 0.0;
  double time_cost = 0.0;
  double time_spare = 1.0;
  double exp_gain = 0.0;
};

struct BranchRegionCandidate {
  int entry_vertex_id = -1;
  std::vector<const BranchFrontierCandidate*> frontiers;
  double aggregated_gain = 0.0;
  double current_to_entry_distance = 0.0;
  double entry_to_home_distance = 0.0;
  double score = 0.0;
};

enum class BranchTargetSelectionMode {
  kPoint,
  kRegion,
};

struct DeadZoneRecoveryState {
  bool escape_pending = false;
  int latest_vertex_id = -1;
  std::vector<int> vertex_ids;
};

struct BranchRepositioningAdaptiveState {
  double point_time_ema = -1.0;
  double region_time_ema = -1.0;
  double frontier_count_ema = -1.0;
  double frontier_count_sq_ema = -1.0;
  double success_ema = 0.5;
  bool cache_ready = false;
};

struct GlobalRepositioningState {
  DeadZoneRecoveryState dead_zone;
  BranchRepositioningAdaptiveState branch;
};

namespace global_repositioning {

struct BranchSelectionWeights {
  double frontier_scale = 1.0;
  double compactness = 1.0;
  double success = 1.0;
  double time = 1.0;
};

struct BranchSelectionScore {
  double scale_pressure = 0.5;
  double compactness = 0.0;
  double reliability = 0.5;
  double time_advantage = 0.0;
  double point_score = 0.0;
  double region_score = 0.0;
  BranchTargetSelectionMode mode = BranchTargetSelectionMode::kPoint;
};

struct FrontierEvaluationOptions {
  bool ignore_time = false;
  bool auto_homing_enable = true;
  bool select_closest_frontier = false;
  bool constrain_to_nearest_branch_entry = true;
  double v_homing_max = 1.0;
  double branch_entry_distance_tolerance = 0.0;
  int region_count = 1;
};

using RemainingTimeEvaluator = std::function<bool(double, double&)>;
using ZoneTravelFactorEvaluator =
    std::function<double(const Eigen::Vector3d&, const Eigen::Vector3d&)>;
using PositionPenaltyEvaluator = std::function<double(const Eigen::Vector3d&)>;

void updateFrontierCountStats(int frontier_count, double memory_alpha,
                              BranchRepositioningAdaptiveState& state);

void updateBranchRegionSuccess(bool success, double memory_alpha,
                               BranchRepositioningAdaptiveState& state);

void updateSelectionTimings(double point_time, double region_time,
                            double memory_alpha,
                            BranchRepositioningAdaptiveState& state);

BranchSelectionScore scoreBranchTargetSelection(
    int frontier_count, double compactness,
    const BranchRepositioningAdaptiveState& state,
    const BranchSelectionWeights& weights);

int getTopologicalMapDegree(const std::shared_ptr<CognitiveGraph>& graph,
                            const Vertex* vertex);

int findBranchRegionEntry(const std::shared_ptr<CognitiveGraph>& graph,
                          Vertex* frontier,
                          const ShortestPathsReport& home_graph_rep);

double estimateBranchRegionCompactness(
    const std::shared_ptr<CognitiveGraph>& graph,
    const std::vector<Vertex*>& global_frontiers,
    const ShortestPathsReport& home_graph_rep);

double scoreFrontierCandidate(double frontier_gain,
                              double current_to_frontier_distance,
                              double zone_factor,
                              double dead_zone_factor,
                              double time_spare,
                              bool select_closest_frontier,
                              bool ignore_time);

double aggregateRegionGain(
    const std::vector<const BranchFrontierCandidate*>& frontiers);

double scoreRegionCandidate(double aggregated_gain,
                            double current_to_entry_distance,
                            double dead_zone_factor);

bool buildScoredBranchRegions(
    const std::vector<BranchFrontierCandidate>& candidates,
    const std::shared_ptr<CognitiveGraph>& graph,
    const ShortestPathsReport& frontier_graph_rep,
    const ShortestPathsReport& home_graph_rep,
    bool require_home_distance,
    const std::function<double(const Eigen::Vector3d&)>& dead_zone_factor,
    std::vector<BranchRegionCandidate>& regions);

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
    std::vector<BranchFrontierCandidate>& candidates);

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
    std::vector<Vertex*>& selected_frontiers);

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
    double& best_gain);

void applyTargetHysteresis(
    const std::vector<BranchFrontierCandidate>& candidates,
    bool enabled,
    bool global_exploration_ongoing,
    int current_global_vertex_id,
    double stickiness_ratio,
    const std::shared_ptr<CognitiveGraph>& graph,
    Vertex*& best_frontier,
    double& best_gain);

bool prependDeadZoneEscapeManeuver(
    const StateVec& current_state,
    const Eigen::Vector3d& robot_center_offset,
    const Eigen::Vector3d& robot_box_size,
    VoxelMap* voxel_map,
    double turn_radius,
    double turn_step,
    double turn_clearance,
    std::vector<geometry_msgs::Pose>& path);

}  // namespace global_repositioning
