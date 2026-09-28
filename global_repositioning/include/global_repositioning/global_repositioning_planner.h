#pragma once

#include "cognitive_graph_model/cognitive_graph.h"
#include "global_repositioning/branch_repositioning.h"
#include "planner_common/params.h"

#include <geometry_msgs/Pose.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

class VoxelMap;

namespace global_repositioning {

struct GlobalRepositioningCallbacks {
  std::function<void(const std::shared_ptr<CognitiveGraph>& graph)>
      visualize_topological_graph;
  std::function<void(const std::vector<geometry_msgs::Pose>& path)>
      visualize_modified_path;
  std::function<void(const std::shared_ptr<CognitiveGraph>& graph,
                     const std::vector<int>& current_to_frontier_path,
                     const std::vector<int>& frontier_to_home_path)>
      visualize_global_paths;
  std::function<void(const std::vector<geometry_msgs::Pose>& path)>
      visualize_reference_path;
  std::function<void(const std::shared_ptr<CognitiveGraph>& graph,
                     int entry_vertex_id)>
      visualize_branch_entry;

  std::function<void(StateVec& state, VolumetricGain& gain)>
      compute_frontier_gain;
  std::function<bool(double time_cost, double& time_spare)>
      is_remaining_time_sufficient;
  std::function<double()> get_time_remaining;
  std::function<double(const Eigen::Vector3d& source,
                       const Eigen::Vector3d& target)>
      get_zone_travel_factor;
  std::function<bool(std::shared_ptr<CognitiveGraph> graph, StateVec& state,
                     Vertex*& added_vertex, double radius_limit)>
      anchor_state_in_cognitive_map;
  std::function<std::vector<geometry_msgs::Pose>(const std::string& frame)>
      get_homing_path;
  std::function<std::vector<geometry_msgs::Pose>()> get_opening_traversal_path;
  std::function<bool(const std::vector<geometry_msgs::Pose>& path,
                     std::vector<geometry_msgs::Pose>& modified_path,
                     bool allow_shortcut)>
      improve_free_path;
  std::function<void(const std::vector<geometry_msgs::Pose>& path)>
      accumulate_global_fallback_path_length;
  std::function<void(const std::vector<geometry_msgs::Pose>& path)>
      accumulate_global_frontier_fallback_metrics;
  std::function<void()> publish_ablation_metrics;
};

struct GlobalRepositioningContext {
  PlanningParams* planning_params = nullptr;
  RobotParams* robot_params = nullptr;
  VoxelMap* voxel_map = nullptr;
  std::shared_ptr<CognitiveGraph>* global_graph = nullptr;
  ShortestPathsReport* global_graph_report = nullptr;
  StateVec* current_state = nullptr;
  Eigen::Vector3d* robot_box_size = nullptr;
  GlobalRepositioningState* state = nullptr;

  int* num_low_gain_iters = nullptr;
  bool* auto_global_planner_triggered = nullptr;
  bool* global_exploration_ongoing = nullptr;
  int* current_global_vertex_id = nullptr;
  bool* homing_engaged = nullptr;
  std::string* world_frame = nullptr;

  GlobalRepositioningCallbacks callbacks;
};

class GlobalRepositioningPlanner {
 public:
  explicit GlobalRepositioningPlanner(GlobalRepositioningContext context);

  std::vector<geometry_msgs::Pose> plan(int vertex_id, bool not_check_frontier,
                                        bool ignore_time, int& status);
  std::vector<geometry_msgs::Pose> replanCurrent(int& status);

  bool isDeadZoneCandidate(bool frontier_exists, double best_gain,
                           int num_leaf_vertices) const;

 private:
  void markDeadZoneRegion(Vertex* center_vertex);
  double getDeadZoneDistance(const Eigen::Vector3d& position) const;
  double getDeadZonePenalty(const Eigen::Vector3d& position) const;
  bool tryPrependDeadZoneEscapeManeuver(
      std::vector<geometry_msgs::Pose>& path) const;
  bool refreshGlobalFrontiers(const std::vector<Vertex*>& frontiers_to_check,
                              std::vector<Vertex*>& verified_frontiers);
  bool evaluatePointGlobalFrontiers(
      const std::vector<Vertex*>& frontiers_to_evaluate,
      const ShortestPathsReport& frontier_graph_rep, bool ignore_time,
      const StateVec& cur_state, Vertex*& best_frontier, double& best_gain,
      int& evaluated_frontier_count, double& point_total_time);
  void updateAdaptiveGlobalFrontierCountStats(int frontier_count);
  void updateAdaptiveBranchRegionSuccess(bool success);
  BranchTargetSelectionMode chooseBranchTargetSelectionMode(
      const std::vector<Vertex*>& global_frontiers,
      const ShortestPathsReport& frontier_graph_rep) const;
  bool selectGlobalFrontierTarget(
      const std::vector<Vertex*>& global_frontiers,
      const ShortestPathsReport& frontier_graph_rep, bool ignore_time,
      const StateVec& cur_state, Vertex*& best_frontier, double& best_gain,
      int& evaluated_frontier_count, bool& final_branch_region_mode);
  bool validateManualGlobalTarget(int vertex_id,
                                  const ShortestPathsReport& frontier_graph_rep,
                                  bool ignore_time, Vertex*& best_frontier,
                                  double& best_gain);
  void updateAdaptiveGlobalSelectionTimings(double point_time,
                                            double region_time);

  void visualizeTopologicalGraph(const std::shared_ptr<CognitiveGraph>& graph);
  void visualizeModPath(const std::vector<geometry_msgs::Pose>& path);
  void visualizeGlobalPaths(const std::shared_ptr<CognitiveGraph>& graph,
                            const std::vector<int>& current_to_frontier_path,
                            const std::vector<int>& frontier_to_home_path);
  void visualizeRefPath(const std::vector<geometry_msgs::Pose>& path);
  void visualizeBranchEntry(int entry_vertex_id);
  void computeVolumetricGainRayModelNoBound(StateVec& state,
                                            VolumetricGain& gain);
  bool isRemainingTimeSufficient(double time_cost, double& time_spare);
  double getTimeRemained() const;
  double getZoneTravelFactor(const Eigen::Vector3d& source,
                             const Eigen::Vector3d& target) const;
  bool anchorStateInCognitiveMap(std::shared_ptr<CognitiveGraph> graph,
                                 StateVec& state, Vertex*& added_vertex,
                                 double radius_limit);
  std::vector<geometry_msgs::Pose> getHomingPath(const std::string& frame);
  std::vector<geometry_msgs::Pose> getOpeningTraversalPath();
  bool improveFreePath(const std::vector<geometry_msgs::Pose>& path,
                       std::vector<geometry_msgs::Pose>& modified_path,
                       bool allow_shortcut);
  void accumulateGlobalFallbackPathLength(
      const std::vector<geometry_msgs::Pose>& path);
  void accumulateGlobalFrontierFallbackMetrics(
      const std::vector<geometry_msgs::Pose>& path);
  void publishAblationMetrics();

  GlobalRepositioningContext context_;
  PlanningParams& planning_params_;
  RobotParams& robot_params_;
  VoxelMap* voxel_map_;
  std::shared_ptr<CognitiveGraph>& global_graph_;
  ShortestPathsReport& global_graph_rep_;
  StateVec& current_state_;
  Eigen::Vector3d& robot_box_size_;
  GlobalRepositioningState& global_repositioning_state_;
  int& num_low_gain_iters_;
  bool& auto_global_planner_trig_;
  bool& global_exploration_ongoing_;
  int& current_global_vertex_id_;
  bool& homing_engaged_;
  std::string& world_frame_;
};

}  // namespace global_repositioning
