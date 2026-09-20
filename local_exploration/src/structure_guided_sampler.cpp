#include "local_exploration/local_exploration_planner.h"

#include <algorithm>
#include <random>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

double LocalExplorationPlanner::getClearanceScore(const Eigen::Vector3d& position) const {
  double clearance =
      voxel_map_->getPointDistance(position + robot_params_.center_offset);
  if (clearance < 0.0) {
    return 0.0;
  }
  const double norm =
      std::max(robot_box_size_.norm(), voxel_map_->getResolution());
  return std::min(clearance / norm, 2.0);
}

Eigen::Vector3d LocalExplorationPlanner::getPrimaryExplorationAxis(
    const Eigen::Vector3d& reference_pos) const {
  Eigen::Vector3d axis(std::cos(exploring_direction_),
                       std::sin(exploring_direction_), 0.0);

  if (global_exploration_ongoing_ && global_graph_ != NULL &&
      current_global_vertex_id_ >= 0 &&
      current_global_vertex_id_ < global_graph_->getNumVertices()) {
    Eigen::Vector3d target =
        global_graph_->getVertex(current_global_vertex_id_)->state.head(3);
    if ((target - reference_pos).norm() > kSmallNorm) {
      axis = target - reference_pos;
    }
  } else if (next_compartment_.x() < std::numeric_limits<double>::max() / 2.0 &&
             (next_compartment_ - reference_pos).norm() > kSmallNorm) {
    axis = next_compartment_ - reference_pos;
  }

  if (axis.norm() <= kSmallNorm) {
    axis = Eigen::Vector3d::UnitX();
  }
  return axis.normalized();
}

void LocalExplorationPlanner::updatePassableDirectionField(
    const Eigen::Vector3d& reference_pos) {
  const double cache_resolution =
      std::max(voxel_map_->getResolution(), planning_params_.edge_length_min);
  if (passable_direction_field_valid_ &&
      (passable_direction_field_reference_ - reference_pos).norm() <
          0.5 * cache_resolution) {
    return;
  }

  passable_direction_candidates_.clear();
  passable_direction_field_reference_ = reference_pos;
  passable_direction_field_valid_ = true;

  const int num_dirs = 32;
  const int top_k = 8;
  const double step =
      std::max(voxel_map_->getResolution(), 0.5 * planning_params_.edge_length_min);
  const double local_span_x =
      std::max(0.0, local_space_params_.max_val.x() -
                        local_space_params_.min_val.x());
  const double local_span_y =
      std::max(0.0, local_space_params_.max_val.y() -
                        local_space_params_.min_val.y());
  const double local_horizontal_radius =
      local_space_params_.radius > kSmallNorm
          ? local_space_params_.radius
          : 0.5 * std::min(local_span_x, local_span_y);
  const double max_range =
      std::max(planning_params_.edge_length_max,
               std::min(local_horizontal_radius,
                        2.0 * planning_params_.edge_length_max));
  const double min_range =
      std::max(planning_params_.edge_length_min, step);
  const Eigen::Vector3d ref_axis = getPrimaryExplorationAxis(reference_pos);
  BoundedSpaceParams local_sampling_space = local_space_params_;
  Eigen::Vector3d local_sampling_center = reference_pos;
  local_sampling_space.setCenter(local_sampling_center, false);

  struct DirectionScore {
    double score;
    double length;
    double unknown;
    double angle;
    Eigen::Vector3d dir;
    Eigen::Vector3d target;
  };
  std::vector<DirectionScore> scores;
  scores.reserve(num_dirs * 4);

  struct AnchorPoint {
    Eigen::Vector3d position;
    double reach_distance;
  };
  std::vector<AnchorPoint> anchors;
  anchors.push_back({reference_pos, 0.0});

  const int anchor_dirs = 16;
  const double anchor_step =
      std::max(planning_params_.edge_length_max,
               2.0 * planning_params_.edge_length_min);
  const double anchor_range =
      std::max(anchor_step, std::min(local_horizontal_radius,
                                     planning_params_.edge_length_max));
  for (int i = 0; i < anchor_dirs; ++i) {
    const double angle = -M_PI + 2.0 * M_PI * static_cast<double>(i) /
                                      static_cast<double>(anchor_dirs);
    const Eigen::Vector3d dir(std::cos(angle), std::sin(angle), 0.0);
    Eigen::Vector3d prev = reference_pos + robot_params_.center_offset;
    Eigen::Vector3d farthest_anchor = reference_pos;
    double farthest_distance = 0.0;
    for (double dist = anchor_step; dist <= anchor_range; dist += anchor_step) {
      Eigen::Vector3d anchor = reference_pos + dist * dir;
      if (!local_sampling_space.isInsideSpace(anchor)) {
        break;
      }
      const Eigen::Vector3d center = anchor + robot_params_.center_offset;
      if (voxel_map_->getPathStatus(prev, center, robot_box_size_, true) !=
          VoxelStatus::kFree) {
        break;
      }
      if (voxel_map_->getBoxStatus(center, robot_box_size_, true) !=
          VoxelStatus::kFree) {
        break;
      }
      farthest_anchor = anchor;
      farthest_distance = dist;
      prev = center;
    }
    if (farthest_distance >= min_range) {
      anchors.push_back({farthest_anchor, farthest_distance});
    }
  }

  for (const AnchorPoint& anchor : anchors) {
    const Eigen::Vector3d start = anchor.position + robot_params_.center_offset;
    for (int i = 0; i < num_dirs; ++i) {
      const double angle = -M_PI + 2.0 * M_PI * static_cast<double>(i) /
                                        static_cast<double>(num_dirs);
      const Eigen::Vector3d dir(std::cos(angle), std::sin(angle), 0.0);
      double free_length = 0.0;
      double unknown_count = 0.0;
      Eigen::Vector3d prev = start;
      Eigen::Vector3d target = anchor.position;
      for (double dist = step; dist <= max_range; dist += step) {
        Eigen::Vector3d sample = anchor.position + dist * dir;
        if (!local_sampling_space.isInsideSpace(sample)) {
          break;
        }

        const Eigen::Vector3d point = sample + robot_params_.center_offset;
        const VoxelStatus sample_status = voxel_map_->getVoxelStatus(sample);
        if (sample_status == VoxelStatus::kUnknown) {
          unknown_count += 1.0;
          break;
        }
        if (voxel_map_->getPathStatus(prev, point, robot_box_size_, true) !=
            VoxelStatus::kFree) {
          break;
        }
        prev = point;
        free_length = dist;
        target = sample;

        Eigen::Vector3d lookahead = sample + step * dir;
        if (local_sampling_space.isInsideSpace(lookahead) &&
            voxel_map_->getVoxelStatus(lookahead) == VoxelStatus::kUnknown) {
          unknown_count += 1.0;
        }
      }

      if (free_length < min_range) {
        continue;
      }
      const Eigen::Vector3d root_to_target = target - reference_pos;
      const double root_target_norm = root_to_target.norm();
      if (root_target_norm < min_range) {
        continue;
      }
      const double length_score = free_length / std::max(max_range, kSmallNorm);
      const double unknown_score =
          std::min(1.0, unknown_count / std::max(1.0, max_range / step));
      const double heading_score = std::max(0.0, dir.dot(ref_axis));
      const double root_heading_score =
          std::max(0.0, root_to_target.normalized().dot(ref_axis));
      const double lateral_score =
          std::max(0.0, 1.0 - std::abs(dir.dot(ref_axis)));
      const double penetration_score =
          std::min(1.0, anchor.reach_distance /
                            std::max(anchor_range, kSmallNorm));
      const double opening_bonus =
          unknown_count > 0.0 ? 0.20 : 0.0;
      const double score =
          0.20 * length_score + 0.40 * unknown_score +
          0.08 * heading_score + 0.07 * root_heading_score +
          0.15 * lateral_score + 0.10 * penetration_score +
          opening_bonus;
      const double target_angle =
          std::atan2(root_to_target.y(), root_to_target.x());
      scores.push_back({score, free_length, unknown_count, target_angle,
                        root_to_target.normalized(), target});
    }
  }

  double max_free_length = 0.0;
  for (const DirectionScore& direction_score : scores) {
    max_free_length = std::max(max_free_length, direction_score.length);
  }

  const double directional_length_floor =
      std::max(min_range, 0.45 * max_free_length);
  const double side_passage_length_floor =
      std::max(min_range, 0.25 * max_free_length);
  scores.erase(
      std::remove_if(scores.begin(), scores.end(),
                     [&](const DirectionScore& direction_score) {
                       const bool main_corridor_candidate =
                           direction_score.length >= directional_length_floor;
                       const bool side_passage_candidate =
                           direction_score.length >= side_passage_length_floor;
                       const bool opening_witness_candidate =
                           direction_score.length >= min_range &&
                           direction_score.unknown >= 1.0;
                       return !main_corridor_candidate && !side_passage_candidate &&
                              !opening_witness_candidate;
                     }),
      scores.end());

  std::sort(scores.begin(), scores.end(),
            [](const DirectionScore& lhs, const DirectionScore& rhs) {
              return lhs.score > rhs.score;
            });

  const double min_angle_sep = M_PI / 10.0;
  auto angle_distance = [](double a, double b) {
    double d = a - b;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return std::abs(d);
  };

  std::vector<double> selected_angles;
  for (const DirectionScore& direction_score : scores) {
    bool too_close = false;
    for (double selected_angle : selected_angles) {
      if (angle_distance(direction_score.angle, selected_angle) <
          min_angle_sep) {
        too_close = true;
        break;
      }
    }
    if (too_close) {
      continue;
    }
    selected_angles.push_back(direction_score.angle);
    passable_direction_candidates_.push_back(
        std::make_pair(std::max(direction_score.score, 1e-3),
                       direction_score.target - reference_pos));
    if (static_cast<int>(passable_direction_candidates_.size()) >= top_k) {
      break;
    }
  }
}

bool LocalExplorationPlanner::samplePassableDirection(const StateVec& root_state,
                                    StateVec& state) {
  if (!planning_params_.axis_guided_sampling_enable ||
      planning_params_.disable_local_sample_guidance ||
      planning_params_.axis_biased_sampling_ratio <= 0.0) {
    return false;
  }

  static thread_local std::mt19937 generator(std::random_device{}());
  static thread_local std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
  const double candidate_ratio =
      std::max(0.0, std::min(1.0, planning_params_.axis_biased_sampling_ratio));
  if (unit_dist(generator) >= candidate_ratio) {
    return false;
  }

  const Eigen::Vector3d reference_pos = root_state.head(3);
  updatePassableDirectionField(reference_pos);
  if (passable_direction_candidates_.empty()) {
    return false;
  }

  std::vector<double> weights;
  weights.reserve(passable_direction_candidates_.size());
  for (const auto& candidate : passable_direction_candidates_) {
    weights.push_back(candidate.first);
  }
  std::discrete_distribution<int> candidate_dist(weights.begin(), weights.end());
  const int candidate_id = candidate_dist(generator);
  Eigen::Vector3d candidate_vec = passable_direction_candidates_[candidate_id].second;
  const double candidate_length = candidate_vec.norm();
  if (candidate_length <= kSmallNorm) {
    return false;
  }

  const Eigen::Vector3d dir = candidate_vec / candidate_length;
  const Eigen::Vector3d lateral(-dir.y(), dir.x(), 0.0);
  std::uniform_real_distribution<double> forward_dist(
      std::max(planning_params_.edge_length_min, voxel_map_->getResolution()),
      std::max(candidate_length, planning_params_.edge_length_min));
  const double lateral_std = std::max(
      0.5 * voxel_map_->getResolution(),
      std::min(0.25 * planning_params_.edge_length_min,
               0.5 * planning_params_.axis_biased_lateral_scale *
                   planning_params_.edge_length_min));
  std::normal_distribution<double> lateral_dist(0.0, lateral_std);
  std::normal_distribution<double> vertical_dist(
      0.0, std::max(0.5 * voxel_map_->getResolution(),
                    0.25 * planning_params_.edge_length_min));

  state = root_state;
  state.head(3) = reference_pos + forward_dist(generator) * dir +
                  lateral_dist(generator) * lateral;
  state[2] += vertical_dist(generator);
  state[3] = std::atan2(dir.y(), dir.x());
  return true;
}

bool LocalExplorationPlanner::isZonePartitionActive() const {
  if (!planning_params_.zone_partition_enable ||
      global_space_params_.type != BoundedSpaceType::kCuboid ||
      planning_params_.zone_partition_rows <= 0 ||
      planning_params_.zone_partition_cols <= 0) {
    return false;
  }
  Eigen::Vector3d span = global_space_params_.max_val - global_space_params_.min_val;
  return span.x() > kSmallNorm && span.y() > kSmallNorm;
}

int LocalExplorationPlanner::getZoneIndex(const Eigen::Vector3d& position) const {
  if (!isZonePartitionActive()) {
    return -1;
  }

  const Eigen::Vector3d span =
      global_space_params_.max_val - global_space_params_.min_val;
  const double x_ratio = std::min(
      0.999999,
      std::max(0.0, (position.x() - global_space_params_.min_val.x()) / span.x()));
  const double y_ratio = std::min(
      0.999999,
      std::max(0.0, (position.y() - global_space_params_.min_val.y()) / span.y()));

  const int col = std::min(planning_params_.zone_partition_cols - 1,
                           static_cast<int>(x_ratio *
                                            planning_params_.zone_partition_cols));
  const int row = std::min(planning_params_.zone_partition_rows - 1,
                           static_cast<int>(y_ratio *
                                            planning_params_.zone_partition_rows));
  return row * planning_params_.zone_partition_cols + col;
}

double LocalExplorationPlanner::getZoneTravelFactor(const Eigen::Vector3d& source,
                                const Eigen::Vector3d& target) const {
  if (!isZonePartitionActive()) {
    return 1.0;
  }

  const int source_zone = getZoneIndex(source);
  const int target_zone = getZoneIndex(target);
  if (source_zone < 0 || target_zone < 0) {
    return 1.0;
  }

  if (source_zone == target_zone) {
    return 1.0 + planning_params_.zone_same_region_bonus;
  }

  const int cols = planning_params_.zone_partition_cols;
  const int source_row = source_zone / cols;
  const int source_col = source_zone % cols;
  const int target_row = target_zone / cols;
  const int target_col = target_zone % cols;
  const int zone_distance =
      std::abs(source_row - target_row) + std::abs(source_col - target_col);

  return std::exp(-planning_params_.zone_cross_penalty * zone_distance);
}

double LocalExplorationPlanner::computeSampleUtility(const StateVec& state,
                                 const Eigen::Vector3d& reference_pos) const {
  const Eigen::Vector3d sample_pos = state.head(3);
  Eigen::Vector3d sample_dir = sample_pos - reference_pos;
  const Eigen::Vector3d axis = getPrimaryExplorationAxis(reference_pos);

  double axis_alignment = 0.0;
  if (sample_dir.norm() > kSmallNorm) {
    sample_dir.normalize();
    axis_alignment = std::max(0.0, sample_dir.dot(axis));
  }

  double goal_alignment = 0.0;
  Eigen::Vector3d goal_dir = Eigen::Vector3d::Zero();
  if (global_exploration_ongoing_ && global_graph_ != NULL &&
      current_global_vertex_id_ >= 0 &&
      current_global_vertex_id_ < global_graph_->getNumVertices()) {
    goal_dir =
        global_graph_->getVertex(current_global_vertex_id_)->state.head(3) -
        reference_pos;
  } else if (next_compartment_.x() < std::numeric_limits<double>::max() / 2.0) {
    goal_dir = next_compartment_ - reference_pos;
  }
  if (goal_dir.norm() > kSmallNorm && sample_dir.norm() > kSmallNorm) {
    goal_alignment =
        std::max(0.0, sample_dir.dot(goal_dir.normalized()));
  }

  const double zone_bonus =
      planning_params_.sample_same_zone_bonus *
      (getZoneTravelFactor(reference_pos, sample_pos) - 1.0);

  return planning_params_.sample_axis_alignment_weight * axis_alignment +
         planning_params_.sample_clearance_weight *
             getClearanceScore(sample_pos) +
         planning_params_.sample_goal_alignment_weight * goal_alignment +
         zone_bonus;
}

void LocalExplorationPlanner::applyAxisBiasedSampling(const Eigen::Vector3d& reference_pos,
                                  StateVec& state) const {
  if (!planning_params_.axis_guided_sampling_enable ||
      planning_params_.disable_local_sample_guidance) {
    return;
  }

  const double ratio =
      std::max(0.0, std::min(1.0, planning_params_.axis_biased_sampling_ratio));
  if (ratio <= 0.0) {
    return;
  }

  static thread_local std::mt19937 generator(std::random_device{}());
  static thread_local std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
  if (unit_dist(generator) >= ratio) {
    return;
  }

  Eigen::Vector3d axis = getPrimaryExplorationAxis(reference_pos);
  if (axis.norm() <= kSmallNorm) {
    return;
  }
  axis.normalize();

  const double lateral_scale = std::max(
      0.0, std::min(1.0, planning_params_.axis_biased_lateral_scale));
  const double backward_scale = std::max(
      0.0, std::min(1.0, planning_params_.axis_biased_backward_scale));

  const Eigen::Vector3d delta = state.head(3) - reference_pos;
  const double axial_dist = delta.dot(axis);
  Eigen::Vector3d axial_delta = axial_dist * axis;
  Eigen::Vector3d lateral_delta = delta - axial_delta;

  if (axial_dist < 0.0) {
    axial_delta *= backward_scale;
  }
  lateral_delta *= lateral_scale;
  state.head(3) = reference_pos + axial_delta + lateral_delta;
}

void LocalExplorationPlanner::publishAblationMetrics() {
  std_msgs::Float64MultiArray ablation_log;
  ablation_log.data.reserve(9);
  ablation_log.data.push_back(static_cast<double>(total_sampling_attempts_));
  ablation_log.data.push_back(static_cast<double>(total_valid_samples_));
  ablation_log.data.push_back(static_cast<double>(total_candidate_viewpoints_));
  ablation_log.data.push_back(static_cast<double>(total_evaluated_viewpoints_));
  const double viewpoint_utilization =
      total_candidate_viewpoints_ > 0
          ? static_cast<double>(total_evaluated_viewpoints_) /
                static_cast<double>(total_candidate_viewpoints_)
          : 0.0;
  ablation_log.data.push_back(viewpoint_utilization);
  ablation_log.data.push_back(total_global_fallback_path_length_);
  ablation_log.data.push_back(total_global_frontier_fallback_path_length_);
  ablation_log.data.push_back(total_global_frontier_fallback_time_);
  ablation_log.data.push_back(
      static_cast<double>(total_global_frontier_fallback_count_));
  ablation_log_pub_.publish(ablation_log);
}

void LocalExplorationPlanner::accumulateSamplingAttempt(bool valid_sample) {
  ++total_sampling_attempts_;
  if (valid_sample) {
    ++total_valid_samples_;
  }
}

void LocalExplorationPlanner::accumulateViewpointEvaluation(int candidate_count,
                                        int evaluated_count) {
  total_candidate_viewpoints_ += std::max(0, candidate_count);
  total_evaluated_viewpoints_ += std::max(0, evaluated_count);
}

void LocalExplorationPlanner::accumulateGlobalFallbackPathLength(
    const std::vector<geometry_msgs::Pose>& path) {
  if (path.size() <= 1) {
    return;
  }
  total_global_fallback_path_length_ += Trajectory::getPathLength(path);
}

void LocalExplorationPlanner::accumulateGlobalFrontierFallbackMetrics(
    const std::vector<geometry_msgs::Pose>& path) {
  if (path.size() <= 1) {
    return;
  }
  const double path_length = Trajectory::getPathLength(path);
  const double reference_speed = std::max(planning_params_.v_homing_max, 1e-3);
  total_global_frontier_fallback_path_length_ += path_length;
  total_global_frontier_fallback_time_ += path_length / reference_speed;
  ++total_global_frontier_fallback_count_;
}

void LocalExplorationPlanner::augmentFreeStartupArea(const StateVec& state, double reach) {
  Eigen::Vector3d center = state.head(3) + robot_params_.center_offset;
  const double voxel_size = std::max(voxel_map_->getResolution(), 1e-3);
  const double step = std::max(
      voxel_size, 0.5 * std::max(planning_params_.edge_length_min, voxel_size));
  Eigen::Vector3d clear_box = robot_box_size_;
  clear_box.x() += 2.0 * step;
  clear_box.y() += 2.0 * step;
  clear_box.z() += step;

  voxel_map_->augmentFreeBox(center, clear_box);

  if (reach <= step) {
    return;
  }

  std::vector<double> headings = {
      state[3],
      state[3] + M_PI_2,
      state[3] - M_PI_2,
      state[3] + M_PI,
      state[3] + M_PI_4,
      state[3] - M_PI_4,
      state[3] + 3.0 * M_PI_4,
      state[3] - 3.0 * M_PI_4};

  for (double heading : headings) {
    Eigen::Vector3d dir(std::cos(heading), std::sin(heading), 0.0);
    for (double dist = step; dist <= reach; dist += step) {
      voxel_map_->augmentFreeBox(center + dist * dir, clear_box);
    }
  }
}

double LocalExplorationPlanner::getLocalNavigationRecoveryReach() const {
  if (planning_params_.local_navigation_recovery_distance > 0.0) {
    return planning_params_.local_navigation_recovery_distance;
  }
  return std::max(
      planning_params_.edge_length_min,
      std::min(planning_params_.edge_length_max,
               std::max(planning_params_.nearest_range_min,
                        planning_params_.nearest_range)));
}

bool LocalExplorationPlanner::maybeRecoverLocalNavigation(const Eigen::Vector3d& local_target,
                                      const std::string& reason) {
  if (!planning_params_.local_navigation_recovery_enable ||
      planning_params_.local_navigation_recovery_max_attempts <= 0) {
    return false;
  }
  if (local_navigation_recovery_attempts_ >=
      planning_params_.local_navigation_recovery_max_attempts) {
    return false;
  }

  const Eigen::Vector3d robot_center =
      current_state_.head(3) + robot_params_.center_offset;
  const double robot_radius = std::max(0.05, robot_box_size_.maxCoeff() * 0.5);
  const double min_path_clearance =
      std::max(planning_params_.local_navigation_min_clearance,
               robot_radius + 0.05);
  const double recovery_trigger_clearance = 1.25 * min_path_clearance;
  const double current_clearance = voxel_map_->getPointDistance(robot_center);
  const VoxelStatus box_status =
      voxel_map_->getBoxStatus(robot_center, robot_box_size_, true);
  const bool trapped_near_wall_or_corner =
      (box_status != VoxelStatus::kFree) || (current_clearance < 0.0) ||
      (current_clearance < recovery_trigger_clearance);
  if (!trapped_near_wall_or_corner) {
    return false;
  }

  const double voxel_size = std::max(voxel_map_->getResolution(), 1e-3);
  const double recovery_reach =
      std::max(voxel_size, getLocalNavigationRecoveryReach());
  const double step = std::max(
      voxel_size, 0.5 * std::max(planning_params_.edge_length_min, voxel_size));

  Eigen::Vector3d clear_box = robot_box_size_;
  clear_box.x() += 2.0 * step;
  clear_box.y() += 2.0 * step;
  clear_box.z() += 2.0 * step;

  augmentFreeStartupArea(current_state_, recovery_reach);
  voxel_map_->augmentFreeBox(robot_center, clear_box);

  std::vector<Eigen::Vector3d> recovery_dirs;
  auto add_dir = [&](const Eigen::Vector3d& raw_dir) {
    Eigen::Vector3d dir = raw_dir;
    dir.z() = 0.0;
    const double norm = dir.norm();
    if (norm <= 1e-3) {
      return;
    }
    recovery_dirs.push_back(dir / norm);
  };
  auto add_heading = [&](double yaw) {
    add_dir(Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0));
  };

  const Eigen::Vector3d goal_delta = local_target - current_state_.head(3);
  add_dir(goal_delta);

  const double current_yaw = current_state_[3];
  add_heading(current_yaw);
  add_heading(current_yaw + M_PI_4);
  add_heading(current_yaw - M_PI_4);
  add_heading(current_yaw + M_PI_2);
  add_heading(current_yaw - M_PI_2);

  if (goal_delta.head(2).norm() > 1e-3) {
    const double goal_yaw = std::atan2(goal_delta.y(), goal_delta.x());
    add_heading(goal_yaw);
    add_heading(goal_yaw + M_PI_4);
    add_heading(goal_yaw - M_PI_4);
  }

  for (const Eigen::Vector3d& dir : recovery_dirs) {
    for (double dist = step; dist <= recovery_reach; dist += step) {
      voxel_map_->augmentFreeBox(robot_center + dist * dir, clear_box);
    }
  }

  voxel_map_->augmentFreeBox(robot_center + Eigen::Vector3d(0.0, 0.0, step),
                               clear_box);
  voxel_map_->augmentFreeBox(robot_center + Eigen::Vector3d(0.0, 0.0, -step),
                               clear_box);

  ++local_navigation_recovery_attempts_;
  local_goal_progress_fail_iters_ = 0;
  local_goal_distance_reached_ =
      (local_navigation_goal_ - current_state_.head(3)).norm();

  ROS_WARN_COND(
      global_verbosity >= Verbosity::WARN,
      "[Local Navigation] Trigger corner recovery %d/%d (%s). clearance=%.2f "
      "reach=%.2f",
      local_navigation_recovery_attempts_,
      planning_params_.local_navigation_recovery_max_attempts,
      reason.c_str(), current_clearance, recovery_reach);
  return true;
}

void LocalExplorationPlanner::shortlistVerticesForFineGain(
    bool only_leaf_vertices, std::unordered_set<int>& selected_ids) {
  selected_ids.clear();
  last_gain_eval_partial_ = false;

  if (!planning_params_.fine_gain_evaluation_enable ||
      force_full_gain_evaluation_) {
    return;
  }

  std::vector<std::pair<double, int>> ranked_vertices;
  ranked_vertices.reserve(local_graph_->getNumVertices());
  const Eigen::Vector3d reference_pos =
      root_vertex_ ? root_vertex_->state.head(3) : current_state_.head(3);
  for (const auto& entry : local_graph_->vertices_map_) {
    Vertex* vertex = entry.second;
    if (only_leaf_vertices && !vertex->is_leaf_vertex) {
      continue;
    }
    ranked_vertices.emplace_back(
        computeSampleUtility(vertex->state, reference_pos), vertex->id);
  }

  if (ranked_vertices.empty()) {
    return;
  }

  const int total_vertices = ranked_vertices.size();
  const int desired_count = std::max(
      planning_params_.fine_gain_top_k,
      static_cast<int>(std::ceil(total_vertices *
                                 planning_params_.fine_gain_candidate_ratio)));
  if (desired_count <= 0 || desired_count >= total_vertices) {
    return;
  }

  std::sort(ranked_vertices.begin(), ranked_vertices.end(),
            [](const std::pair<double, int>& lhs,
               const std::pair<double, int>& rhs) {
              return lhs.first > rhs.first;
            });

  const int angular_bins = 12;
  std::vector<std::pair<double, int>> best_per_bin(
      angular_bins, std::make_pair(-std::numeric_limits<double>::infinity(),
                                   -1));
  for (const auto& candidate : ranked_vertices) {
    Vertex* vertex = local_graph_->getVertex(candidate.second);
    Eigen::Vector3d delta = vertex->state.head(3) - reference_pos;
    if (delta.head(2).norm() <= kSmallNorm) {
      continue;
    }
    double angle = std::atan2(delta.y(), delta.x()) + M_PI;
    int bin = std::min(angular_bins - 1,
                       std::max(0, static_cast<int>(
                                      angular_bins * angle / (2.0 * M_PI))));
    if (candidate.first > best_per_bin[bin].first) {
      best_per_bin[bin] = candidate;
    }
  }
  for (const auto& candidate : best_per_bin) {
    if (candidate.second >= 0 &&
        static_cast<int>(selected_ids.size()) < desired_count) {
      selected_ids.insert(candidate.second);
    }
  }

  for (int i = 0; i < total_vertices &&
                  static_cast<int>(selected_ids.size()) < desired_count;
       ++i) {
    selected_ids.insert(ranked_vertices[i].second);
  }
  last_gain_eval_partial_ = true;
}

bool LocalExplorationPlanner::sampleVertexCandidate(RandomSampler& random_sampler,
                                const StateVec& root_state, Vertex& vertex,
                                bool apply_geofence_check,
                                bool apply_axis_bias) {
  StateVec state = StateVec::Zero();
  StateVec best_state = StateVec::Zero();
  StateVec sampler_root_state = root_state;
  bool hanging = false;
  bool best_hanging = false;
  double best_score = -std::numeric_limits<double>::infinity();
  const bool local_sample_guidance_enable =
      planning_params_.axis_guided_sampling_enable &&
      !planning_params_.disable_local_sample_guidance;
  const int candidate_target = local_sample_guidance_enable
                                   ? std::max(1, planning_params_.sample_candidate_batch_size)
                                   : 1;

  int while_thres = 1000;  // magic number.
  int valid_candidates = 0;
  BoundedSpaceParams reduced_global_space = global_space_params_;
  reduced_global_space.min_val += 0.5 * robot_box_size_;
  reduced_global_space.max_val -= 0.5 * robot_box_size_;
  BoundedSpaceParams local_sampling_space = local_space_params_;
  Eigen::Vector3d local_sampling_center = root_state.head(3);
  local_sampling_space.setCenter(local_sampling_center, false);
  const bool apply_local_cylinder_check =
      (&random_sampler == &random_sampler_) &&
      local_sampling_space.type == BoundedSpaceType::kCylinder;

  bool found = false;
  while (while_thres-- && valid_candidates < candidate_target) {
    hanging = false;
    const bool candidate_sampled =
        apply_axis_bias && samplePassableDirection(root_state, state);
    if (!candidate_sampled) {
      random_sampler.generate(sampler_root_state, state);
    }
    if (apply_axis_bias && !candidate_sampled) {
      applyAxisBiasedSampling(root_state.head(3), state);
    }
    accumulateSamplingAttempt(false);
    Eigen::Vector3d sample = state.head(3);
    if (!reduced_global_space.isInsideSpace(sample)) {
      continue;
    }
    if (apply_local_cylinder_check &&
        !local_sampling_space.isInsideSpace(sample)) {
      continue;
    }

    if (apply_geofence_check &&
        planning_params_.geofence_checking_enable &&
        GeofenceManager::CoordinateStatus::kViolated ==
            geofence_manager_->getBoxStatus(
                Eigen::Vector2d(state[0] + robot_params_.center_offset[0],
                                state[1] + robot_params_.center_offset[1]),
                Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))) {
      continue;
    }


    if (VoxelStatus::kFree ==
        voxel_map_->getBoxStatus(
            Eigen::Vector3d(state[0], state[1], state[2]) +
                robot_params_.center_offset,
            robot_box_size_, true)) {
      ++total_valid_samples_;
      random_sampler.pushSample(state, true);
      ++valid_candidates;
      const double score =
          local_sample_guidance_enable
              ? computeSampleUtility(state, root_state.head(3))
              : 0.0;
      if (!found || score > best_score) {
        best_score = score;
        best_state = state;
        best_hanging = hanging;
        found = true;
      }
    } else {
      stat_->num_vertices_fail++;
      random_sampler.pushSample(state, false);
    }
  }

  vertex.state = found ? best_state : state;
  vertex.is_hanging = found ? best_hanging : hanging;
  return found;
}

bool LocalExplorationPlanner::sampleVertex(Vertex& vertex) {
  return sampleVertexCandidate(random_sampler_, root_vertex_->state, vertex,
                               true, true);
}

bool LocalExplorationPlanner::sampleVertex(RandomSampler& random_sampler, StateVec& root_state,
                       Vertex& vertex) {
  return sampleVertexCandidate(random_sampler, root_state, vertex, false, false);
}

bool LocalExplorationPlanner::sampleVertex(RandomSampler& random_sampler, StateVec& root_state,
                       Vertex& vertex, bool apply_axis_bias) {
  return sampleVertexCandidate(random_sampler, root_state, vertex, false,
                               apply_axis_bias);
}

