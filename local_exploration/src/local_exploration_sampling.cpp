#include "local_exploration/local_exploration_sampling.h"

RandomSamplerBase::RandomSamplerBase() {
  pdf_type_ = RandomDistributionType::kUniform;
  init_pdf_type_ = RandomDistributionType::kUniform;
  sample_mode_ = SampleModeType::kManual;
  const_val_ = 0.0;
  mean_val_ = 0.0;
  std_val_ = 0.0;
  init_std_val_ = 0.0;
  min_val_ = 0.0;
  max_val_ = 0.0;
  use_bound_ = false;
}

RandomSamplerBase::~RandomSamplerBase() {}

bool RandomSamplerBase::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;
  std::vector<double> param_val;
  std::string parse_str;

  param_name = ns + "/pdf_type";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kUniform"))
    pdf_type_ = RandomDistributionType::kUniform;
  else if (!parse_str.compare("kNormal"))
    pdf_type_ = RandomDistributionType::kNormal;
  else if (!parse_str.compare("kCauchy"))
    pdf_type_ = RandomDistributionType::kCauchy;
  else if (!parse_str.compare("kConst"))
    pdf_type_ = RandomDistributionType::kConst;
  else if (!parse_str.compare("kNormalUniform"))
    pdf_type_ = RandomDistributionType::kNormalUniform;
  else {
    ROSPARAM_ERROR(ns);
    return false;
  }

  
  if ((pdf_type_ == RandomDistributionType::kNormalUniform) || (pdf_type_ == RandomDistributionType::kNormal)){
    param_name = ns + "/std_val";
    if (!ros::param::get(param_name, std_val_)) {
      ROSPARAM_ERROR(param_name);
      return false;
    }
    init_pdf_type_ = pdf_type_; // In case of kNormalUnform: pdf_type will change
    init_std_val_ = std_val_;   // In case of kAdaptiveExploration: std_val will change based on the local bounding box
  } else {
    init_pdf_type_ = pdf_type_;
  }

  param_name = ns + "/sample_mode";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kManual"))
    sample_mode_ = SampleModeType::kManual;
  else if (!parse_str.compare("kLocal")) {
    sample_mode_ = SampleModeType::kLocal;
    // Have to load later from the BoundedSpaceParams/Local
  } else if (!parse_str.compare("kGlobal")) {
    sample_mode_ = SampleModeType::kGlobal;
    // Have to load later from the BoundedSpaceParams/Global
  } else if (!parse_str.compare("kExploredMap")) {
    sample_mode_ = SampleModeType::kExploredMap;
    // Have to be set online.
  } else {
    ROSPARAM_ERROR(ns);
    return false;
  }

  if (sample_mode_ == SampleModeType::kManual) {
    if (pdf_type_ == RandomDistributionType::kUniform) {
      // Uniform pdf: min, max
      param_name = ns + "/min_val";
      if (!ros::param::get(param_name, min_val_)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
      param_name = ns + "/max_val";
      if (!ros::param::get(param_name, max_val_)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
    } else if (pdf_type_ == RandomDistributionType::kNormalUniform || pdf_type_ == RandomDistributionType::kNormal) {
      // NormalUniform and Normal pdf: mean, as well as min and max to truncate if required
      use_bound_ = true;
      param_name = ns + "/min_val";
      if (!ros::param::get(param_name, min_val_)) {
        ROSPARAM_WARN(param_name, "not used");
        use_bound_ = false;
      }
      param_name = ns + "/max_val";
      if (!ros::param::get(param_name, max_val_)) {
        ROSPARAM_WARN(param_name, "not used");
        use_bound_ = false;
      }
      param_name = ns + "/mean_val";
      if (!ros::param::get(param_name, mean_val_)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
      // param_name = ns + "/std_val";
      // if (!ros::param::get(param_name, std_val_)) {
      //   ROSPARAM_ERROR(param_name);
      //   return false;
      // }
    } else if (pdf_type_ == RandomDistributionType::kConst) {
      // Set const value.
      param_name = ns + "/const_val";
      if (!ros::param::get(param_name, const_val_)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
    }
  }
  ROSPARAM_INFO("Done.");
  return true;
}

void RandomSamplerBase::setParams(int ind, BoundedSpaceParams& global_params,
                                  BoundedSpaceParams& local_params) {
  // First, load some parameters from bounded space settings.
  BoundedSpaceParams* params;
  if (SampleModeType::kGlobal == sample_mode_)
    params = &global_params;
  else if (sample_mode_ == SampleModeType::kLocal)
    params = &local_params;
  else
    return;

  if ((pdf_type_ == RandomDistributionType::kUniform) || 
      (pdf_type_ == RandomDistributionType::kNormal) ||
      (pdf_type_ == RandomDistributionType::kNormalUniform)) {
    if (params->type == BoundedSpaceType::kCuboid) {
      min_val_ = params->min_val[ind];
      max_val_ = params->max_val[ind];
    } else if (params->type == BoundedSpaceType::kSphere) {
      min_val_ = -params->radius;
      max_val_ = params->radius;
    } else if (params->type == BoundedSpaceType::kCylinder) {
      if (ind < 2) {
        min_val_ = -params->radius;
        max_val_ = params->radius;
      } else {
        min_val_ = params->min_val[ind];
        max_val_ = params->max_val[ind];
      }
    }
  }
}

void RandomSamplerBase::setBound(double min_val, double max_val) {
  // Set directly without checking mode.
  min_val_ = min_val;
  max_val_ = max_val;
}

void RandomSamplerBase::setDistributionParams(double mean_val, double std_val) {
  // Set directly without checking mode.
  mean_val_ = mean_val;
  std_val_ = std_val;
}

void RandomSamplerBase::setSTD(double scl_factor){
  // It is applicable only in the case of kAdapiveExploration
  std_val_ = init_std_val_*scl_factor;
}

void RandomSamplerBase::setPDF(RandomSamplerBase::RandomDistributionType pdf_type){
  // It is applicable only in the case of kNormalUniform
  pdf_type_ = pdf_type;
  reset();
}

void RandomSamplerBase::reset() {
  std::random_device rd;
  generator_.seed(rd());
  if (pdf_type_ == RandomDistributionType::kUniform) {
    uniform_pdf_.reset(new std::uniform_real_distribution<>(min_val_, max_val_));
  } else if (pdf_type_ == RandomDistributionType::kNormal) {
    normal_pdf_.reset(new std::normal_distribution<>(mean_val_, std_val_));
  } else if (pdf_type_ == RandomDistributionType::kCauchy) {
    // Note: naming of std_val is slightly inaccurate
    normal_pdf_.reset(new std::normal_distribution<>(0, std_val_));
  }
}

double RandomSamplerBase::generate(double current_val, double chi_sq_var) {
  double r;
  if (pdf_type_ == RandomDistributionType::kUniform) {
    r = (*uniform_pdf_)(generator_);
  } else if (pdf_type_ == RandomDistributionType::kNormal) {
    int while_thres = 1000;  // magic number
    do {
      r = normal_pdf_->operator()(generator_);
    } while ((r < min_val_ || r > max_val_) && while_thres--);
  } else if (pdf_type_ == RandomDistributionType::kCauchy) {
    r = normal_pdf_->operator()(generator_) / sqrt(chi_sq_var) + mean_val_;
  } else if (pdf_type_ == RandomDistributionType::kConst) {
    r = const_val_;
  } else {
    ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Not supported yet.");
    return 0.0;
  }

  // Offset to current pose; except if this is global setting.
  if (sample_mode_ != SampleModeType::kGlobal) r += current_val;

  return r;
}

double RandomSamplerBase::getZOffset() {
  if (pdf_type_ == RandomDistributionType::kConst) {
    return const_val_;
  } else {
    return 0.0;
  }
}

RandomSamplerBase::RandomDistributionType RandomSamplerBase::getInitPDF(){
  // It is applicable only in the case of kNormalUniform
  return init_pdf_type_;
}

RandomSamplerBase::RandomDistributionType RandomSamplerBase::getPDF(){
  return pdf_type_;
}

double RandomSamplerBase::getRange(){
  // It is applicable only in the case of kAdapiveExploration
  return (max_val_-min_val_);
}

RandomSampler::RandomSampler() {
  random_sampler_base_.clear();
  random_sampler_base_.resize(5);
  valid_samples_.clear();
}
RandomSampler::~RandomSampler() {}

bool RandomSampler::loadParams(std::string ns) {
  if (!random_sampler_base_[0].loadParams(ns + "/X")) return false;
  if (!random_sampler_base_[1].loadParams(ns + "/Y")) return false;
  if (!random_sampler_base_[2].loadParams(ns + "/Z")) return false;
  if (!random_sampler_base_[3].loadParams(ns + "/Heading")) return false;
  if (!random_sampler_base_[4].loadParams(ns + "/Pitch")) return false;
  isReady = true;

  return true;
}

bool RandomSampler::setBound(Eigen::Vector3d& min_val,
                             Eigen::Vector3d& max_val) {
  // This applied to x,y,z only
  for (int i = 0; i < 3; ++i) {
    random_sampler_base_[i].setBound(min_val[i], max_val[i]);
  }
  reset();
  return true;
}

bool RandomSampler::setRotation(Eigen::Vector3d& rotations) {
  Eigen::Matrix3d rot_W2B;
  rot_W2B = Eigen::AngleAxisd(rotations[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rotations[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rotations[2], Eigen::Vector3d::UnitX());
  rot_B2W = rot_W2B.inverse();
  return true;
}

bool RandomSampler::setDistributionParams(Eigen::Vector3d& mean_val,
                                          Eigen::Vector3d& std_val) {
  for (int i = 0; i < 3; ++i) {
    random_sampler_base_[i].setDistributionParams(mean_val[i], std_val[i]);
  }
  reset();
  return true;
}

bool RandomSampler::setParams(BoundedSpaceParams& global_params,
                              BoundedSpaceParams& local_params, bool rotation) {
  // This applied to x,y,z only
  double largerRange = 0;
  double scaling_factor;
  for (int i = 0; i < 3; ++i) {
    random_sampler_base_[i].setParams(i, global_params, local_params);
    if (largerRange < random_sampler_base_[i].getRange())
      largerRange = random_sampler_base_[i].getRange();
  }
  // It is applicable only in the case of kBasicExploration
  for (int i = 0; i < 3; ++i){
    scaling_factor = random_sampler_base_[i].getRange()/largerRange;
    random_sampler_base_[i].setSTD(scaling_factor);
  }

  if (rotation) {
    Eigen::Matrix3d rot_W2B;
    rot_W2B =
        Eigen::AngleAxisd(local_params.rotations[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(local_params.rotations[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(local_params.rotations[2], Eigen::Vector3d::UnitX());
    rot_B2W = rot_W2B.inverse();
  } else {
    rot_B2W = Eigen::Matrix3d::Identity();
  }

  reset();
  return true;
}

void RandomSampler::setSTD(){
  // It is applicable only in the case of kAdapiveExploration
  double largerRange = 0;
  double scaling_factor;
  for (int i = 0; i < 3; ++i) {
    if (largerRange < random_sampler_base_[i].getRange())
      largerRange = random_sampler_base_[i].getRange();
  }
  for (int i = 0; i < 3; ++i){
    scaling_factor = random_sampler_base_[i].getRange()/largerRange;
    random_sampler_base_[i].setSTD(scaling_factor);
  }
}

void RandomSampler::setPDF(RandomSamplerBase::RandomDistributionType pdf_type, int axis){
  std::random_device rd;
  generator_.seed(rd());
  chi_squared_.reset(new std::chi_squared_distribution<>(1));
  random_sampler_base_[axis].setPDF(pdf_type);
  random_sampler_base_[axis].reset();
}

std::vector<RandomSamplerBase::RandomDistributionType> RandomSampler::getInitPDF(){
  // It is applicable only in the case of kNormalUniform
  std::vector<RandomSamplerBase::RandomDistributionType> tmp;
  for (int i=0; i<5; i++){
    tmp.push_back(random_sampler_base_[i].getInitPDF());
  }
  return tmp;
}

std::vector<RandomSamplerBase::RandomDistributionType> RandomSampler::getPDF(){
  std::vector<RandomSamplerBase::RandomDistributionType> tmp;
  for (int i=0; i<5; i++){
    tmp.push_back(random_sampler_base_[i].getPDF());
  }
  return tmp;
}

int RandomSampler::getInvalidSamplesNum(){
  return invalid_samples_.size();
}

void RandomSampler::RandomSampler::reset() {
  std::random_device rd;
  generator_.seed(rd());
  chi_squared_.reset(new std::chi_squared_distribution<>(1));
  for (int i = 0; i < 5; ++i) {
    random_sampler_base_[i].reset();
  }
  valid_samples_.clear();
  invalid_samples_.clear();
}

void RandomSampler::generate(StateVec& current_state, StateVec& sample_state) {
  double r = chi_squared_->operator()(generator_);
  for (int i = 0; i < 5; ++i) {
    sample_state[i] = random_sampler_base_[i].generate(0, r);
  }
  Eigen::Vector3d sample_xyz = rot_B2W * sample_state.head(3);
  sample_state[0] = sample_xyz[0] + current_state[0];
  sample_state[1] = sample_xyz[1] + current_state[1];
  sample_state[2] = sample_xyz[2] + current_state[2];
  sample_state[3] = sample_state[3] + current_state[3];
  sample_state[4] = sample_state[4] + current_state[4];
}

void RandomSampler::pushSample(StateVec& state, bool valid) {
  if (valid) {
    if (valid_samples_.size() < kValidSamplesMaxSize)
      valid_samples_.emplace_back(state);
  } else {
    if (invalid_samples_.size() < kInvalidSamplesMaxSize)
      invalid_samples_.emplace_back(state);
  }
}
#include "local_exploration/local_exploration_planner.h"

#include <algorithm>
#include <random>
#include <pcl/common/transforms.h>
#include <std_msgs/Float64MultiArray.h>
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
