#include "local_exploration/local_exploration_planner.h"

#include <algorithm>
#include <random>
#include <pcl/common/transforms.h>
#include <tf/transform_listener.h>

void LocalExplorationPlanner::expandGraph(std::shared_ptr<CognitiveGraph> graph_manager,
                      StateVec& new_state, ExpandGraphReport& rep,
                      bool allow_short_edge) {
  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&new_state, &nearest_vertex)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  if (nearest_vertex == NULL) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                            new_state[2] - origin[2]);
  double direction_norm = direction.norm();
  if (direction_norm > planning_params_.edge_length_max) {
    direction = planning_params_.edge_length_max * direction.normalized();
  } else if ((!allow_short_edge) &&
             (direction_norm <= planning_params_.edge_length_min)) {
    rep.status = ExpandGraphStatus::kErrorShortEdge;
    return;
  }
  direction_norm = direction.norm();
  new_state[0] = origin[0] + direction[0];
  new_state[1] = origin[1] + direction[1];
  new_state[2] = origin[2] + direction[2];

  Eigen::Vector3d overshoot_vec =
      planning_params_.edge_overshoot * direction.normalized();
  Eigen::Vector3d start_pos = origin + robot_params_.center_offset;
  if (nearest_vertex->id != 0) start_pos = start_pos - overshoot_vec;
  Eigen::Vector3d end_pos =
      origin + robot_params_.center_offset + direction + overshoot_vec;

  if (planning_params_.geofence_checking_enable &&
      (GeofenceManager::CoordinateStatus::kViolated ==
       geofence_manager_->getPathStatus(
           Eigen::Vector2d(start_pos[0], start_pos[1]),
           Eigen::Vector2d(end_pos[0], end_pos[1]),
           Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
    rep.status = ExpandGraphStatus::kErrorGeofenceViolated;
    return;
  }

  bool admissible_edge = false;
  VoxelStatus vs =
      voxel_map_->getPathStatus(start_pos, end_pos, robot_box_size_, true);
  if (VoxelStatus::kFree == vs) {
    admissible_edge = true;
  }
  if (admissible_edge) {
    Vertex* new_vertex =
        new Vertex(graph_manager->generateVertexID(), new_state);
    new_vertex->parent = nearest_vertex;
    new_vertex->distance = nearest_vertex->distance + direction_norm;
    nearest_vertex->children.push_back(new_vertex);
    graph_manager->addVertex(new_vertex);
    ++rep.num_vertices_added;
    rep.vertex_added = new_vertex;
    graph_manager->addEdge(new_vertex, nearest_vertex, direction_norm);
    ++rep.num_edges_added;
    if (planning_params_.rr_mode == RRModeType::kGraph) {
      std::vector<Vertex*> nearest_vertices;
      if (!graph_manager->getNearestVertices(
              &new_state, planning_params_.nearest_range, &nearest_vertices)) {
        rep.status = ExpandGraphStatus::kErrorKdTree;
        return;
      }
      origin << new_vertex->state[0], new_vertex->state[1],
          new_vertex->state[2];
      for (int i = 0; i < nearest_vertices.size(); ++i) {
        direction << nearest_vertices[i]->state[0] - origin[0],
            nearest_vertices[i]->state[1] - origin[1],
            nearest_vertices[i]->state[2] - origin[2];
        double d_norm = direction.norm();

        if ((d_norm > planning_params_.nearest_range_min) &&
            (d_norm < planning_params_.nearest_range_max)) {
          Eigen::Vector3d p_overshoot =
              direction / d_norm * planning_params_.edge_overshoot;
          Eigen::Vector3d p_start =
              origin + robot_params_.center_offset - p_overshoot;
          Eigen::Vector3d p_end =
              origin + robot_params_.center_offset + direction;
          if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;

          bool geofence_pass = true;
          if (planning_params_.geofence_checking_enable &&
              (GeofenceManager::CoordinateStatus::kViolated ==
               geofence_manager_->getPathStatus(
                   Eigen::Vector2d(p_start[0], p_start[1]),
                   Eigen::Vector2d(p_end[0], p_end[1]),
                   Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
            geofence_pass = false;
          }

          if (geofence_pass) {
            admissible_edge = false;
            VoxelStatus vs = voxel_map_->getPathStatus(
                p_start, p_end, robot_box_size_, true);
            if (VoxelStatus::kFree == vs) {
              admissible_edge = true;
            }
            if (admissible_edge) {
              graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
              ++rep.num_edges_added;
            }
          }
        }
      }
    }

  } else {
    stat_->num_edges_fail++;
    if (stat_->num_edges_fail < 500) {
      std::vector<double> vtmp = {start_pos[0], start_pos[1], start_pos[2],
                                  end_pos[0],   end_pos[1],   end_pos[2]};
      stat_->edges_fail.push_back(vtmp);
    }
    rep.status = ExpandGraphStatus::kErrorCollisionEdge;
    return;
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

void LocalExplorationPlanner::expandGraphEdges(std::shared_ptr<CognitiveGraph> graph_manager,
                           Vertex* new_vertex, ExpandGraphReport& rep) {
  std::vector<Vertex*> nearest_vertices;
  if (!graph_manager->getNearestVertices(&(new_vertex->state),
                                         planning_params_.nearest_range,
                                         &nearest_vertices)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  Eigen::Vector3d origin;
  origin << new_vertex->state[0], new_vertex->state[1], new_vertex->state[2];
  for (int i = 0; i < nearest_vertices.size(); ++i) {
    Eigen::Vector3d direction;
    direction << nearest_vertices[i]->state[0] - origin[0],
        nearest_vertices[i]->state[1] - origin[1],
        nearest_vertices[i]->state[2] - origin[2];
    double d_norm = direction.norm();
    if ((d_norm > planning_params_.edge_length_min) &&
        (d_norm < planning_params_.edge_length_max)) {
      Eigen::Vector3d p_overshoot =
          direction / d_norm * planning_params_.edge_overshoot;
      Eigen::Vector3d p_start =
          origin + robot_params_.center_offset - p_overshoot;
      Eigen::Vector3d p_end = origin + robot_params_.center_offset + direction;
      if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;

      bool admissible_edge = false;
      VoxelStatus vs =
          voxel_map_->getPathStatus(p_start, p_end, robot_box_size_, false);
      if (VoxelStatus::kFree == vs) {
        admissible_edge = true;
      }
      if (admissible_edge) {
        graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
        ++rep.num_edges_added;
      }
    }
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

void LocalExplorationPlanner::expandGraph(std::shared_ptr<CognitiveGraph> graph_manager,
                      Vertex& new_vertex, ExpandGraphReport& rep,
                      bool allow_short_edge) {
  StateVec new_state;
  new_state = new_vertex.state;

  Vertex* nearest_vertex = NULL;
  if (!graph_manager->getNearestVertex(&new_state, &nearest_vertex)) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  if (nearest_vertex == NULL) {
    rep.status = ExpandGraphStatus::kErrorKdTree;
    return;
  }
  Eigen::Vector3d origin(nearest_vertex->state[0], nearest_vertex->state[1],
                         nearest_vertex->state[2]);
  Eigen::Vector3d direction(new_state[0] - origin[0], new_state[1] - origin[1],
                            new_state[2] - origin[2]);
  double direction_norm = direction.norm();

  if (direction_norm > planning_params_.edge_length_max) {
    direction = planning_params_.edge_length_max * direction.normalized();
  } else if ((!allow_short_edge) &&
             (direction_norm <= planning_params_.edge_length_min)) {
    rep.status = ExpandGraphStatus::kErrorShortEdge;
    return;
  }
  direction_norm = direction.norm();
  new_state[0] = origin[0] + direction[0];
  new_state[1] = origin[1] + direction[1];
  new_state[2] = origin[2] + direction[2];

  Eigen::Vector3d overshoot_vec =
      planning_params_.edge_overshoot * direction.normalized();
  Eigen::Vector3d start_pos = origin + robot_params_.center_offset;
  if (nearest_vertex->id != 0) start_pos = start_pos - overshoot_vec;
  Eigen::Vector3d end_pos =
      origin + robot_params_.center_offset + direction + overshoot_vec;
  if (planning_params_.geofence_checking_enable &&
      (GeofenceManager::CoordinateStatus::kViolated ==
       geofence_manager_->getPathStatus(
           Eigen::Vector2d(start_pos[0], start_pos[1]),
           Eigen::Vector2d(end_pos[0], end_pos[1]),
           Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
    rep.status = ExpandGraphStatus::kErrorGeofenceViolated;
    return;
  }

  bool admissible_edge = false;
  VoxelStatus vs =
      voxel_map_->getPathStatus(start_pos, end_pos, robot_box_size_, true);
  if (VoxelStatus::kFree == vs) {
    admissible_edge = true;
  }

  if (admissible_edge) {
    Vertex* new_vertex_ptr =
        new Vertex(graph_manager->generateVertexID(), new_state);
    new_vertex_ptr->state = new_state;
    new_vertex_ptr->parent = nearest_vertex;
    new_vertex_ptr->distance = nearest_vertex->distance + direction_norm;
    new_vertex_ptr->is_hanging = new_vertex.is_hanging;
    nearest_vertex->children.push_back(new_vertex_ptr);
    graph_manager->addVertex(new_vertex_ptr);
    ++rep.num_vertices_added;
    rep.vertex_added = new_vertex_ptr;
    graph_manager->addEdge(new_vertex_ptr, nearest_vertex, direction_norm);
    ++rep.num_edges_added;
    if (planning_params_.rr_mode == RRModeType::kGraph) {
      std::vector<Vertex*> nearest_vertices;
      if (!graph_manager->getNearestVertices(
              &new_state, planning_params_.nearest_range, &nearest_vertices)) {
        rep.status = ExpandGraphStatus::kErrorKdTree;
        return;
      }
      origin << new_vertex_ptr->state[0], new_vertex_ptr->state[1],
          new_vertex_ptr->state[2];
      for (int i = 0; i < nearest_vertices.size(); ++i) {
        direction << nearest_vertices[i]->state[0] - origin[0],
            nearest_vertices[i]->state[1] - origin[1],
            nearest_vertices[i]->state[2] - origin[2];
        double d_norm = direction.norm();

        if ((d_norm > planning_params_.nearest_range_min) &&
            (d_norm < planning_params_.nearest_range_max)) {
          Eigen::Vector3d p_overshoot =
              direction / d_norm * planning_params_.edge_overshoot;
          Eigen::Vector3d p_start =
              origin + robot_params_.center_offset - p_overshoot;
          Eigen::Vector3d p_end =
              origin + robot_params_.center_offset + direction;
          if (nearest_vertices[i]->id != 0) p_end = p_end + p_overshoot;

          bool geofence_pass = true;
          if (planning_params_.geofence_checking_enable &&
              (GeofenceManager::CoordinateStatus::kViolated ==
               geofence_manager_->getPathStatus(
                   Eigen::Vector2d(p_start[0], p_start[1]),
                   Eigen::Vector2d(p_end[0], p_end[1]),
                   Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) {
            geofence_pass = false;
          }

          if (geofence_pass) {
            admissible_edge = false;
            VoxelStatus vs = voxel_map_->getPathStatus(
                p_start, p_end, robot_box_size_, true);
            if (VoxelStatus::kFree == vs) {
              admissible_edge = true;
            }

            if (admissible_edge) {
              graph_manager->addEdge(new_vertex_ptr, nearest_vertices[i],
                                     d_norm);
              ++rep.num_edges_added;
            }
          }
        }
      }
    }

  } else {
    stat_->num_edges_fail++;
    if (stat_->num_edges_fail < 500) {
      std::vector<double> vtmp = {start_pos[0], start_pos[1], start_pos[2],
                                  end_pos[0],   end_pos[1],   end_pos[2]};
      stat_->edges_fail.push_back(vtmp);
    }
    rep.status = ExpandGraphStatus::kErrorCollisionEdge;
    return;
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

void LocalExplorationPlanner::expandGraphEdgesBlindly(std::shared_ptr<CognitiveGraph> graph_manager,
                                  Vertex* new_vertex, double radius,
                                  ExpandGraphReport& rep) {
  std::vector<Vertex*> nearest_vertices;
  if (!graph_manager->getNearestVertices(&(new_vertex->state), radius,
                                         &nearest_vertices)) {
    rep.status = ExpandGraphStatus::kNull;
    return;
  }
  Eigen::Vector3d origin;
  origin << new_vertex->state[0], new_vertex->state[1], new_vertex->state[2];
  for (int i = 0; i < nearest_vertices.size(); ++i) {
    Eigen::Vector3d direction;
    direction << nearest_vertices[i]->state[0] - origin[0],
        nearest_vertices[i]->state[1] - origin[1],
        nearest_vertices[i]->state[2] - origin[2];
    double d_norm = direction.norm();
    graph_manager->addEdge(new_vertex, nearest_vertices[i], d_norm);
    ++rep.num_edges_added;
  }
  rep.status = ExpandGraphStatus::kSuccess;
}

void LocalExplorationPlanner::expandGraphEdgesBatch(std::shared_ptr<CognitiveGraph> graph_manager,
                        std::vector<Vertex> sampled_vertices)
{
  /*
  Vertices in sampled_vertices are not added to graph_manager
  */
  std::shared_ptr<CognitiveGraph> temp_graph_manager;
  std::map<int, int> temp_to_final;
  temp_graph_manager.reset(new CognitiveGraph());
  std::map<int, int> vertex_ids;
  Vertex* root_vert =
        new Vertex(temp_graph_manager->generateVertexID(), graph_manager->getVertex(0)->state);
  temp_graph_manager->addVertex(root_vert);
  int first_new_vertex_id = temp_graph_manager->vertices_map_.size();
  temp_to_final[0] = 0;

  for(int i=0; i<sampled_vertices.size(); ++i)
  {
    Vertex v = sampled_vertices[i];
    Vertex* new_vert =
          new Vertex(temp_graph_manager->generateVertexID(), v.state);
    temp_graph_manager->addVertex(new_vert);
    vertex_ids[new_vert->id] = i;
  }

  std::vector<std::pair<int, int>> edges_to_check;
  for(auto v_it : temp_graph_manager->vertices_map_)
  {
    v_it.second->is_checked = true;
    std::vector<Vertex*> nearest_vertices;
    if (!temp_graph_manager->getNearestVertices(
            &v_it.second->state, planning_params_.nearest_range, &nearest_vertices)) {
      continue;
    }

    for(auto nv : nearest_vertices)
    {
      if(nv->id == v_it.first) continue;

      if(!nv->is_checked)
      {
        edges_to_check.push_back(std::make_pair(nv->id, v_it.first));
      }
    }
    if(edges_to_check.size() > planning_params_.num_edges_max)
    {
      break;
    }
  }

  for(auto v_it : temp_graph_manager->vertices_map_)
    v_it.second->is_checked = false;

  std::vector<VoxelStatus> edge_statuses;
  edge_statuses.reserve(edges_to_check.size());
  int free_e = 0, occ_e = 0;
  VoxelStatus vs = VoxelStatus::kUnknown;
  for(int i=0; i<edges_to_check.size(); ++i)
  {
    vs = voxel_map_->getPathStatus(
        temp_graph_manager->getVertex(edges_to_check[i].first)->state.head(3),
        temp_graph_manager->getVertex(edges_to_check[i].second)->state.head(3),
        robot_box_size_, true);
    edge_statuses.push_back(vs);
    if (vs == VoxelStatus::kFree)
    {
      ++free_e;
      temp_graph_manager->addEdge(temp_graph_manager->getVertex(edges_to_check[i].first)
        , temp_graph_manager->getVertex(edges_to_check[i].second)
        , (temp_graph_manager->getVertex(edges_to_check[i].first)->state.head(3) - temp_graph_manager->getVertex(edges_to_check[i].second)->state.head(3)).norm());
    }
    else
    {
      ++occ_e;
    }
  }

  ShortestPathsReport paths_rep;
  temp_graph_manager->findShortestPaths(0, paths_rep);
  for(auto d_it : paths_rep.distance_map)
  {
    if(d_it.second < std::numeric_limits<double>::max() )
    {
      temp_graph_manager->getVertex(d_it.first)->is_checked = true;
      if(d_it.first >= first_new_vertex_id)
      {
        Vertex* new_vert =
              new Vertex(graph_manager->generateVertexID(), temp_graph_manager->getVertex(d_it.first)->state);
        graph_manager->addVertex(new_vert);
        temp_to_final[d_it.first] = new_vert->id;
      }
    }
    else
    {
      temp_graph_manager->getVertex(d_it.first)->is_checked = false;
    }
  }
  for(int i=0; i<edges_to_check.size(); ++i)
  {
    if(edge_statuses[i] == VoxelStatus::kFree
       && temp_graph_manager->getVertex(edges_to_check[i].first)->is_checked
       && temp_graph_manager->getVertex(edges_to_check[i].second)->is_checked)
    {
      graph_manager->addEdge(graph_manager->getVertex(temp_to_final[edges_to_check[i].first]), 
        graph_manager->getVertex(temp_to_final[edges_to_check[i].second]),
        (graph_manager->getVertex(temp_to_final[edges_to_check[i].first])->state.head(3) 
        - graph_manager->getVertex(temp_to_final[edges_to_check[i].second])->state.head(3)).norm());
    }
  }

}

LocalExplorationPlanner::GraphStatus LocalExplorationPlanner::batchGraph(){
  int loop_count = 0;
  int num_vertices = 1;
  int num_edges = 0;
  std::vector<RandomSamplerBase::RandomDistributionType> init_pdf_type;

  local_exploration_ongoing_ = true;

  if (global_exploration_ongoing_) {
    Vertex* global_vertex = global_graph_->getVertex(current_global_vertex_id_);
    if ((current_state_.head(3) - global_vertex->state.head(3)).norm() > 5.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Global frontier not reached. Triggering global planner again");
      local_exploration_ongoing_ = false;
      return GraphStatus::NOT_OK;
    }
    else {
      global_exploration_ongoing_ = false;
    }
  }

  if(planning_params_.only_opening_traversal) {
    local_exploration_ongoing_ = false;
    return GraphStatus::OK;
  }

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    Eigen::Vector3d min_val, max_val, rotations, mean_val, std_val;
    Eigen::Vector3d pos = root_vertex_->state.head(3);
    min_val = adaptive_orig_min_val_;
    max_val = adaptive_orig_max_val_;
    repositioning_region_->constructBoundingBox(pos, min_val, max_val, rotations,
                                        mean_val, std_val);
    local_adaptive_params_.setBound(min_val, max_val);
    local_adaptive_params_.setRotation(rotations);
    random_sampler_adaptive_.setBound(min_val, max_val);
    random_sampler_adaptive_.setRotation(rotations);
    random_sampler_adaptive_.setSTD();
    random_sampler_adaptive_.reset();
    init_pdf_type = random_sampler_adaptive_.getInitPDF();
    for(int i=0;i<4;i++){
      if (init_pdf_type[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
        random_sampler_adaptive_.setPDF(RandomSamplerBase::RandomDistributionType::kNormal,i); 
      }
    }
  } else {
    random_sampler_.reset();
    init_pdf_type = random_sampler_.getInitPDF();
    for(int i=0;i<4;i++){
      if (init_pdf_type[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
        random_sampler_.setPDF(RandomSamplerBase::RandomDistributionType::kNormal,i); 
      }
    }
  }

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_adaptive_params_);
  } else {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_space_params_);
  }

  START_TIMER(ttime);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
  std::vector<Vertex> samples;
  while ((loop_count++ < planning_params_.num_loops_max) &&
         (num_vertices <= planning_num_vertices_max_)) {
    Vertex new_vertex(-1, StateVec::Zero());
    
    if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
      if (num_vertices == planning_num_vertices_max_/2){
        for(int i=0;i<4;i++){
          if (init_pdf_type[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
            random_sampler_adaptive_.setPDF(RandomSamplerBase::RandomDistributionType::kUniform,i); 
          }
        }
      }
      if (!sampleVertex(random_sampler_adaptive_, root_vertex_->state,
                        new_vertex, true)) {
        continue;
      }
    } else {
        if (num_vertices == planning_num_vertices_max_/2){
          for(int i=0;i<4;i++){
            if (init_pdf_type[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
              random_sampler_.setPDF(RandomSamplerBase::RandomDistributionType::kUniform,i); 
            }
          }
        }
        if (!sampleVertex(new_vertex)) {
          continue;
        }
    }
    if(planning_params_.limit_vertices_to_surface)
    {
      double vertex_distance = voxel_map_->getPointDistance(new_vertex.state.head(3));
      if(vertex_distance > planning_params_.max_surface_distance)
      {
        if ((loop_count >= planning_params_.num_loops_cutoff) &&
            (local_graph_->getNumVertices() <= 1)) {
          break;
        }
        continue;
      }
    }

    samples.push_back(new_vertex);
    ++num_vertices;
  }
  
  //Build the graph
  ExpandGraphReport rep;
  expandGraphEdgesBatch(local_graph_, samples);
  num_edges = local_graph_->getNumEdges();
  num_vertices = local_graph_->getNumVertices();

  stat_->build_graph_time = GET_ELAPSED_TIME(ttime);
  t2 = std::chrono::high_resolution_clock::now();

  std::shared_ptr<Graph> g = local_graph_->graph_;

  stat_chrono_->build_graph_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
  if (planning_params_.geofence_checking_enable)
    visualization_->visualizeGeofence(geofence_manager_);

  planner_trigger_count_++;
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Formed a graph with [%d] vertices and [%d] edges",
           num_vertices, num_edges);

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration)
    visualization_->visualizeSampler(random_sampler_adaptive_);
  else
    visualization_->visualizeSampler(random_sampler_);

  local_exploration_ongoing_ = false;

  if (local_graph_->getNumVertices() > 1) {
    visualization_->visualizeGraph(local_graph_);
    return LocalExplorationPlanner::GraphStatus::OK;
  } else {
    const double startup_reach = std::max(
        planning_params_.edge_length_min,
        std::min(planning_params_.edge_length_max,
                 std::max(planning_params_.nearest_range_min,
                          planning_params_.nearest_range)));
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "Root-only batch graph detected, augmenting startup free space.");
    augmentFreeStartupArea(root_vertex_->state, startup_reach);
    visualization_->visualizeFailedEdges(stat_);
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Number of failed samples: [%d] vertices and [%d] edges",
             stat_->num_vertices_fail, stat_->num_edges_fail);
    return LocalExplorationPlanner::GraphStatus::ERR_NO_FEASIBLE_PATH;
  }
}

LocalExplorationPlanner::GraphStatus LocalExplorationPlanner::buildGraph() {
  int loop_count = 0;
  int num_vertices = 1;
  int num_edges = 0;
  std::vector<RandomSamplerBase::RandomDistributionType> curr_pdf_type;
  std::vector<RandomSamplerBase::RandomDistributionType> init_pdf_type;

  local_exploration_ongoing_ = true;

  if (global_exploration_ongoing_) {
    Vertex* global_vertex = global_graph_->getVertex(current_global_vertex_id_);
    if ((current_state_.head(3) - global_vertex->state.head(3)).norm() > 5.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Global frontier not reached. Triggering global planner again");
      local_exploration_ongoing_ = false;
      return GraphStatus::NOT_OK;
    }
    else {
      global_exploration_ongoing_ = false;
    }
  }

  if(planning_params_.only_opening_traversal) {
    local_exploration_ongoing_ = false;
    return GraphStatus::OK;
  }

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    Eigen::Vector3d min_val, max_val, rotations, mean_val, std_val;
    Eigen::Vector3d pos = root_vertex_->state.head(3);
    min_val = adaptive_orig_min_val_;
    max_val = adaptive_orig_max_val_;
    repositioning_region_->constructBoundingBox(pos, min_val, max_val, rotations,
                                        mean_val, std_val);
    local_adaptive_params_.setBound(min_val, max_val);
    local_adaptive_params_.setRotation(rotations);
    random_sampler_adaptive_.setBound(min_val, max_val);
    random_sampler_adaptive_.setRotation(rotations);
    random_sampler_adaptive_.setSTD();
    random_sampler_adaptive_.reset();
    init_pdf_type = random_sampler_adaptive_.getInitPDF();
    for(int i=0;i<4;i++){
      if (init_pdf_type[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
        random_sampler_adaptive_.setPDF(RandomSamplerBase::RandomDistributionType::kNormal,i); 
      }
    }
  } else {
    random_sampler_.reset();

    init_pdf_type = random_sampler_.getInitPDF();
    for(int i=0;i<4;i++){
      if (init_pdf_type[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
        random_sampler_.setPDF(RandomSamplerBase::RandomDistributionType::kNormal,i); 
      }
    }
  }

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_adaptive_params_);
  } else {
    visualization_->visualizeWorkspace(
        root_vertex_->state, global_space_params_, local_space_params_);
  }

  START_TIMER(ttime);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;


  bool use_edge_limit = true;
  for(int i=0;i<4;i++){
    if (init_pdf_type[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
      use_edge_limit = false;
      break;
    }
  }
  
  while ((loop_count++ < planning_params_.num_loops_max) &&
         (num_vertices < planning_num_vertices_max_) && 
         (!use_edge_limit || num_edges < planning_num_edges_max_)) {
    Vertex new_vertex(-1, StateVec::Zero());
    
    if (planning_params_.type == PlanningModeType::kAdaptiveExploration) {
      if (num_vertices == planning_num_vertices_max_/2){
        for(int i=0;i<4;i++){
          if (init_pdf_type[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
            random_sampler_adaptive_.setPDF(RandomSamplerBase::RandomDistributionType::kUniform,i); 
          }
        }
      }
      if (!sampleVertex(random_sampler_adaptive_, root_vertex_->state,
                        new_vertex, true)) {
        continue;
      }
    } else {
        if (num_vertices == planning_num_vertices_max_/2){
          for(int i=0;i<4;i++){
            if (init_pdf_type[i] == RandomSamplerBase::RandomDistributionType::kNormalUniform){
              random_sampler_.setPDF(RandomSamplerBase::RandomDistributionType::kUniform,i); 
            }
          }
        }
        if (!sampleVertex(new_vertex)) {
          continue;
        }
    }

    if(planning_params_.limit_vertices_to_surface)
    {
      double vertex_distance = voxel_map_->getPointDistance(new_vertex.state.head(3));
      if(vertex_distance > planning_params_.max_surface_distance)
      {
        if ((loop_count >= planning_params_.num_loops_cutoff) &&
            (local_graph_->getNumVertices() <= 1)) {
          break;
        }
        continue;
      }
    }
      
    ExpandGraphReport rep;
    expandGraph(local_graph_, new_vertex, rep);
    if (rep.status == ExpandGraphStatus::kSuccess) {
      num_vertices += rep.num_vertices_added;
      num_edges += rep.num_edges_added;
    }

    if ((loop_count >= planning_params_.num_loops_cutoff) &&
        (local_graph_->getNumVertices() <= 1)) {
      break;
    }
  }

  stat_->build_graph_time = GET_ELAPSED_TIME(ttime);
  t2 = std::chrono::high_resolution_clock::now();

  std::shared_ptr<Graph> g = local_graph_->graph_;

  stat_chrono_->build_graph_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
  if (planning_params_.geofence_checking_enable)
    visualization_->visualizeGeofence(geofence_manager_);

  planner_trigger_count_++;
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Formed a graph with [%d] vertices and [%d] edges with [%d] loops",
           num_vertices, num_edges, loop_count);

  if (planning_params_.type == PlanningModeType::kAdaptiveExploration)
    visualization_->visualizeSampler(random_sampler_adaptive_);
  else
    visualization_->visualizeSampler(random_sampler_);

  local_exploration_ongoing_ = false;

  if (local_graph_->getNumVertices() > 1) {
    visualization_->visualizeGraph(local_graph_);
    return LocalExplorationPlanner::GraphStatus::OK;
  } else {
    const double startup_reach = std::max(
        planning_params_.edge_length_min,
        std::min(planning_params_.edge_length_max,
                 std::max(planning_params_.nearest_range_min,
                          planning_params_.nearest_range)));
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                  "Root-only graph detected, augmenting startup free space.");
    augmentFreeStartupArea(root_vertex_->state, startup_reach);
    visualization_->visualizeFailedEdges(stat_);
    ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Number of failed samples: [%d] vertices and [%d] edges",
             stat_->num_vertices_fail, stat_->num_edges_fail);
    return LocalExplorationPlanner::GraphStatus::ERR_NO_FEASIBLE_PATH;
  }
}

LocalExplorationPlanner::GraphStatus LocalExplorationPlanner::buildGridGraph(StateVec state, Eigen::Vector3d robot_size,
                                     Eigen::Vector3d grid_min,
                                     Eigen::Vector3d grid_max,
                                     Eigen::Vector3d grid_res, double heading) {

  if ((grid_min[0] > 0.0) || (grid_min[1] > 0.0) || (grid_min[2] > 0.0)) {
    return GraphStatus::NOT_OK;
  }

  if ((grid_max[0] < 0.0) || (grid_max[1] < 0.0) || (grid_max[2] < 0.0)) {
    return GraphStatus::NOT_OK;
  }

  if ((grid_res[0] == 0.0) || (grid_res[1] == 0.0) || (grid_res[2] == 0.0)) {
    return GraphStatus::NOT_OK;
  }
  int num_nodes[3];
  int root_node_ind[3];
  for (int i = 0; i < 3; ++i) {
    grid_min[i] = -grid_res[i] * std::ceil(-grid_min[i] / grid_res[i]);
    grid_max[i] = grid_res[i] * std::ceil(grid_max[i] / grid_res[i]);
    num_nodes[i] = (int)((grid_max[i] - grid_min[i]) / grid_res[i]) + 1;
    if (num_nodes[i] == 0) num_nodes[i] = 1;
    root_node_ind[i] = (int)(-grid_min[i] / grid_res[i]);
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Number of nodes [%d][%d][%d].", num_nodes[0], num_nodes[1],
           num_nodes[2]);

  int num_total_nodes = num_nodes[0] * num_nodes[1] * num_nodes[2];
  Vertex** vertices_mat = new Vertex*[num_total_nodes];
  bool* collision_free_mat = new bool[num_total_nodes];

  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      for (int k = 0; k < num_nodes[2]; ++k) {
        int ind = i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
        collision_free_mat[ind] = false;
      }
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Initialized matrices.");

  double cos_h = 1.0, sin_h = 0.0;
  if (heading) {
    cos_h = std::cos(heading);
    sin_h = std::sin(heading);
  }
  int root_node_ind_arr = root_node_ind[0] * num_nodes[1] * num_nodes[2] +
                          root_node_ind[1] * num_nodes[2] + root_node_ind[2];

  int free_vertex_count = 0;
  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      for (int k = 0; k < num_nodes[2]; ++k) {
        int cur_ind = i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
        if (cur_ind == root_node_ind_arr) {
          collision_free_mat[cur_ind] = true;
          vertices_mat[cur_ind] = root_vertex_;
          continue;
        }

        double x_val = grid_min.x() + i * grid_res.x();
        double y_val = grid_min.y() + j * grid_res.y();
        double z_val = grid_min.z() + k * grid_res.z();
        if (heading) {
          double x_val_, y_val_;
          x_val_ = x_val * cos_h - y_val * sin_h;
          y_val_ = x_val * sin_h + y_val * cos_h;
          x_val = x_val_;
          y_val = y_val_;
        }

        x_val += state.x();
        y_val += state.y();
        z_val += state.z();

        StateVec new_state;
        new_state << x_val, y_val, z_val, heading, 0.0;
        Vertex* new_vertex =
            new Vertex(local_graph_->generateVertexID(), new_state);
        local_graph_->addVertex(new_vertex);
        vertices_mat[cur_ind] = new_vertex;

        Eigen::Vector3d voxel(x_val, y_val, z_val);
        if (VoxelStatus::kFree ==
            voxel_map_->getBoxStatus(voxel, robot_size, true)) {
          collision_free_mat[cur_ind] = true;
          free_vertex_count++;
        }
      }
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Number of free nodes [%d].", free_vertex_count);
  double map_res = voxel_map_->getResolution();
  double dx_len = grid_res.x();
  double dy_len = grid_res.y();
  double dz_len = grid_res.z();
  int free_edge_count = 0;
  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      for (int k = 0; k < num_nodes[2]; ++k) {
        int vertex_ind = i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
        if (!collision_free_mat[vertex_ind]) continue;

        Eigen::Vector3d start(vertices_mat[vertex_ind]->state.x(),
                              vertices_mat[vertex_ind]->state.y(),
                              vertices_mat[vertex_ind]->state.z());
        Eigen::Vector3d end;
        if (i < (num_nodes[0] - 1)) {
          int vertex_ind_x =
              (i + 1) * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
          if (collision_free_mat[vertex_ind_x]) {
            end << vertices_mat[vertex_ind_x]->state.x(),
                vertices_mat[vertex_ind_x]->state.y(),
                vertices_mat[vertex_ind_x]->state.z();
            if ((dx_len <= map_res) ||
                (VoxelStatus::kFree ==
                 voxel_map_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind],
                                    vertices_mat[vertex_ind_x], dx_len);
            }
          }
        }
        if (j < (num_nodes[1] - 1)) {
          int vertex_ind_y =
              i * num_nodes[1] * num_nodes[2] + (j + 1) * num_nodes[2] + k;
          if (collision_free_mat[vertex_ind_y]) {
            end << vertices_mat[vertex_ind_y]->state.x(),
                vertices_mat[vertex_ind_y]->state.y(),
                vertices_mat[vertex_ind_y]->state.z();
            if ((dy_len <= map_res) ||
                (VoxelStatus::kFree ==
                 voxel_map_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind],
                                    vertices_mat[vertex_ind_y], dy_len);
            }
          }
        }
        if (k < (num_nodes[2] - 1)) {
          int vertex_ind_z =
              i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k + 1;
          if (collision_free_mat[vertex_ind_z]) {
            end << vertices_mat[vertex_ind_z]->state.x(),
                vertices_mat[vertex_ind_z]->state.y(),
                vertices_mat[vertex_ind_z]->state.z();
            if ((dz_len <= map_res) ||
                (VoxelStatus::kFree ==
                 voxel_map_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind],
                                    vertices_mat[vertex_ind_z], dz_len);
            }
          }
        }
      }
    }
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Free edges: [%d]", free_edge_count);
  double diag_len = std::sqrt(dx_len * dx_len + dy_len * dy_len);
  for (int i = 0; i < num_nodes[0]; ++i) {
    for (int j = 0; j < num_nodes[1]; ++j) {
      for (int k = 0; k < num_nodes[2]; ++k) {
        if ((i < (num_nodes[0] - 1)) && (j < (num_nodes[1] - 1))) {
          int vertex_ind_d0 =
              i * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
          int vertex_ind_d2 = (i + 1) * num_nodes[1] * num_nodes[2] +
                              (j + 1) * num_nodes[2] + k;
          if (collision_free_mat[vertex_ind_d0] &&
              collision_free_mat[vertex_ind_d2]) {
            Eigen::Vector3d start(vertices_mat[vertex_ind_d0]->state.x(),
                                  vertices_mat[vertex_ind_d0]->state.y(),
                                  vertices_mat[vertex_ind_d0]->state.z());
            Eigen::Vector3d end(vertices_mat[vertex_ind_d2]->state.x(),
                                vertices_mat[vertex_ind_d2]->state.y(),
                                vertices_mat[vertex_ind_d2]->state.z());
            if ((diag_len <= map_res) ||
                (VoxelStatus::kFree ==
                 voxel_map_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind_d0],
                                    vertices_mat[vertex_ind_d2], diag_len);
            }
          }
        }

        if ((i < (num_nodes[0] - 1)) && (j < (num_nodes[1] - 1))) {
          int vertex_ind_d1 =
              i * num_nodes[1] * num_nodes[2] + (j + 1) * num_nodes[2] + k;
          int vertex_ind_d3 =
              (i + 1) * num_nodes[1] * num_nodes[2] + j * num_nodes[2] + k;
          if (collision_free_mat[vertex_ind_d1] &&
              collision_free_mat[vertex_ind_d3]) {
            Eigen::Vector3d start(vertices_mat[vertex_ind_d1]->state.x(),
                                  vertices_mat[vertex_ind_d1]->state.y(),
                                  vertices_mat[vertex_ind_d1]->state.z());
            Eigen::Vector3d end(vertices_mat[vertex_ind_d3]->state.x(),
                                vertices_mat[vertex_ind_d3]->state.y(),
                                vertices_mat[vertex_ind_d3]->state.z());
            if ((diag_len <= map_res) ||
                (VoxelStatus::kFree ==
                 voxel_map_->getPathStatus(start, end, robot_size, true))) {
              free_edge_count++;
              local_graph_->addEdge(vertices_mat[vertex_ind_d1],
                                    vertices_mat[vertex_ind_d3], diag_len);
            }
          }
        }
      }
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Grid graph: %d vertices, %d edges", local_graph_->getNumVertices(),
           local_graph_->getNumEdges());
  return GraphStatus::OK;
}

void LocalExplorationPlanner::correctYaw() {
  if (planning_params_.yaw_tangent_correction) {
    int num_vertices = local_graph_->getNumVertices();
    for (int id = 1; id < num_vertices; ++id) {
      int pid = local_graph_->getParentIDFromShortestPath(id, local_graph_rep_);
      Vertex* v = local_graph_->getVertex(id);
      Vertex* vp = local_graph_->getVertex(pid);
      Eigen::Vector3d vec(v->state[0] - vp->state[0],
                          v->state[1] - vp->state[1],
                          v->state[2] - vp->state[2]);
      if (planning_params_.planning_backward) vec = -vec;
      v->state[3] = std::atan2(vec[1], vec[0]);
    }
  }
  else {
    if(planning_params_.keep_leaf_yaw_only) {
			int num_vertices = local_graph_->getNumVertices();
			for (int id = 1; id < num_vertices; ++id) {
				int pid = local_graph_->getParentIDFromShortestPath(id, local_graph_rep_);
				Vertex* v = local_graph_->getVertex(id);
				if(v->is_leaf_vertex) {
					continue;
				}
				Vertex* vp = local_graph_->getVertex(pid);
				Eigen::Vector3d vec(v->state[0] - vp->state[0],
														v->state[1] - vp->state[1],
														v->state[2] - vp->state[2]);
				if (planning_params_.planning_backward) vec = -vec;
				v->state[3] = std::atan2(vec[1], vec[0]);
			}
		}
  }
}

LocalExplorationPlanner::GraphStatus LocalExplorationPlanner::evaluateGraph() {
  LocalExplorationPlanner::GraphStatus gstatus = LocalExplorationPlanner::GraphStatus::OK;

  if(planning_params_.only_opening_traversal) {
    auto_global_planner_trig_ = true;
    return LocalExplorationPlanner::GraphStatus::NOT_OK;
  }

  START_TIMER(ttime);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
  local_graph_->findShortestPaths(local_graph_rep_);
  local_graph_->findLeafVertices(local_graph_rep_);
  std::vector<Vertex*> leaf_vertices;
  local_graph_->getLeafVertices(leaf_vertices);
  stat_->shortest_path_time = GET_ELAPSED_TIME(ttime);
  t2 = std::chrono::high_resolution_clock::now();
  stat_chrono_->shortest_path_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();

  correctYaw();
  computeExplorationGain(planning_params_.leafs_only_for_volumetric_gain,
                         planning_params_.cluster_vertices_for_gain);

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Evaluate graph]: Gain Calculation Done");
  add_frontiers_to_global_graph_ = true;
  START_TIMER(ttime);
  t1 = std::chrono::high_resolution_clock::now();
  double best_gain = 0.0;
  int best_path_id = 0;
  bool frontier_exists = false;
  std::vector<Eigen::Vector3d> inadmissible_negative_edges;
  int num_leaf_vertices = leaf_vertices.size();
  int path_len_count = 0;
  int path_dir_count = 0;
  double path_len_time = 0.0;
  double path_dir_time = 0.0;

  auto evaluate_leaf_paths = [&]() {
    best_gain = 0.0;
    best_path_id = 0;
    frontier_exists = false;
    inadmissible_negative_edges.clear();
    path_len_count = 0;
    path_dir_count = 0;
    path_len_time = 0.0;
    path_dir_time = 0.0;

    for (int i = 0; i < num_leaf_vertices; ++i) {
      int id = leaf_vertices[i]->id;
      std::vector<Vertex*> path;
      local_graph_->getShortestPath(id, local_graph_rep_, true, path);
      int path_size = path.size();
      if (path_size <= 1) {
        continue;
      }

      double path_gain = 0.0;
      double lambda = planning_params_.path_length_penalty;
      bool inadmissible_edge = false;
      Timer path_timer;
      for (int ind = 0; ind < path_size; ++ind) {
        Vertex* v_id = path[ind];
        double path_length =
            local_graph_->getShortestDistance(v_id->id, local_graph_rep_);
        double vol_gain =
            v_id->vol_gain.gain *
            exp(-v_id->is_hanging * planning_params_.hanging_vertex_penalty);

        if (!inadmissible_edge) {
          path_gain += vol_gain * exp(-lambda * path_length);
          v_id->vol_gain.accumulative_gain = path_gain;
          if (v_id->vol_gain.is_frontier && !v_id->is_hanging) {
            frontier_exists = true;
          }
        }
      }
      path_len_time += path_timer.endTimer();
      ++path_len_count;
      if (inadmissible_edge) {
        continue;
      }

      path_timer.reset();
      double lambda2 = planning_params_.path_direction_penalty;
      std::vector<Eigen::Vector3d> path_list;
      local_graph_->getShortestPath(id, local_graph_rep_, true, path_list);
      double fw_ratio =
          Trajectory::computeDistanceBetweenTrajectoryAndDirection(
              path_list, exploring_direction_, 0.2, true);
      path_gain *= exp(-lambda2 * fw_ratio);

      if (path_gain > best_gain) {
        best_gain = path_gain;
        best_path_id = id;
      }
      ++path_dir_count;
      path_dir_time += path_timer.endTimer();
    }
  };

  evaluate_leaf_paths();
  if (last_gain_eval_partial_ && !frontier_exists) {
    force_full_gain_evaluation_ = true;
    computeExplorationGain(planning_params_.leafs_only_for_volumetric_gain,
                           planning_params_.cluster_vertices_for_gain);
    force_full_gain_evaluation_ = false;
    evaluate_leaf_paths();
  }

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Mean path len timer: %f",
                path_len_count > 0 ? path_len_time / path_len_count : 0.0);
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Mean path dir timer: %f",
                path_dir_count > 0 ? path_dir_time / path_dir_count : 0.0);

  Timer tc;
  double dt;

  if (planning_params_.auto_global_planner_enable) {
    if (!frontier_exists || best_gain <= 0) {
      ++num_low_gain_iters_;
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No frontier found in this round. Total rounds: %d",
               num_low_gain_iters_);
    } else {
      if (num_low_gain_iters_ > 0) --num_low_gain_iters_;
    }
    if (num_low_gain_iters_ >= planning_params_.max_num_low_gain_iters) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "%d consecutinve low gain paths, triggering global planner.",
               num_low_gain_iters_);
      dead_zone_escape_pending_ =
          isDeadZoneCandidate(frontier_exists, best_gain, num_leaf_vertices);
      if (dead_zone_escape_pending_) {
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                      "\033[1;35m[Dead-Zone Recovery]\033[0m Local low-gain state is considered a recovery candidate.");
      }
      num_low_gain_iters_ = 0;
      auto_global_planner_trig_ = true;
      return LocalExplorationPlanner::GraphStatus::CONSEC_LOW_GAIN;
    }
  }

  dt = tc.endTimer();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "No frontier check timer: %f", dt);
  tc.reset();
  visualization_->visualizeShortestPaths(local_graph_, local_graph_rep_);
  if (best_gain > 0) {
    visualization_->visualizeBestPaths(local_graph_, local_graph_rep_, 10,
                                       best_path_id);
  }

  visualization_->visualizeNegativePaths(inadmissible_negative_edges,
                                         local_graph_, local_graph_rep_);
  dt = tc.endTimer();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "MissionVisuals timer: %f", dt);

  tc.reset();
  if (best_gain > 0) {
    std::vector<int> path;
    local_graph_->getShortestPath(best_path_id, local_graph_rep_, false, path);
    for (int i = 0; i < (path.size() - 1); ++i) {
      local_graph_->getVertex(path[i])->parent =
          local_graph_->getVertex(path[i + 1]);
    }
    best_vertex_ = local_graph_->getVertex(path[0]);
    //
    ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Best path: with gain [%f] and ID [%d] ", best_gain, best_path_id);
    gstatus = LocalExplorationPlanner::GraphStatus::OK;
  } else {
    gstatus = LocalExplorationPlanner::GraphStatus::NO_GAIN;
  }
  dt = tc.endTimer();
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Best vertex selection timer: %f", dt);

  stat_->evaluate_graph_time = GET_ELAPSED_TIME(ttime);
  t2 = std::chrono::high_resolution_clock::now();
  stat_chrono_->evaluate_graph_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
  stat_chrono_->printTime("Chrono");
  publishTimings(stat_chrono_);
  publishAblationMetrics();

  return gstatus;
}

LocalExplorationPlanner::LocalPlannerStatus LocalExplorationPlanner::evaluateLocalNavigationPath()
{
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
  local_graph_->findShortestPaths(local_graph_rep_);
  local_graph_->findLeafVertices(local_graph_rep_);
  std::vector<Vertex*> leaf_vertices;
  local_graph_->getLeafVertices(leaf_vertices);
  stat_->shortest_path_time = GET_ELAPSED_TIME(ttime);
  t2 = std::chrono::high_resolution_clock::now();
  stat_chrono_->shortest_path_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();

  correctYaw();

  computeExplorationGain(planning_params_.leafs_only_for_volumetric_gain,
                         planning_params_.cluster_vertices_for_gain);

  visualization_->visualizeShortestPaths(local_graph_, local_graph_rep_);
  addFrontiers(0);
  visualization_->visualizeTopologicalGraph(global_graph_);

  t1 = std::chrono::high_resolution_clock::now();
	  double best_gain = 0;
	  int best_path_id = -1;

  if(local_navigation_goal_set_)
  {
    Eigen::Vector3d local_target = local_navigation_goal_;
    int num_frontiers = 0;
    bool use_final_goal = true;
    double goal_dist = (current_state_.head(3) - local_navigation_goal_).norm();

    VoxelStatus vs = voxel_map_->getVoxelStatus(local_navigation_goal_);

    StateVec local_navigation_goal_state;
    local_navigation_goal_state.head(3) = local_navigation_goal_;
    Vertex* nearest_vertex = NULL;
    if(global_graph_->getNearestVertexInRange(
      &local_navigation_goal_state, planning_params_.local_navigation_reaching_radius, &nearest_vertex))
    {  
      Vertex* current_vertex = NULL;
      if(global_graph_->getNearestVertex(&current_state_, &current_vertex))
      {
        ShortestPathsReport global_graph_rep;
        global_graph_->findShortestPaths(current_vertex->id, global_graph_rep);
        std::vector<StateVec> path_along_global_graph;
        global_graph_->getShortestPath(nearest_vertex->id, global_graph_rep, true, path_along_global_graph);

        if(!path_along_global_graph.empty())
        {
          std::vector<geometry_msgs::Pose> path_poses;
          convert(path_along_global_graph, path_poses);
          visualization_->visualizeRefPath(path_poses, 2);
          double dist_along_path = 0.0;
          local_target = path_along_global_graph.back().head(3);
          for(size_t i=1; i<path_along_global_graph.size(); ++i)
          {
            dist_along_path += (path_along_global_graph[i].head(3) - path_along_global_graph[i-1].head(3)).norm();
            if(dist_along_path >= planning_params_.active_homing_update_radius)
            {
              local_target = path_along_global_graph[i].head(3);
              break;
            }
          }
        }
      }
    }
    else
    {
      if(goal_dist > planning_params_.active_homing_update_radius)
      {
        if(!local_space_params_.isInsideSpace(local_navigation_goal_) || vs == VoxelStatus::kUnknown)
        {
          Vertex* current_vertex = NULL;
          if(global_graph_->getNearestVertex(&current_state_, &current_vertex))
          {
            ShortestPathsReport global_graph_rep;
            global_graph_->findShortestPaths(current_vertex->id, global_graph_rep);

            use_final_goal = false;
            int num_vertices = global_graph_->getNumVertices();
            int best_frontier_id = -1;
            std::tuple<int, double, double, double> best_frontier_info(-1, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()); // id, dist, path_len
            for (int id = 0; id < num_vertices; ++id) {
              if (global_graph_->getVertex(id)->type == VertexType::kFrontier) {
                ++num_frontiers;
                std::vector<StateVec> path;
                global_graph_->getShortestPath(id, global_graph_rep, true, path);
                double dist = (local_navigation_goal_ - global_graph_->getVertex(id)->state.head(3)).norm();
                std::vector<geometry_msgs::Pose> path_poses;
                convert(path, path_poses);
                double path_len = pathLength(path_poses);
                double total_len = path_len + dist;
                double frontier_factor = dist / total_len;
                double cost = path_len + (1.0 + frontier_factor)*dist;
                if(cost < std::get<3>(best_frontier_info))
                {
                  best_frontier_info = std::make_tuple(id, dist, path_len, cost);
                }
              }
            }
            best_frontier_id = std::get<0>(best_frontier_info);
            if(num_frontiers)
            {
              std::vector<StateVec> path_along_global_graph;
              global_graph_->getShortestPath(best_frontier_id, global_graph_rep, true, path_along_global_graph);

              if(!path_along_global_graph.empty())
              {
                std::vector<geometry_msgs::Pose> path_poses;
                convert(path_along_global_graph, path_poses);
                visualization_->visualizeRefPath(path_poses, 2);
                double dist_along_path = 0.0;
                local_target = path_along_global_graph.back().head(3);
                for(size_t i=1; i<path_along_global_graph.size(); ++i)
                {
                  dist_along_path += (path_along_global_graph[i].head(3) - path_along_global_graph[i-1].head(3)).norm();
                  if(dist_along_path >= planning_params_.active_homing_update_radius)
                  {
                    local_target = path_along_global_graph[i].head(3);
                    break;
                  }
                }
              }
            }
          }
        }
      }
    }


    geometry_msgs::PointStamped local_target_msg;
    local_target_msg.header.frame_id = world_frame_;
    local_target_msg.header.stamp = ros::Time::now();
    local_target_msg.point.x = local_target.x();
    local_target_msg.point.y = local_target.y();
    local_target_msg.point.z = local_target.z();
    local_target_pub_.publish(local_target_msg);

    double g_2_c = (local_navigation_goal_ - current_state_.head(3)).norm();
    if(g_2_c <= planning_params_.local_navigation_reaching_radius)
    {
      local_goal_distance_reached_ = std::numeric_limits<double>::max();
      local_navigation_goal_set_ = false;
      local_goal_progress_fail_iters_ = 0;
      local_navigation_recovery_attempts_ = 0;
      publishAblationMetrics();
      return LocalExplorationPlanner::LocalPlannerStatus::L_EXHAUSTED;
    }

    if(local_goal_progress_fail_iters_ >= planning_params_.local_navigation_max_fail_iters)
    {
      if (maybeRecoverLocalNavigation(local_target, "goal_progress_stalled")) {
        publishAblationMetrics();
        return LocalExplorationPlanner::LocalPlannerStatus::L_ERR;
      }
      local_goal_distance_reached_ = std::numeric_limits<double>::max();
      local_navigation_goal_set_ = false;
      local_goal_progress_fail_iters_ = 0;
      local_navigation_recovery_attempts_ = 0;
      publishAblationMetrics();
      return LocalExplorationPlanner::LocalPlannerStatus::L_STUCK;
    }

    double dist = (local_navigation_goal_ - current_state_.head(3)).norm();
    bool progress_made;
    if(use_final_goal)
    {
      progress_made = dist < local_goal_distance_reached_;
    }
    else
    {
      progress_made = num_frontiers > 0;
    }
    if(progress_made)
    {
      local_goal_distance_reached_ = dist;
      local_navigation_recovery_attempts_ = 0;
    }
    else
    {
      local_goal_progress_fail_iters_++;
    }

	    const double robot_radius =
	        std::max(0.05, robot_box_size_.maxCoeff() * 0.5);
	    const double min_path_clearance =
	        std::max(planning_params_.local_navigation_min_clearance,
	                 robot_radius + 0.05);
	    int skipped_root_candidates = 0;
	    int skipped_short_candidates = 0;
	    int skipped_clearance_candidates = 0;
	    for(int i=0; i< local_graph_->getNumVertices(); ++i)
	    {
	      int id = local_graph_->getVertex(i)->id;
	      if (id == 0) {
	        ++skipped_root_candidates;
	        continue;
	      }

	      std::vector<Vertex*> path;
	      local_graph_->getShortestPath(id, local_graph_rep_, true, path);
	      if (path.size() <= 1) {
	        ++skipped_short_candidates;
	        continue;
	      }

	      double path_length = 0.0;
	      double path_clearance = std::numeric_limits<double>::infinity();
	      bool clearance_ok = true;
	      for (size_t j = 0; j < path.size(); ++j) {
	        const Eigen::Vector3d candidate_position =
	            path[j]->state.head(3) + robot_params_.center_offset;
	        const double clearance =
	            voxel_map_->getPointDistance(candidate_position);
	        if (clearance < 0.0) {
	          clearance_ok = false;
	          break;
	        }
	        path_clearance = std::min(path_clearance, clearance);
	        if (j > 0) {
	          path_length +=
	              (path[j]->state.head(3) - path[j - 1]->state.head(3)).norm();
	        }
	      }
	      if (!clearance_ok || path_clearance < min_path_clearance) {
	        ++skipped_clearance_candidates;
	        continue;
	      }
	      if (path_length < planning_params_.local_navigation_min_path_length) {
	        ++skipped_short_candidates;
	        continue;
	      }

	      double goal_dist = (path.back()->state.head(3) - local_target).norm() + 0.001;
	      const double clearance_bonus =
	          1.0 + planning_params_.local_navigation_clearance_weight *
	                    std::min(path_clearance / min_path_clearance, 2.0);
	      double path_gain = clearance_bonus / goal_dist;

	      if (path_gain > best_gain)
	      {
	        best_gain = path_gain;
	        best_path_id = id;
	      }
	    }

	    if(best_path_id >= 0 && best_gain > 0)
	    {
      std::vector<int> path;
      local_graph_->getShortestPath(best_path_id, local_graph_rep_, false, path);
      for (int i = 0; i < (path.size() - 1); ++i) {
        local_graph_->getVertex(path[i])->parent =
            local_graph_->getVertex(path[i + 1]);
      }
      best_vertex_ = local_graph_->getVertex(path[0]);
      //
      visualization_->visualizeBestPaths(local_graph_, local_graph_rep_, 10,
                                          best_path_id);
      ROS_INFO("\033[1;32m[Local Navigation]\033[0m Best path accepted: gain=%.3f, vertex=%d",
               best_gain, best_path_id);
      t2 = std::chrono::high_resolution_clock::now();
      stat_chrono_->evaluate_graph_time =
          std::chrono::duration<double, std::milli>(t2 - t1).count();
      local_navigation_recovery_attempts_ = 0;
      publishAblationMetrics();
      return LocalExplorationPlanner::LocalPlannerStatus::L_OK;
	    }
	    else
	    {
        if (maybeRecoverLocalNavigation(local_target, "no_candidate_path")) {
          t2 = std::chrono::high_resolution_clock::now();
          stat_chrono_->evaluate_graph_time =
              std::chrono::duration<double, std::milli>(t2 - t1).count();
          publishAblationMetrics();
          return LocalExplorationPlanner::LocalPlannerStatus::L_ERR;
        }
	      ROS_WARN_COND(
	          global_verbosity >= Verbosity::WARN,
	          "[Local Navigation] No non-trivial candidate path passed filters. "
	          "skipped_root=%d skipped_short=%d skipped_clearance=%d",
	          skipped_root_candidates, skipped_short_candidates,
	          skipped_clearance_candidates);
	      t2 = std::chrono::high_resolution_clock::now();
	      stat_chrono_->evaluate_graph_time =
	          std::chrono::duration<double, std::milli>(t2 - t1).count();
        publishAblationMetrics();
	      return LocalExplorationPlanner::LocalPlannerStatus::L_ERR;
	    }
  }
  else
  {
    ROS_WARN("\033[1;32m[Local Navigation]\033[0m No local navigation goal set.");
    local_navigation_recovery_attempts_ = 0;
    publishAblationMetrics();
    return LocalExplorationPlanner::LocalPlannerStatus::L_EXHAUSTED;
  }
}

bool LocalExplorationPlanner::modifyPath(pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl,
                     Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                     Eigen::Vector3d& p1_mod) {
  p1_mod = p1;

  Eigen::Vector3d p_center;
  p_center = (p0 + p1) / 2.0;
  Eigen::Vector3d p_dir;
  p_dir = (p1 - p0);
  double radius = p_dir.norm() / 2.0;
  Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  Eigen::Quaternion<double> quat_W2S;
  Eigen::Vector3d p_dir_norm = p_dir.normalized();
  double yaw_angle = std::atan2(p_dir_norm.y(), p_dir_norm.x());
  double pitch_angle =
      -std::atan2(p_dir_norm.z(), std::sqrt(p_dir_norm.x() * p_dir_norm.x() +
                                            p_dir_norm.y() * p_dir_norm.y()));
  quat_W2S = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY());

  pcl::PointCloud<pcl::PointXYZ>* pcl_tf(new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Translation<double, 3> trans_W2S(p_center);
  Eigen::Transform<double, 3, Eigen::Affine> tf_W2S(trans_W2S * quat_W2S);
  pcl::transformPointCloud(*obstacle_pcl, *pcl_tf, tf_W2S.inverse());
  double kDx = robot_params_.safety_extension[0];
  double kDy = robot_params_.safety_extension[1];
  double kDz = robot_params_.safety_extension[2];
  std::vector<Eigen::Vector3d> u_l;
  std::vector<Eigen::Vector3d> p_l;
  u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  u_l.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
  u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  u_l.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  p_l.push_back(Eigen::Vector3d(-radius - kDx, 0.0, 0.0));
  p_l.push_back(Eigen::Vector3d(radius + kDx, 0.0, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, -kDy, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, kDy, 0.0));
  p_l.push_back(Eigen::Vector3d(0.0, 0.0, -kDz));
  p_l.push_back(Eigen::Vector3d(0.0, 0.0, kDz));
  std::vector<Eigen::Vector3d> hyperplane_list;
  std::vector<Eigen::Vector3d> tangent_point_list;
  for (int i = 0; i < 6; ++i) {
    Eigen::Vector3d a_l;
    a_l = u_l[i] / (u_l[i].dot(p_l[i]));
    tangent_point_list.push_back(p_l[i]);
    hyperplane_list.push_back(a_l);
  }
  pcl::PointCloud<pcl::PointXYZ>* pcl_in_box(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
    const double kDSign = 0.05;  // numeric issue
    double sign;
    int i = 0;
    for (i = 0; i < 6; ++i) {
      sign = p->x * hyperplane_list[i].x() + p->y * hyperplane_list[i].y() +
             p->z * hyperplane_list[i].z() - 1;
      if (sign > kDSign) break;
    }
    if (i == 6) {
      pcl_in_box->push_back(*p);
    }
  }
  if (pcl_in_box->size())
    pcl::copyPointCloud(*pcl_in_box, *pcl_tf);
  else {
    return true;
  }
  double dist_min_sq = std::numeric_limits<double>::max();
  Eigen::Vector3d p_tangent;
  for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
    double dist_t = p->x * p->x + p->y * p->y + p->z * p->z;
    if (dist_t < dist_min_sq) {
      dist_min_sq = dist_t;
      p_tangent << p->x, p->y, p->z;
    }
  }

  const double kDDist = 0.01;  // deal with numeric error.
  if ((dist_min_sq == std::numeric_limits<double>::max()) ||
      (dist_min_sq < kDDist)) {
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[IMPRV] Path too close to obstacle");
    return false;
  }

  double a = radius, b = radius, c = radius;  // dimensions of the ellipsoid.
  if (dist_min_sq < (radius * radius)) {
    b = std::sqrt(
        (p_tangent.y() * p_tangent.y() + p_tangent.z() * p_tangent.z()) /
        (1 - p_tangent.x() * p_tangent.x() / (a * a)));
    c = b;  // Set equal b for now; but could increase.???
    Eigen::Vector3d hyperplane_last =
        Eigen::Vector3d(p_tangent.x() / (a * a), p_tangent.y() / (b * b),
                        p_tangent.z() / (c * c));
    hyperplane_list.push_back(hyperplane_last);
    tangent_point_list.push_back(p_tangent);
  }
  bool stop = false;
  int n_max = 0;  // magic number: max 50 hyperplanes
  while ((!stop) && (n_max < 50)) {
    ++n_max;
    pcl::PointCloud<pcl::PointXYZ>* pcl_reduced(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (hyperplane_list.size()) {
      Eigen::Vector3d hyperplane_last;
      hyperplane_last = hyperplane_list.back();
      for (auto p = pcl_tf->begin(); p != pcl_tf->end(); ++p) {
        double sign = p->x * hyperplane_last.x() + p->y * hyperplane_last.y() +
                      p->z * hyperplane_last.z() - 1;
        const double kDSign = -0.05;  // numeric issue
        if (sign < kDSign) {
          pcl_reduced->push_back(*p);
        }
      }
    } else {
      pcl::copyPointCloud(*pcl_tf, *pcl_reduced);
    }

    Eigen::Vector3d p_tangent1;
    dist_min_sq = std::numeric_limits<double>::max();
    for (auto p = pcl_reduced->begin(); p != pcl_reduced->end(); ++p) {
      pcl::PointXYZ pv;
      pv.x = p->x / a;
      pv.y = p->y / b;
      pv.z = p->z / c;
      double dist_t = pv.x * pv.x + pv.y * pv.y + pv.z * pv.z;
      if (dist_t < dist_min_sq) {
        dist_min_sq = dist_t;
        p_tangent1 << p->x, p->y, p->z;
      }
    }
    if ((pcl_reduced->size() == 0) ||
        (dist_min_sq == std::numeric_limits<double>::max())) {
      stop = true;
    } else {
      double e_ext = dist_min_sq;
      Eigen::Vector3d hyperplane_new = Eigen::Vector3d(
          p_tangent1.x() / (a * a * e_ext), p_tangent1.y() / (b * b * e_ext),
          p_tangent1.z() / (c * c * e_ext));
      hyperplane_list.push_back(hyperplane_new);
      tangent_point_list.push_back(p_tangent1);
      pcl_tf->clear();
      pcl::copyPointCloud(*pcl_reduced, *pcl_tf);
    }
  }
  if (!stop) {
    return false;
  }
  std::vector<Eigen::Vector3d> feasible_samples;
  for (double dy = -kDy; dy < kDy; dy += 0.1) {
    for (double dz = -kDz; dz < kDz; dz += 0.1) {
      Eigen::Vector3d p(radius, dy, dz);
      const double kDSign = -0.05;  // numeric issue
      double sign;
      int i = 0;
      for (i = 0; i < hyperplane_list.size(); ++i) {
        sign = p.x() * hyperplane_list[i].x() + p.y() * hyperplane_list[i].y() +
               p.z() * hyperplane_list[i].z() - 1;
        if (sign > kDSign) break;
      }
      if (i == hyperplane_list.size()) {
        feasible_samples.push_back(p);
      }
    }
  }

  for (int i = 0; i < hyperplane_list.size(); ++i) {
    tangent_point_list[i] =
        tf_W2S * tangent_point_list[i];  // convert back to world
    Eigen::Matrix4d tf_inv_T = tf_W2S.matrix().inverse().transpose();
    Eigen::Vector4d v_t;
    v_t = tf_inv_T * Eigen::Vector4d(hyperplane_list[i].x(),
                                     hyperplane_list[i].y(),
                                     hyperplane_list[i].z(), -1.0);
    v_t = v_t / (-v_t[3]);
    hyperplane_list[i] << v_t.x(), v_t.y(), v_t.z();
  }

  p1_mod << 0.0, 0.0, 0.0;
  int feasible_count = 0;
  for (int i = 0; i < feasible_samples.size(); ++i) {
    feasible_samples[i] =
        tf_W2S * feasible_samples[i];  // convert back to world
    if (voxel_map_->getVoxelStatus(feasible_samples[i]) ==
        VoxelStatus::kFree) {
      feasible_corridor_pcl_->push_back(pcl::PointXYZ(feasible_samples[i].x(),
                                                      feasible_samples[i].y(),
                                                      feasible_samples[i].z()));
      p1_mod = p1_mod + feasible_samples[i];
      ++feasible_count;
    }
  }

  if (feasible_count) {
    p1_mod = p1_mod / feasible_count;
  } else {
    return false;
  }

  visualization_->visualizeHyperplanes(p_center, hyperplane_list,
                                       tangent_point_list);
  return true;
}

void LocalExplorationPlanner::computeExplorationGain(bool only_leaf_vertices) {
  computeExplorationGain(only_leaf_vertices, false);
}

void LocalExplorationPlanner::computeExplorationGain(bool only_leaf_vertices, bool clustering) {
  const int id_viz = 20;  // random vertex to show volumetric gain.
  const double clustering_range = planning_params_.clustering_radius;
  int num_clusters = 0;
  ros::Time tim;
  START_TIMER(tim);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;
  std::unordered_map<int, Vertex*> vertex_map = local_graph_->vertices_map_;
  std::list<int> vertex_ids;
  int candidate_viewpoints = 0;
  int evaluated_viewpoints = 0;
  for (int i = 0; i < local_graph_->getNumVertices(); ++i) {
    if ((!only_leaf_vertices) || vertex_map[i]->is_leaf_vertex) {
      ++candidate_viewpoints;
    }
    if (vertex_map[i]->is_leaf_vertex) {
      vertex_ids.push_front(i);
    } else {
      vertex_ids.push_back(i);
    }
  }

  std::unordered_set<int> fine_gain_vertices;
  shortlistVerticesForFineGain(only_leaf_vertices, fine_gain_vertices);

  while (!vertex_ids.empty()) {
    int v_id = (*vertex_ids.begin());
    vertex_ids.remove(v_id);
    ++num_clusters;
    bool viz_en = false;
    if (v_id == id_viz) viz_en = true;
    bool should_evaluate =
        (!only_leaf_vertices) || (vertex_map[v_id]->is_leaf_vertex);
    if (last_gain_eval_partial_) {
      should_evaluate =
          should_evaluate &&
          fine_gain_vertices.find(v_id) != fine_gain_vertices.end();
    }

    if (should_evaluate) {
      ++evaluated_viewpoints;
      if (planning_params_.use_ray_model_for_volumetric_gain) {
        if (vertex_map[v_id]->is_leaf_vertex) {
          computeVolumetricGainRayModel(vertex_map[v_id]->state,
                                        vertex_map[v_id]->vol_gain, viz_en,
                                        false);
        } else {
          computeVolumetricGainRayModel(vertex_map[v_id]->state,
                                        vertex_map[v_id]->vol_gain, viz_en,
                                        true);
        }
      } else {
        computeVolumetricGain(vertex_map[v_id]->state,
                              vertex_map[v_id]->vol_gain, viz_en);
      }
    } else {
      vertex_map[v_id]->vol_gain.reset();
      if (vertex_map[v_id]->type == VertexType::kFrontier) {
        vertex_map[v_id]->type = VertexType::kUnvisited;
      }
    }
    if (clustering) {
      if (should_evaluate) {
        std::vector<Vertex*> nearest_vertices;
        local_graph_->getNearestVertices(&vertex_map[v_id]->state,
                                         clustering_range, &nearest_vertices);
        for (auto v : nearest_vertices) {
          std::list<int>::iterator it;
          it = std::find(vertex_ids.begin(), vertex_ids.end(), v->id);
          if (it != vertex_ids.end()) {
            v->vol_gain = vertex_map[v_id]->vol_gain;
            vertex_ids.remove(v->id);
          }
        }
      }
    }
    if (vertex_map[v_id]->vol_gain.is_frontier)
      vertex_map[v_id]->type = VertexType::kFrontier;
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Num clusters: %d", num_clusters);
  accumulateViewpointEvaluation(candidate_viewpoints, evaluated_viewpoints);
  stat_->compute_exp_gain_time = GET_ELAPSED_TIME(tim);
  t2 = std::chrono::high_resolution_clock::now();
  stat_chrono_->compute_exp_gain_time =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
}

void LocalExplorationPlanner::computeVolumetricGain(StateVec& state, VolumetricGain& vgain,
                                bool vis_en) {
  vgain.reset();
  double step_size = planning_params_.exp_gain_voxel_size;
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;
  if (local_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; ++i) {
      bound_min[i] = root_vertex_->state[i] - local_space_params_.radius -
                     local_space_params_.radius_extension;
      bound_max[i] = root_vertex_->state[i] + local_space_params_.radius +
                     local_space_params_.radius_extension;
    }
  } else if (local_space_params_.type == BoundedSpaceType::kCylinder) {
    bound_min[0] = root_vertex_->state[0] - local_space_params_.radius -
                   local_space_params_.radius_extension;
    bound_max[0] = root_vertex_->state[0] + local_space_params_.radius +
                   local_space_params_.radius_extension;
    bound_min[1] = root_vertex_->state[1] - local_space_params_.radius -
                   local_space_params_.radius_extension;
    bound_max[1] = root_vertex_->state[1] + local_space_params_.radius +
                   local_space_params_.radius_extension;
    bound_min[2] = root_vertex_->state[2] + local_space_params_.min_val[2] +
                   local_space_params_.min_extension[2];
    bound_max[2] = root_vertex_->state[2] + local_space_params_.max_val[2] +
                   local_space_params_.max_extension[2];
  } else if (local_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; ++i) {
      bound_min[i] = root_vertex_->state[i] + local_space_params_.min_val[i] +
                     local_space_params_.min_extension[i];
      bound_max[i] = root_vertex_->state[i] + local_space_params_.max_val[i] +
                     local_space_params_.max_extension[i];
    }
  } else {
    PLANNER_ERROR("Local space is not defined.");
    return;
  }
  if (global_space_params_.type == BoundedSpaceType::kSphere) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], -global_space_params_.radius -
                                     global_space_params_.radius_extension);
      bound_max[i] =
          std::min(bound_max[i], global_space_params_.radius +
                                     global_space_params_.radius_extension);
    }
  } else if (global_space_params_.type == BoundedSpaceType::kCuboid) {
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i], global_space_params_.min_val[i] +
                                     global_space_params_.min_extension[i]);
      bound_max[i] =
          std::min(bound_max[i], global_space_params_.max_val[i] +
                                     global_space_params_.max_extension[i]);
    }
  } else {
    PLANNER_ERROR("Global space is not defined.");
    return;
  }

  std::vector<std::tuple<int, int, int>> gain_log;
  gain_log.clear();
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> voxel_log;
  voxel_log.clear();
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];
    for (int i = 0; i < 3; i++) {
      bound_min[i] =
          std::max(bound_min[i],
                   state[i] - sensor_params_.sensor[sensor_name].max_range);
      bound_max[i] =
          std::min(bound_max[i],
                   state[i] + sensor_params_.sensor[sensor_name].max_range);
    }

    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;
    Eigen::Vector3d origin(state[0], state[1], state[2]);
    Eigen::Vector3d voxel;
    for (voxel[0] = bound_min[0]; voxel[0] < bound_max[0];
         voxel[0] += step_size) {
      for (voxel[1] = bound_min[1]; voxel[1] < bound_max[1];
           voxel[1] += step_size) {
        for (voxel[2] = bound_min[2]; voxel[2] < bound_max[2];
             voxel[2] += step_size) {
          if (sensor_params_.sensor[sensor_name].isInsideFOV(state, voxel)) {
            VoxelStatus vs_ray =
                voxel_map_->getRayStatus(origin, voxel, true);
            if (vs_ray != VoxelStatus::kOccupied) {
              VoxelStatus vs = voxel_map_->getVoxelStatus(voxel);
              if (vs == VoxelStatus::kUnknown) {
                ++num_unknown_voxels;
              } else if (vs == VoxelStatus::kFree) {
                ++num_free_voxels;
              } else if (vs == VoxelStatus::kOccupied) {
                ++num_occupied_voxels;
              }
              if (vis_en) voxel_log.push_back(std::make_pair(voxel, vs));
            }
          }
        }
      }
    }
    gain_log.push_back(std::make_tuple(num_unknown_voxels, num_free_voxels,
                                       num_occupied_voxels));
  }
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    vgain.gain += num_unknown_voxels * planning_params_.unknown_voxel_gain +
                  num_free_voxels * planning_params_.free_voxel_gain +
                  num_occupied_voxels * planning_params_.occupied_voxel_gain;
  }
  if (vis_en) {
    visualization_->visualizeVolumetricGain(bound_min, bound_max, voxel_log,
                                            step_size);
  }
}

void LocalExplorationPlanner::computeInspectionGainRayModel(Vertex* vert) {
  vert->vol_gain.reset();

  for(std::string sensor_name : planning_params_.inspection_sensor_list) {
    std::vector<VoxelLog> voxel_logs;
    SensorParamsBase sensor = sensor_params_.sensor[sensor_name];
    
    voxel_map_->getCameraScanStatus(vert->state, sensor, voxel_logs);

    for(VoxelLog log : voxel_logs) {
      if (global_space_params_.isInsideSpace(log.voxel_center)) {
        bool no_gain_zone_cleared = true;
        if (use_no_gain_space_) {
          for (auto& zone : no_gain_zones_) {
            if (zone.isInsideSpace(log.voxel_center)) {
              no_gain_zone_cleared = false;
              break;
            }
          }
        }
        if(no_gain_zone_cleared) {
          vert->vol_gain.unseen_voxel_hash_keys.insert(log.voxel_hash);
        }
      }
    }
  }

}

void LocalExplorationPlanner::computeVolumetricGainRayModel(StateVec& state, VolumetricGain& vgain,
                                        bool vis_en, bool iterative) {
  vgain.reset();

  std::vector<std::tuple<int, int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> voxel_log;
  int raw_unk_voxels_count = 0;
  int raw_occ_voxels_count = 0;
  int num_ray_endpoints = 0;
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    std::tuple<int, int, int> gain_log_tmp;
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>
        voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;
    sensor_params_.sensor[sensor_name].getFrustumEndpoints(state,
                                                           multiray_endpoints);

    num_ray_endpoints += multiray_endpoints.size();
    if(planning_params_.use_camera_gain) {
      voxel_map_->getCameraScanStatus(origin, multiray_endpoints, gain_log_tmp,
                                  voxel_log_tmp,
                                  sensor_params_.sensor[sensor_name]);  
    }
    else {
      voxel_map_->getScanStatus(origin, multiray_endpoints, gain_log_tmp,
                                  voxel_log_tmp,
                                  sensor_params_.sensor[sensor_name]);
    }

    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0,
        num_unknown_surf_voxels = 0;

    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      VoxelStatus vs = vl.second;
      if (vs == VoxelStatus::kUnknown) ++raw_unk_voxels_count;
      if (vs == VoxelStatus::kOccupied) ++raw_occ_voxels_count;
      if (global_space_params_.isInsideSpace(voxel)) {
        bool no_gain_zone_cleared = true;
        if (use_no_gain_space_) {
          for (auto& zone : no_gain_zones_) {
            if (zone.isInsideSpace(voxel)) {
              no_gain_zone_cleared = false;
              break;
            }
          }
        }
        if (no_gain_zone_cleared) {
          if (vs == VoxelStatus::kUnknown) {
            ++num_unknown_voxels;
          } else if (vs == VoxelStatus::kFree) {
            ++num_free_voxels;
          } else if (vs == VoxelStatus::kOccupied) {
            ++num_occupied_voxels;
          } else {
            ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Unsupported voxel type.");
          }
          if (vis_en) voxel_log.push_back(std::make_pair(voxel, vs));
        }
      }
    }
    gain_log.push_back(std::make_tuple(num_unknown_voxels, num_free_voxels,
                                       num_occupied_voxels,
                                       num_unknown_surf_voxels));
    if (vis_en) {
      visualization_->visualizeRays(state, multiray_endpoints);
    }
    if (sensor_params_.sensor[sensor_name].isFrontier(
            num_unknown_voxels * voxel_map_->getResolution())) {
      vgain.is_frontier = true;  // Event E2
    }
  }
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    vgain.gain += num_unknown_voxels * planning_params_.unknown_voxel_gain +
                  num_free_voxels * planning_params_.free_voxel_gain +
                  num_occupied_voxels * planning_params_.occupied_voxel_gain;
  }

  if((float)raw_occ_voxels_count/(float)num_ray_endpoints <= planning_params_.min_occ_surface)
  {
    vgain.is_frontier = false;
    vgain.num_unknown_voxels += 0;
    vgain.gain = 0.0;
  }
if (vis_en) {
  Eigen::Vector3d bound_min;
  Eigen::Vector3d bound_max;
  visualization_->visualizeVolumetricGain(bound_min, bound_max, voxel_log,
                                          voxel_map_->getResolution());
}
}

void LocalExplorationPlanner::computeVolumetricGainRayModelNoBound(StateVec& state,
                                               VolumetricGain& vgain) {
  vgain.reset();

  std::vector<std::tuple<int, int, int>> gain_log;
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>> voxel_log;
  for (int ind = 0; ind < planning_params_.exp_sensor_list.size(); ++ind) {
    std::string sensor_name = planning_params_.exp_sensor_list[ind];

    Eigen::Vector3d origin(state[0], state[1], state[2]);
    std::tuple<int, int, int> gain_log_tmp;
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>
        voxel_log_tmp;
    std::vector<Eigen::Vector3d> multiray_endpoints;
    sensor_params_.sensor[sensor_name].getFrustumEndpoints(state,
                                                           multiray_endpoints);
    voxel_map_->getScanStatus(origin, multiray_endpoints, gain_log_tmp,
                                voxel_log_tmp,
                                sensor_params_.sensor[sensor_name]);
    int num_unknown_voxels = 0, num_free_voxels = 0, num_occupied_voxels = 0;

    for (auto& vl : voxel_log_tmp) {
      Eigen::Vector3d voxel = vl.first;
      VoxelStatus vs = vl.second;
      if (global_space_params_.isInsideSpace(voxel)) {
        if (vs == VoxelStatus::kUnknown) {
          ++num_unknown_voxels;
        } else if (vs == VoxelStatus::kFree) {
          ++num_free_voxels;
        } else if (vs == VoxelStatus::kOccupied) {
          ++num_occupied_voxels;
        } else {
          ROS_ERROR_COND(global_verbosity >= Verbosity::ERROR, "Unsupported voxel type.");
        }
      }
    }
    gain_log.push_back(std::make_tuple(num_unknown_voxels, num_free_voxels,
                                       num_occupied_voxels));
    if (sensor_params_.sensor[sensor_name].isFrontier(
            num_unknown_voxels * voxel_map_->getResolution())) {
      vgain.is_frontier = true;  // Event E2
    }
  }
  for (int i = 0; i < gain_log.size(); ++i) {
    int num_unknown_voxels = std::get<0>(gain_log[i]);
    int num_free_voxels = std::get<1>(gain_log[i]);
    int num_occupied_voxels = std::get<2>(gain_log[i]);
    vgain.num_unknown_voxels += num_unknown_voxels;
    vgain.num_free_voxels += num_free_voxels;
    vgain.num_occupied_voxels += num_occupied_voxels;
    vgain.gain += num_unknown_voxels * planning_params_.unknown_voxel_gain +
                  num_free_voxels * planning_params_.free_voxel_gain +
                  num_occupied_voxels * planning_params_.occupied_voxel_gain;
  }
}

void LocalExplorationPlanner::setRootStateForPlanning(const geometry_msgs::Pose& root_pose) {
  state_for_planning_[0] = root_pose.position.x;
  state_for_planning_[1] = root_pose.position.y;
  state_for_planning_[2] = root_pose.position.z;
  state_for_planning_[3] = tf::getYaw(root_pose.orientation);
  if ((state_for_planning_[0] == 0.0) && (state_for_planning_[1] == 0.0) &&
      (state_for_planning_[2] == 0.0)) {
    planning_params_.use_current_state = true;
  } else {
    planning_params_.use_current_state = false;
  }
}



std::vector<geometry_msgs::Pose> LocalExplorationPlanner::getBestPathSimplified()
{
  std::vector<geometry_msgs::Pose> ret, empty_path;
  int id = best_vertex_->id;
  if (id == 0) return ret;
  double traverse_length = 0;
  double traverse_time = 0;
  std::vector<StateVec> best_path;
  local_graph_->getShortestPath(id, local_graph_rep_, true, best_path);
  Eigen::Vector3d p0(best_path[0][0], best_path[0][1], best_path[0][2]);
  std::vector<Vertex*> best_path_vertices;
  local_graph_->getShortestPath(best_vertex_->id, local_graph_rep_, true,
                                best_path_vertices);

  std::vector<Eigen::Vector3d> path_vec;
  local_graph_->getShortestPath(best_vertex_->id, local_graph_rep_, true,
                                path_vec);
  
  double best_path_direction = Trajectory::estimateDirectionFromPath(path_vec);
  constexpr double kDiffAngleForwardThres = 0.5 * (M_PI + M_PI / 3);
  bool res = compareAngles(exploring_direction_, best_path_direction, kDiffAngleForwardThres);
  if(!planning_params_.allow_sudden_dir_change)
  {
    if(dir_change_count_ < 3)
    {
      if (!res) {
        ROS_WARN("Changing exploration direction.[%f --> %f]", exploring_direction_,
                best_path_direction);
        ++dir_change_count_;
        return ret;
      }
    }
    else
    {
      dir_change_count_ = 0;
    }
  }

  std::vector<Vertex*> ref_vertices;
  for (int i = 0; i < best_path.size(); ++i) {
    if (best_path_vertices[i]->is_hanging) {
      break;
    }

    Eigen::Vector3d p1(best_path[i][0], best_path[i][1], best_path[i][2]);
    Eigen::Vector3d dir_vec = p1 - p0;
    Eigen::Vector3d p_overshoot =
        dir_vec.normalized() * planning_params_.edge_overshoot;
    Eigen::Vector3d p_start = p0 + robot_params_.center_offset - p_overshoot;
    Eigen::Vector3d p_end =
        p0 + robot_params_.center_offset + dir_vec + p_overshoot;
    if ((dir_vec.norm() > 0) &&
        (VoxelStatus::kFree !=
         voxel_map_->getPathStatus(p_start, p_end, robot_box_size_, true))) {
      ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Segment [%d] is not clear.", i);
    }

    tf::Quaternion quat;
    Eigen::Matrix3d rot_eigen;
    rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(best_path[i][3], Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    rot_eigen = rot_eigen * Eigen::AngleAxisd(best_path[i][4], Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q_eigen(rot_eigen);
    quat.setX(q_eigen.x());
    quat.setY(q_eigen.y());
    quat.setZ(q_eigen.z());
    quat.setW(q_eigen.w());
    tf::Vector3 origin(best_path[i][0], best_path[i][1], best_path[i][2]);
    tf::Pose poseTF(quat, origin);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(poseTF, pose);
    ret.push_back(pose);
    ref_vertices.push_back(best_path_vertices[i]);

    double seg_length = (p1 - p0).norm();
    traverse_length += seg_length;
    traverse_time += seg_length / planning_params_.v_max;
    if ((traverse_length > planning_params_.traverse_length_max) ||
        (traverse_time > planning_params_.traverse_time_max)) {
      break;
    }
    p0 = p1;
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Best path:  size = %d, length = %f, time = %f", (int)ret.size(),
           traverse_length, traverse_time);
  bool path_added = false;
  if ((int)ret.size() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No feasible local exploration path.");
    return empty_path;
  }
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret, mod_path, false)) {
      ret = mod_path;
      commitVerifiedPathToCognitiveMap(global_graph_, mod_path);
      path_added = true;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Computed an alternate path in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }

  if (!path_added) {
    commitVerifiedPathToCognitiveMap(global_graph_, ref_vertices);
  }

  if (planning_params_.local_path_smoothing_enable) {
    std::vector<geometry_msgs::Pose> smoothed_path;
    if (smoothLocalPathLightweight(ret, smoothed_path)) {
      ret = smoothed_path;
    }
  }
  if(planning_params_.path_interpolation_distance > 0.0) {
    const double kInterpolationDistance =
        planning_params_.path_interpolation_distance;
    std::vector<geometry_msgs::Pose> interp_path;
    if (Trajectory::interpolatePath(ret, kInterpolationDistance, interp_path)) {
      ret = interp_path;
    }
  }

  visualization_->visualizeRefPath(ret);

  return ret;
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::getBestPath(std::string tgt_frame,
                                                  int& status) {
  if (planning_params_.auto_landing_enable) {
    double time_elapsed = 0.0;
    std::vector<geometry_msgs::Pose> empty_path;
    if ((ros::Time::now()).toSec() != 0.0) {
      if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
      time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
    }
    double time_budget_remaining =
        planning_params_.time_budget_before_landing - time_elapsed;
    if (time_budget_remaining <= 0.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "RAN OUT OF TIME BUDGET --> LANDING.");
      landing_engaged_ = true;
      std_msgs::Bool stop_msg;
      stop_msg.data = true;

      std_srvs::Empty empty_srv;
      if (!landing_engaged_) {
        landing_srv_client_.call(empty_srv);
        pci_reset_pub_.publish(stop_msg);
      }
      status = planner_msgs::planner_srv::Response::kForward;
      return empty_path;
    }
  }
  if (planning_params_.auto_homing_enable) {
    status = planner_msgs::planner_srv::Response::kHoming;
    std::vector<geometry_msgs::Pose> homing_path;
    double time_elapsed = 0.0;
    if ((ros::Time::now()).toSec() != 0.0) {
      if (rostime_start_.toSec() == 0.0) rostime_start_ = ros::Time::now();
      time_elapsed = (double)((ros::Time::now() - rostime_start_).toSec());
    }
    double time_budget_remaining =
        planning_params_.time_budget_limit - time_elapsed;
    if (time_budget_remaining <= 0.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "RAN OUT OF TIME BUDGET --> STOP HERE.");
      return homing_path;
    }
    if (current_battery_time_remaining_ <= 0.0) {
      ROS_WARN_COND(global_verbosity >= Verbosity::PLANNER_STATUS, "RAN OUT OF BATTERY --> STOP HERE.");
      return homing_path;
    }
    double time_remaining =
        std::min(time_budget_remaining, current_battery_time_remaining_);
    Vertex* root_vertex = local_graph_->getVertex(0);
    homing_path = searchHomingPath(tgt_frame, root_vertex->state);
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
        return homing_path;
      }
    } else {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Cannot find a path to return home from here.");
    }
  }

  std::vector<geometry_msgs::Pose> ret, empty_path;
  int id = best_vertex_->id;
  if (id == 0) return ret;
  std::vector<Eigen::Vector3d> best_path_3d;
  local_graph_->getShortestPath(id, local_graph_rep_, true, best_path_3d);

  status = planner_msgs::planner_srv::Response::kForward;
  double traverse_length = 0;
  double traverse_time = 0;
  std::vector<StateVec> best_path;
  local_graph_->getShortestPath(id, local_graph_rep_, true, best_path);
  Eigen::Vector3d p0(best_path[0][0], best_path[0][1], best_path[0][2]);
  std::vector<Vertex*> best_path_vertices;
  local_graph_->getShortestPath(best_vertex_->id, local_graph_rep_, true,
                                best_path_vertices);

  const double kLenMin = 1.0;
  const double kLenMinMin = 0.3;
  std::vector<Eigen::Vector3d> path_vec;
  local_graph_->getShortestPath(best_vertex_->id, local_graph_rep_, true,
                                path_vec);
  double total_len = Trajectory::getPathLength(path_vec);
  double len_min_thres = kLenMin;

  len_min_thres = kLenMinMin;

  if (total_len <= len_min_thres) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Best path is too short.");
    return ret;
  }

  std::vector<Vertex*> ref_vertices;
  for (int i = 0; i < best_path.size(); ++i) {
    if (best_path_vertices[i]->is_hanging) {
      break;
    }

    Eigen::Vector3d p1(best_path[i][0], best_path[i][1], best_path[i][2]);
    Eigen::Vector3d dir_vec = p1 - p0;
    Eigen::Vector3d p_overshoot =
        dir_vec.normalized() * planning_params_.edge_overshoot;
    Eigen::Vector3d p_start = p0 + robot_params_.center_offset - p_overshoot;
    Eigen::Vector3d p_end =
        p0 + robot_params_.center_offset + dir_vec + p_overshoot;
    if ((dir_vec.norm() > 0) &&
        (VoxelStatus::kFree !=
         voxel_map_->getPathStatus(p_start, p_end, robot_box_size_, true))) {
      ROS_INFO_COND(global_verbosity >= Verbosity::DEBUG, "Segment [%d] is not clear.", i);
    }

    tf::Quaternion quat;
    Eigen::Matrix3d rot_eigen;
    rot_eigen = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(best_path[i][3], Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    rot_eigen = rot_eigen * Eigen::AngleAxisd(best_path[i][4], Eigen::Vector3d::UnitY());
    Eigen::Quaterniond q_eigen(rot_eigen);
    quat.setX(q_eigen.x());
    quat.setY(q_eigen.y());
    quat.setZ(q_eigen.z());
    quat.setW(q_eigen.w());
    tf::Vector3 origin(best_path[i][0], best_path[i][1], best_path[i][2]);
    tf::Pose poseTF(quat, origin);
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(poseTF, pose);
    ret.push_back(pose);
    ref_vertices.push_back(best_path_vertices[i]);

    double seg_length = (p1 - p0).norm();
    traverse_length += seg_length;
    traverse_time += seg_length / planning_params_.v_max;
    if ((traverse_length > planning_params_.traverse_length_max) ||
        (traverse_time > planning_params_.traverse_time_max)) {
      break;
    }
    p0 = p1;
  }
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Best path:  size = %d, length = %f, time = %f", (int)ret.size(),
           traverse_length, traverse_time);
  bool path_added = false;
  if ((int)ret.size() <= 1) {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No feasible local exploration path.");
    return empty_path;
  }
  if (planning_params_.path_safety_enhance_enable) {
    ros::Time mod_time;
    START_TIMER(mod_time);
    std::vector<geometry_msgs::Pose> mod_path;
    if (improveFreePath(ret, mod_path, false)) {
      ret = mod_path;
      commitVerifiedPathToCognitiveMap(global_graph_, mod_path);
      path_added = true;
    }
    double dmod_time = GET_ELAPSED_TIME(mod_time);
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Computed an alternate path in %f(s)", dmod_time);
    visualization_->visualizeModPath(mod_path);
  }
  if (!path_added) {
    commitVerifiedPathToCognitiveMap(global_graph_, ref_vertices);
  }
  if(planning_params_.path_interpolation_distance > 0.0) {
    const double kInterpolationDistance =
        planning_params_.path_interpolation_distance;
    std::vector<geometry_msgs::Pose> interp_path;
    if (Trajectory::interpolatePath(ret, kInterpolationDistance, interp_path)) {
      ret = interp_path;
    }
  }

  visualization_->visualizeRefPath(ret);

  return ret;
}

bool LocalExplorationPlanner::smoothLocalPathLightweight(
    const std::vector<geometry_msgs::Pose>& path_orig,
    std::vector<geometry_msgs::Pose>& path_smoothed) {
  path_smoothed.clear();
  if (path_orig.size() < 3) {
    return false;
  }

  std::vector<geometry_msgs::Pose> current = path_orig;
  const int iterations = std::max(
      1, std::min(3, planning_params_.local_path_smoothing_iterations));
  const double min_segment = std::max(
      0.05, planning_params_.local_path_smoothing_min_segment);

  for (int iter = 0; iter < iterations; ++iter) {
    std::vector<geometry_msgs::Pose> next;
    next.reserve(current.size() * 2);
    next.push_back(current.front());
    for (size_t i = 0; i + 1 < current.size(); ++i) {
      Eigen::Vector3d p0(current[i].position.x, current[i].position.y,
                         current[i].position.z);
      Eigen::Vector3d p1(current[i + 1].position.x, current[i + 1].position.y,
                         current[i + 1].position.z);
      const double segment_len = (p1 - p0).norm();
      if (segment_len < min_segment) {
        continue;
      }

      geometry_msgs::Pose q = current[i];
      q.position.x = 0.75 * p0.x() + 0.25 * p1.x();
      q.position.y = 0.75 * p0.y() + 0.25 * p1.y();
      q.position.z = 0.75 * p0.z() + 0.25 * p1.z();
      geometry_msgs::Pose r = current[i + 1];
      r.position.x = 0.25 * p0.x() + 0.75 * p1.x();
      r.position.y = 0.25 * p0.y() + 0.75 * p1.y();
      r.position.z = 0.25 * p0.z() + 0.75 * p1.z();
      next.push_back(q);
      next.push_back(r);
    }
    next.push_back(current.back());
    if (next.size() < 3) {
      return false;
    }
    current.swap(next);
  }

  for (size_t i = 1; i < current.size(); ++i) {
    Eigen::Vector3d p0(current[i - 1].position.x, current[i - 1].position.y,
                       current[i - 1].position.z);
    Eigen::Vector3d p1(current[i].position.x, current[i].position.y,
                       current[i].position.z);
    if (voxel_map_->getPathStatus(p0, p1, robot_box_size_, true) !=
        VoxelStatus::kFree) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                    "[LocalPathSmoothing] Reject smoothed path: segment is not free.");
      return false;
    }
    if (planning_params_.geofence_checking_enable &&
        GeofenceManager::CoordinateStatus::kViolated ==
            geofence_manager_->getPathStatus(
                Eigen::Vector2d(p0.x(), p0.y()),
                Eigen::Vector2d(p1.x(), p1.y()),
                Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1]))) {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN,
                    "[LocalPathSmoothing] Reject smoothed path: geofence violation.");
      return false;
    }
  }

  if (planning_params_.yaw_tangent_correction) {
    for (size_t i = 1; i < current.size(); ++i) {
      Eigen::Vector3d vec(current[i].position.x - current[i - 1].position.x,
                          current[i].position.y - current[i - 1].position.y,
                          current[i].position.z - current[i - 1].position.z);
      if (vec.norm() <= kSmallNorm) {
        continue;
      }
      if (planning_params_.planning_backward) {
        vec = -vec;
      }
      const double yawhalf = 0.5 * std::atan2(vec.y(), vec.x());
      current[i].orientation.x = 0.0;
      current[i].orientation.y = 0.0;
      current[i].orientation.z = std::sin(yawhalf);
      current[i].orientation.w = std::cos(yawhalf);
    }
  }
  current.front().orientation = path_orig.front().orientation;
  current.back().orientation = path_orig.back().orientation;

  path_smoothed = current;
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG,
                "[LocalPathSmoothing] Smooth local path from %d to %d poses.",
                static_cast<int>(path_orig.size()),
                static_cast<int>(path_smoothed.size()));
  return true;
}

bool LocalExplorationPlanner::improveFreePath(const std::vector<geometry_msgs::Pose>& path_orig,
                          std::vector<geometry_msgs::Pose>& path_mod,
                          bool relaxed) {

  if (path_orig.empty()) return false;
  std::vector<geometry_msgs::Pose> path_orig_interp = path_orig;
  double kInterpolationDistance =
      planning_params_.path_interpolation_distance * 2.0;
  std::vector<geometry_msgs::Pose> interp_path_orig;
  if (Trajectory::interpolatePath(path_orig, kInterpolationDistance,
                                  interp_path_orig)) {
    path_orig_interp = interp_path_orig;
  }
  std::vector<geometry_msgs::Pose> path_mod1 = path_orig_interp;

  const double kSegmentLenMin = 0.5;
  bool cont_refine = true;
  while (cont_refine) {
    cont_refine = false;
    if (path_mod1.size() > 2) {
      for (int i = 0; i < (path_mod1.size() - 2); ++i) {
        Eigen::Vector3d p_start(path_mod1[i].position.x,
                                path_mod1[i].position.y,
                                path_mod1[i].position.z);
        Eigen::Vector3d p_int(path_mod1[i + 1].position.x,
                              path_mod1[i + 1].position.y,
                              path_mod1[i + 1].position.z);
        Eigen::Vector3d p_end(path_mod1[i + 2].position.x,
                              path_mod1[i + 2].position.y,
                              path_mod1[i + 2].position.z);
        Eigen::Vector3d segment = p_int - p_start;
        double segment_len = segment.norm();
        if (segment_len < kSegmentLenMin) {
          if ((VoxelStatus::kFree ==
               voxel_map_->getPathStatus(p_start, p_end, robot_box_size_,
                                           false)) &&
              (!planning_params_.geofence_checking_enable ||
               (GeofenceManager::CoordinateStatus::kOK ==
                geofence_manager_->getPathStatus(
                    Eigen::Vector2d(p_start[0], p_start[1]),
                    Eigen::Vector2d(p_end[0], p_end[1]),
                    Eigen::Vector2d(robot_box_size_[0],
                                    robot_box_size_[1]))))) {
            ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Combine nodes to remove short segments.");
            path_mod1.erase(path_mod1.begin() + i + 1);
            cont_refine = true;
            break;
          }
        }
      }
    } else {
      return false;
    }
  }
  feasible_corridor_pcl_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  geometry_msgs::Pose pose0;
  pose0.position.x = path_mod1[0].position.x;
  pose0.position.y = path_mod1[0].position.y;
  pose0.position.z = path_mod1[0].position.z;
  pose0.orientation.x = path_mod1[0].orientation.x;
  pose0.orientation.y = path_mod1[0].orientation.y;
  pose0.orientation.z = path_mod1[0].orientation.z;
  pose0.orientation.w = path_mod1[0].orientation.w;
  path_mod.push_back(pose0);
  bool mod_success = true;

  for (int i = 1; i < path_mod1.size(); ++i) {
    Eigen::Vector3d p0(path_mod1[i - 1].position.x, path_mod1[i - 1].position.y,
                       path_mod1[i - 1].position.z);
    Eigen::Vector3d p0_mod(path_mod[i - 1].position.x,
                           path_mod[i - 1].position.y,
                           path_mod[i - 1].position.z);
    Eigen::Vector3d p1(path_mod1[i].position.x, path_mod1[i].position.y,
                       path_mod1[i].position.z);
    Eigen::Vector3d p1_parallel = p0_mod + p1 - p0;

    Eigen::Vector3d p2;
    bool do_check_p2 = false;
    if (i < path_mod1.size() - 1) {
      do_check_p2 = true;
      p2 = Eigen::Vector3d(path_mod1[i + 1].position.x,
                           path_mod1[i + 1].position.y,
                           path_mod1[i + 1].position.z);
    }

    Eigen::Vector3d p1_target;
    bool seg_free = true;

    bool e1_admissible = false;
    bool e2_admissible = false;
    VoxelStatus vs1 = voxel_map_->getPathStatus(
        p0_mod, p1_parallel, robot_box_size_, true);
      if (vs1 == VoxelStatus::kFree) e1_admissible = true;
      VoxelStatus vs2 =
          voxel_map_->getPathStatus(p0_mod, p1, robot_box_size_, true);
      if (vs2 == VoxelStatus::kFree) e2_admissible = true;
    if (e1_admissible) {
      p1_target = p1_parallel;
    } else if (e2_admissible) {
      p1_target = p1;
    } else {
      ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Segment not free");
      seg_free = false;
    }

    Eigen::Vector3d p1_mod;
    geometry_msgs::Pose pose;

    Eigen::Vector3d p_center;
    p_center = (p0_mod + p1_target) / 2.0;
    Eigen::Vector3d p_dir;
    p_dir = (p1 - p0);
    double radius = p_dir.norm() / 2.0;
    Eigen::Vector3d safety_extension;
    safety_extension = robot_params_.safety_extension;
    if (relaxed)
      safety_extension(1) *= planning_params_.relaxed_corridor_multiplier;
    Eigen::Vector3d local_bbx(2 * (radius + safety_extension[0]),
                              2 * safety_extension[1], 2 * safety_extension[2]);
    std::vector<Eigen::Vector3d> occupied_voxels;
    std::vector<Eigen::Vector3d> free_voxels;
    voxel_map_->extractLocalMapAlongAxis(p_center, p_dir, local_bbx,
                                           occupied_voxels, free_voxels);

    pcl::PointCloud<pcl::PointXYZ>* obstacle_pcl(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (auto& v : occupied_voxels) {
      obstacle_pcl->push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
    }

    bool modification_successful =
        (modifyPath(obstacle_pcl, p0_mod, p1_target, p1_mod));
    if (seg_free && modification_successful) {
      bool e1_admissible = false;
      bool e2_admissible = false;
      VoxelStatus vs1 =
          voxel_map_->getPathStatus(p0_mod, p1_mod, robot_box_size_, true);
        if (vs1 == VoxelStatus::kFree) e1_admissible = true;
        VoxelStatus vs2 =
            voxel_map_->getPathStatus(p1_mod, p2, robot_box_size_, true);
        if (vs2 == VoxelStatus::kFree) e2_admissible = true;

      if (!(e1_admissible) ||
          (planning_params_.geofence_checking_enable &&
           (GeofenceManager::CoordinateStatus::kViolated ==
            geofence_manager_->getPathStatus(
                Eigen::Vector2d(p0_mod[0], p0_mod[1]),
                Eigen::Vector2d(p1_mod[0], p1_mod[1]),
                Eigen::Vector2d(robot_box_size_[0], robot_box_size_[1])))) ||
          (do_check_p2 && (!(e2_admissible) ||
                           (planning_params_.geofence_checking_enable &&
                            (GeofenceManager::CoordinateStatus::kViolated ==
                             geofence_manager_->getPathStatus(
                                 Eigen::Vector2d(p1_mod[0], p1_mod[1]),
                                 Eigen::Vector2d(p2[0], p2[1]),
                                 Eigen::Vector2d(robot_box_size_[0],
                                                 robot_box_size_[1]))))))) {
        p1_mod = p1;
        mod_success = false;
        ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Newly modified path is not collision-free.");
      }
    } else {
      p1_mod = p1;
    }
    pose.position.x = p1_mod.x();
    pose.position.y = p1_mod.y();
    pose.position.z = p1_mod.z();
    path_mod.push_back(pose);
  }
  path_mod[0].orientation.x = path_orig_interp[0].orientation.x;
  path_mod[0].orientation.y = path_orig_interp[0].orientation.y;
  path_mod[0].orientation.z = path_orig_interp[0].orientation.z;
  path_mod[0].orientation.w = path_orig_interp[0].orientation.w;
  if ((mod_success) && (planning_params_.yaw_tangent_correction)) {
    for (int i = 1; i < path_mod.size(); ++i) {
      Eigen::Vector3d vec(path_mod[i].position.x - path_mod[i - 1].position.x,
                          path_mod[i].position.y - path_mod[i - 1].position.y,
                          path_mod[i].position.z - path_mod[i - 1].position.z);
      if (planning_params_.planning_backward) vec = -vec;
      double yawhalf = 0.5 * std::atan2(vec[1], vec[0]);
      path_mod[i].orientation.x = 0.0;
      path_mod[i].orientation.y = 0.0;
      path_mod[i].orientation.z = sin(yawhalf);
      path_mod[i].orientation.w = cos(yawhalf);
    }
  }

  visualization_->visualizePCL(feasible_corridor_pcl_.get());
  return mod_success;
}

void LocalExplorationPlanner::getBestViewpointAngles(StateVec in_state, std::vector<std::pair<StateVec, VolumetricGain>> &out_states)
{
  double rad_to_deg = 180.0 / M_PI;
  
  StateVec state = in_state;
  state(4) = 0.0;

  std::vector<VoxelLog> voxel_logs_og;
  VolumetricGain vgain;
  voxel_map_->getCameraScanStatus(in_state, sensor_params_.sensor[planning_params_.inspection_sensor_list[0]], voxel_logs_og);
  for(int i = 0; i < voxel_logs_og.size(); ++i) {
    VoxelLog log = voxel_logs_og[i];

    if (global_space_params_.isInsideSpace(log.voxel_center)) {
      bool no_gain_zone_cleared = true;
      if (use_no_gain_space_) {
        for (auto& zone : no_gain_zones_) {
          if (zone.isInsideSpace(log.voxel_center)) {
            no_gain_zone_cleared = false;
            break;
          }
        }
      }
      if(no_gain_zone_cleared) {
        vgain.unseen_voxel_hash_keys.insert(log.voxel_hash);
      }
    }
  }
  if(vgain.unseen_voxel_hash_keys.size())
    out_states.push_back(std::make_pair(in_state, vgain));

  SensorParamsBase og_actuated_sensor = sensor_params_.sensor[planning_params_.inspection_sensor_list[0]];
  SensorParamsBase actuated_sensor = og_actuated_sensor;
  Eigen::Vector2d state_yaw_dir(std::cos(state(3)), std::sin(state(3)));
  Eigen::Vector2i res_deg(actuated_sensor.resolution[0]*rad_to_deg, actuated_sensor.resolution[1]*rad_to_deg);
  
  int vert_angle_limit = (actuated_sensor.rot_lims[1] - actuated_sensor.rot_lims[0]) * rad_to_deg;
  if(vert_angle_limit < res_deg[1])
  {
    vert_angle_limit = res_deg[1]+1;
  }
  
  actuated_sensor.fov[0] = 2.0 * M_PI;
  actuated_sensor.fov[1] = std::min((actuated_sensor.rot_lims[1] - actuated_sensor.rot_lims[0]) + actuated_sensor.fov[1], M_PI);
  int min_rot_lim_deg = actuated_sensor.rot_lims[0] * rad_to_deg;
  int max_rot_lim_deg = actuated_sensor.rot_lims[1] * rad_to_deg;
  Eigen::Vector2i num_rays(360/(res_deg[0]), actuated_sensor.fov[1] * 180 /(M_PI * res_deg[1]));
  Eigen::Vector2i num_rays_sensor((og_actuated_sensor.fov[0]*rad_to_deg)/(res_deg[0]), (og_actuated_sensor.fov[1]*rad_to_deg)/(res_deg[1]));

  int min_ray_deg = min_rot_lim_deg - og_actuated_sensor.fov[1]*rad_to_deg/2.0; // Min angle seen by the sensor
  int min_ray_ind = std::max(min_ray_deg, -90) / res_deg[1]; // Min index (signed) seen by the sensor
  int min_rot_lim_ray = min_rot_lim_deg / res_deg[1] - min_ray_ind; // Index (unsigned) of the ray corresponding to min_rot of sensor
  int max_rot_lim_ray = max_rot_lim_deg / res_deg[1] - min_ray_ind; // Index (unsigned) of the ray corresponding to max_rot of sensor

  actuated_sensor.type = SensorType::kSpherical;
  
  actuated_sensor.updateFrustumEndpoints();
  std::vector<Eigen::Vector3d> ray_endpoints;
  actuated_sensor.getFrustumEndpoints(state, ray_endpoints);
  std::vector<VoxelLog> voxel_logs;
  voxel_map_->getCameraScanStatus(state, actuated_sensor, voxel_logs);

  Eigen::MatrixXi angle_map(num_rays[1], num_rays[0]);
  angle_map.setZero();
  std::set<size_t> all_seen_keys;
  std::vector<std::vector<std::vector<size_t>>> angle_keys_map;
  std::vector<std::vector<size_t>> empty_row;
  empty_row.resize(num_rays[0]);
  for(int i=0; i<num_rays[1]; ++i)
    angle_keys_map.push_back(empty_row);

  for(int i = 0; i < voxel_logs.size(); ++i) {

    VoxelLog log = voxel_logs[i];
    int curr_num_keys_seen = all_seen_keys.size();
    all_seen_keys.insert(log.voxel_hash);
    if(all_seen_keys.size() <= curr_num_keys_seen)
      continue;


    if (global_space_params_.isInsideSpace(log.voxel_center)) {
      bool no_gain_zone_cleared = true;
      if (use_no_gain_space_) {
        for (auto& zone : no_gain_zones_) {
          if (zone.isInsideSpace(log.voxel_center)) {
            no_gain_zone_cleared = false;
            break;
          }
        }
      }
      if(no_gain_zone_cleared) {

        Eigen::Vector3d dir = log.voxel_center - state.head(3);
        double elev, azim;

        if(std::abs(dir.z()) <= 0.001)
          elev = 0.0;
        else
        {
          elev = std::atan2(-dir.z(), dir.head(2).norm());
        }
        if(elev > M_PI / 2.0)
        {
          elev = M_PI_2;
        }
        else if(elev < -M_PI / 2.0)
        {
          elev = -M_PI_2;
        }
        int elev_int = std::round(elev * 180.0 / (M_PI*res_deg[1])) - min_ray_ind;
        if(elev_int < 0)
        {
          elev_int = 0;
        }
        else if(elev_int >= num_rays[1])
        {
          elev_int = num_rays[1] - 1;
        }

        azim = std::atan2(dir.y(), dir.x());
        int azim_int = std::round(azim * 180.0 / (M_PI*res_deg[0])) + num_rays[0]/2;

        if(azim_int >= num_rays[0])
        {
          azim_int -= num_rays[0];
        }
        else if(azim_int < 0)
        {
          azim_int += num_rays[0];
        }

        ++angle_map(elev_int, azim_int);

        angle_keys_map[elev_int][azim_int].push_back(log.voxel_hash);

      }
    }
  }



  int i=0;
  while (i<num_rays[0])
  {
    int i_incr = 1;
    int j=min_rot_lim_ray;
    while(j<=max_rot_lim_ray)
    {
      
        VolumetricGain vgain;
        for(int cs=i; cs<i+num_rays_sensor[0]; ++cs)
        {
          if(cs >= num_rays[0])
            break;
          for(int rs=j-num_rays_sensor[1]/2; rs<j+num_rays_sensor[1]/2; ++rs)
          {
            
            int r_ind_corrected = rs;
            int c_ind_corrected = cs;
            if(rs < 0)
            {
              r_ind_corrected = -rs;
              c_ind_corrected += num_rays[0]/2;
              if(c_ind_corrected >= num_rays[0])
                c_ind_corrected = c_ind_corrected - num_rays[0];
              
            }
            else if(rs > num_rays[1])
            {
              r_ind_corrected = 2 * num_rays[1] - 2 - rs;
              c_ind_corrected += num_rays[0]/2;
              if(c_ind_corrected >= num_rays[0])
                c_ind_corrected = c_ind_corrected - num_rays[0];
              
            }
            else if(rs == num_rays[1])
            {
              r_ind_corrected = num_rays[1] - 1;
            }

            if(c_ind_corrected >= num_rays[0])
            {
              c_ind_corrected = num_rays[0]-1;
              
            }
            

            angle_map(r_ind_corrected, c_ind_corrected) = 0;
            vgain.unseen_voxel_hash_keys.insert(angle_keys_map[r_ind_corrected][c_ind_corrected].begin(), angle_keys_map[r_ind_corrected][c_ind_corrected].end());
          }
        }
        if(!vgain.unseen_voxel_hash_keys.empty())
        {
          StateVec new_state = state;
          new_state(3) = (i - num_rays[0]/2) * (M_PI*res_deg[0]) / 180.0 + og_actuated_sensor.fov[0]/2.0;
          new_state(4) = (j + min_ray_ind) * (M_PI*res_deg[1]) / 180.0;
          out_states.push_back(std::make_pair(new_state, vgain));

          i_incr = num_rays_sensor[0];
          j+= num_rays_sensor[1];
        }
        else
        {
          ++j;
        }

    }
    i += i_incr;
  }
}



void LocalExplorationPlanner::generateGridSamplesBasic(std::vector<int> &viewpoint_ids) {
  std::vector<std::pair<Eigen::Vector3d, double>> selected_points;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d max(-9999, -99999, -9999), min(9999, 9999, 9999);
  for(double x = inspection_bound_.min_val.x(); x <= inspection_bound_.max_val.x(); x+=planning_params_.inspection_xy_spacing) {
    for(double y = inspection_bound_.min_val.y(); y <= inspection_bound_.max_val.y(); y+=planning_params_.inspection_xy_spacing) {
      for(double z = inspection_bound_.min_val.z() + planning_params_.inspection_z_spacing; z <= inspection_bound_.max_val.z() - planning_params_.inspection_z_spacing; z+=planning_params_.inspection_z_spacing) {
        Eigen::Vector3d sampled_point(x,y,z);
        double dist = voxel_map_->getPointDistance(sampled_point);
        if(dist >= planning_params_.inspection_thr_esdf_dist - planning_params_.inspection_xy_spacing && dist <= planning_params_.inspection_thr_esdf_dist + planning_params_.inspection_xy_spacing) {
          selected_points.push_back(std::make_pair(sampled_point, dist));
          max = max.cwiseMax(sampled_point);
          min = min.cwiseMin(sampled_point);
        }
      }
    }
  }
  centroid = (max + min) / 2.0;

  viewpoint_ids.clear();
  double edge_len_max_og = planning_params_.edge_length_max;
  double nearest_range_max_og = planning_params_.nearest_range_max;
  planning_params_.edge_length_max *= 3.0;
  planning_params_.nearest_range_max *= 3.0;
  planning_params_.nearest_range *= 3.0;
  for(auto sample : selected_points) {
    StateVec new_state;
    Eigen::Vector3d dir = sample.first - centroid;
    dir.z() = 0.0;
    sample.first -= dir.normalized() * (planning_params_.inspection_target_viewing_range - sample.second);
    new_state.head(3) = sample.first;
    new_state(3) = std::atan2(dir.y(), dir.x());
    bool success = false;
    if(voxel_map_->getBoxStatus(new_state.head(3), robot_box_size_, true) == VoxelStatus::kFree) {
      success = true;
    }
    else {
      Eigen::Vector3d dir_normed = dir.normalized();
      for(double dr=voxel_map_->getResolution(); dr<(planning_params_.inspection_target_viewing_range - sample.second); dr+=voxel_map_->getResolution()) {
        new_state.head(3) += dir_normed * dr;
        if(voxel_map_->getBoxStatus(new_state.head(3), robot_box_size_, true) == VoxelStatus::kFree) {
          success = true;
          sample.first = new_state.head(3);
          break;
        }
      }
    }
    if(success) {
      if(global_space_params_.isInsideSpace(sample.first)) {
        double edge_len_min_og = planning_params_.edge_length_min;
        planning_params_.edge_length_min = -0.01;
        Vertex new_v(-1, new_state);
        ExpandGraphReport rep;
        expandGraph(local_graph_, new_v, rep);
        if(rep.status == ExpandGraphStatus::kSuccess) {
          viewpoint_ids.push_back(rep.vertex_added->id);
		if(false){
          //if(planning_params_.use_flipped_yaw) {
            StateVec fliped_state;
            fliped_state = new_state;
            fliped_state[3] = M_PI + fliped_state[3];
            truncateYaw(fliped_state[3]);
            ExpandGraphReport fliped_rep;
            Vertex fliped_v(-1, fliped_state);
            expandGraph(local_graph_, fliped_v, fliped_rep);
            if(fliped_rep.status == ExpandGraphStatus::kSuccess) {
              viewpoint_ids.push_back(fliped_rep.vertex_added->id);
            }
          }
        }
        planning_params_.edge_length_min = edge_len_min_og;
      }
    }
  }
  planning_params_.edge_length_max = edge_len_max_og;
  planning_params_.nearest_range_max = nearest_range_max_og;
  planning_params_.nearest_range = nearest_range_max_og;
}

void LocalExplorationPlanner::generateGridSamples(std::vector<int> &viewpoint_ids) {
  double max_edge_len = 1.2 * std::max(planning_params_.inspection_xy_spacing, planning_params_.inspection_z_spacing);
  std::vector<std::pair<Eigen::Vector3d, double>> selected_points;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d max(-9999, -99999, -9999), min(9999, 9999, 9999);
  for(double x = inspection_bound_.min_val.x(); x <= inspection_bound_.max_val.x(); x+=planning_params_.inspection_xy_spacing) {
    for(double y = inspection_bound_.min_val.y(); y <= inspection_bound_.max_val.y(); y+=planning_params_.inspection_xy_spacing) {
      for(double z = inspection_bound_.min_val.z() + planning_params_.inspection_z_spacing; z <= inspection_bound_.max_val.z() - planning_params_.inspection_z_spacing; z+=planning_params_.inspection_z_spacing) {
        Eigen::Vector3d sampled_point(x,y,z);
        double dist = voxel_map_->getPointDistance(sampled_point);
        if(dist > voxel_map_->getResolution() && dist <= planning_params_.inspection_thr_esdf_dist + voxel_map_->getResolution()) {
          selected_points.push_back(std::make_pair(sampled_point, dist));
          max = max.cwiseMax(sampled_point);
          min = min.cwiseMin(sampled_point);
        }
      }
    }
  }
  centroid = (max + min) / 2.0;

  viewpoint_ids.clear();
  double edge_len_max_og = planning_params_.edge_length_max;
  double nearest_range_max_og = planning_params_.nearest_range_max;
  planning_params_.edge_length_max = max_edge_len;
  planning_params_.nearest_range_max = max_edge_len;
  planning_params_.nearest_range = max_edge_len;
  int min_ang_mult = (int)((sensor_params_.sensor[planning_params_.inspection_sensor_list[0]].rot_lims[0] - M_PI/8) / M_PI_4);
  int max_ang_mult = (int)((sensor_params_.sensor[planning_params_.inspection_sensor_list[0]].rot_lims[1] + M_PI/8) / M_PI_4);
  double max_range = planning_params_.inspection_target_viewing_range;
  for(auto sample : selected_points) {
    StateVec new_state;
    Eigen::Vector3d dir = sample.first - centroid;
    
    double min_dist = max_range * 1.5;
    Eigen::Vector3d closest_occ_voxel;
    Eigen::Vector3d cumulative_dir(0.0, 0.0, 0.0);
    int num_hits = 0;
    bool closest_voxel_found = false;
    for(int elev_ind = min_ang_mult; elev_ind <= max_ang_mult; ++elev_ind)
    {
      for(int azim_ind = -4; azim_ind < 4; ++azim_ind)
      {
        double elev = elev_ind * M_PI_4;
        double azim = azim_ind * M_PI_4;
        Eigen::Vector3d ray_dir(max_range * std::cos(elev) * std::cos(azim),
                                max_range * std::cos(elev) * std::sin(azim),
                                -max_range * std::sin(elev));
        Eigen::Vector3d ray_end = ray_dir + sample.first;
        Eigen::Vector3d end_voxel;
        double tsdf_dist;
        VoxelStatus vs = voxel_map_->getRayStatus(sample.first, ray_end, false, end_voxel, tsdf_dist);
        if(vs == VoxelStatus::kOccupied)
        {
          cumulative_dir += ray_dir;
          ++num_hits;
          double dist = (end_voxel - sample.first).norm();
          if(dist < min_dist)
          {
            min_dist = dist;
            closest_occ_voxel = end_voxel;
            closest_voxel_found = true;
            sample.second = min_dist;
          }
        }
      }
    }
    if(closest_voxel_found)
    {
      cumulative_dir /= num_hits;
      dir = cumulative_dir;
    }
    else
    {
      continue;
    }
    sample.first -= dir.normalized() * (planning_params_.inspection_target_viewing_range - sample.second);
    new_state.head(3) = sample.first;
    new_state(3) = std::atan2(dir.y(), dir.x()); /* &*& */
    new_state(4) = std::atan2(-dir.z(), dir.head(2).norm());
    bool success = false;
    if(voxel_map_->getBoxStatus(new_state.head(3), robot_box_size_, true) == VoxelStatus::kFree) {
      success = true;
    }
    else {
      Eigen::Vector3d dir_normed = dir.normalized();
      for(double dr=voxel_map_->getResolution(); dr<(planning_params_.inspection_target_viewing_range - sample.second); dr+=voxel_map_->getResolution()) {
        new_state.head(3) += dir_normed * dr;
        if(voxel_map_->getBoxStatus(new_state.head(3), robot_box_size_, true) == VoxelStatus::kFree) {
          success = true;
          sample.first = new_state.head(3);
          break;
        }
      }
    }
    if(success) {
      if(global_space_params_.isInsideSpace(sample.first)) {
        double edge_len_min_og = planning_params_.edge_length_min;
        planning_params_.edge_length_min = -0.01;

        std::vector<std::pair<StateVec, VolumetricGain>> viewpoint_pitches;
        getBestViewpointAngles(new_state, viewpoint_pitches);
        if(viewpoint_pitches.empty())
          continue;

        Vertex new_v(-1, viewpoint_pitches[0].first);
        ExpandGraphReport rep;
        expandGraph(local_graph_, new_v, rep);
        if(rep.status == ExpandGraphStatus::kSuccess) {
          viewpoint_ids.push_back(rep.vertex_added->id);
          int sub_id = 0;
          for(int v_id = 1; v_id < viewpoint_pitches.size(); ++v_id)
          {
            Vertex* sub_v = new Vertex(sub_id++, viewpoint_pitches[v_id].first);
            sub_v->vol_gain = viewpoint_pitches[v_id].second;
            rep.vertex_added->orientation_sub_vertices.push_back(sub_v);
          }
        }
        planning_params_.edge_length_min = edge_len_min_og;
      }
    }
  }
  planning_params_.edge_length_max = edge_len_max_og;
  planning_params_.nearest_range_max = nearest_range_max_og;
  planning_params_.nearest_range = nearest_range_max_og;

}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::getInspectionPathBasic() {
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Inspection bounds:");
  if(global_verbosity >= Verbosity::DEBUG) {
    std::cout << "Min val: " << inspection_bound_.min_val.transpose() << std::endl;
    std::cout << "Max val: " << inspection_bound_.max_val.transpose() << std::endl;
  }

  bool og_param = planning_params_.use_current_state;
  planning_params_.use_current_state = true;
  int local_planner_graph_vertices_og = planning_params_.num_vertices_max;
  planning_params_.num_vertices_max = planning_params_.inspection_graph_vertices;
  planning_num_vertices_max_ = planning_params_.inspection_graph_vertices;
  reset();
  planning_params_.use_current_state = og_param;

  Timer graph_timer;
  
  GraphStatus status = buildGraph();
  if(status != GraphStatus::OK)
  {
    std::vector<geometry_msgs::Pose> empty_path;
    return empty_path;
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Num of vertices in connecting graph: %d", local_graph_->getNumVertices());
  
  std::vector<int> viewpoint_ids;
  generateGridSamplesBasic(viewpoint_ids);  
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Total Num of vertices graph: %d", local_graph_->getNumVertices());

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Number of Viewpoints: %zu", viewpoint_ids.size());
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: Graph building: %f", graph_timer.endTimer());

  planning_params_.num_vertices_max = local_planner_graph_vertices_og;
  planning_num_vertices_max_ = planning_params_.num_vertices_max;

  local_graph_->findShortestPaths(local_graph_rep_);
  add_frontiers_to_global_graph_ = true;
  Timer gain_timer;
  std::set<size_t> cumulative_unseen_voxels;
  std::vector<std::pair<int, std::set<std::size_t>>> remaining_vertices;
  std::vector<int> all_vertex_ids;
  Eigen::Vector3d centroid = (inspection_bound_.max_val + inspection_bound_.min_val) / 2.0;
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Centroid: %f, %f, %f", centroid.x(), centroid.y(), centroid.z());
  for(int i : viewpoint_ids) {
    all_vertex_ids.push_back(i);
    Vertex* v = local_graph_->getVertex(i);
    if(true) {
      
      computeInspectionGainRayModel(v);

      remaining_vertices.push_back(std::make_pair(i, v->vol_gain.unseen_voxel_hash_keys));
      cumulative_unseen_voxels.merge(v->vol_gain.unseen_voxel_hash_keys);
    }
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: Gain calculation: %f", gain_timer.endTimer());

  visualization_->visualizeGraph(local_graph_);

  Timer sorting_timer;
  int min_coverage = (int)(planning_params_.min_coverage_percentage * cumulative_unseen_voxels.size());
  int current_coverage = 0;  // Number of unseen voxels seen by all vertices selected so far
  std::vector<std::pair<int, std::set<std::size_t>>> selected_vertices;
  std::set<std::size_t> current_visibility;
  while(current_coverage <= min_coverage && !remaining_vertices.empty()) {
    std::sort(remaining_vertices.begin(), remaining_vertices.end(),
					  [](const std::pair<int, std::set<std::size_t>> &v1, const std::pair<int, std::set<std::size_t>> &v2)
					  {
						  return v1.second.size() > v2.second.size();
					  });
    auto vert_selected = remaining_vertices[0];
    remaining_vertices.erase(remaining_vertices.begin());
    std::set<std::size_t> vert_visibility = vert_selected.second;
    current_visibility.merge(vert_visibility);
    selected_vertices.push_back(vert_selected);
    current_coverage = current_visibility.size();

    std::set<std::size_t> vert_sel_visibility = vert_selected.second;
    for (int i = 0; i < remaining_vertices.size(); ++i)
    {
      std::vector<std::size_t> remaining_visibility;
      std::set_difference(remaining_vertices[i].second.begin(), remaining_vertices[i].second.end(),
                vert_sel_visibility.begin(), vert_sel_visibility.end(),
                std::back_inserter(remaining_visibility));
      remaining_vertices[i].second.clear();
      remaining_vertices[i].second.insert(remaining_visibility.begin(), remaining_visibility.end());
    }

    if(selected_vertices.size() > planning_params_.max_inspection_vertices) {
      ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Max inspection vertices limit reached");
      break;
    }
  }

  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Selected vertices: %zu, total vertices: %d", selected_vertices.size(), local_graph_->getNumVertices());
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Achieved coverage: %d, Max possible coverage: %zu, percentage: %f", current_coverage, cumulative_unseen_voxels.size(), (100.0 * current_coverage) / cumulative_unseen_voxels.size());

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: Sorting: %f", sorting_timer.endTimer());

  
  Timer tsp_timer;  
  std::map<int, ShortestPathsReport> path_rep_map;
  viewpoint_ids.push_back(0);  // Adding current node

  std::vector<geometry_msgs::Pose> ret_path;

  if(!selected_vertices.empty()) {
    Timer T1;
    std::vector<int> selected_vertex_ids;
    T1.reset();
    for(auto p : selected_vertices) {
      ShortestPathsReport rep;
      local_graph_->findShortestPaths(p.first, rep);
      path_rep_map[p.first] = rep;
      selected_vertex_ids.push_back(p.first);
    }
    {
      ShortestPathsReport rep;
      local_graph_->findShortestPaths(0, rep);
      path_rep_map[0] = rep;
    }
    selected_vertex_ids.push_back(0);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: Shortest path calculations: %f", T1.endTimer());
    T1.reset();
    std::vector<std::vector<int>> cost_matrix;
		generateCostMatrix(selected_vertex_ids, cost_matrix, path_rep_map);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: Cost matrix: %f", T1.endTimer());

    visualization_->visualizeGraphVertices(local_graph_, selected_vertex_ids);
    
    T1.reset();
    std::vector<int> tsp_order = doTSP(selected_vertex_ids, cost_matrix);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: doTSP: %f", T1.endTimer());

    std::vector<int> reordered_tsp_sol;
    if(tsp_order[0] == 0) {
      reordered_tsp_sol = tsp_order;
    }
    else if(tsp_order.back() == 0) {
      reordered_tsp_sol.push_back(0);
      reordered_tsp_sol.insert(reordered_tsp_sol.begin()+1, tsp_order.begin(), tsp_order.end()-1);
    }
    else {
      auto it = std::find(tsp_order.begin(), tsp_order.end(), 0);
      if(it == tsp_order.end()) {
        ROS_WARN_COND(global_verbosity >= Verbosity::ERROR, "Root vertex not in TSP order");
      }
      reordered_tsp_sol.push_back(0);
      reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), it+1, tsp_order.end());
      reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), tsp_order.begin(), it);
    }
    tsp_order = reordered_tsp_sol;
		
    T1.reset();
    std::vector<geometry_msgs::Pose> current_path;
		current_path = connectTSPOrder(tsp_order, path_rep_map, local_graph_);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: connectTSPOrder: %f", T1.endTimer());

    visualization_->visualizeRefPath(current_path);
    ret_path = current_path;
  }
  else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No vertex selected");
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: %f", tsp_timer.endTimer());

  t2 = std::chrono::high_resolution_clock::now();

  ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Inspection path calculation time: %f", std::chrono::duration<double, std::milli>(t2 - t1).count());

  return ret_path;
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::getInspectionPath() {
  planning_params_.annotate_map_with_camera = true;
  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "Inspection bounds:");
  if(global_verbosity >= Verbosity::DEBUG) {
    std::cout << "Min val: " << inspection_bound_.min_val.transpose() << std::endl;
    std::cout << "Max val: " << inspection_bound_.max_val.transpose() << std::endl;
  }


  bool og_param = planning_params_.use_current_state;
  planning_params_.use_current_state = true;
  int local_planner_graph_vertices_og = planning_params_.num_vertices_max;
  planning_params_.num_vertices_max = planning_params_.inspection_graph_vertices;
  planning_num_vertices_max_ = planning_params_.inspection_graph_vertices;
  reset();
  planning_params_.use_current_state = og_param;

  Timer graph_timer;
  
  GraphStatus status = buildGraph();
  if(status != GraphStatus::OK)
  {
    std::vector<geometry_msgs::Pose> empty_path;
    return empty_path;
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Num of vertices in connecting graph: %d", local_graph_->getNumVertices());
  
  std::vector<int> viewpoint_ids;
  generateGridSamples(viewpoint_ids);  
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Total Num of vertices graph: %d", local_graph_->getNumVertices());

  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Number of Viewpoints: %zu", viewpoint_ids.size());
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[Inspection]: Graph building: %f", graph_timer.endTimer());

  visualization_->visualizeGraph(local_graph_);
  visualization_->visualizeGraphVertices(local_graph_, viewpoint_ids);


  planning_params_.num_vertices_max = local_planner_graph_vertices_og;
  planning_num_vertices_max_ = planning_params_.num_vertices_max;

  local_graph_->findShortestPaths(local_graph_rep_);
  add_frontiers_to_global_graph_ = true;
  Timer gain_timer;
  std::set<size_t> cumulative_unseen_voxels;
  std::vector<std::tuple<int, int, std::set<size_t>>> remaining_vertices;
  std::vector<int> all_vertex_ids;

  Eigen::Vector3d centroid = (inspection_bound_.max_val + inspection_bound_.min_val) / 2.0;
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Centroid: %f, %f, %f", centroid.x(), centroid.y(), centroid.z());

  int max_voxels_per_viewpoint 
    = (int)
      ((sensor_params_.sensor[planning_params_.inspection_sensor_list[0]].fov[0] * sensor_params_.sensor[planning_params_.inspection_sensor_list[0]].fov[1] * 
        sensor_params_.sensor[planning_params_.inspection_sensor_list[0]].max_range * sensor_params_.sensor[planning_params_.inspection_sensor_list[0]].max_range)
        / (voxel_map_->getResolution() * voxel_map_->getResolution()));
  for(int i : viewpoint_ids) {
    all_vertex_ids.push_back(i);
    Vertex* v = local_graph_->getVertex(i);
    for(auto subv : v->orientation_sub_vertices)
    {
      if(subv->vol_gain.unseen_voxel_hash_keys.size() < 0.1 * max_voxels_per_viewpoint)
        continue;
      remaining_vertices.push_back(std::make_tuple(v->id, subv->id, subv->vol_gain.unseen_voxel_hash_keys));
      cumulative_unseen_voxels.merge(subv->vol_gain.unseen_voxel_hash_keys);
    }

  }
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "[Inspection]: Gain calculation: %f", gain_timer.endTimer());

  Timer sorting_timer;
  int min_coverage = (int)(planning_params_.min_coverage_percentage * cumulative_unseen_voxels.size());
  int current_coverage = 0;  // Number of unseen voxels seen by all vertices selected so far
  std::vector<int> selected_vertices;
  std::set<int> selected_vertices_set;
  std::map<int, std::vector<int>> selected_subvertices;
  std::set<std::size_t> current_visibility;
  while(current_coverage <= min_coverage && !remaining_vertices.empty()) {
    std::sort(remaining_vertices.begin(), remaining_vertices.end(),
					  [](const std::tuple<int, int, std::set<std::size_t>> &v1, const std::tuple<int, int, std::set<std::size_t>> &v2)
					  {
              return std::get<2>(v1).size() > std::get<2>(v2).size();
					  });
    auto vert_selected = remaining_vertices[0];
    remaining_vertices.erase(remaining_vertices.begin());
    std::set<std::size_t> vert_visibility = std::get<2>(vert_selected);
    current_visibility.merge(vert_visibility);
    selected_vertices_set.insert(std::get<0>(vert_selected));
    selected_subvertices[std::get<0>(vert_selected)].push_back(std::get<1>(vert_selected));
    current_coverage = current_visibility.size();

    std::set<std::size_t> vert_sel_visibility = std::get<2>(vert_selected);
    for (int i = 0; i < remaining_vertices.size(); ++i)
    {
      std::vector<std::size_t> remaining_visibility;
      std::set_difference(std::get<2>(remaining_vertices[i]).begin(), std::get<2>(remaining_vertices[i]).end(),
                vert_sel_visibility.begin(), vert_sel_visibility.end(),
                std::back_inserter(remaining_visibility));
      
      std::get<2>(remaining_vertices[i]).clear();
      std::get<2>(remaining_vertices[i]).insert(remaining_visibility.begin(), remaining_visibility.end());
    }

    if(selected_vertices.size() > planning_params_.max_inspection_vertices) {
      ROS_INFO_COND(global_verbosity >= Verbosity::WARN, "Max inspection vertices limit reached");
      break;
    }
  }

  for(auto v : selected_vertices_set)
    selected_vertices.push_back(v);

  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Selected vertices: %zu, total vertices: %d", selected_vertices.size(), local_graph_->getNumVertices());
  ROS_INFO_COND(global_verbosity >= Verbosity::INFO, "Achieved coverage: %d, Max possible coverage: %zu, percentage: %f", current_coverage, cumulative_unseen_voxels.size(), (100.0 * current_coverage) / cumulative_unseen_voxels.size());

  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: Sorting: %f", sorting_timer.endTimer());

  
  Timer tsp_timer;  
  std::map<int, ShortestPathsReport> path_rep_map;
  viewpoint_ids.push_back(0);  // Adding current node

  std::vector<geometry_msgs::Pose> ret_path;

  if(!selected_vertices.empty()) {
    Timer T1;
    T1.reset();

    for(auto p : selected_vertices) {
      ShortestPathsReport rep;
      local_graph_->findShortestPaths(p, rep);
      path_rep_map[p] = rep;
    }
    {
      ShortestPathsReport rep;
      local_graph_->findShortestPaths(0, rep);
      path_rep_map[0] = rep;
    }
    selected_vertices.push_back(0);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: Shortest path calculations: %f", T1.endTimer());
    T1.reset();
    std::vector<std::vector<int>> cost_matrix;
		generateCostMatrix(selected_vertices, cost_matrix, path_rep_map);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: Cost matrix: %f", T1.endTimer());

    visualization_->visualizeGraphVertices(local_graph_, selected_vertices);
    std::vector<StateVec> viewpoints_vis;
    for(auto it : selected_subvertices)
    {
      for(auto subv_id : it.second)
      {
        viewpoints_vis.push_back(local_graph_->getVertex(it.first)->orientation_sub_vertices[subv_id]->state);
      }
    }
    visualization_->visualizeViewpoints(viewpoints_vis);

    T1.reset();
    std::vector<int> tsp_order = doTSP(selected_vertices, cost_matrix);
    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: doTSP: %f", T1.endTimer());

    std::vector<int> reordered_tsp_sol;
    if(tsp_order[0] == 0) {
      reordered_tsp_sol = tsp_order;
    }
    else if(tsp_order.back() == 0) {
      reordered_tsp_sol.push_back(0);
      reordered_tsp_sol.insert(reordered_tsp_sol.begin()+1, tsp_order.begin(), tsp_order.end()-1);
    }
    else {
      auto it = std::find(tsp_order.begin(), tsp_order.end(), 0);
      if(it == tsp_order.end()) {
        ROS_WARN_COND(global_verbosity >= Verbosity::ERROR, "Root vertex not in TSP order");
      }
      reordered_tsp_sol.push_back(0);
      reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), it+1, tsp_order.end());
      reordered_tsp_sol.insert(reordered_tsp_sol.begin()+reordered_tsp_sol.size(), tsp_order.begin(), it);
    }
    tsp_order = reordered_tsp_sol;

    T1.reset();
    std::vector<geometry_msgs::Pose> current_path;

    std::vector<std::pair<int, std::vector<int>>> tsp_nodes_with_subv;
    for(auto n_id : tsp_order)
    {
      tsp_nodes_with_subv.push_back(std::make_pair(n_id, selected_subvertices[n_id]));
    }
    current_path = connectTSPOrderWithSubvertices(tsp_nodes_with_subv, path_rep_map, local_graph_);

    ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: connectTSPOrder: %f", T1.endTimer());

    visualization_->visualizeRefPath(current_path);

    ret_path = current_path;
  }
  else {
    ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "No vertex selected");
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::DEBUG, "[Inspection]: TSP: %f", tsp_timer.endTimer());

  t2 = std::chrono::high_resolution_clock::now();

  ROS_WARN_COND(global_verbosity >= Verbosity::INFO, "Inspection path calculation time: %f", std::chrono::duration<double, std::milli>(t2 - t1).count());

  return ret_path;
}


void LocalExplorationPlanner::generateCostMatrix(std::vector<int> nodes, std::vector<std::vector<int>> &cost_matrix, std::map<int, ShortestPathsReport> &path_rep_map)
{
	std::vector<int> temp_cost_mat_row(nodes.size(), 9999999);
	std::vector<std::vector<int>> temp_cost_mat(nodes.size(), temp_cost_mat_row);
	cost_matrix = temp_cost_mat;

	for (int i = 0; i < nodes.size(); ++i)
	{
		for (int j = 0; j < nodes.size(); ++j)
		{
			if (i == j)
			{
				cost_matrix[i][j] = 0;
			}
			else
			{
        if(nodes[j] == 0)
        {
          cost_matrix[i][j] = 0;
        }
        else
        {
          if(planning_params_.use_flipped_yaw) {
            double d_yaw = local_graph_->getVertex(nodes[i])->state[3] - local_graph_->getVertex(nodes[j])->state[3];
            truncateYaw(d_yaw);
            cost_matrix[i][j] = std::max(path_rep_map[nodes[i]].distance_map[nodes[j]]/planning_params_.v_max, d_yaw / planning_params_.yaw_rate_max);
          }
          else {
            cost_matrix[i][j] = path_rep_map[nodes[i]].distance_map[nodes[j]];
          }
        }
			}
		}
	}
}
double LocalExplorationPlanner::calculateTourCost(const std::vector<int>& tour, const std::vector<std::vector<double>>& costMatrix) {
    double total = 0.0;
    int n = tour.size();
    for (int i = 0; i < n; ++i) {
        total += costMatrix[tour[i]][tour[(i + 1) % n]];
    }
    return total;
}
void LocalExplorationPlanner::twoOptSwap(std::vector<int>& tour, int i, int k) {
    std::reverse(tour.begin() + i, tour.begin() + k + 1);
}
std::vector<int> LocalExplorationPlanner::solveTSP(const std::vector<std::vector<double>>& costMatrix) {
    int n = costMatrix.size();
    std::vector<int> tour(n);
    for (int i = 0; i < n; ++i) tour[i] = i;

    bool improved = true;
    double bestCost = calculateTourCost(tour, costMatrix);

    while (improved) {
        improved = false;
        for (int i = 1; i < n - 1; ++i) {
            for (int k = i + 1; k < n; ++k) {
                std::vector<int> newTour = tour;
                twoOptSwap(newTour, i, k);
                double newCost = calculateTourCost(newTour, costMatrix);
                if (newCost < bestCost) {
                    tour = newTour;
                    bestCost = newCost;
                    improved = true;
                }
            }
        }
    }

    return tour;
}

std::vector<int> LocalExplorationPlanner::doTSP(std::vector<int> &nodes, std::vector<std::vector<int>> &cost_matrix)
{
	std::vector<int> tsp_order;
  std::vector<std::vector<double>> cost_matrix_double;
  cost_matrix_double.reserve(cost_matrix.size());

  for (const auto& row : cost_matrix) {
      std::vector<double> doubleRow;
      doubleRow.reserve(row.size());

      for (int val : row) {
          doubleRow.push_back(static_cast<double>(val));
      }

      cost_matrix_double.push_back(std::move(doubleRow));
  }
  std::vector<int> sol = solveTSP(cost_matrix_double);
	for (int i = 0; i < sol.size(); i++)
	{
		int id = nodes[sol[i]];
		tsp_order.push_back(id);
	}

	return tsp_order;
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::connectTSPOrder(std::vector<int> &tsp_nodes, std::map<int, ShortestPathsReport> &path_rep_map, std::shared_ptr<CognitiveGraph> graph)
{
	std::vector<geometry_msgs::Pose> tsp_path;
	for (int i = 0; i < tsp_nodes.size() - 1; ++i)
	{
		std::vector<StateVec> current_path_segment_vec;
		std::vector<geometry_msgs::Pose> current_path_segment;
		graph->getShortestPath(tsp_nodes[i + 1], path_rep_map[tsp_nodes[i]], true, current_path_segment_vec);
		convert(current_path_segment_vec, current_path_segment);
		linearlyInterpolateYaw(current_path_segment);
		for (int j = 0; j < current_path_segment.size() - 1; ++j)
		{
			tsp_path.push_back(current_path_segment[j]);
		}
	}

	geometry_msgs::Pose last_pose;
	convert(graph->getVertex(tsp_nodes.back())->state, last_pose);
	tsp_path.push_back(last_pose);

	return tsp_path;
}

std::vector<geometry_msgs::Pose> LocalExplorationPlanner::connectTSPOrderWithSubvertices(std::vector<std::pair<int, std::vector<int>>> &tsp_nodes, std::map<int, ShortestPathsReport> &path_rep_map, std::shared_ptr<CognitiveGraph> graph)
{
  std::vector<geometry_msgs::Pose> tsp_path;

  std::set<size_t> cumulative_hash_keys;
  int num_vps_skipped = 0;
  
  geometry_msgs::Pose p;
  convert(graph->getVertex(tsp_nodes[0].first)->state, p);
  tsp_path.push_back(p);
  cumulative_hash_keys.merge(graph->getVertex(tsp_nodes[0].first)->vol_gain.unseen_voxel_hash_keys);
  double prev_yaw = graph->getVertex(tsp_nodes[0].first)->state(3);
  double prev_pitch = graph->getVertex(tsp_nodes[0].first)->state(4);

  auto angularDistance = [](double a, double b) {
      double diff = b - a;
      truncateAngle(diff);
      return std::abs(diff);
  };

  for (int i = 1; i < tsp_nodes.size(); ++i)
  {
    std::vector<StateVec> current_path_segment_vec;
		std::vector<geometry_msgs::Pose> current_path_segment;
    graph->getShortestPath(tsp_nodes[i].first, path_rep_map[tsp_nodes[i-1].first], true, current_path_segment_vec);
    current_path_segment_vec[0](3) = prev_yaw;
    current_path_segment_vec[0](4) = prev_pitch;

    Vertex * current_vertex = graph->getVertex(tsp_nodes[i].first);

    if(tsp_nodes[i].second.size() == 1)
    {
      current_path_segment_vec.back()(3) = current_vertex->orientation_sub_vertices[tsp_nodes[i].second[0]]->state(3);
      linearlyInterpolateYaw(current_path_segment_vec);
      for (int j = 1; j < current_path_segment_vec.size() - 1; ++j)
      {
        current_path_segment_vec[j](4) = current_path_segment_vec[0](4);
        geometry_msgs::Pose p;
        convert(current_path_segment_vec[j], p);
        tsp_path.push_back(p);
      }
      geometry_msgs::Pose p;
      convert(graph->getVertex(tsp_nodes[i].first)->orientation_sub_vertices[tsp_nodes[i].second[0]]->state, p);
      tsp_path.push_back(p);
      prev_yaw = graph->getVertex(tsp_nodes[i].first)->orientation_sub_vertices[tsp_nodes[i].second[0]]->state(3);
      prev_pitch = graph->getVertex(tsp_nodes[i].first)->orientation_sub_vertices[tsp_nodes[i].second[0]]->state(4);
    }
    else
    {
      std::vector<std::pair<int, double>> yaw_map, pitch_diff_map;
      for(auto subv_id : tsp_nodes[i].second)
      {
        yaw_map.push_back(std::make_pair(subv_id, current_vertex->orientation_sub_vertices[subv_id]->state(3)));
      }
      std::sort(yaw_map.begin(), yaw_map.end(), 
                [](const std::pair<int, double> &v1, const std::pair<int, double> &v2)
                {
                  return v1.second > v2.second;
                });
      convert(current_path_segment_vec, current_path_segment);
      double path_len = pathLength(current_path_segment);
      double translation_time = path_len / planning_params_.v_max;
      double possible_rotation_during_translation = translation_time * planning_params_.yaw_rate_max;
      int best_start_index = 0;
      double min_total_time = std::numeric_limits<double>::max();
      bool best_is_clockwise = true;

      for (size_t start_idx=0; start_idx<yaw_map.size(); ++start_idx) {
        double initial_yaw = yaw_map[start_idx].second;
        double initial_angular_dist = angularDistance(prev_yaw, initial_yaw);
        double initial_rotation_time = 0;
        if (initial_angular_dist > possible_rotation_during_translation) {
            initial_rotation_time = (initial_angular_dist - possible_rotation_during_translation) / planning_params_.yaw_rate_max;
        }
        double clockwise_rotation_time = 0;
        double current_yaw = initial_yaw;
        for (size_t i = 1; i < yaw_map.size(); ++i) {
            size_t next_idx = (start_idx + i) % yaw_map.size();
            double next_yaw = yaw_map[next_idx].second;
            
            double angular_dist = angularDistance(current_yaw, next_yaw);
            clockwise_rotation_time += angular_dist / planning_params_.yaw_rate_max;
            current_yaw = next_yaw;
        }
        double clockwise_total_time = translation_time + initial_rotation_time + clockwise_rotation_time;
        double counterclockwise_rotation_time = 0;
        current_yaw = initial_yaw;
        for (size_t i = 1; i < yaw_map.size(); ++i) {
            size_t next_idx = (start_idx - i + yaw_map.size()) % yaw_map.size();
            double next_yaw = yaw_map[next_idx].second;
            
            double angular_dist = angularDistance(current_yaw, next_yaw);
            counterclockwise_rotation_time += angular_dist / planning_params_.yaw_rate_max;
            current_yaw = next_yaw;
        }
        double counterclockwise_total_time = translation_time + initial_rotation_time + counterclockwise_rotation_time;
        if (clockwise_total_time < min_total_time) {
            min_total_time = clockwise_total_time;
            best_start_index = start_idx;
            best_is_clockwise = true;
        }
        if (counterclockwise_total_time < min_total_time) {
            min_total_time = counterclockwise_total_time;
            best_start_index = start_idx;
            best_is_clockwise = false;
        }
        
      }

      std::vector<int> optimal_yaw_order;
      optimal_yaw_order.reserve(yaw_map.size());
      optimal_yaw_order.push_back(yaw_map[best_start_index].first);
      if (best_is_clockwise) {
          for (size_t i = 1; i < yaw_map.size(); ++i) {
              size_t idx = (best_start_index + i) % yaw_map.size();
              optimal_yaw_order.push_back(yaw_map[idx].first);
          }
      } else {
          for (size_t i = 1; i < yaw_map.size(); ++i) {
              size_t idx = (best_start_index - i + yaw_map.size()) % yaw_map.size();
              optimal_yaw_order.push_back(yaw_map[idx].first);
          }
      }

      std::vector<int> final_ordering;
      int ind = 0;
      while(ind < optimal_yaw_order.size())
      {
        std::vector<std::pair<int, double>> pitch_map;
        pitch_map.push_back(std::make_pair(optimal_yaw_order[ind], 
          current_vertex->orientation_sub_vertices[optimal_yaw_order[ind]]->state(4)));
        for(int ind2=ind+1; ind2<optimal_yaw_order.size(); ++ind2)
        {
          double dyaw = angularDistance(current_vertex->orientation_sub_vertices[optimal_yaw_order[ind2]]->state(3), 
            current_vertex->orientation_sub_vertices[optimal_yaw_order[ind]]->state(3));
          if(dyaw <= 0.1)
          {
            pitch_map.push_back(std::make_pair(optimal_yaw_order[ind2], 
              current_vertex->orientation_sub_vertices[optimal_yaw_order[ind2]]->state(4)));
          }
        }
        std::sort(pitch_map.begin(), pitch_map.end(), 
                [](const std::pair<int, double> &v1, const std::pair<int, double> &v2)
                {
                  return v1.second > v2.second;
                });
        for(auto vp : pitch_map)
          final_ordering.push_back(vp.first);
        ind += pitch_map.size();
      }

      current_path_segment_vec.back()(3) = current_vertex->orientation_sub_vertices[final_ordering[0]]->state(3);
      linearlyInterpolateYaw(current_path_segment_vec);
      for (int j = 1; j < current_path_segment_vec.size() - 1; ++j)
      {
        current_path_segment_vec[j](4) = current_path_segment_vec[0](4);
        geometry_msgs::Pose p;
        convert(current_path_segment_vec[j], p);
        tsp_path.push_back(p);
      }      
      for(auto subv_id : final_ordering)
      {
        int current_num_keys = cumulative_hash_keys.size();
        cumulative_hash_keys.merge(graph->getVertex(tsp_nodes[i].first)->orientation_sub_vertices[subv_id]->vol_gain.unseen_voxel_hash_keys);
        if(cumulative_hash_keys.size() <= current_num_keys)
        {
          ++num_vps_skipped;
          continue;
        }
        geometry_msgs::Pose p;
        convert(graph->getVertex(tsp_nodes[i].first)->orientation_sub_vertices[subv_id]->state, p);
        tsp_path.push_back(p);
      }
      StateVec last_pt_vec;
      convert(tsp_path.back(), last_pt_vec);
      prev_yaw = last_pt_vec(3);
      prev_pitch = last_pt_vec(4);
    }

  }

  std::cout << "Num skipped viewpoints: " << num_vps_skipped << std::endl;

	return tsp_path;
}
