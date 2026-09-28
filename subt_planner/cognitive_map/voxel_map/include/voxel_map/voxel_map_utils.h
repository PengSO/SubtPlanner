#pragma once

#include <eigen3/Eigen/Dense>

#include "planner_common/params.h"
#include "cognitive_graph_model/graph_types.h"

enum VoxelStatus { kUnknown = 0, kOccupied, kFree };