#pragma once

#include <eigen3/Eigen/Dense>

#include "planner_common/params.h"
#include "planner_common/graph_base.h"

enum VoxelStatus { kUnknown = 0, kOccupied, kFree };