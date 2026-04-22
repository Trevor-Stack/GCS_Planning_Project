#pragma once

#include "gcs_structs.h"

#include <Eigen/Dense>

#include <drake/geometry/optimization/convex_set.h>
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>
#include <drake/solvers/mathematical_program_result.h>

#include <memory>
#include <string>

class GcsPlannerPointRobot {
public:

    GcsPlannerPointRobot() = default;

    bool SolvePath(
        const std::shared_ptr<MapData>& map,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        const GCSOptions& options);
};