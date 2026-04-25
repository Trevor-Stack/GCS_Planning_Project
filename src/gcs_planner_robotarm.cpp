#include "gcs_planner_robotarm.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include <filesystem>
#include <Eigen/Dense>

#include <drake/common/random.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/rigid_body.h>
#include <drake/multibody/tree/spatial_inertia.h>
#include <drake/multibody/tree/unit_inertia.h>
#include <drake/planning/collision_checker_params.h>
#include <drake/planning/iris/iris_from_clique_cover.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/planning/scene_graph_collision_checker.h>
#include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>
#include <drake/solvers/mathematical_program_result.h>
#include <limits>
#include <chrono>
#include <iomanip>
#include <drake/common/parallelism.h>
#include <drake/solvers/gurobi_solver.h>

using drake::geometry::Box;
using drake::geometry::optimization::GraphOfConvexSetsOptions;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Point;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::planning::CollisionCheckerParams;
using drake::planning::IrisFromCliqueCoverOptions;
using drake::planning::IrisInConfigurationSpaceFromCliqueCover;
using drake::planning::RobotDiagramBuilder;
using drake::planning::SceneGraphCollisionChecker;
using drake::planning::trajectory_optimization::GcsTrajectoryOptimization;

using Clock = std::chrono::steady_clock;

double SecondsSince(const Clock::time_point& start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

struct BoxSpec2D {
    double cx;
    double cy;
    double sx;
    double sy;
};

inline BoxSpec2D GetAxisAlignedBoxFromShape(const Shape& shape) {
    if (shape.points.empty()) {
        throw std::runtime_error("Shape has no points: " + shape.name);
    }

    double xmin = shape.points[0].x;
    double xmax = shape.points[0].x;
    double ymin = shape.points[0].y;
    double ymax = shape.points[0].y;

    for (const auto& p : shape.points) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }

    const double sx = xmax - xmin;
    const double sy = ymax - ymin;

    if (sx <= 0.0 || sy <= 0.0) {
        throw std::runtime_error("Obstacle has non-positive size: " + shape.name);
    }

    return BoxSpec2D{
        0.5 * (xmin + xmax),
        0.5 * (ymin + ymax),
        sx,
        sy
    };
}

bool GcsPlannerRobotArm::SolvePath(
    const GCSOptions& opts,
    const std::shared_ptr<MapData>& map) {
    
    const auto overall_start_time = Clock::now(); 

    if (!map) {
        throw std::runtime_error("SolvePath received null map.");
    }

    const auto& arm = opts.robot_arm;

    if (arm.q_start.size() != static_cast<size_t>(arm.num_joints) ||
        arm.q_goal.size()  != static_cast<size_t>(arm.num_joints) ||
        arm.joint_lower_limits.size() != static_cast<size_t>(arm.num_joints) ||
        arm.joint_upper_limits.size() != static_cast<size_t>(arm.num_joints)) {
        throw std::runtime_error("Robot arm option vector sizes must match num_joints.");
    }

    RobotDiagramBuilder<double> builder;
    auto& plant = builder.plant();

    const double link_length = arm.link_length;
    const double link_width = arm.link_width;
    const double link_thickness = arm.link_thickness;
    const double link_mass = arm.link_mass;
    const double obstacle_thickness = 0.5;

    const UnitInertia<double> G_link =
        UnitInertia<double>::SolidBox(link_length, link_width, link_thickness);
    const SpatialInertia<double> M_link(
        link_mass, Eigen::Vector3d::Zero(), G_link);

    const CoulombFriction<double> friction(
        0.9, 0.8);

    const auto& world_body = plant.world_body();

    const drake::multibody::ModelInstanceIndex arm_instance =
        plant.AddModelInstance("arm");

    const RigidBody<double>* parent_body = &world_body;

    for (int i = 0; i < arm.num_joints; ++i) {
        const std::string link_name = "link_" + std::to_string(i + 1);
        const std::string joint_name = "joint_" + std::to_string(i + 1);

        const auto& link = plant.AddRigidBody(link_name, arm_instance, M_link);

        RigidTransformd X_PJ(
            (i == 0)
                ? Eigen::Vector3d(
                      arm.base_position[0],
                      arm.base_position[1],
                      arm.base_position[2])
                : Eigen::Vector3d(link_length, 0.0, 0.0));

        const RigidTransformd X_CJ(Eigen::Vector3d::Zero());

        plant.AddJoint<RevoluteJoint>(
            joint_name,
            *parent_body, X_PJ,
            link, X_CJ,
            Eigen::Vector3d::UnitZ());

        auto& joint = plant.GetMutableJointByName<RevoluteJoint>(joint_name);
        joint.set_default_angle(0.0);
        joint.set_position_limits(
            Eigen::VectorXd::Constant(1, arm.joint_lower_limits.at(i)),
            Eigen::VectorXd::Constant(1, arm.joint_upper_limits.at(i)));

        const RigidTransformd X_BG(
            Eigen::Vector3d(link_length / 2.0, 0.0, 0.0));

        plant.RegisterVisualGeometry(
            link, X_BG,
            Box(link_length, link_width, link_thickness),
            link_name + "_visual",
            Eigen::Vector4d(0.7, 0.7, 0.7, 1.0));

        plant.RegisterCollisionGeometry(
            link, X_BG,
            Box(link_length, link_width, link_thickness),
            link_name + "_collision",
            friction);

        parent_body = &link;
    }

    // Build workspace obstacles from the map JSON.
    for (const auto& obs : map->obstacles) {
        const BoxSpec2D box = GetAxisAlignedBoxFromShape(obs);

        plant.RegisterVisualGeometry(
            world_body,
            RigidTransformd(Eigen::Vector3d(box.cx, box.cy, 0.0)),
            Box(box.sx, box.sy, obstacle_thickness),
            obs.name + "_visual",
            Eigen::Vector4d(0.85, 0.2, 0.2, 1.0));

        plant.RegisterCollisionGeometry(
            world_body,
            RigidTransformd(Eigen::Vector3d(box.cx, box.cy, 0.0)),
            Box(box.sx, box.sy, obstacle_thickness),
            obs.name + "_collision",
            friction);
    }

    auto robot_diagram = builder.Build();

    CollisionCheckerParams params;
    params.model = std::move(robot_diagram);
    params.robot_model_instances =
        std::vector<drake::multibody::ModelInstanceIndex>{arm_instance};
    params.edge_step_size = arm.collision_checker.edge_step_size;
    params.env_collision_padding = arm.collision_checker.env_collision_padding;
    params.self_collision_padding = arm.collision_checker.self_collision_padding;

    SceneGraphCollisionChecker checker(params);

    Eigen::VectorXd q_start =
        Eigen::Map<const Eigen::VectorXd>(arm.q_start.data(), arm.q_start.size());
    Eigen::VectorXd q_goal =
        Eigen::Map<const Eigen::VectorXd>(arm.q_goal.data(), arm.q_goal.size());

    if (!checker.CheckConfigCollisionFree(q_start)) {
        std::cerr << "Start is in collision.\n";
        return false;
    }
    if (!checker.CheckConfigCollisionFree(q_goal)) {
        std::cerr << "Goal is in collision.\n";
        return false;
    }

    IrisFromCliqueCoverOptions iris_options;
    iris_options.coverage_termination_threshold =
        arm.iris.coverage_termination_threshold;
    std::cout << "iris iteration_limit = "
          << arm.iris.iteration_limit << std::endl;
    iris_options.iteration_limit = arm.iris.iteration_limit;
    std::cout << "assigned Drake iris iteration_limit = "
          << iris_options.iteration_limit << std::endl;
    iris_options.num_points_per_visibility_round =
        arm.iris.num_points_per_visibility_round;
    iris_options.num_points_per_coverage_check =
        arm.iris.num_points_per_coverage_check;
    iris_options.minimum_clique_size = arm.iris.minimum_clique_size;
    iris_options.parallelism = drake::Parallelism::Max();

    auto& internal_iris =
        std::get<drake::geometry::optimization::IrisOptions>(
            iris_options.iris_options);
    internal_iris.require_sample_point_is_contained =
        arm.iris.require_sample_point_is_contained;
    internal_iris.iteration_limit = arm.iris.internal_iteration_limit;
    // internal_iris.configuration_space_margin = 0.0;

    std::vector<HPolyhedron> cspace_regions;
    drake::RandomGenerator generator(0);
    
    const auto ciris_start_time = Clock::now();

    IrisInConfigurationSpaceFromCliqueCover(
        checker, iris_options, &generator, &cspace_regions);

    const double ciris_time = SecondsSince(ciris_start_time);

    std::cout << "Found " << cspace_regions.size() << " C-space regions\n";
    if (cspace_regions.empty()) {
        std::cerr << "No collision-free C-space regions found.\n";
        return false;
    }

    GcsTrajectoryOptimization gcs(arm.num_joints);

    drake::geometry::optimization::ConvexSets region_sets;
    region_sets.reserve(cspace_regions.size());
    for (const auto& region : cspace_regions) {
        region_sets.emplace_back(region.Clone());
    }

    auto& free_space = gcs.AddRegions(
        region_sets,
        opts.bezier_order,
        opts.h_min,
        opts.h_max,
        "free");

    drake::geometry::optimization::ConvexSets start_set;
    start_set.emplace_back(std::make_unique<Point>(q_start));
    auto& source = gcs.AddRegions(
        start_set, 0, 0.0, 0.0, "start");

    drake::geometry::optimization::ConvexSets goal_set;
    goal_set.emplace_back(std::make_unique<Point>(q_goal));
    auto& target = gcs.AddRegions(
        goal_set, 0, 0.0, 0.0, "goal");

    gcs.AddEdges(source, free_space);
    gcs.AddEdges(free_space, free_space);
    gcs.AddEdges(free_space, target);

    if (opts.path_length_weight > 0.0) {
        gcs.AddPathLengthCost(opts.path_length_weight);
    }
    if (opts.time_weight > 0.0) {
        gcs.AddTimeCost(opts.time_weight);
    }
    if (opts.continuity_order > 0) {
        gcs.AddPathContinuityConstraints(opts.continuity_order);
    }
    if (opts.path_energy_weight > 0.0){
        gcs.AddPathEnergyCost(opts.path_energy_weight);
    }

    GraphOfConvexSetsOptions gcs_options;
    gcs_options.convex_relaxation = opts.use_convex_relaxation;
    gcs_options.preprocessing = opts.preprocessing;
    gcs_options.max_rounded_paths = opts.max_rounded_paths;
    gcs_options.parallelism = drake::Parallelism::Max();
    drake::solvers::GurobiSolver gurobi;
    gcs_options.solver = &gurobi;
    gcs_options.restriction_solver = &gurobi;
    gcs_options.preprocessing_solver = &gurobi;

    const auto gcs_start_time = Clock::now();
    auto [traj, result] = gcs.SolvePath(source, target, gcs_options);
    const double gcs_solve_time = SecondsSince(gcs_start_time);

    if (!result.is_success()) {
        std::cerr << "GCS solve failed.\n";
        return false;
    }

    // auto q_traj = GcsTrajectoryOptimization::NormalizeSegmentTimes(traj);
    const double overall_time = SecondsSince(overall_start_time);
    std::cout << "Solved GCS trajectory.\n";
    std::cout << "Overall Cost: " << result.get_optimal_cost() << "\n";
    std::cout << "C-IRIS time: " << ciris_time << " s\n";
    std::cout << "GCS solver time: " << gcs_solve_time << " s\n";
    std::cout << "Overall time: " << overall_time << " s\n";


    if (opts.use_convex_relaxation) {
        std::cout << "\nRelaxed edge variables phi:\n";

        for (const auto* edge : gcs.graph_of_convex_sets().Edges()) {
            const double phi = result.GetSolution(edge->phi());

            // if (phi > 1e-6) {
            std::cout << "  ("
                    << edge->u().name()
                    << ") -> ("
                    << edge->v().name()
                    << ") : phi = "
                    << phi
                    << "\n";
            // }
        }
    }
    std::cout << "\nEdges selected in final path:\n";

    for (const auto* edge : gcs.graph_of_convex_sets().Edges()) {
        const double phi = result.GetSolution(edge->phi());

        if (phi > 1.0 - 1e-6) {
            std::cout << edge->u().name()
                    << " -> "
                    << edge->v().name()
                    << " | phi = "
                    << phi
                    << "\n";
        }
    }

    const int kNumSamples = opts.num_samples;  // e.g. 100 or 500
    const double t0 = traj.start_time();
    const double tf = traj.end_time();

    

    std::filesystem::create_directories(opts.results_path);
    std::ofstream file(opts.results_path + "/trajectory.csv");

    file << "t";
    for (int j = 0; j < arm.num_joints; ++j) {
        file << ",q" << (j + 1);
    }
    file << "\n";

    for (int i = 0; i < kNumSamples; ++i) {
        const double alpha =
            (kNumSamples == 1) ? 0.0 : static_cast<double>(i) / (kNumSamples - 1);
        const double t = (1.0 - alpha) * t0 + alpha * tf;
        const Eigen::VectorXd q = traj.value(t);

        file << t;
        for (int j = 0; j < q.size(); ++j) {
            file << "," << q(j);
        }
        file << "\n";
    }

    file.close();
    std::cout << "Saved trajectory to "
            << opts.results_path + "/trajectory.csv" << "\n";

    // for (int i = 0; i <= 10; ++i) {
    //     const double t = q_traj.start_time() +
    //         (q_traj.end_time() - q_traj.start_time()) * i / 10.0;
    //     const Eigen::VectorXd q = q_traj.value(t);
    //     std::cout << "q(" << t << ") = " << q.transpose() << "\n";
    // }

    return true;
}