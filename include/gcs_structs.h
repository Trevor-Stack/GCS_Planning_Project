#pragma once

#include <string>
#include <vector>
#include <cmath>

struct Point2D {
    double x;
    double y;
    Point2D() = default;
    Point2D(double x_val, double y_val) : x(x_val), y(y_val) {}

};
struct VelocityBounds {
    std::vector<double> lb;
    std::vector<double> ub;
};

enum class ShapeType {
    AO,
    OBSTACLE,
    FREESPACE
};

struct Shape {
    std::string name;
    ShapeType type;
    std::vector<Point2D> points;

    Shape() = default;

    // Constructor
    Shape(const std::string& name_,
          ShapeType type_,
          const std::vector<Point2D>& points_)
        : name(name_), type(type_), points(points_) {}
};

struct MapData {
    Shape AO;
    std::vector<Shape> obstacles;
    std::vector<Shape> freespace_regions;

    // Default constructor
    MapData() = default;

    // Constructor
    MapData(const Shape& AO_,
            const std::vector<Shape>& obstacles_,
            const std::vector<Shape>& freespace_regions_)
        : AO(AO_), obstacles(obstacles_), freespace_regions(freespace_regions_) {}
};

struct IrisOptionsConfig {
    double coverage_termination_threshold = 0.9;
    int iteration_limit = 30;
    int internal_iteration_limit = 1;
    int num_points_per_visibility_round = 400;
    int num_points_per_coverage_check = 5000;
    int minimum_clique_size = 3;
    bool require_sample_point_is_contained = true;
};

struct CollisionCheckerOptionsConfig {
    double edge_step_size = 0.05;
    double env_collision_padding = 0.0;
    double self_collision_padding = 0.0;
};

struct RobotArmOptions {
    int num_joints = 3;

    double link_length = 2.0;
    double link_width = 0.001;
    double link_thickness = 0.001;
    double link_mass = 1.0;

    std::vector<double> joint_lower_limits{-2.8, -2.8, -2.8};
    std::vector<double> joint_upper_limits{ 2.8,  2.8,  2.8};

    std::vector<double> q_start{M_PI / 2.0, 0.0, 0.0};
    std::vector<double> q_goal{0.0, -M_PI / 2.0, 0.0};

    std::vector<double> base_position{5.0, 0.0, 0.0};

    IrisOptionsConfig iris;
    CollisionCheckerOptionsConfig collision_checker;
};

struct GCSOptions {
    std::string planner_type = "point_robot";

    std::string map_path = "../maps/map1.json";
    int dimension = 2;
    int bezier_order = 1;
    double h_min = 1e-3;
    double h_max = 5.0;
    double path_length_weight = 1.0;
    double time_weight = 0.0;
    int continuity_order = 1;
    int num_samples = 81;
    bool use_convex_relaxation = true;
    int max_rounded_paths = 20;
    bool preprocessing = true;
    std::string results_path = "../results/config1";
    Point2D start{9.0, 9.0};
    Point2D goal{1.0, 0.5};
    VelocityBounds velocity_bounds{{-2.0, -2.0}, {2.0, 2.0}};
    double path_energy_weight = 0.0;

    RobotArmOptions robot_arm;
};