#include "gcs_planner_pointrobot.h"
#include "gcs_planner_robotarm.h"
#include "gcs_structs.h"
#include "free_space_cdt.h"

#include "drake/solvers/solve.h"
#include <iostream>

#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <Eigen/Dense>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/graph_of_convex_sets.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/point.h"
#include "drake/planning/trajectory_optimization/gcs_trajectory_optimization.h"
#include "drake/solvers/mathematical_program_result.h"
#include <fstream>

using drake::geometry::optimization::ConvexSets;
using drake::geometry::optimization::GraphOfConvexSetsOptions;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Point;
using drake::planning::trajectory_optimization::GcsTrajectoryOptimization;

using json = nlohmann::json;

std::shared_ptr<MapData> loadMapFromJson(const std::string& filepath){

    std::shared_ptr<MapData> map = std::make_shared<MapData>();

    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open JSON file: " + filepath);
    }

    json j;
    file >> j;

    if (!j.contains("shapes") || !j["shapes"].is_array()) {
        throw std::runtime_error("JSON does not contain a valid 'shapes' array.");
    }

    for (const auto& shape_json : j["shapes"]) {
        Shape new_shape;

        if (!shape_json.contains("name") || !shape_json["name"].is_string()) {
            throw std::runtime_error("Shape is missing a valid 'name'.");
        }
        if (!shape_json.contains("type") || !shape_json["type"].is_string()) {
            throw std::runtime_error("Shape is missing a valid 'type'.");
        }
        if (!shape_json.contains("points") || !shape_json["points"].is_array()) {
            throw std::runtime_error("Shape is missing a valid 'points' array.");
        }

        new_shape.name = shape_json["name"].get<std::string>();

        if (shape_json["type"].get<std::string>() == "obstacle"){
            new_shape.type = ShapeType::OBSTACLE;
        }
        else if (shape_json["type"].get<std::string>() == "AO"){
            new_shape.type = ShapeType::AO;
        }

        for (const auto& pt_json : shape_json["points"]) {
            if (!pt_json.is_array() || pt_json.size() != 2) {
                throw std::runtime_error("Each point must be an array of size 2.");
            }

            Point2D pt;
            pt.x = pt_json[0].get<double>();
            pt.y = pt_json[1].get<double>();
            new_shape.points.push_back(pt);
        }

        if (new_shape.type == ShapeType::OBSTACLE){
            map->obstacles.push_back(new_shape);
        }
        else if (new_shape.type == ShapeType::AO)
        {
            map->AO = new_shape;
        }
    }
    return map;
}

void saveConvexRegionsToJson(const std::shared_ptr<MapData>& map, const std::string& filepath)
{
    if (!map) {
        throw std::runtime_error("saveConvexRegionsToJson received null map");
    }

    json j;
    j["convex_regions"] = json::array();

    for (const auto& region : map->freespace_regions)
    {
        if (region.type != ShapeType::FREESPACE) {
            continue;
        }

        if (region.points.size() < 3) {
            throw std::runtime_error("Encountered a freespace region with fewer than 3 points.");
        }

        json region_json;
        region_json["name"] = region.name;
        region_json["type"] = "FREESPACE";
        region_json["points"] = json::array();

        for (const auto& pt : region.points)
        {
            region_json["points"].push_back({pt.x, pt.y});
        }

        j["convex_regions"].push_back(region_json);
    }

    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open output JSON file: " + filepath);
    }

    file << j.dump(4);
}


inline void from_json(const json& j, Point2D& p) {
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
}

inline void from_json(const json& j, VelocityBounds& v) {
    if (j.contains("lb")) j.at("lb").get_to(v.lb);
    if (j.contains("ub")) j.at("ub").get_to(v.ub);
}

inline void from_json(const json& j, IrisOptionsConfig& o) {
    if (j.contains("coverage_termination_threshold")) j.at("coverage_termination_threshold").get_to(o.coverage_termination_threshold);
    if (j.contains("iteration_limit")) j.at("iteration_limit").get_to(o.iteration_limit);
    if (j.contains("internal_iteration_limit")) j.at("internal_iteration_limit").get_to(o.internal_iteration_limit);
    if (j.contains("num_points_per_visibility_round")) j.at("num_points_per_visibility_round").get_to(o.num_points_per_visibility_round);
    if (j.contains("num_points_per_coverage_check")) j.at("num_points_per_coverage_check").get_to(o.num_points_per_coverage_check);
    if (j.contains("minimum_clique_size")) j.at("minimum_clique_size").get_to(o.minimum_clique_size);
    if (j.contains("require_sample_point_is_contained")) j.at("require_sample_point_is_contained").get_to(o.require_sample_point_is_contained);
}

inline void from_json(const json& j, CollisionCheckerOptionsConfig& o) {
    if (j.contains("edge_step_size")) j.at("edge_step_size").get_to(o.edge_step_size);
    if (j.contains("env_collision_padding")) j.at("env_collision_padding").get_to(o.env_collision_padding);
    if (j.contains("self_collision_padding")) j.at("self_collision_padding").get_to(o.self_collision_padding);
}

inline void from_json(const json& j, RobotArmOptions& o) {
    if (j.contains("num_joints")) j.at("num_joints").get_to(o.num_joints);
    if (j.contains("link_length")) j.at("link_length").get_to(o.link_length);
    if (j.contains("link_width")) j.at("link_width").get_to(o.link_width);
    if (j.contains("link_thickness")) j.at("link_thickness").get_to(o.link_thickness);
    if (j.contains("link_mass")) j.at("link_mass").get_to(o.link_mass);

    if (j.contains("joint_lower_limits")) j.at("joint_lower_limits").get_to(o.joint_lower_limits);
    if (j.contains("joint_upper_limits")) j.at("joint_upper_limits").get_to(o.joint_upper_limits);

    if (j.contains("q_start")) j.at("q_start").get_to(o.q_start);
    if (j.contains("q_goal")) j.at("q_goal").get_to(o.q_goal);

    if (j.contains("base_position")) j.at("base_position").get_to(o.base_position);

    if (j.contains("iris")) j.at("iris").get_to(o.iris);
    if (j.contains("collision_checker")) j.at("collision_checker").get_to(o.collision_checker);
}

inline void from_json(const json& j, GCSOptions& o) {
    if (j.contains("planner_type")) j.at("planner_type").get_to(o.planner_type);
    if (j.contains("map_path")) j.at("map_path").get_to(o.map_path);
    if (j.contains("dimension")) j.at("dimension").get_to(o.dimension);
    if (j.contains("bezier_order")) j.at("bezier_order").get_to(o.bezier_order);
    if (j.contains("continuity_order")) j.at("continuity_order").get_to(o.continuity_order);
    if (j.contains("h_min")) j.at("h_min").get_to(o.h_min);
    if (j.contains("h_max")) j.at("h_max").get_to(o.h_max);
    if (j.contains("path_length_weight")) j.at("path_length_weight").get_to(o.path_length_weight);
    if (j.contains("time_weight")) j.at("time_weight").get_to(o.time_weight);
    if (j.contains("num_samples")) j.at("num_samples").get_to(o.num_samples);
    if (j.contains("use_convex_relaxation")) j.at("use_convex_relaxation").get_to(o.use_convex_relaxation);
    if (j.contains("max_rounded_paths")) j.at("max_rounded_paths").get_to(o.max_rounded_paths);
    if (j.contains("preprocessing")) j.at("preprocessing").get_to(o.preprocessing);
    if (j.contains("results_path")) j.at("results_path").get_to(o.results_path);
    if (j.contains("start")) j.at("start").get_to(o.start);
    if (j.contains("goal")) j.at("goal").get_to(o.goal);
    if (j.contains("velocity_bounds")) j.at("velocity_bounds").get_to(o.velocity_bounds);
    if (j.contains("path_energy_weight")) j.at("path_energy_weight").get_to(o.path_energy_weight);

    if (j.contains("robot_arm")) j.at("robot_arm").get_to(o.robot_arm);
}

GCSOptions LoadGcsOptions(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open GCS options file: " + filepath);
    }

    json j;
    file >> j;
    return j.get<GCSOptions>();
}

void PrintOptions(const GCSOptions& o) {
    std::cout << "===== GCS Planner Options =====\n";

    std::cout << "map_path: " << o.map_path << "\n";
    std::cout << "dimension: " << o.dimension << "\n";

    std::cout << "bezier_order: " << o.bezier_order << "\n";
    std::cout << "continuity_order: " << o.continuity_order << "\n";

    std::cout << "h_min: " << o.h_min << "\n";
    std::cout << "h_max: " << o.h_max << "\n";

    std::cout << "path_length_weight: " << o.path_length_weight << "\n";
    std::cout << "time_weight: " << o.time_weight << "\n";

    std::cout << "num_samples: " << o.num_samples << "\n";

    std::cout << "use_convex_relaxation: " << std::boolalpha << o.use_convex_relaxation << "\n";
    std::cout << "max_rounded_paths: " << o.max_rounded_paths << "\n";
    std::cout << "preprocessing: " << std::boolalpha << o.preprocessing << "\n";

    std::cout << "results_path: " << o.results_path << "\n";

    std::cout << "start: (" << o.start.x << ", " << o.start.y << ")\n";
    std::cout << "goal: (" << o.goal.x << ", " << o.goal.y << ")\n";

    std::cout << "================================\n";
}

int main(int argc, char** argv) {

    std::string config_filepath = "../configs/gcs_config1.json"; // default

    if (argc >= 2) {
        config_filepath = argv[1];
    } else {
        std::cout << "No path provided, using default: " << config_filepath << "\n";
    }


    GCSOptions opts = LoadGcsOptions(config_filepath);

    PrintOptions(opts);

    if (!std::filesystem::exists(opts.results_path)) {
        std::filesystem::create_directories(opts.results_path);
    }

    std::shared_ptr<MapData> map = loadMapFromJson(opts.map_path);

    if (opts.planner_type == "point_robot"){
        FreeSpaceCDT cdt;
        cdt.triangulate_free_space(map);
        saveConvexRegionsToJson(map, opts.results_path + "triangles.json");

        cdt.merge_free_space_triangles_into_convex_regions(map);
        saveConvexRegionsToJson(map, opts.results_path + "convex_regions.json");

        GcsPlannerPointRobot planner;

        // const Eigen::Vector2d start(1.0, 9.0);
        const Eigen::Vector2d start(opts.start.x, opts.start.y);
        const Eigen::Vector2d goal(opts.goal.x, opts.goal.y);

        const bool ok = planner.SolvePath(map, start, goal, opts);

        if (!ok) {
            return 1;
        }

    }

    if (opts.planner_type == "robot_arm"){
        GcsPlannerRobotArm planner_arm;
        const bool ok = planner_arm.SolvePath(opts, map);

        if (!ok) {
            return 1;
        }
    }

   return 0;
   
   
   
}
   
