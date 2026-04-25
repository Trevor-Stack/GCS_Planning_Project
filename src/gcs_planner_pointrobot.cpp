#include "gcs_planner_pointrobot.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <algorithm>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/convexity_check_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_with_holes_2.h>

using K = CGAL::Exact_predicates_exact_constructions_kernel;
using CGALPoint = K::Point_2;
using Polygon2 = CGAL::Polygon_2<K>;

using drake::geometry::optimization::ConvexSets;
using drake::geometry::optimization::GraphOfConvexSetsOptions;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Point;
using drake::planning::trajectory_optimization::GcsTrajectoryOptimization;
using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<K>;

namespace {

constexpr double kEps = 1e-10;

void make_CCW(std::vector<Point2D>& pts){
    Polygon2 poly;
    for (const auto& p : pts) {
        poly.push_back(CGALPoint(p.x, p.y));
    }
    if (poly.is_clockwise_oriented()) {
        std::reverse(pts.begin(), pts.end());
        return;
    }
    else{
        return;
    }

}

Polygon2 ToCgalPolygon(const std::vector<Point2D>& pts) {
    Polygon2 poly;
    for (const auto& p : pts) {
        poly.push_back(CGALPoint(p.x, p.y));
    }

    if (poly.is_clockwise_oriented()) {
        poly.reverse_orientation();
    }
    return poly;
}

bool IsConvexPolygon(const std::vector<Point2D>& pts) {
    if (pts.size() < 3) {
        return false;
    }

    std::vector<CGALPoint> cgal_pts;
    cgal_pts.reserve(pts.size());
    for (const auto& p : pts) {
        cgal_pts.emplace_back(p.x, p.y);
    }

    return CGAL::is_convex_2(cgal_pts.begin(), cgal_pts.end(), K());
}
bool HasPositiveAreaOverlap(const Polygon2& A, const Polygon2& B) {
    std::vector<Polygon_with_holes_2> inters;
    CGAL::intersection(A, B, std::back_inserter(inters));
    if (!inters.empty()){
        return true;
    }
    // for (auto vit = A.vertices_begin(); vit != A.vertices_end(); ++vit) {
    //     auto side = B.bounded_side(*vit);
    //     if (side == CGAL::ON_BOUNDED_SIDE || side == CGAL::ON_BOUNDARY ) {
    //         return true;
    //     }
    // }
    // for (auto vit = B.vertices_begin(); vit != B.vertices_end(); ++vit) {
    //     auto side = A.bounded_side(*vit);
    //     if (side == CGAL::ON_BOUNDED_SIDE || side == CGAL::ON_BOUNDARY) {
    //         return true;
    //     }
    // }
    return false;
}

bool SharesBoundarySegment(const Polygon2& A, const Polygon2& B) {
    for (auto ea = A.edges_begin(); ea != A.edges_end(); ++ea) {
        for (auto eb = B.edges_begin(); eb != B.edges_end(); ++eb) {
            auto result = CGAL::intersection(*ea, *eb);
            if (result) {
                if (const K::Segment_2* seg = std::get_if<K::Segment_2>(&*result)) {
                    if (seg->squared_length() > 1e-12) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool AreAdjacentCGAL(const std::vector<Point2D>& a_pts,
                     const std::vector<Point2D>& b_pts) {
    Polygon2 A = ToCgalPolygon(a_pts);
    Polygon2 B = ToCgalPolygon(b_pts);

    if (A.size() < 3 || B.size() < 3) return false;
    if (!A.is_simple() || !B.is_simple()) return false;

    if (SharesBoundarySegment(A, B)) return true;

    if (HasPositiveAreaOverlap(A, B)) return true;

    return false;
}

bool PointInOrOnPolygonCGAL(const Eigen::Vector2d& p,
                            const std::vector<Point2D>& poly_pts) {
    Polygon2 poly = ToCgalPolygon(poly_pts);
    CGALPoint q(p.x(), p.y());

    auto side = poly.bounded_side(q);
    return side == CGAL::ON_BOUNDED_SIDE || side == CGAL::ON_BOUNDARY;
}


HPolyhedron ConvexPolygonToHPolyhedron(const Shape& region) {
    if (region.points.size() < 3) {
        throw std::runtime_error("Free-space region has fewer than 3 vertices.");
    }

    std::vector<Point2D> pts = region.points;
    make_CCW(pts);

    if (!IsConvexPolygon(pts)) {
        throw std::runtime_error(
            "Encountered a free-space region that is not convex: " + region.name);
    }

    const int n = static_cast<int>(pts.size());
    Eigen::MatrixXd A(n, 2);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i) {
        const Point2D& p_i = pts[i];
        const Point2D& p_j = pts[(i + 1) % n];

        const double dx = p_j.x - p_i.x;
        const double dy = p_j.y - p_i.y;
        const double norm = std::hypot(dx, dy);

        if (norm < kEps) {
            throw std::runtime_error("Degenerate edge found in region: " + region.name);
        }

        // For CCW vertex ordering, [dy, -dx] is the outward normal.
        const double nx = dy / norm;
        const double ny = -dx / norm;

        A(i, 0) = nx;
        A(i, 1) = ny;
        b(i) = nx * p_i.x + ny * p_i.y;
    }

    return HPolyhedron(A, b);
}

ConvexSets BuildFreeRegionSets(const std::shared_ptr<MapData>& map) {
    if (!map) {
        throw std::runtime_error("BuildFreeRegionSets received null map.");
    }

    ConvexSets free_regions;
    free_regions.reserve(map->freespace_regions.size());

    for (const auto& region : map->freespace_regions) {
        if (region.type != ShapeType::FREESPACE) {
            continue;
        }
        std::cout << "Region: " << region.name << "\n";
        std::cout << "Num points: " << region.points.size() << "\n";
        for (const auto& p : region.points) {
            std::cout << "  (" << p.x << ", " << p.y << ")\n";
        }
        free_regions.emplace_back(std::make_unique<HPolyhedron>(
            ConvexPolygonToHPolyhedron(region)));
    }

    if (free_regions.empty()) {
        throw std::runtime_error("No free-space convex regions found in map.");
    }

    return free_regions;
}

void SaveTrajectoryCsv(
    const drake::trajectories::CompositeTrajectory<double>& traj,
    int num_samples,
    const std::string& filepath) {

    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open trajectory CSV: " + filepath);
    }

    file << "t,x,y\n";

    const double t0 = traj.start_time();
    const double tf = traj.end_time();

    for (int i = 0; i < num_samples; ++i) {
        const double alpha =
            (num_samples == 1) ? 0.0 : static_cast<double>(i) / (num_samples - 1);
        const double t = (1.0 - alpha) * t0 + alpha * tf;
        const Eigen::VectorXd q = traj.value(t);

        file << std::setprecision(16) << t << "," << q(0) << "," << q(1) << "\n";
    }
}

void SaveTextFile(const std::string& filepath, const std::string& text) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open text output file: " + filepath);
    }
    file << text;
}

}  // namespace

bool GcsPlannerPointRobot::SolvePath(
    const std::shared_ptr<MapData>& map,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    const GCSOptions& options) {

    if (!map) {
        throw std::runtime_error("SolvePath received null map.");
    }

    ConvexSets start_region;
    start_region.emplace_back(std::make_unique<Point>(start));

    ConvexSets goal_region;
    goal_region.emplace_back(std::make_unique<Point>(goal));

    // ConvexSets free_regions = BuildFreeRegionSets(map);

    GcsTrajectoryOptimization gcs_traj(options.dimension);

    auto& source = gcs_traj.AddRegions(
        start_region,
        /*order=*/0,
        /*h_min=*/1e-6,
        /*h_max=*/1.0,
        "start");

    std::vector<GcsTrajectoryOptimization::Subgraph*> free_subgraphs;
    std::vector<const Shape*> free_region_shapes;


    for (const auto& region : map->freespace_regions) {
        if (region.type != ShapeType::FREESPACE) {
            continue;
        }

        ConvexSets one_region;
        one_region.emplace_back(
            std::make_unique<HPolyhedron>(ConvexPolygonToHPolyhedron(region)));

        auto& subgraph = gcs_traj.AddRegions(
            one_region,
            options.bezier_order,
            options.h_min,
            options.h_max,
            region.name);

        free_region_shapes.push_back(&region);
        free_subgraphs.push_back(&subgraph);
    }

    auto& target = gcs_traj.AddRegions(
        goal_region,
        /*order=*/0,
        /*h_min=*/1e-6,
        /*h_max=*/1.0,
        "goal");

    // gcs_traj.AddEdges(source, free);
    // gcs_traj.AddEdges(free, target);
    // gcs_traj.AddEdges(source, target);  // Allows direct path if feasible.
    // gcs_traj.AddEdges(free, free);
    for (size_t i = 0; i < free_region_shapes.size(); ++i) {
        if (PointInOrOnPolygonCGAL(start, free_region_shapes[i]->points)) {
            gcs_traj.AddEdges(source, *free_subgraphs[i]);
        }
        if (PointInOrOnPolygonCGAL(goal, free_region_shapes[i]->points)) {
            gcs_traj.AddEdges(*free_subgraphs[i], target);
        }
    }
    for (size_t i = 0; i < free_region_shapes.size(); ++i) {
        for (size_t j = i + 1; j < free_region_shapes.size(); ++j) {
            if (AreAdjacentCGAL(free_region_shapes[i]->points,
                                free_region_shapes[j]->points)) {
                gcs_traj.AddEdges(*free_subgraphs[i], *free_subgraphs[j]);
                gcs_traj.AddEdges(*free_subgraphs[j], *free_subgraphs[i]);
            }
        }
    }

    gcs_traj.AddPathLengthCost(options.path_length_weight);
    gcs_traj.AddPathEnergyCost(options.path_energy_weight);
    gcs_traj.AddTimeCost(options.time_weight);
    if (options.continuity_order > 0){
        gcs_traj.AddPathContinuityConstraints(options.continuity_order);
        // gcs_traj.AddContinuityConstraints(options.continuity_order);
    }
    Eigen::VectorXd lb = Eigen::Map<const Eigen::VectorXd>(
    options.velocity_bounds.lb.data(),
    options.velocity_bounds.lb.size());

    Eigen::VectorXd ub = Eigen::Map<const Eigen::VectorXd>(
        options.velocity_bounds.ub.data(),
        options.velocity_bounds.ub.size());

    gcs_traj.AddVelocityBounds(lb, ub);

    // Eigen::Vector2d acc_lb(-2.0, -2.0);
    // Eigen::Vector2d acc_ub( 2.0,  2.0);
    // gcs_traj.AddNonlinearDerivativeBounds(acc_lb, acc_ub, 2);

    GraphOfConvexSetsOptions gcs_options;
    gcs_options.convex_relaxation = options.use_convex_relaxation;
    gcs_options.max_rounded_paths = options.max_rounded_paths;
    gcs_options.preprocessing = options.preprocessing;

    const auto [traj, result] = gcs_traj.SolvePath(source, target, gcs_options);
    std::cout << "result.is_success() = " << result.is_success() << "\n";
    std::cout << "solution_result = "
            << static_cast<int>(result.get_solution_result()) << "\n";
    std::cout << "solver name = " << result.get_solver_id().name() << "\n";

    if (!result.is_success()) {
        std::cerr << "GCS trajectory solve failed.\n";
        return false;
    }

    const auto normalized =
        GcsTrajectoryOptimization::NormalizeSegmentTimes(traj);

    std::cout << "Solved GCS trajectory.\n";
    std::cout << "Overall Cost: " << result.get_optimal_cost() << "\n";


    if (options.use_convex_relaxation) {
        std::cout << "\nRelaxed edge variables phi:\n";

        for (const auto* edge : gcs_traj.graph_of_convex_sets().Edges()) {
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

    for (const auto* edge : gcs_traj.graph_of_convex_sets().Edges()) {
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

    SaveTrajectoryCsv(normalized, options.num_samples, options.results_path + "trajectory.csv");
    SaveTextFile(options.results_path + "gcs_graphviz.dot", gcs_traj.GetGraphvizString(&result));

    std::cout << "Saved trajectory CSV to: " << options.results_path + "trajectory.csv" << "\n";
    std::cout << "Saved graphviz to: " << options.results_path + "gcs_graphviz.dot" << "\n";

    // for (int i = 0; i < options.num_samples; ++i) {
    //     const double alpha =
    //         (options.num_samples == 1)
    //             ? 0.0
    //             : static_cast<double>(i) / (options.num_samples - 1);
    //     const double t =
    //         (1.0 - alpha) * normalized.start_time() +
    //         alpha * normalized.end_time();
    //     const Eigen::VectorXd q = normalized.value(t);
    //     std::cout << "  t=" << t << "  q=" << q.transpose() << "\n";
    // }

    return true;
}