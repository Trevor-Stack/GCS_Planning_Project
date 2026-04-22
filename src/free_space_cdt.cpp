#include "free_space_cdt.h"

#include <vector>
#include <queue>
#include <iostream>
#include <stdexcept>

#include <algorithm>
#include <cmath>


namespace {

constexpr double kEps = 1e-9;

bool same_point(const Point2D& a, const Point2D& b, double eps = kEps) {
    return std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps;
}

double cross(const Point2D& o, const Point2D& a, const Point2D& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

double polygon_area(const std::vector<Point2D>& pts) {
    if (pts.size() < 3) return 0.0;

    double area = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Point2D& p = pts[i];
        const Point2D& q = pts[(i + 1) % pts.size()];
        area += p.x * q.y - p.y * q.x;
    }
    return 0.5 * std::abs(area);
}

std::vector<Point2D> unique_points(const std::vector<Point2D>& pts) {
    std::vector<Point2D> out;
    for (const auto& p : pts) {
        bool found = false;
        for (const auto& q : out) {
            if (same_point(p, q)) {
                found = true;
                break;
            }
        }
        if (!found) out.push_back(p);
    }
    return out;
}

std::vector<Point2D> convex_hull(std::vector<Point2D> pts) {
    pts = unique_points(pts);

    if (pts.size() <= 1) return pts;

    std::sort(pts.begin(), pts.end(), [](const Point2D& a, const Point2D& b) {
        if (std::abs(a.x - b.x) > kEps) return a.x < b.x;
        return a.y < b.y;
    });

    std::vector<Point2D> lower, upper;

    for (const auto& p : pts) {
        while (lower.size() >= 2 &&
               cross(lower[lower.size() - 2], lower[lower.size() - 1], p) <= kEps) {
            lower.pop_back();
        }
        lower.push_back(p);
    }

    for (int i = static_cast<int>(pts.size()) - 1; i >= 0; --i) {
        const auto& p = pts[i];
        while (upper.size() >= 2 &&
               cross(upper[upper.size() - 2], upper[upper.size() - 1], p) <= kEps) {
            upper.pop_back();
        }
        upper.push_back(p);
    }

    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());

    return lower;
}

int shared_vertex_count(const Shape& a, const Shape& b) {
    int count = 0;
    for (const auto& pa : a.points) {
        for (const auto& pb : b.points) {
            if (same_point(pa, pb)) {
                ++count;
                break;
            }
        }
    }
    return count;
}

bool can_merge_convex(const Shape& a, const Shape& b, std::vector<Point2D>& merged_pts) {
    // Must share an edge, not just a single vertex
    if (shared_vertex_count(a, b) < 2) {
        return false;
    }

    std::vector<Point2D> combined = a.points;
    combined.insert(combined.end(), b.points.begin(), b.points.end());

    std::vector<Point2D> hull = convex_hull(combined);

    double area_a = polygon_area(a.points);
    double area_b = polygon_area(b.points);
    double hull_area = polygon_area(hull);

    if (std::abs(hull_area - (area_a + area_b)) < 1e-8) {
        merged_pts = hull;
        return true;
    }

    return false;
}

}  // namespace



void FreeSpaceCDT::insert_polygon_constraint(CDT& cdt, const Shape& shape)
{
    if (shape.points.size() < 2) return;

    for (std::size_t i = 0; i < shape.points.size(); ++i)
    {
        const Point2D& a = shape.points[i];
        const Point2D& b = shape.points[(i + 1) % shape.points.size()];

        cdt.insert_constraint(Point(a.x, a.y), Point(b.x, b.y));
    }
}

void FreeSpaceCDT::mark_domains(CDT& cdt, Face_handle start, int index, std::list<Edge>& border)
{
    if (start->info().nesting_level != -1) return;

    std::queue<Face_handle> q;
    q.push(start);

    while (!q.empty())
    {
        Face_handle fh = q.front();
        q.pop();

        if (fh->info().nesting_level == -1)
        {
            fh->info().nesting_level = index;

            for (int i = 0; i < 3; ++i)
            {
                Edge e(fh, i);
                Face_handle n = fh->neighbor(i);

                if (n->info().nesting_level == -1)
                {
                    if (!cdt.is_constrained(e))
                    {
                        q.push(n);
                    }
                    else
                    {
                        border.push_back(e);
                    }
                }
            }
        }
    }
}

void FreeSpaceCDT::mark_domains(CDT& cdt)
{
    for (auto f = cdt.all_faces_begin(); f != cdt.all_faces_end(); ++f)
    {
        f->info().nesting_level = -1;
    }

    std::list<Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);

    while (!border.empty())
    {
        Edge e = border.front();
        border.pop_front();

        Face_handle n = e.first->neighbor(e.second);
        if (n->info().nesting_level == -1)
        {
            mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
        }
    }
}

void FreeSpaceCDT::triangulate_free_space(std::shared_ptr<MapData>& map)
{
    if (!map) {
        throw std::runtime_error("triangulate_free_space received null map");
    }

    if (map->AO.points.size() < 3) {
        throw std::runtime_error("AO polygon is invalid");
    }

    CDT cdt;

    // Insert outer boundary first
    insert_polygon_constraint(cdt, map->AO);

    // Insert all obstacles
    for (const auto& obstacle : map->obstacles) {
        insert_polygon_constraint(cdt, obstacle);
    }

    // Classify faces by nesting level
    mark_domains(cdt);

    // Print all free-space triangles
    for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
        if (fit->info().in_free_space()) {

            std::vector<Point2D> free_triangle_pts;

            Point p0 = fit->vertex(0)->point();
            free_triangle_pts.push_back(Point2D(p0.x(), p0.y()));

            Point p1 = fit->vertex(1)->point();
            free_triangle_pts.push_back(Point2D(p1.x(), p1.y()));

            Point p2 = fit->vertex(2)->point();
            free_triangle_pts.push_back(Point2D(p2.x(), p2.y()));

            double area = 0.5 * std::abs(
                (p1.x() - p0.x()) * (p2.y() - p0.y()) -
                (p2.x() - p0.x()) * (p1.y() - p0.y())
            );

            // std::cout << "Free-space triangle: "
            //         << "(" << p0.x() << ", " << p0.y() << "), "
            //         << "(" << p1.x() << ", " << p1.y() << "), "
            //         << "(" << p2.x() << ", " << p2.y() << ")"
            //         << " | Area: " << area << "\n";
            if (area < 0.01) {
                continue;
            }

            Shape new_triangle("free_space_shape", ShapeType::FREESPACE, free_triangle_pts);
            map->freespace_regions.push_back(new_triangle);
            
        }
    }
}
void FreeSpaceCDT::merge_free_space_triangles_into_convex_regions(std::shared_ptr<MapData>& map)
{
    if (!map) {
        throw std::runtime_error("merge_free_space_triangles_into_convex_regions received null map");
    }

    bool merged_something = true;
    int region_id = 0;

    while (merged_something) {
        merged_something = false;

        for (std::size_t i = 0; i < map->freespace_regions.size() && !merged_something; ++i) {
            for (std::size_t j = i + 1; j < map->freespace_regions.size() && !merged_something; ++j) {
                std::vector<Point2D> merged_pts;

                if (can_merge_convex(map->freespace_regions[i], map->freespace_regions[j], merged_pts)) {
                    Shape merged_shape(
                        "convex_region_" + std::to_string(region_id++),
                        ShapeType::FREESPACE,
                        merged_pts
                    );

                    map->freespace_regions[i] = merged_shape;
                    map->freespace_regions.erase(map->freespace_regions.begin() + j);

                    merged_something = true;
                }
            }
        }
    }
}


FreeSpaceCDT::FreeSpaceCDT(){

}