#pragma once

#include "gcs_structs.h"

#include <memory>
#include <list>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/Triangulation_data_structure_2.h>

struct FaceInfo2 {
    int nesting_level = -1;

    bool in_free_space() const {
        return nesting_level % 2 == 1;
    }
};

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_2;

using Vb = CGAL::Triangulation_vertex_base_2<K>;
using Fbb = CGAL::Triangulation_face_base_with_info_2<FaceInfo2, K>;
using Fb = CGAL::Constrained_triangulation_face_base_2<K, Fbb>;
using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using Itag = CGAL::Exact_intersections_tag;

using CDT = CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>;
using Face_handle = CDT::Face_handle;
using Edge = CDT::Edge;

class FreeSpaceCDT {
public:
    FreeSpaceCDT();

    void triangulate_free_space(std::shared_ptr<MapData>& map);
    void merge_free_space_triangles_into_convex_regions(std::shared_ptr<MapData>& map);

private:
    void insert_polygon_constraint(CDT& cdt, const Shape& shape);
    void mark_domains(CDT& cdt, Face_handle start, int index, std::list<Edge>& border);
    void mark_domains(CDT& cdt);
};