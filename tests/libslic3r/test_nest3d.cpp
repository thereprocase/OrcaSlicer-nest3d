// Behaviour tests for Nest3D, the shape-true arranger. Every test goes through the adapter, the
// interface the arrange job uses (nest3d::arrange_polygons, nest3d::ArrangeSettings and
// nest3d::candidate_angles), and judges the arranged model the way a user would see it printed: every
// part inside the bed, the spacing kept on the exact printed geometry, stacking only where the
// physical rules allow, no more plates than the outline arranger, piles centred on the anchor.
// Nothing here depends on the engine's internal types, which strategy won, or exact coordinates
// beyond a tolerance, so the engine can be rewritten without touching these tests.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "libslic3r/Nest3D/Footprint.hpp"
#include "libslic3r/Nest3D/Nest3DArrange.hpp"
#include "libslic3r/Nest3D/Report.hpp"

#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "libslic3r/libslic3r.h"

#include <tbb/global_control.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace Slic3r;
using arrangement::ArrangeParams;
using arrangement::ArrangePolygon;
using arrangement::ArrangePolygons;
using nest3d::ArrangeSettings;

namespace {

constexpr double Spacing   = 1.;   // mm between arranged parts
constexpr double CellMM    = 0.5;  // the engine's cell on beds up to 360 mm; "within one cell" below
constexpr double Tolerance = 1e-3; // mm
constexpr int    PlatesCap = 64;   // the arrange job's plate limit

// ---------------------------------------------------------------------------
// Meshes
// ---------------------------------------------------------------------------

// Closed box [0, x] x [0, y] x [0, z] moved by `at`.
indexed_triangle_set cube(double x, double y, double z, const Vec3f &at = Vec3f::Zero())
{
    indexed_triangle_set its = its_make_cube(x, y, z);
    its_translate(its, at);
    return its;
}

// Closed prism over a convex counter-clockwise polygon, from z0 to z1.
indexed_triangle_set prism(const std::vector<Vec2f> &poly, float z0, float z1)
{
    indexed_triangle_set its;
    const int            n = int(poly.size());
    for (float z : {z0, z1})
        for (const Vec2f &p : poly)
            its.vertices.emplace_back(p.x(), p.y(), z);
    for (int i = 1; i + 1 < n; ++i) {
        its.indices.emplace_back(0, i + 1, i);
        its.indices.emplace_back(n, n + i, n + i + 1);
    }
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        its.indices.emplace_back(i, j, n + j);
        its.indices.emplace_back(i, n + j, n + i);
    }
    return its;
}

std::vector<Vec2f> circle(float radius, int steps = 48)
{
    std::vector<Vec2f> out;
    for (int k = 0; k < steps; ++k)
        out.emplace_back(radius * std::cos(2. * PI * k / steps), radius * std::sin(2. * PI * k / steps));
    return out;
}

// Round ring about the origin, 10 mm tall: a hole of radius `inner` that encloses free bed.
indexed_triangle_set ring(float inner, float outer)
{
    const std::vector<Vec2f> in = circle(inner), out = circle(outer);
    indexed_triangle_set     its;
    for (size_t k = 0; k < in.size(); ++k) {
        const size_t j = (k + 1) % in.size();
        its_merge(its, prism({in[k], out[k], out[j], in[j]}, 0.f, 10.f));
    }
    return its;
}

// A table: a `side` mm square top from z = `under` to `under` + 2 on four 5 mm legs. The top is an
// overhang; without support, the space under it is free for parts short enough to clear it.
indexed_triangle_set table(float side, float under)
{
    const float          leg = 5.f;
    indexed_triangle_set its = cube(side, side, 2, {0.f, 0.f, under});
    for (Vec2f at : {Vec2f(0, 0), Vec2f(side - leg, 0), Vec2f(0, side - leg), Vec2f(side - leg, side - leg)})
        its_merge(its, cube(leg, leg, under, {at.x(), at.y(), 0.f}));
    return its;
}

using Squares = std::vector<std::pair<int, int>>;
const Squares TTetromino{{0, 0}, {1, 0}, {2, 0}, {1, 1}};
const Squares STetromino{{0, 0}, {1, 0}, {1, 1}, {2, 1}};
const Squares LTetromino{{0, 0}, {1, 0}, {2, 0}, {2, 1}};

// Polyomino of 10 mm squares, 5 mm tall.
indexed_triangle_set polyomino(const Squares &squares)
{
    indexed_triangle_set its;
    for (auto [x, y] : squares)
        its_merge(its, cube(10, 10, 5, {10.f * x, 10.f * y, 0.f}));
    return its;
}

// ---------------------------------------------------------------------------
// Model and arrange calls
// ---------------------------------------------------------------------------

ModelObject *add_object(Model &model, const indexed_triangle_set &its, int copies = 1, const std::string &name = "part")
{
    ModelObject *o = model.add_object(name.c_str(), "", TriangleMesh(its));
    for (int i = 0; i < copies; ++i)
        o->add_instance();
    return o;
}

Points rect_bed(double w, double h)
{
    return {Point(0, 0), Point(scaled(w), 0), Point(scaled(w), scaled(h)), Point(0, scaled(h))};
}

// The arrange dialog's defaults with rotations allowed, without the progress callback (it prints).
ArrangeParams quiet_params()
{
    ArrangeParams params;
    params.progressind     = nullptr;
    params.allow_rotations = true;
    return params;
}

ArrangeSettings nest_settings(bool stacking = false, float vertical_gap = 2.f)
{
    ArrangeSettings s;
    s.enabled         = true;
    s.allow_stacking  = stacking;
    s.vertical_gap_mm = vertical_gap;
    return s;
}

// The slicer's defaults without a brim: the tests that judge spacing alone should not print one. The ones
// that do set brim_type themselves.
DynamicPrintConfig base_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btNoBrim));
    return config;
}

const DynamicPrintConfig &print_config()
{
    static const DynamicPrintConfig config = base_config();
    return config;
}

// Every instance of `model` as the arrange job hands it over, spaced `Spacing` apart.
ArrangePolygons items_of(Model &model, double brim_width = 0.)
{
    ArrangePolygons items;
    for (ModelObject *o : model.objects)
        for (ModelInstance *mi : o->instances) {
            items.push_back(get_instance_arrange_poly(mi, print_config()));
            items.back().inflation = scaled(0.5 * Spacing);
            items.back().itemid    = int(items.size()) - 1;
            items.back().brim_width = brim_width;
        }
    return items;
}

// Arranges every instance of `model` with Nest3D and applies the result to the model.
ArrangePolygons nest(Model &model, const Points &bed, const ArrangeParams &params, const ArrangeSettings &settings,
                     const ArrangePolygons &fixed = {}, int max_plates = PlatesCap,
                     const DynamicPrintConfig &config = print_config(), double brim_width = 0.)
{
    ArrangePolygons items = items_of(model, brim_width);
    nest3d::arrange_polygons(items, fixed, bed, params, settings, config, max_plates);
    for (ArrangePolygon &ap : items)
        ap.apply();
    return items;
}

// The same with the outline arranger, the reference for the "never more plates" check.
ArrangePolygons stock(Model &model, const Points &bed, const ArrangeParams &params)
{
    ArrangePolygons items = items_of(model);
    arrangement::arrange(items, {}, bed, params);
    for (ArrangePolygon &ap : items)
        ap.apply();
    return items;
}

int plates_used(const ArrangePolygons &items)
{
    int plates = 0;
    for (const ArrangePolygon &ap : items)
        plates = std::max(plates, ap.bed_idx + 1);
    return plates;
}

int unplaced(const ArrangePolygons &items)
{
    return int(std::count_if(items.begin(), items.end(), [](const ArrangePolygon &ap) { return !ap.is_arranged(); }));
}

// ---------------------------------------------------------------------------
// Exact printed geometry
// ---------------------------------------------------------------------------

indexed_triangle_set printed_mesh(const ModelInstance &mi)
{
    indexed_triangle_set its = mi.get_object()->raw_indexed_triangle_set();
    its_transform(its, mi.get_matrix(), /*fix_left_handed=*/true);
    return its;
}

Polygons footprint(const indexed_triangle_set &its) { return project_mesh(its, Transform3d::Identity(), [] {}); }

// Plan view of the part of a closed mesh below height z. The lowest point of a solid on any
// vertical line lies on its surface, so projecting the surface clipped below z gives the solid below z.
Polygons footprint_below(const indexed_triangle_set &its, double z)
{
    Polygons tris;
    for (const stl_triangle_vertex_indices &f : its.indices) {
        std::vector<Vec3d> kept; // the triangle clipped to the half-space below z
        for (int k = 0; k < 3; ++k) {
            const Vec3d a = its.vertices[size_t(f[k])].cast<double>(), b = its.vertices[size_t(f[(k + 1) % 3])].cast<double>();
            if (a.z() <= z)
                kept.push_back(a);
            if ((a.z() <= z) != (b.z() <= z))
                kept.push_back(a + (z - a.z()) / (b.z() - a.z()) * (b - a));
        }
        if (kept.size() < 3)
            continue;
        Polygon p;
        for (const Vec3d &v : kept)
            p.points.emplace_back(scaled(v.x()), scaled(v.y()));
        if (std::abs(p.area()) <= 0.)
            continue;
        p.make_counter_clockwise();
        tris.push_back(std::move(p));
    }
    return union_(tris);
}

// Whether two plan views come closer than the spacing (by more than a rounding tolerance).
bool closer_than_spacing(const Polygons &a, const Polygons &b)
{
    const float half = float(scaled(0.5 * Spacing - 0.01));
    return !intersection(offset(a, half), offset(b, half)).empty();
}

struct Printed
{
    int                  plate = -1;
    indexed_triangle_set mesh;
    Polygons             plan;
    BoundingBoxf3        box;
};

std::vector<Printed> printed(const ArrangePolygons &items)
{
    std::vector<Printed> out;
    for (const ArrangePolygon &ap : items) {
        Printed p;
        p.plate = ap.is_arranged() ? ap.bed_idx : -1;
        p.mesh  = printed_mesh(*ap.instance);
        p.plan  = footprint(p.mesh);
        p.box   = bounding_box(p.mesh);
        out.push_back(std::move(p));
    }
    return out;
}

// Parts printing outside the bed outline.
int off_bed(const ArrangePolygons &items, const Points &bed)
{
    const Polygon outline = offset(Polygons{Polygon(bed)}, float(scaled(Tolerance))).front();
    int           n       = 0;
    for (const Printed &p : printed(items)) {
        if (p.plate < 0)
            continue;
        const bool outside = std::any_of(p.mesh.vertices.begin(), p.mesh.vertices.end(), [&](const Vec3f &v) {
            return !outline.contains(Point(scaled(v.x()), scaled(v.y())));
        });
        n += outside;
    }
    return n;
}

// Whether `upper` clears `lower` by the stacking rule: below the top of `lower` plus the vertical
// clearance, `upper` keeps the spacing from `lower`. With stacking off nothing may share a column.
bool stacked_clear(const Printed &upper, const Printed &lower, double vertical_gap)
{
    const Polygons below = footprint_below(upper.mesh, lower.box.max.z() + vertical_gap);
    return below.empty() || !closer_than_spacing(below, lower.plan);
}

// Pairs of parts on one plate that break the spacing in plan view without a valid stacking.
int collisions(const ArrangePolygons &items, bool stacking = false, double vertical_gap = 2.)
{
    const std::vector<Printed> parts = printed(items);
    int                        n     = 0;
    for (size_t i = 0; i < parts.size(); ++i)
        for (size_t j = 0; j < i; ++j) {
            const Printed &a = parts[i], &b = parts[j];
            if (a.plate < 0 || a.plate != b.plate || !closer_than_spacing(a.plan, b.plan))
                continue;
            const bool ok = stacking && (stacked_clear(a, b, vertical_gap) || stacked_clear(b, a, vertical_gap));
            if (!ok)
                UNSCOPED_INFO("parts " << i << " and " << j << " on plate " << a.plate << " collide");
            n += !ok;
        }
    return n;
}

// Parts printing within their own half of the spacing of an exclusion region, which has no
// spacing of its own.
int in_region(const ArrangePolygons &items, const Polygon &region)
{
    const float half = float(scaled(0.5 * Spacing - 0.01));
    int         n    = 0;
    for (const Printed &p : printed(items))
        n += p.plate >= 0 && !intersection(offset(p.plan, half), Polygons{region}).empty();
    return n;
}

// Plan-view bounding box of the parts on one plate.
BoundingBoxf plate_box(const ArrangePolygons &items, int plate)
{
    BoundingBoxf bb;
    for (const Printed &p : printed(items))
        if (p.plate == plate)
            bb.merge(BoundingBoxf(Vec2d(p.box.min.x(), p.box.min.y()), Vec2d(p.box.max.x(), p.box.max.y())));
    return bb;
}

Vec2d centre_of(const ArrangePolygon &ap)
{
    const BoundingBoxf3 bb = bounding_box(printed_mesh(*ap.instance));
    return {0.5 * (bb.min.x() + bb.max.x()), 0.5 * (bb.min.y() + bb.max.y())};
}

// Random boxes and L shapes, 8 to 45 mm across and 3 to 40 mm tall, one object each.
void add_random_parts(Model &model, int n, unsigned seed)
{
    std::mt19937                          rng(seed);
    std::uniform_real_distribution<float> size(8.f, 45.f), height(3.f, 40.f);
    for (int i = 0; i < n; ++i) {
        const float          a = size(rng), b = size(rng), h = height(rng);
        indexed_triangle_set its;
        if (i % 3 == 0) {
            its = cube(a, a / 3, h);
            its_merge(its, cube(a / 3, std::max(b - a / 3, 3.f), h, {0.f, a / 3, 0.f}));
        } else {
            its = cube(a, b, h);
        }
        add_object(model, its, 1, "p" + std::to_string(i));
    }
}

// Copies a plain grid of the part's bounding box holds on a w x h bed, in its better orientation.
int grid_count(double part_w, double part_h, double bed_w, double bed_h)
{
    auto along = [](double bed, double part) { return int(std::floor((bed + Spacing) / (part + Spacing))); };
    return std::max(along(bed_w, part_w) * along(bed_h, part_h), along(bed_w, part_h) * along(bed_h, part_w));
}

// Per object name: plate, x, y (um) and rotation (urad) of its first instance.
std::map<std::string, std::tuple<int, long, long, long>> placements_by_name(const ArrangePolygons &items)
{
    std::map<std::string, std::tuple<int, long, long, long>> out;
    for (const ArrangePolygon &ap : items) {
        const Vec3d off = ap.instance->get_offset();
        out[ap.instance->get_object()->name] = {ap.bed_idx, std::lround(off.x() * 1e3), std::lround(off.y() * 1e3),
                                                std::lround(ap.rotation * 1e6)};
    }
    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// Safety: inside the bed, spacing kept, plates
// ---------------------------------------------------------------------------

TEST_CASE("Every arranged part prints inside the bed and keeps its spacing, on every plate", "[Nest3D]")
{
    // 30 random parts overflow a 120 mm bed, so several plates are used.
    const unsigned seed = GENERATE(1u, 2u, 3u);
    Model          model;
    add_random_parts(model, 30, seed);
    const Points          bed   = rect_bed(120, 120);
    const ArrangePolygons items = nest(model, bed, quiet_params(), nest_settings());
    INFO("seed " << seed);
    CHECK(unplaced(items) == 0);
    CHECK(plates_used(items) > 1);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
}

TEST_CASE("Arranged parts keep their spacing as printed, mirrored or not", "[Nest3D]")
{
    // Two Ls of 30 mm arms and 10 mm width fit a 42 mm bed only interlocked, one turned half a
    // turn. Mirrored, an L is a J: a footprint that missed the mirroring would pack Ls whose printed
    // Js overlap or leave the bed.
    const bool           mirrored = GENERATE(false, true);
    Model                model;
    indexed_triangle_set l = cube(30, 10, 5);
    its_merge(l, cube(10, 20, 5, {0.f, 10.f, 0.f}));
    ModelObject *o = add_object(model, l, 0);
    for (int i = 0; i < 2; ++i)
        o->add_instance()->set_mirror(X, mirrored ? -1. : 1.);
    const Points          bed   = rect_bed(42, 42);
    const ArrangePolygons items = nest(model, bed, quiet_params(), nest_settings());
    INFO("mirrored " << mirrored);
    CHECK(plates_used(items) == 1);
    CHECK(unplaced(items) == 0);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
}

TEST_CASE("Parts on a round bed stay inside its outline", "[Nest3D]")
{
    // 60 cubes of 8 mm overflow a bed of radius 50 mm centred on the origin.
    const double radius = 50.;
    Points       round_bed;
    for (int k = 0; k < 360; ++k)
        round_bed.emplace_back(scaled(radius * std::cos(k * PI / 180)), scaled(radius * std::sin(k * PI / 180)));
    Model model;
    add_object(model, cube(8, 8, 5), 60);
    const ArrangePolygons items = nest(model, round_bed, quiet_params(), nest_settings());
    CHECK(unplaced(items) == 0);
    CHECK(off_bed(items, round_bed) == 0);
    CHECK(collisions(items) == 0);
}

TEST_CASE("Nest3D never uses more plates than the outline arranger", "[Nest3D]")
{
    // Two 20 mm cubes on a 25 mm bed need a plate each, even capped at one plate; and random mixes
    // that overflow a small bed.
    const int  seed  = GENERATE(0, 1, 2);
    const bool cubes = seed == 0;
    auto       build = [&](Model &model) {
        if (cubes)
            add_object(model, cube(20, 20, 5), 2);
        else
            add_random_parts(model, 40, unsigned(seed));
    };
    const Points bed = cubes ? rect_bed(25, 25) : rect_bed(100, 100);
    const int    cap = cubes ? 1 : PlatesCap;
    Model        stock_model;
    Model        nest_model;
    build(stock_model);
    build(nest_model);
    const ArrangePolygons outline = stock(stock_model, bed, quiet_params());
    const ArrangePolygons items   = nest(nest_model, bed, quiet_params(), nest_settings(), {}, cap);
    INFO("case " << seed << ": stock " << plates_used(outline) << " plates, " << unplaced(outline) << " unplaced; nest3d "
                 << plates_used(items) << " plates, " << unplaced(items) << " unplaced");
    CHECK(unplaced(items) <= unplaced(outline));
    CHECK(plates_used(items) <= plates_used(outline));
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
}

TEST_CASE("Parts avoid exclusion regions and fixed parts, wherever the pile is centred", "[Nest3D]")
{
    // A 40 mm exclusion square and a fixed 30 mm cube where the pile would centre, with the anchor
    // at the bed centre or near a corner. Centring may only shift the pile onto free bed.
    const auto [ax, ay] = GENERATE(std::make_pair(0.5, 0.5), std::make_pair(0.95, 0.05));
    const Points bed    = rect_bed(200, 200);
    Model        model;
    add_random_parts(model, 12, 7u);
    Model        fixed_model;
    ModelObject *fixed_object = add_object(fixed_model, cube(30, 30, 10));
    fixed_object->instances.front()->set_offset(Vec3d(120., 60., 0.));
    ArrangePolygon fixed = get_instance_arrange_poly(fixed_object->instances.front(), print_config());
    fixed.bed_idx        = 0;
    fixed.inflation      = scaled(0.5 * Spacing);

    const Polygon  zone({Point(scaled(80.), scaled(80.)), Point(scaled(120.), scaled(80.)), Point(scaled(120.), scaled(120.)),
                         Point(scaled(80.), scaled(120.))});
    ArrangePolygon excluded;
    excluded.poly.contour = zone;
    ArrangeParams params  = quiet_params();
    params.align_center   = Vec2d(ax, ay);
    params.excluded_regions.push_back(excluded);

    const ArrangePolygons items = nest(model, bed, params, nest_settings(), {fixed});
    INFO("anchor " << ax << ", " << ay);
    CHECK(unplaced(items) == 0);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
    CHECK(in_region(items, zone) == 0);
    int near_fixed = 0;
    for (const Printed &p : printed(items))
        near_fixed += p.plate == 0 && closer_than_spacing(p.plan, footprint(printed_mesh(*fixed_object->instances.front())));
    CHECK(near_fixed == 0);
}

// A P1S-like printer on a 256 mm bed: `exclude` as bed_exclude_area, `start` as the start G-code, and the
// excluded regions the plate hands the arranger for it, the bounding box of every four points.
struct Printer
{
    DynamicPrintConfig config = base_config();
    ArrangeParams      params = quiet_params();
};

Printer printer(const std::vector<Vec2d> &exclude, const std::string &start = "")
{
    Printer p;
    p.config.set_key_value("printable_area", new ConfigOptionPoints({Vec2d(0, 0), Vec2d(256, 0), Vec2d(256, 256), Vec2d(0, 256)}));
    p.config.set_key_value("bed_exclude_area", new ConfigOptionPoints(exclude));
    p.config.set_key_value("extruder_offset", new ConfigOptionPoints({Vec2d(0, 2)}));
    if (!start.empty())
        p.config.set_key_value("machine_start_gcode", new ConfigOptionString(start));
    for (size_t i = 0; i + 3 < exclude.size(); i += 4) {
        BoundingBoxf box;
        for (size_t k = i; k < i + 4; ++k)
            box.merge(exclude[k]);
        ArrangePolygon region;
        region.poly.contour = Polygon({scaled(box.min), scaled(Vec2d(box.max.x(), box.min.y())), scaled(box.max),
                                       scaled(Vec2d(box.min.x(), box.max.y()))});
        region.inflation    = scaled(0.5 * Spacing);
        p.params.excluded_regions.push_back(region);
    }
    return p;
}

// Parts closer than the spacing to `region`: its excluded-region inflation and the part's own half.
int near_region(const ArrangePolygons &items, const Polygon &region)
{
    const float spacing = float(scaled(Spacing - 0.01));
    int         n       = 0;
    for (const Printed &p : printed(items))
        n += p.plate >= 0 && !intersection(offset(p.plan, spacing), Polygons{region}).empty();
    return n;
}

Polygon rect(double x0, double y0, double x1, double y1)
{
    return Polygon({Point(scaled(x0), scaled(y0)), Point(scaled(x1), scaled(y0)), Point(scaled(x1), scaled(y1)),
                    Point(scaled(x0), scaled(y1))});
}

TEST_CASE("Parts stay off every corner of an excluded area polygon", "[Nest3D]")
{
    // An L over the front edge and a cutter in the front-left corner. The plate passes the box of the
    // first four corners, the front strip; the cutter arm is only in the polygon. More 20 mm squares than
    // one plate holds, so the first plate fills up to its corners.
    const std::vector<Vec2d> l{{0, 0}, {256, 0}, {256, 10}, {23, 10}, {23, 33}, {0, 33}};
    const Printer            p   = printer(l);
    const Points             bed = rect_bed(256, 256);
    Model                    model;
    add_object(model, cube(20, 20, 4), 150);
    const ArrangePolygons items = nest(model, bed, p.params, nest_settings(), {}, PlatesCap, p.config);
    CHECK(unplaced(items) == 0);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
    CHECK(near_region(items, rect(0, 10, 23, 33)) == 0);
    CHECK(near_region(items, rect(0, 0, 256, 10)) == 0);
}

TEST_CASE("Parts stay off the purge line the start G-code lays", "[Nest3D]")
{
    // A strip along the front edge from X22 to X240, 0.2 mm from Y0 in G-code coordinates, which the 2 mm
    // extruder offset puts at Y2 on the plate. E is an expression over the filament settings.
    const std::string start = "G28\nG90\nM83\nG1 X22 Y0 Z0.3 F12000\n"
                              "{local fa = 0.7853981633974483 * filament_diameter[0] * filament_diameter[0]}\n"
                              "G1 X240 Y0 E{21.9 / fa}\nG1 E-0.5\nG1 Z1\n";
    const Printer p   = printer({{0, 0}, {18, 0}, {18, 28}, {0, 28}}, start);
    const Points  bed = rect_bed(256, 256);
    Model         model;
    add_object(model, cube(20, 20, 4), 150);
    const ArrangePolygons items = nest(model, bed, p.params, nest_settings(), {}, PlatesCap, p.config);
    CHECK(unplaced(items) == 0);
    CHECK(collisions(items) == 0);
    CHECK(near_region(items, rect(0, 0, 18, 28)) == 0);
    // The bead, as wide as the 0.4 mm nozzle, with square ends.
    CHECK(near_region(items, rect(21.8, 1.8, 240.2, 2.2)) == 0);
}

// Shortest distance from `box`, a plan-view bounding box, to the edges of the rectangle [0, w] x [0, h].
double edge_distance(const BoundingBoxf &box, double w, double h)
{
    return std::min({box.min.x(), box.min.y(), w - box.max.x(), h - box.max.y()});
}

TEST_CASE("A big disc and a small one share a plate that the old edge margin split", "[Nest3D]")
{
    // The discs (radii 99 and 49 mm) fit a 256 mm bed only pushed into opposite corners, clear of the
    // cutter zone and the purge strip along the front. Their centres have to be 149 mm apart: the
    // diagonal leaves about 150 mm when the geometry may reach the bed's edge, and under 148 mm when the
    // parts also keep a margin from it.
    const std::vector<Vec2d> cutter{{0, 0}, {18, 0}, {18, 28}, {0, 28}};
    const std::string        start = "G28\nG90\nM83\nG1 X22 Y0 Z0.3 F12000\n"
                                     "{local fa = 0.7853981633974483 * filament_diameter[0] * filament_diameter[0]}\n"
                                     "G1 X240 Y0 E{21.9 / fa}\nG1 E-0.5\nG1 Z1\n";
    Printer p = printer(cutter, start);
    // The bed as the printable area, as a caller hands it over without a shrink.
    p.params.bed_shrink_x = p.params.bed_shrink_y = 0.f;
    const Points bed = rect_bed(256, 256);
    Model        model;
    add_object(model, prism(circle(99.f, 96), 0.f, 5.f), 1, "big");
    add_object(model, prism(circle(49.f, 96), 0.f, 5.f), 1, "small");
    const ArrangePolygons items = nest(model, bed, p.params, nest_settings(), {}, PlatesCap, p.config);
    std::string boxes;
    for (const Printed &part : printed(items))
        boxes += "plate " + std::to_string(part.plate) + " box " + std::to_string(part.box.min.x()) + "," + std::to_string(part.box.min.y()) + " - " + std::to_string(part.box.max.x()) + "," + std::to_string(part.box.max.y()) + "; ";
    INFO(boxes);
    CHECK(unplaced(items) == 0);
    CHECK(plates_used(items) == 1);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
    CHECK(near_region(items, rect(0, 0, 18, 28)) == 0);
    CHECK(near_region(items, rect(21.8, 1.8, 240.2, 2.2)) == 0);
}

TEST_CASE("A brim stays on the plate at the bed's corner", "[Nest3D]")
{
    // The printable area is the bed, so the geometry may reach its edge, and a brim, which prints
    // beyond the geometry, may not. Nine 39 mm cubes fill a 120 mm bed 3 x 3 with 1 mm between them,
    // which leaves no margin to the edge: they fit one plate only without a brim.
    const double brim = GENERATE(0., 5.);
    Model        model;
    add_object(model, cube(39, 39, 10), 9);
    ArrangeParams params = quiet_params();
    params.bed_shrink_x = params.bed_shrink_y = 0.f;
    DynamicPrintConfig config = base_config();
    config.set_key_value("printable_area", new ConfigOptionPoints({Vec2d(0, 0), Vec2d(120, 0), Vec2d(120, 120), Vec2d(0, 120)}));
    config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(brim > 0. ? btOuterOnly : btNoBrim));
    config.set_key_value("brim_width", new ConfigOptionFloat(brim));
    const Points          bed   = rect_bed(120, 120);
    const ArrangePolygons items = nest(model, bed, params, nest_settings(), {}, PlatesCap, config, brim);
    // The brim Orca makes is a whole number of beads wide, a little under the setting.
    const double reach = brim > 0. ? nest3d::object_brim(*model.objects.front()->instances.front(), Transform3d::Identity(), config, {}).reach : 0.;
    INFO("brim " << brim << " reaches " << reach);
    CHECK(unplaced(items) == 0);
    CHECK(collisions(items) == 0);
    CHECK((plates_used(items) == 1) == (brim == 0.));
    for (const Printed &part : printed(items)) {
        const double edge = edge_distance(BoundingBoxf(Vec2d(part.box.min.x(), part.box.min.y()), Vec2d(part.box.max.x(), part.box.max.y())), 120, 120);
        CHECK(edge >= reach - Tolerance);
    }
}

TEST_CASE("A skirt stays on the plate at the bed's corner", "[Nest3D]")
{
    // The skirt prints its distance and all of its loops out from the geometry. The bed comes as the
    // arrange job builds it: the printable area less a shrink that update_arrange_params() has grown by
    // that full skirt width.
    DynamicPrintConfig config = base_config();
    config.set_key_value("printable_area", new ConfigOptionPoints({Vec2d(0, 0), Vec2d(120, 0), Vec2d(120, 120), Vec2d(0, 120)}));
    config.set_key_value("skirt_loops", new ConfigOptionInt(2));
    config.set_key_value("skirt_height", new ConfigOptionInt(1));
    config.set_key_value("skirt_distance", new ConfigOptionFloat(3.));
    ArrangeParams params = quiet_params();
    params.bed_shrink_x = params.bed_shrink_y = 0.f;
    arrangement::update_arrange_params(params, &config, {});
    const double skirt = get_real_skirt_dist(config);
    REQUIRE(skirt > 3.); // the distance and the loops' width
    Model model;
    add_object(model, cube(39, 39, 10), 9);
    const ArrangePolygons items = nest(model, get_shrink_bedpts(&config, params), params, nest_settings(), {}, PlatesCap, config);
    CHECK(unplaced(items) == 0);
    CHECK(collisions(items) == 0);
    CHECK(plates_used(items) > 1); // 3 x 3 cubes leave no room for a skirt
    for (const Printed &part : printed(items))
        CHECK(edge_distance(BoundingBoxf(Vec2d(part.box.min.x(), part.box.min.y()), Vec2d(part.box.max.x(), part.box.max.y())), 120, 120) >=
              skirt - Tolerance);
}

TEST_CASE("A start G-code without extrusion leaves the arrangement unchanged", "[Nest3D]")
{
    const std::string start = "G28 ; home\nG1 Z5 F5000\nG92 E0\nG1 X100 Y-2 F3000\nG1 E-1 F300\n";
    const Printer     with_start = printer({{0, 0}, {18, 0}, {18, 28}, {0, 28}}, start);
    const Printer     without    = printer({{0, 0}, {18, 0}, {18, 28}, {0, 28}});
    const Points      bed        = rect_bed(256, 256);
    Model             a, b;
    add_random_parts(a, 30, 11u);
    add_random_parts(b, 30, 11u);
    const ArrangePolygons first  = nest(a, bed, with_start.params, nest_settings(), {}, PlatesCap, with_start.config);
    const ArrangePolygons second = nest(b, bed, without.params, nest_settings(), {}, PlatesCap, without.config);
    REQUIRE(first.size() == second.size());
    for (size_t i = 0; i < first.size(); ++i) {
        INFO("item " << i);
        CHECK(first[i].bed_idx == second[i].bed_idx);
        CHECK(first[i].translation == second[i].translation);
        CHECK_THAT(first[i].rotation, Catch::Matchers::WithinAbs(second[i].rotation, 1e-9));
    }
}

TEST_CASE("Parts keep clear of a fixed part whose Euler angles hide a half turn about Z", "[Nest3D]")
{
    // A fixed L flipped about X and Y and turned a quarter turn about Z. The two flips add up to a half
    // turn about Z, which the stored Euler angles (pi, pi, pi/2) keep out of the Z angle, so only the
    // printed geometry tells where the L is. Its arms reach out from the bed centre, where the pile grows,
    // so parts packed around a ghost L half a turn away would land on the real one.
    const Points         bed = rect_bed(120, 120);
    Model                fixed_model;
    indexed_triangle_set l = cube(60, 15, 10);
    its_merge(l, cube(15, 60, 10));
    ModelInstance *fixed_instance = add_object(fixed_model, l, 1, "L")->instances.front();
    fixed_instance->set_offset(Vec3d(60., 60., 0.));
    fixed_instance->set_rotation(Vec3d(PI, PI, 0.5 * PI));
    REQUIRE_THAT(fixed_instance->get_rotation().x(), Catch::Matchers::WithinAbs(PI, 1e-6)); // the half turn stays hidden
    ArrangePolygon fixed = get_instance_arrange_poly(fixed_instance, print_config());
    fixed.bed_idx        = 0;
    fixed.inflation      = scaled(0.5 * Spacing);

    Model model;
    add_object(model, cube(20, 20, 10), 12);
    const ArrangePolygons items = nest(model, bed, quiet_params(), nest_settings(), {fixed});
    CHECK(unplaced(items) == 0);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
    const Polygons printed_l   = footprint(printed_mesh(*fixed_instance));
    int            near_fixed = 0;
    for (const Printed &p : printed(items))
        near_fixed += p.plate == 0 && closer_than_spacing(p.plan, printed_l);
    CHECK(near_fixed == 0);
}

TEST_CASE("A part larger than the bed is left unplaced and the rest still arranges", "[Nest3D]")
{
    // 300 mm square: too large for a 256 mm bed at any angle. (A long thin part can still fit
    // along the diagonal, so it would not test this.)
    Model model;
    add_object(model, cube(300, 300, 5), 1, "too large");
    add_object(model, cube(20, 20, 5), 3, "small");
    const Points          bed   = rect_bed(256, 256);
    const ArrangePolygons items = nest(model, bed, quiet_params(), nest_settings());
    for (const ArrangePolygon &ap : items) {
        INFO(ap.instance->get_object()->name);
        CHECK(ap.is_arranged() == (ap.instance->get_object()->name == "small"));
    }
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
}

TEST_CASE("An empty arrange does nothing and does not fail", "[Nest3D]")
{
    ArrangePolygons none;
    CHECK_NOTHROW(nest3d::arrange_polygons(none, {}, rect_bed(100, 100), quiet_params(), nest_settings(), print_config(), PlatesCap));
    CHECK(none.empty());
}

// ---------------------------------------------------------------------------
// Pockets and stacking
// ---------------------------------------------------------------------------

TEST_CASE("Small parts land inside the hole of a ring", "[Nest3D]")
{
    // A ring with a 30 mm hole and four 8 mm pegs on a bed with plenty of room around it. The hole
    // holds all four pegs 1 mm apart; outside it they would only add to the pile.
    Model model;
    add_object(model, ring(15.f, 25.f), 1, "ring");
    add_object(model, cube(8, 8, 6), 4, "peg");
    const Points          bed   = rect_bed(150, 150);
    const ArrangePolygons items = nest(model, bed, quiet_params(), nest_settings());
    REQUIRE(unplaced(items) == 0);
    REQUIRE(plates_used(items) == 1);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
    const Vec2d ring_centre = centre_of(items.front());
    for (size_t i = 1; i < items.size(); ++i) {
        const double d = (centre_of(items[i]) - ring_centre).norm();
        INFO("peg " << i << " centre " << d << " mm from the ring centre");
        CHECK(d < 15. - 4.);
    }
}

TEST_CASE("A short part tucks under an overhang only when stacking is on and the clearance allows", "[Nest3D]")
{
    // A 60 mm table with its top from 20 to 22 mm fills a 62 mm bed; a 40 mm block fits only under
    // the top, between the legs. It may go there only with stacking on, only when its top plus the
    // vertical clearance stays below the table top, and never under a table printed with support,
    // whose support fills the space below the top.
    struct Case
    {
        float height;
        float gap;
        bool  stacking;
        bool  support;
        bool  tucks;
    };
    const Case c = GENERATE(Case{6.f, 2.f, true, false, true}, Case{17.f, 2.f, true, false, true}, Case{19.f, 2.f, true, false, false},
                            Case{17.f, 5.f, true, false, false}, Case{6.f, 2.f, false, false, false}, Case{6.f, 2.f, true, true, false});
    Model        model;
    ModelObject *top = add_object(model, table(60.f, 20.f), 1, "table");
    if (c.support)
        top->config.set_key_value("enable_support", new ConfigOptionBool(true));
    add_object(model, cube(40, 40, c.height), 1, "block");
    const Points          bed   = rect_bed(62, 62);
    const ArrangePolygons items = nest(model, bed, quiet_params(), nest_settings(c.stacking, c.gap));
    INFO("block " << c.height << " mm, clearance " << c.gap << " mm, stacking " << c.stacking << ", support " << c.support);
    CHECK(unplaced(items) == 0);
    CHECK((items[0].bed_idx == items[1].bed_idx) == c.tucks);
    CHECK(off_bed(items, bed) == 0);
    // The exact check applies the clearance asked for, so a tuck that ignored it fails here too.
    CHECK(collisions(items, c.stacking && !c.support, c.gap) == 0);
}

// ---------------------------------------------------------------------------
// Centring
// ---------------------------------------------------------------------------

TEST_CASE("A single part lands centred on the anchor within one cell", "[Nest3D]")
{
    const auto [ax, ay] = GENERATE(std::make_pair(0.5, 0.5), std::make_pair(0.25, 0.75));
    Model      model;
    add_object(model, cube(30, 20, 10));
    ArrangeParams params = quiet_params();
    params.align_center  = Vec2d(ax, ay);
    const ArrangePolygons items = nest(model, rect_bed(200, 200), params, nest_settings());
    REQUIRE(items.front().is_arranged());
    const Vec2d c = centre_of(items.front());
    INFO("anchor " << ax << ", " << ay << ": centre " << c.x() << ", " << c.y());
    CHECK(std::abs(c.x() - 200. * ax) <= CellMM + Tolerance);
    CHECK(std::abs(c.y() - 200. * ay) <= CellMM + Tolerance);
}

TEST_CASE("Four equal cubes form a centred 2 x 2 block", "[Nest3D]")
{
    // A strip or an L has the same or a larger bounding box than the square but is not round.
    Model model;
    add_object(model, cube(20, 20, 10), 4);
    const ArrangePolygons items = nest(model, rect_bed(200, 200), quiet_params(), nest_settings());
    REQUIRE(unplaced(items) == 0);
    REQUIRE(plates_used(items) == 1);
    CHECK(collisions(items) == 0);
    const BoundingBoxf bb = plate_box(items, 0);
    const Vec2d        size = bb.size(), c = bb.center();
    INFO("pile " << size.x() << " x " << size.y() << " mm centred at " << c.x() << ", " << c.y());
    CHECK(std::max(size.x(), size.y()) <= 1.5 * std::min(size.x(), size.y()));
    CHECK(std::abs(c.x() - 100.) <= CellMM + Tolerance);
    CHECK(std::abs(c.y() - 100.) <= CellMM + Tolerance);
}

TEST_CASE("A tall part sits nearer the centre than the short parts around it", "[Nest3D]")
{
    // Nine 25 mm squares, one 60 mm tall and eight 8 mm tall, form a 3 x 3 block. Weighted by
    // height, the block is most central with the tall part in the middle: when all else is equal,
    // clustering tall parts in the middle reduces travel. Where the tall part is listed must not matter.
    const int position = GENERATE(0, 4, 8);
    Model     model;
    for (int i = 0; i < 9; ++i)
        add_object(model, cube(25, 25, i == position ? 60 : 8), 1, i == position ? "tall" : "short");
    const ArrangePolygons items = nest(model, rect_bed(200, 200), quiet_params(), nest_settings());
    REQUIRE(unplaced(items) == 0);
    CHECK(collisions(items) == 0);
    const Vec2d centre(100., 100.);
    double      tall = 0., nearest_short = 1e9;
    for (const ArrangePolygon &ap : items) {
        const double d = (centre_of(ap) - centre).norm();
        if (ap.instance->get_object()->name == "tall")
            tall = d;
        else
            nearest_short = std::min(nearest_short, d);
    }
    INFO("tall part listed at " << position << ": " << tall << " mm from the centre, nearest short part " << nearest_short << " mm");
    CHECK(tall + CellMM < nearest_short);
}

// ---------------------------------------------------------------------------
// Identical copies (lattice)
// ---------------------------------------------------------------------------

TEST_CASE("Copies of a tetromino fill a plate at least as densely as a grid of their boxes", "[Nest3D]")
{
    // 40 copies of a tetromino of 10 mm squares overflow a 120 mm bed. A grid of 31 x 21 mm boxes
    // holds 15; the area bound (each copy claims at least 451 mm^2 with its spacing) is 31.
    const auto [name, squares] = GENERATE(table<std::string, Squares>({{"T", TTetromino}, {"S", STetromino}, {"L", LTetromino}}));
    Model model;
    add_object(model, polyomino(squares), 40);
    const Points          bed   = rect_bed(120, 120);
    const ArrangePolygons items = nest(model, bed, quiet_params(), nest_settings());
    const int first = int(std::count_if(items.begin(), items.end(), [](const ArrangePolygon &ap) { return ap.bed_idx == 0; }));
    INFO(name << " tetromino: " << first << " copies on the first plate, grid " << grid_count(30, 20, 120, 120));
    CHECK(first >= grid_count(30, 20, 120, 120));
    CHECK(first >= 22); // 70% of the area bound
    CHECK(unplaced(items) == 0);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
}

TEST_CASE("Two copies of an S interlock on a bed that holds only the interlocked pair", "[Nest3D]")
{
    // Interlocked side by side, the pair spans 51 x 21 mm, plus half the spacing on each side; side
    // by side as boxes it needs 61 mm, stacked as boxes 41 mm.
    Model model;
    add_object(model, polyomino(STetromino), 2);
    ArrangeParams params   = quiet_params();
    params.allow_rotations = false;
    const Points          bed   = rect_bed(52.5, 22.5);
    const ArrangePolygons items = nest(model, bed, params, nest_settings());
    CHECK(plates_used(items) == 1);
    CHECK(unplaced(items) == 0);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
}

TEST_CASE("Copies of a disc fill a bed that holds them only on a square grid", "[Nest3D]")
{
    // Discs 40 mm across, 1 mm apart: a 132 mm bed holds 3 x 3 of them on a square grid, but only
    // 3 + 2 + 3 in the denser hexagonal rows. The densest lattice is not always the best on a finite bed.
    Model model;
    add_object(model, prism(circle(20.f), 0.f, 5.f), 9);
    const Points          bed   = rect_bed(132, 132);
    const ArrangePolygons items = nest(model, bed, quiet_params(), nest_settings());
    CHECK(plates_used(items) == 1);
    CHECK(unplaced(items) == 0);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
}

TEST_CASE("Copies left at different quarter turns pack as densely as aligned copies", "[Nest3D]")
{
    // An earlier arrange leaves copies at mixed quarter turns; they are still one group of copies.
    int first[2] = {0, 0};
    for (int mixed : {0, 1}) {
        Model        model;
        ModelObject *o = add_object(model, polyomino(LTetromino), 0);
        for (int i = 0; i < 35; ++i)
            o->add_instance()->set_rotation(Z, mixed ? (i % 4) * 0.5 * PI : 0.);
        const ArrangePolygons items = nest(model, rect_bed(120, 120), quiet_params(), nest_settings());
        first[mixed] = int(std::count_if(items.begin(), items.end(), [](const ArrangePolygon &ap) { return ap.bed_idx == 0; }));
        CHECK(collisions(items) == 0);
    }
    INFO("aligned " << first[0] << ", mixed " << first[1]);
    CHECK(first[0] >= 22);
    CHECK(first[1] == first[0]);
}

TEST_CASE("Copies at different quarter turns are offered the same candidate angles", "[Nest3D]")
{
    const std::vector<double> offsets{0., 0.5 * PI, PI, 1.5 * PI};
    const std::vector<double> base = nest3d::candidate_angles(0., offsets);
    for (int q = 1; q < 4; ++q) {
        const std::vector<double> turned = nest3d::candidate_angles(q * 0.5 * PI, offsets);
        REQUIRE(turned.size() == base.size());
        for (size_t i = 0; i < base.size(); ++i)
            CHECK(std::abs(turned[i] - base[i]) < 1e-9);
    }
}

// ---------------------------------------------------------------------------
// Determinism, order, cancellation
// ---------------------------------------------------------------------------

TEST_CASE("The result does not depend on the order the parts are listed in", "[Nest3D]")
{
    // Twelve unique parts that fit one plate, listed forwards, backwards and shuffled.
    auto arrange_in = [](const std::vector<int> &order) {
        Model source;
        add_random_parts(source, 12, 11u);
        Model model;
        for (int i : order)
            model.add_object(*source.objects[size_t(i)]);
        const ArrangePolygons items = nest(model, rect_bed(220, 220), quiet_params(), nest_settings());
        CHECK(unplaced(items) == 0);
        return placements_by_name(items);
    };
    std::vector<int> order(12);
    for (int i = 0; i < 12; ++i)
        order[size_t(i)] = i;
    const auto forwards = arrange_in(order);
    std::reverse(order.begin(), order.end());
    CHECK(arrange_in(order) == forwards);
    std::shuffle(order.begin(), order.end(), std::mt19937(5u));
    CHECK(arrange_in(order) == forwards);
}

TEST_CASE("Repeated runs and a single-threaded run give the same arrangement", "[Nest3D]")
{
    auto run = [](int threads) {
        std::unique_ptr<tbb::global_control> cap;
        if (threads > 0)
            cap = std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism, size_t(threads));
        Model model;
        add_random_parts(model, 25, 3u);
        add_object(model, polyomino(TTetromino), 12, "copies");
        const ArrangePolygons items = nest(model, rect_bed(150, 150), quiet_params(), nest_settings());
        std::vector<std::tuple<int, long, long, long>> out;
        for (const ArrangePolygon &ap : items)
            out.emplace_back(ap.bed_idx, std::lround(unscaled(ap.translation.x()) * 1e3), std::lround(unscaled(ap.translation.y()) * 1e3),
                             std::lround(ap.rotation * 1e6));
        return out;
    };
    const auto first = run(0);
    CHECK(run(0) == first);
    CHECK(run(0) == first);
    CHECK(run(1) == first);
}

TEST_CASE("Cancelling stops the arrange and leaves no invalid placement", "[Nest3D]")
{
    // The stop condition turns true after a few polls; the call returns, the parts not reached stay
    // unplaced, and whatever it placed is inside the bed and spaced. A fallback to the outline
    // arranger must not replace a cancelled result: run under the same stop condition, the outline
    // arranger returns every part "placed" on top of each other.
    Model model;
    add_random_parts(model, 60, 9u);
    std::atomic<int> polls{0};
    ArrangeParams    params = quiet_params();
    params.stopcondition    = [&polls] { return ++polls > 3; };
    const Points          bed   = rect_bed(120, 120);
    const ArrangePolygons items = nest(model, bed, params, nest_settings());
    INFO("stop condition polled " << polls.load() << " times, " << unplaced(items) << " of " << items.size() << " unplaced");
    CHECK(polls.load() > 3);
    CHECK(unplaced(items) > 0);
    CHECK(off_bed(items, bed) == 0);
    CHECK(collisions(items) == 0);
}

// ---------------------------------------------------------------------------
// Brim and skirt in the footprint, and the messages for a part that does not fit
// ---------------------------------------------------------------------------

namespace {

// `base_config()` on a square bed of `side` mm with a brim of the given type and width, and the skirt
// off unless `skirt_loops` says otherwise.
DynamicPrintConfig brim_config(double side, BrimType type, double width, int skirt_loops = 0)
{
    DynamicPrintConfig config = base_config();
    config.set_key_value("printable_area", new ConfigOptionPoints({Vec2d(0, 0), Vec2d(side, 0), Vec2d(side, side), Vec2d(0, side)}));
    config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(type));
    config.set_key_value("brim_width", new ConfigOptionFloat(width));
    config.set_key_value("skirt_loops", new ConfigOptionInt(skirt_loops));
    return config;
}

// Orca's brim for the first instance of the model's first object, as the footprint uses it.
nest3d::ObjectBrim brim_of(Model &model, const DynamicPrintConfig &config)
{
    return nest3d::object_brim(*model.objects.front()->instances.front(), Transform3d::Identity(), config, {});
}

BoundingBoxf box_of(const Printed &part)
{
    return BoundingBoxf(Vec2d(part.box.min.x(), part.box.min.y()), Vec2d(part.box.max.x(), part.box.max.y()));
}

// Plan-view gap between two parts' bounding boxes (they are axis-parallel cubes).
double gap_between(const Printed &a, const Printed &b)
{
    const BoundingBoxf p = box_of(a), q = box_of(b);
    const double       dx = std::max({q.min.x() - p.max.x(), p.min.x() - q.max.x(), 0.});
    const double       dy = std::max({q.min.y() - p.max.y(), p.min.y() - q.max.y(), 0.});
    return std::hypot(dx, dy);
}

} // namespace

TEST_CASE("The brim shape is Orca's own for each brim type and per-object override", "[Nest3D][Brim]")
{
    Model model;
    add_object(model, cube(30, 30, 10));
    const BoundingBoxf3 mesh = bounding_box(printed_mesh(*model.objects.front()->instances.front()));

    SECTION("outer brim reaches its width, rounded down to whole beads, on every side")
    {
        const nest3d::ObjectBrim brim = brim_of(model, brim_config(200, btOuterOnly, 5.));
        REQUIRE_FALSE(brim.areas.empty());
        CHECK(brim.reach > 4.5);
        CHECK(brim.reach <= 5. + Tolerance);
        // The brim is in the frame of the printed mesh: it surrounds it evenly.
        const BoundingBox box = get_extents(brim.areas);
        CHECK(unscaled(box.min.x()) == Catch::Approx(mesh.min.x() - brim.reach).margin(0.05));
        CHECK(unscaled(box.max.y()) == Catch::Approx(mesh.max.y() + brim.reach).margin(0.05));
    }
    SECTION("no brim leaves no shape")
    {
        CHECK(brim_of(model, brim_config(200, btNoBrim, 5.)).areas.empty());
    }
    SECTION("ears cover less than the full ring")
    {
        const nest3d::ObjectBrim outer = brim_of(model, brim_config(200, btOuterOnly, 5.));
        const nest3d::ObjectBrim ears  = brim_of(model, brim_config(200, btEar, 5.));
        REQUIRE_FALSE(ears.areas.empty());
        CHECK(area(ears.areas) < 0.5 * area(outer.areas));
        CHECK(ears.reach > 0.);
    }
    SECTION("the auto brim picks a width of its own")
    {
        // A tall thin part has little to hold it down; a low cube needs no brim at all.
        Model tall;
        add_object(tall, cube(10, 10, 90));
        const nest3d::ObjectBrim brim = brim_of(tall, brim_config(200, btAutoBrim, 0.));
        REQUIRE_FALSE(brim.areas.empty());
        CHECK(brim.reach > 0.);
    }
    SECTION("an object's own brim width overrides the print's")
    {
        model.objects.front()->config.set_key_value("brim_width", new ConfigOptionFloat(2.));
        const nest3d::ObjectBrim brim = brim_of(model, brim_config(200, btOuterOnly, 8.));
        CHECK(brim.reach > 1.5);
        CHECK(brim.reach <= 2. + Tolerance);
    }
}

TEST_CASE("Neighbouring brims stay apart and keep to the plate, each part with its own brim", "[Nest3D][Brim]")
{
    // Two 30 mm cubes with a 5 mm brim: side by side they need the brims clear of the edge and of each other.
    // The cube with the wider brim of its own (10 mm) keeps that much from the edge; the plain one gets close.
    Model model;
    add_object(model, cube(30, 30, 10), 1, "wide");
    add_object(model, cube(30, 30, 10), 1, "plain");
    model.objects.front()->config.set_key_value("brim_width", new ConfigOptionFloat(10.));
    const DynamicPrintConfig config = brim_config(110, btOuterOnly, 3.);
    ArrangeParams            params = quiet_params();
    params.bed_shrink_x = params.bed_shrink_y = 0.f;
    params.allow_rotations = false;
    const Points          bed   = rect_bed(110, 110);
    const ArrangePolygons items = nest(model, bed, params, nest_settings(), {}, PlatesCap, config);
    const double wide_reach  = brim_of(model, config).reach;
    const double plain_reach = nest3d::object_brim(*model.objects.back()->instances.front(), Transform3d::Identity(), config, {}).reach;
    CHECK(unplaced(items) == 0);
    CHECK(plates_used(items) == 1);
    CHECK(collisions(items) == 0);
    const std::vector<Printed> parts = printed(items);
    REQUIRE(parts.size() == 2);
    CHECK(edge_distance(box_of(parts[0]), 110, 110) >= wide_reach - Tolerance);
    CHECK(edge_distance(box_of(parts[1]), 110, 110) >= plain_reach - Tolerance);
    // Brim to brim is at least the two spacing allowances apart, so the geometry is both brims plus those.
    CHECK(gap_between(parts[0], parts[1]) >= wide_reach + plain_reach + Spacing - Tolerance);
}

TEST_CASE("A brim takes room: parts that fit side by side without it need a second plate with it", "[Nest3D][Brim]")
{
    Model model;
    add_object(model, cube(40, 40, 10), 4);
    ArrangeParams params = quiet_params();
    params.bed_shrink_x = params.bed_shrink_y = 0.f;
    params.allow_rotations = false;
    const Points bed = rect_bed(100, 100);
    const ArrangePolygons plain = nest(model, bed, params, nest_settings(), {}, PlatesCap, brim_config(100, btNoBrim, 0.));
    CHECK(plates_used(plain) == 1); // 2 x 2 cubes of 40 mm with 1 mm between them
    const ArrangePolygons brimmed = nest(model, bed, params, nest_settings(), {}, PlatesCap, brim_config(100, btOuterOnly, 6.));
    CHECK(unplaced(brimmed) == 0);
    CHECK(plates_used(brimmed) > 1); // 40 + 2 x 6 mm of brim each does not fit twice across 100 mm
    CHECK(collisions(brimmed) == 0);
}

TEST_CASE("A skirt per object is reserved around each object, a combined skirt only around the plate", "[Nest3D][Skirt]")
{
    const double side = 200.;
    DynamicPrintConfig config = brim_config(side, btNoBrim, 0., 2);
    config.set_key_value("skirt_distance", new ConfigOptionFloat(4.));
    config.set_key_value("skirt_height", new ConfigOptionInt(1));
    const double extent = get_real_skirt_dist(config);
    REQUIRE(extent > 4.);
    Model model;
    add_object(model, cube(30, 30, 10), 2);
    ArrangeParams params = quiet_params();
    params.bed_shrink_x = params.bed_shrink_y = 0.f;
    params.allow_rotations = false;
    arrangement::update_arrange_params(params, &config, {});
    const Points bed = get_shrink_bedpts(&config, params);

    SECTION("combined")
    {
        config.set_key_value("skirt_type", new ConfigOptionEnum<SkirtType>(stCombined));
        const ArrangePolygons items = nest(model, bed, params, nest_settings(), {}, PlatesCap, config);
        const std::vector<Printed> parts = printed(items);
        CHECK(unplaced(items) == 0);
        CHECK(gap_between(parts[0], parts[1]) < 2.);                          // one skirt around both: parts stay close
        for (const Printed &part : parts)
            CHECK(edge_distance(box_of(part), side, side) >= extent - Tolerance); // and it stays on the plate
    }
    SECTION("per object")
    {
        config.set_key_value("skirt_type", new ConfigOptionEnum<SkirtType>(stPerObject));
        const ArrangePolygons items = nest(model, bed, params, nest_settings(), {}, PlatesCap, config);
        const std::vector<Printed> parts = printed(items);
        CHECK(unplaced(items) == 0);
        // Each object's skirt ring, from its distance out to its last loop, clears the other object's.
        CHECK(gap_between(parts[0], parts[1]) >= 2. * extent - Tolerance);
        for (const Printed &part : parts)
            CHECK(edge_distance(box_of(part), side, side) >= extent - Tolerance);
    }
}

TEST_CASE("A combined skirt that would cross the excluded area moves the pile inward", "[Nest3D][Skirt]")
{
    // A strip 18 mm wide along the left edge of a 60 mm bed. The cube of 30 mm fits beside it with 1 mm
    // to spare, but the skirt (distance 3 mm and a loop) printed around it would run over the strip.
    DynamicPrintConfig config = brim_config(60, btNoBrim, 0., 1);
    config.set_key_value("skirt_distance", new ConfigOptionFloat(3.));
    config.set_key_value("skirt_height", new ConfigOptionInt(1));
    config.set_key_value("bed_exclude_area", new ConfigOptionPoints({Vec2d(0, 0), Vec2d(18, 0), Vec2d(18, 60), Vec2d(0, 60)}));
    const double extent = get_real_skirt_dist(config);
    ArrangeParams params = quiet_params();
    params.bed_shrink_x = params.bed_shrink_y = 0.f;
    params.allow_rotations = false;
    arrangement::update_arrange_params(params, &config, {});
    ArrangePolygon region;
    region.poly.contour = rect(0, 0, 18, 60);
    region.inflation    = scaled(0.5 * Spacing);
    params.excluded_regions.push_back(region);
    Model model;
    add_object(model, cube(30, 30, 10));
    const ArrangePolygons items = nest(model, get_shrink_bedpts(&config, params), params, nest_settings(), {}, PlatesCap, config);
    REQUIRE(unplaced(items) == 0);
    const Printed part = printed(items).front();
    CHECK(box_of(part).min.x() >= 18. + extent - Tolerance);
    CHECK(box_of(part).max.x() <= 60. - extent + Tolerance);
}

TEST_CASE("A part that fits no plate is named, measured with its brim, and told which limit stops it", "[Nest3D][Messages]")
{
    const std::string start = "G28\nG90\nM83\nG1 X22 Y0 Z0.3 F12000\n"
                              "{local fa = 0.7853981633974483 * filament_diameter[0] * filament_diameter[0]}\n"
                              "G1 X240 Y0 E{21.9 / fa}\nG1 E-0.5\nG1 Z1\n";
    std::vector<std::string> asked; // the msgids a catalog would be asked for
    const auto translate = [&asked](const char *msgid) { asked.push_back(msgid); return std::string(msgid); };

    SECTION("larger than the bed")
    {
        Model model;
        add_object(model, cube(98, 98, 10), 1, "Gear");
        const DynamicPrintConfig config = brim_config(100, btOuterOnly, 5.);
        ArrangeParams            params = quiet_params();
        params.bed_shrink_x = params.bed_shrink_y = 0.f;
        std::vector<nest3d::Issue> issues;
        ArrangePolygons            items = items_of(model, 5.);
        nest3d::arrange_polygons(items, {}, rect_bed(100, 100), params, nest_settings(), config, PlatesCap, &issues);
        REQUIRE(unplaced(items) == 1);
        REQUIRE(issues.size() == 1);
        const nest3d::Issue &issue = issues.front();
        CHECK(issue.kind == nest3d::Issue::Kind::Unplaced);
        CHECK(issue.limit == nest3d::Issue::Limit::Bed);
        CHECK(issue.part == "Gear");
        const double reach = brim_of(model, config).reach;
        const std::string text = nest3d::issue_text(issue);
        INFO(text);
        CHECK_THAT(text, Catch::Matchers::StartsWith("Gear measures " + nest3d::length_text(98 + 2 * reach) + " × " +
                                                     nest3d::length_text(98 + 2 * reach) + " mm with its " + nest3d::length_text(reach) + " mm brim."));
        CHECK_THAT(text, Catch::Matchers::ContainsSubstring("The room left on the bed is"));
        CHECK_THAT(text, Catch::Matchers::ContainsSubstring("make the part smaller"));
        // Each sentence is its own msgid with placeholders, so a translator sees whole sentences.
        nest3d::issue_text(issue, translate);
        CHECK(std::find(asked.begin(), asked.end(), "%1% measures %2% × %3% mm with its %4% mm brim.") != asked.end());
    }
    SECTION("beside the excluded area")
    {
        Model model;
        add_object(model, cube(80, 20, 10), 1, "Rail");
        DynamicPrintConfig config = brim_config(100, btNoBrim, 0.);
        config.set_key_value("bed_exclude_area", new ConfigOptionPoints({Vec2d(0, 0), Vec2d(30, 0), Vec2d(30, 100), Vec2d(0, 100)}));
        ArrangeParams params = quiet_params();
        params.bed_shrink_x = params.bed_shrink_y = 0.f;
        params.allow_rotations = false;
        ArrangePolygon region;
        region.poly.contour = rect(0, 0, 30, 100);
        region.inflation    = scaled(0.5 * Spacing);
        params.excluded_regions.push_back(region);
        std::vector<nest3d::Issue> issues;
        ArrangePolygons            items = items_of(model);
        nest3d::arrange_polygons(items, {}, rect_bed(100, 100), params, nest_settings(), config, PlatesCap, &issues);
        REQUIRE(unplaced(items) == 1);
        REQUIRE(issues.size() == 1);
        CHECK(issues.front().limit == nest3d::Issue::Limit::ExcludedArea);
        const std::string text = nest3d::issue_text(issues.front());
        INFO(text);
        CHECK_THAT(text, Catch::Matchers::StartsWith("Rail measures 80 × 20 mm."));
        CHECK_THAT(text, Catch::Matchers::ContainsSubstring("The room left beside the excluded area is"));
    }
    SECTION("beside the purge strip")
    {
        Model model;
        add_object(model, cube(100, 254, 5), 1, "Rail");
        const Printer p = printer({{0, 0}, {18, 0}, {18, 28}, {0, 28}}, start);
        std::vector<nest3d::Issue> issues;
        ArrangeParams              params = p.params;
        params.bed_shrink_x = params.bed_shrink_y = 0.f;
        params.allow_rotations = false;
        ArrangePolygons items = items_of(model);
        nest3d::arrange_polygons(items, {}, rect_bed(256, 256), params, nest_settings(), p.config, PlatesCap, &issues);
        REQUIRE(unplaced(items) == 1);
        REQUIRE(issues.size() == 1);
        CHECK(issues.front().limit == nest3d::Issue::Limit::PurgeStrip);
        CHECK_THAT(nest3d::issue_text(issues.front()), Catch::Matchers::ContainsSubstring("start G-code purge line"));
    }
}

TEST_CASE("A part that moves to a later plate says the others left it no room", "[Nest3D][Messages]")
{
    Model model;
    add_object(model, cube(60, 60, 10), 2, "Block");
    ArrangeParams params = quiet_params();
    params.bed_shrink_x = params.bed_shrink_y = 0.f;
    std::vector<nest3d::Issue> issues;
    ArrangePolygons            items = items_of(model);
    nest3d::arrange_polygons(items, {}, rect_bed(70, 70), params, nest_settings(), base_config(), PlatesCap, &issues);
    REQUIRE(plates_used(items) == 2);
    REQUIRE(issues.size() == 1);
    CHECK(issues.front().kind == nest3d::Issue::Kind::Spilled);
    CHECK(issues.front().part == "Block");
    CHECK_THAT(nest3d::issue_text(issues.front()), Catch::Matchers::ContainsSubstring("next plate"));
}

TEST_CASE("Parts that all fit report nothing", "[Nest3D][Messages]")
{
    Model model;
    add_object(model, cube(20, 20, 10), 3);
    std::vector<nest3d::Issue> issues;
    ArrangePolygons            items = items_of(model);
    nest3d::arrange_polygons(items, {}, rect_bed(200, 200), quiet_params(), nest_settings(), base_config(), PlatesCap, &issues);
    CHECK(unplaced(items) == 0);
    CHECK(issues.empty());
}
