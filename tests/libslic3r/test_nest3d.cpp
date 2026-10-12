#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Nest3D/Nest3D.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <random>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

using namespace Slic3r::nest3d;
using Catch::Matchers::WithinAbs;

namespace {

using Mesh = std::vector<Triangle>;

constexpr double Pi = 3.14159265358979323846;

// Closed axis-aligned box with outward-facing triangles.
Mesh box(float x0, float y0, float z0, float x1, float y1, float z1)
{
    const float p[8][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                           {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
    const int   f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                            {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    Mesh m;
    for (const auto &t : f) {
        Triangle tri;
        for (int k = 0; k < 3; ++k)
            for (int c = 0; c < 3; ++c)
                tri.v[k][c] = p[t[k]][c];
        m.push_back(tri);
    }
    return m;
}

Mesh join(std::initializer_list<Mesh> meshes)
{
    Mesh out;
    for (const Mesh &m : meshes)
        out.insert(out.end(), m.begin(), m.end());
    return out;
}

Part make_part(int id, const Mesh &mesh, int group = 0)
{
    Part p;
    p.id           = id;
    p.compat_group = group;
    p.candidates.push_back({rasterize(mesh, 1., 0., false)});
    return p;
}

Params bed(double w, double h)
{
    Params p;
    p.bed_w = w, p.bed_h = h;
    return p;
}

// Polyomino of `unit` mm squares, 5 mm tall.
Mesh polyomino(std::initializer_list<std::pair<int, int>> squares, float unit)
{
    Mesh m;
    for (auto [x, y] : squares)
        m = join({m, box(x * unit, y * unit, 0, (x + 1) * unit, (y + 1) * unit, 5)});
    return m;
}

// Closed prism over a convex counter-clockwise polygon, from z0 to z1.
Mesh prism(const std::vector<std::pair<float, float>> &poly, float z0, float z1)
{
    auto tri = [](std::pair<float, float> a, float za, std::pair<float, float> b, float zb, std::pair<float, float> c, float zc) {
        return Triangle{{{a.first, a.second, za}, {b.first, b.second, zb}, {c.first, c.second, zc}}};
    };
    Mesh m;
    for (size_t i = 1; i + 1 < poly.size(); ++i) {
        m.push_back(tri(poly[0], z0, poly[i + 1], z0, poly[i], z0));
        m.push_back(tri(poly[0], z1, poly[i], z1, poly[i + 1], z1));
    }
    for (size_t i = 0; i < poly.size(); ++i) {
        const auto &a = poly[i], &b = poly[(i + 1) % poly.size()];
        m.push_back(tri(a, z0, b, z0, b, z1));
        m.push_back(tri(a, z0, b, z1, a, z1));
    }
    return m;
}

// Round ring about the origin, 10 mm tall, built from 48 quads.
Mesh ring(float inner, float outer)
{
    Mesh m;
    for (int k = 0; k < 48; ++k) {
        const double a0 = 2. * Pi * k / 48, a1 = 2. * Pi * (k + 1) / 48;
        m = join({m, prism({{float(inner * std::cos(a0)), float(inner * std::sin(a0))}, {float(outer * std::cos(a0)), float(outer * std::sin(a0))},
                            {float(outer * std::cos(a1)), float(outer * std::sin(a1))}, {float(inner * std::cos(a1)), float(inner * std::sin(a1))}},
                           0, 10)});
    }
    return m;
}

// `steps` Z angles evenly spaced over a full turn, as the arrange dialog offers them.
std::vector<double> turns(int steps)
{
    std::vector<double> offsets;
    for (int k = 0; k < steps; ++k)
        offsets.push_back(2. * Pi * k / steps);
    return candidate_angles(0., offsets);
}

// A copy of `mesh` on 0.5 mm cells, offered at `rotations` quarter turns, grown by 0.5 mm so that
// copies keep 1 mm apart.
Part copy_of(int id, const Mesh &mesh, int rotations, int shape_key)
{
    Part p;
    p.id        = id;
    p.shape_key = shape_key;
    Mesh m      = mesh;
    for (int r = 0; r < rotations; ++r) {
        p.candidates.push_back({rasterize(m, 0.5, 0.5, false), r * 0.5 * Pi});
        for (Triangle &t : m)
            for (auto &v : t.v)
                std::swap(v[0], v[1]), v[0] = -v[0];
    }
    return p;
}

// A copy of `mesh` on `cell` mm cells, offered at each of `angles` (radians about the origin),
// grown by 0.5 mm so that copies keep 1 mm apart.
Part copy_at(int id, const Mesh &mesh, const std::vector<double> &angles, int shape_key, double cell = 0.5)
{
    Part p;
    p.id        = id;
    p.shape_key = shape_key;
    for (double a : angles) {
        Mesh         m = mesh;
        const double c = std::cos(a), s = std::sin(a);
        for (Triangle &t : m)
            for (auto &v : t.v) {
                const float x = v[0], y = v[1];
                v[0] = float(c * x - s * y), v[1] = float(s * x + c * y);
            }
        p.candidates.push_back({rasterize(m, cell, 0.5, false), a});
    }
    return p;
}

// `n` copies of `part`, with ids from part.id on.
std::vector<Part> copies(const Part &part, int n)
{
    std::vector<Part> out(size_t(n), part);
    for (int i = 0; i < n; ++i)
        out[size_t(i)].id = part.id + i;
    return out;
}

// Bounding-box area of the parts placed on plate 0, mm^2.
double plate_bbox_area(const std::vector<Part> &parts, const std::vector<Placement> &out)
{
    double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30;
    for (size_t i = 0; i < parts.size(); ++i)
        if (out[i].plate == 0)
            for (const Candidate &c : parts[i].candidates)
                if (c.rotation_z == out[i].rotation_z && c.orientation == out[i].orientation) {
                    x0 = std::min(x0, out[i].x + c.map.origin_x), x1 = std::max(x1, out[i].x + c.map.origin_x + c.map.width * c.map.cell_mm);
                    y0 = std::min(y0, out[i].y + c.map.origin_y), y1 = std::max(y1, out[i].y + c.map.origin_y + c.map.height * c.map.cell_mm);
                    break;
                }
    return x1 > x0 ? (x1 - x0) * (y1 - y0) : 0.;
}

// Bed cells covered by a placed part, keyed y * 10000 + x, with the candidate chosen by rotation.
std::vector<std::pair<int, Span>> placed_cells(const Part &part, const Placement &pl, double cell)
{
    std::vector<std::pair<int, Span>> out;
    for (const Candidate &c : part.candidates) {
        if (c.rotation_z != pl.rotation_z || c.orientation != pl.orientation)
            continue;
        int ox = int(std::lround((pl.x + c.map.origin_x) / cell)), oy = int(std::lround((pl.y + c.map.origin_y) / cell));
        for (int y = 0; y < c.map.height; ++y)
            for (int x = 0; x < c.map.width; ++x)
                if (!c.map.at(x, y).empty())
                    out.emplace_back((oy + y) * 10000 + ox + x, c.map.at(x, y));
        break;
    }
    return out;
}

// Number of cells claimed by more than one placed part on the same plate.
int overlapping_cells(const std::vector<Part> &parts, const std::vector<Placement> &out, double cell)
{
    std::set<std::pair<int, int>> seen;
    int                           overlaps = 0;
    for (size_t i = 0; i < parts.size(); ++i)
        if (out[i].plate >= 0)
            for (const auto &c : placed_cells(parts[i], out[i], cell))
                overlaps += !seen.insert({out[i].plate, c.first}).second;
    return overlaps;
}

} // namespace

TEST_CASE("A cube rasterizes to full-height columns over its footprint", "[Nest3D]")
{
    const float     side = 20.f;
    const HeightMap m    = rasterize(box(5, 5, 0, 5 + side, 5 + side, side), 1., 0., false);
    REQUIRE(m.width == int(side));
    REQUIRE(m.height == int(side));
    CHECK_THAT(m.origin_x, WithinAbs(5., 1e-9));
    for (const Span &s : m.cells) {
        CHECK_THAT(s.lo, WithinAbs(0., 1e-4));
        CHECK_THAT(s.hi, WithinAbs(side, 1e-4));
    }
}

TEST_CASE("Growth claims the cells within the clearance of the geometry, not of whole cells", "[Nest3D]")
{
    // A 10 mm cube grown by 3.5 mm on 1 mm cells. The map starts 3.5 mm before the cube, so its
    // sides lie half way through cells 3 and 13, and a cell is claimed when it comes within 3.5 mm
    // of the cube itself. That leaves out just the four corner cells, 2.5 mm off both sides
    // (3.54 mm away); growing from the whole cells the sides cross would claim the corners too and
    // reach a cell beyond the map.
    const HeightMap m = rasterize(box(0, 0, 0, 10, 10, 5), 1., 3.5, false);
    REQUIRE(m.width == 17);
    REQUIRE(m.height == 17);
    CHECK_THAT(m.origin_x, WithinAbs(-3.5, 1e-9));
    int empty = 0;
    for (int y = 0; y < m.height; ++y)
        for (int x = 0; x < m.width; ++x) {
            const double gx = std::max({0., 3.5 - (x + 1.), x - 13.5}), gy = std::max({0., 3.5 - (y + 1.), y - 13.5});
            CHECK(m.at(x, y).empty() == (std::hypot(gx, gy) >= 3.5));
            empty += m.at(x, y).empty();
        }
    CHECK(empty == 4);
}

TEST_CASE("Parts grown on coarse cells keep their spacing without rounding it up", "[Nest3D]")
{
    // Two 10 mm cubes grown by 0.5 mm on 1 mm cells claim 11 mm each, so they fit side by side on
    // a 22 mm bed at exactly 1 mm apart. Growing whole cells instead would claim 12 mm each.
    std::vector<Part> parts(2);
    for (int i = 0; i < 2; ++i) {
        parts[i].id = i;
        parts[i].candidates.push_back({rasterize(box(0, 0, 0, 10, 10, 5), 1., 0.5, false)});
    }
    Params p     = bed(22, 11);
    p.max_plates = 1;
    const auto out = arrange(parts, {}, p);
    REQUIRE(out[0].plate == 0);
    REQUIRE(out[1].plate == 0);
    CHECK_THAT(std::abs(out[0].x - out[1].x), WithinAbs(11., 1e-9));
}

TEST_CASE("A mesh of vertical walls only still marks the cells under the walls", "[Nest3D]")
{
    // A zero-thickness panel 10 mm long and 5 mm tall in the plane y = 3.5. Its normal
    // points to -y or +y; either way the wall lies on the map's edge and must stay in.
    Mesh panel(2);
    const float q[4][3] = {{0, 3.5f, 0}, {10, 3.5f, 0}, {10, 3.5f, 5}, {0, 3.5f, 5}};
    const bool  flip    = GENERATE(false, true);
    const int   idx[2][3] = {{0, flip ? 2 : 1, flip ? 1 : 2}, {0, flip ? 3 : 2, flip ? 2 : 3}};
    for (int t = 0; t < 2; ++t)
        for (int k = 0; k < 3; ++k)
            for (int c = 0; c < 3; ++c)
                panel[t].v[k][c] = q[idx[t][k]][c];

    const HeightMap m = rasterize(panel, 1., 0., false);
    REQUIRE(m.width == 10);
    REQUIRE(m.height == 1);
    for (int x = 0; x < m.width; ++x) {
        CHECK_THAT(m.at(x, 0).lo, WithinAbs(0., 1e-4));
        CHECK_THAT(m.at(x, 0).hi, WithinAbs(5., 1e-4));
    }
}

TEST_CASE("Dilation grows a column by a disc and adds the bottom-only ring", "[Nest3D]")
{
    HeightMap m(1, 1, 1.);
    m.at(0, 0).add(0.f, 10.f);
    const HeightMap d = dilate(m, 1., 2., 0.3);
    REQUIRE(d.width == 5);
    CHECK_THAT(d.origin_x, WithinAbs(-2., 1e-9));
    int full = 0, ring = 0;
    for (const Span &s : d.cells)
        full += !s.empty() && s.hi > 5.f, ring += !s.empty() && s.hi < 1.f;
    CHECK(full == 9);  // every cell whose square comes within 1 cell of the source square
    CHECK(ring == 16); // the whole 5 x 5 block within 2 cells, minus the 9 above
}

TEST_CASE("Dilated parts keep the requested spacing for any size and inflation", "[Nest3D]")
{
    // Part sizes off the grid and inflations below one cell are the cases a
    // centre-distance disc gets wrong.
    const auto [w, infl] = GENERATE(table<float, double>({{10.5f, 0.5}, {10.95f, 0.9}, {10.2f, 0.5}, {10.5f, 1.5}, {10.5f, 0.25}}));
    std::vector<Part> parts(4);
    for (int i = 0; i < 4; ++i) {
        parts[i].id = i;
        parts[i].candidates.push_back({dilate(rasterize(box(0, 0, 0, w, w, 5), 1., 0., false), infl)});
    }
    Params p  = bed(40, 40);
    p.align_x = p.align_y = -1.;
    const auto out = arrange(parts, {}, p);
    for (int i = 0; i < 4; ++i) {
        REQUIRE(out[i].plate == 0);
        for (int j = 0; j < i; ++j) {
            const double gx = std::max(0., std::abs(out[i].x - out[j].x) - w);
            const double gy = std::max(0., std::abs(out[i].y - out[j].y) - w);
            CHECK(std::hypot(gx, gy) >= 2. * infl - 1e-6);
        }
    }
}

TEST_CASE("Two L-shaped parts interlock on a bed too small for them side by side", "[Nest3D]")
{
    // Arms 30 mm long and 10 mm wide; the second L is the first turned by 180 degrees.
    std::vector<Part> parts{make_part(1, join({box(0, 0, 0, 30, 10, 5), box(0, 10, 0, 10, 30, 5)})),
                            make_part(2, join({box(0, 20, 0, 30, 30, 5), box(20, 0, 0, 30, 20, 5)}))};
    Params p     = bed(41, 41);
    p.max_plates = 1;
    const auto out = arrange(parts, {}, p);
    CHECK(out[0].plate == 0);
    CHECK(out[1].plate == 0);
    CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
}

TEST_CASE("Two copies of an S interlock on a bed that holds only the interlocked pair", "[Nest3D]")
{
    // S tetromino of 10 mm squares, rotation 0 only, copies 1 mm apart. Interlocked side by side,
    // the right copy sits 1 mm lower than the left one (the gap between the overlapping arms), so
    // the pair spans 30 + 1 + 20 = 51 by 20 + 1 = 21 mm, plus 0.5 mm of dilation on each side.
    // A part that grows from a wall or from the centre pins the first copy where the second
    // cannot drop below it; placing the pair as a unit finds the fit.
    const Mesh        s = polyomino({{0, 0}, {1, 0}, {1, 1}, {2, 1}}, 10.f);
    std::vector<Part> parts{copy_of(1, s, 1, 0), copy_of(2, s, 1, 0)};
    Params            p = bed(52, 22);
    p.cell_mm    = 0.5;
    p.max_plates = 1;
    const auto out = arrange(parts, {}, p);
    CHECK(out[0].plate == 0);
    CHECK(out[1].plate == 0);
    CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
}

TEST_CASE("Copies of a tetromino fill most of the bed's area bound", "[Nest3D]")
{
    // T, S and L tetrominoes of 10 mm squares, at four quarter turns, 1 mm apart. Each copy claims
    // at least its area plus half the spacing along its perimeter plus a corner allowance,
    // 400 + 100 * 1 / 2 + 1 = 451 mm^2, so a 120 mm bed holds at most 14400 / 451 = 31 copies.
    // Packed by bounding box (31 x 21 mm each) it holds 3 * 5 = 15.
    const int  shape = GENERATE(0, 1, 2);
    const Mesh mesh  = shape == 0 ? polyomino({{0, 0}, {1, 0}, {2, 0}, {1, 1}}, 10.f)
                     : shape == 1 ? polyomino({{0, 0}, {1, 0}, {1, 1}, {2, 1}}, 10.f)
                                  : polyomino({{0, 0}, {1, 0}, {2, 0}, {2, 1}}, 10.f);
    const int  bound = 31;
    const std::vector<Part> parts = copies(copy_of(0, mesh, 4, 7), bound + 4);
    Params p     = bed(120, 120);
    p.cell_mm    = 0.5;
    p.max_plates = 1;
    Stats      stats;
    const auto out    = arrange(parts, {}, p, &stats);
    const int  placed = int(parts.size()) - stats.unplaced;
    INFO("shape " << shape << ": " << placed << " of " << bound);
    CHECK(placed >= 22); // 70% of the bound
    CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
}

TEST_CASE("Copies of a tetromino fill the bed with the default arrange settings", "[Nest3D]")
{
    // As the arrange dialog sets it up by default: balanced effort and 8 rotation steps, and fast
    // effort's 1 mm cells, here with 1 mm spacing. By the bound above, a 160 mm bed holds at most
    // 25600 / 451 = 56 copies. Spacing rounded up to whole cells on 0.6 and 1 mm grids gives 38 to 44.
    const int  shape = GENERATE(0, 1, 2);
    const int  level = GENERATE(0, 1);
    const Mesh mesh  = shape == 0 ? polyomino({{0, 0}, {1, 0}, {2, 0}, {1, 1}}, 10.f)
                     : shape == 1 ? polyomino({{0, 0}, {1, 0}, {1, 1}, {2, 1}}, 10.f)
                                  : polyomino({{0, 0}, {1, 0}, {2, 0}, {2, 1}}, 10.f);
    const Effort      effort = effort_settings(level, 160, 160);
    const std::vector<Part> parts = copies(copy_at(0, mesh, turns(8), 0, effort.cell_mm), 60);
    Params p     = bed(160, 160);
    p.cell_mm    = effort.cell_mm;
    p.orderings  = effort.orderings;
    p.max_plates = 1;
    Stats      stats;
    const auto out    = arrange(parts, {}, p, &stats);
    const int  placed = int(parts.size()) - stats.unplaced;
    INFO("shape " << shape << ", effort " << level << ": " << placed << " of 56");
    CHECK(placed >= (level == 1 ? 45 : 43)); // 80% and 77% of the bound
    CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
}

TEST_CASE("Sawtooth strips mesh on 1 mm cells", "[Nest3D]")
{
    // A 40 x 5 mm strip with four 5 mm teeth, offered as is and turned half a turn. Meshed 1 mm
    // apart, a pair spans 45 x 16.4 mm (the 45 degree flanks sit 1.41 mm apart vertically), which
    // with 0.5 mm of growth on each side fits a 47 x 19 mm bed; side by side or stacked unmeshed it
    // does not (81 or 21 mm). Spacing rounded up to whole cells would keep the teeth 2 mm apart and
    // the pair 1 mm too tall.
    std::vector<std::pair<float, float>> base{{0, 0}, {40, 0}, {40, 5}, {0, 5}};
    Mesh strip = prism(base, 0, 5);
    for (int t = 0; t < 4; ++t)
        strip = join({strip, prism({{10.f * t, 5}, {10.f * t + 10, 5}, {10.f * t + 5, 10}}, 0, 5)});
    std::vector<Part> parts{copy_at(0, strip, {0., Pi}, 0, 1.), copy_at(1, strip, {0., Pi}, 0, 1.)};
    Params            p = bed(47, 19);
    p.cell_mm    = 1.;
    p.max_plates = 1;
    const auto out = arrange(parts, {}, p);
    CHECK(out[0].plate == 0);
    CHECK(out[1].plate == 0);
    CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
}

TEST_CASE("Small parts fill the holes of rings before the open bed", "[Nest3D][Regression]")
{
    // Two rings with a 30 mm hole and eight 8 mm pegs on a bed with plenty of room around them.
    // Each hole holds four pegs 1 mm apart. Placed by bounding box and contact alone, the pegs
    // settle in the crevices beside the rings, where they touch more.
    const Mesh        r = ring(15.f, 25.f), peg = box(0, 0, 0, 8, 8, 6);
    std::vector<Part>       parts = copies(copy_at(0, r, turns(4), 0), 2);
    const std::vector<Part> pegs  = copies(copy_at(2, peg, turns(8), 1), 8);
    parts.insert(parts.end(), pegs.begin(), pegs.end());
    Params p     = bed(150, 150);
    p.cell_mm    = 0.5;
    p.max_plates = 1;
    const auto out = arrange(parts, {}, p);
    CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
    for (int i = 2; i < 10; ++i) {
        REQUIRE(out[i].plate == 0);
        // Peg centre in world coordinates, from its reference point (corner) and rotation.
        const double c = std::cos(out[i].rotation_z), s = std::sin(out[i].rotation_z);
        const double px = out[i].x + 4. * c - 4. * s, py = out[i].y + 4. * s + 4. * c;
        const double d = std::min(std::hypot(px - out[0].x, py - out[0].y), std::hypot(px - out[1].x, py - out[1].y));
        INFO("peg " << i << " centre " << d << " mm from the nearest ring centre");
        CHECK(d < 15.);
    }
}

TEST_CASE("With stacking, a ring's hole under a table top is still a pocket for small parts", "[Nest3D][Regression]")
{
    // Two rings with a 30 mm hole, a 60 mm table whose 20 mm high top covers one of them, and
    // eight 8 mm pegs, 6 mm tall, that fit under the top with the 2 mm clearance. Each hole holds
    // four pegs. Read as walls, the table top would close the covered hole.
    const Mesh table = join({box(0, 0, 20, 60, 60, 22), box(0, 0, 0, 5, 5, 20), box(55, 0, 0, 60, 5, 20), box(0, 55, 0, 5, 60, 20),
                             box(55, 55, 0, 60, 60, 20)});
    std::vector<Part> parts = copies(copy_at(0, ring(15.f, 25.f), turns(4), 0), 2);
    parts.push_back(copy_at(2, table, turns(4), 1));
    const std::vector<Part> pegs = copies(copy_at(3, box(0, 0, 0, 8, 8, 6), turns(8), 2), 8);
    parts.insert(parts.end(), pegs.begin(), pegs.end());
    for (Part &part : parts)
        part.priority = part.id < 2 ? 2. : part.id == 2 ? 1. : 0.; // rings, then the table, then the pegs
    Params p         = bed(200, 200);
    p.cell_mm        = 0.5;
    p.max_plates     = 1;
    p.allow_stacking = true;
    p.vertical_gap   = 2.;
    const auto out = arrange(parts, {}, p);
    REQUIRE(out[2].plate == 0);
    for (int i = 3; i < 11; ++i) {
        REQUIRE(out[i].plate == 0);
        const double c = std::cos(out[i].rotation_z), s = std::sin(out[i].rotation_z);
        const double px = out[i].x + 4. * c - 4. * s, py = out[i].y + 4. * s + 4. * c;
        const double d = std::min(std::hypot(px - out[0].x, py - out[0].y), std::hypot(px - out[1].x, py - out[1].y));
        INFO("peg " << i << " centre " << d << " mm from the nearest ring centre");
        CHECK(d < 15.);
    }
}

TEST_CASE("Small parts fill a large enclosed region without leaving it", "[Nest3D][Regression]")
{
    // A 100 mm square frame with 8 mm walls and 60 small unique rectangles. All of them fit inside
    // the frame, so the pile is the frame. The pocket search scans this region for every part and
    // stops once no position further from its corner can win, which keeps this case fast.
    const Mesh frame = join({box(0, 0, 0, 100, 8, 5), box(0, 92, 0, 100, 100, 5), box(0, 8, 0, 8, 92, 5), box(92, 8, 0, 100, 92, 5)});
    std::vector<Part> parts{copy_at(0, frame, {0.}, -1)};
    for (int i = 1; i <= 60; ++i)
        parts.push_back(copy_at(i, box(0, 0, 0, 4.f + (i % 7) * 0.7f, 3.f + (i % 5) * 0.9f, 5), turns(8), -1));
    Params p     = bed(150, 150);
    p.cell_mm    = 0.5;
    p.max_plates = 1;
    const auto out = arrange(parts, {}, p);
    for (const Placement &pl : out)
        REQUIRE(pl.plate == 0);
    const HeightMap &f = parts[0].candidates.front().map;
    CHECK_THAT(plate_bbox_area(parts, out), WithinAbs(f.width * f.height * p.cell_mm * p.cell_mm, 1e-6));
    CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
}

TEST_CASE("Copies that overflow a plate fill the first plate as fully as a single-plate arrange", "[Nest3D]")
{
    // 40 T tetrominoes overflow a 120 mm bed, which holds at most 31 (see above). Parts that do
    // not fit go to the next plate, so the first plate can hold as many as when it is the only one.
    const Mesh        t = polyomino({{0, 0}, {1, 0}, {2, 0}, {1, 1}}, 10.f);
    const std::vector<Part> parts = copies(copy_of(0, t, 4, 0), 40);
    Params p     = bed(120, 120);
    p.cell_mm    = 0.5;
    p.max_plates = 1;
    Stats single;
    arrange(parts, {}, p, &single);
    p.max_plates   = 3;
    const auto out = arrange(parts, {}, p);
    const int  on_first = int(std::count_if(out.begin(), out.end(), [](const Placement &pl) { return pl.plate == 0; }));
    CHECK(on_first == int(parts.size()) - single.unplaced);
    CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
}

TEST_CASE("Copies laid on a lattice still avoid a penalised pose", "[Nest3D]")
{
    // Each copy may stand as a disc (no penalty) or as a 9 x 9 square, which tiles without gaps
    // but carries a large penalty. Copies, with or without a shape key, keep the disc.
    HeightMap disc(11, 11, 1.), square(9, 9, 1.);
    for (int y = 0; y < 11; ++y)
        for (int x = 0; x < 11; ++x)
            if (std::hypot(x - 5., y - 5.) <= 5.2)
                disc.at(x, y).add(0.f, 5.f);
    for (Span &c : square.cells)
        c.add(0.f, 5.f);
    const int         key = GENERATE(-1, 0);
    std::vector<Part> parts(30);
    for (int i = 0; i < 30; ++i) {
        parts[i].id         = i;
        parts[i].shape_key  = key;
        parts[i].candidates = {{disc, 0., 0, 0.}, {square, 0., 1, 100.}};
    }
    Params p = bed(80, 80);
    p.max_plates = 1;
    const auto out = arrange(parts, {}, p);
    for (const Placement &pl : out) {
        CHECK(pl.plate == 0);
        CHECK(pl.orientation == 0);
    }
}

TEST_CASE("Copies an arrange left at different quarter turns are still copies when arranged again", "[Nest3D]")
{
    // Every L may turn by quarter turns. After a first arrange the copies sit at different angles;
    // offered from their new angles, they still get one list of candidate angles, in one order,
    // so a second arrange packs them as copies again and places as many.
    const Mesh                l = polyomino({{0, 0}, {1, 0}, {2, 0}, {2, 1}}, 10.f);
    const std::vector<double> quarter{0., 0.5 * Pi, Pi, 1.5 * Pi};
    const std::vector<Part>   parts = copies(copy_at(0, l, candidate_angles(0., quarter), 0), 35);
    Params p     = bed(120, 120);
    p.cell_mm    = 0.5;
    p.max_plates = 1;
    Stats      first;
    const auto out = arrange(parts, {}, p, &first);
    std::set<double> turns;
    for (const Placement &pl : out)
        turns.insert(pl.rotation_z);
    REQUIRE(turns.size() > 1);
    const std::vector<double> angles = candidate_angles(out[0].rotation_z, quarter);
    for (const Placement &pl : out) {
        const std::vector<double> mine = candidate_angles(pl.rotation_z, quarter);
        REQUIRE(mine.size() == angles.size());
        for (size_t k = 0; k < mine.size(); ++k)
            CHECK_THAT(mine[k], WithinAbs(angles[k], 1e-9));
    }
    const std::vector<Part> again = copies(copy_at(0, l, angles, 0), 35);
    Stats second;
    arrange(again, {}, p, &second);
    CHECK(second.unplaced == first.unplaced);
    CHECK(second.lattice_used == first.lattice_used);
}

TEST_CASE("A single part lands centred on the anchor, or in the min corner without one", "[Nest3D]")
{
    std::vector<Part> parts{make_part(1, box(0, 0, 0, 10, 20, 5))};
    Params            p = bed(100, 80);
    SECTION("anchor at 70 % of the width and 50 % of the depth") {
        p.align_x = 0.7, p.align_y = 0.5;
        const auto out = arrange(parts, {}, p);
        CHECK_THAT(out[0].x + 5., WithinAbs(70., 1e-9));
        CHECK_THAT(out[0].y + 10., WithinAbs(40., 1e-9));
    }
    SECTION("no anchor") {
        p.align_x = p.align_y = -1.;
        const auto out = arrange(parts, {}, p);
        CHECK_THAT(out[0].x, WithinAbs(0., 1e-9));
        CHECK_THAT(out[0].y, WithinAbs(0., 1e-9));
    }
}

TEST_CASE("A peg nests inside the hole of a ring", "[Nest3D]")
{
    // 30 mm square ring with a 10 mm hole, and a 6 mm peg; the bed fits only the ring.
    Mesh ring = join({box(0, 0, 0, 30, 10, 5), box(0, 20, 0, 30, 30, 5), box(0, 10, 0, 10, 20, 5), box(20, 10, 0, 30, 20, 5)});
    std::vector<Part> parts{make_part(1, ring), make_part(2, box(0, 0, 0, 6, 6, 5))};
    Params p     = bed(30, 30);
    p.max_plates = 1;
    const auto out = arrange(parts, {}, p);
    REQUIRE(out[1].plate == 0);
    CHECK(out[1].x >= out[0].x + 10. - 1e-9);
    CHECK(out[1].x + 6. <= out[0].x + 20. + 1e-9);
    CHECK(out[1].y >= out[0].y + 10. - 1e-9);
    CHECK(out[1].y + 6. <= out[0].y + 20. + 1e-9);
}

TEST_CASE("A short part fits under a table top only with stacking and enough clearance", "[Nest3D]")
{
    // 40 mm table: top slab from z = 20 to 25 on four 5 mm legs; the bed is just the table.
    Mesh table = join({box(0, 0, 20, 40, 40, 25), box(0, 0, 0, 5, 5, 20), box(35, 0, 0, 40, 5, 20),
                       box(0, 35, 0, 5, 40, 20), box(35, 35, 0, 40, 40, 20)});
    std::vector<Part> parts{make_part(1, table), make_part(2, box(0, 0, 0, 10, 10, 8))};
    Params p     = bed(40, 40);
    p.max_plates = 1;

    SECTION("stacking with an 8 mm part plus 2 mm gap under a 20 mm top") {
        p.allow_stacking = true;
        p.vertical_gap   = 2.;
        CHECK(arrange(parts, {}, p)[1].plate == 0);
    }
    SECTION("stacking with a gap larger than the free height") {
        p.allow_stacking = true;
        p.vertical_gap   = 15.;
        CHECK(arrange(parts, {}, p)[1].plate == -1);
    }
    SECTION("no stacking") {
        p.allow_stacking = false;
        CHECK(arrange(parts, {}, p)[1].plate == -1);
    }
}

TEST_CASE("With stacking and no vertical clearance, parts resting on the bed still go largest first", "[Nest3D]")
{
    // Sixteen 9 mm cubes listed before two 49 x 99 mm plates, on a 120 x 100 mm bed with a single
    // ordering. Nothing fits under a part resting on the bed, so the plates go first, side by side,
    // and the cubes fill the strip beside them. Cubes first would spread over the bed's middle and
    // push a plate onto a second bed.
    std::vector<Part> parts;
    for (int i = 0; i < 18; ++i)
        parts.push_back(copy_at(i, i < 16 ? box(0, 0, 0, 9, 9, 5) : box(0, 0, 0, 49, 99, 5), {0.}, -1, 1.));
    Params p         = bed(120, 100);
    p.orderings      = 1;
    p.allow_stacking = true;
    p.vertical_gap   = 0.;
    Stats stats;
    arrange(parts, {}, p, &stats);
    CHECK(stats.unplaced == 0);
    CHECK(stats.plates_used == 1);
}

TEST_CASE("Parts taller than the printable height are left unplaced", "[Nest3D]")
{
    std::vector<Part> parts{make_part(1, box(0, 0, 0, 10, 10, 60)), make_part(2, box(0, 0, 0, 10, 10, 40))};
    Params p     = bed(100, 100);
    p.max_height = 50.;
    Stats stats;
    const auto out = arrange(parts, {}, p, &stats);
    CHECK(out[0].plate == -1);
    CHECK(out[1].plate == 0);
    CHECK(stats.unplaced == 1);
}

TEST_CASE("Compatibility groups split plates unless mixing is allowed", "[Nest3D]")
{
    std::vector<Part> parts{make_part(1, box(0, 0, 0, 10, 10, 5), 0), make_part(2, box(0, 0, 0, 10, 10, 5), 1)};
    Params p     = bed(100, 100);
    p.mix_groups = GENERATE(false, true);
    const auto out = arrange(parts, {}, p);
    CHECK((out[0].plate != out[1].plate) == !p.mix_groups);
}

TEST_CASE("A fixed part's group keeps other groups off its plate", "[Nest3D]")
{
    Obstacle fixed{0, HeightMap(10, 10, 1.), 1};
    for (Span &s : fixed.map.cells)
        s.add(0.f, 5.f);
    Obstacle zone{-1, HeightMap(5, 5, 1.)}; // on every plate, no group
    for (Span &s : zone.map.cells)
        s.add(0.f, 5.f);
    std::vector<Part> parts{make_part(1, box(0, 0, 0, 10, 10, 5), 0), make_part(2, box(0, 0, 0, 10, 10, 5), 1)};
    Params p     = bed(100, 100);
    p.mix_groups = false;
    const auto out = arrange(parts, {fixed, zone}, p);
    CHECK(out[0].plate == 1);
    CHECK(out[1].plate == 0);
    for (const Placement &pl : out)
        CHECK((pl.x >= 5. - 1e-9 || pl.y >= 5. - 1e-9));
}

TEST_CASE("Locked plates receive no parts", "[Nest3D]")
{
    std::vector<Part> parts{make_part(1, box(0, 0, 0, 10, 10, 5)), make_part(2, box(0, 0, 0, 10, 10, 5))};
    Params p        = bed(100, 100);
    p.locked_plates = {0};
    const auto out  = arrange(parts, {}, p);
    CHECK(out[0].plate == 1);
    CHECK(out[1].plate == 1);
}

TEST_CASE("Parts avoid obstacles on the plate", "[Nest3D]")
{
    // The obstacle fills x < 30 of a 50 x 50 bed.
    Obstacle o{0, HeightMap(30, 50, 1.)};
    for (Span &s : o.map.cells)
        s.add(0.f, 5.f);
    std::vector<Part> parts{make_part(1, box(0, 0, 0, 15, 15, 5))};
    const auto out = arrange(parts, {o}, bed(50, 50));
    REQUIRE(out[0].plate == 0);
    CHECK(out[0].x >= 30. - 1e-9);
    CHECK(out[0].x + 15. <= 50. + 1e-9);
}

TEST_CASE("Stopping early leaves the remaining parts unplaced", "[Nest3D]")
{
    // Copies are stopped while their lattices are built, before any part is placed.
    const bool        copies = GENERATE(false, true);
    std::vector<Part> parts{make_part(1, box(0, 0, 0, 10, 10, 5)), make_part(2, box(0, 0, 0, 10, 10, 5))};
    for (Part &part : parts)
        part.shape_key = copies ? 0 : -1;
    Params p      = bed(100, 100);
    p.should_stop = [] { return true; };
    Stats      stats;
    const auto out = arrange(parts, {}, p, &stats);
    CHECK(out[0].plate == -1);
    CHECK(out[1].plate == -1);
    CHECK(stats.unplaced == 2);
}

namespace {

// Random boxes and L-shapes, each offered at 0 and 90 degrees, spaced by 1 mm.
std::vector<Part> random_parts(int n, unsigned seed)
{
    std::mt19937                          rng(seed);
    std::uniform_real_distribution<float> size(8.f, 45.f), height(3.f, 40.f);
    std::vector<Part>                     parts;
    for (int i = 0; i < n; ++i) {
        float a = size(rng), b = size(rng), h = height(rng);
        Mesh  m = (i % 3 == 0) ? join({box(0, 0, 0, a, a / 3, h), box(0, a / 3, 0, a / 3, b, h)}) : box(0, 0, 0, a, b, h);
        Part  p;
        p.id = i;
        for (int r = 0; r < 2; ++r) {
            Mesh rotated = m;
            if (r == 1)
                for (Triangle &t : rotated)
                    for (auto &v : t.v)
                        std::swap(v[0], v[1]), v[0] = -v[0];
            p.candidates.push_back({dilate(rasterize(rotated, 1., 0., false), 1.), r * 0.5 * Pi});
        }
        parts.push_back(std::move(p));
    }
    return parts;
}

} // namespace

TEST_CASE("The result does not depend on the order parallel tasks run in", "[Nest3D]")
{
    // Unique parts plus copies, which also exercise the lattice search and seeding.
    std::vector<Part> parts = random_parts(30, 7);
    for (const Part &copy : copies(copy_of(100, polyomino({{0, 0}, {1, 0}, {2, 0}, {2, 1}}, 8.f), 2, 0), 8))
        parts.push_back(copy);
    Params                  p     = bed(150, 150);
    p.orderings = 3;
    const auto serial = arrange(parts, {}, p);
    p.parallel_for = [](size_t n, const std::function<void(size_t)> &fn) {
        for (size_t i = n; i-- > 0;)
            fn(i);
    };
    const auto reversed = arrange(parts, {}, p);
    CHECK(overlapping_cells(parts, serial, p.cell_mm) == 0);
    for (size_t i = 0; i < parts.size(); ++i) {
        CHECK(serial[i].plate == reversed[i].plate);
        CHECK_THAT(serial[i].x, WithinAbs(reversed[i].x, 1e-12));
        CHECK_THAT(serial[i].y, WithinAbs(reversed[i].y, 1e-12));
        CHECK_THAT(serial[i].rotation_z, WithinAbs(reversed[i].rotation_z, 1e-12));
    }
}

TEST_CASE("The result does not depend on the order the parts are listed in", "[Nest3D]")
{
    // Boxes and L-shapes of one height whose sides take few values, so that the size keys tie
    // between different shapes, plus a group of copies; listed forwards and backwards with the
    // same ids.
    std::vector<Part> parts;
    for (int i = 0; i < 24; ++i) {
        const float a = 10.f + 6.f * (i % 4), b = 10.f + 6.f * (i / 4 % 3);
        const Mesh  m = i % 5 == 0 ? join({box(0, 0, 0, a, 6, 5), box(0, 6, 0, 6, b, 5)}) : box(0, 0, 0, a, b, 5);
        parts.push_back(copy_at(i, m, turns(4), -1, 1.));
    }
    for (const Part &copy : copies(copy_at(100, box(0, 0, 0, 6, 6, 5), turns(4), 0, 1.), 8))
        parts.push_back(copy);
    const std::vector<Part> reversed(parts.rbegin(), parts.rend());
    Params                  p = bed(150, 150);
    p.orderings               = 3;
    const auto a = arrange(parts, {}, p), b = arrange(reversed, {}, p);
    // Copies are interchangeable, so compare the placements of each shape as a set.
    auto spots = [](const std::vector<Part> &ps, const std::vector<Placement> &out) {
        std::set<std::tuple<int, int, long, long, long>> s;
        for (size_t i = 0; i < ps.size(); ++i)
            s.insert({ps[i].shape_key < 0 ? ps[i].id : -1 - ps[i].shape_key, out[i].plate, std::lround(out[i].x * 1e6),
                      std::lround(out[i].y * 1e6), std::lround(out[i].rotation_z * 1e6)});
        return s;
    };
    CHECK(spots(parts, a) == spots(reversed, b));
}

TEST_CASE("A mix of unique parts packs into a compact pile", "[Nest3D][Regression]")
{
    // 15 random boxes and L-shapes, 1 mm apart, use a small part of a 256 mm bed. The pile's
    // bounding box stays within 1.4 times the parts' summed footprint (with spacing). Growing the
    // pile evenly around the anchor, as copies do, gives 1.5 to 1.7 on these sets.
    const unsigned          seed  = GENERATE(13u, 24u, 26u);
    const std::vector<Part> parts = random_parts(15, seed);
    Params                  p     = bed(256, 256);
    p.max_plates                  = 1;
    const auto out = arrange(parts, {}, p);
    double     footprint = 0.;
    for (size_t i = 0; i < parts.size(); ++i) {
        REQUIRE(out[i].plate == 0);
        footprint += double(placed_cells(parts[i], out[i], p.cell_mm).size());
    }
    INFO("seed " << seed << ": bbox " << plate_bbox_area(parts, out) << " mm^2, footprint " << footprint << " mm^2");
    CHECK(plate_bbox_area(parts, out) <= 1.4 * footprint);
}

TEST_CASE("A mix of copies packs into a compact pile", "[Nest3D][Regression]")
{
    // Like the unique mix above, with every part present two or three times as copies, scored by
    // the bounding box like unique parts: the pile stays within 1.28 times the parts' footprint (it
    // takes 1.19 to 1.23). Growing each group of copies evenly around the anchor gives 1.23 to 1.33.
    const unsigned          seed   = GENERATE(13u, 24u, 26u);
    const std::vector<Part> unique = random_parts(8, seed);
    std::vector<Part>       parts;
    for (size_t k = 0; k < unique.size(); ++k)
        for (size_t n = 0; n < 2 + k % 2; ++n) {
            parts.push_back(unique[k]);
            parts.back().id        = int(parts.size());
            parts.back().shape_key = int(k);
        }
    Params p     = bed(256, 256);
    p.max_plates = 1;
    const auto out       = arrange(parts, {}, p);
    double     footprint = 0.;
    for (size_t i = 0; i < parts.size(); ++i) {
        REQUIRE(out[i].plate == 0);
        footprint += double(placed_cells(parts[i], out[i], p.cell_mm).size());
    }
    INFO("seed " << seed << ": bbox " << plate_bbox_area(parts, out) << " mm^2, footprint " << footprint << " mm^2");
    CHECK(plate_bbox_area(parts, out) <= 1.28 * footprint);
}

// Hidden: prints timings for growing part counts on a 256 mm bed.
TEST_CASE("Arranging random boxes and L-shapes scales polynomially", "[Nest3D][.][Benchmark]")
{
    for (int n : {50, 100, 200}) {
        const std::vector<Part> parts = random_parts(n, 42);
        Params                  p     = bed(256, 256);
        Stats                   stats;
        auto                    t0  = std::chrono::steady_clock::now();
        const auto              out = arrange(parts, {}, p, &stats);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::cout << "n=" << n << " " << ms << " ms, plates " << stats.plates_used << ", unplaced " << stats.unplaced
                  << ", footprint " << stats.footprint_ratio << ", ordering " << stats.ordering_used << "\n";
        CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
    }
    // Copies of an L tetromino of 10 mm squares filling one plate.
    for (int n : {50, 150}) {
        const std::vector<Part> parts = copies(copy_of(0, polyomino({{0, 0}, {1, 0}, {2, 0}, {2, 1}}, 10.f), 4, 0), n);
        Params p     = bed(256, 256);
        p.cell_mm    = 0.5;
        p.max_plates = 1;
        Stats      stats;
        auto       t0  = std::chrono::steady_clock::now();
        const auto out = arrange(parts, {}, p, &stats);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::cout << "copies n=" << n << " " << ms << " ms, unplaced " << stats.unplaced << ", lattice " << stats.lattice_used << "\n";
        CHECK(overlapping_cells(parts, out, p.cell_mm) == 0);
    }
    // Two copies of a 120 mm L at 72 rotation steps: the lattices are built from a few candidates,
    // so keying them as copies costs about what arranging them as unique parts does.
    std::vector<double> steps;
    for (int k = 0; k < 72; ++k)
        steps.push_back(2. * Pi * k / 72);
    double ms[2];
    for (int key : {-1, 0}) {
        const Mesh        l = join({box(0, 0, 0, 120, 30, 5), box(0, 30, 0, 30, 120, 5)});
        std::vector<Part> parts{copy_at(0, l, steps, key), copy_at(1, l, steps, key)};
        Params            p = bed(256, 256);
        p.cell_mm           = 0.5;
        auto t0             = std::chrono::steady_clock::now();
        arrange(parts, {}, p);
        ms[key + 1] = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
    std::cout << "2 copies at 72 steps: " << ms[1] << " ms as copies, " << ms[0] << " ms unique\n";
    CHECK(ms[1] <= 4. * ms[0] + 50.);
}
