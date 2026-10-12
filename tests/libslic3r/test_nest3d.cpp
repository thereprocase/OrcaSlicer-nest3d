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
#include <utility>
#include <vector>

using namespace Slic3r::nest3d;
using Catch::Matchers::WithinAbs;

namespace {

using Mesh = std::vector<Triangle>;

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

// A copy of `mesh` on 0.5 mm cells, offered at `rotations` quarter turns, with 0.5 mm of
// dilation so that copies keep 1 mm apart.
Part copy_of(int id, const Mesh &mesh, int rotations, int shape_key)
{
    Part p;
    p.id        = id;
    p.shape_key = shape_key;
    Mesh m      = mesh;
    for (int r = 0; r < rotations; ++r) {
        p.candidates.push_back({dilate(rasterize(m, 0.5, 0., false), 0.5), r * 1.5707963267948966});
        for (Triangle &t : m)
            for (auto &v : t.v)
                std::swap(v[0], v[1]), v[0] = -v[0];
    }
    return p;
}

// Bed cells covered by a placed part, keyed y * 10000 + x, with the candidate chosen by rotation.
std::vector<std::pair<int, Span>> placed_cells(const Part &part, const Placement &pl, double cell)
{
    std::vector<std::pair<int, Span>> out;
    for (const Candidate &c : part.candidates) {
        if (c.rotation_z != pl.rotation_z)
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
    const float     side = 20.f, inflate = 2.f;
    const HeightMap m    = rasterize(box(5, 5, 0, 5 + side, 5 + side, side), 1., inflate, false);
    REQUIRE(m.width == int(side + 2 * inflate));
    REQUIRE(m.height == int(side + 2 * inflate));
    CHECK_THAT(m.origin_x, WithinAbs(5. - inflate, 1e-9));

    int occupied = 0;
    for (int y = 0; y < m.height; ++y)
        for (int x = 0; x < m.width; ++x) {
            const Span &s      = m.at(x, y);
            bool        inside = x >= inflate && x < inflate + side && y >= inflate && y < inflate + side;
            CHECK(s.empty() == !inside);
            if (inside) {
                ++occupied;
                CHECK_THAT(s.lo, WithinAbs(0., 1e-4));
                CHECK_THAT(s.hi, WithinAbs(side, 1e-4));
            }
        }
    CHECK(occupied == int(side * side));
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
    std::vector<Part> parts;
    for (int i = 0; i < bound + 4; ++i)
        parts.push_back(copy_of(i, mesh, 4, 7));
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
    std::vector<Part> parts{make_part(1, box(0, 0, 0, 10, 10, 5))};
    Params p      = bed(100, 100);
    p.should_stop = [] { return true; };
    CHECK(arrange(parts, {}, p)[0].plate == -1);
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
            p.candidates.push_back({dilate(rasterize(rotated, 1., 0., false), 1.), r * 1.5707963267948966});
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
    for (int i = 0; i < 8; ++i)
        parts.push_back(copy_of(100 + i, polyomino({{0, 0}, {1, 0}, {2, 0}, {2, 1}}, 8.f), 2, 0));
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
        std::vector<Part> parts;
        for (int i = 0; i < n; ++i)
            parts.push_back(copy_of(i, polyomino({{0, 0}, {1, 0}, {2, 0}, {2, 1}}, 10.f), 4, 0));
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
}
