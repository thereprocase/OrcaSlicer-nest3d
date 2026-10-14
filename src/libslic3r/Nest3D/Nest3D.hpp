#pragma once

// Nest3D: shape-true, height-aware auto-arrange (see docs/HLSD/nest3d.md).
//
// Every part is a column grid, the height map: one cell per cell_mm square of the bed, holding the
// z-interval the part occupies above that square. Two parts may share a cell only when their intervals
// are at least Params::vertical_gap apart. The engine uses no libslic3r types, so tests can run
// it alone; Nest3DArrange adapts it to ArrangePolygon.

#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace Slic3r { namespace nest3d {

constexpr double Inf = std::numeric_limits<double>::infinity();

// z-interval occupied above one bed cell, in mm. Empty when lo > hi.
struct Span
{
    float lo = std::numeric_limits<float>::max();
    float hi = std::numeric_limits<float>::lowest();

    bool empty() const { return lo > hi; }
    void add(float z_lo, float z_hi)
    {
        lo = lo < z_lo ? lo : z_lo;
        hi = hi > z_hi ? hi : z_hi;
    }
    void add(const Span &s) { add(s.lo, s.hi); }
};

// Row-major grid of Spans. Cell (x, y) covers [origin_x + x * cell_mm, origin_x + (x + 1) * cell_mm) on
// X, likewise on Y.
struct HeightMap
{
    int               width = 0, height = 0;
    double            cell_mm = 1., origin_x = 0., origin_y = 0.;
    std::vector<Span> cells;

    HeightMap() = default;
    HeightMap(int w, int h, double cell) : width(w), height(h), cell_mm(cell), cells(size_t(w) * size_t(h)) {}

    Span       &at(int x, int y) { return cells[size_t(y) * size_t(width) + size_t(x)]; }
    const Span &at(int x, int y) const { return cells[size_t(y) * size_t(width) + size_t(x)]; }
};

struct Triangle { float v[3][3]; }; // three vertices, xyz in mm

// Height map of a triangle soup over its XY bounding box grown by `inflate_mm` on every side. A cell
// takes the z-range of the geometry inside it, and every cell closer than `inflate_mm` to that geometry
// takes it too, so parts grown by r1 and r2 whose maps share no cell keep at least r1 + r2 apart.
// Distances are measured from the geometry inside each cell, so the clearance is not rounded up to
// whole cells. With `fill_to_bed` (the part prints with support) every column reaches down to z = 0.
HeightMap rasterize(const std::vector<Triangle> &tris, double cell_mm, double inflate_mm, bool fill_to_bed);

// Runs fn(i) for i in [0, n), in any order and on any threads.
using ParallelFor = std::function<void(size_t n, const std::function<void(size_t)> &fn)>;

// One Z rotation of a shape: its height map in a frame where the part's reference point is (0, 0), and
// the rotation in radians about Z relative to the part's input pose.
struct Candidate { HeightMap map; double rotation_z = 0.; };

// A part to arrange. `id` is echoed in its Placement; `shape` indexes the shapes passed to arrange(), and
// parts of one shape are copies; higher `priority` is placed earlier; parts of different `group`s never
// share a plate.
struct Part { int id = 0, shape = 0; double priority = 0.; int group = 0; };

// What must not move, in plate coordinates, on `plate` or on every plate for -1: a fixed part, with its
// group, or an exclusion zone or the bed's outline, with group -1.
struct Obstacle { int plate = 0; HeightMap map; int group = -1; };

// Where a part goes: its plate (-1: not placed), the offset in mm from the bed's min corner to its
// reference point, and its rotation.
struct Placement { int id = 0, plate = -1; double x = 0., y = 0., rotation_z = 0.; };

struct Params
{
    double bed_w = 256., bed_h = 256., cell_mm = 1., max_height = Inf;
    // Min z gap for parts sharing a column. Inf: no column is shared. It is only ever added and
    // compared, never subtracted or multiplied, so Inf needs no special case.
    double vertical_gap = Inf;
    int    max_plates   = 64;
    // Anchor every plate's pile grows from and is centred on, as a fraction of the bed; align_x < 0
    // grows piles from the bed's min corner and leaves them there.
    double                                   align_x = 0.5, align_y = 0.5;
    ParallelFor                              parallel_for; // serial when empty
    std::function<bool()>                    should_stop;  // polled from any thread
    std::function<void(int done, int total)> progress;
};

// Arranges `parts` around `obstacles`; shapes[part.shape] lists a part's candidates. Returns one
// Placement per part, in input order.
std::vector<Placement> arrange(const std::vector<std::vector<Candidate>> &shapes, const std::vector<Part> &parts,
                               const std::vector<Obstacle> &obstacles, const Params &params);

}} // namespace Slic3r::nest3d
