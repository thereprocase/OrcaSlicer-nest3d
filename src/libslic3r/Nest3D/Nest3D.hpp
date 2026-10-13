#pragma once

// Nest3D: shape-true, height-aware auto-arrange (see docs/HLSD/nest3d.md).
//
// Every part is a column grid ("height map"): one cell per cell_mm x cell_mm square of the bed,
// holding the z-interval [lo, hi] that the part (plus any support it generates) occupies above
// that square. Two parts may share a cell only when their z-intervals are at least
// Params::vertical_gap apart.
//
// The engine uses no libslic3r types. Its core (the column rasterizer, raster no-fit sets,
// lattice reduction and the bitset scans) has no libslic3r counterpart and runs on plain integer
// grids, and what it duplicates is small: a bit count, pi, a two-field integer vector and a
// rectangle. Policy that needs the model (poses, Z angles, spacing, copy keys) stays in
// Nest3DArrange.hpp, which adapts the engine to ModelInstance / ArrangePolygon.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace Slic3r { namespace nest3d {

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
};

// Row-major grid of Spans. Cell (x, y) covers
// [origin_x + x * cell_mm, origin_x + (x + 1) * cell_mm) on X, likewise on Y.
struct HeightMap
{
    int               width  = 0;
    int               height = 0;
    double            cell_mm  = 1.0;
    double            origin_x = 0.0; // mm, in the part's local frame
    double            origin_y = 0.0;
    std::vector<Span> cells;

    HeightMap() = default;
    HeightMap(int w, int h, double cell) : width(w), height(h), cell_mm(cell), cells(size_t(w) * size_t(h)) {}

    Span       &at(int x, int y) { return cells[size_t(y) * size_t(width) + size_t(x)]; }
    const Span &at(int x, int y) const { return cells[size_t(y) * size_t(width) + size_t(x)]; }
    bool        in_bounds(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
};

struct Triangle
{
    float v[3][3]; // three vertices, xyz in mm
};

// One way a part may be placed: a pre-rotated geometry whose height map is
// expressed in a frame where the part's reference point (the instance origin
// after rotation) sits at (0, 0).
struct Candidate
{
    HeightMap map;
    double    rotation_z = 0.0;   // radians about Z applied relative to the part's input pose
    int       orientation = 0;    // index into the caller's orientation list (0 = as given)
    double    penalty = 0.0;      // >= 0, added to the placement score (e.g. worse overhangs)
};

struct Part
{
    int                    id = 0;     // caller's identifier, echoed in Placement
    std::vector<Candidate> candidates; // at least one
    double                 priority = 0.0; // higher is placed earlier, before size ordering
    int                    compat_group = 0; // parts in different groups never share a plate
    int                    shape_key = -1;   // parts with the same key >= 0 have identical candidates, in
                                             // the same order (copies of one object); -1: unique
};

// A part already on a plate that must not move (unselected items, wipe tower,
// exclusion zones). Its map is expressed in plate coordinates (origin = bed min corner).
struct Obstacle
{
    int       plate = 0;  // -1: on every plate
    HeightMap map;
    int       group = -1; // compat group of a fixed part; -1 for zones that hold no part
};

struct Placement
{
    int    id = 0;
    int    plate = -1;          // -1: could not be placed
    double x = 0.0, y = 0.0;    // mm, position of the reference point relative to the bed min corner
    double rotation_z = 0.0;
    int    orientation = 0;
};

struct Params
{
    double bed_w = 256.0, bed_h = 256.0; // mm; the bed is [0, bed_w] x [0, bed_h]
    double cell_mm = 1.0;
    double max_height = std::numeric_limits<double>::max(); // printable height
    double vertical_gap = 2.0;  // min z separation for parts sharing a column
    bool   allow_stacking = false; // false: any occupied cell blocks the whole column
    int    orderings = 3;       // number of greedy part orderings to try (1..6)
    int    max_plates = 64;
    // Anchor the packing grows from, as a fraction of the bed; the group is finally centred on it.
    // align_x < 0 grows from the bed's min corner (bottom-left) without final alignment.
    double align_x = 0.5, align_y = 0.5;

    // Cells outside the printable bed shape, row-major on the grid_cols() x grid_rows() bed grid.
    // Empty means the whole rectangle is printable.
    std::vector<uint8_t> bed_mask;

    // Runs fn(i) for i in [0, n). Defaults to a serial loop; the adapter
    // injects a TBB-backed version.
    std::function<void(size_t n, const std::function<void(size_t)> &fn)> parallel_for;
    // Polled between parts and inside parallel tasks, so it must be thread-safe.
    std::function<bool()>                 should_stop;
    std::function<void(int done, int total)> progress;

    // The bed grid: whole cells only, with a tolerance so a bed that is a multiple of the cell
    // keeps its last one.
    int grid_cols() const { return grid_cells(bed_w); }
    int grid_rows() const { return grid_cells(bed_h); }

private:
    int grid_cells(double extent) const { return std::max(0, int(std::floor(extent / cell_mm + 1e-6))); }
};

// User-facing switches of the shape-true arranger, as the arrange dialog stores them. Values are
// clamped where they are read in (app config, dialog); the adapter takes them as they are.
struct ArrangeSettings
{
    static constexpr int MaxRotationSteps = 72;

    bool  enabled         = false;
    bool  allow_stacking  = false; // let short parts sit under unsupported overhangs
    float vertical_gap_mm = 2.f;   // >= 0
    bool  reorient        = false; // also try lay-flat orientations
    int   rotation_steps  = 8;     // Z rotations tried when rotation is allowed, 1..MaxRotationSteps
    int   effort          = 1;     // 0 fast, 1 balanced, 2 thorough
};

struct Stats
{
    int plates_used = 0;
    int unplaced    = 0;
};

// Builds a height map from a closed or open triangle soup, covering its XY bounding box grown by
// `inflate_mm` on every side. Every cell a triangle crosses takes the triangle's z-range within
// the cell, and every cell closer than `inflate_mm` to the geometry inside such a cell takes that
// cell's z-range, so parts grown by r1 and r2 whose maps share no cell keep at least r1 + r2
// apart. Distances are measured from the extent of the geometry inside each cell along the axes
// and diagonals, so the clearance is not rounded up to whole cells. With `fill_to_bed` (the part
// prints with support) every non-empty column reaches down to z = 0.
HeightMap rasterize(const std::vector<Triangle> &tris, double cell_mm, double inflate_mm, bool fill_to_bed);

// Grid and search settings of an effort level (0 fast, 1 balanced, 2 thorough) on a bed of
// bed_w x bed_h mm.
struct Effort
{
    double cell_mm   = 0.5;
    int    orderings = 3;
};
Effort effort_settings(int effort, double bed_w, double bed_h);

// Arranges `parts` around fixed `obstacles`. Returns one Placement per part,
// in input order.
std::vector<Placement> arrange(const std::vector<Part> &parts, const std::vector<Obstacle> &obstacles, const Params &params, Stats *stats = nullptr);

}} // namespace Slic3r::nest3d

