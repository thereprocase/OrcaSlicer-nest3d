#pragma once

// Nest3D: shape-true, height-aware auto-arrange.
//
// Every part is described by a column grid ("height map"): one cell per
// cell_mm x cell_mm square of the bed, holding the z-interval [lo, hi] that
// the part (plus any support it generates) occupies above that square. Two
// parts may share a cell only when their z-intervals are separated by at
// least the vertical clearance. This lets parts nest into each other's
// concavities and holes, and (when enabled) tuck short parts under
// unsupported overhangs of taller ones.
//
// The placement search is greedy first-fit-decreasing over a constant number
// of part orderings, with an exhaustive scan of grid positions per candidate.
// Packing grows outward from an anchor point and prefers positions where a
// part touches what is already placed. Copies of one shape (Part::shape_key)
// grow outward from an anchor point and are additionally tried laid out on the
// densest lattice found for one copy or an interlocked pair; unique parts keep
// the pile's bounding box small. Cost is polynomial (see docs/HLSD/nest3d.md);
// no step enumerates permutations or subsets.
//
// This header is deliberately free of libslic3r types so the core can be
// unit-tested without the rest of the slicer; Nest3DArrange.hpp adapts it to
// ModelInstance / ArrangePolygon.

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
                                             // unless Params::mix_groups is set
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
    bool   mix_groups = true;
    int    orderings = 3;       // number of greedy part orderings to try (1..6)
    int    max_plates = 64;
    // Anchor the packing grows from, as a fraction of the bed; the group is finally centred on it.
    // align_x < 0 grows from the bed's min corner (bottom-left) without final alignment.
    double align_x = 0.5, align_y = 0.5;
    std::vector<int> locked_plates;      // plates that accept no new parts

    // Cells outside the printable bed shape, row-major bed_w/cell_mm columns.
    // Empty means the whole rectangle is printable.
    std::vector<uint8_t> bed_mask;

    // Runs fn(i) for i in [0, n). Defaults to a serial loop; the adapter
    // injects a TBB-backed version.
    std::function<void(size_t n, const std::function<void(size_t)> &fn)> parallel_for;
    // Polled between parts and inside parallel tasks, so it must be thread-safe.
    std::function<bool()>                 should_stop;
    std::function<void(int done, int total)> progress;
};

struct Stats
{
    int    plates_used = 0;
    int    unplaced = 0;
    double footprint_ratio = 0.0; // occupied bed area / bed area over used plates
    int    ordering_used = 0;
    bool   lattice_used = false; // the result lays copies of a shape on a lattice
};

// Builds a height map from a closed or open triangle soup. The map covers the
// XY bounding box of the triangles, expanded by `inflate_mm` on every side,
// and every cell touched by a triangle receives that triangle's z-range
// within the cell. When `fill_to_bed` is set (the part prints with support),
// every non-empty column is extended down to z = 0.
HeightMap rasterize(const std::vector<Triangle> &tris, double cell_mm, double inflate_mm, bool fill_to_bed);

// Grows every non-empty column by `radius_mm` in XY: each cell that comes within
// the radius of a non-empty cell takes the union of their z-spans. The growth is
// conservative, so geometry anywhere inside a cell keeps at least `radius_mm`
// of clearance. Used for spacing and brims; brim-style growth only for the first
// layer is done with `bottom_only_mm` > 0: cells within `bottom_only_radius_mm`
// of the first-layer footprint receive [0, bottom_only_mm]. Vertical clearance
// is applied at placement by Params::vertical_gap.
HeightMap dilate(const HeightMap &m, double radius_mm, double bottom_only_radius_mm = 0.0, double bottom_only_mm = 0.0);

// Z angles current + offset for every offset, normalised to [0, 2 pi) and sorted. Copies whose
// current angles differ by a multiple of an evenly spaced offset list get the same angles in the
// same order, so they can share a Part::shape_key.
std::vector<double> candidate_angles(double current, const std::vector<double> &offsets);

// Arranges `parts` around fixed `obstacles`. Returns one Placement per part,
// in input order.
std::vector<Placement> arrange(const std::vector<Part> &parts, const std::vector<Obstacle> &obstacles, const Params &params, Stats *stats = nullptr);

}} // namespace Slic3r::nest3d

