// Height maps from triangles, and footprints from height maps. Two ideas carry the rasterizer:
// - Slicing: a triangle is cut into columns and each column into cells, only at the grid lines inside
//   its range, so a piece is cut once per line it crosses and a piece inside one cell is not cut at all.
// - Run claims: along a row, the distance bound from a cell's geometry is convex, so the cells it claims
//   form a run, found by trimming the row from both ends instead of testing every cell.

#include "Grid.hpp"
#include "Nest3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace Slic3r { namespace nest3d {
namespace {

struct Point { double x, y, z; };

// A convex polygon in cell units. A triangle cut by four grid lines keeps at most 7 vertices; 12 leaves
// room for rounding on near-degenerate walls.
struct Piece
{
    Point vertices[12];
    int   count = 0;

    void add(const Point &p) { vertices[count++] = p; }
};

Point lerp(const Point &a, const Point &b, double t)
{
    return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
}

// Cuts a convex piece into unit strips along `axis` and calls fn(i, strip) for each strip [i, i + 1] it
// reaches in a map `size` strips long. Each cut splits off the strip below one grid line inside the
// piece's range. z is interpolated along cut edges, which is exact for a planar triangle.
template<class Fn> void slice(Piece rest, double Point::*axis, int size, Fn &&fn)
{
    const auto less_along_axis   = [axis](const Point &a, const Point &b) { return a.*axis < b.*axis; };
    const auto [lowest, highest] = std::minmax_element(rest.vertices, rest.vertices + rest.count, less_along_axis);
    const int first              = std::clamp(int(std::floor(lowest->*axis)), 0, size - 1);
    const int last               = std::clamp(int(std::ceil(highest->*axis)) - 1, first, size - 1);
    for (int i = first; i < last; ++i) {
        const double line = i + 1;
        Piece        below, above;
        for (int k = 0, j = rest.count - 1; k < rest.count; j = k++) {
            const Point &a      = rest.vertices[j];
            const Point &b      = rest.vertices[k];
            const double dist_a = a.*axis - line;
            const double dist_b = b.*axis - line;
            if (dist_a <= 0.)
                below.add(a);
            if (dist_a >= 0.)
                above.add(a);
            if (dist_a * dist_b < 0.) {
                const Point crossing = lerp(a, b, dist_a / (dist_a - dist_b));
                below.add(crossing);
                above.add(crossing);
            }
        }
        fn(i, below);
        rest = above;
    }
    fn(last, rest);
}

// A piece that only touches the cell's border does not occupy it: a face needs area inside the cell, a
// wall needs length.
bool occupies(const Piece &piece, bool wall)
{
    double size = 0.;
    for (int i = 0, j = piece.count - 1; i < piece.count; j = i++) {
        const Point &a = piece.vertices[j];
        const Point &b = piece.vertices[i];
        if (wall)
            size += std::abs(b.x - a.x) + std::abs(b.y - a.y);
        else
            size += a.x * b.y - b.x * a.y;
    }
    return std::abs(size) > 1e-9;
}

// The geometry inside one cell: its z span, and its ranges along x, y, x + y and x - y, cell-local, which
// bound an octagon holding it. Growth measures from the octagon, so clearance is not rounded up to whole
// cells.
struct Extent
{
    Span along[4]; // the ranges along x, y, x + y and x - y
    Span z;

    void add(const Point &p, int cell_x, int cell_y)
    {
        const double x = p.x - cell_x, y = p.y - cell_y;
        const double coordinate[4] = {x, y, x + y, x - y};
        for (int k = 0; k < 4; ++k)
            along[k].add(float(coordinate[k]), float(coordinate[k]));
        z.add(float(p.z), float(p.z));
    }
    // Geometry reaching all four corners, the ends of both diagonals, has the whole cell as its octagon,
    // so it claims a whole cell's disc and can join the stencil dilation.
    bool whole() const
    {
        return along[2].hi - along[2].lo > 2.f - 1e-4f && along[3].hi - along[3].lo > 2.f - 1e-4f;
    }
};

constexpr Extent WholeCell = {{{0, 1}, {0, 1}, {0, 2}, {-1, 1}}, {}};

// The gap along frame axis k between an extent's range and a whole cell `at` units away on that axis.
double gap_along(const Extent &e, int k, int at)
{
    return std::max({0.f, at + WholeCell.along[k].lo - e.along[k].hi, e.along[k].lo - at - WholeCell.along[k].hi});
}

// A lower bound on the squared distance, in cells, from an extent's octagon to the cell (dx, dy) cells
// away: the gap between their bounding boxes in the axis frame and in the 45 degree frame, whose units
// are sqrt(2) long. Exact for axis-parallel and diagonal edges, and for a whole cell.
double distance2(const Extent &e, int dx, int dy)
{
    const double gap_x = gap_along(e, 0, dx);
    const double gap_y = gap_along(e, 1, dy);
    const double gap_u = gap_along(e, 2, dx + dy);
    const double gap_v = gap_along(e, 3, dx - dy);
    return std::max(gap_x * gap_x + gap_y * gap_y, 0.5 * (gap_u * gap_u + gap_v * gap_v));
}

// Adds a triangle to the extent of every cell it occupies. The triangle goes to cell units, so cell
// (i, j) is the unit square at (i, j). A vertical wall covers no area, so it is nudged a hair against
// its normal, onto the solid side: on a grid line it then marks the cells of its closed part instead of
// widening it by one. Clamping keeps a wall on the map's outer edge inside the map.
void paint(std::vector<Extent> &extents, const HeightMap &map, const Triangle &tri)
{
    auto         edge  = [&tri](int k, int c) { return double(tri.v[k][c]) - tri.v[0][c]; };
    auto         cross = [&edge](int a, int b) { return edge(1, a) * edge(2, b) - edge(1, b) * edge(2, a); };
    const double nx = cross(1, 2), ny = cross(2, 0), nz = cross(0, 1), nxy = std::hypot(nx, ny);
    const bool   wall = std::abs(nz) <= 1e-5 * std::hypot(nxy, nz); // relative: float input leaves residue
    const double push = wall && nxy > 0. ? 1e-3 / nxy : 0.;
    auto         to_cells = [&](double v, double origin, double normal, int size) {
        return std::clamp((v - origin) / map.cell_mm - push * normal, 0., double(size));
    };
    Piece triangle;
    for (const auto &v : tri.v) {
        const double x = to_cells(v[0], map.origin_x, nx, map.width);
        const double y = to_cells(v[1], map.origin_y, ny, map.height);
        triangle.add({x, y, double(v[2])});
    }
    slice(triangle, &Point::x, map.width, [&](int cell_x, const Piece &column) {
        slice(column, &Point::y, map.height, [&](int cell_y, const Piece &cell) {
            if (occupies(cell, wall))
                for (int k = 0; k < cell.count; ++k)
                    extents[size_t(cell_y) * map.width + cell_x].add(cell.vertices[k], cell_x, cell_y);
        });
    });
}

// The cells of row dy, from lo to hi, that an extent comes closer than `radius` to. Its distance bound
// is convex along a row, so they form a run, found by trimming the row from both ends. Empty when
// hi < lo.
std::pair<int, int> claimed_run(const Extent &e, int dy, int lo, int hi, double radius)
{
    while (hi >= lo && distance2(e, hi, dy) >= radius * radius - 1e-9)
        --hi;
    while (lo < hi && distance2(e, lo, dy) >= radius * radius - 1e-9)
        ++lo;
    return {lo, hi};
}

// Every cell closer than `radius` (in cells) to the geometry of an occupied cell takes that cell's span.
// Spans are read from the extents, which growth leaves alone, so a claimed span is not passed on.
//
// A whole cell whose eight neighbours are whole and hold its span claims nothing they do not: a point
// within the radius of it is within the radius of the neighbour that the segment to its nearest point
// leaves through. Skipping those keeps the cost of growth near that of the outline, which matters at the
// 12 mm of tree supports.
void grow(HeightMap &map, const std::vector<Extent> &extents, double radius)
{
    const int reach = int(std::ceil(radius));
    auto      covers = [&](int x, int y, const Span &span) {
        if (x < 0 || y < 0 || x >= map.width || y >= map.height)
            return false;
        const Extent &e = extents[size_t(y) * map.width + x];
        return e.whole() && e.z.lo <= span.lo && e.z.hi >= span.hi;
    };
    for (int y = 0; y < map.height; ++y)
        for (int x = 0; x < map.width; ++x) {
            const Extent &e = extents[size_t(y) * map.width + x];
            if (e.z.empty())
                continue;
            bool surrounded = true;
            for (int k = 0; k < 9 && surrounded; ++k)
                surrounded = covers(x + k % 3 - 1, y + k / 3 - 1, e.z);
            if (surrounded)
                continue;
            for (int dy = std::max(-reach, -y); dy <= std::min(reach, map.height - 1 - y); ++dy) {
                const int  dx_lo    = std::max(-reach, -x);
                const int  dx_hi    = std::min(reach, map.width - 1 - x);
                const auto [lo, hi] = claimed_run(e, dy, dx_lo, dx_hi, radius);
                for (int dx = lo; dx <= hi; ++dx)
                    map.at(x + dx, y + dy).add(e.z);
            }
        }
}

} // namespace

HeightMap rasterize(const std::vector<Triangle> &tris, double cell_mm, double inflate_mm, bool fill_to_bed)
{
    if (tris.empty())
        return HeightMap(0, 0, cell_mm);
    Span x_range, y_range; // the XY bounding box
    for (const Triangle &t : tris)
        for (const auto &v : t.v) {
            x_range.add(v[0], v[0]);
            y_range.add(v[1], v[1]);
        }
    auto cells_across = [&](const Span &range) {
        return std::max(1, int(std::ceil((double(range.hi) - range.lo + 2. * inflate_mm) / cell_mm - 1e-6)));
    };
    HeightMap map(cells_across(x_range), cells_across(y_range), cell_mm);
    map.origin_x = x_range.lo - inflate_mm;
    map.origin_y = y_range.lo - inflate_mm;
    std::vector<Extent> extents(map.cells.size());
    for (const Triangle &t : tris)
        paint(extents, map, t);
    std::transform(extents.begin(), extents.end(), map.cells.begin(), [](const Extent &e) { return e.z; });
    grow(map, extents, inflate_mm / cell_mm);
    for (Span &s : map.cells)
        if (fill_to_bed && !s.empty())
            s.lo = 0.f;
    return map;
}

Footprint make_footprint(const HeightMap &map, double max_height, double vertical_gap)
{
    Footprint fp;
    Rect      tight;
    for (int y = 0; y < map.height; ++y)
        for (int x = 0; x < map.width; ++x) {
            const Span &cell = map.at(x, y);
            if (cell.empty())
                continue;
            tight = tight.united({x, y, x + 1, y + 1});
            fp.top = std::max(fp.top, cell.hi);
            fp.volume += double(cell.hi - cell.lo) * map.cell_mm * map.cell_mm;
            fp.floor_cells += on_floor(cell.lo, vertical_gap);
            ++fp.cells;
        }
    if (fp.cells == 0 || fp.top > max_height)
        return fp;
    fp.x0    = tight.x0;
    fp.y0    = tight.y0;
    fp.w     = tight.x1 - tight.x0;
    fp.h     = tight.y1 - tight.y0;
    fp.words = (fp.w + 63) / 64;
    fp.spans.resize(size_t(fp.w) * fp.h);
    fp.bits.assign(size_t(fp.h) * fp.words, 0);
    for (int y = 0; y < fp.h; ++y)
        for (int x = 0; x < fp.w; ++x) {
            const Span &cell = map.at(fp.x0 + x, fp.y0 + y);
            if (cell.empty())
                continue;
            fp.spans[size_t(y) * fp.w + x] = cell;
            fp.bits[size_t(y) * fp.words + x / 64] |= uint64_t(1) << (x % 64);
        }
    while (!fp.occupied(fp.first_x, 0))
        ++fp.first_x;
    fp.halo_words = (fp.w + 2 + 63) / 64;
    fp.halo.assign(size_t(fp.h + 2) * fp.halo_words, 0);
    for (int y = -1; y <= fp.h; ++y)
        for (int x = -1; x <= fp.w; ++x) {
            const bool next_to_part = fp.occupied(x - 1, y) || fp.occupied(x + 1, y) || fp.occupied(x, y - 1) ||
                                      fp.occupied(x, y + 1);
            if (fp.occupied(x, y) || !next_to_part)
                continue;
            fp.halo[size_t(y + 1) * fp.halo_words + size_t(x + 1) / 64] |= uint64_t(1) << ((x + 1) % 64);
            ++fp.halo_cells;
        }
    fp.usable = true;
    return fp;
}

}} // namespace Slic3r::nest3d
