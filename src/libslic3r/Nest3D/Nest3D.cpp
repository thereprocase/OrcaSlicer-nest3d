#include "Nest3D.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace Slic3r { namespace nest3d {

namespace {

// ---------------------------------------------------------------------------
// Rasterization
// ---------------------------------------------------------------------------

struct ClipPoint { double x, y, z; };
using ClipPoly = std::array<ClipPoint, 12>; // a triangle clipped by 4 half-planes has at most 7 vertices

// Sutherland-Hodgman step: keeps the part of `in` where sign * (coord - bound) >= 0.
// z is interpolated linearly along clipped edges, which is exact for a planar triangle.
int clip(const ClipPoly &in, int n, ClipPoly &out, bool on_y, double bound, double sign)
{
    auto dist = [&](const ClipPoint &p) { return sign * ((on_y ? p.y : p.x) - bound); };
    int m = 0;
    for (int i = 0; i < n; ++i) {
        const ClipPoint &a = in[i], &b = in[(i + 1) % n];
        double da = dist(a), db = dist(b);
        if (da >= 0.)
            out[m++] = a;
        if ((da >= 0.) != (db >= 0.)) {
            double t = da / (da - db);
            out[m++] = {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
        }
    }
    return m;
}

// ---------------------------------------------------------------------------
// Arrangement
// ---------------------------------------------------------------------------

constexpr double NoScore   = std::numeric_limits<double>::infinity();
constexpr int    Mixed     = -2;  // plate group when its fixed parts span several groups
// Scored area, in mm^2, that one mm of contact outweighs: small next to the area growth of moving
// a part by a cell, so contact mostly decides between positions of equal area.
constexpr double ContactMM = 16.;
constexpr size_t MaxBasis  = 2048; // lattice basis vectors tried per cluster, shortest first
// Candidates of a group of copies that lattices are built from: enough for the four quarter turns
// of a part, and independent of the candidate count.
constexpr size_t MaxLatticeCandidates = 4;
constexpr long long MaxCosets = 16384; // lattice phases counted when seeding copies that fill a plate
constexpr double Pi = 3.14159265358979323846;

int popcount(uint64_t v) { return int(std::bitset<64>(v).count()); }

// Index of the lowest set bit of v != 0 (std::countr_zero is C++20).
int lowest_bit(uint64_t v) { return popcount((v & (~v + 1)) - 1); }

// True when nothing fits under a column starting at height lo: it starts within the vertical gap
// of the bed, a zero gap included.
bool on_floor(float lo, double vertical_gap) { return lo <= vertical_gap + 1e-3; }

// A candidate reduced to its occupied cells: the tight rectangle of non-empty
// cells and one 64-bit occupancy bitset per row of it.
struct Shape
{
    const HeightMap      *map = nullptr;
    int                   x0 = 0, y0 = 0, w = 0, h = 0; // tight rect in map cells
    int                   words = 0;                   // bitset words per row
    std::vector<uint64_t> bits;
    // Empty cells 4-adjacent to the shape, rows -1..h and columns -1..w of the tight rect:
    // what the shape touches when they are occupied.
    int                   halo_words = 0;
    std::vector<uint64_t> halo;
    int                   halo_cells = 0;
    std::array<int, 4>    edge{}; // halo cells left of, right of, below and above the tight rect
    int                   cells = 0;
    int                   floor_cells = 0; // cells with nothing fitting under them (see on_floor())
    float                 top = 0.f;
    double                volume = 0.;
    double                factor = 1.; // score multiplier from the candidate's penalty
    bool                  usable = false;

    bool test(int x, int y) const
    {
        return x >= 0 && y >= 0 && x < w && y < h && (bits[size_t(y) * words + x / 64] >> (x % 64) & 1);
    }
};

Shape make_shape(const HeightMap &m, double max_height, double vertical_gap)
{
    Shape s;
    s.map    = &m;
    int x1 = -1, y1 = -1;
    s.x0 = m.width, s.y0 = m.height;
    for (int y = 0; y < m.height; ++y)
        for (int x = 0; x < m.width; ++x)
            if (const Span &c = m.at(x, y); !c.empty()) {
                s.x0 = std::min(s.x0, x), s.y0 = std::min(s.y0, y);
                x1 = std::max(x1, x), y1 = std::max(y1, y);
                s.top = std::max(s.top, c.hi);
                s.volume += double(c.hi - c.lo) * m.cell_mm * m.cell_mm;
                ++s.cells;
                s.floor_cells += on_floor(c.lo, vertical_gap);
            }
    if (s.cells == 0 || s.top > max_height)
        return s;
    s.w = x1 - s.x0 + 1, s.h = y1 - s.y0 + 1;
    s.words = (s.w + 63) / 64;
    s.bits.assign(size_t(s.h) * s.words, 0);
    for (int y = 0; y < s.h; ++y)
        for (int x = 0; x < s.w; ++x)
            if (!m.at(s.x0 + x, s.y0 + y).empty())
                s.bits[size_t(y) * s.words + x / 64] |= uint64_t(1) << (x % 64);
    s.halo_words = (s.w + 2 + 63) / 64;
    s.halo.assign(size_t(s.h + 2) * s.halo_words, 0);
    for (int y = -1; y <= s.h; ++y)
        for (int x = -1; x <= s.w; ++x)
            if (!s.test(x, y) && (s.test(x - 1, y) || s.test(x + 1, y) || s.test(x, y - 1) || s.test(x, y + 1))) {
                s.halo[size_t(y + 1) * s.halo_words + (x + 1) / 64] |= uint64_t(1) << ((x + 1) % 64);
                ++s.halo_cells;
                s.edge[0] += x < 0, s.edge[1] += x == s.w, s.edge[2] += y < 0, s.edge[3] += y == s.h;
            }
    s.usable = true;
    return s;
}

// Occupancy of one bed: spans per cell, a per-row bitset of non-empty cells and their
// summed-area table. Cell x of a row is bit x + 64 of it: one zero word pads each side, so
// shifted reads may start up to 64 cells left of the bed and end one word right of it.
struct Board
{
    int                   w = 0, h = 0, words = 0;
    int                   filled = 0; // non-empty cells
    std::vector<Span>     occ;
    std::vector<uint64_t> bits;
    // With stacking, the cells with nothing fitting under them (see on_floor()), laid out like
    // `bits`; empty without stacking, where every occupied cell blocks its column.
    std::vector<uint64_t> floor;
    double                vertical_gap = 0.;
    std::vector<int>      sums; // (w + 1) x (h + 1): non-empty cells below and left of each corner

    Board(int w_, int h_, bool stacking = false, double vertical_gap_ = 0.)
        : w(w_), h(h_), words((w_ + 63) / 64 + 2), occ(size_t(w_) * h_), bits(size_t(h_) * words, 0)
        , floor(stacking ? bits.size() : 0, 0), vertical_gap(vertical_gap_), sums(size_t(w_ + 1) * (h_ + 1), 0)
    {}

    // Brings the summed-area table up to date after rows [y0, y1) changed: those rows are recounted,
    // and every row above takes the change in their cumulative count, a plain vector add.
    void update_sums(int y0, int y1)
    {
        y0 = std::max(y0, 0), y1 = std::min(y1, h);
        int *const       top = &sums[size_t(y1) * (w + 1)];
        std::vector<int> delta(top, top + w + 1);
        for (int y = y0; y < y1; ++y)
            for (int x = 0, row = 0; x < w; ++x) {
                row += (bits[size_t(y) * words + x / 64 + 1] >> (x % 64)) & 1;
                sums[size_t(y + 1) * (w + 1) + x + 1] = sums[size_t(y) * (w + 1) + x + 1] + row;
            }
        for (int x = 0; x <= w; ++x)
            delta[size_t(x)] = top[x] - delta[size_t(x)];
        for (int y = y1 + 1; y <= h; ++y) {
            int *row = &sums[size_t(y) * (w + 1)];
            for (int x = 0; x <= w; ++x)
                row[x] += delta[size_t(x)];
        }
    }

    // Non-empty cells in [x0, x1) x [y0, y1), clipped to the bed.
    int count(int x0, int y0, int x1, int y1) const
    {
        x0 = std::max(x0, 0), y0 = std::max(y0, 0), x1 = std::min(x1, w), y1 = std::min(y1, h);
        auto at = [&](int x, int y) { return sums[size_t(y) * (w + 1) + x]; };
        return x0 < x1 && y0 < y1 ? at(x1, y1) - at(x0, y1) - at(x1, y0) + at(x0, y0) : 0;
    }

    void add(int x, int y, const Span &s)
    {
        Span &c = occ[size_t(y) * w + x];
        c.add(s.lo, s.hi);
        const size_t   i   = size_t(y) * words + x / 64 + 1;
        const uint64_t bit = uint64_t(1) << (x % 64);
        filled += (bits[i] & bit) == 0;
        bits[i] |= bit;
        if (!floor.empty() && on_floor(c.lo, vertical_gap))
            floor[i] |= bit;
    }

    // Occupancy of the 64 cells of row y starting at cell x, -64 <= x <= w; with `at_floor`, only
    // of the cells with nothing fitting under them.
    uint64_t read(int y, int x, bool at_floor = false) const
    {
        const uint64_t *row = &(at_floor && !floor.empty() ? floor : bits)[size_t(y) * words];
        const int       pos = x + 64, i = pos / 64, sh = pos % 64;
        return sh ? (row[i] >> sh) | (row[i + 1] << (64 - sh)) : row[i];
    }

    // First cell x' in [x, x_end) of row y that is occupied (or free, with `occupied` false), as
    // read(); x_end if none.
    int next(int y, int x, int x_end, bool occupied, bool at_floor = false) const
    {
        for (; x < x_end; x += 64) {
            uint64_t v = occupied ? read(y, x, at_floor) : ~read(y, x, at_floor);
            if (x_end - x < 64)
                v &= (uint64_t(1) << (x_end - x)) - 1;
            if (v)
                return x + lowest_bit(v);
        }
        return x_end;
    }

    // True when `s`, with its tight rect at bed cell (tx, ty), collides with nothing.
    // Overlapping cells are found 64 at a time; spans are compared only when stacking.
    // The middle row goes first: a collision is likelier there than at the rect's edge.
    bool fits(const Shape &s, int tx, int ty, const Params &p) const
    {
        for (int i = 0; i < s.h; ++i) {
            const int r = i == 0 ? s.h / 2 : i <= s.h / 2 ? i - 1 : i;
            const uint64_t *srow = &s.bits[size_t(r) * s.words];
            for (int k = 0; k < s.words; ++k) {
                uint64_t hit = read(ty + r, tx + 64 * k) & srow[k];
                if (hit == 0)
                    continue;
                if (!p.allow_stacking)
                    return false;
                for (int col = 64 * k; hit; hit >>= 1, ++col)
                    if (hit & 1) {
                        const Span &a = s.map->at(s.x0 + col, s.y0 + r);
                        const Span &b = occ[size_t(ty + r) * w + tx + col];
                        if (!(a.hi + p.vertical_gap <= b.lo || b.hi + p.vertical_gap <= a.lo))
                            return false;
                    }
            }
        }
        return true;
    }

    // As fits(), for a position that may leave the bed. A rect free of occupied cells, read from
    // the summed-area table, needs no collision test.
    bool fits_in_bed(const Shape &s, int tx, int ty, const Params &p) const
    {
        return tx >= 0 && ty >= 0 && tx + s.w <= w && ty + s.h <= h && (count(tx, ty, tx + s.w, ty + s.h) == 0 || fits(s, tx, ty, p));
    }

    // Cells of the outline of `s` at (tx, ty) that touch an occupied cell.
    int contact(const Shape &s, int tx, int ty) const
    {
        int c = 0;
        for (int r = std::max(0, 1 - ty); r < std::min(s.h + 2, h + 1 - ty); ++r) {
            const uint64_t *hrow = &s.halo[size_t(r) * s.halo_words];
            for (int k = 0; k < s.halo_words; ++k)
                if (hrow[k])
                    c += popcount(read(ty - 1 + r, tx - 1 + 64 * k) & hrow[k]);
        }
        return c;
    }

    void place(const Shape &s, int tx, int ty)
    {
        for (int y = 0; y < s.h; ++y)
            for (int x = 0; x < s.w; ++x)
                if (const Span &c = s.map->at(s.x0 + x, s.y0 + y); !c.empty())
                    add(tx + x, ty + y, c);
        update_sums(ty, ty + s.h);
    }
};

// ---------------------------------------------------------------------------
// Raster no-fit sets and lattices, for copies of one shape
// ---------------------------------------------------------------------------

using Runs = std::vector<std::vector<std::pair<int, int>>>; // per row, [begin, end) of occupied cells

Runs runs_of(const Shape &s)
{
    Runs out(size_t(s.h));
    for (int y = 0; y < s.h; ++y)
        for (int x = 0; x < s.w; ++x)
            if (s.test(x, y) && !s.test(x - 1, y)) {
                int end = x;
                while (s.test(end, y))
                    ++end;
                out[size_t(y)].emplace_back(x, end);
            }
    return out;
}

// Raster no-fit set of b around a: offsets (dx, dy) at which b moved by them shares a cell
// with a. Offsets outside the window [x0, x0 + w) x [y0, y0 + h) never collide.
struct Offsets
{
    int                  x0 = 0, y0 = 0, w = 0, h = 0;
    std::vector<uint8_t> hit;

    bool blocked(int dx, int dy) const
    {
        dx -= x0, dy -= y0;
        return dx >= 0 && dy >= 0 && dx < w && dy < h && hit[size_t(dy) * w + dx];
    }
    bool touching(int dx, int dy) const
    {
        return !blocked(dx, dy) && (blocked(dx - 1, dy) || blocked(dx + 1, dy) || blocked(dx, dy - 1) || blocked(dx, dy + 1));
    }
};

// Per window row, every pair of runs meeting in it blocks one interval of dx, accumulated in a
// difference array: O((ha + hb) * min(ha, hb) * runs^2 + window).
Offsets collisions(const Runs &a, int wa, const Runs &b, int wb)
{
    const int ha = int(a.size()), hb = int(b.size());
    Offsets   o{-(wb - 1), -(hb - 1), wa + wb - 1, ha + hb - 1, {}};
    o.hit.assign(size_t(o.w) * o.h, 0);
    std::vector<int> diff(size_t(o.w) + 1);
    for (int dy = o.y0; dy < o.y0 + o.h; ++dy) {
        std::fill(diff.begin(), diff.end(), 0);
        for (int ra = std::max(0, dy); ra < std::min(ha, hb + dy); ++ra)
            for (auto [a0, a1] : a[size_t(ra)])
                for (auto [b0, b1] : b[size_t(ra - dy)]) {
                    ++diff[size_t(a0 - b1 + 1 - o.x0)]; // overlap for dx in [a0 - b1 + 1, a1 - b0 - 1]
                    --diff[size_t(a1 - b0 - o.x0)];
                }
        for (int i = 0, run = 0; i < o.w; ++i)
            o.hit[size_t(dy - o.y0) * o.w + i] = (run += diff[size_t(i)]) > 0;
    }
    return o;
}

// Cells shared by a and b moved by (dx, dy).
long shared_cells(const Runs &a, const Runs &b, int dx, int dy)
{
    long n = 0;
    for (int ra = std::max(0, dy); ra < std::min(int(a.size()), int(b.size()) + dy); ++ra)
        for (auto [a0, a1] : a[size_t(ra)])
            for (auto [b0, b1] : b[size_t(ra - dy)])
                n += std::max(0, std::min(a1, b1 + dx) - std::max(a0, b0 + dx));
    return n;
}

struct Vec
{
    long long x = 0, y = 0;
};

long long cross(const Vec &u, const Vec &v) { return u.x * v.y - u.y * v.x; }
long long dot(const Vec &u, const Vec &v) { return u.x * v.x + u.y * v.y; }

// Lagrange-Gauss reduction: the same lattice, spanned by its two shortest independent vectors.
void reduce(Vec &u, Vec &v)
{
    for (;;) {
        if (dot(u, u) > dot(v, v))
            std::swap(u, v);
        // floor(x + 0.5) stops on the tie x = -0.5 that would otherwise cycle with x = 0.5.
        const long long q = (long long) std::floor(double(dot(u, v)) / double(dot(u, u)) + 0.5);
        if (q == 0)
            return;
        v.x -= q * u.x, v.y -= q * u.y;
    }
}

// Calls fn(x, y) for every point m * u + n * v in [x0, x1] x [y0, y1] until it returns false.
template<class Fn> bool for_lattice_points(const Vec &u, const Vec &v, long long x0, long long y0, long long x1, long long y1, Fn fn)
{
    const double d = double(cross(u, v));
    double       m_lo = NoScore, m_hi = -NoScore, n_lo = NoScore, n_hi = -NoScore;
    for (long long px : {x0, x1})
        for (long long py : {y0, y1}) {
            const double m = double(px * v.y - py * v.x) / d, n = double(u.x * py - u.y * px) / d;
            m_lo = std::min(m_lo, m), m_hi = std::max(m_hi, m), n_lo = std::min(n_lo, n), n_hi = std::max(n_hi, n);
        }
    for (long long m = (long long) std::floor(m_lo); m <= (long long) std::ceil(m_hi); ++m)
        for (long long n = (long long) std::floor(n_lo); n <= (long long) std::ceil(n_hi); ++n) {
            const long long x = m * u.x + n * v.x, y = m * u.y + n * v.y;
            if (x >= x0 && x <= x1 && y >= y0 && y <= y1 && !fn(x, y))
                return false;
        }
    return true;
}

// Hermite normal form of the lattice spanned by u and v: the basis (a, 0), (b, c) with a, c > 0.
// The offsets [0, a) x [0, c) represent each of its a * c = det cosets once.
std::pair<long long, long long> coset_box(const Vec &u, const Vec &v)
{
    // Extended Euclid on the y components: s * u.y + t * v.y = g.
    long long g0 = u.y, g1 = v.y, s0 = 1, s1 = 0, t0 = 0, t1 = 1;
    while (g1 != 0) {
        const long long q = g0 / g1;
        g0 -= q * g1, s0 -= q * s1, t0 -= q * t1;
        std::swap(g0, g1), std::swap(s0, s1), std::swap(t0, t1);
    }
    const long long c = std::abs(g0);
    return {std::abs(cross(u, v)) / c, c};
}

// Copies of one shape placed as a unit: one copy, or two interlocked ones, with the lattice
// on which translates of the unit never collide.
struct Cluster
{
    struct Member
    {
        int cand = 0, dx = 0, dy = 0; // candidate, and its tight rect relative to the cluster's
    };
    std::vector<Member>        members;
    std::unique_ptr<HeightMap> map;   // union of the members, when there are several
    Shape                      shape; // union occupancy, searched like a part; its factor is the members' mean
    Vec                        v1, v2;
    long long                  det = 0; // cells per lattice site; 0: no lattice
};

// Densest lattice whose translates of the cluster miss each other, given `self`, the cluster's
// raster no-fit set against itself. Basis vectors are drawn from the MaxBasis shortest offsets at
// which a translate touches the cluster: every locally densest lattice has two independent
// touching vectors. The bounding-box lattice is always valid and is the fallback.
void find_lattice(Cluster &k, const Offsets &self)
{
    const int W = k.shape.w, H = k.shape.h;
    std::vector<Vec> basis;
    for (int dy = 0; dy <= H; ++dy)
        for (int dx = -W; dx <= W; ++dx)
            if ((dy > 0 || dx > 0) && self.touching(dx, dy)) // v and -v span the same lattices
                basis.push_back({dx, dy});
    std::stable_sort(basis.begin(), basis.end(), [](const Vec &a, const Vec &b) { return dot(a, a) < dot(b, b); });
    basis.resize(std::min(basis.size(), MaxBasis));

    k.v1 = {W, 0}, k.v2 = {0, H}, k.det = (long long) W * H;
    for (size_t i = 0; i < basis.size(); ++i)
        for (size_t j = i + 1; j < basis.size(); ++j) {
            const long long det = std::abs(cross(basis[i], basis[j]));
            if (det < k.shape.cells || det >= k.det) // translates are disjoint, so det >= cells
                continue;
            Vec u = basis[i], v = basis[j];
            reduce(u, v);
            if (for_lattice_points(u, v, 1 - W, 1 - H, W - 1, H - 1,
                                   [&](long long x, long long y) { return (x == 0 && y == 0) || !self.blocked(int(x), int(y)); }))
                k.v1 = u, k.v2 = v, k.det = det;
        }
}

// Two copies side by side: candidates, offset of b's tight rect from a's, and how tight it is.
struct PairFit
{
    double area = NoScore; // common bounding box, scaled by the candidates' mean score multiplier
    long   contact = 0;
    int    ca = -1, cb = -1, dx = 0, dy = 0;
    bool   better_than(const PairFit &o) const { return std::make_tuple(area, -contact) < std::make_tuple(o.area, -o.contact); }
};

// Tightest touching offset of b next to a, given their raster no-fit set: smallest common
// bounding box, then most contact.
void best_pair(const Shape &a, const Runs &ra, int ca, const Shape &b, const Runs &rb, int cb, const Offsets &nfp, PairFit &best)
{
    const double factor = 0.5 * (a.factor + b.factor);
    for (int dy = nfp.y0 - 1; dy <= nfp.y0 + nfp.h; ++dy)
        for (int dx = nfp.x0 - 1; dx <= nfp.x0 + nfp.w; ++dx) {
            if (!nfp.touching(dx, dy))
                continue;
            PairFit fit{double(std::max(a.w, dx + b.w) - std::min(0, dx)) * (std::max(a.h, dy + b.h) - std::min(0, dy)) * factor, 0, ca, cb, dx, dy};
            if (fit.area > best.area)
                continue;
            fit.contact = shared_cells(ra, rb, dx - 1, dy) + shared_cells(ra, rb, dx + 1, dy) + shared_cells(ra, rb, dx, dy - 1) +
                          shared_cells(ra, rb, dx, dy + 1);
            if (fit.better_than(best))
                best = fit;
        }
}

// ---------------------------------------------------------------------------
// Placement
// ---------------------------------------------------------------------------

struct Rect
{
    int  x0 = 0, y0 = 0, x1 = 0, y1 = 0; // max exclusive
    bool empty() const { return x1 <= x0; }
    long area() const { return long(x1 - x0) * (y1 - y0); }
    Rect united(const Rect &o) const
    {
        return empty() ? o : Rect{std::min(x0, o.x0), std::min(y0, o.y0), std::max(x1, o.x1), std::max(y1, o.y1)};
    }
};

// Free regions that the parts placed by a run enclose: the 4-connected components of free cells
// inside their bounding box that do not reach its border. With stacking, cells with room under
// them count as free (see Board::floor), so that a hole under a table top is still a pocket;
// Board::fits() decides what fits there.
struct Pockets
{
    struct Run
    {
        int x0, x1, id; // [x0, x1) of a free run, its component
    };
    struct Region
    {
        Rect box;
        int  cells = 0;
        bool open  = false; // reaches the bounding box's border
    };
    Rect                y_range;    // the bounding box scanned
    std::vector<size_t> row_begin;  // per row of it, index of its first run; one more at the end
    std::vector<Run>    runs;
    std::vector<Region> regions;    // connected components, indexed by Run::id

    // Free runs per row, joined to the runs they overlap in the row below by a union-find.
    Pockets(const Board &b, const Rect &bb) : y_range(bb)
    {
        std::vector<int> parent;
        auto find = [&](int i) {
            while (parent[size_t(i)] != i)
                i = parent[size_t(i)] = parent[size_t(parent[size_t(i)])];
            return i;
        };
        for (int y = bb.y0; y < bb.y1; ++y) {
            row_begin.push_back(runs.size());
            const size_t below = row_begin.size() > 1 ? row_begin[row_begin.size() - 2] : runs.size();
            size_t       k     = below;
            for (int x = b.next(y, bb.x0, bb.x1, false, true); x < bb.x1; x = b.next(y, x, bb.x1, false, true)) {
                const int end = b.next(y, x, bb.x1, true, true), id = int(parent.size());
                parent.push_back(id);
                runs.push_back({x, end, id});
                for (; k < row_begin.back() && runs[k].x1 <= x; ++k) {}
                for (size_t j = k; j < row_begin.back() && runs[j].x0 < end; ++j)
                    parent[size_t(find(id))] = find(runs[j].id);
                x = end;
            }
        }
        row_begin.push_back(runs.size());
        std::vector<int> region(parent.size(), -1); // per root
        for (size_t r = 0; r + 1 < row_begin.size(); ++r)
            for (size_t i = row_begin[r]; i < row_begin[r + 1]; ++i) {
                Run      &run  = runs[i];
                const int y    = bb.y0 + int(r);
                int      &root = region[size_t(find(run.id))];
                if (root < 0)
                    root = int(regions.size()), regions.emplace_back();
                run.id    = root;
                Region &g = regions[size_t(root)];
                g.box     = g.box.united({run.x0, y, run.x1, y + 1});
                g.cells += run.x1 - run.x0;
                g.open |= run.x0 == bb.x0 || run.x1 == bb.x1 || y == bb.y0 || y == bb.y1 - 1;
            }
    }

    // Free cells in the pockets: space the pile encloses and leaves unused.
    long enclosed() const
    {
        long n = 0;
        for (const Region &g : regions)
            n += g.open ? 0 : g.cells;
        return n;
    }

    // Region holding free cell (x, y) of the scanned box, free as Board::next() reads it at the floor.
    int region_at(int x, int y) const
    {
        const size_t r = size_t(y - y_range.y0);
        auto it = std::upper_bound(runs.begin() + long(row_begin[r]), runs.begin() + long(row_begin[r + 1]), x,
                                   [](int v, const Run &run) { return v < run.x0; });
        return runs[size_t(it - runs.begin()) - 1].id;
    }
};

// Where a part landed: candidate index and bed cell of its shape's tight rect.
struct Spot { int cand = -1, x = 0, y = 0; };

struct Plate
{
    int              index = 0;
    int              group = -1; // compat group of the parts on it: -1 while none, Mixed for several
    Board            board;
    Rect             bbox;   // of the parts placed by this run
    std::vector<int> placed; // part indices
    std::shared_ptr<const Pockets> pockets; // of the current board, built on demand
};

struct Hit
{
    double score = NoScore;
    double dist  = 0.; // squared distance from the shape's centre to the anchor, cells
    int    y = 0, x = 0;
    bool   better_than(const Hit &o) const { return std::tie(score, dist, y, x) < std::tie(o.score, o.dist, o.y, o.x); }
};

struct Pass
{
    std::vector<Spot>  spots; // per part
    std::vector<Plate> plates;
    int                unplaced = 0, plates_used = 0;
    std::vector<int>   fill;      // parts placed by this run per plate, without trailing empty plates
    double             cost = 0.; // per plate, bbox area times its parts' cell-weighted score multiplier,
                                  // plus the free area left in its pockets
    bool               better_than(const Pass &o) const
    {
        if (unplaced != o.unplaced) return unplaced < o.unplaced;
        if (plates_used != o.plates_used) return plates_used < o.plates_used;
        // Filling the earlier plates first beats spreading the parts more compactly over them.
        if (fill != o.fill) return std::lexicographical_compare(o.fill.begin(), o.fill.end(), fill.begin(), fill.end());
        return cost < o.cost;
    }
};

// Half extent, about anchor coordinate a, of the smallest anchor-centred interval holding [t0, t1].
double half(double a, double t0, double t1) { return std::max(a - t0, t1 - a); }

// Score term along one axis of a part at [t, t + size) joining a pile at [lo, hi) (none when
// lo >= hi): twice the half extent about anchor coordinate a when growing from the anchor, else
// the plain extent. Convex in t, so it never shrinks moving away from its minimum.
double axis_term(bool centred, double a, int t, int size, int lo, int hi)
{
    lo = lo < hi ? std::min(lo, t) : t, hi = std::max(hi, t + size);
    return centred ? 2. * half(a, lo, hi) : double(hi - lo);
}

// Position t in [0, bed - size] nearest the one centred on a among those minimising axis_term().
int axis_start(bool centred, double a, int size, int lo, int hi, int bed)
{
    int t = int(std::lround(a - 0.5 * size));
    if (!centred && lo < hi)
        t = std::clamp(t, std::min(lo, hi - size), std::max(lo, hi - size));
    return std::clamp(t, 0, bed - size);
}

class Arranger
{
public:
    Arranger(const std::vector<Part> &parts, const std::vector<Obstacle> &obstacles, const Params &p)
        : m_parts(parts), m_obstacles(obstacles), m_p(p)
        , m_parallel_for(p.parallel_for ? p.parallel_for : [](size_t n, const std::function<void(size_t)> &fn) { for (size_t i = 0; i < n; ++i) fn(i); })
    {
        m_w = p.grid_cols();
        m_h = p.grid_rows();
        m_centre = p.align_x >= 0.;
        m_anchor_x = m_centre ? std::clamp(p.align_x, 0., 1.) * m_w : 0.;
        m_anchor_y = m_centre ? std::clamp(p.align_y, 0., 1.) * m_h : 0.;
        m_lambda = ContactMM / p.cell_mm;
        m_shapes.resize(parts.size());
        for (size_t i = 0; i < parts.size(); ++i)
            for (const Candidate &c : parts[i].candidates) {
                m_shapes[i].emplace_back(make_shape(c.map, p.max_height, p.vertical_gap));
                m_shapes[i].back().factor = 1. + std::max(0., c.penalty);
            }
        for (const Obstacle &o : obstacles)
            m_obstacle_plates = std::max(m_obstacle_plates, o.plate + 1);

        // Copies: parts sharing a shape key and a compat group, two or more of them.
        std::map<std::pair<int, int>, std::vector<int>> copies;
        for (size_t i = 0; i < parts.size(); ++i)
            if (parts[i].shape_key >= 0)
                copies[{parts[i].shape_key, parts[i].compat_group}].push_back(int(i));
        m_group.assign(parts.size(), -1);
        for (auto &[key, members] : copies)
            if (members.size() >= 2) {
                for (int i : members)
                    m_group[size_t(i)] = int(m_groups.size());
                m_groups.emplace_back().parts = std::move(members);
            }
    }

    std::vector<Placement> run(Stats *stats)
    {
        build_clusters();
        // A pass per distinct part ordering, then the first ordering again with copies seeded on
        // lattices. Growing from the centre pins the first part there, which loses parts that
        // only fit pushed into opposite corners, so a pass grown from the min corner follows
        // when the others leave parts unplaced or need more than one plate. Equal results keep
        // the earlier pass.
        struct Strategy { size_t order; bool seeded, corner; };
        std::vector<std::vector<int>> orders;
        std::vector<Strategy>         strategies;
        for (int k = 0; k < std::clamp(m_p.orderings, 1, 6); ++k)
            if (std::vector<int> o = order(k); std::find(orders.begin(), orders.end(), o) == orders.end())
                orders.push_back(std::move(o)), strategies.push_back({orders.size() - 1, false, !m_centre});
        if (std::any_of(m_groups.begin(), m_groups.end(), [](const Group &g) { return !g.clusters.empty(); }))
            strategies.push_back({0, true, !m_centre});
        if (m_centre)
            strategies.push_back({0, false, true});

        Pass best;
        best.spots.resize(m_parts.size()); // all unplaced, should a stop come before any pass
        int  best_strategy = -1, done = 0;
        const int total = int(strategies.size() * m_parts.size());
        for (size_t k = 0; k < strategies.size() && !stopped(); ++k) {
            const Strategy &s = strategies[k];
            if (m_centre && s.corner && best.unplaced == 0 && best.plates_used <= 1)
                break;
            m_ax = s.corner ? 0. : m_anchor_x, m_ay = s.corner ? 0. : m_anchor_y;
            Pass pass = greedy(orders[s.order], s.seeded, done, total);
            if (best_strategy < 0 || pass.better_than(best))
                best = std::move(pass), best_strategy = int(k);
        }
        if (best_strategy < 0)
            best.unplaced = int(m_parts.size());
        if (m_p.progress && !m_stopped && done < total)
            m_p.progress(total, total);
        for (Plate &plate : best.plates)
            align(plate, best.spots);

        std::vector<Placement> out(m_parts.size());
        for (size_t i = 0; i < m_parts.size(); ++i) {
            out[i].id = m_parts[i].id;
            const Spot &spot = best.spots[i];
            if (spot.cand < 0)
                continue;
            const Candidate &c = m_parts[i].candidates[spot.cand];
            const Shape     &s = m_shapes[i][spot.cand];
            // Bed cell of the map's cell (0, 0) gives the reference point's bed position.
            out[i].x           = (spot.x - s.x0) * m_p.cell_mm - c.map.origin_x;
            out[i].y           = (spot.y - s.y0) * m_p.cell_mm - c.map.origin_y;
            out[i].rotation_z  = c.rotation_z;
            out[i].orientation = c.orientation;
        }
        for (const Plate &plate : best.plates)
            for (int i : plate.placed)
                out[i].plate = plate.index;
        if (stats)
            *stats = {best.plates_used, best.unplaced};
        return out;
    }

private:
    struct Group
    {
        std::vector<int>     parts;    // identical copies, input order
        std::vector<Cluster> clusters; // with a lattice each, densest per copy first
    };

    // Candidate of part i turned half a turn from candidate c in the same orientation, or -1.
    int half_turn(int i, int c) const
    {
        const std::vector<Candidate> &cs = m_parts[size_t(i)].candidates;
        for (size_t k = 0; k < cs.size(); ++k)
            if (m_shapes[size_t(i)][k].usable && cs[k].orientation == cs[size_t(c)].orientation &&
                std::abs(std::remainder(cs[k].rotation_z - cs[size_t(c)].rotation_z - Pi, 2. * Pi)) < 1e-6)
                return int(k);
        return -1;
    }

    // Candidates of group g that lattices are built from: the MaxLatticeCandidates usable ones that
    // fill their rect best (weighted by penalty), one per distinct shape, in candidate order, which
    // breaks ties between equally dense lattices.
    std::vector<int> lattice_candidates(const Group &g) const
    {
        const std::vector<Shape> &ss = m_shapes[size_t(g.parts.front())];
        std::vector<int>          cs;
        for (size_t c = 0; c < ss.size(); ++c)
            if (ss[c].usable)
                cs.push_back(int(c));
        auto fill = [&](int c) { const Shape &s = ss[size_t(c)]; return s.cells / (double(s.w) * s.h * s.factor); };
        std::stable_sort(cs.begin(), cs.end(), [&](int a, int b) { return fill(a) > fill(b); });
        // A symmetric part repeats its shape at several angles; one of them is enough.
        std::vector<int> picked;
        for (int c : cs)
            if (picked.size() < MaxLatticeCandidates && std::none_of(picked.begin(), picked.end(), [&](int p) {
                    const Shape &a = ss[size_t(p)], &b = ss[size_t(c)];
                    return a.w == b.w && a.h == b.h && a.bits == b.bits && a.factor == b.factor;
                }))
                picked.push_back(c);
        std::sort(picked.begin(), picked.end());
        return picked;
    }

    // Per group of copies: a cluster of one copy in each of its lattice candidates, and one of the
    // tightest pair of such a candidate with itself or its half turn, each with its densest lattice.
    // Candidates are tasks reduced in task order, so the result does not depend on scheduling.
    void build_clusters()
    {
        struct Task { size_t g; int c; };
        std::vector<Task> tasks;
        for (size_t g = 0; g < m_groups.size(); ++g)
            for (int c : lattice_candidates(m_groups[g]))
                tasks.push_back({g, c});
        std::vector<Cluster> singles(tasks.size());
        std::vector<PairFit> pairs(tasks.size());
        m_parallel_for(tasks.size(), [&](size_t t) {
            if (stopped())
                return;
            const int                 i  = m_groups[tasks[t].g].parts.front(), c = tasks[t].c;
            const std::vector<Shape> &ss = m_shapes[size_t(i)];
            const Shape              &s  = ss[size_t(c)];
            singles[t].members = {{c, 0, 0}};
            singles[t].shape   = s;
            const Runs    r    = runs_of(s);
            const Offsets self = collisions(r, s.w, r, s.w);
            find_lattice(singles[t], self);
            best_pair(s, r, c, s, r, c, self, pairs[t]);
            if (const int h = half_turn(i, c); h >= 0) {
                const Shape &sh = ss[size_t(h)];
                const Runs   rh = runs_of(sh);
                best_pair(s, r, c, sh, rh, h, collisions(r, s.w, rh, sh.w), pairs[t]);
            }
        });
        std::vector<PairFit> best_pairs(m_groups.size());
        for (size_t t = 0; t < tasks.size(); ++t) {
            if (pairs[t].ca >= 0 && pairs[t].better_than(best_pairs[tasks[t].g]))
                best_pairs[tasks[t].g] = pairs[t];
            if (singles[t].det <= 0)
                continue;
            // The densest lattice need not hold the most copies on a finite bed (discs: hexagonal
            // rows of 3, 2, 3 where a square grid takes 3 by 3), so the grid of tight rects is
            // offered too; pick_lattice() takes it only where it holds more.
            const Shape &s = singles[t].shape;
            if (singles[t].det < (long long) s.w * s.h) {
                Cluster grid;
                grid.members = singles[t].members;
                grid.shape   = s;
                grid.v1 = {s.w, 0}, grid.v2 = {0, s.h}, grid.det = (long long) s.w * s.h;
                m_groups[tasks[t].g].clusters.push_back(std::move(grid));
            }
            m_groups[tasks[t].g].clusters.push_back(std::move(singles[t]));
        }
        std::vector<Cluster> pair_clusters(m_groups.size());
        m_parallel_for(m_groups.size(), [&](size_t g) {
            if (best_pairs[g].ca >= 0 && !stopped())
                make_pair(m_groups[g].parts.front(), best_pairs[g], pair_clusters[g]);
        });
        for (size_t g = 0; g < m_groups.size(); ++g) {
            std::vector<Cluster> &cs = m_groups[g].clusters;
            if (pair_clusters[g].det > 0)
                cs.push_back(std::move(pair_clusters[g]));
            // Densest per copy first, a penalised pose's lattice counting as that much less dense.
            std::stable_sort(cs.begin(), cs.end(), [](const Cluster &a, const Cluster &b) {
                return double(a.det) * a.shape.factor * double(b.members.size()) < double(b.det) * b.shape.factor * double(a.members.size());
            });
        }
    }

    // Builds the union of the two copies of part i.
    void make_pair(int i, const PairFit &fit, Cluster &k) const
    {
        const std::vector<Shape> &ss = m_shapes[size_t(i)];
        const Shape &a = ss[size_t(fit.ca)], &b = ss[size_t(fit.cb)];
        const int    ax = std::max(0, -fit.dx), ay = std::max(0, -fit.dy);
        k.members = {{fit.ca, ax, ay}, {fit.cb, ax + fit.dx, ay + fit.dy}};
        k.map     = std::make_unique<HeightMap>(std::max(a.w, fit.dx + b.w) - std::min(0, fit.dx),
                                                std::max(a.h, fit.dy + b.h) - std::min(0, fit.dy), a.map->cell_mm);
        for (const Cluster::Member &m : k.members) {
            const Shape &s = ss[size_t(m.cand)];
            for (int y = 0; y < s.h; ++y)
                for (int x = 0; x < s.w; ++x)
                    if (const Span &c = s.map->at(s.x0 + x, s.y0 + y); !c.empty())
                        k.map->at(m.dx + x, m.dy + y) = c; // the members share no cell
        }
        k.shape        = make_shape(*k.map, m_p.max_height, m_p.vertical_gap);
        k.shape.factor = 0.5 * (a.factor + b.factor);
        const Runs r = runs_of(k.shape);
        find_lattice(k, collisions(r, k.shape.w, r, k.shape.w));
    }

    // Part order for greedy pass k: priority first, then one size key per pass. Ties fall to the
    // other size keys, then the shape key and id, so that the order does not follow the input
    // order (the selection order in the GUI), which moves mixed piles by several percent.
    std::vector<int> order(int k) const
    {
        auto key = [&](int i, int j) -> double {
            const std::vector<Shape> &ss = m_shapes[i];
            if (ss.empty())
                return 0.;
            const Shape &s = ss.front();
            switch (j) {
            // With stacking, a part that leaves room below it comes after the parts that could
            // fill that room, so it can be placed over them.
            case 0: return m_p.allow_stacking ? s.floor_cells : s.cells;
            case 1: return s.top;
            case 2: return std::max(s.w, s.h);
            case 3: return s.volume;
            case 4: return s.w + s.h;
            default: return double(s.w) * s.h;
            }
        };
        std::vector<int> idx(m_parts.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) {
            if (m_parts[a].priority != m_parts[b].priority)
                return m_parts[a].priority > m_parts[b].priority;
            for (int j : {k, 0, 1, 2, 3, 4, 5})
                if (const double ka = key(a, j), kb = key(b, j); ka != kb)
                    return ka > kb;
            return std::make_pair(m_parts[a].shape_key, m_parts[a].id) < std::make_pair(m_parts[b].shape_key, m_parts[b].id);
        });
        return idx;
    }

    // Bed mask and obstacles of plate `index`.
    Board fixed_board(int index) const
    {
        const Span full{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max()};
        Board      b(m_w, m_h, m_p.allow_stacking, m_p.vertical_gap);
        for (int y = 0; y < m_h; ++y)
            for (int x = 0; x < m_w; ++x)
                if (size_t i = size_t(y) * m_w + x; i < m_p.bed_mask.size() && m_p.bed_mask[i])
                    b.add(x, y, full);
        const double eps = 1e-6;
        for (const Obstacle &o : m_obstacles) {
            if (o.plate != index && o.plate >= 0)
                continue;
            const HeightMap &m = o.map;
            // Resample: every bed cell overlapped by an obstacle cell takes its span.
            for (int y = 0; y < m.height; ++y)
                for (int x = 0; x < m.width; ++x) {
                    const Span &s = m.at(x, y);
                    if (s.empty())
                        continue;
                    int bx0 = std::max(0, int(std::floor((m.origin_x + x * m.cell_mm) / m_p.cell_mm + eps)));
                    int by0 = std::max(0, int(std::floor((m.origin_y + y * m.cell_mm) / m_p.cell_mm + eps)));
                    int bx1 = std::min(m_w, int(std::ceil((m.origin_x + (x + 1) * m.cell_mm) / m_p.cell_mm - eps)));
                    int by1 = std::min(m_h, int(std::ceil((m.origin_y + (y + 1) * m.cell_mm) / m_p.cell_mm - eps)));
                    for (int by = by0; by < by1; ++by)
                        for (int bx = bx0; bx < bx1; ++bx)
                            b.add(bx, by, s);
                }
        }
        b.update_sums(0, m_h);
        return b;
    }

    // The board is built by open(), on first use: obstacles on every plate put all
    // plates in play, and most of them never receive a part.
    Plate make_plate(int index) const
    {
        int group = -1;
        for (const Obstacle &o : m_obstacles)
            if (o.plate == index && o.group >= 0)
                group = group == -1 || group == o.group ? o.group : Mixed;
        return Plate{index, group, Board(0, 0), {}, {}};
    }

    void open(Plate &plate) const
    {
        if (plate.board.w != m_w)
            plate.board = fixed_board(plate.index);
    }

    // Candidate shapes of part i, the options search() and pocket_search() choose from.
    std::vector<const Shape *> shapes_of(int i) const
    {
        std::vector<const Shape *> out;
        for (const Shape &s : m_shapes[size_t(i)])
            out.push_back(&s);
        return out;
    }

    // Best position on `plate` over `shapes`: the area of the bounding box of this run's parts on
    // the plate and this one, times the shape's factor, minus ContactMM per mm of its outline that
    // touches occupied cells or the bed edge; with `centred`, the area of the smallest
    // anchor-centred rectangle holding them instead. Ties go to the position nearest the anchor,
    // then the lowest y, x and shape. Contact is at most the halo size and at most the occupied
    // cells around the shape's rect plus its wall contact, and each axis term of the area is convex
    // (see axis_term()), so rows and columns are visited outward from their best position and a
    // side stops once nothing further out can win; that pruning never changes the result. Shapes
    // and row bands are tasks reduced in task order, so the result does not depend on scheduling.
    std::pair<int, Hit> search(const std::vector<const Shape *> &shapes, const Plate &plate, bool centred) const
    {
        const int        n_bands = std::clamp(m_h / 32, 1, 8);
        std::vector<Hit> hits(shapes.size() * n_bands);
        std::atomic<double> shared{NoScore}; // best score any task has found; prunes all of them
        m_parallel_for(hits.size(), [&](size_t task) {
            const Shape &s    = *shapes[task / n_bands];
            const double f    = s.factor;
            const int    band = int(task % n_bands);
            // Without stacking every cell of the shape needs a free bed cell.
            if (!s.usable || s.w > m_w || s.h > m_h || (!m_p.allow_stacking && s.cells > m_w * m_h - plate.board.filled))
                return;
            const Rect  &bb    = plate.bbox;
            const double bonus = m_lambda * s.halo_cells;
            Hit          best;
            auto         limit = [&] { return std::min(best.score, shared.load(std::memory_order_relaxed)); };
            const int walls_all = s.edge[0] + s.edge[1] + s.edge[2] + s.edge[3];
            auto x_term = [&](int tx) { return axis_term(centred, m_ax, tx, s.w, bb.x0, bb.x1); };
            auto y_term = [&](int ty) { return axis_term(centred, m_ay, ty, s.h, bb.y0, bb.y1); };
            // Columns to advance from (tx, ty) along a row, `dir` +1 or -1 away from the start; 0
            // once nothing further on can win. The area only grows that way, and once the board is
            // empty from here on, contact can only come from the bed edges.
            auto try_at = [&](int tx, int ty, int dir) {
                const double env = x_term(tx) * y_term(ty) * f;
                if (env - bonus > limit())
                    return 0;
                if (env - m_lambda * walls_all > limit() &&
                    plate.board.count(dir > 0 ? tx - 1 : 0, ty - 1, dir > 0 ? m_w : tx + s.w + 1, ty + s.h + 1) == 0)
                    return 0;
                // Without stacking, occupied cells inside the rect must fit in the shape's empty
                // ones. A step changes that count by at most s.h, so too many rule out the next
                // positions as well.
                if (const int excess = m_p.allow_stacking ? 0 : plate.board.count(tx, ty, tx + s.w, ty + s.h) - (s.w * s.h - s.cells); excess > 0)
                    return 1 + (excess - 1) / s.h;
                const int walls = (tx == 0 ? s.edge[0] : 0) + (tx + s.w == m_w ? s.edge[1] : 0) + (ty == 0 ? s.edge[2] : 0) +
                                  (ty + s.h == m_h ? s.edge[3] : 0);
                const int around = plate.board.count(tx - 1, ty - 1, tx + s.w + 1, ty + s.h + 1);
                if (env - m_lambda * std::min(s.halo_cells, around + walls) > limit() || (around > 0 && !plate.board.fits(s, tx, ty, m_p)))
                    return 1;
                const double cx = tx + 0.5 * s.w - m_ax, cy = ty + 0.5 * s.h - m_ay;
                const Hit    hit{env - m_lambda * (walls + (around > 0 ? plate.board.contact(s, tx, ty) : 0)), cx * cx + cy * cy, ty, tx};
                if (hit.better_than(best)) {
                    best = hit;
                    for (double cur = shared.load(); hit.score < cur && !shared.compare_exchange_weak(cur, hit.score);) {}
                }
                return 1;
            };
            // Rows at distance d = band, band + n_bands, ... above and below the starting one; each
            // side ends at the first row whose area alone loses. Columns likewise.
            const int    r0 = axis_start(centred, m_ay, s.h, bb.y0, bb.y1, m_h);
            const int    c0 = axis_start(centred, m_ax, s.w, bb.x0, bb.x1, m_w);
            const double x_min = x_term(c0);
            bool         open[2] = {true, true};
            for (int d = band; open[0] || open[1]; d += n_bands)
                for (int side = 0; side < 2; ++side) {
                    const int ty = side == 0 ? r0 + d : r0 - d;
                    if (!open[side] || (d == 0 && side == 1))
                        continue;
                    const double lb = x_min * y_term(ty) * f;
                    if (ty < 0 || ty > m_h - s.h || lb - bonus > limit() ||
                        (lb - m_lambda * walls_all > limit() &&
                         plate.board.count(0, side == 0 ? ty - 1 : 0, m_w, side == 0 ? m_h : ty + s.h + 1) == 0)) {
                        open[side] = false;
                        continue;
                    }
                    for (int tx = c0, step; tx <= m_w - s.w && (step = try_at(tx, ty, 1)) > 0; tx += step) {}
                    for (int tx = c0 - 1, step; tx >= 0 && (step = try_at(tx, ty, -1)) > 0; tx -= step) {}
                }
            hits[task] = best;
        });
        int best = -1;
        for (size_t t = 0; t < hits.size(); ++t)
            if (hits[t].score < NoScore && (best < 0 || hits[t].better_than(hits[best])))
                best = int(t);
        return {best < 0 ? -1 : best / n_bands, best < 0 ? Hit{} : hits[best]};
    }

    // Best position over `shapes` inside a pocket of `plate` (see Pockets), or -1. The score is the
    // pocket's area, scaled by the shape's factor, plus the cells the shape claims: the part goes
    // to the smallest pocket that holds it, in the pose that claims the least space, nearest the
    // pocket's min corner, which leaves the pocket's free space in one piece. Contact plays no
    // part: the staircase edge of a part turned 45 degrees touches more cells than a straight one,
    // which would turn small parts askew in round holes, where fewer of them fit. A pocket is
    // scanned from its min corner and stops once a position is farther from it than a fit as good.
    std::pair<int, Hit> pocket_search(const std::vector<const Shape *> &shapes, Plate &plate) const
    {
        if (plate.bbox.empty())
            return {-1, Hit{}};
        if (!plate.pockets)
            plate.pockets = std::make_shared<const Pockets>(plate.board, plate.bbox);
        const Pockets &pk = *plate.pockets;
        auto holds = [](const Pockets::Region &g, const Shape &s) {
            return !g.open && g.cells >= s.cells && g.box.x1 - g.box.x0 >= s.w && g.box.y1 - g.box.y0 >= s.h;
        };
        if (std::none_of(pk.regions.begin(), pk.regions.end(), [&](const Pockets::Region &g) {
                return std::any_of(shapes.begin(), shapes.end(), [&](const Shape *s) { return s->usable && holds(g, *s); });
            }))
            return {-1, Hit{}};
        std::vector<Hit> hits(shapes.size());
        m_parallel_for(shapes.size(), [&](size_t o) {
            const Shape &s = *shapes[o];
            if (!s.usable || stopped())
                return;
            // The shape's first cell, in its bottom row: a connected shape lies in that cell's pocket.
            int first = 0;
            while (!s.test(first, 0))
                ++first;
            Hit best;
            for (const Pockets::Region &g : pk.regions) {
                if (!holds(g, s))
                    continue;
                const int    id    = int(&g - pk.regions.data());
                const double score = g.cells * s.factor + s.cells;
                if (score > best.score)
                    continue;
                // The distance from the corner only grows along a row and from row to row.
                auto loses = [&](double dist) { return score == best.score && dist > best.dist; };
                for (int ty = g.box.y0; ty <= g.box.y1 - s.h; ++ty) {
                    const double cy = ty - g.box.y0;
                    if (loses(cy * cy))
                        break;
                    for (int tx = g.box.x0; tx <= g.box.x1 - s.w; ++tx) {
                        const double cx = tx - g.box.x0;
                        if (loses(cx * cx + cy * cy))
                            break;
                        if (plate.board.next(ty, tx + first, tx + first + 1, true, true) == tx + first ||
                            pk.region_at(tx + first, ty) != id ||
                            (plate.board.count(tx, ty, tx + s.w, ty + s.h) > 0 && !plate.board.fits(s, tx, ty, m_p)))
                            continue;
                        if (const Hit hit{score, cx * cx + cy * cy, ty, tx}; hit.better_than(best))
                            best = hit;
                    }
                }
            }
            hits[o] = best;
        });
        int best = -1;
        for (size_t o = 0; o < hits.size(); ++o)
            if (hits[o].score < NoScore && (best < 0 || hits[o].better_than(hits[size_t(best)])))
                best = int(o);
        return {best, best < 0 ? Hit{} : hits[size_t(best)]};
    }

    void place(int i, int cand, int x, int y, Plate &plate, Pass &pass) const
    {
        plate.pockets.reset();
        const Shape &s = m_shapes[i][cand];
        plate.board.place(s, x, y);
        plate.bbox  = plate.bbox.united({x, y, x + s.w, y + s.h});
        plate.group = m_parts[i].compat_group;
        plate.placed.push_back(i);
        pass.spots[i] = {cand, x, y};
    }

    // Copies of part i's cluster `k` that fit on `plate` with the lattice through bed cell (ox, oy),
    // at most `cap`. Sites never collide with each other, so each member is tested on its own.
    int lattice_room(int i, const Cluster &k, const Plate &plate, long long ox, long long oy, int cap) const
    {
        int n = 0;
        for_lattice_points(k.v1, k.v2, 1 - k.shape.w - ox, 1 - k.shape.h - oy, m_w - 1 - ox, m_h - 1 - oy, [&](long long x, long long y) {
            for (const Cluster::Member &m : k.members)
                n += plate.board.fits_in_bed(m_shapes[size_t(i)][size_t(m.cand)], int(ox + x) + m.dx, int(oy + y) + m.dy, m_p);
            return n < cap;
        });
        return std::min(n, cap);
    }

    // Where to lay a group's lattice on `plate` for `cap` copies of part i. Each cluster's lattice is
    // tried through the position where the whole cluster scores best (see search() with
    // `centred`), densest first; when that does not hold all copies (the copies fill the plate, or
    // the centred site leaves too little room around it), its cosets, up to MaxCosets of them, are
    // counted, as the lattice phase decides how much the bed edges cut off. Of the sites that hold
    // all copies, the one whose copies leave the pile the smallest rectangle wins, the densest
    // cluster and then the earliest tried on a tie: on a finite bed a less dense lattice can lay
    // the copies more compactly. Failing that, the site holding the most copies wins.
    struct LatticeSite
    {
        const Cluster *cluster = nullptr;
        long long      x = 0, y = 0; // a lattice point, bed cells
    };
    LatticeSite pick_lattice(int i, const Group &g, const Plate &plate, int cap) const
    {
        LatticeSite best;
        int         room = 0;
        long        area = 0; // of the pile with best's copies, once best holds them all
        auto offer = [&](const LatticeSite &site, int n, long a) {
            if (n == cap ? room < cap || a < area : n > room)
                best = site, room = n, area = a;
        };
        for (const Cluster &k : g.clusters) {
            const auto [o, at] = search({&k.shape}, plate, true);
            if (o < 0)
                continue;
            const LatticeSite site{&k, at.x, at.y};
            const int         n = lattice_room(i, k, plate, at.x, at.y, cap);
            offer(site, n, n == cap ? pile_area(i, site, plate, cap) : 0);
            if (n == cap)
                continue;
            const std::pair<long long, long long> box = coset_box(k.v1, k.v2);
            const long long                       a = box.first, c = box.second;
            // Large clusters have more cosets than are worth counting; neighbouring ones fit
            // about as many copies, so every stride-th one stands in for its neighbours.
            const long long   stride = (a * c + MaxCosets - 1) / MaxCosets;
            std::vector<int>  counts(size_t((a * c + stride - 1) / stride));
            std::vector<long> areas(counts.size());
            auto coset = [&](size_t t) { return LatticeSite{&k, (long long) t * stride % a, (long long) t * stride / a}; };
            m_parallel_for(counts.size(), [&](size_t t) {
                if (stopped())
                    return;
                const LatticeSite at = coset(t);
                counts[t]            = lattice_room(i, k, plate, at.x, at.y, cap);
                if (counts[t] == cap)
                    areas[t] = pile_area(i, at, plate, cap);
            });
            for (size_t t = 0; t < counts.size(); ++t)
                offer(coset(t), counts[t], areas[t]);
        }
        return best;
    }

    // Lays up to `cap` copies of part i on the lattice sites of `at` nearest the anchor, by the
    // square rings the anchor-centred area grows in, calling lay(cand, x, y) for each. Each copy
    // goes where its own map fits, so sites cut by the bed edge, obstacles or earlier parts still
    // take what fits there. Sites never collide, so a copy fits whether or not the ones before it
    // were placed. Returns the copies laid.
    template<class Lay> int lay_sites(int i, const LatticeSite &at, int cap, const Plate &plate, Lay lay) const
    {
        const Cluster &k = *at.cluster;
        struct Site { double ring, dist; int y, x; };
        std::vector<Site> sites;
        const int         W = k.shape.w, H = k.shape.h;
        for_lattice_points(k.v1, k.v2, 1 - W - at.x, 1 - H - at.y, m_w - 1 - at.x, m_h - 1 - at.y, [&](long long dx, long long dy) {
            const int    x = int(at.x + dx), y = int(at.y + dy);
            const double cx = x + 0.5 * W - m_ax, cy = y + 0.5 * H - m_ay;
            sites.push_back({std::max(half(m_ax, x, x + W), half(m_ay, y, y + H)), cx * cx + cy * cy, y, x});
            return true;
        });
        std::sort(sites.begin(), sites.end(), [](const Site &a, const Site &b) {
            return std::tie(a.ring, a.dist, a.y, a.x) < std::tie(b.ring, b.dist, b.y, b.x);
        });
        int n = 0;
        for (const Site &site : sites)
            for (const Cluster::Member &m : k.members) {
                if (n == cap)
                    return n;
                const Shape &s = m_shapes[size_t(i)][size_t(m.cand)];
                const int    x = site.x + m.dx, y = site.y + m.dy;
                if (x >= 0 && y >= 0 && x + s.w <= m_w && y + s.h <= m_h && plate.board.fits(s, x, y, m_p))
                    lay(m.cand, x, y), ++n;
            }
        return n;
    }

    // Area of the plate's pile once `cap` copies of part i are laid at `at`, in cells.
    long pile_area(int i, const LatticeSite &at, const Plate &plate, int cap) const
    {
        Rect bb = plate.bbox;
        lay_sites(i, at, cap, plate, [&](int cand, int x, int y) {
            const Shape &s = m_shapes[size_t(i)][size_t(cand)];
            bb             = bb.united({x, y, x + s.w, y + s.h});
        });
        return bb.area();
    }

    // Seeds the copies of part i not placed yet, i first, on one of its group's cluster lattices.
    // Returns the number of copies placed.
    int seed(int i, const Group &g, Plate &plate, Pass &pass, const std::vector<int> &rank) const
    {
        std::vector<int> todo;
        for (int j : g.parts)
            if (pass.spots[size_t(j)].cand < 0 && rank[size_t(j)] >= rank[size_t(i)])
                todo.push_back(j);
        std::sort(todo.begin(), todo.end(), [&](int a, int b) { return rank[size_t(a)] < rank[size_t(b)]; });
        const LatticeSite at = pick_lattice(i, g, plate, int(todo.size()));
        if (!at.cluster)
            return 0;
        // Copies of one group share their shapes, so part i's stand in for each copy's.
        size_t next = 0;
        return lay_sites(i, at, int(todo.size()), plate, [&](int cand, int x, int y) { place(todo[next++], cand, x, y, plate, pass); });
    }

    // Polls the caller's stop request from any thread; once seen, it stays set.
    bool stopped() const
    {
        if (!m_stopped && m_p.should_stop && m_p.should_stop())
            m_stopped = true;
        return m_stopped;
    }

    bool accepts(const Plate &plate, int i) const { return plate.group == -1 || plate.group == m_parts[i].compat_group; }

    // First-fit greedy over `order`: the first open plate with room wins, else a new plate. With
    // seeding, the first copy of a group to reach a plate lays the group's lattice there. A part
    // goes into a pocket of the pile when one holds it (see pocket_search()), else where it keeps
    // the bounding box smallest (see search()).
    Pass greedy(const std::vector<int> &order, bool seeding, int &done, int total)
    {
        Pass pass;
        pass.spots.resize(m_parts.size());
        for (int k = 0; k < m_obstacle_plates; ++k)
            pass.plates.emplace_back(make_plate(k));
        std::vector<int> rank(m_parts.size());
        for (size_t k = 0; k < order.size(); ++k)
            rank[size_t(order[k])] = int(k);
        // (group, plate) pairs already seeded, and those where a copy found no room: the board
        // only fills up, so no later copy fits there either.
        std::set<std::pair<int, int>> seeded, full;
        auto try_plate = [&](int i, Plate &plate) {
            open(plate);
            const int g = m_group[size_t(i)];
            if (g >= 0 && full.count({g, plate.index}))
                return false;
            if (g >= 0 && seeding && seeded.insert({g, plate.index}).second && seed(i, m_groups[size_t(g)], plate, pass, rank) > 0)
                return true;
            for (bool pocket : {true, false})
                if (auto [cand, hit] = pocket ? pocket_search(shapes_of(i), plate) : search(shapes_of(i), plate, false); cand >= 0) {
                    place(i, cand, hit.x, hit.y, plate, pass);
                    return true;
                }
            if (g >= 0)
                full.insert({g, plate.index});
            return false;
        };
        for (int i : order) {
            if (stopped())
                break;
            bool placed = pass.spots[size_t(i)].cand >= 0; // seeded with an earlier copy
            for (size_t k = 0; k < pass.plates.size() && !placed; ++k)
                placed = accepts(pass.plates[k], i) && try_plate(i, pass.plates[k]);
            if (!placed && int(pass.plates.size()) < m_p.max_plates) {
                // Keep the new plate only if the part fits on it.
                Plate plate = make_plate(int(pass.plates.size()));
                if ((placed = try_plate(i, plate)))
                    pass.plates.emplace_back(std::move(plate));
            }
            if (m_p.progress)
                m_p.progress(++done, total);
        }
        for (const Spot &s : pass.spots)
            pass.unplaced += s.cand < 0;
        for (const Plate &plate : pass.plates) {
            long   cells = 0;
            double weighted = 0.;
            for (int i : plate.placed) {
                const Shape &s = m_shapes[size_t(i)][size_t(pass.spots[size_t(i)].cand)];
                cells += s.cells, weighted += s.cells * s.factor;
            }
            pass.fill.push_back(int(plate.placed.size()));
            // Free area left in pockets is inside the bounding box already and counts once more as
            // waste, the measure pocket_search() minimises: a pass that fills its pockets beats one
            // whose bounding box is smaller by less than the area it left empty in them.
            if (cells > 0)
                ++pass.plates_used, pass.cost += double(plate.bbox.area()) * weighted / double(cells) + double(Pockets(plate.board, plate.bbox).enclosed());
        }
        while (!pass.fill.empty() && pass.fill.back() == 0)
            pass.fill.pop_back();
        return pass;
    }

    // Moves the plate's placed parts as one group so their bbox centres on the anchor, halving
    // the shift until it clears the bed mask and obstacles. Growth from the anchor leaves little
    // asymmetry to remove; this mostly centres the corner-grown pass.
    void align(Plate &plate, std::vector<Spot> &spots) const
    {
        if (plate.placed.empty() || !m_centre)
            return;
        const Rect &bb = plate.bbox;
        auto shift = [](double a, int bed, int lo, int hi) { return std::clamp(int(std::lround(a - 0.5 * (lo + hi))), -lo, bed - hi); };
        int   dx = shift(m_anchor_x, m_w, bb.x0, bb.x1), dy = shift(m_anchor_y, m_h, bb.y0, bb.y1);
        Board fixed = (dx || dy) ? fixed_board(plate.index) : Board(0, 0);
        auto  feasible = [&] {
            for (int i : plate.placed)
                if (!fixed.fits(m_shapes[i][spots[i].cand], spots[i].x + dx, spots[i].y + dy, m_p))
                    return false;
            return true;
        };
        while ((dx || dy) && !feasible())
            dx /= 2, dy /= 2;
        for (int i : plate.placed)
            spots[i].x += dx, spots[i].y += dy;
        plate.bbox = {bb.x0 + dx, bb.y0 + dy, bb.x1 + dx, bb.y1 + dy};
    }

    const std::vector<Part>                 &m_parts;
    const std::vector<Obstacle>             &m_obstacles;
    const Params                            &m_p;
    std::function<void(size_t, const std::function<void(size_t)> &)> m_parallel_for;
    std::vector<std::vector<Shape>>          m_shapes;  // per part, per candidate
    std::vector<Group>                       m_groups;
    std::vector<int>                         m_group;   // per part: index into m_groups, -1 when unique
    int                                      m_w = 0, m_h = 0;
    bool                                     m_centre = true;              // grow from the anchor, else the min corner
    double                                   m_anchor_x = 0., m_anchor_y = 0.; // cells
    double                                   m_ax = 0., m_ay = 0.;         // growth origin of the current pass, cells
    double                                   m_lambda = 0.;        // scored cells^2 per contact cell
    int                                      m_obstacle_plates = 0;
    mutable std::atomic<bool>                m_stopped{false};
};

} // namespace

HeightMap rasterize(const std::vector<Triangle> &tris, double cell_mm, double inflate_mm, bool fill_to_bed)
{
    if (tris.empty())
        return HeightMap(0, 0, cell_mm);
    double min_x = std::numeric_limits<double>::max(), min_y = min_x;
    double max_x = std::numeric_limits<double>::lowest(), max_y = max_x;
    for (const Triangle &t : tris)
        for (const auto &v : t.v) {
            min_x = std::min(min_x, double(v[0])), max_x = std::max(max_x, double(v[0]));
            min_y = std::min(min_y, double(v[1])), max_y = std::max(max_y, double(v[1]));
        }
    auto cells = [&](double extent) { return std::max(1, int(std::ceil((extent + 2. * inflate_mm) / cell_mm - 1e-6))); };
    HeightMap m(cells(max_x - min_x), cells(max_y - min_y), cell_mm);
    m.origin_x = min_x - inflate_mm;
    m.origin_y = min_y - inflate_mm;
    // Per occupied cell, the extent of the geometry inside it along x, y, x + y and x - y (min and max
    // each, cell-local): an octagon holding it. Growth measures from that rather than from the whole
    // cell, so the clearance is not rounded up to whole cells, along axis-parallel and diagonal edges.
    const double grow = std::max(0., inflate_mm) / cell_mm;
    std::vector<std::array<float, 8>> extent(grow > 0. ? m.cells.size() : 0, {1.f, 0.f, 1.f, 0.f, 2.f, 0.f, 1.f, -1.f});

    for (const Triangle &t : tris) {
        // Work in cell units so cell (i, j) is the unit square at (i, j).
        ClipPoly tri;
        for (int k = 0; k < 3; ++k)
            tri[k] = {(t.v[k][0] - m.origin_x) / cell_mm, (t.v[k][1] - m.origin_y) / cell_mm, double(t.v[k][2])};
        double ux = tri[1].x - tri[0].x, uy = tri[1].y - tri[0].y, uz = tri[1].z - tri[0].z;
        double vx = tri[2].x - tri[0].x, vy = tri[2].y - tri[0].y, vz = tri[2].z - tri[0].z;
        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        double nxy = std::hypot(nx, ny);
        // A vertical wall projects to a segment and covers no area, so it marks every
        // cell its segment touches. A wall lying exactly on a grid line is nudged a hair
        // against its normal, onto the solid side, so it does not widen a closed part.
        bool wall = std::abs(nz) <= 1e-5 * std::hypot(nxy, nz);
        // Clamping keeps a wall on the map's outer edge inside the map.
        if (wall && nxy > 0.)
            for (int k = 0; k < 3; ++k) {
                tri[k].x = std::clamp(tri[k].x - 1e-3 * nx / nxy, 0., double(m.width));
                tri[k].y = std::clamp(tri[k].y - 1e-3 * ny / nxy, 0., double(m.height));
            }

        auto [lx, hx] = std::minmax({tri[0].x, tri[1].x, tri[2].x});
        auto [ly, hy] = std::minmax({tri[0].y, tri[1].y, tri[2].y});
        int x0 = std::clamp(int(std::floor(lx)), 0, m.width - 1), x1 = std::clamp(int(std::floor(hx)), 0, m.width - 1);
        int y0 = std::clamp(int(std::floor(ly)), 0, m.height - 1), y1 = std::clamp(int(std::floor(hy)), 0, m.height - 1);
        // Clip to each column once and visit only the rows its piece reaches, down to the row whose
        // top border it touches: a sliver such as one triangle of a cap's fan would otherwise cost
        // its whole bounding box.
        for (int cx = x0; cx <= x1; ++cx) {
            ClipPoly col, tmp;
            int nc = clip(tri, 3, tmp, false, cx, 1.);
            nc = clip(tmp, nc, col, false, cx + 1, -1.);
            if (nc == 0)
                continue;
            auto [lo, hi] = std::minmax_element(col.begin(), col.begin() + nc, [](const ClipPoint &p, const ClipPoint &q) { return p.y < q.y; });
            const int r0 = std::clamp(int(std::ceil(lo->y)) - 1, y0, y1), r1 = std::clamp(int(std::floor(hi->y)), y0, y1);
            for (int cy = r0; cy <= r1; ++cy) {
                ClipPoly a, b;
                int n = clip(col, nc, a, true, cy, 1.);
                n = clip(a, n, b, true, cy + 1, -1.);
                if (n == 0)
                    continue;
                // A piece that only touches the cell's border does not occupy it: faces
                // need area inside the cell, walls need length.
                double size = 0.;
                for (int k = 0; k < n; ++k) {
                    const ClipPoint &p = b[k], &q = b[(k + 1) % n];
                    size += wall ? std::abs(q.x - p.x) + std::abs(q.y - p.y) : p.x * q.y - q.x * p.y;
                }
                if (std::abs(size) <= 1e-9)
                    continue;
                Span &s = m.at(cx, cy);
                for (int k = 0; k < n; ++k)
                    s.add(float(b[k].z), float(b[k].z));
                if (grow > 0.) {
                    std::array<float, 8> &e = extent[size_t(cy) * m.width + cx];
                    for (int k = 0; k < n; ++k) {
                        const float v[4] = {float(b[k].x - cx), float(b[k].y - cy), float(b[k].x - cx + b[k].y - cy), float(b[k].x - cx - b[k].y + cy)};
                        for (int a = 0; a < 4; ++a)
                            e[2 * a] = std::min(e[2 * a], v[a]), e[2 * a + 1] = std::max(e[2 * a + 1], v[a]);
                    }
                }
            }
        }
    }
    if (grow > 0.) {
        // Every cell closer than `grow` to the octagon of an occupied cell takes that cell's span.
        // The distance between them is at least their gap along each of the four directions, and
        // at least the distance between the octagon's bounding box and the cell; the larger of these
        // lower bounds is exact for axis-parallel and diagonal edges. The geometry lies on the map's
        // grid from inflate_mm in, so on a part whose sizes are multiples of the cell an edge keeps
        // exactly inflate_mm.
        // Offsets whose target cell lies within `grow` of the whole source cell are claimed and those
        // farther than `grow` from it are not, whatever the octagon, so only the ring between needs
        // the octagon test; a target whose span already holds the source's needs nothing. A cell
        // whose 8 neighbours hold its span and are filled by geometry claims nothing they do not:
        // every target beyond them is nearer to one of them by at least a cell, so only cells near
        // the outline grow.
        struct Offset { int dx, dy; bool always; };
        std::vector<Offset> offsets;
        const int           r = int(std::ceil(grow));
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                // Distances from the target cell to the source cell's nearest and farthest points.
                const double nx = std::max(std::abs(dx) - 1, 0), ny = std::max(std::abs(dy) - 1, 0);
                if (nx * nx + ny * ny < grow * grow + 1e-6)
                    offsets.push_back({dx, dy, double(dx) * dx + double(dy) * dy < grow * grow - 1e-6});
            }
        const HeightMap src = m;
        auto holds  = [](const Span &t, const Span &s) { return t.lo <= s.lo && t.hi >= s.hi; };
        auto filled = [&](int x, int y) {
            const std::array<float, 8> &e = extent[size_t(y) * m.width + x];
            const float                 eps = 1e-4f;
            return e[0] <= eps && e[1] >= 1.f - eps && e[2] <= eps && e[3] >= 1.f - eps && e[4] <= eps && e[5] >= 2.f - eps &&
                   e[6] <= eps - 1.f && e[7] >= 1.f - eps;
        };
        auto inner = [&](int x, int y, const Span &s) {
            if (x == 0 || y == 0 || x + 1 == m.width || y + 1 == m.height)
                return false;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                    if ((dx || dy) && !(holds(src.at(x + dx, y + dy), s) && filled(x + dx, y + dy)))
                        return false;
            return true;
        };
        auto gap = [](double lo, double hi, double e_lo, double e_hi) { return std::max({0., lo - e_hi, e_lo - hi}); };
        for (int sy = 0; sy < m.height; ++sy)
            for (int sx = 0; sx < m.width; ++sx) {
                const Span &s = src.at(sx, sy);
                if (s.empty() || inner(sx, sy, s))
                    continue;
                const std::array<float, 8> &e = extent[size_t(sy) * m.width + sx];
                for (const Offset &o : offsets) {
                    if (!m.in_bounds(sx + o.dx, sy + o.dy))
                        continue;
                    Span &t = m.at(sx + o.dx, sy + o.dy);
                    if (holds(t, s))
                        continue;
                    // The target cell is [dx, dx + 1] x [dy, dy + 1] in the source cell's frame.
                    const int    dx = o.dx, dy = o.dy;
                    const double gx = gap(dx, dx + 1, e[0], e[1]), gy = gap(dy, dy + 1, e[2], e[3]);
                    const double gu = gap(dx + dy, dx + dy + 2, e[4], e[5]), gv = gap(dx - dy - 1, dx - dy + 1, e[6], e[7]);
                    if (o.always || std::max(gx * gx + gy * gy, 0.5 * std::max(gu * gu, gv * gv)) < grow * grow - 1e-9)
                        t.add(s.lo, s.hi);
                }
            }
    }
    if (fill_to_bed)
        for (Span &s : m.cells)
            if (!s.empty())
                s.lo = 0.f;
    return m;
}

Effort effort_settings(int effort, double bed_w, double bed_h)
{
    // Cells of 1 and 1/2 mm divide whole and half millimetres, so an axis-parallel edge at such a
    // size and spacing falls on a cell boundary and keeps its spacing exactly (see rasterize()).
    // Thorough spends its extra time on orderings; finer cells cost far more for little gain. Past
    // MaxBedCells per side the cell grows with the bed, which bounds the cost of a scan.
    constexpr double Cell[]      = {1., 0.5, 0.5};
    constexpr int    Orderings[] = {1, 3, 6};
    constexpr double MaxBedCells = 720.;
    effort = std::clamp(effort, 0, 2);
    return {std::max(Cell[effort], std::max(bed_w, bed_h) / MaxBedCells), Orderings[effort]};
}

std::vector<Placement> arrange(const std::vector<Part> &parts, const std::vector<Obstacle> &obstacles, const Params &params, Stats *stats)
{
    return Arranger(parts, obstacles, params).run(stats);
}

}} // namespace Slic3r::nest3d
