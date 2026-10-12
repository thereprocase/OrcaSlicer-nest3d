#include "Nest3D.hpp"

#include <algorithm>
#include <array>
#include <atomic>
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

const Span FullSpan{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::max()};

// ---------------------------------------------------------------------------
// Arrangement
// ---------------------------------------------------------------------------

constexpr double NoScore   = std::numeric_limits<double>::infinity();
constexpr int    Mixed     = -2;  // plate group when its fixed parts span several groups
// Envelope area, in mm^2, that one mm of contact outweighs. Small next to the envelope growth of
// moving a part by a cell, so contact mostly decides between positions of equal envelope; chosen
// by a sweep over 0..128 on fill-the-bed and mixed sets.
constexpr double ContactMM = 16.;
constexpr size_t MaxBasis  = 2048; // lattice basis vectors tried per cluster, shortest first

// std::popcount is C++20.
int popcount(uint64_t v)
{
    v = v - ((v >> 1) & 0x5555555555555555ull);
    v = (v & 0x3333333333333333ull) + ((v >> 2) & 0x3333333333333333ull);
    v = (v + (v >> 4)) & 0x0f0f0f0f0f0f0f0full;
    return int((v * 0x0101010101010101ull) >> 56);
}

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
    float                 top = 0.f;
    double                volume = 0.;
    bool                  usable = false;

    bool test(int x, int y) const
    {
        return x >= 0 && y >= 0 && x < w && y < h && (bits[size_t(y) * words + x / 64] >> (x % 64) & 1);
    }
};

Shape make_shape(const HeightMap &m, double max_height)
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
    std::vector<int>      sums; // (w + 1) x (h + 1): non-empty cells below and left of each corner

    Board(int w_, int h_)
        : w(w_), h(h_), words((w_ + 63) / 64 + 2), occ(size_t(w_) * h_), bits(size_t(h_) * words, 0), sums(size_t(w_ + 1) * (h_ + 1), 0)
    {}

    // Brings the summed-area table up to date from row y0 on.
    void update_sums(int y0)
    {
        for (int y = std::max(y0, 0); y < h; ++y)
            for (int x = 0, row = 0; x < w; ++x) {
                row += (bits[size_t(y) * words + x / 64 + 1] >> (x % 64)) & 1;
                sums[size_t(y + 1) * (w + 1) + x + 1] = sums[size_t(y) * (w + 1) + x + 1] + row;
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
        occ[size_t(y) * w + x].add(s.lo, s.hi);
        uint64_t &word = bits[size_t(y) * words + x / 64 + 1];
        const uint64_t bit = uint64_t(1) << (x % 64);
        filled += (word & bit) == 0;
        word |= bit;
    }

    // Occupancy of the 64 cells of row y starting at cell x, -64 <= x <= w.
    uint64_t read(int y, int x) const
    {
        const uint64_t *row = &bits[size_t(y) * words];
        const int       pos = x + 64, i = pos / 64, sh = pos % 64;
        return sh ? (row[i] >> sh) | (row[i + 1] << (64 - sh)) : row[i];
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
        update_sums(ty);
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
// With a reduced basis the scanned coefficient box holds O(1 + area / det) points.
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
    Shape                      shape; // union occupancy, searched like a part
    Vec                 v1, v2;
    long long           det = 0; // cells per lattice site; 0: no lattice
};

// Densest lattice whose translates of the cluster miss each other. Basis vectors are drawn from
// the offsets at which a translate touches the cluster (free, next to a colliding offset), the
// MaxBasis shortest of them; every locally densest lattice has two independent touching vectors.
// The bounding-box lattice is always valid and is the fallback. With B <= MaxBasis offsets this
// tests O(B^2) bases, and only those denser than the best so far are validated against all
// O(1 + W * H / det) lattice points of the collision window.
void find_lattice(Cluster &k)
{
    const Runs    r    = runs_of(k.shape);
    const Offsets self = collisions(r, k.shape.w, r, k.shape.w);
    const int     W = k.shape.w, H = k.shape.h;
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
    long area = std::numeric_limits<long>::max(), contact = 0;
    int  ca = -1, cb = -1, dx = 0, dy = 0;
    bool better_than(const PairFit &o) const { return area < o.area || (area == o.area && contact > o.contact); }
};

// Tightest touching offset of b next to a: smallest common bounding box, then most contact.
void best_pair(const Shape &a, const Runs &ra, int ca, const Shape &b, const Runs &rb, int cb, PairFit &best)
{
    const Offsets nfp = collisions(ra, a.w, rb, b.w);
    for (int dy = nfp.y0 - 1; dy <= nfp.y0 + nfp.h; ++dy)
        for (int dx = nfp.x0 - 1; dx <= nfp.x0 + nfp.w; ++dx) {
            if (!nfp.touching(dx, dy))
                continue;
            PairFit fit{long(std::max(a.w, dx + b.w) - std::min(0, dx)) * (std::max(a.h, dy + b.h) - std::min(0, dy)), 0, ca, cb, dx, dy};
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

// Where a part landed: candidate index and bed cell of its shape's tight rect.
struct Spot { int cand = -1, x = 0, y = 0; };

struct Plate
{
    int              index = 0;
    int              group = -1; // compat group of the parts on it: -1 while none, Mixed for several
    bool             locked = false;
    Board            board;
    Rect             bbox;   // of the parts placed by this run
    std::vector<int> placed; // part indices
};

struct Hit
{
    double score = NoScore;
    double dist  = 0.; // squared distance from the shape's centre to the anchor, cells
    int    y = 0, x = 0;
    bool   better_than(const Hit &o) const
    {
        if (score != o.score)
            return score < o.score;
        if (dist != o.dist)
            return dist < o.dist;
        return y < o.y || (y == o.y && x < o.x);
    }
};

struct Pass
{
    std::vector<Spot>  spots; // per part
    std::vector<Plate> plates;
    int                unplaced = 0, plates_used = 0;
    long               bbox_area = 0;
    bool               lattice = false; // copies were laid on a lattice
    bool               better_than(const Pass &o) const
    {
        if (unplaced != o.unplaced) return unplaced < o.unplaced;
        if (plates_used != o.plates_used) return plates_used < o.plates_used;
        return bbox_area < o.bbox_area;
    }
};

// Half extent, about anchor coordinate a, of the smallest anchor-centred interval holding [t0, t1].
double half(double a, double t0, double t1) { return std::max(a - t0, t1 - a); }

class Arranger
{
public:
    Arranger(const std::vector<Part> &parts, const std::vector<Obstacle> &obstacles, const Params &p)
        : m_parts(parts), m_obstacles(obstacles), m_p(p)
    {
        m_w = std::max(0, int(std::floor(p.bed_w / p.cell_mm + 1e-6)));
        m_h = std::max(0, int(std::floor(p.bed_h / p.cell_mm + 1e-6)));
        m_centre = p.align_x >= 0.;
        m_anchor_x = m_centre ? std::clamp(p.align_x, 0., 1.) * m_w : 0.;
        m_anchor_y = m_centre ? std::clamp(p.align_y, 0., 1.) * m_h : 0.;
        m_lambda = ContactMM / p.cell_mm;
        m_shapes.resize(parts.size());
        m_options.resize(parts.size());
        for (size_t i = 0; i < parts.size(); ++i) {
            for (const Candidate &c : parts[i].candidates)
                m_shapes[i].emplace_back(make_shape(c.map, p.max_height));
            for (size_t c = 0; c < parts[i].candidates.size(); ++c)
                m_options[i].push_back({&m_shapes[i][c], 1. + std::max(0., parts[i].candidates[c].penalty)});
        }
        for (const Obstacle &o : obstacles)
            m_obstacle_plates = std::max(m_obstacle_plates, o.plate + 1);
        if (p.parallel_for)
            m_parallel_for = p.parallel_for;
        else
            m_parallel_for = [](size_t n, const std::function<void(size_t)> &fn) { for (size_t i = 0; i < n; ++i) fn(i); };

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
        struct Strategy { size_t order; int ordering; bool seeded, corner; };
        std::vector<std::vector<int>> orders;
        std::vector<Strategy>         strategies;
        for (int k = 0; k < std::clamp(m_p.orderings, 1, 6); ++k)
            if (std::vector<int> o = order(k); std::find(orders.begin(), orders.end(), o) == orders.end())
                orders.push_back(std::move(o)), strategies.push_back({orders.size() - 1, k, false, !m_centre});
        if (std::any_of(m_groups.begin(), m_groups.end(), [](const Group &g) { return !g.clusters.empty(); }))
            strategies.push_back({0, 0, true, !m_centre});
        if (m_centre)
            strategies.push_back({0, 0, false, true});

        Pass best;
        int  best_strategy = -1, done = 0;
        const int total = int(strategies.size() * m_parts.size());
        for (size_t k = 0; k < strategies.size() && !m_stopped; ++k) {
            const Strategy &s = strategies[k];
            if (m_centre && s.corner && best.unplaced == 0 && best.plates_used <= 1)
                break;
            m_ax = s.corner ? 0. : m_anchor_x, m_ay = s.corner ? 0. : m_anchor_y;
            Pass pass = greedy(orders[s.order], s.seeded, done, total);
            if (best_strategy < 0 || pass.better_than(best))
                best = std::move(pass), best_strategy = int(k);
        }
        if (m_p.progress && !m_stopped && done < total)
            m_p.progress(total, total);
        for (Plate &plate : best.plates)
            align(plate, best.spots);

        std::vector<Placement> out(m_parts.size());
        long                   cells = 0;
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
            cells += s.cells;
        }
        for (const Plate &plate : best.plates)
            for (int i : plate.placed)
                out[i].plate = plate.index;
        if (stats) {
            stats->plates_used     = best.plates_used;
            stats->unplaced        = best.unplaced;
            stats->ordering_used   = best_strategy < 0 ? 0 : strategies[size_t(best_strategy)].ordering;
            stats->lattice_used    = best.lattice;
            stats->footprint_ratio = best.plates_used > 0 ? double(cells) / (double(m_w) * m_h * best.plates_used) : 0.;
        }
        return out;
    }

private:
    struct Option
    {
        const Shape *shape  = nullptr;
        double       factor = 1.; // score multiplier from the candidate penalty
    };

    struct Group
    {
        std::vector<int>     parts;    // identical copies, input order
        std::vector<Cluster> clusters; // with a lattice each, densest per copy first
    };

    // Per group of copies: a cluster of one copy per candidate and one of the tightest pair, each
    // with its densest lattice. Candidates are tasks reduced in task order, so the result does not
    // depend on scheduling. Cost per group with C candidates of W x H cells:
    // O(C^2 * NFP(W, H)) for the pairs plus C + 1 lattice searches (see find_lattice).
    void build_clusters()
    {
        struct Task { size_t g; int c; };
        std::vector<Task>              tasks;
        std::vector<std::vector<Runs>> runs(m_groups.size());
        for (size_t g = 0; g < m_groups.size(); ++g)
            for (const Shape &s : m_shapes[size_t(m_groups[g].parts.front())]) {
                tasks.push_back({g, int(runs[g].size())});
                runs[g].push_back(s.usable ? runs_of(s) : Runs{});
            }
        std::vector<Cluster> singles(tasks.size());
        std::vector<PairFit> pairs(tasks.size());
        m_parallel_for(tasks.size(), [&](size_t t) {
            const std::vector<Shape> &ss = m_shapes[size_t(m_groups[tasks[t].g].parts.front())];
            const int                 c  = tasks[t].c;
            if (!ss[size_t(c)].usable)
                return;
            singles[t].members = {{c, 0, 0}};
            singles[t].shape   = ss[size_t(c)];
            find_lattice(singles[t]);
            for (size_t cb = 0; cb < ss.size(); ++cb)
                if (ss[cb].usable)
                    best_pair(ss[size_t(c)], runs[tasks[t].g][size_t(c)], c, ss[cb], runs[tasks[t].g][cb], int(cb), pairs[t]);
        });
        std::vector<PairFit> best_pairs(m_groups.size());
        for (size_t t = 0; t < tasks.size(); ++t) {
            if (singles[t].det > 0)
                m_groups[tasks[t].g].clusters.push_back(std::move(singles[t]));
            if (pairs[t].ca >= 0 && pairs[t].better_than(best_pairs[tasks[t].g]))
                best_pairs[tasks[t].g] = pairs[t];
        }
        std::vector<Cluster> pair_clusters(m_groups.size());
        m_parallel_for(m_groups.size(), [&](size_t g) {
            if (best_pairs[g].ca >= 0)
                make_pair(m_shapes[size_t(m_groups[g].parts.front())], best_pairs[g], pair_clusters[g]);
        });
        for (size_t g = 0; g < m_groups.size(); ++g) {
            std::vector<Cluster> &cs = m_groups[g].clusters;
            if (pair_clusters[g].det > 0)
                cs.push_back(std::move(pair_clusters[g]));
            std::stable_sort(cs.begin(), cs.end(), [](const Cluster &a, const Cluster &b) {
                return a.det * (long long) b.members.size() < b.det * (long long) a.members.size();
            });
        }
    }

    // Builds the union of the two copies.
    void make_pair(const std::vector<Shape> &ss, const PairFit &fit, Cluster &k) const
    {
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
        k.shape = make_shape(*k.map, m_p.max_height);
        find_lattice(k);
    }

    // Part order for greedy pass k: priority first, then one size key per pass.
    std::vector<int> order(int k) const
    {
        auto key = [&](int i) -> double {
            const std::vector<Shape> &ss = m_shapes[i];
            if (ss.empty())
                return 0.;
            const Shape &s = ss.front();
            switch (k) {
            case 0: return s.cells;
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
            return key(a) > key(b);
        });
        return idx;
    }

    // Bed mask and obstacles of plate `index`.
    Board fixed_board(int index) const
    {
        Board b(m_w, m_h);
        for (int y = 0; y < m_h; ++y)
            for (int x = 0; x < m_w; ++x)
                if (size_t i = size_t(y) * m_w + x; i < m_p.bed_mask.size() && m_p.bed_mask[i])
                    b.add(x, y, FullSpan);
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
        b.update_sums(0);
        return b;
    }

    // The board is built by open(), on first use: obstacles on every plate put all
    // plates in play, and most of them never receive a part.
    Plate make_plate(int index) const
    {
        bool locked = std::find(m_p.locked_plates.begin(), m_p.locked_plates.end(), index) != m_p.locked_plates.end();
        int  group  = -1;
        for (const Obstacle &o : m_obstacles)
            if (o.plate == index && o.group >= 0)
                group = group == -1 || group == o.group ? o.group : Mixed;
        return Plate{index, group, locked, Board(0, 0), {}, {}};
    }

    void open(Plate &plate) const
    {
        if (!plate.locked && plate.board.w != m_w)
            plate.board = fixed_board(plate.index);
    }

    // Area of the smallest anchor-centred rectangle holding `r`, cells.
    double envelope(const Rect &r) const { return 4. * half(m_ax, r.x0, r.x1) * half(m_ay, r.y0, r.y1); }

    // Best position on `plate` over `options`. The score is the envelope of this run's parts on
    // the plate after placement (see envelope()), times the option's factor (a 0.1 penalty
    // competes as if the envelope were 10% larger), minus ContactMM per mm of the shape's outline
    // that touches occupied cells or the bed edge. The envelope makes the packing grow outward
    // from the anchor, so no wall pins the first part; contact decides between positions inside
    // the envelope, which is where the parts interlock. Ties go to the position nearest the
    // anchor, then the lowest y, x and option.
    // Contact is at most the halo size, and at most the occupied cells around the shape's rect
    // (read in O(1) from the summed-area table) plus its wall contact. Rows and positions whose
    // score cannot reach the best even with that much contact are skipped before the collision
    // test, and a position with nothing around it needs neither test. Rows and columns are
    // visited from the anchor outward so that good scores are found early, and each side stops
    // once no position further out can win. Options and row bands are independent tasks reduced
    // in task order, so the result does not depend on scheduling.
    std::pair<int, Hit> search(const std::vector<Option> &options, const Plate &plate) const
    {
        const int        n_bands = std::clamp(m_h / 32, 1, 8);
        std::vector<Hit> hits(options.size() * n_bands);
        std::atomic<double> shared{NoScore}; // best score any task has found; prunes all of them
        m_parallel_for(hits.size(), [&](size_t task) {
            const Shape &s    = *options[task / n_bands].shape;
            const double f    = options[task / n_bands].factor;
            const int    band = int(task % n_bands);
            // Without stacking every cell of the shape needs a free bed cell.
            if (!s.usable || s.w > m_w || s.h > m_h || (!m_p.allow_stacking && s.cells > m_w * m_h - plate.board.filled))
                return;
            const Rect  &bb    = plate.bbox;
            const double bonus = m_lambda * s.halo_cells;
            Hit          best;
            auto         limit = [&] { return std::min(best.score, shared.load(std::memory_order_relaxed)); };
            const int walls_all = s.edge[0] + s.edge[1] + s.edge[2] + s.edge[3];
            // Columns to advance from (tx, ty) along a row, `dir` +1 or -1 away from the anchor; 0
            // once nothing further on can win. The envelope only grows that way, and once the
            // board is empty from here on, contact can only come from the bed edges.
            auto try_at = [&](int tx, int ty, int dir) {
                const double env = envelope(bb.united({tx, ty, tx + s.w, ty + s.h})) * f;
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
            // Rows at distance d = band, band + n_bands, ... above and below the most centred one;
            // each side ends at the first row whose envelope alone loses. Columns likewise.
            const int    r0 = std::clamp(int(std::lround(m_ay - 0.5 * s.h)), 0, m_h - s.h);
            const int    c0 = std::clamp(int(std::lround(m_ax - 0.5 * s.w)), 0, m_w - s.w);
            const double hx_min = std::max(half(m_ax, c0, c0 + s.w), bb.empty() ? 0. : half(m_ax, bb.x0, bb.x1));
            bool         open[2] = {true, true};
            for (int d = band; open[0] || open[1]; d += n_bands)
                for (int side = 0; side < 2; ++side) {
                    const int ty = side == 0 ? r0 + d : r0 - d;
                    if (!open[side] || (d == 0 && side == 1))
                        continue;
                    const double hy = std::max(half(m_ay, ty, ty + s.h), bb.empty() ? 0. : half(m_ay, bb.y0, bb.y1));
                    const double lb = 4. * hx_min * hy * f;
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

    void place(int i, int cand, int x, int y, Plate &plate, Pass &pass) const
    {
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
            for (const Cluster::Member &m : k.members) {
                const Shape &s  = m_shapes[size_t(i)][size_t(m.cand)];
                const int    sx = int(ox + x) + m.dx, sy = int(oy + y) + m.dy;
                n += sx >= 0 && sy >= 0 && sx + s.w <= m_w && sy + s.h <= m_h && plate.board.fits(s, sx, sy, m_p);
            }
            return n < cap;
        });
        return std::min(n, cap);
    }

    // Seeds the copies of part i not placed yet, i first, on one of its group's cluster lattices.
    // Each lattice passes through the position where its whole cluster scores best; the densest
    // lattice that holds all copies there wins. When none does (the copies fill the plate), every
    // coset of every lattice is counted and the one holding the most copies wins: the lattice
    // phase decides how much the bed edges cut off. The sites nearest the anchor, by the square
    // rings the envelope grows in, take the copies; each copy goes where its own map fits, so
    // sites cut by the bed edge, obstacles or earlier parts still take what fits there.
    // Returns the number of copies placed. Cost: per cluster one search, plus, when the plate
    // fills, O(det) cosets of O(1 + bed / det) sites each: O(bed cells * members) collision tests.
    int seed(int i, const Group &g, Plate &plate, Pass &pass, const std::vector<int> &rank) const
    {
        std::vector<int> todo;
        for (int j : g.parts)
            if (pass.spots[size_t(j)].cand < 0 && rank[size_t(j)] >= rank[size_t(i)])
                todo.push_back(j);
        std::sort(todo.begin(), todo.end(), [&](int a, int b) { return rank[size_t(a)] < rank[size_t(b)]; });
        const int cap = int(todo.size());

        const Cluster *best = nullptr;
        long long      bx = 0, by = 0;
        int            room = 0;
        for (const Cluster &k : g.clusters) {
            const auto [o, at] = search({{&k.shape, 1.}}, plate);
            if (o < 0)
                continue;
            if (int n = lattice_room(i, k, plate, at.x, at.y, cap); n > room)
                best = &k, bx = at.x, by = at.y, room = n;
            if (room == cap)
                break;
            const std::pair<long long, long long> box = coset_box(k.v1, k.v2);
            const long long                       a   = box.first;
            std::vector<int>                      counts(size_t(a * box.second));
            m_parallel_for(counts.size(), [&](size_t t) { counts[t] = lattice_room(i, k, plate, (long long) t % a, (long long) t / a, cap); });
            for (size_t t = 0; t < counts.size(); ++t)
                if (counts[t] > room)
                    best = &k, bx = (long long) t % a, by = (long long) t / a, room = counts[t];
            if (room == cap)
                break;
        }
        if (best == nullptr)
            return 0;

        struct Site { double ring, dist; int y, x; };
        std::vector<Site> sites;
        const int         W = best->shape.w, H = best->shape.h;
        for_lattice_points(best->v1, best->v2, 1 - W - bx, 1 - H - by, m_w - 1 - bx, m_h - 1 - by, [&](long long dx, long long dy) {
            const int    x = int(bx + dx), y = int(by + dy);
            const double cx = x + 0.5 * W - m_ax, cy = y + 0.5 * H - m_ay;
            sites.push_back({std::max(half(m_ax, x, x + W), half(m_ay, y, y + H)), cx * cx + cy * cy, y, x});
            return true;
        });
        std::sort(sites.begin(), sites.end(), [](const Site &a, const Site &b) {
            return std::tie(a.ring, a.dist, a.y, a.x) < std::tie(b.ring, b.dist, b.y, b.x);
        });
        size_t next = 0;
        for (const Site &site : sites)
            for (const Cluster::Member &m : best->members) {
                if (next == todo.size())
                    break;
                const Shape &s = m_shapes[size_t(todo[next])][size_t(m.cand)];
                const int    x = site.x + m.dx, y = site.y + m.dy;
                if (x >= 0 && y >= 0 && x + s.w <= m_w && y + s.h <= m_h && plate.board.fits(s, x, y, m_p))
                    place(todo[next++], m.cand, x, y, plate, pass);
            }
        pass.lattice |= next > 1;
        return int(next);
    }

    bool accepts(const Plate &plate, int i) const
    {
        return !plate.locked && (m_p.mix_groups || plate.group == -1 || plate.group == m_parts[i].compat_group);
    }

    // First-fit greedy over `order`: the first open plate with room wins, else a new plate. With
    // seeding, the first copy of a group to reach a plate lays the group's lattice there.
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
            if (auto [cand, hit] = search(m_options[size_t(i)], plate); cand >= 0) {
                place(i, cand, hit.x, hit.y, plate, pass);
                return true;
            }
            if (g >= 0)
                full.insert({g, plate.index});
            return false;
        };
        for (int i : order) {
            if (m_p.should_stop && m_p.should_stop()) {
                m_stopped = true;
                break;
            }
            bool placed = pass.spots[size_t(i)].cand >= 0; // seeded with an earlier copy
            for (size_t k = 0; k < pass.plates.size() && !placed; ++k)
                placed = accepts(pass.plates[k], i) && try_plate(i, pass.plates[k]);
            while (!placed && int(pass.plates.size()) < m_p.max_plates) {
                Plate plate = make_plate(int(pass.plates.size()));
                if (plate.locked) {
                    pass.plates.emplace_back(std::move(plate));
                    continue;
                }
                // Keep the new plate only if the part fits on it.
                if ((placed = try_plate(i, plate)))
                    pass.plates.emplace_back(std::move(plate));
                break;
            }
            if (m_p.progress)
                m_p.progress(++done, total);
        }
        for (const Spot &s : pass.spots)
            pass.unplaced += s.cand < 0;
        for (const Plate &plate : pass.plates)
            if (!plate.placed.empty())
                ++pass.plates_used, pass.bbox_area += plate.bbox.area();
        return pass;
    }

    // Moves the plate's placed parts as one group so their bbox centres on the anchor, halving
    // the shift until it clears the bed mask and obstacles. Growth from the anchor leaves little
    // asymmetry to remove; this mostly centres the corner-grown pass. Legacy mode (align_x < 0)
    // keeps the corner packing.
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
    std::vector<std::vector<Shape>>          m_shapes;  // per part, per candidate
    std::vector<std::vector<Option>>         m_options; // per part, per candidate
    std::vector<Group>                       m_groups;  // never resized after construction: pair shapes point into it
    std::vector<int>                         m_group;   // per part: index into m_groups, -1 when unique
    int                                      m_w = 0, m_h = 0;
    bool                                     m_centre = true;              // grow from the anchor, else the min corner
    double                                   m_anchor_x = 0., m_anchor_y = 0.; // cells
    double                                   m_ax = 0., m_ay = 0.;         // growth origin of the current pass, cells
    double                                   m_lambda = 0.;        // envelope cells^2 per contact cell
    int                                      m_obstacle_plates = 0;
    bool                                     m_stopped = false;
    std::function<void(size_t, const std::function<void(size_t)> &)> m_parallel_for;
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
        for (int cy = y0; cy <= y1; ++cy)
            for (int cx = x0; cx <= x1; ++cx) {
                ClipPoly a, b;
                int n = clip(tri, 3, a, false, cx, 1.);
                n = clip(a, n, b, false, cx + 1, -1.);
                n = clip(b, n, a, true, cy, 1.);
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
            }
    }
    if (fill_to_bed)
        for (Span &s : m.cells)
            if (!s.empty())
                s.lo = 0.f;
    return m;
}

HeightMap dilate(const HeightMap &m, double radius_mm, double bottom_only_radius_mm, double bottom_only_mm)
{
    // Offsets of the cells that come closer than `radius_mm` to a cell, measured between
    // the nearest points of the two squares: geometry may reach a cell's edge, so this is
    // the smallest disc that still guarantees the clearance.
    auto disc = [&](double radius) {
        std::vector<std::pair<int, int>> d;
        const double r_cells = radius / m.cell_mm;
        const int    r       = std::max(0, int(std::ceil(r_cells - 1e-9)));
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                const int gx = std::max(std::abs(dx) - 1, 0), gy = std::max(std::abs(dy) - 1, 0);
                if ((dx == 0 && dy == 0) || gx * gx + gy * gy < r_cells * r_cells - 1e-9)
                    d.emplace_back(dx, dy);
            }
        return std::make_pair(r, d);
    };
    auto [r_main, main_disc] = disc(radius_mm);
    auto [r_bottom, bottom_disc] = disc(bottom_only_mm > 0. ? bottom_only_radius_mm : 0.);
    if (bottom_only_mm <= 0.)
        r_bottom = 0, bottom_disc.clear();
    const int g = std::max(r_main, r_bottom);

    HeightMap out(m.width + 2 * g, m.height + 2 * g, m.cell_mm);
    out.origin_x = m.origin_x - g * m.cell_mm;
    out.origin_y = m.origin_y - g * m.cell_mm;
    for (int y = 0; y < m.height; ++y)
        for (int x = 0; x < m.width; ++x) {
            const Span &s = m.at(x, y);
            if (s.empty())
                continue;
            for (auto [dx, dy] : main_disc)
                out.at(x + g + dx, y + g + dy).add(s.lo, s.hi);
            // Brim-like growth starts from the first-layer footprint only.
            if (s.lo <= bottom_only_mm)
                for (auto [dx, dy] : bottom_disc)
                    out.at(x + g + dx, y + g + dy).add(0.f, float(bottom_only_mm));
        }
    return out;
}

// Cost, with N parts, L open plates, C candidates per part, P grid positions per plate and A
// cells per part map: every greedy pass is O(N * L * C * P * A / 64) (a position costs O(1) when
// its score bound prunes it, else a bitset collision test and contact count), and there are at
// most 6 orderings plus 2 lattice-seeded passes. Lattices cost per group of copies what
// build_clusters() states. Polynomial throughout, with no permutation or subset enumeration.
std::vector<Placement> arrange(const std::vector<Part> &parts, const std::vector<Obstacle> &obstacles, const Params &params, Stats *stats)
{
    return Arranger(parts, obstacles, params).run(stats);
}

}} // namespace Slic3r::nest3d
