// Identical copies on lattices.
//
// A lattice is the set of integer combinations of two basis vectors. Many bases span the same lattice,
// so every lattice here is kept in Hermite normal form: a basis (a, 0) and (b, c) with a, c > 0 and
// 0 <= b < a, which is unique for the lattice. Euclid's algorithm on the y components finds it:
// subtracting an integer multiple of one basis vector from the other keeps the lattice, and the steps
// end with one vector on the x axis. In that form the points are easy to walk: rows c apart, points a
// apart within a row, each row shifted by b from the one below. Shifting the whole lattice gives its
// phases, the a * c distinct point sets with a site in [0, a) x [0, c); a * c is also the area each
// point owns, so it measures how dense the lattice is.

#include "Lattice.hpp"
#include "Grid.hpp"
#include "Nest3D.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace Slic3r { namespace nest3d {
namespace {

constexpr size_t    MaxBasis  = 2048;  // touching offsets tried as basis vectors, shortest first
constexpr size_t    MaxUnits  = 4;     // single-copy units, which bounds the no-fit sets a shape costs
constexpr long long MaxPhases = 16384; // lattice phases counted when the copies fill the plate
constexpr double    Pi        = 3.14159265358979323846;

long long floor_mod(long long x, long long m) { return (x % m + m) % m; }

// Raster no-fit set of `other` around `base`: the offsets of other's tight rect from base's at which the
// two collide, by Board::fits on a scratch board, so copies tuck under overhangs exactly as they do on
// the plate.
struct NoFit
{
    const Footprint     &other;
    Board                board;        // base at (other.w + 1, other.h + 1): the spare cell keeps other's halo on it
    int                  x0, y0, w, h; // the window of offsets that can collide
    std::vector<uint8_t> collides;     // per offset of the window

    NoFit(const Footprint &base, const Footprint &other_, double gap, const ParallelFor &pf)
        : other(other_), board(base.w + 2 * other.w + 2, base.h + 2 * other.h + 2, gap)
        , x0(1 - other.w), y0(1 - other.h), w(base.w + other.w - 1), h(base.h + other.h - 1)
        , collides(size_t(w) * h)
    {
        board.place(base, other.w + 1, other.h + 1);
        // Window cell (x, y) is offset (x0 + x, y0 + y), which puts `other` at board cell (x + 2, y + 2).
        pf(size_t(h), [&](size_t y) {
            for (int x = 0; x < w; ++x)
                collides[y * w + x] = !board.fits(other, x + 2, int(y) + 2);
        });
    }
    bool blocked(long long x, long long y) const
    {
        return x >= x0 && y >= y0 && x < x0 + w && y < y0 + h && collides[size_t((y - y0) * w + x - x0)];
    }
    bool touching(int x, int y) const
    {
        if (blocked(x, y))
            return false;
        return blocked(x - 1, y) || blocked(x + 1, y) || blocked(x, y - 1) || blocked(x, y + 1);
    }
};

struct Vec { long long x, y; };

// The Hermite normal form of the lattice spanned by u and v (see the top of the file).
Lattice hermite(Vec u, Vec v)
{
    while (v.y != 0) {
        const long long quotient = u.y / v.y;
        u                        = {u.x - quotient * v.x, u.y % v.y};
        std::swap(u, v);
    }
    // v lies on the x axis now. The other vector is flipped to point up and its x taken modulo a.
    const long long sign = u.y < 0 ? -1 : 1;
    const long long a    = std::abs(v.x);
    return {a, floor_mod(sign * u.x, a), sign * u.y};
}

// Calls fn(x, y) for the points of the lattice through (px, py) that lie in the cells of r, until fn
// returns false, and returns whether it got through them all.
template<class Fn> bool for_lattice_points(const Lattice &l, long long px, long long py, const Rect &r, Fn fn)
{
    const long long first_row = r.y0 + floor_mod(py - r.y0, l.c);
    for (long long y = first_row; y < r.y1; y += l.c) {
        const long long rows_up   = (y - py) / l.c; // exact: y and py are a whole number of rows apart
        const long long row_start = px + rows_up * l.b;
        const long long first_x   = r.x0 + floor_mod(row_start - r.x0, l.a);
        for (long long x = first_x; x < r.x1; x += l.a)
            if (!fn(x, y))
                return false;
    }
    return true;
}

// The box lattice, then the densest valid lattice when denser. Both are offered because on a finite bed a
// looser lattice can hold more copies (discs: 3 x 3 on a square grid where hexagonal rows hold 3 + 2 + 3).
// Copies on a locally densest lattice touch along two independent lattice vectors, so its basis is a pair
// of touching offsets; v and -v span the same lattices, so one half plane holds them all.
std::vector<Lattice> lattices_of(const Footprint &fp, const NoFit &self)
{
    std::vector<Vec> touching;
    for (int dy = 0; dy <= fp.h; ++dy)
        for (int dx = dy > 0 ? -fp.w : 1; dx <= fp.w; ++dx)
            if (self.touching(dx, dy))
                touching.push_back({dx, dy});
    const auto shorter = [](const Vec &p, const Vec &q) { return p.x * p.x + p.y * p.y < q.x * q.x + q.y * q.y; };
    std::stable_sort(touching.begin(), touching.end(), shorter);
    touching.resize(std::min(touching.size(), MaxBasis));

    const Lattice box{fp.w, 0, fp.h};
    Lattice       densest = box;
    // Valid: no other copy within reach of the one at the origin collides with it. Copies are disjoint, so
    // a lattice holds at least `cells` per point, and one that holds exactly that tiles: nothing beats it.
    const auto clear  = [&](long long x, long long y) { return (x == 0 && y == 0) || !self.blocked(x, y); };
    const auto tiles  = [&] { return densest.a * densest.c <= fp.cells; };
    const Rect within = {1 - fp.w, 1 - fp.h, fp.w, fp.h}; // the offsets at which two copies can collide
    for (size_t i = 0; i < touching.size() && !tiles(); ++i)
        for (size_t j = i + 1; j < touching.size() && !tiles(); ++j) {
            const Vec       u   = touching[i], v = touching[j];
            const long long det = std::abs(u.x * v.y - u.y * v.x); // the lattice's cells per point
            if (det < fp.cells || det >= densest.a * densest.c)
                continue;
            // u + v and u - v collide most often, so they are tested before the whole window.
            if (!clear(u.x + v.x, u.y + v.y) || !clear(u.x - v.x, u.y - v.y))
                continue;
            const Lattice candidate = hermite(u, v);
            if (for_lattice_points(candidate, 0, 0, within, clear))
                densest = candidate;
        }
    if (densest.a * densest.c < box.a * box.c)
        return {box, densest};
    return {box};
}

// Candidates ca and cb with cb's tight rect at (dx, dy) from ca's, their common bbox area and contact.
struct Pair
{
    long area    = std::numeric_limits<long>::max();
    int  contact = 0;
    int  ca = -1, cb = -1;
    int  dx = 0, dy = 0;
};

// Keeps in `best` the touching offset of cb from ca with the smallest common bbox, then the most contact.
void best_pair_offset(int ca, int cb, const NoFit &nofit, const std::vector<Footprint> &fps, Pair &best)
{
    const Footprint &a = fps[ca], &b = fps[cb];
    for (int dy = -b.h; dy <= a.h; ++dy)
        for (int dx = -b.w; dx <= a.w; ++dx) {
            const long width  = std::max(a.w, dx + b.w) - std::min(0, dx);
            const long height = std::max(a.h, dy + b.h) - std::min(0, dy);
            const long area   = width * height;
            if (area > best.area || !nofit.touching(dx, dy)) // contact, the costly part, only for a contender
                continue;
            const int contact = nofit.board.contact(b, b.w + 1 + dx, b.h + 1 + dy);
            if (std::tie(area, best.contact) < std::tie(best.area, contact))
                best = {area, contact, ca, cb, dx, dy};
        }
}

// The pair as one unit: the members' spans stacked on a scratch board are their union.
Unit pair_unit(const std::vector<Footprint> &fps, const Pair &pair, const Params &p)
{
    Unit unit;
    unit.members = {{pair.ca, std::max(0, -pair.dx), std::max(0, -pair.dy)},
                    {pair.cb, std::max(0, pair.dx), std::max(0, pair.dy)}};
    const int width  = std::max(fps[pair.ca].w, pair.dx + fps[pair.cb].w) - std::min(0, pair.dx);
    const int height = std::max(fps[pair.ca].h, pair.dy + fps[pair.cb].h) - std::min(0, pair.dy);
    HeightMap map(width, height, p.cell_mm);
    Board     canvas(map.width, map.height);
    for (const Unit::Member &member : unit.members)
        canvas.place(fps[member.candidate], member.dx, member.dy);
    map.cells     = canvas.spans;
    unit.fp       = make_footprint(map, p.max_height, p.vertical_gap);
    unit.lattices = lattices_of(unit.fp, NoFit(unit.fp, unit.fp, p.vertical_gap, p.parallel_for));
    return unit;
}

// A unit's lattice through bed cell (x, y).
struct Phase
{
    size_t    unit;
    Lattice   lattice;
    long long x, y;
};

// Copies are picked one at a time by the pile they leave: roundest about the anchor (the ring envelope),
// then smallest, then nearest the anchor, then lowest, then leftmost.
struct RoundnessKey
{
    double reach;
    long   area;
    double dist2;
    int    y, x;
    bool   operator<(const RoundnessKey &o) const
    {
        return std::tie(reach, area, dist2, y, x) < std::tie(o.reach, o.area, o.dist2, o.y, o.x);
    }
};

// Up to cap copies of a phase, from the members that fit the board at its sites. A lattice pile grows
// round like the ring pass, and fills one side of a ring before it starts the other.
std::vector<Spot> lay(const Unit &unit, const std::vector<Footprint> &fps, const Board &board, const Phase &phase,
                      const Rect &pile, double anchor_x, double anchor_y, int cap)
{
    std::vector<Spot> fitting;
    const Rect        sites{1 - unit.fp.w, 1 - unit.fp.h, board.w, board.h};
    for_lattice_points(phase.lattice, phase.x, phase.y, sites, [&](long long x, long long y) {
        for (const Unit::Member &member : unit.members)
            if (board.fits(fps[member.candidate], int(x) + member.dx, int(y) + member.dy))
                fitting.push_back({member.candidate, int(x) + member.dx, int(y) + member.dy});
        return true;
    });
    Rect       laid = pile;
    const auto rect = [&](const Spot &s) {
        return Rect{s.x, s.y, s.x + fps[s.candidate].w, s.y + fps[s.candidate].h};
    };
    const auto key  = [&](const Spot &s) {
        const Rect   r       = laid.united(rect(s));
        const double reach_x = std::max(anchor_x - r.x0, r.x1 - anchor_x) / board.w;
        const double reach_y = std::max(anchor_y - r.y0, r.y1 - anchor_y) / board.h;
        const double dx      = s.x + 0.5 * fps[s.candidate].w - anchor_x;
        const double dy      = s.y + 0.5 * fps[s.candidate].h - anchor_y;
        return RoundnessKey{std::max(reach_x, reach_y), r.area(), dx * dx + dy * dy, s.y, s.x};
    };
    const auto        roundest = [&](const Spot &p, const Spot &q) { return key(p) < key(q); };
    std::vector<Spot> out;
    while (int(out.size()) < cap && !fitting.empty()) {
        const auto next = std::min_element(fitting.begin(), fitting.end(), roundest);
        laid            = laid.united(rect(*next));
        out.push_back(*next);
        fitting.erase(next);
    }
    return out;
}

// Copies held by each phase of a unit's lattice, from one pass over the bed: every cell where a member
// fits counts once, for the phase of the site that puts the member there. Phase (i, j), with its site at
// (i, j), is entry j * a + i. Atomic adds commute, so the counts do not depend on the order rows run in.
std::vector<std::atomic<int>> phase_counts(const Unit &unit, const Lattice &l, const std::vector<Footprint> &fps,
                                           const Board &board, const ParallelFor &pf)
{
    std::vector<std::atomic<int>> held(size_t(l.a * l.c));
    pf(unit.members.size() * size_t(board.h), [&](size_t task) {
        const Unit::Member &member = unit.members[task / board.h];
        const int           y      = int(task % board.h);
        // The member at bed cell (x, y) puts its site at (x - dx, y - dy). The site's row fixes the row
        // phase j; along the row the column phase i advances with x, wrapping at a.
        const long long site_y       = y - member.dy;
        const long long row_phase    = floor_mod(site_y, l.c);
        const long long rows_up      = (site_y - row_phase) / l.c;
        long long       column_phase = floor_mod(-member.dx - rows_up * l.b, l.a); // of the site for x = 0
        for (int x = 0; x < board.w; ++x) {
            if (board.fits(fps[member.candidate], x, y))
                ++held[size_t(row_phase * l.a + column_phase)];
            if (++column_phase == l.a)
                column_phase = 0;
        }
    });
    return held;
}

long pile_area_after(const Rect &pile, const std::vector<Spot> &spots, const std::vector<Footprint> &fps)
{
    Rect laid = pile;
    for (const Spot &s : spots)
        laid = laid.united({s.x, s.y, s.x + fps[s.candidate].w, s.y + fps[s.candidate].h});
    return laid.area();
}

} // namespace

std::vector<Unit> lattice_units(const std::vector<Footprint> &fps, const std::vector<Candidate> &cands,
                                const Params &p)
{
    // A unit for each of the MaxUnits usable candidates that fill their rect best, one per distinct footprint
    // (a symmetric part looks the same at several turns), and the best pair of one of them with itself or a
    // half turn of it. A half turn that looks the same pairs as the shape itself does, unless stacking
    // compares spans.
    std::vector<int> by_fill(fps.size());
    std::iota(by_fill.begin(), by_fill.end(), 0);
    const auto fill_ratio = [&](int c) {
        return fps[c].usable ? fps[c].cells / (double(fps[c].w) * fps[c].h) : 0.;
    };
    std::stable_sort(by_fill.begin(), by_fill.end(), [&](int c, int k) { return fill_ratio(c) > fill_ratio(k); });
    const auto same_footprint = [&](int c, int k) { return fps[k].w == fps[c].w && fps[k].bits == fps[c].bits; };
    const auto half_turn_of   = [&](int c, int k) {
        return std::abs(std::remainder(cands[k].rotation_z - cands[c].rotation_z - Pi, 2. * Pi)) < 1e-6;
    };
    std::vector<Unit> units;
    Pair              best_pair;
    for (int c : by_fill) {
        const auto already = [&](const Unit &u) { return same_footprint(c, u.members[0].candidate); };
        if (!fps[c].usable || units.size() == MaxUnits || std::any_of(units.begin(), units.end(), already))
            continue;
        const NoFit self(fps[c], fps[c], p.vertical_gap, p.parallel_for);
        units.push_back({{{c, 0, 0}}, fps[c], lattices_of(fps[c], self)});
        best_pair_offset(c, c, self, fps, best_pair);
        for (int k = 0; k < int(fps.size()); ++k) {
            if (!fps[k].usable || !half_turn_of(c, k))
                continue;
            if (same_footprint(c, k) && std::isinf(p.vertical_gap))
                continue;
            best_pair_offset(c, k, NoFit(fps[c], fps[k], p.vertical_gap, p.parallel_for), fps, best_pair);
        }
    }
    if (best_pair.ca >= 0)
        units.push_back(pair_unit(fps, best_pair, p));
    return units;
}

std::vector<Spot> lay_lattice(const std::vector<Unit> &units, const std::vector<Footprint> &fps, const Board &board,
                              const Rect &pile, double anchor_x, double anchor_y, int cap, const ParallelFor &pf)
{
    // How phases compare, best first: all cap copies beat fewer, then the smaller pile, then more
    // copies, then the denser lattice per copy; ties keep the earlier unit, lattice and phase.
    using PhaseKey = std::tuple<bool, long, int, double>;
    std::optional<Phase> best;
    PhaseKey             best_key;
    std::vector<Spot>    best_spots; // when the best phase was laid; otherwise laid at the end
    const auto           offer = [&](const Phase &phase, int copies, const std::vector<Spot> &spots) {
        if (copies == 0)
            return;
        const Lattice &l              = phase.lattice;
        const double   cells_per_copy = double(l.a * l.c) / double(units[phase.unit].members.size());
        const long     pile_area      = copies < cap ? 0 : pile_area_after(pile, spots, fps);
        const PhaseKey key{copies < cap, pile_area, -copies, cells_per_copy};
        if (!best || key < best_key) {
            best       = phase;
            best_key   = key;
            best_spots = spots;
        }
    };
    for (size_t u = 0; u < units.size(); ++u)
        for (const Lattice &l : units[u].lattices) {
            // First the phase that centres a site on the anchor. When it falls short of cap, every phase is
            // counted, and every stride-th one offered past MaxPhases.
            const Footprint &fp = units[u].fp;
            const Phase      anchored{u, l, std::llround(anchor_x - 0.5 * fp.w), std::llround(anchor_y - 0.5 * fp.h)};
            const std::vector<Spot> spots = lay(units[u], fps, board, anchored, pile, anchor_x, anchor_y, cap);
            offer(anchored, int(spots.size()), spots);
            if (int(spots.size()) == cap)
                continue;
            const std::vector<std::atomic<int>> counts = phase_counts(units[u], l, fps, board, pf);
            const long long                     stride = (l.a * l.c + MaxPhases - 1) / MaxPhases;
            for (long long t = 0; t < l.a * l.c; t += stride) {
                const Phase phase{u, l, t % l.a, t / l.a};
                const int   copies = std::min<int>(counts[size_t(t)], cap);
                std::vector<Spot> laid; // the pile's area only matters for a phase holding all cap copies
                if (copies == cap)
                    laid = lay(units[u], fps, board, phase, pile, anchor_x, anchor_y, cap);
                offer(phase, copies, laid);
            }
        }
    if (!best)
        return {};
    if (!best_spots.empty())
        return best_spots;
    return lay(units[best->unit], fps, board, *best, pile, anchor_x, anchor_y, cap);
}

}} // namespace Slic3r::nest3d
