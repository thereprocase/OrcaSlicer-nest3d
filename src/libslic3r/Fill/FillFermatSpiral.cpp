#include "FillMorphingSpiral.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/libslic3r.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

// Connected Fermat spiral (after Zhao et al., "Connected Fermat Spirals for Layered Fabrication",
// SIGGRAPH 2016): one continuous variable-width path for an island of any shape, holes included.
// It is the second choice after the radial morphing spiral, which has no U-turns but needs one
// bead count around the whole island.
//
// 1. Rings: offsets of the island at depths w/2 + k w, each opened by w/2 so that no part of a ring
//    region is thinner than one bead. Without the opening a thin part prints as a pinched hairpin
//    with both beads on top of each other; with it every ring end is a U-turn of full track width.
// 2. Strands: chains of rings in which each ring has exactly one ring of the same kind (shrinking
//    outer loop or growing hole loop) at the next depth. A strand is a spirallable region.
// 3. The root strand (the one holding the outer edge) is a plain inward spiral: every ring the same
//    way round with a one-lane step in a shared zone, so it has no U-turn. Every other strand is a
//    Fermat spiral that goes in on its even rings and comes back out on its odd rings, so both of
//    its ends lie on its first ring; lane changes are smoothstep blends two lanes wide.
// 4. Strands are joined along a spanning tree: a child whose end ring lies next to a ring of its
//    parent is spliced into that ring's traversal, entered with a U-turn and left with a lane change.
// 5. Widths: an innermost bead (no bead of the next depth within reach) widens or narrows to meet
//    the medial axis, and the change is shared by every bead outward along the same line, so all
//    neighbouring edges stay shared.
//
// The Python reference this mirrors and its shape lab are kept outside the repository.

namespace Slic3r {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double step_mm = .08;            // ring resampling step
constexpr double outline_step_mm = .04;    // outline sampling for depths and normals
constexpr double open_fraction = .5;       // opening radius as a fraction of the spacing
constexpr double seam_reach = 3.1;         // farthest two neighbouring rings can be, in spacings
constexpr double zone_spacings = 2.;       // lane-change zone length, in spacings
constexpr double inner_reach = 1.6;        // a deeper bead this close (in spacings) is inside; covers mitres
constexpr double smooth_radius_mm = 1.5;   // seam leftovers are averaged over innermost beads this close
constexpr double seam_overlap_mm = .03;    // each innermost bead reaches past the medial axis, like infill overlap
constexpr double min_segment_mm = .05;
constexpr size_t npos = std::numeric_limits<size_t>::max();

struct FermatError
{
    std::string reason;
};

Vec2d mm(const Point& p) { return Vec2d(unscale<double>(p.x()), unscale<double>(p.y())); }
double cross(const Vec2d& a, const Vec2d& b) { return a.x() * b.y() - a.y() * b.x(); }

double smoothstep(double s)
{
    s = std::clamp(s, 0., 1.);
    return s * s * (3. - 2. * s);
}

// Uniform grid of points for nearest-point and radius queries.
class PointGrid
{
public:
    PointGrid(std::vector<Vec2d> points, double cell) : m_points(std::move(points)), m_cell(cell)
    {
        for (size_t i = 0; i < m_points.size(); ++i) {
            const auto [x, y] = cell_of(m_points[i]);
            m_cells[key(x, y)].push_back(i);
            m_min_x = std::min(m_min_x, x);
            m_max_x = std::max(m_max_x, x);
            m_min_y = std::min(m_min_y, y);
            m_max_y = std::max(m_max_y, y);
        }
    }
    const std::vector<Vec2d>& points() const { return m_points; }
    size_t size() const { return m_points.size(); }

    // Index of the nearest point, or npos when there are no points.
    size_t nearest(const Vec2d& p, double* distance = nullptr) const
    {
        if (m_points.empty())
            return npos;
        const auto [cx, cy] = cell_of(p);
        size_t best = npos;
        double best_d2 = std::numeric_limits<double>::max();
        const int64_t reach = std::max({std::abs(cx - m_min_x), std::abs(cx - m_max_x), std::abs(cy - m_min_y),
                                        std::abs(cy - m_max_y)});
        auto visit = [&](int64_t x, int64_t y) {
            const auto it = m_cells.find(key(x, y));
            if (it == m_cells.end())
                return;
            for (size_t i : it->second) {
                const double d2 = (m_points[i] - p).squaredNorm();
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best    = i;
                }
            }
        };
        for (int64_t r = 0; r <= reach; ++r) {
            // The ring of cells at Chebyshev distance r from the query cell.
            if (r == 0)
                visit(cx, cy);
            for (int64_t x = cx - r; r > 0 && x <= cx + r; ++x) {
                visit(x, cy - r);
                visit(x, cy + r);
            }
            for (int64_t y = cy - r + 1; r > 0 && y <= cy + r - 1; ++y) {
                visit(cx - r, y);
                visit(cx + r, y);
            }
            // Every point outside the searched square is at least r cells away.
            if (best != npos && std::sqrt(best_d2) <= double(r) * m_cell)
                break;
        }
        if (distance)
            *distance = std::sqrt(best_d2);
        return best;
    }

    template<typename Fn> void within(const Vec2d& p, double radius, Fn fn) const
    {
        const auto [x0, y0] = cell_of(p - Vec2d(radius, radius));
        const auto [x1, y1] = cell_of(p + Vec2d(radius, radius));
        for (int64_t x = x0; x <= x1; ++x)
            for (int64_t y = y0; y <= y1; ++y) {
                const auto it = m_cells.find(key(x, y));
                if (it == m_cells.end())
                    continue;
                for (size_t i : it->second)
                    if ((m_points[i] - p).squaredNorm() <= radius * radius)
                        fn(i);
            }
    }

private:
    std::pair<int64_t, int64_t> cell_of(const Vec2d& p) const
    {
        return {int64_t(std::floor(p.x() / m_cell)), int64_t(std::floor(p.y() / m_cell))};
    }
    static uint64_t key(int64_t x, int64_t y) { return (uint64_t(uint32_t(int32_t(x))) << 32) | uint32_t(int32_t(y)); }

    std::vector<Vec2d> m_points;
    double m_cell;
    std::unordered_map<uint64_t, std::vector<size_t>> m_cells;
    int64_t m_min_x = std::numeric_limits<int64_t>::max(), m_max_x = std::numeric_limits<int64_t>::min();
    int64_t m_min_y = std::numeric_limits<int64_t>::max(), m_max_y = std::numeric_limits<int64_t>::min();
};

std::vector<Vec2d> resample(const std::vector<Vec2d>& loop, double step)
{
    std::vector<double> cum{0.};
    for (size_t i = 0; i < loop.size(); ++i)
        cum.push_back(cum.back() + (loop[(i + 1) % loop.size()] - loop[i]).norm());
    const double length = cum.back();
    const size_t n = std::max<size_t>(8, size_t(std::ceil(length / step)));
    std::vector<Vec2d> out;
    out.reserve(n);
    size_t i = 0;
    for (size_t k = 0; k < n; ++k) {
        const double s = length * double(k) / double(n);
        while (i + 1 < loop.size() && cum[i + 1] < s)
            ++i;
        const double span = cum[i + 1] - cum[i];
        const double t = span > 0 ? (s - cum[i]) / span : 0.;
        out.emplace_back(loop[i] + (loop[(i + 1) % loop.size()] - loop[i]) * t);
    }
    return out;
}

std::vector<Vec2d> to_mm(const Polygon& polygon)
{
    std::vector<Vec2d> out;
    out.reserve(polygon.points.size());
    for (const Point& p : polygon.points)
        out.emplace_back(mm(p));
    return out;
}

struct Ring
{
    Polygon polygon; // as offset, for containment tests
    int level = 0;
    bool outer = true; // shrinking outer loop, or growing hole loop
    std::vector<Vec2d> pts;
    std::vector<double> cum;
    std::unique_ptr<PointGrid> grid;

    void finish()
    {
        cum.assign(1, 0.);
        for (size_t i = 0; i < pts.size(); ++i)
            cum.push_back(cum.back() + (pts[(i + 1) % pts.size()] - pts[i]).norm());
        grid = std::make_unique<PointGrid>(pts, .5);
    }
    double length() const { return cum.back(); }
    void reverse()
    {
        std::reverse(pts.begin(), pts.end());
        finish();
    }
    Vec2d at(double s) const
    {
        const double L = length();
        s = std::fmod(s, L);
        if (s < 0)
            s += L;
        size_t i = size_t(std::upper_bound(cum.begin(), cum.end(), s) - cum.begin());
        i = std::clamp<size_t>(i, 1, pts.size()) - 1;
        const double span = cum[i + 1] - cum[i];
        const double t = span > 0 ? (s - cum[i]) / span : 0.;
        return pts[i] + (pts[(i + 1) % pts.size()] - pts[i]) * t;
    }
    double nearest_s(const Vec2d& p) const { return cum[grid->nearest(p)]; }
    double distance(const Vec2d& p) const
    {
        double d = 0;
        grid->nearest(p, &d);
        return d;
    }
    Vec2d tangent(double s) const { return at(s + .05) - at(s - .05); }
    std::vector<Vec2d> arc(double s0, double length, int direction) const
    {
        const size_t n = std::max<size_t>(2, size_t(std::ceil(length / step_mm)) + 1);
        std::vector<Vec2d> out;
        out.reserve(n);
        for (size_t k = 0; k < n; ++k)
            out.emplace_back(at(s0 + direction * length * double(k) / double(n - 1)));
        return out;
    }
};

using Levels = std::vector<std::vector<std::unique_ptr<Ring>>>;

Levels offset_rings(const ExPolygon& domain, double w)
{
    Levels levels;
    const double opening = open_fraction * w;
    const double arc_tolerance = scale_(.01);
    for (int k = 0;; ++k) {
        if (k > 2000)
            throw FermatError{"too many rings"};
        const double d = w / 2 + k * w;
        const Polygons shrunk = offset(domain, -float(scale_(d + opening)), jtRound, arc_tolerance);
        if (shrunk.empty())
            break;
        const Polygons opened = offset(shrunk, float(scale_(opening)), jtRound, arc_tolerance);
        std::vector<std::unique_ptr<Ring>> loops;
        for (const Polygon& polygon : opened) {
            if (polygon.points.size() < 3)
                continue;
            const std::vector<Vec2d> loop = to_mm(polygon);
            double perimeter = 0;
            for (size_t i = 0; i < loop.size(); ++i)
                perimeter += (loop[(i + 1) % loop.size()] - loop[i]).norm();
            // A loop under two track widths across has nothing inside it to fill and prints as a
            // curl; the beads outside it absorb its strip instead.
            if (perimeter < 2 * pi * w)
                continue;
            auto ring = std::make_unique<Ring>();
            ring->polygon = polygon;
            ring->level   = k;
            ring->outer   = polygon.is_counter_clockwise();
            ring->pts     = resample(loop, step_mm);
            ring->finish();
            loops.emplace_back(std::move(ring));
        }
        if (loops.empty())
            break;
        levels.emplace_back(std::move(loops));
    }
    return levels;
}

using Strand = std::vector<Ring*>;

std::vector<Strand> build_strands(const Levels& levels)
{
    std::map<const Ring*, std::vector<Ring*>> succ, pred;
    for (size_t k = 0; k + 1 < levels.size(); ++k)
        for (const auto& a : levels[k])
            for (const auto& b : levels[k + 1]) {
                if (a->outer != b->outer)
                    continue;
                const bool linked = a->outer ? a->polygon.contains(b->polygon.points.front()) :
                                               b->polygon.contains(a->polygon.points.front());
                if (linked) {
                    succ[a.get()].push_back(b.get());
                    pred[b.get()].push_back(a.get());
                }
            }
    auto list = [](const std::map<const Ring*, std::vector<Ring*>>& m, const Ring* r) -> const std::vector<Ring*>& {
        static const std::vector<Ring*> none;
        const auto it = m.find(r);
        return it == m.end() ? none : it->second;
    };
    std::vector<Strand> strands;
    std::map<const Ring*, bool> seen;
    for (const auto& level : levels)
        for (const auto& r : level) {
            if (seen[r.get()])
                continue;
            const auto& p = list(pred, r.get());
            if (p.size() == 1 && list(succ, p.front()).size() == 1)
                continue; // inside a chain; it is collected from the chain's head
            Strand chain{r.get()};
            seen[r.get()] = true;
            Ring* cur = r.get();
            for (;;) {
                const auto& s = list(succ, cur);
                if (s.size() != 1 || list(pred, s.front()).size() != 1)
                    break;
                cur = s.front();
                chain.push_back(cur);
                seen[cur] = true;
            }
            strands.emplace_back(std::move(chain));
        }
    return strands;
}

struct Attachment
{
    size_t parent;
    size_t host_ring; // index in the parent strand
    bool reversed;    // the child attaches by its last ring
};

double bbox_gap(const Ring& a, const Ring& b)
{
    const BoundingBox ba = a.polygon.bounding_box(), bb = b.polygon.bounding_box();
    const double dx = std::max({0., unscale<double>(ba.min.x() - bb.max.x()), unscale<double>(bb.min.x() - ba.max.x())});
    const double dy = std::max({0., unscale<double>(ba.min.y() - bb.max.y()), unscale<double>(bb.min.y() - ba.max.y())});
    return std::hypot(dx, dy);
}

// Breadth-first spanning tree. A child attaches by one of its end rings to any ring of its parent
// that runs alongside it for at least three spacings.
std::pair<size_t, std::map<size_t, Attachment>> build_tree(const std::vector<Strand>& strands, double w)
{
    size_t root = npos;
    for (size_t i = 0; i < strands.size() && root == npos; ++i)
        if (strands[i].front()->level == 0 && strands[i].front()->outer)
            root = i;
    if (root == npos)
        throw FermatError{"no outer ring"};
    std::map<size_t, Attachment> parent;
    std::vector<bool> placed(strands.size(), false);
    placed[root] = true;
    std::deque<size_t> queue{root};
    const double reach = seam_reach * w;
    while (!queue.empty()) {
        const size_t a = queue.front();
        queue.pop_front();
        for (size_t b = 0; b < strands.size(); ++b) {
            if (placed[b])
                continue;
            std::optional<std::tuple<size_t, bool, size_t>> best; // (run length, reversed, host ring)
            for (bool reversed : {false, true}) {
                const Ring& ring_b = reversed ? *strands[b].back() : *strands[b].front();
                for (size_t hi = 0; hi < strands[a].size(); ++hi) {
                    const Ring& ring_a = *strands[a][hi];
                    if (std::abs(ring_a.level - ring_b.level) > 1 || bbox_gap(ring_a, ring_b) > reach)
                        continue;
                    size_t count = 0;
                    for (const Vec2d& p : ring_a.pts)
                        count += ring_b.distance(p) <= reach;
                    if (double(count) * step_mm >= 3 * w && (!best || count > std::get<0>(*best)))
                        best = std::make_tuple(count, reversed, hi);
                }
            }
            if (best) {
                parent[b] = Attachment{a, std::get<2>(*best), std::get<1>(*best)};
                placed[b] = true;
                queue.push_back(b);
            }
        }
    }
    if (parent.size() + 1 != strands.size())
        throw FermatError{"strands could not be connected"};
    return {root, std::move(parent)};
}

// Interior points of a smooth U-turn from p to q: a cubic Hermite curve leaving p along the travel
// direction and arriving at q against it. For q beside p at one lane spacing it is close to a
// semicircle, so the turning bead keeps track width; unlike a circular arc it cannot loop.
std::vector<Vec2d> uturn(const Vec2d& p, const Vec2d& q, const Vec2d& forward)
{
    const double dist = (q - p).norm();
    if (dist < 1e-6)
        return {};
    const Vec2d f = forward / std::max(forward.norm(), 1e-9) * (1.3 * dist);
    const size_t n = std::max<size_t>(4, size_t(std::ceil(1.6 * dist / step_mm)));
    std::vector<Vec2d> out;
    for (size_t k = 1; k < n; ++k) {
        const double s = double(k) / double(n);
        const double h00 = 2 * s * s * s - 3 * s * s + 1, h10 = s * s * s - 2 * s * s + s;
        const double h01 = -2 * s * s * s + 3 * s * s, h11 = s * s * s - s * s;
        out.emplace_back(p * h00 + f * h10 + q * h01 - f * h11);
    }
    return out;
}

struct Path
{
    std::vector<Vec2d> pts;
    std::vector<double> levels;

    void add(const std::vector<Vec2d>& points, double level)
    {
        append(pts, points);
        levels.insert(levels.end(), points.size(), level);
    }
    void add(const std::vector<Vec2d>& points, const std::vector<double>& point_levels)
    {
        append(pts, points);
        append(levels, point_levels);
    }
    void extend(const Path& other) { add(other.pts, other.levels); }
};

class Builder
{
public:
    Builder(std::vector<Strand>& strands, const std::map<size_t, Attachment>& parent, double w)
        : m_strands(strands), m_w(w), m_zone(zone_spacings * w)
    {
        for (const auto& [child, attachment] : parent) {
            if (attachment.reversed)
                std::reverse(m_strands[child].begin(), m_strands[child].end());
            m_children[{attachment.parent, attachment.host_ring}].push_back(child);
        }
        const size_t n = std::max<size_t>(8, size_t(std::ceil(m_zone / step_mm)));
        for (size_t k = 0; k < n; ++k) {
            m_u.push_back(double(k) / double(n - 1));
            m_s.push_back(smoothstep(m_u.back()));
        }
    }

    // Plain inward spiral of the root strand: every ring the same way round with a one-lane step in
    // the zone, so it has no U-turn. The fill ends on its innermost ring.
    Path spiral_path(size_t si, double a0)
    {
        const Strand& rings = m_strands[si];
        const auto g = gaps(si);
        std::vector<double> a, b;
        cut(si, a0, g, a, b);
        Path path;
        for (size_t i = 0; i < rings.size(); ++i) {
            const Ring& r = *rings[i];
            path.extend(splice_children(si, i, r.arc(a[i], r.length() - g[i], +1)));
            if (i + 1 < rings.size()) {
                std::vector<Vec2d> pts;
                std::vector<double> lv;
                for (size_t k = 1; k + 1 < m_u.size(); ++k) {
                    const Vec2d pa = r.at(b[i] + m_u[k] * g[i]);
                    const Vec2d pb = rings[i + 1]->at(b[i + 1] + m_u[k] * g[i + 1]);
                    pts.emplace_back(pa * (1 - m_s[k]) + pb * m_s[k]);
                    lv.emplace_back(r.level * (1 - m_s[k]) + rings[i + 1]->level * m_s[k]);
                }
                path.add(pts, lv);
            }
        }
        return path;
    }

private:
    std::vector<double> gaps(size_t si) const
    {
        std::vector<double> g;
        for (const Ring* r : m_strands[si])
            g.push_back(std::min(m_zone, r->length() / 4));
        return g;
    }

    // Lane-change zone of each ring: it runs from b (where the ring's arc ends) to a = b + gap, and
    // the zones of all rings line up along one spoke from the strand's start.
    void cut(size_t si, double a0, const std::vector<double>& g, std::vector<double>& a, std::vector<double>& b) const
    {
        const Strand& rings = m_strands[si];
        a.assign(rings.size(), 0.);
        b.assign(rings.size(), 0.);
        a[0] = a0;
        b[0] = a0 - g[0];
        for (size_t i = 1; i < rings.size(); ++i) {
            b[i] = rings[i]->nearest_s(rings[i - 1]->at(b[i - 1]));
            a[i] = b[i] + g[i];
        }
    }

    // Blend from ring i to ring j across the zone, travelling forward (+1) or backward (-1).
    void lane_change(const Strand& rings, const std::vector<double>& b, const std::vector<double>& g, size_t i, size_t j,
                     int direction, Path& path) const
    {
        std::vector<Vec2d> pts;
        std::vector<double> lv;
        for (size_t k = 1; k + 1 < m_u.size(); ++k) {
            const double x = direction > 0 ? m_u[k] : 1 - m_u[k];
            const Vec2d pa = rings[i]->at(b[i] + x * g[i]);
            const Vec2d pb = rings[j]->at(b[j] + x * g[j]);
            pts.emplace_back(pa * (1 - m_s[k]) + pb * m_s[k]);
            lv.emplace_back(rings[i]->level * (1 - m_s[k]) + rings[j]->level * m_s[k]);
        }
        path.add(pts, lv);
    }

    // Fermat spiral of a child strand from arc position a0 on its ring 0: even rings forward going
    // in, odd rings backward coming out. It ends back on the host ring: exit(x) is the host point a
    // fraction x through the splice.
    Path strand_path(size_t si, double a0, const std::function<Vec2d(double)>& exit, double exit_level)
    {
        Strand& rings = m_strands[si];
        const size_t n = rings.size();
        const auto g = gaps(si);
        std::vector<double> a, b;
        cut(si, a0, g, a, b);
        auto ring_arc = [&](size_t i) {
            const Ring& r = *rings[i];
            return splice_children(si, i, i % 2 == 0 ? r.arc(a[i], r.length() - g[i], +1) : r.arc(b[i], r.length() - g[i], -1));
        };
        Path path;
        std::vector<size_t> evens, odds;
        for (size_t i = 0; i < n; ++i)
            (i % 2 == 0 ? evens : odds).push_back(i);
        for (size_t k = 0; k < evens.size(); ++k) {
            path.extend(ring_arc(evens[k]));
            if (k + 1 < evens.size())
                lane_change(rings, b, g, evens[k], evens[k + 1], +1, path);
        }
        if (!odds.empty()) {
            const size_t e = evens.back(), o = odds.back();
            path.add(uturn(rings[e]->at(b[e]), rings[o]->at(b[o]), rings[e]->tangent(b[e])), .5 * (rings[e]->level + rings[o]->level));
            for (size_t k = odds.size(); k-- > 0;) {
                path.extend(ring_arc(odds[k]));
                if (odds[k] >= 3)
                    lane_change(rings, b, g, odds[k], odds[k] - 2, -1, path);
            }
            // Back to the host: a two-lane blend from ring 1 onto the host ring.
            std::vector<Vec2d> pts;
            std::vector<double> lv;
            for (size_t k = 1; k < m_u.size(); ++k) {
                const Vec2d pa = rings[1]->at(b[1] + (1 - m_u[k]) * g[1]);
                pts.emplace_back(pa * (1 - m_s[k]) + exit(m_u[k]) * m_s[k]);
                lv.emplace_back(rings[1]->level * (1 - m_s[k]) + exit_level * m_s[k]);
            }
            path.add(pts, lv);
        } else {
            const Vec2d h2 = exit(1.);
            path.add(uturn(rings[0]->at(b[0]), h2, rings[0]->tangent(b[0])), rings[0]->level - .5);
            path.add({h2}, exit_level);
        }
        return path;
    }

    Path splice_children(size_t si, size_t ring_index, const std::vector<Vec2d>& arc)
    {
        const double level = m_strands[si][ring_index]->level;
        Path out;
        const auto kids = m_children.find({si, ring_index});
        if (kids == m_children.end()) {
            out.add(arc, level);
            return out;
        }
        std::vector<double> t_arc{0.};
        for (size_t i = 1; i < arc.size(); ++i)
            t_arc.push_back(t_arc.back() + (arc[i] - arc[i - 1]).norm());
        const double total = t_arc.back();
        const double margin = 1.5 * m_zone;
        auto arc_at = [&](double t) {
            const size_t i = std::clamp<size_t>(size_t(std::upper_bound(t_arc.begin(), t_arc.end(), t) - t_arc.begin()), 1, arc.size() - 1);
            const double span = t_arc[i] - t_arc[i - 1];
            const double x = span > 0 ? std::clamp((t - t_arc[i - 1]) / span, 0., 1.) : 0.;
            return Vec2d(arc[i - 1] + (arc[i] - arc[i - 1]) * x);
        };
        struct Insert
        {
            double t1, t2;
            Vec2d h1, host_dir, start;
            Path child;
        };
        std::vector<std::pair<double, double>> taken;
        std::vector<Insert> inserts;
        for (size_t c : kids->second) {
            Ring& child_ring = *m_strands[c].front();
            const double gc = gaps(c).front();
            std::vector<double> d(arc.size());
            std::vector<size_t> idx;
            for (size_t i = 0; i < arc.size(); ++i) {
                d[i] = child_ring.distance(arc[i]);
                bool ok = d[i] <= seam_reach * m_w && t_arc[i] > margin && t_arc[i] < total - margin - gc;
                for (const auto& [t0, t1] : taken)
                    ok = ok && (t_arc[i] < t0 - margin || t_arc[i] > t1 + margin);
                if (ok)
                    idx.push_back(i);
            }
            if (idx.empty())
                throw FermatError{"no room to splice a region"};
            // Enter where the rings are closest, so the entry U-turn stays near track width.
            double closest = std::numeric_limits<double>::max();
            for (size_t i : idx)
                closest = std::min(closest, d[i]);
            idx.erase(std::remove_if(idx.begin(), idx.end(), [&](size_t i) { return d[i] > closest + .15 * m_w; }), idx.end());
            // Middle of the longest run of consecutive admissible positions.
            size_t best_start = 0, best_len = 0;
            for (size_t s = 0; s < idx.size();) {
                size_t e = s + 1;
                while (e < idx.size() && idx[e] == idx[e - 1] + 1)
                    ++e;
                if (e - s > best_len) {
                    best_len   = e - s;
                    best_start = s;
                }
                s = e;
            }
            const size_t j = idx[best_start + best_len / 2];
            const double t1 = t_arc[j], t2 = t1 + gc;
            taken.emplace_back(t1, t2);
            const Vec2d h1 = arc[j];
            const Vec2d host_dir = arc[std::min(j + 1, arc.size() - 1)] - arc[j == 0 ? 0 : j - 1];
            double a0 = child_ring.nearest_s(h1);
            // The child's first ring runs opposite to the host, so entering it is a U-turn.
            if (child_ring.tangent(a0).dot(host_dir) > 0) {
                for (Ring* r : m_strands[c])
                    r->reverse();
                a0 = child_ring.nearest_s(h1);
            }
            Path child = strand_path(c, a0, [&](double x) { return arc_at(t1 + x * gc); }, level);
            inserts.push_back(Insert{t1, t2, h1, host_dir, child_ring.at(a0), std::move(child)});
        }
        std::sort(inserts.begin(), inserts.end(), [](const Insert& l, const Insert& r) { return l.t1 < r.t1; });
        size_t cur = 0;
        for (const Insert& ins : inserts) {
            const size_t j1 = size_t(std::upper_bound(t_arc.begin(), t_arc.end(), ins.t1) - t_arc.begin());
            const size_t j2 = size_t(std::upper_bound(t_arc.begin(), t_arc.end(), ins.t2) - t_arc.begin());
            out.add(std::vector<Vec2d>(arc.begin() + cur, arc.begin() + std::max(cur, j1)), level);
            out.add(uturn(ins.h1, ins.start, ins.host_dir), level + .5);
            out.extend(ins.child);
            cur = std::max(cur, j2);
        }
        out.add(std::vector<Vec2d>(arc.begin() + std::min(cur, arc.size()), arc.end()), level);
        return out;
    }

    std::vector<Strand>& m_strands;
    double m_w, m_zone;
    std::vector<double> m_u, m_s;
    std::map<std::pair<size_t, size_t>, std::vector<size_t>> m_children;
};

// Bead widths and positions from geometry. The inward normal points away from the nearest point
// of the island outline. A bead is innermost when no bead of the next depth lies within reach; its
// leftover t runs from its edge to the medial depth, found by marching along the normal while the
// distance to the outline keeps rising. The leftover is smoothed along the seam, then shared by
// every bead outward along the same line: each changes by t / n and shifts by the changes outside
// it plus half its own, so neighbouring edges stay shared.
void medial_widths(const ExPolygon& domain, std::vector<Vec2d>& pts, const std::vector<double>& levels, double w,
                   double min_w, double max_w, std::vector<double>& widths)
{
    std::vector<Vec2d> outline = resample(to_mm(domain.contour), outline_step_mm);
    for (const Polygon& hole : domain.holes)
        append(outline, resample(to_mm(hole), outline_step_mm));
    // Coarse cells: beads deep inside a large island would otherwise search many empty cells.
    const PointGrid outline_grid(std::move(outline), 2.);
    const size_t n = pts.size();
    std::vector<Vec2d> normal(n, Vec2d::Zero());
    for (size_t i = 0; i < n; ++i) {
        const Vec2d v = pts[i] - outline_grid.points()[outline_grid.nearest(pts[i])];
        if (v.norm() > 1e-9)
            normal[i] = v / v.norm();
    }
    std::vector<int> lv(n);
    int max_level = 0;
    for (size_t i = 0; i < n; ++i) {
        lv[i] = int(std::lround(levels[i]));
        max_level = std::max(max_level, lv[i]);
    }
    std::vector<std::vector<size_t>> by_level(size_t(max_level) + 1);
    for (size_t i = 0; i < n; ++i)
        by_level[size_t(lv[i])].push_back(i);
    std::vector<bool> innermost(n, true);
    std::vector<size_t> nearest_deeper(n, npos);
    for (int k = 0; k < max_level; ++k) {
        const auto& deeper = by_level[size_t(k + 1)];
        if (deeper.empty())
            continue;
        std::vector<Vec2d> deeper_pts;
        for (size_t i : deeper)
            deeper_pts.push_back(pts[i]);
        const PointGrid grid(std::move(deeper_pts), .5);
        for (size_t i : by_level[size_t(k)]) {
            double d = 0;
            const size_t nn = grid.nearest(pts[i], &d);
            if (d <= inner_reach * w) {
                innermost[i]      = false;
                nearest_deeper[i] = deeper[nn];
            }
        }
    }
    std::vector<double> t(n, 0.);
    std::vector<int> nd(n, 1), j(n, 0);
    std::vector<size_t> inner;
    const double march = .02;
    for (size_t i = 0; i < n; ++i) {
        if (!innermost[i])
            continue;
        inner.push_back(i);
        Vec2d q = pts[i];
        double m = 0;
        outline_grid.nearest(q, &m);
        for (int s = 0; s < int(std::ceil(inner_reach * w / march)); ++s) {
            double next = 0;
            outline_grid.nearest(q + normal[i] * march, &next);
            if (next <= m + 1e-6)
                break;
            q += normal[i] * march;
            m = next;
        }
        t[i]  = std::clamp(m - (w / 2 + levels[i] * w) - w / 2 + seam_overlap_mm, -w / 2, w);
        nd[i] = lv[i] + 1;
    }
    if (!inner.empty()) {
        std::vector<Vec2d> inner_pts;
        for (size_t i : inner)
            inner_pts.push_back(pts[i]);
        const PointGrid grid(std::move(inner_pts), .5);
        std::vector<double> smoothed(inner.size());
        for (size_t k = 0; k < inner.size(); ++k) {
            double sum = 0;
            size_t count = 0;
            grid.within(pts[inner[k]], smooth_radius_mm, [&](size_t m) {
                sum += t[inner[m]];
                ++count;
            });
            smoothed[k] = count ? sum / double(count) : t[inner[k]];
        }
        for (size_t k = 0; k < inner.size(); ++k)
            t[inner[k]] = smoothed[k];
    }
    for (int k = max_level - 1; k >= 0; --k)
        for (size_t i : by_level[size_t(k)])
            if (!innermost[i]) {
                const size_t src = nearest_deeper[i];
                t[i]  = t[src];
                nd[i] = nd[src];
                j[i]  = j[src] + 1;
            }
    widths.resize(n);
    for (size_t i = 0; i < n; ++i) {
        const bool shared = j[i] < nd[i];
        const double delta = shared ? t[i] / nd[i] : 0.;
        widths[i] = std::clamp(w + delta, min_w, max_w);
        pts[i] += normal[i] * (shared ? (nd[i] - j[i] - .5) * delta : 0.);
    }
}

// Root cut: the straightest stretch of the outer ring, where the lane-change zone looks cleanest.
double straightest(const Ring& ring)
{
    const size_t n = ring.pts.size();
    std::vector<double> turn(n);
    for (size_t i = 0; i < n; ++i) {
        const Vec2d t0 = ring.pts[(i + 1) % n] - ring.pts[i];
        const Vec2d t1 = ring.pts[(i + 2) % n] - ring.pts[(i + 1) % n];
        turn[i] = std::abs(cross(t0, t1));
    }
    const int half = 12;
    size_t best = 0;
    double best_sum = std::numeric_limits<double>::max();
    for (size_t i = 0; i < n; ++i) {
        double sum = 0;
        for (int k = -half; k <= half; ++k)
            sum += turn[(i + size_t(int(n) + k)) % n];
        if (sum < best_sum) {
            best_sum = sum;
            best     = i;
        }
    }
    return ring.cum[best];
}
} // namespace

MorphingSpiralResult fermat_spiral(const ExPolygon& domain, double spacing, double min_spacing, double max_spacing)
{
    MorphingSpiralResult result;
    auto reject = [&](std::string reason) {
        result.paths.clear();
        result.fallback_reason = std::move(reason);
        return result;
    };
    if (!std::isfinite(spacing) || !std::isfinite(min_spacing) || !std::isfinite(max_spacing) || min_spacing <= 0 ||
        spacing < min_spacing || spacing > max_spacing)
        return reject("invalid spacing bounds");
    std::vector<Vec2d> pts;
    std::vector<double> levels;
    try {
        Levels rings = offset_rings(domain, spacing);
        if (rings.empty())
            return reject("island too small for one bead");
        std::vector<Strand> strands = build_strands(rings);
        auto [root, parent] = build_tree(strands, spacing);
        Builder builder(strands, parent, spacing);
        Path path = builder.spiral_path(root, straightest(*strands[root].front()));
        pts    = std::move(path.pts);
        levels = std::move(path.levels);
    } catch (const FermatError& e) {
        return reject(e.reason);
    }
    std::vector<double> widths;
    medial_widths(domain, pts, levels, spacing, min_spacing, max_spacing, widths);

    ThickPolyline spiral;
    Vec2d last(std::numeric_limits<double>::quiet_NaN(), 0.);
    for (size_t i = 0; i < pts.size(); ++i) {
        if (!pts[i].allFinite() || !std::isfinite(widths[i]))
            return reject("non-finite path");
        if (i > 0 && i + 1 < pts.size() && (pts[i] - last).norm() < min_segment_mm)
            continue;
        morphing_spiral_detail::append_point(spiral, pts[i], widths[i]);
        last = pts[i];
    }
    if (spiral.points.size() < 2)
        return reject("empty spiral");
    ThickPolylines candidate;
    candidate.emplace_back(std::move(spiral));

    std::optional<morphing_spiral_detail::Coverage> coverage = morphing_spiral_detail::measure_coverage(domain, candidate, result);
    if (!coverage)
        return reject("invalid domain area");
    // Looser than the radial spiral's budget: every ring end and splice leaves a few hundredths of a
    // square millimetre, so the island total is larger even where each gap stays small.
    if (result.largest_gap_mm2 > .45 || result.largest_spill_mm2 > .10 ||
        !offset_ex(coverage->gaps, -float(scale_(spacing * .25))).empty() ||
        !offset_ex(coverage->spills, -float(scale_(spacing * .15))).empty())
        return reject("local gap or spill exceeds area/inscribed-width limit");
    if (result.uncovered_fraction > .012 || result.outside_fraction > .002 || std::abs(result.excess_fraction) > .03)
        return reject("bead footprint coverage or deposition budget failed");
    result.paths = std::move(candidate);
    return result;
}
} // namespace Slic3r
