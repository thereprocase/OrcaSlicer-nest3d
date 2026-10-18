#include "FillMorphingSpiral.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/libslic3r.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
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
constexpr double sliver_mm = .05;          // gaps thinner than this are not counted
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

    // Index of the nearest point, or npos when there are none (within max_distance, if given: the
    // search stops there, which keeps "is anything close?" queries from scanning the whole grid).
    size_t nearest(const Vec2d& p, double* distance = nullptr, double max_distance = std::numeric_limits<double>::infinity()) const
    {
        if (distance)
            *distance = std::numeric_limits<double>::infinity();
        if (m_points.empty())
            return npos;
        const auto [cx, cy] = cell_of(p);
        size_t best = npos;
        double best_d2 = std::numeric_limits<double>::max();
        int64_t reach = std::max({std::abs(cx - m_min_x), std::abs(cx - m_max_x), std::abs(cy - m_min_y),
                                  std::abs(cy - m_max_y)});
        if (std::isfinite(max_distance))
            reach = std::min<int64_t>(reach, int64_t(std::ceil(max_distance / m_cell)) + 1);
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
        if (best != npos && std::sqrt(best_d2) > max_distance)
            best = npos;
        if (distance && best != npos)
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
    // Distance to the ring, or infinity when it is farther than limit.
    double distance(const Vec2d& p, double limit = std::numeric_limits<double>::infinity()) const
    {
        double d = 0;
        grid->nearest(p, &d, limit);
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
                        count += ring_b.distance(p, reach) <= reach;
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
                d[i] = child_ring.distance(arc[i], seam_reach * m_w);
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
        // U-turns and bridges sit half a level outside their ring; level -0.5 belongs to level 0.
        lv[i] = std::max(0, int(std::lround(levels[i])));
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
            const size_t nn = grid.nearest(pts[i], &d, inner_reach * w);
            if (nn != npos) {
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
// Builds the variable-width path and audits it against the Fermat budget. The first exempt_head and
// last exempt_tail input points are start and end tapers, which may change width faster than the
// limit: they ramp from nothing to a full bead where the path begins and ends.
MorphingSpiralResult finish_path(const ExPolygon& domain, const std::vector<Vec2d>& pts, const std::vector<double>& widths,
                                 double spacing, size_t exempt_head = 0, size_t exempt_tail = 0)
{
    MorphingSpiralResult result;
    auto reject = [&](std::string reason) {
        result.paths.clear();
        result.fallback_reason = std::move(reason);
        return result;
    };
    // Merge micro-segments, then limit the width change rate as the radial spiral does.
    std::vector<Vec2d> kept;
    std::vector<double> kept_widths;
    std::vector<char> kept_exempt;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (!pts[i].allFinite() || !std::isfinite(widths[i]))
            return reject("non-finite path");
        if (!kept.empty() && i + 1 < pts.size() && (pts[i] - kept.back()).norm() < min_segment_mm)
            continue;
        kept.push_back(pts[i]);
        kept_widths.push_back(widths[i]);
        kept_exempt.push_back(i < exempt_head || i + exempt_tail >= pts.size());
    }
    if (kept.size() < 2)
        return reject("empty spiral");
    std::vector<double> segment(kept.size() - 1);
    for (size_t i = 0; i + 1 < kept.size(); ++i)
        segment[i] = (kept[i + 1] - kept[i]).norm();
    {
        // Only the stretch between the tapers is rate-limited.
        size_t lo = 0, hi = kept.size();
        while (lo < hi && kept_exempt[lo])
            ++lo;
        while (hi > lo && kept_exempt[hi - 1])
            --hi;
        if (hi - lo >= 2) {
            std::vector<double> body(kept_widths.begin() + lo, kept_widths.begin() + hi);
            const std::vector<double> body_segment(segment.begin() + lo, segment.begin() + (hi - 1));
            morphing_spiral_detail::limit_width_change(body, body_segment, .98 * morphing_spiral_detail::max_width_change);
            std::copy(body.begin(), body.end(), kept_widths.begin() + lo);
        }
    }
    ThickPolyline spiral;
    for (size_t i = 0; i < kept.size(); ++i) {
        if (i > 0 && !kept_exempt[i] && !kept_exempt[i - 1]) {
            const double change = std::abs(kept_widths[i] - kept_widths[i - 1]) / std::max(1e-9, segment[i - 1]);
            result.max_width_change_rate = std::max(result.max_width_change_rate, change);
        }
        morphing_spiral_detail::append_point(spiral, kept[i], kept_widths[i]);
    }
    if (result.max_width_change_rate > morphing_spiral_detail::max_width_change)
        return reject("width change per millimetre exceeds limit");
    ThickPolylines candidate;
    candidate.emplace_back(std::move(spiral));
    const std::optional<morphing_spiral_detail::Coverage> coverage = morphing_spiral_detail::measure_coverage(domain, candidate, result, true);
    if (!coverage)
        return reject("invalid domain area");
    // Ring points are chords, so the outer bead's edge sits microns inside a curved outline and the
    // exact footprint leaves a hairline along the whole edge, one large but empty "gap". Slivers
    // under 0.05 mm wide are below what an extrusion resolves, so gaps are opened by that much
    // before they are measured.
    result.raw_uncovered_fraction = result.uncovered_fraction;
    result.raw_largest_gap_mm2    = result.largest_gap_mm2;
    const ExPolygons gaps = opening_ex(coverage->gaps, float(scale_(sliver_mm / 2)));
    double uncovered = 0;
    result.largest_gap_mm2 = 0;
    for (const ExPolygon& gap : gaps) {
        const double a = unscale<double>(unscale<double>(gap.area()));
        uncovered += a;
        result.largest_gap_mm2 = std::max(result.largest_gap_mm2, a);
    }
    result.uncovered_fraction = uncovered / unscale<double>(unscale<double>(domain.area()));
    // Looser than the radial spiral's budget: every ring end and splice leaves a few hundredths of a
    // square millimetre, so the island total is larger even where each gap stays small.
    if (result.largest_gap_mm2 > .45 || result.largest_spill_mm2 > .10 ||
        !offset_ex(gaps, -float(scale_(spacing * .25))).empty() ||
        !offset_ex(coverage->spills, -float(scale_(spacing * .15))).empty())
        return reject("local gap or spill exceeds area/inscribed-width limit");
    if (result.uncovered_fraction > .012 || result.outside_fraction > .002 || std::abs(result.excess_fraction) > .03)
        return reject("bead footprint coverage or deposition budget failed");
    result.paths = std::move(candidate);
    return result;
}

bool valid_bounds(double spacing, double min_spacing, double max_spacing)
{
    return std::isfinite(spacing) && std::isfinite(min_spacing) && std::isfinite(max_spacing) && min_spacing > 0 &&
           spacing >= min_spacing && spacing <= max_spacing;
}

MorphingSpiralResult rejected(std::string reason)
{
    MorphingSpiralResult result;
    result.fallback_reason = std::move(reason);
    return result;
}

std::vector<double> point_widths(const ThickPolyline& path)
{
    std::vector<double> widths;
    for (size_t i = 0; i < path.points.size(); ++i)
        widths.push_back(unscale<double>(i + 1 < path.points.size() ? path.width[2 * i] : path.width.back()));
    return widths;
}
} // namespace

MorphingSpiralResult fermat_spiral(const ExPolygon& domain, double spacing, double min_spacing, double max_spacing)
{
    if (!valid_bounds(spacing, min_spacing, max_spacing))
        return rejected("invalid spacing bounds");
    std::vector<Vec2d> pts;
    std::vector<double> levels;
    try {
        Levels rings = offset_rings(domain, spacing);
        if (rings.empty())
            return rejected("island too small for one bead");
        std::vector<Strand> strands = build_strands(rings);
        auto [root, parent] = build_tree(strands, spacing);
        Builder builder(strands, parent, spacing);
        Path path = builder.spiral_path(root, straightest(*strands[root].front()));
        pts    = std::move(path.pts);
        levels = std::move(path.levels);
    } catch (const FermatError& e) {
        return rejected(e.reason);
    }
    std::vector<double> widths;
    medial_widths(domain, pts, levels, spacing, min_spacing, max_spacing, widths);
    return finish_path(domain, pts, widths, spacing);
}

namespace {
// First crossing of the ray o + t d (t > 0) with the closed loop, or infinity.
double ray_hit(const std::vector<Vec2d>& loop, const Vec2d& o, const Vec2d& d)
{
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec2d a = loop[i], e = loop[(i + 1) % loop.size()] - a;
        const double denom = cross(d, e);
        if (std::abs(denom) < 1e-12)
            continue;
        const Vec2d ao = a - o;
        const double t = cross(ao, e) / denom, s = cross(ao, d) / denom;
        if (t > 1e-6 && s >= 0 && s <= 1)
            best = std::min(best, t);
    }
    return best;
}

// Positions along u where the line {q : q . v = c} crosses the closed loop.
std::vector<double> line_hits(const std::vector<Vec2d>& loop, const Vec2d& u, const Vec2d& v, double c)
{
    std::vector<double> out;
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec2d a = loop[i], b = loop[(i + 1) % loop.size()];
        const double va = a.dot(v) - c, vb = b.dot(v) - c;
        if (va == 0 || va * vb < 0) {
            const double t = va == vb ? 0. : va / (va - vb);
            out.push_back((a + (b - a) * t).dot(u));
        }
    }
    return out;
}

// Outward normals of a counter-clockwise loop (right of the direction of travel).
std::vector<Vec2d> outward_normals(const std::vector<Vec2d>& loop)
{
    std::vector<Vec2d> out(loop.size());
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec2d t = loop[(i + 1) % loop.size()] - loop[(i + loop.size() - 1) % loop.size()];
        out[i] = Vec2d(t.y(), -t.x()) / std::max(t.norm(), 1e-9);
    }
    return out;
}

// The single loop of an offset, counter-clockwise, or nothing when the offset split or vanished.
std::optional<Polygon> single_loop(Polygons loops)
{
    loops.erase(std::remove_if(loops.begin(), loops.end(), [](const Polygon& p) { return std::abs(p.area()) < scale_(scale_(1e-3)); }),
                loops.end());
    if (loops.size() != 1)
        return std::nullopt;
    loops.front().make_counter_clockwise();
    return loops.front();
}

constexpr double accordion_turn_reach = .7; // lane widths a fold's U-turn bead reaches past its lane end
} // namespace

MorphingSpiralResult accordion_spiral(const ExPolygon& domain, double spacing_req, double min_spacing, double max_spacing)
{
    if (!valid_bounds(spacing_req, min_spacing, max_spacing))
        return rejected("invalid spacing bounds");
    if (domain.holes.size() != 1)
        return rejected("accordion: not one hole");
    const double w = spacing_req;
    Polygon outer = domain.contour, hole = domain.holes.front();
    outer.make_counter_clockwise();
    hole.make_counter_clockwise();
    const std::vector<Vec2d> outer_mm = to_mm(outer);

    // Narrowest thickness along the hole's outward normals sets the ring spacing, so the rings fill
    // it exactly.
    const std::vector<Vec2d> hs = resample(to_mm(hole), step_mm);
    const std::vector<Vec2d> hn = outward_normals(hs);
    double t_min = std::numeric_limits<double>::infinity();
    for (size_t j = 0; j < hs.size(); ++j)
        t_min = std::min(t_min, ray_hit(outer_mm, hs[j], hn[j]));
    if (!std::isfinite(t_min))
        return rejected("accordion: a ray leaves the island");
    const int slots = std::max(3, int(std::lround(t_min / w)));
    const double spacing = t_min / slots;
    if (spacing < min_spacing || spacing > max_spacing)
        return rejected("accordion: ring spacing outside the width bounds");
    const int inner_n = slots / 2, outer_n = slots - 1 - inner_n;
    const double miter = 4.;

    // Outline rings, hole rings and the two edges of the middle strip, as true offsets.
    std::vector<std::unique_ptr<Ring>> loops;
    for (int k = 0; k < outer_n; ++k) {
        const auto ring = single_loop(offset(outer, -float(scale_((k + .5) * spacing)), jtMiter, miter));
        if (!ring)
            return rejected("accordion: an outline ring split");
        auto r = std::make_unique<Ring>();
        r->level = k;
        r->pts   = resample(to_mm(*ring), step_mm);
        r->finish();
        loops.emplace_back(std::move(r));
    }
    const auto inner_edge = single_loop(offset(hole, float(scale_(inner_n * spacing)), jtMiter, miter));
    const auto outer_edge = single_loop(offset(outer, -float(scale_(outer_n * spacing)), jtMiter, miter));
    if (!inner_edge || !outer_edge)
        return rejected("accordion: the middle strip split");
    const std::vector<Vec2d> outer_edge_mm = to_mm(*outer_edge);
    const std::vector<Vec2d> inner_edge_mm = to_mm(*inner_edge);

    // Middle loop: along the inner edge's outward normals, the centre of the strip out to the outer
    // edge, as wide as the strip where one bead can fill it.
    const std::vector<Vec2d> ie = resample(to_mm(*inner_edge), step_mm);
    const std::vector<Vec2d> nn = outward_normals(ie);
    std::vector<double> strip(ie.size());
    std::vector<char> wide(ie.size());
    std::vector<Vec2d> middle(ie.size());
    for (size_t j = 0; j < ie.size(); ++j) {
        strip[j] = ray_hit(outer_edge_mm, ie[j], nn[j]);
        if (!std::isfinite(strip[j]))
            return rejected("accordion: middle strip lost");
        wide[j]   = strip[j] > .97 * max_spacing;
        middle[j] = ie[j] + nn[j] * (strip[j] / 2);
    }
    {
        auto r = std::make_unique<Ring>();
        r->level = outer_n;
        r->pts   = middle;
        r->finish();
        loops.emplace_back(std::move(r));
    }
    for (int k = inner_n - 1; k >= 0; --k) {
        const auto ring = single_loop(offset(hole, float(scale_((k + .5) * spacing)), jtMiter, miter));
        if (!ring)
            return rejected("accordion: a hole ring split");
        auto r = std::make_unique<Ring>();
        r->level = outer_n + inner_n - k;
        r->pts   = resample(to_mm(*ring), step_mm);
        r->finish();
        loops.emplace_back(std::move(r));
    }

    // Lane-change zone where the middle strip is narrow and the rings are straightest. The first
    // ring's lane change has no lap arriving beside it and the last ring's none leaving, so each
    // leaves a wedge up to a bead wide that a taper fills. A zone long enough for those tapers to stay
    // within the width change limit (smoothstep peaks at 1.5x its mean slope) is preferred; where no
    // narrow stretch is that long, the short zone is used and the audit exempts the tapers. Every
    // ring uses the same length: a lane change between rings stepping at different rates hooks back
    // on itself and stops covering either ring.
    const size_t n = ie.size();
    double zone = 0;
    size_t k0 = npos;
    for (const double candidate : {std::max(zone_spacings * spacing, 1.6 * spacing / morphing_spiral_detail::max_width_change),
                                   zone_spacings * spacing}) {
        const size_t win = std::max<size_t>(3, size_t(candidate / step_mm));
        double best = std::numeric_limits<double>::infinity();
        for (size_t k = 0; k < n; ++k) {
            double score = 0;
            bool narrow = true;
            for (size_t m = 0; m < win && narrow; ++m) {
                const size_t j = (k + m) % n;
                narrow = !wide[j];
                score += std::abs(cross(nn[(j + n - 1) % n], nn[j]));
            }
            if (narrow && score < best) {
                best = score;
                k0   = k;
            }
        }
        if (k0 != npos) {
            zone = candidate;
            break;
        }
    }
    if (k0 == npos)
        return rejected("accordion: no narrow stretch for the lane-change zone");

    // One inward spiral through every loop, each cut on one anchor in the narrow stretch: chaining
    // nearest points ring to ring can drift into a wide stretch, where the middle loop runs far from
    // the rings either side of it.
    const Vec2d anchor = middle[k0];
    const size_t nz = std::max<size_t>(8, size_t(std::ceil(zone / step_mm)));
    std::vector<Vec2d> pts;
    std::vector<double> levels;
    std::vector<double> zone_len, cut;
    for (size_t i = 0; i < loops.size(); ++i) {
        zone_len.push_back(std::min(zone, loops[i]->length() / 4));
        cut.push_back(int(i) == outer_n ? loops[i]->cum[k0] : loops[i]->nearest_s(anchor));
    }
    {
        for (size_t i = 0; i < loops.size(); ++i) {
            const Ring& r = *loops[i];
            const std::vector<Vec2d> arc = r.arc(cut[i] + zone_len[i], r.length() - zone_len[i], +1);
            append(pts, arc);
            levels.insert(levels.end(), arc.size(), double(r.level));
            if (i + 1 < loops.size()) {
                const Ring& next = *loops[i + 1];
                for (size_t k = 1; k + 1 < nz; ++k) {
                    const double u = double(k) / double(nz - 1), s = smoothstep(u);
                    pts.push_back(r.at(cut[i] + u * zone_len[i]) * (1 - s) + next.at(cut[i + 1] + u * zone_len[i + 1]) * s);
                    levels.push_back(r.level * (1 - s) + next.level * s);
                }
            }
        }
    }
    const PointGrid middle_grid(middle, .5);
    const PointGrid ie_grid(ie, .5);
    std::vector<double> widths(pts.size(), spacing);
    std::vector<size_t> near_mid(pts.size(), npos);
    for (size_t i = 0; i < pts.size(); ++i)
        if (std::abs(levels[i] - outer_n) < 1e-9) {
            near_mid[i] = middle_grid.nearest(pts[i]);
            widths[i]   = std::clamp(strip[near_mid[i]], min_spacing, max_spacing);
        }

    // Accordions: over each wide run of the middle loop, parallel folds across the strip, an even
    // number so the loop comes out where it went in. A fold covers the part of its line inside the
    // strip whose nearest inner-edge sample belongs to this run's wide stretch.
    std::vector<Vec2d> out_pts;
    std::vector<double> out_w;
    size_t cur = 0;
    for (size_t i = 0; i < pts.size();) {
        if (near_mid[i] == npos || !wide[near_mid[i]]) {
            ++i;
            continue;
        }
        size_t e = i;
        while (e + 1 < pts.size() && near_mid[e + 1] != npos && wide[near_mid[e + 1]])
            ++e;
        const size_t a = i, b = e;
        i = e + 1;
        if (b - a < 2)
            continue;
        // This run's stretch: the wide inner-edge samples contiguous with the ones it passes.
        Vec2d u = Vec2d::Zero();
        std::vector<char> stretch(n, 0);
        auto grow = [&](size_t j0) {
            if (stretch[j0])
                return;
            for (size_t j = j0, s = 0; s < n && wide[j] && !stretch[j]; j = (j + 1) % n, ++s)
                stretch[j] = 1;
            for (size_t j = (j0 + n - 1) % n, s = 0; s < n && wide[j] && !stretch[j]; j = (j + n - 1) % n, ++s)
                stretch[j] = 1;
        };
        for (size_t p = a; p <= b; ++p) {
            u += nn[near_mid[p]];
            grow(near_mid[p]);
        }
        u /= std::max(u.norm(), 1e-9);
        const Vec2d v(-u.y(), u.x());
        double c_lo = std::numeric_limits<double>::infinity(), c_hi = -c_lo;
        for (size_t j = 0; j < n; ++j)
            if (stretch[j])
                for (int f = 0; f <= 8; ++f) {
                    const double c = (ie[j] + nn[j] * (strip[j] * f / 8)).dot(v);
                    c_lo = std::min(c_lo, c);
                    c_hi = std::max(c_hi, c);
                }
        const bool reverse = pts[b].dot(v) < pts[a].dot(v);
        // A sharp outer corner (the outer edge peaking along u, as at a square's corner) must sit in
        // the middle of an outer U-turn: lanes 2i and 2i + 1 in travel order turn together at the
        // outer end, and a corner between two such pairs is a V pocket neither reaches. The lane
        // count (always even) is chosen so the corner's position in lanes is close to an odd number.
        double reach_max = -std::numeric_limits<double>::infinity(), apex = 0;
        for (size_t j = 0; j < n; ++j)
            if (stretch[j]) {
                const Vec2d q = ie[j] + nn[j] * strip[j];
                if (q.dot(u) > reach_max) {
                    reach_max = q.dot(u);
                    apex      = q.dot(v);
                }
            }
        double near_lo = std::numeric_limits<double>::infinity(), near_hi = -near_lo;
        for (size_t j = 0; j < n; ++j)
            if (stretch[j]) {
                const Vec2d q = ie[j] + nn[j] * strip[j];
                if (q.dot(u) > reach_max - .5 * w) {
                    near_lo = std::min(near_lo, q.dot(v));
                    near_hi = std::max(near_hi, q.dot(v));
                }
            }
        const bool sharp = near_hi - near_lo < 2 * w;
        const int m0 = std::max(2, int(std::lround((c_hi - c_lo) / w / 2)) * 2);
        int m = 0;
        bool best_off = true;
        double best_dw = std::numeric_limits<double>::infinity();
        for (int cand = std::max(2, m0 - 4); cand <= m0 + 4; cand += 2) {
            const double lw = (c_hi - c_lo) / cand;
            if (lw < min_spacing || lw > max_spacing)
                continue;
            bool off = false;
            if (sharp) {
                const double j = (reverse ? c_hi - apex : apex - c_lo) / lw;
                off = std::abs(j - (2 * std::floor(j / 2) + 1)) > .4;
            }
            const double dw = std::abs(lw - w);
            if (m == 0 || (!off && best_off) || (off == best_off && dw < best_dw)) {
                m        = cand;
                best_off = off;
                best_dw  = dw;
            }
        }
        if (m == 0)
            return rejected("accordion: fold spacing outside the width bounds");
        const double lane_w = (c_hi - c_lo) / m;
        std::vector<double> cs;
        for (int k = 0; k < m; ++k)
            cs.push_back(c_lo + (k + .5) * lane_w);
        if (reverse)
            std::reverse(cs.begin(), cs.end());
        std::vector<std::vector<Vec2d>> lanes;
        for (double c : cs) {
            std::vector<double> hits = line_hits(outer_edge_mm, u, v, c);
            if (hits.size() < 2)
                continue;
            std::sort(hits.begin(), hits.end());
            std::vector<double> inner_hits = line_hits(inner_edge_mm, u, v, c);
            std::sort(inner_hits.begin(), inner_hits.end());
            const double t0 = hits.front(), t1 = hits.back();
            // Longest run of samples inside the strip and owned by this stretch. A sample is inside a
            // loop when an odd number of the loop's crossings lie before it on the line; counting
            // crossings as t advances replaces a point-in-polygon test per sample, which dominated
            // the run time on long loops.
            double run_s = 0, run_e = -1, cur_s = 0;
            bool in = false;
            size_t outer_passed = 0, inner_passed = 0;
            for (double t = t0; t <= t1; t += step_mm / 2) {
                while (outer_passed < hits.size() && hits[outer_passed] <= t)
                    ++outer_passed;
                while (inner_passed < inner_hits.size() && inner_hits[inner_passed] <= t)
                    ++inner_passed;
                const Vec2d q = u * t + v * c;
                const bool keep = outer_passed % 2 == 1 && inner_passed % 2 == 0 && stretch[ie_grid.nearest(q)];
                if (keep && !in)
                    cur_s = t;
                if (keep && t - cur_s > run_e - run_s) {
                    run_s = cur_s;
                    run_e = t;
                }
                in = keep;
            }
            if (run_e <= run_s)
                continue;
            // A fold's U-turn bulges past its lane end and its bead reaches further still.
            double s0 = run_s + accordion_turn_reach * lane_w, s1 = run_e - accordion_turn_reach * lane_w;
            if (s1 <= s0) {
                s0 = (run_s + run_e) / 2;
                s1 = s0 + .05;
            }
            std::vector<Vec2d> lane;
            const size_t cnt = std::max<size_t>(2, size_t(std::ceil((s1 - s0) / step_mm)) + 1);
            for (size_t k = 0; k < cnt; ++k)
                lane.push_back(u * (s0 + (s1 - s0) * double(k) / double(cnt - 1)) + v * c);
            lanes.emplace_back(std::move(lane));
        }
        if (lanes.size() % 2)
            lanes.pop_back();
        if (lanes.size() < 2)
            continue;
        for (size_t p = cur; p < a; ++p) {
            out_pts.push_back(pts[p]);
            out_w.push_back(widths[p]);
        }
        Vec2d prev = Vec2d::Zero();
        for (size_t k = 0; k < lanes.size(); ++k) {
            std::vector<Vec2d>& lane = lanes[k];
            if (k % 2)
                std::reverse(lane.begin(), lane.end());
            if (k > 0)
                for (const Vec2d& q : uturn(prev, lane.front(), k % 2 ? u : Vec2d(-u))) {
                    out_pts.push_back(q);
                    out_w.push_back(lane_w);
                }
            for (const Vec2d& q : lane) {
                out_pts.push_back(q);
                out_w.push_back(lane_w);
            }
            prev = lane.back();
        }
        cur = b + 1;
    }
    for (size_t p = cur; p < pts.size(); ++p) {
        out_pts.push_back(pts[p]);
        out_w.push_back(widths[p]);
    }

    // Tapers over the first and last lane-change stretches. At step u of the first, the lap leaving
    // the outermost ring has moved s(u) of a lane inwards, so the bead fills the outer s(u) of the
    // lane and ends where the first arc begins; the last mirrors it on the innermost ring after the
    // final arc.
    const size_t nt = std::max<size_t>(8, size_t(std::ceil(zone / step_mm)) + 1);
    std::vector<Vec2d> taper_pts;
    std::vector<double> taper_w;
    {
        const Ring &first = *loops[0], &second = *loops[1];
        for (size_t k = 0; k + 1 < nt; ++k) {
            const double u = double(k) / double(nt - 1), s = smoothstep(u);
            const Vec2d pa = first.at(cut[0] + u * zone_len[0]), pb = second.at(cut[1] + u * zone_len[1]);
            taper_pts.push_back(pa + (pb - pa) * ((s - 1) / 2));
            taper_w.push_back(std::max(s, .02) * spacing);
        }
    }
    const size_t head = taper_pts.size();
    out_pts.insert(out_pts.begin(), taper_pts.begin(), taper_pts.end());
    out_w.insert(out_w.begin(), taper_w.begin(), taper_w.end());
    {
        const size_t nl = loops.size();
        const Ring &last = *loops[nl - 1], &before = *loops[nl - 2];
        for (size_t k = 1; k < nt; ++k) {
            const double u = double(k) / double(nt - 1), s = smoothstep(u);
            const Vec2d pa = before.at(cut[nl - 2] + u * zone_len[nl - 2]), pb = last.at(cut[nl - 1] + u * zone_len[nl - 1]);
            out_pts.push_back(pb + (pb - pa) * (s / 2));
            out_w.push_back(std::max(1 - s, .02) * spacing);
        }
    }
    MorphingSpiralResult result = finish_path(domain, out_pts, out_w, spacing, head, nt - 1);
    result.beads = slots;
    return result;
}} // namespace Slic3r