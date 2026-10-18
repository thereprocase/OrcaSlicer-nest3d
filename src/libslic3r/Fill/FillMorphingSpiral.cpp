#include "FillMorphingSpiral.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"
#include <boost/log/trivial.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// One continuous, variable-width "stepped" spiral for an island with one hole, or none.
//
// The island is described in polar form around a centre: E_o(theta) is the outer bead-domain edge
// and E_i(theta) the hole edge (for a disk, a pinhole at the centre). B bead loops sit at fractions
// f = (level + 1/2) / B between the edges, so each bead's normal width is
//     (E_o - E_i) / B * r / hypot(r, dr/dtheta).
// B is chosen inside the window where every width fits the nozzle bounds, nearest the requested width.
//
// The path is a start taper, loop 0, a lane change to loop 1, ..., loop B-1 and an end taper. All
// lane changes share one angular seam zone, so neighbouring beads stay parallel and the region is
// covered exactly. The start taper fills from the outer edge (level (S-1)/2, width S) and the end
// taper fills to the hole (level B-1+S/2, width 1-S); no bead ever widens beyond its level width,
// which is why the requested line width is honoured instead of being rejected at the endpoints.

namespace Slic3r {
namespace {
constexpr int samples  = 720;
constexpr double tau   = 6.2831853071795864769;
constexpr double phase = .000123;
// Below this width a taper tip is not printed; the sliver it leaves is checked by the coverage audit.
constexpr double tip_fraction = .10;
// Seam zone length on the innermost loop, in mm. A smoothstep ramp peaks at 1.5x its mean slope, so
// this keeps the taper width change under the per-millimetre limit.
constexpr double seam_zone_mm = 4.5;
// Points closer together than this are merged; the centre of a disk otherwise gets micron segments.
constexpr double min_segment_mm = .05;
constexpr double max_width_change = .20; // mm of width per mm of path

double cross(const Vec2d& a, const Vec2d& b) { return a.x() * b.y() - a.y() * b.x(); }
Vec2d mm(const Point& p) { return Vec2d(unscale<double>(p.x()), unscale<double>(p.y())); }
Point point(const Vec2d& p) { return Point::new_scale(p.x(), p.y()); }

struct Radii
{
    std::vector<double> r;
    std::vector<double> slope; // dr/dtheta
};

// Radius and exact dr/dtheta along each sample ray, or nothing unless every ray hits the polygon
// exactly once. The slope comes from the direction of the edge the ray hits, so a sharp corner does
// not produce the averaged, wrong slope a finite difference across the corner would.
std::optional<Radii> radii(const Polygon& polygon, const Vec2d& center)
{
    if (polygon.points.size() < 3 || polygon.points.size() > 10000)
        return std::nullopt;
    Radii out;
    out.r.resize(samples);
    out.slope.resize(samples);
    for (int k = 0; k < samples; ++k) {
        const double angle = phase + tau * k / samples;
        const Vec2d ray(std::cos(angle), std::sin(angle));
        const Vec2d tangent(-ray.y(), ray.x());
        int hits = 0;
        for (size_t i = 0; i < polygon.points.size(); ++i) {
            const Vec2d a    = mm(polygon.points[i]) - center;
            const Vec2d edge = mm(polygon.points[(i + 1) % polygon.points.size()]) - center - a;
            const double den = cross(ray, edge);
            if (std::abs(den) < 1e-12)
                continue;
            const double distance = cross(a, edge) / den;
            const double t        = cross(a, ray) / den;
            if (distance > 0 && t >= -1e-10 && t < 1. - 1e-10) {
                const double across = edge.dot(tangent);
                out.r[k]            = distance;
                // Moving along the edge: dr = along * ds and r dtheta = across * ds.
                out.slope[k] = std::abs(across) > 1e-12 ? distance * edge.dot(ray) / across : 0.;
                ++hits;
            }
        }
        if (hits != 1)
            return std::nullopt;
    }
    return out;
}

struct Hit
{
    double r;
    double slope_before; // dr/dtheta of the edge on one side of a vertex
    double slope_after;  // and on the other; equal away from vertices
};

// Exact ray hit at an arbitrary angle. At a polygon vertex the ray touches two edges, and both
// slopes are returned so the bead width there can use both sides.
std::optional<Hit> ray_hit(const Polygon& polygon, const Vec2d& center, double angle)
{
    const Vec2d ray(std::cos(angle), std::sin(angle));
    const Vec2d tangent(-ray.y(), ray.x());
    double r_sum = 0;
    int hits     = 0;
    Hit hit{0., 0., 0.};
    for (size_t i = 0; i < polygon.points.size(); ++i) {
        const Vec2d a    = mm(polygon.points[i]) - center;
        const Vec2d edge = mm(polygon.points[(i + 1) % polygon.points.size()]) - center - a;
        const double den = cross(ray, edge);
        if (std::abs(den) < 1e-12)
            continue;
        const double distance = cross(a, edge) / den;
        const double t        = cross(a, ray) / den;
        if (distance > 0 && t >= -1e-9 && t <= 1. + 1e-9) {
            const double across = edge.dot(tangent);
            const double slope  = std::abs(across) > 1e-12 ? distance * edge.dot(ray) / across : 0.;
            if (hits == 0)
                hit.slope_before = slope;
            hit.slope_after = slope;
            r_sum += distance;
            ++hits;
        }
    }
    if (hits == 0)
        return std::nullopt;
    hit.r = r_sum / hits;
    return hit;
}

// Disk centre: among the centroid and an interior grid, the candidate with the largest minimum edge
// radius that still sees the whole boundary from inside.
std::optional<Vec2d> disk_center(const Polygon& contour)
{
    const BoundingBox box = get_extents(contour);
    std::vector<Vec2d> candidates{mm(contour.centroid())};
    for (int i = 1; i < 8; ++i)
        for (int j = 1; j < 8; ++j)
            candidates.emplace_back(unscale<double>(box.min.x() + (box.max.x() - box.min.x()) * i / 8.),
                                    unscale<double>(box.min.y() + (box.max.y() - box.min.y()) * j / 8.));
    std::optional<Vec2d> best;
    double best_score = -1.;
    for (const Vec2d& c : candidates) {
        if (!contour.contains(point(c)))
            continue;
        const auto hit = radii(contour, c);
        if (!hit)
            continue;
        const double score = *std::min_element(hit->r.begin(), hit->r.end());
        if (score > best_score) {
            best_score = score;
            best       = c;
        }
    }
    return best;
}

double smoothstep(double s)
{
    s = std::clamp(s, 0., 1.);
    return s * s * (3. - 2. * s);
}

// Limit |dw/ds| to `limit`: the mean of the upper and lower rate-limited envelopes. Each envelope
// is the tightest curve within the limit that stays above (or below) the commanded widths, so the
// mean obeys the limit and equals the commanded width wherever the limit was already met.
void limit_width_change(std::vector<double>& widths, const std::vector<double>& segment, double limit)
{
    const size_t n = widths.size();
    if (n < 2)
        return;
    std::vector<double> up = widths, low = widths;
    for (size_t i = 1; i < n; ++i) {
        up[i]  = std::max(up[i], up[i - 1] - limit * segment[i - 1]);
        low[i] = std::min(low[i], low[i - 1] + limit * segment[i - 1]);
    }
    for (size_t i = n - 1; i-- > 0;) {
        up[i]  = std::max(up[i], up[i + 1] - limit * segment[i]);
        low[i] = std::min(low[i], low[i + 1] + limit * segment[i]);
    }
    for (size_t i = 0; i < n; ++i)
        widths[i] = .5 * (up[i] + low[i]);
}

void append_point(ThickPolyline& path, const Vec2d& p, double width)
{
    const double scaled_width = scale_(width);
    if (path.points.empty()) {
        path.points.emplace_back(point(p));
        path.width.emplace_back(scaled_width);
        return;
    }
    const double previous = path.width.back();
    if (path.points.size() == 1)
        path.width.clear();
    path.width.emplace_back(previous);
    path.width.emplace_back(scaled_width);
    path.points.emplace_back(point(p));
}
// Polygonal swept-strip audit using the sampled path's vertex normals. Building
// one strip per path avoids an expensive union of a cap at every vertex.
// This is geometric coverage, not a molten-bead or local-height simulation.
Polygons footprint(const ThickPolylines& paths, double& strip_area)
{
    Polygons strips;
    strip_area = 0;
    for (const ThickPolyline& path : paths) {
        const size_t n = path.points.size();
        Polygon strip;
        Points right;
        for (size_t i = 0; i < n; ++i) {
            const Vec2d p       = mm(path.points[i]);
            Vec2d tangent       = mm(path.points[std::min(i + 1, n - 1)]) - mm(path.points[i == 0 ? 0 : i - 1]);
            const double length = tangent.norm();
            if (length < 1e-9)
                continue;
            const Vec2d normal(-tangent.y() / length, tangent.x() / length);
            const double width = unscale<double>(i == 0 ? path.width.front() : path.width[2 * i - 1]);
            strip.points.emplace_back(point(p + normal * width / 2));
            right.emplace_back(point(p - normal * width / 2));
            if (i + 1 < n)
                strip_area += (mm(path.points[i + 1]) - p).norm() * unscale<double>(path.width[2 * i] + path.width[2 * i + 1]) / 2;
        }
        std::reverse(right.begin(), right.end());
        append(strip.points, std::move(right));
        strip.make_counter_clockwise();
        strips.emplace_back(std::move(strip));
        for (size_t i : {size_t(0), n - 1}) {
            const double width = unscale<double>(i == 0 ? path.width.front() : path.width.back());
            Polygon cap;
            for (int j = 0; j < 16; ++j)
                cap.points.emplace_back(point(mm(path.points[i]) + Vec2d(std::cos(tau * j / 16), std::sin(tau * j / 16)) * width / 2));
            strips.emplace_back(std::move(cap));
        }
    }
    return union_(strips);
}
double area_mm2(const ExPolygons& polygons)
{
    double area = 0;
    for (const ExPolygon& p : polygons)
        area += unscale<double>(unscale<double>(p.area()));
    return area;
}
} // namespace

MorphingSpiralResult morphing_spiral(const ExPolygon& domain, double spacing, double min_spacing, double max_spacing)
{
    MorphingSpiralResult result;
    auto reject = [&](std::string reason) {
        result.paths.clear();
        result.fallback_reason = std::move(reason);
        return result;
    };
    if (!std::isfinite(spacing) || !std::isfinite(min_spacing) || !std::isfinite(max_spacing) || min_spacing <= 0 ||
        spacing < min_spacing || spacing > max_spacing)
        return reject("invalid spacing limits");
    if (domain.holes.size() > 1)
        return reject("more than one hole");

    // Edges in polar form around the centre.
    const bool disk = domain.holes.empty();
    Vec2d center;
    std::vector<double> e_o, d_o, e_i, d_i;
    if (disk) {
        const auto c = disk_center(domain.contour);
        if (!c)
            return reject("no centre sees the whole boundary");
        center         = *c;
        const auto out = radii(domain.contour, center);
        if (!out)
            return reject("boundary is not single-valued around the centre");
        e_o = out->r;
        d_o = out->slope;
        // A pinhole at the centre: the last loop closes onto it.
        e_i.assign(samples, tip_fraction * spacing);
        d_i.assign(samples, 0.);
    } else {
        if (domain.holes.front().points.size() < 3)
            return reject("degenerate hole");
        center         = mm(domain.holes.front().centroid());
        const auto out = radii(domain.contour, center);
        const auto in  = radii(domain.holes.front(), center);
        if (!out || !in)
            return reject("boundaries are not single-valued around the hole centre");
        e_o = out->r;
        d_o = out->slope;
        e_i = in->r;
        d_i = in->slope;
    }
    for (int k = 0; k < samples; ++k)
        if (e_o[k] - e_i[k] < 2 * min_spacing)
            return reject("region too narrow for two beads");

    // Normal width of one bead per unit bead count at fraction f along every ray.
    auto unit_width = [&](int k, double f) {
        const double r     = e_o[k] * (1 - f) + e_i[k] * f;
        const double slope = d_o[k] * (1 - f) + d_i[k] * f;
        return (e_o[k] - e_i[k]) * r / std::hypot(r, slope);
    };
    double w_min = std::numeric_limits<double>::max(), w_max = 0., w_sum = 0.;
    int w_count = 0;
    for (int j = 1; j < 128; ++j)
        for (int k = 0; k < samples; ++k) {
            const double w = unit_width(k, j / 128.);
            w_min          = std::min(w_min, w);
            w_max          = std::max(w_max, w);
            w_sum += w;
            ++w_count;
        }
    // The bead count window where every width fits, and the count nearest the requested width.
    const int fewest = int(std::ceil(w_max / max_spacing));
    const int most   = int(std::floor(w_min / min_spacing));
    if (fewest > most)
        return reject("bead widths cannot fit the nozzle bounds on one radial spiral (needs between " + std::to_string(fewest) +
                      " and " + std::to_string(most) + " beads)");
    const int beads = std::clamp(std::max(2, int(std::round(w_sum / w_count / spacing))), fewest, most);
    if (beads < 2)
        return reject("fewer than two beads fit");
    if (beads > 400)
        return reject("bead budget exceeded");

    // One shared angular seam zone, at least seam_zone_mm long on the innermost loop and a whole
    // number of samples. A disk's innermost loops shrink to the centre, so its zone is sized on the
    // outer loop instead.
    const double step = tau / samples;
    double reference  = std::numeric_limits<double>::max();
    for (int k = 0; k < samples; ++k)
        reference = std::min(reference, disk ? e_o[k] : e_o[k] + (e_i[k] - e_o[k]) * (beads - .5) / beads);
    const int zone_samples = std::max(2, int(std::ceil(std::min(tau / 4, seam_zone_mm / std::max(reference, 1e-6)) / step)));
    const double zone      = zone_samples * step;
    // Seam at the smoothest window, so lane changes and tapers never straddle a corner.
    int seam_k = 0;
    {
        double best = std::numeric_limits<double>::max();
        for (int k = 0; k < samples; ++k) {
            double rough = 0;
            for (int m = 0; m < zone_samples; ++m) {
                const int a = (k + m) % samples, b = (k + m + 1) % samples;
                rough += std::abs(d_o[b] - d_o[a]) + std::abs(d_i[b] - d_i[a]);
            }
            if (rough < best) {
                best   = rough;
                seam_k = k;
            }
        }
    }
    const double seam = phase + seam_k * step;

    // Flat loops run on the sample angles plus every polygon vertex angle, evaluated exactly, so loops
    // turn real corners instead of chords that cut them.
    std::vector<double> offsets; // angle after the seam, in (zone, tau)
    auto add_offset = [&](double angle) {
        const double rel = std::fmod(std::fmod(angle - seam, tau) + tau, tau);
        if (rel > zone + 1e-9 && rel < tau - 1e-9)
            offsets.emplace_back(rel);
    };
    for (int k = 0; k < samples; ++k)
        add_offset(phase + k * step);
    auto add_vertices = [&](const Polygon& polygon) {
        for (const Point& p : polygon.points) {
            const Vec2d d = mm(p) - center;
            add_offset(std::atan2(d.y(), d.x()));
        }
    };
    add_vertices(domain.contour);
    if (!disk)
        add_vertices(domain.holes.front());
    std::sort(offsets.begin(), offsets.end());
    offsets.erase(std::unique(offsets.begin(), offsets.end(), [](double a, double b) { return b - a < 1e-9; }), offsets.end());
    std::vector<Hit> flat_outer, flat_inner;
    flat_outer.reserve(offsets.size());
    flat_inner.reserve(offsets.size());
    for (double rel : offsets) {
        const auto out = ray_hit(domain.contour, center, seam + rel);
        if (!out)
            return reject("boundary lost during loop evaluation");
        flat_outer.emplace_back(*out);
        if (disk)
            flat_inner.push_back({tip_fraction * spacing, 0., 0.});
        else {
            const auto in = ray_hit(domain.holes.front(), center, seam + rel);
            if (!in)
                return reject("hole lost during loop evaluation");
            flat_inner.emplace_back(*in);
        }
    }

    std::vector<Vec2d> points;
    std::vector<double> widths;
    auto width_of = [&](double eo, double ei, double so, double si, double f, double r) {
        return (eo - ei) / beads * r / std::hypot(r, so * (1 - f) + si * f);
    };
    // Seam zone points: interpolated on the sample grid; the window was chosen smooth.
    auto emit_zone = [&](double theta, double level, double factor) {
        const double position = (theta - phase) / tau * samples;
        const double whole    = std::floor(position);
        const double frac     = position - whole;
        const int k           = ((int(whole) % samples) + samples) % samples;
        const int k1          = (k + 1) % samples;
        const double eo       = e_o[k] * (1 - frac) + e_o[k1] * frac;
        const double ei       = e_i[k] * (1 - frac) + e_i[k1] * frac;
        const double so       = d_o[k] * (1 - frac) + d_o[k1] * frac;
        const double si       = d_i[k] * (1 - frac) + d_i[k1] * frac;
        const double f        = std::clamp((level + .5) / beads, 0., 1.);
        const double r        = eo * (1 - f) + ei * f;
        points.emplace_back(center + Vec2d(std::cos(theta), std::sin(theta)) * r);
        widths.emplace_back(width_of(eo, ei, so, si, f, r) * factor);
    };
    auto emit_loop = [&](int level) {
        const double f = (level + .5) / beads;
        for (size_t i = 0; i < offsets.size(); ++i) {
            const Hit& o   = flat_outer[i];
            const Hit& in  = flat_inner[i];
            const double r = o.r * (1 - f) + in.r * f;
            const double theta = seam + offsets[i];
            points.emplace_back(center + Vec2d(std::cos(theta), std::sin(theta)) * r);
            // At a vertex the two sides have different slopes; use the mean of both widths.
            widths.emplace_back(.5 * (width_of(o.r, in.r, o.slope_before, in.slope_before, f, r) +
                                      width_of(o.r, in.r, o.slope_after, in.slope_after, f, r)));
        }
    };
    for (double t = 0; t < zone; t += step / 2) // start taper on loop 0
        emit_zone(seam + t, (smoothstep(t / zone) - 1) / 2, smoothstep(t / zone));
    for (int j = 0; j < beads; ++j) {
        emit_loop(j);
        if (j + 1 < beads)
            for (double t = 0; t < zone; t += step / 2) // lane change j -> j+1
                emit_zone(seam + t, j + smoothstep(t / zone), 1.);
    }
    for (double t = 0; t < zone; t += step / 2) // end taper on the last loop
        emit_zone(seam + t, beads - 1 + smoothstep(t / zone) / 2, 1 - smoothstep(t / zone));

    // Drop the sub-printable taper tips, merge micro-segments, then limit the width change rate.
    size_t first = 0, last = widths.size();
    while (first < last && widths[first] < tip_fraction * spacing)
        ++first;
    while (last > first && widths[last - 1] < tip_fraction * spacing)
        --last;
    std::vector<Vec2d> kept;
    std::vector<double> kept_widths;
    for (size_t i = first; i < last; ++i)
        if (kept.empty() || i + 1 == last || (points[i] - kept.back()).norm() >= min_segment_mm) {
            kept.emplace_back(points[i]);
            kept_widths.emplace_back(widths[i]);
        }
    if (kept.size() < 2)
        return reject("empty spiral");
    std::vector<double> segment(kept.size() - 1);
    for (size_t i = 0; i + 1 < kept.size(); ++i)
        segment[i] = (kept[i + 1] - kept[i]).norm();
    // A small margin keeps rounding from tripping the gate below.
    limit_width_change(kept_widths, segment, .98 * max_width_change);

    ThickPolyline spiral;
    for (size_t i = 0; i < kept.size(); ++i) {
        if (!std::isfinite(kept_widths[i]) || kept_widths[i] <= 0 || kept_widths[i] > max_spacing + 1e-9)
            return reject("bead width outside nozzle bounds");
        if (i > 0) {
            const double change = std::abs(kept_widths[i] - kept_widths[i - 1]) / std::max(1e-9, segment[i - 1]);
            result.max_width_change_rate = std::max(result.max_width_change_rate, change);
            if (change > max_width_change)
                return reject("width change per millimetre exceeds limit");
        }
        append_point(spiral, kept[i], kept_widths[i]);
    }
    ThickPolylines candidate;
    candidate.emplace_back(std::move(spiral));

    double strip_area      = 0;
    const Polygons covered = footprint(candidate, strip_area);
    const double area      = unscale<double>(unscale<double>(domain.area()));
    if (area <= 0)
        return reject("invalid domain area");
    const ExPolygons gaps     = diff_ex(ExPolygons{domain}, covered);
    const ExPolygons spills   = diff_ex(covered, ExPolygons{domain});
    result.uncovered_fraction = area_mm2(gaps) / area;
    result.outside_fraction   = area_mm2(spills) / area;
    for (const auto& gap : gaps)
        result.largest_gap_mm2 = std::max(result.largest_gap_mm2, area_mm2(ExPolygons{gap}));
    for (const auto& spill : spills)
        result.largest_spill_mm2 = std::max(result.largest_spill_mm2, area_mm2(ExPolygons{spill}));
    if (result.largest_gap_mm2 > .25 || result.largest_spill_mm2 > .10 || !offset_ex(gaps, -float(scale_(spacing * .15))).empty() ||
        !offset_ex(spills, -float(scale_(spacing * .15))).empty())
        return reject("local gap or spill exceeds area/inscribed-width limit");
    result.excess_fraction = strip_area / area - 1;
    if (result.uncovered_fraction > .002 || result.outside_fraction > .002 || std::abs(result.excess_fraction) > .005)
        return reject("bead footprint coverage or deposition budget failed");
    result.paths = std::move(candidate);
    return result;
}

void FillMorphingSpiral::_fill_surface_single(
    const FillParams& params, unsigned int layers, const std::pair<float, Point>& direction, ExPolygon expolygon, ThickPolylines& out)
{
    if (params.density <= .9999f || params.dont_adjust || params.flow.bridge()) {
        FillSpiralInset::_fill_surface_single(params, layers, direction, std::move(expolygon), out);
        return;
    }
    // FillBase contracted by half the line spacing. Recover the bead domain;
    // retain the normal writer's variable-width flow, acceleration and speed caps.
    const ExPolygons domains = offset_ex(expolygon, float(scale_(spacing * .5)));
    if (domains.size() == 1) {
        const double nozzle             = params.flow.nozzle_diameter();
        const double rounded_correction = params.layer_height * (1. - .25 * 3.14159265358979323846);
        auto result                     = morphing_spiral(domains.front(), spacing, std::max(.01, .75 * nozzle - rounded_correction),
                                                          1.5 * nozzle - rounded_correction);
        if (!result.paths.empty()) {
            // The spiral runs outer edge to centre; print it the other way for an outward fill.
            if (params.fill_order != SurfaceFillOrder::Inward)
                for (auto& path : result.paths)
                    path.reverse();
            append(out, std::move(result.paths));
            return;
        }
        BOOST_LOG_TRIVIAL(warning) << "Morphing spiral fallback: " << result.fallback_reason;
    } else
        BOOST_LOG_TRIVIAL(warning) << "Morphing spiral fallback: offset topology";
    FillSpiralInset::_fill_surface_single(params, layers, direction, std::move(expolygon), out);
}
} // namespace Slic3r
