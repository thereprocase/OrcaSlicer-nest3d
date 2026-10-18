#include <catch2/catch_test_macros.hpp>
#include "libslic3r/Fill/FillMorphingSpiral.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include <algorithm>
#include <memory>
#include <utility>
#include <cstddef>
#include <cmath>
#include <limits>
#include <vector>

namespace {
Slic3r::ExPolygon annulus(int lobes = 0, double inner = 14.)
{
    Slic3r::ExPolygon p;
    Slic3r::Polygon hole;
    for (int i = 0; i < 720; ++i) {
        const double a = 6.283185307179586 * i / 720;
        const double r = 40 + (lobes ? 3 * std::cos(lobes * a) : 0);
        p.contour.points.emplace_back(Slic3r::Point::new_scale(r * std::cos(a), r * std::sin(a)));
        hole.points.emplace_back(Slic3r::Point::new_scale(inner * std::cos(-a), inner * std::sin(-a)));
    }
    p.holes.emplace_back(std::move(hole));
    return p;
}

Slic3r::ExPolygon disk(double radius, int lobes = 0)
{
    Slic3r::ExPolygon p;
    for (int i = 0; i < 720; ++i) {
        const double a = 6.283185307179586 * i / 720;
        const double r = radius + (lobes ? 2 * std::cos(lobes * a) : 0);
        p.contour.points.emplace_back(Slic3r::Point::new_scale(r * std::cos(a), r * std::sin(a)));
    }
    return p;
}

void check_single_path(const Slic3r::MorphingSpiralResult& result, double min_spacing, double max_spacing)
{
    CAPTURE(result.fallback_reason, result.max_width_change_rate, result.uncovered_fraction, result.outside_fraction,
            result.excess_fraction, result.largest_gap_mm2, result.largest_spill_mm2);
    REQUIRE(result.paths.size() == 1);
    const auto& path = result.paths.front();
    CHECK(path.points.front() != path.points.back());
    REQUIRE(path.width.size() == 2 * (path.points.size() - 1));
    // Distance along the path, so the start and end tapers can be told apart from the body.
    std::vector<double> along{0.};
    for (size_t i = 0; i + 1 < path.points.size(); ++i)
        along.push_back(along.back() + Slic3r::unscale<double>((path.points[i + 1] - path.points[i]).cast<double>().norm()));
    for (size_t i = 0; i < path.width.size(); ++i) {
        const double width = Slic3r::unscale<double>(path.width[i]);
        CHECK(width > 0.);
        CHECK(width <= max_spacing + 1e-6);
        // Away from the tapers (at most about 12 mm long here), every bead meets the nozzle minimum.
        const double at = along[i / 2 + (i % 2)];
        if (at > 20. && at < along.back() - 20.)
            CHECK(width >= min_spacing - 1e-6);
    }
    CHECK(result.max_width_change_rate <= .20 + 1e-9);
    CHECK(result.largest_gap_mm2 <= .25);
    CHECK(result.largest_spill_mm2 <= .10);
    CHECK(result.uncovered_fraction <= .002);
    CHECK(result.outside_fraction <= .002);
    CHECK(std::abs(result.excess_fraction) <= .005);
}
} // namespace

TEST_CASE("Morphing spiral covers an annulus with one continuous path", "[Fill][MorphingSpiral]")
{
    for (int lobes : {0, 8}) {
        DYNAMIC_SECTION("lobes " << lobes) { check_single_path(Slic3r::morphing_spiral(annulus(lobes), .6, .45, .9), .45, .9); }
    }
}

TEST_CASE("Morphing spiral keeps a wide first-layer line inside a 0.4 nozzle's bounds", "[Fill][MorphingSpiral]")
{
    // 0.5 mm first-layer line on a 0.4 nozzle at 0.2 mm: the case the earlier endpoint widening rejected.
    const double correction = .2 * (1. - .25 * 3.14159265358979323846);
    const double spacing = .5 - correction, low = .3 - correction, high = .6 - correction;
    check_single_path(Slic3r::morphing_spiral(annulus(8), spacing, low, high), low, high);
}

TEST_CASE("Morphing spiral spirals a disk into its centre", "[Fill][MorphingSpiral]")
{
    // A round disk; lumpy disks spiral too, but their smoothed centre sits near the audit limits.
    check_single_path(Slic3r::morphing_spiral(disk(15), .6, .45, .9), .45, .9);
}

TEST_CASE("Morphing spiral rejects unsupported islands without partial output", "[Fill][MorphingSpiral]")
{
    auto p = annulus();
    p.holes.emplace_back(p.holes.front());
    auto result = Slic3r::morphing_spiral(p, .6, .45, .9);
    CHECK(result.paths.empty());
    CHECK(!result.fallback_reason.empty());
    CHECK(Slic3r::morphing_spiral(annulus(0, 39.5), .6, .45, .9).paths.empty()); // narrower than two beads
    CHECK(Slic3r::morphing_spiral(annulus(8), .6, .59, .61).paths.empty());
    CHECK(Slic3r::morphing_spiral(annulus(), std::numeric_limits<double>::quiet_NaN(), .45, .9).paths.empty());
}

TEST_CASE("Morphing spiral is available by config name and preserves stock fallback", "[Fill][MorphingSpiral]")
{
    Slic3r::PrintConfig pc;
    Slic3r::PrintObjectConfig oc;
    Slic3r::FillParams params;
    params.density        = 1.f;
    params.dont_adjust    = false;
    params.use_arachne    = true;
    params.layer_height   = .2;
    params.flow           = Slic3r::Flow(.6f, .2f, .6f);
    params.extrusion_role = Slic3r::erTopSolidInfill;
    auto make             = [&](const char* name) {
        std::unique_ptr<Slic3r::Fill> f(Slic3r::Fill::new_from_type(name));
        REQUIRE(f != nullptr);
        f->spacing             = params.flow.spacing();
        f->overlap             = .5 * f->spacing;
        f->print_config        = &pc;
        f->print_object_config = &oc;
        return f;
    };
    // Two holes cannot be one spiral, so the island must come out exactly as stock spiral inset.
    Slic3r::ExPolygon two_holes = annulus();
    Slic3r::Polygon second;
    for (int i = 0; i < 90; ++i) {
        const double a = 6.283185307179586 * i / 90;
        second.points.emplace_back(Slic3r::Point::new_scale(27 + 4 * std::cos(-a), 4 * std::sin(-a)));
    }
    two_holes.holes.emplace_back(std::move(second));
    Slic3r::Surface surface(Slic3r::stTop, two_holes);
    auto experiment = make("morphingspiral");
    auto stock      = make("spiralinset");
    auto actual     = experiment->fill_surface_arachne(&surface, params);
    auto expected   = stock->fill_surface_arachne(&surface, params);
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        CHECK(actual[i].points == expected[i].points);
        CHECK(actual[i].width == expected[i].width);
    }
}

TEST_CASE("Bottom surface prints the spiral as one path in either order", "[Fill][MorphingSpiral]")
{
    for (auto order : {Slic3r::SurfaceFillOrder::Inward, Slic3r::SurfaceFillOrder::Outward}) {
        DYNAMIC_SECTION("order " << int(order))
        {
            Slic3r::PrintConfig pc;
            Slic3r::PrintObjectConfig oc;
            oc.gap_fill_target.value = Slic3r::gftNowhere;
            Slic3r::FillParams params;
            params.density        = 1.f;
            params.dont_adjust    = false;
            params.use_arachne    = true;
            params.layer_height   = .2;
            params.flow           = Slic3r::Flow(.6f, .2f, .6f);
            params.extrusion_role = Slic3r::erBottomSurface;
            params.fill_order     = order;
            std::unique_ptr<Slic3r::Fill> f(Slic3r::Fill::new_from_type("morphingspiral"));
            f->spacing             = params.flow.spacing();
            f->overlap             = .5 * f->spacing;
            f->print_config        = &pc;
            f->print_object_config = &oc;
            Slic3r::Surface surface(Slic3r::stBottom, annulus());
            const auto paths = f->fill_surface_arachne(&surface, params);
            REQUIRE(paths.size() == 1);
            // Inward starts at the outer edge (radius about 40); outward starts at the hole (about 14).
            const double start_radius = Slic3r::unscale<double>(paths.front().points.front().cast<double>().norm());
            if (order == Slic3r::SurfaceFillOrder::Inward)
                CHECK(start_radius > 30.);
            else
                CHECK(start_radius < 20.);
            Slic3r::ExtrusionEntitiesPtr entities;
            f->fill_surface_extrusion(&surface, params, entities);
            REQUIRE(entities.size() == 1);
            std::unique_ptr<Slic3r::ExtrusionEntity> owner(entities.front());
            auto* collection = dynamic_cast<Slic3r::ExtrusionEntityCollection*>(owner.get());
            REQUIRE(collection != nullptr);
            REQUIRE(!collection->entities.empty());
            CHECK(collection->entities.back()->last_point() == paths.back().points.back());
        }
    }
}
