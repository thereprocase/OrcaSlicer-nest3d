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
} // namespace
TEST_CASE("Morphing spiral covers a smooth annulus with one interior path", "[Fill][MorphingSpiral]")
{
    for (int lobes : {0, 8}) {
        DYNAMIC_SECTION("lobes " << lobes)
        {
            auto result = Slic3r::morphing_spiral(annulus(lobes), .6, .45, .9);
            CAPTURE(result.max_width_change_rate, result.fallback_reason, result.uncovered_fraction, result.outside_fraction,
                    result.excess_fraction);
            REQUIRE(result.paths.size() == 5);
            CHECK(result.paths[2].points.front() != result.paths[2].points.back());
            CHECK(result.largest_gap_mm2 <= .25);
            CHECK(result.largest_spill_mm2 <= .10);
            CHECK(result.uncovered_fraction <= .002);
            CHECK(result.outside_fraction <= .002);
            CHECK(std::abs(result.excess_fraction) <= .005);
            for (size_t index = 0; index < result.paths.size(); ++index) {
                const auto& path = result.paths[index];
                REQUIRE(path.width.size() == 2 * (path.points.size() - 1));
                for (double width : path.width) {
                    CHECK(Slic3r::unscale<double>(width) >= (index < 3 ? .45 : .06) - 1e-6);
                    CHECK(Slic3r::unscale<double>(width) <= .9 + 1e-6);
                }
            }
        }
    }
}
TEST_CASE("Morphing spiral rejects unsupported islands without partial output", "[Fill][MorphingSpiral]")
{
    auto p = annulus();
    p.holes.clear();
    auto result = Slic3r::morphing_spiral(p, .6, .45, .9);
    CHECK(result.paths.empty());
    CHECK(!result.fallback_reason.empty());
    p = annulus();
    p.holes.emplace_back(p.holes.front());
    CHECK(Slic3r::morphing_spiral(p, .6, .45, .9).paths.empty());
    CHECK(Slic3r::morphing_spiral(annulus(0, 39), .6, .45, .9).paths.empty());
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
    Slic3r::ExPolygon disk;
    disk.contour.points = {Slic3r::Point::new_scale(0, 0), Slic3r::Point::new_scale(12, 0), Slic3r::Point::new_scale(12, 12),
                           Slic3r::Point::new_scale(0, 12)};
    Slic3r::Surface surface(Slic3r::stTop, disk);
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

TEST_CASE("Bottom surface keeps tapered gap arcs after the main spiral", "[Fill][MorphingSpiral]")
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
            REQUIRE(paths.size() == 5);
            // Both supplemental paths are open arcs, regardless of surface order.
            for (size_t i = 3; i < 5; ++i) {
                CHECK(paths[i].points.front() != paths[i].points.back());
                CHECK(*std::min_element(paths[i].width.begin(), paths[i].width.end()) < scale_(.2));
            }
            Slic3r::ExtrusionEntitiesPtr entities;
            f->fill_surface_extrusion(&surface, params, entities);
            REQUIRE(entities.size() == 1);
            std::unique_ptr<Slic3r::ExtrusionEntity> owner(entities.front());
            auto* collection = dynamic_cast<Slic3r::ExtrusionEntityCollection*>(owner.get());
            REQUIRE(collection != nullptr);
            CHECK(collection->no_sort);
            REQUIRE(!collection->entities.empty());
            CHECK(collection->entities.back()->last_point() == paths.back().points.back());
        }
    }
}
