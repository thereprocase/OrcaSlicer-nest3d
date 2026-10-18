#include <catch2/catch_test_macros.hpp>
#include "libslic3r/Fill/FillMorphingSpiral.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Emboss.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "test_helpers.hpp"
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

// A 1 mm round plate (r 20) with two holes (r 4 at x = +-9): no single spiral can fill it.
Slic3r::TriangleMesh two_hole_plate()
{
    // Emboss::ProjectZ extrudes raw coordinates, so build in 0.01 mm units and scale down.
    Slic3r::ExPolygon plate;
    for (int i = 0; i < 360; ++i) {
        const double a = 6.283185307179586 * i / 360;
        plate.contour.points.emplace_back(Slic3r::Point(2000 * std::cos(a), 2000 * std::sin(a)));
    }
    for (int x : {-900, 900}) {
        Slic3r::Polygon hole;
        for (int i = 0; i < 90; ++i) {
            const double a = 6.283185307179586 * i / 90;
            hole.points.emplace_back(Slic3r::Point(x + 400 * std::cos(-a), 400 * std::sin(-a)));
        }
        plate.holes.emplace_back(std::move(hole));
    }
    Slic3r::TriangleMesh mesh(Slic3r::Emboss::polygons2model(Slic3r::ExPolygons{plate}, Slic3r::Emboss::ProjectZ(100.)));
    mesh.scale(.01f);
    return mesh;
}

// A square of the given side with a round hole, both centred on the origin.
Slic3r::ExPolygon square_with_hole(double side, double hole_diameter)
{
    Slic3r::ExPolygon p;
    const double h = side / 2;
    for (const auto& [x, y] : {std::pair{-h, -h}, std::pair{h, -h}, std::pair{h, h}, std::pair{-h, h}})
        p.contour.points.emplace_back(Slic3r::Point::new_scale(x, y));
    Slic3r::Polygon hole;
    for (int i = 0; i < 360; ++i) {
        const double a = 6.283185307179586 * i / 360;
        hole.points.emplace_back(Slic3r::Point::new_scale(hole_diameter / 2 * std::cos(-a), hole_diameter / 2 * std::sin(-a)));
    }
    p.holes.emplace_back(std::move(hole));
    return p;
}

// A 1 mm tall 20 mm square plate with a 17.2 mm round hole. Without walls each side is a 1.4 mm
// band: three beads wide, but only one ring fits each side, so the Fermat spiral would need beads
// past the nozzle maximum; the corners are far too thick for one radial spiral.
Slic3r::TriangleMesh narrow_square_frame()
{
    Slic3r::ExPolygon frame;
    for (const auto& [x, y] : {std::pair{-1000, -1000}, std::pair{1000, -1000}, std::pair{1000, 1000}, std::pair{-1000, 1000}})
        frame.contour.points.emplace_back(Slic3r::Point(x, y));
    Slic3r::Polygon hole;
    for (int i = 0; i < 180; ++i) {
        const double a = 6.283185307179586 * i / 180;
        hole.points.emplace_back(Slic3r::Point(860 * std::cos(-a), 860 * std::sin(-a)));
    }
    frame.holes.emplace_back(std::move(hole));
    Slic3r::TriangleMesh mesh(Slic3r::Emboss::polygons2model(Slic3r::ExPolygons{frame}, Slic3r::Emboss::ProjectZ(100.)));
    mesh.scale(.01f);
    return mesh;
}

std::vector<std::string> current_morphing_warnings(const Slic3r::Print& print)
{
    std::vector<std::string> messages;
    for (const Slic3r::PrintObject* object : print.objects())
        for (const auto& warning : object->step_state_with_warnings(Slic3r::posInfill).warnings)
            if (warning.current && warning.message_id == Slic3r::PrintStateBase::SlicingMorphingSpiralFallback)
                messages.push_back(warning.message);
    return messages;
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

// The Fermat spiral's own budget (see fermat_spiral): one path, every bead inside the nozzle
// bounds, and coverage within its looser limits.
void check_fermat_path(const Slic3r::MorphingSpiralResult& result, double min_spacing, double max_spacing)
{
    CAPTURE(result.fallback_reason, result.uncovered_fraction, result.outside_fraction, result.excess_fraction,
            result.largest_gap_mm2, result.largest_spill_mm2);
    REQUIRE(result.paths.size() == 1);
    const auto& path = result.paths.front();
    REQUIRE(path.width.size() == 2 * (path.points.size() - 1));
    for (const auto width : path.width) {
        CHECK(Slic3r::unscale<double>(width) >= min_spacing - 1e-6);
        CHECK(Slic3r::unscale<double>(width) <= max_spacing + 1e-6);
    }
    CHECK(result.largest_gap_mm2 <= .45);
    CHECK(result.largest_spill_mm2 <= .10);
    CHECK(result.uncovered_fraction <= .012);
    CHECK(result.outside_fraction <= .002);
    CHECK(std::abs(result.excess_fraction) <= .03);
}

// A 0.5 mm line on a 0.4 nozzle at 0.2 mm layers, as spacing and bead bounds.
constexpr double rounded = .2 * (1. - .25 * 3.14159265358979323846);
constexpr double spacing_04 = .5 - rounded, min_04 = .3 - rounded, max_04 = .6 - rounded;
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
    // A 2 mm band around a 16 mm hole in a 20 mm square: too thick at the corners for the radial
    // spiral, and three beads wide on the sides where only one Fermat ring fits each side, so
    // neither spiral fits and the island must come out exactly as stock spiral inset.
    Slic3r::Surface surface(Slic3r::stTop, square_with_hole(20, 16));
    auto experiment = make("morphingspiral");
    auto stock      = make("spiralinset");
    auto actual     = experiment->fill_surface_arachne(&surface, params);
    auto expected   = stock->fill_surface_arachne(&surface, params);
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        CHECK(actual[i].points == expected[i].points);
        CHECK(actual[i].width == expected[i].width);
    }
    // The fallback is recorded so the layer can report it as a slicing warning.
    const auto* morphing = dynamic_cast<const Slic3r::FillMorphingSpiral*>(experiment.get());
    REQUIRE(morphing != nullptr);
    REQUIRE(morphing->fallback_reasons.size() == 1);
    CHECK(!morphing->fallback_reasons.front().empty());
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

TEST_CASE("A fallback island raises one slicing warning that a stock pattern clears", "[Fill][MorphingSpiral]")
{
    Slic3r::Print print;
    Slic3r::Model model;
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict(
        {{"bottom_surface_pattern", "morphingspiral"}, {"top_surface_pattern", "morphingspiral"}, {"wall_loops", "0"}});
    Slic3r::Test::init_print(std::vector<Slic3r::TriangleMesh>{narrow_square_frame()}, print, model, config);
    print.process();
    // Every layer's island falls back, but they are reported as one aggregated warning.
    const auto first = current_morphing_warnings(print);
    REQUIRE(first.size() == 1);
    CHECK(first.front().find("Spiral Inset") != std::string::npos);

    // Regenerating infill with the spiral still selected must give the same single report,
    // with the same island count, not an accumulated one.
    config.set_deserialize_strict({{"infill_direction", "30"}});
    print.apply(model, config);
    print.process();
    CHECK(current_morphing_warnings(print) == first);

    config.set_deserialize_strict({{"bottom_surface_pattern", "monotonic"}, {"top_surface_pattern", "monotonic"}});
    print.apply(model, config);
    print.process();
    CHECK(current_morphing_warnings(print).empty());
}

TEST_CASE("Fermat spiral fills islands the radial spiral cannot as one path", "[Fill][MorphingSpiral][Fermat]")
{
    auto two_holes = annulus();
    Slic3r::Polygon second;
    for (int i = 0; i < 90; ++i) {
        const double a = 6.283185307179586 * i / 90;
        second.points.emplace_back(Slic3r::Point::new_scale(27 + 4 * std::cos(-a), 4 * std::sin(-a)));
    }
    two_holes.holes.emplace_back(std::move(second));
    SECTION("two holes")
    {
        CHECK(Slic3r::morphing_spiral(two_holes, spacing_04, min_04, max_04).paths.empty());
        check_fermat_path(Slic3r::fermat_spiral(two_holes, spacing_04, min_04, max_04), min_04, max_04);
    }
    SECTION("a square around a large round hole")
    {
        const auto square = square_with_hole(39.5, 28.5);
        CHECK(Slic3r::morphing_spiral(square, spacing_04, min_04, max_04).paths.empty());
        check_fermat_path(Slic3r::fermat_spiral(square, spacing_04, min_04, max_04), min_04, max_04);
    }
}

TEST_CASE("Fermat spiral rejects what it cannot fill without partial output", "[Fill][MorphingSpiral][Fermat]")
{
    CHECK(Slic3r::fermat_spiral(disk(.4), spacing_04, min_04, max_04).paths.empty()); // smaller than one ring
    CHECK(Slic3r::fermat_spiral(annulus(), std::numeric_limits<double>::quiet_NaN(), min_04, max_04).paths.empty());
    const auto narrow = Slic3r::fermat_spiral(square_with_hole(20, 17.2), spacing_04, min_04, max_04);
    CHECK(narrow.paths.empty());
    CHECK(!narrow.fallback_reason.empty());
}

TEST_CASE("A two-hole plate spirals without a fallback warning", "[Fill][MorphingSpiral][Fermat]")
{
    Slic3r::Print print;
    Slic3r::Model model;
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"bottom_surface_pattern", "morphingspiral"}, {"top_surface_pattern", "morphingspiral"}});
    Slic3r::Test::init_print(std::vector<Slic3r::TriangleMesh>{two_hole_plate()}, print, model, config);
    print.process();
    CHECK(current_morphing_warnings(print).empty());
}