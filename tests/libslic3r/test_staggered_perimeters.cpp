#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/StaggeredPerimeters.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Point.hpp"
#include <cmath>
#include <vector>
#include <utility>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
constexpr double height = .2;
constexpr double width = .45;
double bead_area(double h) { return h * (width - h * (1. - std::acos(-1.) / 4.)); }

ExtrusionPaths square(double shift = 0.)
{
    ExtrusionPath path(erPerimeter, bead_area(height), width, height);
    for (const auto &[x,y] : {std::pair{0.,0.}, {10.,0.}, {10.,10.}, {0.,10.}, {0.,0.}})
        path.polyline.points.push_back(Point3::new_scale(x + shift, y, 0.));
    return {path};
}

struct Fixture {
    std::vector<ExtrusionPaths> paths;
    std::vector<std::vector<StaggeredWall>> layers;
    explicit Fixture(size_t count) : paths(count), layers(count) {
        for (size_t i = 0; i < count; ++i) {
            paths[i] = square();
            layers[i].push_back({&paths[i], 0, 1, (i + 1) * height, height});
        }
    }
};
}

TEST_CASE("Staggered wall beads meet without a vertical gap and finish at the local top", "[StaggeredPerimeters]")
{
    const size_t count = GENERATE(4u, 5u, 12u);
    Fixture f(count);
    REQUIRE(stagger_perimeters(f.layers) > 0);
    double previous_top = 0.;
    for (size_t i = 0; i < count; ++i) {
        const auto &p = f.paths[i].front();
        const double top = (i + 1) * height + p.staggered_z_offset;
        CHECK_THAT(top - p.height, WithinAbs(previous_top, 1e-6));
        CHECK_THAT(p.mm3_per_mm, WithinAbs(bead_area(p.height), 1e-8));
        previous_top = top;
    }
    CHECK_THAT(previous_top, WithinAbs(count * height, 1e-6));
    CHECK_THAT(f.paths.front().front().staggered_z_offset, WithinAbs(0., 1e-6));
    CHECK_THAT(f.paths.back().front().staggered_z_offset, WithinAbs(0., 1e-6));
}

TEST_CASE("Short wall tracks stay nominal", "[StaggeredPerimeters]")
{
    Fixture f(GENERATE(1u, 2u, 3u));
    CHECK(stagger_perimeters(f.layers) == 0);
    for (const auto &paths : f.paths) {
        CHECK_THAT(paths.front().height, WithinAbs(height, 1e-6));
        CHECK_THAT(paths.front().mm3_per_mm, WithinAbs(bead_area(height), 1e-8));
    }
}

TEST_CASE("A disappearing wall finishes independently of taller geometry", "[StaggeredPerimeters]")
{
    Fixture f(12);
    for (size_t i = 5; i < 12; ++i) f.paths[i] = square(20.);
    stagger_perimeters(f.layers);
    CHECK_THAT(f.paths[3].front().height, WithinAbs(.1, 1e-6));
    CHECK_THAT(f.paths[4].front().staggered_z_offset, WithinAbs(0., 1e-6));
    CHECK_THAT(f.paths[5].front().height, WithinAbs(height, 1e-6));
    CHECK_THAT(f.paths[6].front().height, WithinAbs(.3, 1e-6));
}

TEST_CASE("Sloping wall footprints stay nominal", "[StaggeredPerimeters]")
{
    Fixture f(12);
    for (size_t i = 0; i < 12; ++i) f.paths[i] = square(i * .05);
    CHECK(stagger_perimeters(f.layers) == 0);
}

TEST_CASE("Open and ambiguous tracks are not staggered", "[StaggeredPerimeters]")
{
    Fixture f(8);
    const bool open = GENERATE(true, false);
    for (size_t i = 0; i < 8; ++i) {
        if (open) f.paths[i].front().polyline.points.pop_back();
        else f.layers[i].push_back(f.layers[i].front());
    }
    CHECK(stagger_perimeters(f.layers) == 0);
}

TEST_CASE("Reversed closed walls still match their track", "[StaggeredPerimeters]")
{
    Fixture f(8);
    for (size_t i = 0; i < 8; i += 2) f.paths[i].front().reverse();
    REQUIRE(stagger_perimeters(f.layers) > 0);
}

TEST_CASE("A layer gap breaks the wall track", "[StaggeredPerimeters]")
{
    Fixture f(7);
    f.layers[3].clear();
    CHECK(stagger_perimeters(f.layers) == 0);
}

TEST_CASE("Splitting a multi-path seam preserves staggered bead geometry", "[StaggeredPerimeters]")
{
    auto paths = square();
    paths.front().height = .3f;
    paths.front().mm3_per_mm = bead_area(.3);
    paths.front().staggered_z_offset = .1f;
    ExtrusionPath second(paths.front());
    auto &points = paths.front().polyline.points;
    second.polyline.points.assign(points.begin() + 2, points.end());
    points.resize(3);
    paths.push_back(second);
    ExtrusionLoop loop(paths);
    loop.split_at(Point::new_scale(5., 0.), false);
    REQUIRE(loop.paths.size() >= 2);
    for (const auto &path : loop.paths) {
        CHECK_THAT(path.staggered_z_offset, WithinAbs(.1, 1e-6));
        CHECK_THAT(path.height, WithinAbs(.3, 1e-6));
        CHECK_THAT(path.mm3_per_mm, WithinAbs(bead_area(.3), 1e-8));
    }
}

TEST_CASE("Oversized starting beads leave the track nominal", "[StaggeredPerimeters]")
{
    Fixture f(8);
    for (auto &layer : f.layers) layer.front().maximum_bead_height = .29;
    CHECK(stagger_perimeters(f.layers) == 0);
}

TEST_CASE("Malformed empty paths do not form staggered tracks", "[StaggeredPerimeters]")
{
    Fixture f(8);
    for (auto &paths : f.paths) paths.front().polyline.points.clear();
    CHECK(stagger_perimeters(f.layers) == 0);
}

TEST_CASE("A layer height change closes the previous staggered track", "[StaggeredPerimeters]")
{
    Fixture f(10);
    for (size_t i = 5; i < 10; ++i) {
        f.layers[i].front().height = .15;
        f.layers[i].front().print_z = 1. + (i - 4) * .15;
        f.paths[i].front().height = .15f;
        f.paths[i].front().mm3_per_mm = bead_area(.15);
    }
    REQUIRE(stagger_perimeters(f.layers) > 0);
    CHECK_THAT(f.paths[4].front().height, WithinAbs(.2, 1e-6));
    CHECK_THAT(f.paths[4].front().staggered_z_offset, WithinAbs(0., 1e-6));
    CHECK_THAT(f.paths[5].front().height, WithinAbs(.15, 1e-6));
    CHECK_THAT(f.paths[6].front().height, WithinAbs(.225, 1e-6));
    CHECK_THAT(f.paths[9].front().staggered_z_offset, WithinAbs(0., 1e-6));
}
