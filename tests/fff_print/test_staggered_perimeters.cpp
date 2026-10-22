#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "test_helpers.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include <functional>
#include <limits>
#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
DynamicPrintConfig config(bool enabled = true)
{
    auto c = Test::multifilament_config(1, {
        {"staggered_perimeters", enabled ? "1" : "0"},
        {"wall_generator", "arachne"}, {"wall_loops", "4"},
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"outer_wall_line_width", "0.45"}, {"inner_wall_line_width", "0.45"},
        {"line_width", "0.45"}, {"initial_layer_line_width", "0.45"},
        {"top_surface_line_width", "0.45"}, {"seam_slope_type", "none"},
        {"only_one_wall_top", "0"}, {"only_one_wall_first_layer", "0"},
        {"arc_fitting", "0"}, {"enable_support", "0"},
        {"raft_layers", "0"}, {"sparse_infill_density", "15%"}
    });
    return c;
}

void visit(const ExtrusionEntity &entity, const std::function<void(const ExtrusionPath &)> &fn)
{
    if (const auto *c = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const auto *child : c->entities) visit(*child, fn);
    else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const auto &p : loop->paths) fn(p);
    else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const auto &p : multi->paths) fn(p);
    else if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) fn(*path);
}
}

TEST_CASE("Brick layers finish both heights of a stepped object at nominal Z", "[StaggeredPerimeters]")
{
    auto low = make_cube(10., 10., 2.);
    auto high = make_cube(10., 10., 4.);
    high.translate(20., 0., 0.);
    low.merge(high);
    Print print;
    Model model;
    Test::init_print(std::vector<TriangleMesh>{low}, print, model, config(), nullptr, false);
    auto *object = print.objects().front();
    print.process();
    double min_x = std::numeric_limits<double>::max(), max_x = -min_x;
    for (const auto *region : object->layers().front()->regions())
        visit(region->perimeters, [&](const ExtrusionPath &path) {
            for (const auto &point : path.polyline.points) {
                min_x = std::min(min_x, double(unscale_(point.x())));
                max_x = std::max(max_x, double(unscale_(point.x())));
            }
        });
    const double middle_x = (min_x + max_x) * .5;
    size_t raised_low = 0, raised_high = 0;
    for (const Layer *layer : object->layers())
        for (const auto *region : layer->regions())
            visit(region->perimeters, [&](const ExtrusionPath &path) {
                const bool lower = unscale_(path.first_point().x()) < middle_x;
                if (path.staggered_z_offset > 0.) {
                    if (lower) ++raised_low;
                    else ++raised_high;
                }
                const double top = lower ? 2. : 4.;
                CHECK(layer->print_z + path.staggered_z_offset <= top + 1e-5);
                if (layer->print_z > top - .21)
                    CHECK_THAT(path.staggered_z_offset, WithinAbs(0., 1e-6));
            });
    REQUIRE(raised_low > 0);
    REQUIRE(raised_high > 0);
}

TEST_CASE("Scarf and Z contouring cannot silently combine with brick layers", "[StaggeredPerimeters]")
{
    auto c = config();
    const std::string option = GENERATE(std::string("seam_slope_type"), std::string("zaa_enabled"));
    c.set_deserialize_strict(option, option == "seam_slope_type" ? "all" : "1");
    Print print;
    Model model;
    Test::init_print(std::vector<TriangleMesh>{make_cube(10., 10., 2.)}, print, model, c, nullptr, false);
    const auto error = print.validate();
    CHECK(error.opt_key == "staggered_perimeters");
    CHECK_THROWS(print.process());
}

TEST_CASE("Disabled brick layers leave every wall at nominal height", "[StaggeredPerimeters]")
{
    Print print;
    Model model;
    Test::init_print(std::vector<TriangleMesh>{make_cube(10., 10., 2.)}, print, model, config(false), nullptr, false);
    auto *object = print.objects().front();
    print.process();
    size_t paths = 0;
    for (const auto *layer : object->layers())
        for (const auto *region : layer->regions())
            visit(region->perimeters, [&](const ExtrusionPath &path) {
                ++paths;
                CHECK_THAT(path.staggered_z_offset, WithinAbs(0., 1e-6));
                CHECK_THAT(path.height, WithinAbs(layer->height, 1e-6));
            });
    REQUIRE(paths > 0);
}

TEST_CASE("Brick-layer travel lifts vertically before XY and descends at the destination", "[StaggeredPerimeters]")
{
    Print print;
    Model model;
    auto c = config();
    c.set_deserialize_strict("gcode_comments", "1");
    Test::init_print(std::vector<TriangleMesh>{make_cube(10., 10., 2.)}, print, model, c, nullptr, false);
    const auto code = Test::gcode(print);
    std::istringstream lines(code);
    std::string line;
    size_t lifts = 0, descents = 0, xy = 0;
    bool crossing = false;
    while (std::getline(lines, line)) {
        if (line.find("clear staggered walls") != std::string::npos) {
            CHECK_FALSE(crossing);
            CHECK(line.find(" X") == std::string::npos);
            CHECK(line.find(" Y") == std::string::npos);
            crossing = true;
            ++lifts;
        } else if (line.find("descend at wall start") != std::string::npos) {
            CHECK(crossing);
            CHECK(line.find(" X") == std::string::npos);
            CHECK(line.find(" Y") == std::string::npos);
            crossing = false;
            ++descents;
        } else if (crossing && line.rfind("G1 ", 0) == 0) {
            CHECK(line.find(" Z") == std::string::npos);
            CHECK(line.find(" E") == std::string::npos);
            ++xy;
        }
    }
    REQUIRE(lifts > 0);
    CHECK(lifts == descents);
    CHECK(xy >= lifts);
    CHECK_FALSE(crossing);
}

TEST_CASE("Changing to scarf after slicing cannot export cached staggered walls", "[StaggeredPerimeters]")
{
    Print print;
    Model model;
    auto c = config();
    Test::init_print(std::vector<TriangleMesh>{make_cube(10., 10., 2.)}, print, model, c, nullptr, false);
    print.process();
    c.set_deserialize_strict("seam_slope_type", "all");
    print.apply(model, c);
    CHECK_THROWS(Test::gcode(print));
}

TEST_CASE("Short brick-layer objects warn once and disabling clears the warning", "[StaggeredPerimeters]")
{
    Print print;
    Model model;
    auto c = config();
    Test::init_print(std::vector<TriangleMesh>{make_cube(10., 10., .4)}, print, model, c, nullptr, false);
    print.process();
    const auto count_warnings = [&] {
        size_t count = 0;
        for (const auto &warning : print.objects().front()->step_state_with_warnings(posPerimeters).warnings)
            if (warning.current && warning.message_id == PrintStateBase::SlicingStaggeredPerimetersFallback) ++count;
        return count;
    };
    CHECK(count_warnings() == 1);
    print.process();
    CHECK(count_warnings() == 1);
    c.set_deserialize_strict("staggered_perimeters", "0");
    print.apply(model, c);
    print.process();
    CHECK(count_warnings() == 0);
}
