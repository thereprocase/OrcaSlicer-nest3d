#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/GCode/StartGCodeExtrusion.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

#include <cmath>
#include <string>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// A bed-edge purge strip in the style of a P1S start G-code: literal X/Y, E and F from expressions over
// filament settings, relative E, and a narrow tab. Two passes 0.3356 mm apart from X22 to X232, then a
// tab from X232.2 to X239.8 between Y-0.8 and Y0.8, in G-code coordinates.
const std::string PurgeStrip = R"(G28
G90
M83
G1 Z0.8 F1200
G1 X22 Y-0.16781 F12000
M109 S{nozzle_temperature_initial_layer[initial_extruder]}
G1 Z0.3 F600
{local fa = 0.7853981633974483 * filament_diameter[initial_extruder] * filament_diameter[initial_extruder]}
{local ratio = filament_flow_ratio[initial_extruder]}
G1 X232 Y-0.16781 E{21.144 * ratio / fa} F{min(filament_max_volumetric_speed[initial_extruder] / 0.1007 * 60, 12000)}
G1 X232 Y0.16781 E{0.0338 * ratio / fa}
G1 X22 Y0.16781 E{21.144 * ratio / fa}
G1 X22 Y5 F12000 ; travel
G1 X232.2 Y-0.8 F12000
G1 X239.8 Y-0.8 E{0.7712 * ratio / fa}
G1 X239.8 Y0.8 E{0.1625 * ratio / fa}
G1 X232.2 Y0.8 E{0.7712 * ratio / fa}
G1 E-0.5 F1200
G1 Z0.9
G1 E0.48 F1200
)";

const Polygon Plate = Polygon::new_scale({{0, 0}, {256, 0}, {256, 256}, {0, 256}});

DynamicPrintConfig p1s_config(const std::string &start_gcode)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("printable_area", new ConfigOptionPoints({Vec2d(0, 0), Vec2d(256, 0), Vec2d(256, 256), Vec2d(0, 256)}));
    config.set_key_value("extruder_offset", new ConfigOptionPoints({Vec2d(0, 2)}));
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats({0.4}));
    config.set_key_value("filament_diameter", new ConfigOptionFloats({1.75}));
    config.set_key_value("machine_start_gcode", new ConfigOptionString(start_gcode));
    return config;
}

} // namespace

TEST_CASE("The start G-code purge strip is found in plate coordinates", "[StartGCodeExtrusion]")
{
    std::string      error;
    const ExPolygons area = start_gcode_extrusion_area(p1s_config(PurgeStrip), 0, &error);
    INFO(error);
    REQUIRE_FALSE(area.empty());
    // The extruder offset of 0x2 moves G-code Y up by 2 mm on the plate. Every bead is at least the
    // 0.4 mm nozzle wide, with square ends: X from 22 - 0.2 to 239.8 + 0.2, Y from -0.8 + 2 - 0.2 to
    // 0.8 + 2 + 0.2. The 0.3 mm high strip beads come out about 0.4 mm wide.
    const BoundingBoxf bb(unscaled(get_extents(area).min), unscaled(get_extents(area).max));
    CHECK_THAT(bb.min.x(), WithinAbs(21.8, 0.05));
    CHECK_THAT(bb.max.x(), WithinAbs(240.0, 0.05));
    CHECK_THAT(bb.min.y(), WithinAbs(1.0, 0.05));
    CHECK_THAT(bb.max.y(), WithinAbs(3.0, 0.05));
    // The travel to Y5 extrudes nothing, and the final unretraction only refills the nozzle.
    const ExPolygons travel = intersection_ex(area, ExPolygon(Polygon::new_scale({{21, 3.2}, {23, 3.2}, {23, 8}, {21, 8}})));
    CHECK(travel.empty());
}

TEST_CASE("Relative moves, G92 and arcs land where the firmware puts them", "[StartGCodeExtrusion]")
{
    // Absolute X10 Y10, then a relative 20 mm extrusion along X, then G92 renames the spot X0 and an
    // absolute move to X5 extrudes 5 more mm, ending at plate X35. Still 30 mm off, X65 is plate X95, from
    // where a G3 full circle of radius 5 about (100, 100) extrudes too. One unit of E per mm³ this thin
    // gives beads as wide as the 0.4 mm nozzle.
    const std::string gcode = "G90\nM82\nG92 E0\nG1 X10 Y10 Z0.2\nG91\nG1 X20 E1\nG90\nG92 X0\nG1 X5 E1.25\n"
                              "G1 X65 Y100\nG3 I5 J0 E3\n";
    const ExPolygons  area  = gcode_extrusion_area(gcode, Plate, Vec2d::Zero(), 0.4, 1.);
    const BoundingBox bb    = get_extents(area);
    CHECK_THAT(unscaled(bb.min.x()), WithinAbs(9.8, 0.05));
    CHECK_THAT(unscaled(bb.max.x()), WithinAbs(105.2, 0.05));
    CHECK_THAT(unscaled(bb.max.y()), WithinAbs(105.2, 0.05));
    // The strip ends at X35 plus half a bead.
    const ExPolygons strip = intersection_ex(area, ExPolygon(Polygon::new_scale({{0, 0}, {60, 0}, {60, 20}, {0, 20}})));
    CHECK_THAT(unscaled(get_extents(strip).max.x()), WithinAbs(35.2, 0.05));
}

TEST_CASE("A start G-code without extrusion reserves nothing", "[StartGCodeExtrusion]")
{
    // Homing, travel and a retraction, as the default start G-code does no purge.
    const std::string gcode = "G28 ; home\nG1 Z5 F5000\nG92 E0\nG1 X100 Y-2 F3000\nG1 E-1 F300\nG1 X120 E-1.5\n";
    std::string       error;
    CHECK(start_gcode_extrusion_area(p1s_config(gcode), 0, &error).empty());
    CHECK(error.empty());
    CHECK(start_gcode_extrusion_area(DynamicPrintConfig::full_print_config(), 0).empty());
}

TEST_CASE("A flush beside the plate reserves nothing and one over it a puddle", "[StartGCodeExtrusion]")
{
    // 50 mm³ pushed out standing still: behind the plate into the chute, then 0.4 mm above the plate.
    const std::string chute = "G90\nM83\nG1 X60 Y265 Z5\nG1 E20.8\n";
    CHECK(gcode_extrusion_area(chute, Plate, Vec2d::Zero(), 0.4, 2.405).empty());
    const std::string on_plate = "G90\nM83\nG1 X60 Y100 Z0.4\nG1 E20.8\n";
    const ExPolygons  puddle   = gcode_extrusion_area(on_plate, Plate, Vec2d::Zero(), 0.4, 2.405);
    REQUIRE(puddle.size() == 1);
    // A disc of 50 mm³ / 0.4 mm, about 6.3 mm in radius, drawn as a polygon around the circle.
    CHECK_THAT(unscaled(get_extents(puddle).size().x()), WithinRel(2. * std::sqrt(20.8 * 2.405 / (PI * 0.4)), 0.01));
}

TEST_CASE("Moves from an unknown position reserve nothing", "[StartGCodeExtrusion]")
{
    // After homing, X is unknown until the G-code names it, so the first extruding move has no start.
    const std::string gcode = "G28\nG1 Y20 Z0.3 E2\nG1 X30 Y20 E4\n";
    CHECK(gcode_extrusion_area(gcode, Plate, Vec2d::Zero(), 0.4, 2.405).empty());
}

TEST_CASE("A start G-code that cannot be evaluated before slicing reserves nothing and says why", "[StartGCodeExtrusion]")
{
    std::string error;
    CHECK(start_gcode_extrusion_area(p1s_config("G1 X10 Y10 Z0.3\nG1 X50 E{no_such_variable}\n"), 0, &error).empty());
    CHECK_FALSE(error.empty());
}
