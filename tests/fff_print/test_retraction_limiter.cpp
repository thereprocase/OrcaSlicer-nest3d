#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "test_helpers.hpp"

#include <string>
#include <sstream>

using namespace Slic3r;

namespace {
DynamicPrintConfig limiter_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"retraction_length", "0.3"}, {"retraction_minimum_travel", "0"},
        {"retract_when_changing_layer", "1"}, {"reduce_infill_retraction", "0"},
        {"reduce_crossing_wall", "0"}, {"max_travel_detour_distance", "1"},
        {"sparse_infill_density", "35%"}, {"top_shell_layers", "0"}, {"bottom_shell_layers", "0"},
        {"wall_loops", "2"}, {"z_hop", "0.4"},
        {"gcode_comments", "1"}, {"machine_start_gcode", ""}, {"machine_end_gcode", ""},
        {"retraction_extrusion_window", "100000"},
    });
    config.set_key_value("z_hop_types", new ConfigOptionEnumsGeneric{zhtNormal});
    return config;
}

int retracts(const std::string &gcode)
{
    int count = 0;
    bool retracting = false;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if ((line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) && line.has_e()) {
            const double de = line.dist_E(self);
            if (de < -1e-6 && !retracting)
                ++count;
            if (de != 0.)
                retracting = de < 0.;
        }
    });
    return count;
}

int skipped_travels_with_lift(const std::string &gcode)
{
    int count = 0;
    bool skipped = false;
    float start_z = 0.f;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.comment().find("travel retraction skipped:") != std::string::npos) {
            skipped = true;
            start_z = self.z();
        }
        if (skipped && line.has_z() && line.z() > start_z + 1e-3f) {
            ++count;
            skipped = false;
        }
        if (line.extruding(self))
            skipped = false;
    });
    return count;
}
} // namespace

TEST_CASE("Travel retraction limiting reduces cycles while retaining protected retracts", "[RetractionLimiter]")
{
    auto config = limiter_config();
    const bool wipe = GENERATE(false, true);
    config.set_deserialize_strict({{"wipe", wipe ? "1" : "0"}, {"wipe_distance", "1"}});
    const auto baseline = Test::slice({Test::TestMesh::cube_with_hole}, config);
    config.set_deserialize_strict({{"retraction_count_max", "1"}});
    const auto limited = Test::slice({Test::TestMesh::cube_with_hole}, config);
    CHECK(limited.find("travel retraction skipped: frequency limit") != std::string::npos);
    CHECK(retracts(limited) < retracts(baseline));
    // A huge window exhausts optional travel after one cycle, but visible walls
    // and the fifty layer changes must still be able to retract.
    CHECK(retracts(limited) >= 50);
    CHECK(skipped_travels_with_lift(limited) > 0);
}

TEST_CASE("A zero extrusion window disables travel retraction limiting", "[RetractionLimiter]")
{
    auto config = limiter_config();
    config.set_deserialize_strict({{"retraction_count_max", "1"}, {"retraction_extrusion_window", "0"}});
    const auto gcode = Test::slice({Test::TestMesh::cube_with_hole}, config);
    CHECK(gcode.find("travel retraction skipped: frequency limit") == std::string::npos);
}

TEST_CASE("Hard extrusion spacing overrides layer and outer wall quality retractions", "[RetractionLimiter]")
{
    auto config = limiter_config();
    config.set_deserialize_strict({{"retraction_count_max", "0"}, {"retraction_minimum_extrusion", "100000"}});
    const auto gcode = Test::slice({Test::TestMesh::cube_with_hole}, config);
    CHECK(gcode.find("travel retraction skipped: minimum extrusion") != std::string::npos);
    // Only the first normal cycle and required final retraction remain possible.
    CHECK(retracts(gcode) <= 2);
    CHECK(retracts(gcode) >= 1);
    CHECK(skipped_travels_with_lift(gcode) > 0);
}

TEST_CASE("Filament overrides can disable the printer travel retraction budget", "[RetractionLimiter]")
{
    auto config = limiter_config();
    config.set_deserialize_strict({{"retraction_count_max", "1"}, {"filament_retraction_count_max", "0"},
                                   {"filament_retraction_minimum_extrusion", "0"}});
    const auto gcode = Test::slice({Test::TestMesh::cube_with_hole}, config);
    CHECK(gcode.find("travel retraction skipped:") == std::string::npos);
}

TEST_CASE("An unused retraction budget preserves existing machine commands", "[RetractionLimiter]")
{
    auto config = limiter_config();
    config.set_deserialize_strict({{"exclude_object", "0"}, {"gcode_label_objects", "0"}});
    const auto original = Test::slice({Test::TestMesh::cube_with_hole}, config);
    config.set_deserialize_strict({{"retraction_count_max", "100000"}});
    const auto guarded = Test::slice({Test::TestMesh::cube_with_hole}, config);
    const auto commands = [](const std::string &gcode) {
        std::istringstream stream(gcode);
        std::string line, result;
        while (std::getline(stream, line)) {
            line = line.substr(0, line.find(';'));
            const auto start = line.find_first_not_of(" \t\r");
            if (start != std::string::npos)
                result += line.substr(start) + '\n';
        }
        return result;
    };
    CHECK(commands(guarded) == commands(original));
}
