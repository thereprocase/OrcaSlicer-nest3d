#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "test_helpers.hpp"

#include <algorithm>
#include <cstddef>
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {
struct FanSample {
    int layer;
    int tool;
    int part;
    int auxiliary;
};

// Observe fan state during deposition, after both layer hooks and cooling-buffer restoration.
std::vector<FanSample> deposition_fans(const std::string &gcode, const DynamicPrintConfig &config)
{
    std::vector<FanSample> samples;
    GCodeReader reader;
    reader.apply_config(config);
    FanSample state{-1, 0, -1, -1};
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.raw().rfind(";AUX_TEST_LAYER=", 0) == 0)
            state.layer = std::stoi(line.raw().substr(16));
        else if (line.cmd_is("M106")) {
            float channel = 1.f;
            float speed = 0.f;
            line.has_value('P', channel);
            if (line.has_value('S', speed)) {
                if (channel == 2.f)
                    state.auxiliary = int(speed);
                else if (channel == 1.f)
                    state.part = int(speed);
            }
        } else if (line.cmd_is("M107")) {
            state.part = 0;
        } else if (!line.cmd().empty() && line.cmd().front() == 'T') {
            state.tool = std::stoi(std::string(line.cmd().substr(1)));
        } else if (state.layer >= 0 && line.extruding(self) && (line.has_x() || line.has_y())) {
            samples.push_back(state);
        }
    });
    return samples;
}

DynamicPrintConfig auxiliary_cooling_config(unsigned int filaments = 1)
{
    return multifilament_config(filaments, {
        {"auxiliary_fan", true},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"layer_change_gcode", ";AUX_TEST_LAYER={layer_num}"},
        {"machine_start_gcode", ""},
        {"machine_end_gcode", ""},
        {"use_relative_e_distances", false},
        {"close_fan_the_first_x_layers", 1},
        {"full_fan_speed_layer", 0},
        {"initial_layer_fan_speed", -1},
        {"fan_min_speed", "20"},
        {"fan_max_speed", "20"},
        {"reduce_fan_stop_start_freq", true},
        {"slow_down_for_layer_cooling", false},
        {"enable_overhang_bridge_fan", false},
        {"close_additional_fan_first_x_layers", 3},
        {"first_x_layer_fan_speed", "0"},
        {"additional_fan_full_speed_layer", 0},
        {"additional_cooling_fan_speed", 60},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"enable_prime_tower", false},
    });
}
} // namespace

TEST_CASE("Auxiliary cooling follows its own initial layers and ramp", "[Cooling][AuxiliaryFan][Regression]")
{
    auto [part_delay, first_speed, aux_delay, full_layer, target, expected] =
        GENERATE(table<int, int, int, int, int, std::vector<int>>({
            {1, 0, 3, 0, 60, {0, 0, 0, 60, 60, 60}},
            {3, 10, 1, 0, 60, {10, 60, 60, 60, 60, 60}},
            {1, 20, 2, 5, 80, {20, 20, 40, 60, 80, 80}},
            {1, 0, 0, 4, 80, {20, 40, 60, 80, 80, 80}},
            {1, 80, 1, 4, 20, {80, 60, 40, 20, 20, 20}},
            {1, 10, 3, 1, 60, {10, 10, 10, 60, 60, 60}},
        }));
    auto config = auxiliary_cooling_config();
    config.set_deserialize_strict({
        {"close_fan_the_first_x_layers", part_delay},
        {"first_x_layer_fan_speed", first_speed},
        {"close_additional_fan_first_x_layers", aux_delay},
        {"additional_fan_full_speed_layer", full_layer},
        {"additional_cooling_fan_speed", target},
    });
    // Six 0.2 mm layers exercise the delayed start and the end of each ramp.
    const auto samples = deposition_fans(slice({make_cube(10., 10., 1.2)}, config), config);
    REQUIRE_FALSE(samples.empty());
    std::vector<bool> seen(expected.size(), false);
    for (const auto &sample : samples) {
        REQUIRE(sample.layer < int(expected.size()));
        seen[sample.layer] = true;
        INFO("layer " << sample.layer << ", initial auxiliary " << first_speed << ", delay " << aux_delay << ", full layer " << full_layer);
        CHECK(sample.auxiliary == int(255.0 * expected[sample.layer] / 100.0));
        CHECK(sample.part == (sample.layer < part_delay ? 0 : 51));
    }
    CHECK(std::all_of(seen.begin(), seen.end(), [](bool value) { return value; }));
}

TEST_CASE("First layer part cooling does not override auxiliary cooling", "[Cooling][AuxiliaryFan]")
{
    auto config = auxiliary_cooling_config();
    config.set_deserialize_strict({{"close_fan_the_first_x_layers", 0}, {"initial_layer_fan_speed", 40}});
    const auto samples = deposition_fans(slice({make_cube(10., 10., 1.2)}, config), config);
    REQUIRE_FALSE(samples.empty());
    for (const auto &sample : samples) {
        CHECK(sample.auxiliary == (sample.layer < 3 ? 0 : 153));
        CHECK(sample.part == (sample.layer == 0 ? 102 : 51));
    }
}

TEST_CASE("A printer without an auxiliary fan emits no auxiliary commands", "[Cooling][AuxiliaryFan]")
{
    auto config = auxiliary_cooling_config();
    config.set_deserialize_strict({{"auxiliary_fan", false}, {"first_x_layer_fan_speed", "25"}});
    const auto gcode = slice({make_cube(10., 10., 1.2)}, config);
    CHECK(gcode.find("M106 P2") == std::string::npos);
}

TEST_CASE("Material changes restore each filament's auxiliary cooling schedule", "[Cooling][AuxiliaryFan][Regression]")
{
    const auto nozzle_volume_type = GENERATE(nvtStandard, nvtHighFlow);
    auto config = auxiliary_cooling_config(2);
    config.set_deserialize_strict({
        // Timing stays per filament even when the target speeds occupy three variant slots.
        {"extruder_variant_list", "Direct Drive Standard,Direct Drive High Flow"},
        {"filament_extruder_variant", "Direct Drive Standard;Direct Drive High Flow;Direct Drive Standard"},
        {"filament_self_index", "1,1,2"},
        {"close_fan_the_first_x_layers", "0,0"},
        {"close_additional_fan_first_x_layers", "3,1"},
        {"first_x_layer_fan_speed", "0,20"},
        {"additional_fan_full_speed_layer", "0,5"},
        {"additional_cooling_fan_speed", "60,90,80"},
    });
    config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {nozzle_volume_type};
    const auto gcode = slice_with_object_overrides({make_cube(10., 10., 1.2), make_cube(10., 10., 1.2)}, config,
        {{}, {{"outer_wall_filament_id", 2}, {"inner_wall_filament_id", 2}, {"sparse_infill_filament_id", 2},
              {"internal_solid_filament_id", 2}, {"top_surface_filament_id", 2}, {"bottom_surface_filament_id", 2}}});
    const auto samples = deposition_fans(gcode, config);
    const int first_target = nozzle_volume_type == nvtStandard ? 60 : 90;
    const std::vector<std::vector<int>> expected{{0, 0, 0, first_target, first_target, first_target}, {20, 35, 50, 65, 80, 80}};
    std::vector<std::vector<bool>> seen(2, std::vector<bool>(6, false));
    REQUIRE_FALSE(samples.empty());
    for (const auto &sample : samples) {
        REQUIRE(sample.tool < 2);
        REQUIRE(sample.layer < 6);
        INFO("nozzle volume " << int(nozzle_volume_type) << ", tool " << sample.tool << ", layer " << sample.layer);
        seen[sample.tool][sample.layer] = true;
        CHECK(sample.auxiliary == int(255.0 * expected[sample.tool][sample.layer] / 100.0));
        CHECK(sample.part == 51);
    }
    for (const auto &layers : seen)
        CHECK(std::all_of(layers.begin(), layers.end(), [](bool value) { return value; }));
}

// The fan is held off for the first close_fan_the_first_x_layers layers, so an explicit
// fan-off command is emitted.
TEST_CASE("Fan is held off for the initial layers", "[Cooling]")
{
    const std::string gcode = slice({ cube(20) }, {
        { "cooling",                      true },
        { "close_fan_the_first_x_layers", 5 },
    });
    CHECK(gcode.find("M106 S0") != std::string::npos);
}

// The cooling pass resolves and strips its internal speed placeholders; none leak into
// the final G-code.
TEST_CASE("Cooling consumes its internal speed markers", "[Cooling]")
{
    const std::string gcode = slice({ cube(20) }, { { "layer_height", 0.2 } });
    CHECK(gcode.find(";_EXTRUDE_SET_SPEED") == std::string::npos);
}

TEST_CASE("Arc fitting remains active with overhang fan control", "[Cooling][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "enable_arc_fitting",             true },
        { "enable_overhang_bridge_fan",     true },
        { "enable_overhang_speed",          true },
        { "initial_layer_print_height",     0.2 },
        { "layer_height",                   0.2 },
        { "slow_down_for_layer_cooling",    false },
    });
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{100});
    config.set_key_value("overhang_fan_threshold", new ConfigOptionEnumsGeneric{Overhang_threshold_2_4});
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER=[layer_num]"});

    const auto has_arc_after_first_layer = [](const std::string &gcode) {
        std::istringstream input(gcode);
        std::string line;
        size_t layers_seen = 0;
        while (std::getline(input, line)) {
            if (line.rfind(";TEST_LAYER=", 0) == 0)
                ++layers_seen;
            else if (layers_seen > 1 && (line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0))
                return true;
        }
        return false;
    };
    const auto has_arc_with_overhang_fan = [](const std::string &gcode) {
        std::istringstream input(gcode);
        std::string line;
        bool overhang_fan_active = false;
        bool outer_wall = false;
        while (std::getline(input, line)) {
            if (line.rfind("M106 S255", 0) == 0)
                overhang_fan_active = true;
            else if (line.rfind("M106 S0", 0) == 0 || line.rfind("M107", 0) == 0)
                overhang_fan_active = false;
            else if (line.rfind(";TYPE:", 0) == 0)
                outer_wall = line == ";TYPE:Outer wall";
            else if (overhang_fan_active && outer_wall && (line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0))
                return true;
        }
        return false;
    };

    const std::string with_overhang_speed = slice({ make_cylinder(25.0, 10.0, 2.0 * PI / 72.0) }, config);
    CHECK(has_arc_after_first_layer(with_overhang_speed));

    config.set_deserialize_strict({ { "enable_overhang_speed", false } });
    const std::string without_overhang_speed = slice({ make_cylinder(25.0, 10.0, 2.0 * PI / 72.0) }, config);
    CHECK(has_arc_after_first_layer(without_overhang_speed));

    const std::string uniform_overhang_gcode = slice({ make_sphere(50.0, 2.0 * PI / 72.0) }, config);
    CHECK(has_arc_with_overhang_fan(uniform_overhang_gcode));
}

TEST_CASE("Overhang fan transitions do not depend on overhang speed", "[Cooling][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "bridge_speed",                   2.0 },
        { "enable_arc_fitting",             false },
        { "enable_overhang_bridge_fan",     true },
        { "enable_overhang_speed",          false },
        { "initial_layer_print_height",     0.3 },
        { "inner_wall_speed",              30.0 },
        { "layer_height",                   0.3 },
        { "outer_wall_speed",              30.0 },
        { "overhang_1_4_speed",            "30" },
        { "overhang_2_4_speed",            "29" },
        { "overhang_3_4_speed",             "6" },
        { "overhang_4_4_speed",             "3" },
        { "slow_down_for_layer_cooling",    false },
        { "slowdown_for_curled_perimeters", false },
    });
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{100});
    config.set_key_value("overhang_fan_threshold", new ConfigOptionEnumsGeneric{Overhang_threshold_2_4});
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER_Z=[layer_z]"});

    const auto fan_commands = [](const std::string &gcode) {
        std::vector<std::pair<std::string, std::string>> commands;
        std::istringstream input(gcode);
        std::string layer;
        std::string line;
        while (std::getline(input, line)) {
            if (line.rfind(";TEST_LAYER_Z=", 0) == 0)
                layer = line;
            else if (!layer.empty() && (line.rfind("M106", 0) == 0 || line.rfind("M107", 0) == 0))
                commands.emplace_back(layer, line);
        }
        return commands;
    };
    const auto feedrates = [](const std::string &gcode) {
        std::vector<std::string> values;
        std::istringstream input(gcode);
        std::string word;
        while (input >> word)
            if (!word.empty() && word.front() == 'F')
                values.push_back(word);
        return values;
    };

    constexpr double sphere_radius = 50.0; // 100 mm diameter.
    const std::string without_speed_gcode = slice({make_sphere(sphere_radius, PI / 24.0)}, config);
    config.set_deserialize_strict({{"enable_overhang_speed", true}});
    const std::string with_speed_gcode = slice({make_sphere(sphere_radius, PI / 24.0)}, config);

    const auto without_speed_fan = fan_commands(without_speed_gcode);
    const auto with_speed_fan = fan_commands(with_speed_gcode);
    const auto without_speed_feedrates = feedrates(without_speed_gcode);
    const auto with_speed_feedrates = feedrates(with_speed_gcode);

    REQUIRE_FALSE(without_speed_fan.empty());
    REQUIRE(std::any_of(without_speed_fan.begin(), without_speed_fan.end(),
                        [](const auto &command) { return command.second.find("S255") != std::string::npos; }));
    REQUIRE(with_speed_feedrates != without_speed_feedrates);
    CHECK(with_speed_fan == without_speed_fan);
}
