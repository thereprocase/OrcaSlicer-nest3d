#include "libslic3r/CalibrationRecipe.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/PressureAdvanceCalibration.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/calib.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <utility>
#include <stdexcept>
#include <string>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
DynamicPrintConfig pa_profile()
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"nozzle_diameter", "0.4"}, {"line_width", "0.42"},
        {"layer_height", "0.2"}, {"filament_flow_ratio", "0.95"}, {"filament_max_volumetric_speed", "12"},
        {"outer_wall_speed", "40"}, {"inner_wall_speed", "80"}, {"pressure_advance", "0.064"},
        {"wipe_inward", "1"}, {"overhang_reverse", "1"}, {"precise_z_height", "1"},
        {"resonance_avoidance", "1"}, {"belt_printer", "0"}});
    return config;
}

Model pa_model()
{
    Model model;
    model.add_object("PA", "", make_cube(40, 30, 40))->add_instance();
    return model;
}
}

TEST_CASE("PA line calibration preserves the requested generator range and number selection", "[PressureAdvanceCalibration]")
{
    const bool numbers = GENERATE(false, true);
    auto config = pa_profile();
    auto model = pa_model();
    Calib_Params request;
    request.start = 0.;
    request.end = 0.03125;
    request.step = 0.015625;
    request.print_numbers = numbers;
    const auto runtime = apply_calibration_recipe(model, config, *find_calibration_recipe("pressure-advance-line"), "archimedeanchords", request).params;
    CHECK(runtime.mode == CalibMode::Calib_PA_Line);
    CHECK(runtime.print_numbers == numbers);
    CHECK_THAT(runtime.step, WithinAbs(request.step, 1e-8));
    CHECK_THAT(model.objects.front()->bounding_box_exact().size().z(), WithinAbs(40., 1e-8));
    CHECK_FALSE(config.opt_bool("wipe_inward"));
    CHECK_FALSE(config.opt_bool("overhang_reverse"));
    CHECK_FALSE(config.opt_bool("precise_z_height"));
    CHECK_FALSE(config.opt_bool("resonance_avoidance"));
    CHECK_THAT(config.option<ConfigOptionFloats>("pressure_advance")->get_at(0), WithinAbs(0.064, 1e-8));
}

TEST_CASE("PA tower calibration cuts its bands and uses the GUI volumetric wall speed", "[PressureAdvanceCalibration]")
{
    auto config = pa_profile();
    auto model = pa_model();
    Calib_Params request;
    request.start = 0.;
    request.end = 0.03125;
    request.step = 0.015625;
    const auto runtime = apply_calibration_recipe(model, config, *find_calibration_recipe("pressure-advance-tower"), "archimedeanchords", request).params;
    REQUIRE(model.objects.size() == 1);
    CHECK_THAT(model.objects.front()->bounding_box_exact().size().z(), WithinAbs(3., 1e-5));
    const auto &object = model.objects.front()->config.get();
    CHECK(object.opt_int("wall_loops") == 2);
    CHECK(object.opt_int("top_shell_layers") == 0);
    CHECK(object.opt_int("bottom_shell_layers") == 0);
    CHECK(object.opt_enum<BrimType>("brim_type") == btEar);
    const double speed = 12. / Flow(0.42, 0.2, 0.4).mm3_per_mm();
    CHECK_THAT(config.option<ConfigOptionFloats>("outer_wall_speed")->get_at(0), WithinAbs(speed, 1e-4));
    CHECK_THAT(config.option<ConfigOptionFloats>("inner_wall_speed")->get_at(0), WithinAbs(speed, 1e-4));
    CHECK_THAT(config.option<ConfigOptionFloats>("slow_down_layer_time")->get_at(0), WithinAbs(1., 1e-8));
    CHECK_THAT(config.option<ConfigOptionFloats>("filament_flow_ratio")->get_at(0), WithinAbs(0.95, 1e-8));
    CHECK(runtime.mode == CalibMode::Calib_PA_Tower);
}

TEST_CASE("PA calibration rejects an empty range before changing the model or profile", "[PressureAdvanceCalibration]")
{
    const std::string recipe = GENERATE("pressure-advance-line", "pressure-advance-tower");
    auto config = pa_profile();
    const auto before = config;
    auto model = pa_model();
    const auto before_object = model.objects.front()->config.get();
    Calib_Params request;
    request.start = request.end = 0.;
    CHECK_THROWS_AS(apply_calibration_recipe(model, config, *find_calibration_recipe(recipe), "archimedeanchords", request), std::invalid_argument);
    CHECK(config == before);
    CHECK(model.objects.front()->config.get() == before_object);
}

TEST_CASE("PA pattern batches retain every requested speed and acceleration across native plates", "[PressureAdvanceCalibration]")
{
    auto config = pa_profile();
    config.set_deserialize_strict({{"printable_area", "0x0,80x0,80x80,0x80"}, {"initial_layer_print_height", "0.2"}});
    Model model;
    Calib_Params request;
    request.start = 0.;
    request.end = 0.03125;
    request.step = 0.015625;
    request.speeds = {30., 60.};
    request.accelerations = {1000., 2000.};
    const auto result = apply_calibration_recipe(model, config, *find_calibration_recipe("pressure-advance-pattern"), "archimedeanchords", request);
    REQUIRE(model.objects.size() == request.speeds.size() * request.accelerations.size());
    REQUIRE(result.object_plates.size() == model.objects.size());
    CHECK(*std::max_element(result.object_plates.begin(), result.object_plates.end()) >= 1);
    CHECK(result.params.mode == CalibMode::Calib_PA_Pattern);
    std::set<std::pair<double, double>> combinations;
    for (const auto *object : model.objects) {
        CHECK_THAT(object->bounding_box_exact().size().x(), WithinAbs(5., 1e-5));
        CHECK_THAT(object->bounding_box_exact().size().y(), WithinAbs(5., 1e-5));
        CHECK_THAT(object->bounding_box_exact().size().z(), WithinAbs(0.8, 1e-5));
        combinations.emplace(object->config.get().option<ConfigOptionFloats>("outer_wall_speed")->get_at(0),
                             object->config.get().option<ConfigOptionFloats>("outer_wall_acceleration")->get_at(0));
    }
    for (double speed : request.speeds)
        for (double acceleration : request.accelerations)
            CHECK(combinations.count({speed, acceleration}) == 1);
    CHECK_FALSE(config.option<ConfigOptionBools>("retract_when_changing_layer")->get_at(0));
    CHECK_FALSE(config.option<ConfigOptionBools>("wipe")->get_at(0));
}

TEST_CASE("PA pattern copies own their requests and merge custom G-code for the selected objects", "[PressureAdvanceCalibration]")
{
    auto config = pa_profile();
    config.set_deserialize_strict({{"printable_area", "0x0,256x0,256x256,0x256"}, {"initial_layer_print_height", "0.2"}});
    Model model;
    Model copied;
    {
        Calib_Params request;
        request.start = 0.;
        request.end = 0.03125;
        request.step = 0.015625;
        request.speeds = {30., 60.};
        request.accelerations = {1000.};
        apply_calibration_recipe(model, config, *find_calibration_recipe("pressure-advance-pattern"), "archimedeanchords", request);
        request.start = 0.2;
        request.end = 0.4;
        copied = model;
    }
    model.calib_pa_pattern.reset();
    model.clear_objects();
    REQUIRE(copied.calib_pa_pattern != nullptr);
    REQUIRE(copied.objects.size() == 2);
    auto info = generate_pressure_advance_pattern_gcode(*copied.calib_pa_pattern, copied.objects, config, false, Vec3d::Zero());
    REQUIRE(info.gcodes.size() == 4);
    for (const auto &item : info.gcodes) {
        CHECK(item.extra.find("M900 K0.03125") != std::string::npos);
        CHECK(item.extra.find("M900 K0.2") == std::string::npos);
        const auto first = item.extra.find("; start pressure advance pattern for layer");
        REQUIRE(first != std::string::npos);
        CHECK(item.extra.find("; start pressure advance pattern for layer", first + 1) != std::string::npos);
    }
}

TEST_CASE("PA pattern resolves the same default acceleration fallback as the GUI", "[PressureAdvanceCalibration]")
{
    auto config = pa_profile();
    config.set_deserialize_strict({{"outer_wall_acceleration", "0"}, {"inner_wall_acceleration", "1200"},
                                  {"default_acceleration", "4000"}});
    Calib_Params request;
    const auto plan = configure_pressure_advance_pattern(config, config, config, [&config] { return config; }, request);
    REQUIRE(plan.accelerations.size() == 1);
    REQUIRE(plan.speeds.size() == 1);
    CHECK(plan.default_acceleration_used);
    CHECK(plan.default_speed_used);
    CHECK_THAT(plan.accelerations.front(), WithinAbs(1200., 1e-8));
    CHECK(plan.speeds.front() > 0.);
    CHECK(config.opt_enum<PrintSequence>("print_sequence") == PrintSequence::ByLayer);
}

TEST_CASE("Calibration numeric lists reject malformed values before generation", "[PressureAdvanceCalibration]")
{
    const std::string malformed = GENERATE("30,broken", "30,60,", "30,,60", "nan,60", "30 60");
    CHECK_THROWS_AS(parse_calibration_values(malformed), std::invalid_argument);
}

TEST_CASE("PA pattern default speed uses the resolved active material after recipe edits", "[PressureAdvanceCalibration]")
{
    auto config = pa_profile();
    config.set_deserialize_strict({{"outer_wall_acceleration", "0"}, {"inner_wall_acceleration", "1200"}});
    double observed_acceleration = -1.;
    Calib_Params request;
    const auto plan = configure_pressure_advance_pattern(config, config, config, [&] {
        observed_acceleration = config.get_abs_value_at("outer_wall_acceleration", 0);
        DynamicPrintConfig active = config;
        active.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloats{3.});
        return active;
    }, request);
    CHECK_THAT(observed_acceleration, WithinAbs(1200., 1e-8));
    REQUIRE(plan.speeds.size() == 1);
    const double bead = Flow(0.45, 0.2, 0.4).mm3_per_mm();
    CHECK(plan.speeds.front() * bead <= 3.);
    CHECK((plan.speeds.front() + 1.) * bead > 3.);
}
