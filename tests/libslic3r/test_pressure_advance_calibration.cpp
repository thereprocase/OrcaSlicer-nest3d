#include "libslic3r/CalibrationRecipe.hpp"
#include "libslic3r/Config.hpp"
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
    const auto runtime = apply_calibration_recipe(model, config, *find_calibration_recipe("pressure-advance-line"), "archimedeanchords", request);
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
    const auto runtime = apply_calibration_recipe(model, config, *find_calibration_recipe("pressure-advance-tower"), "archimedeanchords", request);
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
    Calib_Params request;
    request.start = request.end = 0.;
    CHECK_THROWS_AS(apply_calibration_recipe(model, config, *find_calibration_recipe(recipe), "archimedeanchords", request), std::invalid_argument);
    CHECK(config == before);
    CHECK(model.objects.front()->config.empty());
}
