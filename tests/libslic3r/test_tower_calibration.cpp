#include "libslic3r/CalibrationRecipe.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/CutUtils.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TowerCalibration.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/calib.hpp"
#include "libslic3r/libslic3r.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
DynamicPrintConfig tower_profile()
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"nozzle_diameter", "0.4"}, {"layer_height", "0.2"},
        {"initial_layer_print_height", "0.2"}, {"filament_flow_ratio", "0.95"},
        {"filament_max_volumetric_speed", "12"}, {"max_layer_height", "0.28"},
        {"printable_area", "0x0,256x0,256x256,0x256"}, {"belt_printer", "0"}});
    return config;
}

Model tower_model()
{
    Model model;
    model.add_object("tower", "", make_cube(40, 30, 40))->add_instance();
    return model;
}
}

TEST_CASE("Retraction calibration cuts its ramp and chooses nozzle dependent layers", "[TowerCalibration]")
{
    const double nozzle = GENERATE(0.1, 0.2, 0.4);
    auto config = tower_profile();
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{nozzle});
    config.set_key_value("use_firmware_retraction", new ConfigOptionBool(true));
    auto model = tower_model();
    Calib_Params request;
    request.start = 0.2;
    request.end = 1.2;
    request.step = 0.2;
    const auto runtime = apply_calibration_recipe(model, config, *find_calibration_recipe("retraction"), "archimedeanchords", request);
    const double height = nozzle <= 0.1 ? 0.05 : nozzle <= 0.2 ? 0.1 : 0.2;
    REQUIRE(model.objects.size() == 1);
    const auto &object = *model.objects.front();
    CHECK_THAT(object.bounding_box_exact().size().z(), WithinAbs(1.4 + (request.end - request.start) / request.step - EPSILON, 1e-5));
    CHECK_THAT(object.config.opt_float("layer_height"), WithinAbs(height, 1e-6));
    CHECK_THAT(config.opt_float("initial_layer_print_height"), WithinAbs(height, 1e-6));
    CHECK_FALSE(config.opt_bool("use_firmware_retraction"));
    CHECK_FALSE(config.opt_bool("wipe_inward"));
    CHECK(object.config.opt_int("wall_loops") == 2);
    CHECK(object.config.opt_int("top_shell_layers") == 0);
    CHECK(runtime.mode == CalibMode::Calib_Retraction_tower);
    CHECK_THAT(runtime.start, WithinAbs(request.start, 1e-8));
    CHECK_THAT(runtime.step, WithinAbs(request.step, 1e-8));
}

TEST_CASE("Volumetric calibration converts the requested ramp using its actual thick bead", "[TowerCalibration]")
{
    auto config = tower_profile();
    // The tower fits inside the usable bed width by scaling only X.
    config.set_key_value("printable_area", new ConfigOptionPoints{{Vec2d(0, 0), Vec2d(35, 0), Vec2d(35, 50), Vec2d(0, 50)}});
    auto model = tower_model();
    Calib_Params request;
    request.start = 4.;
    request.end = 8.;
    request.step = 1.;
    const auto runtime = apply_calibration_recipe(model, config, *find_calibration_recipe("max-volumetric-speed"), "archimedeanchords", request);
    REQUIRE(model.objects.size() == 1);
    const auto &object = *model.objects.front();
    CHECK_THAT(object.bounding_box_exact().size().x(), WithinAbs(25., 1e-5));
    CHECK_THAT(object.bounding_box_exact().size().z(), WithinAbs(5., 1e-5));
    CHECK_THAT(object.config.opt_float("layer_height"), WithinAbs(0.32, 1e-6));
    CHECK_THAT(object.config.get().option<ConfigOptionFloatOrPercent>("outer_wall_line_width")->value, WithinAbs(0.7, 1e-6));
    CHECK_THAT(config.option<ConfigOptionFloats>("max_layer_height")->get_at(0), WithinAbs(0.32, 1e-6));
    CHECK(config.opt_bool("spiral_mode"));
    CHECK_THAT(config.option<ConfigOptionFloats>("filament_max_volumetric_speed")->get_at(0), WithinAbs(200., 1e-6));
    const double volume_per_mm = Flow(0.7, 0.32, 0.4).mm3_per_mm() * 0.95;
    CHECK(runtime.mode == CalibMode::Calib_Vol_speed_Tower);
    CHECK_THAT(runtime.start * volume_per_mm, WithinAbs(request.start, 1e-8));
    CHECK_THAT(runtime.step * volume_per_mm, WithinAbs(request.step, 1e-8));
}

TEST_CASE("VFA calibration preserves the layer count of each speed block when resizing", "[TowerCalibration]")
{
    const bool resize = GENERATE(false, true);
    auto config = tower_profile();
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.6});
    auto model = tower_model();
    Calib_Params request;
    request.start = 40.;
    request.end = 100.;
    request.step = 30.;
    request.vfa_layer_height = 0.25;
    request.nozzle_based_resize = resize;
    const auto runtime = apply_calibration_recipe(model, config, *find_calibration_recipe("vfa"), "archimedeanchords", request);
    REQUIRE(model.objects.size() == 1);
    const auto &object = *model.objects.front();
    CHECK_THAT(object.bounding_box_exact().size().x(), WithinAbs(resize ? 60. : 40., 1e-5));
    CHECK_THAT(object.bounding_box_exact().size().z(), WithinAbs((15. - EPSILON) * (resize ? 1.25 : 1.), 1e-5));
    CHECK_THAT(config.opt_float("initial_layer_print_height"), WithinAbs(resize ? 0.25 : 0.2, 1e-6));
    CHECK(config.opt_bool("spiral_mode"));
    CHECK_THAT(runtime.vfa_layer_height, WithinAbs(resize ? 0.25 : 0., 1e-8));
    CHECK(runtime.mode == CalibMode::Calib_VFA_Tower);
    if (resize)
        CHECK_THAT(object.config.opt_float("layer_height") * vfa_layers_per_block, WithinAbs(6.25, 1e-6));
    else
        CHECK_FALSE(object.config.has("layer_height"));
}

TEST_CASE("The CLI tower dispatcher and GUI shared routine produce identical retraction geometry and settings", "[TowerCalibration]")
{
    auto cli_config = tower_profile();
    auto print = cli_config;
    auto filament = cli_config;
    auto printer = cli_config;
    auto cli_model = tower_model();
    auto gui_model = tower_model();
    Calib_Params request;
    request.mode = CalibMode::Calib_Retraction_tower;
    request.start = 0.;
    request.end = 1.;
    request.step = 0.25;
    const auto gui_runtime = apply_retraction_calibration(gui_model, print, filament, printer, request,
        [&gui_model](double z, ModelObjectCutAttributes attributes) { cut_calibration_model(gui_model, z, attributes); });
    const auto cli_runtime = apply_calibration_recipe(cli_model, cli_config, *find_calibration_recipe("retraction"), "archimedeanchords", request);
    CHECK(gui_runtime.mode == cli_runtime.mode);
    CHECK_THAT(gui_runtime.step, WithinAbs(cli_runtime.step, 1e-8));
    CHECK_THAT(gui_model.objects.front()->bounding_box_exact().size().z(), WithinAbs(cli_model.objects.front()->bounding_box_exact().size().z(), 1e-8));
    CHECK(gui_model.objects.front()->config.get() == cli_model.objects.front()->config.get());
    for (const auto *key : {"initial_layer_print_height", "enable_wrapping_detection", "wipe_inward"})
        CHECK(print.opt_serialize(key) == cli_config.opt_serialize(key));
    for (const auto *key : {"resonance_avoidance", "use_firmware_retraction", "max_layer_height"})
        CHECK(printer.opt_serialize(key) == cli_config.opt_serialize(key));
}

TEST_CASE("Tower calibration rejects invalid ranges before mutating the model", "[TowerCalibration]")
{
    const std::string recipe = GENERATE("retraction", "max-volumetric-speed", "vfa");
    const double step = GENERATE(0., -1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN());
    auto config = tower_profile();
    auto model = tower_model();
    Calib_Params request;
    request.step = step;
    CHECK_THROWS_AS(apply_calibration_recipe(model, config, *find_calibration_recipe(recipe), "archimedeanchords", request), std::invalid_argument);
    CHECK_THAT(model.objects.front()->bounding_box_exact().size().z(), WithinAbs(40., 1e-8));
    CHECK(model.objects.front()->config.empty());
    CHECK(config == tower_profile());
}

TEST_CASE("Temperature calibration cuts the labelled blocks and initializes the selected temperature", "[TowerCalibration]")
{
    auto config = tower_profile();
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.6});
    Model model;
    model.add_object("temperature", "", make_cube(40, 30, 1000))->add_instance();
    Calib_Params request;
    request.start = 260.;
    request.end = 250.;
    request.step = 5.;
    const auto runtime = apply_calibration_recipe(model, config, *find_calibration_recipe("temperature"), "archimedeanchords", request);
    REQUIRE(model.objects.size() == 1);
    const auto &object = *model.objects.front();
    CHECK_THAT(object.bounding_box_exact().size().z(), WithinAbs((30. - 2 * EPSILON) * 1.5, 1e-4));
    CHECK_THAT(object.bounding_box_exact().min.z(), WithinAbs(0., 1e-5));
    CHECK_THAT(object.config.opt_float("layer_height"), WithinAbs(0.3, 1e-6));
    CHECK_FALSE(object.config.get().opt_bool("enable_support"));
    CHECK(config.option<ConfigOptionInts>("nozzle_temperature")->get_at(0) == 260);
    CHECK(config.option<ConfigOptionInts>("nozzle_temperature_initial_layer")->get_at(0) == 260);
    CHECK(runtime.mode == CalibMode::Calib_Temp_Tower);
    CHECK_THAT(runtime.end, WithinAbs(250., 1e-6));
}

TEST_CASE("Fan calibration preserves each fan mode and enables its required overhang classification", "[TowerCalibration]")
{
    const std::string recipe = GENERATE("overhang-fan", "part-fan", "bridge-fan", "small-feature-fan");
    auto config = tower_profile();
    config.set_deserialize_strict({{"enable_overhang_bridge_fan", "0"}, {"overhang_fan_threshold", "95%"}});
    Model model;
    model.add_object("fan", "", make_cube(40, 30, 110))->add_instance();
    Calib_Params request;
    request.start = 20.;
    request.end = 40.;
    request.step = 10.;
    const auto *entry = find_calibration_recipe(recipe);
    REQUIRE(entry != nullptr);
    const auto runtime = apply_calibration_recipe(model, config, *entry, "archimedeanchords", request);
    REQUIRE(model.objects.size() == 1);
    const auto &object = *model.objects.front();
    CHECK_THAT(object.bounding_box_exact().size().z(), WithinAbs(30. - 2 * EPSILON, 1e-5));
    CHECK_THAT(object.bounding_box_exact().min.z(), WithinAbs(0., 1e-5));
    CHECK_FALSE(object.config.get().opt_bool("enable_support"));
    CHECK(runtime.mode == entry->mode);
    CHECK(config.option<ConfigOptionBools>("enable_overhang_bridge_fan")->get_at(0) == fan_tower_steps_overhang_fan(runtime.mode));
    CHECK(config.option<ConfigOptionEnumsGeneric>("overhang_fan_threshold")->get_at(0) ==
        (recipe == "overhang-fan" ? Overhang_threshold_2_4 : Overhang_threshold_bridge));
}
