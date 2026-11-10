#include "libslic3r/FlowRatioCalibration.hpp"
#include "libslic3r/CalibrationRecipe.hpp"
#include <set>
#include "libslic3r/Flow.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
DynamicPrintConfig calibration_profile()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"nozzle_diameter", "0.4"}, {"line_width", "0.42"},
        {"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"},
        {"filament_flow_ratio", "0.95"}, {"filament_max_volumetric_speed", "6"},
        {"top_surface_speed", "200"}, {"internal_solid_infill_speed", "200"},
        {"pressure_advance", "0.064"}, {"belt_printer", "0"}});
    return config;
}
}

TEST_CASE("Flow calibration replaces a profile's surface strategy with the GUI recipe", "[FlowRatioCalibration]")
{
    DynamicPrintConfig config = calibration_profile();
    config.set_deserialize_strict({{"top_surface_pattern", "monotonic"}, {"top_surface_fill_order", "inward"},
        {"wall_loops", "4"}, {"top_shell_layers", "2"}, {"sparse_infill_density", "80%"}});
    const auto plan = plan_flow_ratio_calibration(config, false, 1);
    Model model;
    model.add_object("flowrate_m20", "", make_cube(40, 30, 2))->add_instance();
    model.add_object("flowrate_20", "", make_cube(40, 30, 2))->add_instance();
    apply_flow_ratio_calibration(model.objects, config, plan, false, 1);

    const auto &low = model.objects.front()->config.get();
    CHECK(low.opt_enum<InfillPattern>("top_surface_pattern") == ipArchimedeanChords);
    CHECK(low.opt_enum<SurfaceFillOrder>("top_surface_fill_order") == SurfaceFillOrder::Default);
    CHECK(low.opt_bool("calib_flowrate_topinfill_special_order"));
    CHECK(low.opt_int("bottom_shell_layers") == 2);
    CHECK(low.opt_int("top_shell_layers") == 5);
    CHECK(low.opt_int("wall_loops") == 1);
    CHECK_THAT(low.opt_float("print_flow_ratio"), WithinAbs(0.8, 1e-6));
    CHECK_THAT(model.objects.back()->config.opt_float("print_flow_ratio"), WithinAbs(1.2, 1e-6));
    CHECK_THAT(low.option<ConfigOptionFloatOrPercent>("top_surface_line_width")->value, WithinAbs(0.48, 1e-6));
    CHECK_THAT(config.option<ConfigOptionFloats>("pressure_advance")->get_at(0), WithinAbs(0.064, 1e-6));
    CHECK_THAT(config.option<ConfigOptionFloats>("filament_flow_ratio")->get_at(0), WithinAbs(0.95, 1e-6));
}

TEST_CASE("Flow calibration shares a speed ceiling that accommodates its largest flow modifier", "[FlowRatioCalibration]")
{
    const auto config = calibration_profile();
    const auto coarse = plan_flow_ratio_calibration(config, false, 1);
    const auto fine = plan_flow_ratio_calibration(config, false, 2);
    const Flow flow(0.48, 0.2, 0.4);
    CHECK(coarse.top_surface_speeds.front() * flow.mm3_per_mm() * 1.2 <= 6.);
    CHECK(coarse.top_surface_speeds.front() == 57.);
    CHECK(coarse.internal_solid_speeds.front() == 57.);
    CHECK(fine.top_surface_speeds.front() > coarse.top_surface_speeds.front());
    CHECK_THAT(coarse.layer_height, WithinAbs(0.2, 1e-6));
    CHECK_THAT(coarse.scale.z(), WithinAbs(1., 1e-6));
}

TEST_CASE("YOLO flow calibration adds its modifier to the filament ratio", "[FlowRatioCalibration]")
{
    auto config = calibration_profile();
    const auto plan = plan_flow_ratio_calibration(config, true, 1);
    Model model;
    model.add_object("flowrate_m0.05", "", make_cube(40, 30, 2))->add_instance();
    apply_flow_ratio_calibration(model.objects, config, plan, true, 1);
    CHECK_THAT(model.objects.front()->config.opt_float("print_flow_ratio") * plan.flow_ratio, WithinAbs(0.90, 1e-6));
    CHECK(flow_ratio_calibration_asset(true, 2) == "filament_flow/Orca-LinearFlow_fine.3mf");
}

TEST_CASE("Flow calibration rejects invalid names and ranges before changing the objects", "[FlowRatioCalibration]")
{
    auto config = calibration_profile();
    const auto plan = plan_flow_ratio_calibration(config, false, 1);
    Model model;
    model.add_object("flowrate_0", "", make_cube(40, 30, 2))->add_instance();
    model.add_object("flowrate_invalid", "", make_cube(40, 30, 2))->add_instance();
    CHECK_THROWS_AS(apply_flow_ratio_calibration(model.objects, config, plan, false, 1), std::invalid_argument);
    CHECK_FALSE(model.objects.front()->config.has("print_flow_ratio"));
    CHECK_THROWS_AS(flow_ratio_calibration_modifier("flowrate_5extra"), std::invalid_argument);
    CHECK_THROWS_AS(plan_flow_ratio_calibration(config, false, 3), std::invalid_argument);
    config.set_deserialize_strict({{"filament_flow_ratio", "0"}});
    CHECK_THROWS_AS(plan_flow_ratio_calibration(config, false, 1), std::invalid_argument);
}

TEST_CASE("The calibration catalogue reports GUI-only recipes without substituting flow tests", "[FlowRatioCalibration]")
{
    std::set<std::string> names;
    size_t supported = 0;
    for (const auto &recipe : calibration_recipes()) {
        CHECK(names.insert(recipe.name).second);
        if (recipe.cli_supported) {
            ++supported;
            CHECK(recipe.generator == CalibrationGenerator::FlowRatio);
        }
    }
    CHECK(supported == 4);
    REQUIRE(find_calibration_recipe("temperature") != nullptr);
    CHECK_FALSE(find_calibration_recipe("temperature")->cli_supported);
    CHECK(find_calibration_recipe("not-a-calibration") == nullptr);
}

TEST_CASE("The shared dispatcher accepts both patterns offered by the flow calibration dialog", "[FlowRatioCalibration]")
{
    const auto *recipe = find_calibration_recipe("flow-ratio");
    REQUIRE(recipe != nullptr);
    for (const std::string pattern : {"archimedeanchords", "monotonic"}) {
        auto config = calibration_profile();
        Model model;
        model.add_object("flowrate_0", "", make_cube(40, 30, 2))->add_instance();
        apply_calibration_recipe(model, config, *recipe, pattern);
        CHECK(model.objects.front()->config.opt_serialize("top_surface_pattern") == pattern);
        CHECK_FALSE(config.opt_bool("resonance_avoidance"));
    }
    DynamicPrintConfig missing;
    CHECK_THROWS_AS(plan_flow_ratio_calibration(missing, false, 1), std::invalid_argument);
}
