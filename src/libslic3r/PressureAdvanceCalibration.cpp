#include "PressureAdvanceCalibration.hpp"

#include "Arrange.hpp"
#include "BoundingBox.hpp"
#include "CustomGCode.hpp"
#include "Point.hpp"
#include "TriangleMesh.hpp"
#include "libslic3r.h"

#include "Config.hpp"
#include "CutUtils.hpp"
#include "Flow.hpp"
#include "I18N.hpp"
#include "Model.hpp"
#include "ParameterUtils.hpp"
#include "PrintConfig.hpp"
#include "TowerCalibration.hpp"
#include "calib.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace Slic3r {

std::string pressure_advance_calibration_asset(CalibMode mode)
{
    switch (mode) {
    case CalibMode::Calib_PA_Line: return "pressure_advance/pressure_advance_test.drc";
    case CalibMode::Calib_PA_Tower: return "pressure_advance/tower_with_seam.drc";
    default: throw std::invalid_argument(_u8L("Unsupported pressure-advance calibration model"));
    }
}

void configure_pressure_advance_calibration(DynamicPrintConfig &print, DynamicPrintConfig &printer)
{
    print.set_key_value("overhang_reverse", new ConfigOptionBool(false));
    print.set_key_value("precise_z_height", new ConfigOptionBool(false));
    print.set_key_value("wipe_inward", new ConfigOptionBool(false));
    printer.set_key_value("resonance_avoidance", new ConfigOptionBool(false));
}

void set_pressure_advance_calibration_speed(DynamicPrintConfig &print_config, const DynamicPrintConfig &filament_config,
                                          const DynamicPrintConfig &printer_config, const DynamicPrintConfig &project,
                                          const std::string &key)
{
    const size_t extruder_nums = printer_config.option<ConfigOptionFloats>("nozzle_diameter")->size();
    const auto *extruder_types = printer_config.option<ConfigOptionEnumsGeneric>("extruder_type");
    const auto *nozzle_volume_types = project.option<ConfigOptionEnumsGeneric>("nozzle_volume_type");

    float nozzle_diameter = printer_config.option<ConfigOptionFloats>("nozzle_diameter")->values[0];
    float layer_height = print_config.option<ConfigOptionFloat>("layer_height")->value;
    float line_width = print_config.get_abs_value("line_width", nozzle_diameter);
    if (line_width <= 0.) line_width = Flow::auto_extrusion_width(frPerimeter, nozzle_diameter);

    Flow flow = Flow(line_width, layer_height, nozzle_diameter);

    for (size_t i = 0; i < extruder_nums; ++i) {
        int index = get_index_for_extruder_parameter(filament_config, "filament_max_volumetric_speed", i, ExtruderType(extruder_types->get_at(i)), NozzleVolumeType(nozzle_volume_types->get_at(i)));
        double filament_max_volumetric_speed = filament_config.option<ConfigOptionFloats>("filament_max_volumetric_speed")->get_at(index);
        double max_speed = filament_max_volumetric_speed / flow.mm3_per_mm();

        index = get_index_for_extruder_parameter(print_config, key, i, ExtruderType(extruder_types->get_at(i)), NozzleVolumeType(nozzle_volume_types->get_at(i)));
        ConfigOptionFloatsNullable *speed_opt = print_config.option<ConfigOptionFloatsNullable>(key);
        if (speed_opt->size() <= size_t(index))
            speed_opt->resize(size_t(index) + 1);
        speed_opt->values[index] = max_speed;
    }
}

Calib_Params apply_pressure_advance_tower_calibration(Model &model, DynamicPrintConfig &print,
                                                    DynamicPrintConfig &filament, DynamicPrintConfig &printer,
                                                    const DynamicPrintConfig &project, const Calib_Params &params,
                                                    const CalibrationCut &cut)
{
    auto &print_config = print;
    auto filament_config = &filament;
    print_config.set_key_value("enable_wrapping_detection", new ConfigOptionBool(false));
    filament_config->set_key_value("slow_down_layer_time", new ConfigOptionFloats{ 1.0f });


    auto& obj_cfg = model.objects[0]->config;

    obj_cfg.set_key_value("alternate_extra_wall", new ConfigOptionBool(false));
    set_pressure_advance_calibration_speed(print, filament, printer, project, "outer_wall_speed");
    set_pressure_advance_calibration_speed(print, filament, printer, project, "inner_wall_speed");
    obj_cfg.set_key_value("seam_position", new ConfigOptionEnum<SeamPosition>(spRear));
    obj_cfg.set_key_value("wall_loops", new ConfigOptionInt(2));
    obj_cfg.set_key_value("top_shell_layers", new ConfigOptionInt(0));
    obj_cfg.set_key_value("bottom_shell_layers", new ConfigOptionInt(0));
    obj_cfg.set_key_value("sparse_infill_density", new ConfigOptionPercent(0));
    obj_cfg.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btEar));
    obj_cfg.set_key_value("brim_object_gap", new ConfigOptionFloat(.0f));
    obj_cfg.set_key_value("brim_ears_max_angle", new ConfigOptionFloat(135.f));
    obj_cfg.set_key_value("brim_width", new ConfigOptionFloat(6.f));
    obj_cfg.set_key_value("seam_slope_type", new ConfigOptionEnum<SeamScarfType>(SeamScarfType::None));
    print_config.set_key_value("max_volumetric_extrusion_rate_slope", new ConfigOptionFloat(0));

    auto new_height = std::ceil((params.end - params.start) / params.step) + 1;
    auto obj_bb = model.objects[0]->bounding_box_exact();
    if (new_height < obj_bb.size().z()) {
        cut(new_height, ModelObjectCutAttribute::KeepLower);
    }

    return params;
}

PressureAdvancePatternPlan configure_pressure_advance_pattern(DynamicPrintConfig &print_config,
    DynamicPrintConfig &filament, DynamicPrintConfig &printer,
    const std::function<DynamicPrintConfig()> &effective_config, const Calib_Params &params)
{
    PressureAdvancePatternPlan plan;
    auto filament_config = &filament;
    auto printer_config = &printer;
    std::vector<double> speeds{params.speeds};
    std::vector<double> accels{params.accelerations};
    /* Set common parameters */
    double nozzle_diameter = printer_config->option<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
    set_config_values<bool, ConfigOptionBoolsNullable>(filament_config, "filament_retract_when_changing_layer", false);
    set_config_values<bool, ConfigOptionBoolsNullable>(filament_config, "filament_wipe", false);
    set_config_values<bool, ConfigOptionBools>(printer_config, "wipe", false);
    set_config_values<bool, ConfigOptionBools>(printer_config, "retract_when_changing_layer", false);
    printer_config->set_key_value("resonance_avoidance", new ConfigOptionBool(false));

    //Orca: find acceleration to use in the test
    auto accel = print_config.get_abs_value_at("outer_wall_acceleration", params.extruder_id); // get the outer wall acceleration
    if (accel == 0) // if outer wall accel isnt defined, fall back to inner wall accel
        accel = print_config.get_abs_value_at("inner_wall_acceleration", params.extruder_id);
    if (accel == 0) // if inner wall accel is not defined fall back to default accel
        accel = print_config.get_abs_value_at("default_acceleration", params.extruder_id);
    // Orca: Set all accelerations except first layer, as the first layer accel doesnt affect the PA test since accel
    // is set to the travel accel before printing the pattern.
    if (accels.empty()) {
        accels.assign({accel});
        plan.default_acceleration_used = true;
    } else {
        // set max acceleration in case of batch mode to get correct test pattern size
        accel = *std::max_element(accels.begin(), accels.end());
    }
    set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "outer_wall_acceleration", accel);
    print_config.set_key_value( "print_sequence", new ConfigOptionEnum(PrintSequence::ByLayer));

    //Orca: find jerk value to use in the test
    if(!has_junction_deviation(printer_config) && print_config.get_abs_value_at("default_jerk", params.extruder_id) > 0){ // we have set a jerk value
        auto jerk = print_config.get_abs_value_at("outer_wall_jerk", params.extruder_id); // get outer wall jerk
        if (jerk == 0) // if outer wall jerk is not defined, get inner wall jerk
            jerk = print_config.get_abs_value_at("inner_wall_jerk", params.extruder_id);
        if (jerk == 0) // if inner wall jerk is not defined, get the default jerk
            jerk = print_config.get_abs_value_at("default_jerk", params.extruder_id);

        //Orca: Set jerk values. Again first layer jerk should not matter as it is reset to the travel jerk before the
        // first PA pattern is printed.
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "default_jerk", jerk);
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "outer_wall_jerk", jerk);
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "inner_wall_jerk", jerk);
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "top_surface_jerk", jerk);
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "infill_jerk", jerk);
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "travel_jerk", jerk);
    }

    if (has_junction_deviation(printer_config)){
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "default_junction_deviation", 0);
    }

    for (const auto& opt : SuggestedConfigCalibPAPattern().floats_pairs) {
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, opt.first, opt.second[0]);
    }

    for (const auto& opt : SuggestedConfigCalibPAPattern().nozzle_ratio_pairs) {
        print_config.set_key_value(
            opt.first,
            new ConfigOptionFloatOrPercent(nozzle_diameter * opt.second / 100, false)
        );
    }

    for (const auto& opt : SuggestedConfigCalibPAPattern().int_pairs) {
        print_config.set_key_value(
            opt.first,
            new ConfigOptionInt(opt.second)
        );
    }

    print_config.set_key_value(SuggestedConfigCalibPAPattern().brim_pair.first,
        new ConfigOptionEnum<BrimType>(SuggestedConfigCalibPAPattern().brim_pair.second));

    print_config.set_key_value("enable_wrapping_detection", new ConfigOptionBool(false));

    // Orca: Set the outer wall speed to the optimal speed for the test, cap it with max volumetric speed
    if (speeds.empty()) {
        // TODO: per-variant cap
        // Resolve edited presets after applying the recipe, retaining the GUI's active variant mapping.
        double speed = CalibPressureAdvance::find_optimal_PA_speed(
            effective_config(),
            print_config.get_abs_value("line_width", nozzle_diameter),
            print_config.get_abs_value("layer_height"), 0, 0);
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "outer_wall_speed", speed);

        speeds.assign({speed});
        plan.default_speed_used = true;
    } else if (speeds.size() == 1) {
        // If we have single value provided, set speed using global configuration.
        // per-object config is not set in this case
        set_config_values<double, ConfigOptionFloatsNullable>(&print_config, "outer_wall_speed", speeds.front());
    }

    plan.speeds = std::move(speeds);
    plan.accelerations = std::move(accels);
    return plan;
}

Model make_pressure_advance_pattern_seed()
{
    Model model;
    auto *cube = model.add_object("Cube", "", make_cube(20., 20., 20.));
    cube->config.set_key_value("extruder", new ConfigOptionInt(1));
    cube->translate(-cube->raw_bounding_box().center());
    cube->add_instance();
    cube->ensure_on_bed();
    return model;
}

PressureAdvancePatternLayout build_pressure_advance_pattern(const PressureAdvancePatternPlan &plan,
    const DynamicPrintConfig &full_config, const Calib_Params &params, bool is_bbl_machine)
{
    if (plan.speeds.empty() || plan.accelerations.empty())
        throw std::invalid_argument(_u8L("PA pattern requires resolved speeds and accelerations"));
    PressureAdvancePatternLayout layout;
    layout.model = make_pressure_advance_pattern_seed();
    auto *cube = layout.model.objects.front();
    const auto &speeds = plan.speeds;
    const auto &accels = plan.accelerations;
    for (const auto *values : {&speeds, &accels})
        if (std::any_of(values->begin(), values->end(), [](double value) { return !std::isfinite(value) || value <= 0.; }))
            throw std::invalid_argument(_u8L("PA pattern requires positive finite speeds and accelerations"));
    const BoundingBoxf bed(get_extents(full_config.option<ConfigOptionPoints>("printable_area")->values));
    cube->instances.front()->set_offset(Vec3d(bed.center().x(), bed.center().y(), cube->instances.front()->get_offset().z()));
    CalibPressureAdvancePattern pa_pattern(params, full_config, is_bbl_machine, *cube, Vec3d::Zero());
    /* Having PA pattern configured, we could make a set of polygons resembling N test patterns.
     * We'll arrange this set of polygons, so we would know position of each test pattern and
     * could position test cubes later on
     *
     * We'll take advantage of already existing cube: scale it up to test pattern size to use
     * as a reference for objects arrangement. Polygon is slightly oversized to add spaces between patterns.
     * That arrangement will be used to place 'handle cubes' for each test. */
    auto cube_bb = cube->raw_bounding_box();
    cube->scale((pa_pattern.print_size_x() + 4) / cube_bb.size().x(),
                (pa_pattern.print_size_y() + 4) / cube_bb.size().y(),
                pa_pattern.max_layer_z() / cube_bb.size().z());

    arrangement::ArrangePolygons arranged_items;
    {
        arrangement::ArrangeParams ap;
        Points bedpts = arrangement::get_shrink_bedpts(&full_config, ap);

        for(size_t i = 0; i < speeds.size() * accels.size(); i++) {
            arrangement::ArrangePolygon p;
            cube->instances[0]->get_arrange_polygon(&p);
            p.bed_idx = 0;
            arranged_items.emplace_back(p);
        }

        arrangement::arrange(arranged_items, bedpts, ap);
    }

    /* scale cube back to the size of test pattern 'handle' */
    cube_bb = cube->raw_bounding_box();
    cube->scale(pa_pattern.handle_xy_size() / cube_bb.size().x(),
                pa_pattern.handle_xy_size() / cube_bb.size().y(),
                pa_pattern.max_layer_z() / cube_bb.size().z());

    /* Set speed and acceleration on per-object basis and arrange anchor object on the plates.
     * Test gcode will be genecated during plate slicing */
    for(size_t test_idx = 0; test_idx < arranged_items.size(); test_idx++) {
        const auto &ai = arranged_items[test_idx];
        size_t plate_idx = arranged_items[test_idx].bed_idx;
        auto tspd = speeds[test_idx % speeds.size()];
        auto tacc = accels[test_idx / speeds.size()];

        /* make an own copy of anchor cube for each test */
        auto obj = test_idx == 0 ? cube : layout.model.add_object(*cube);

        obj->name.assign(std::string("pa_pattern_") + std::to_string(int(tspd)) + std::string("_") + std::to_string(int(tacc)));

        auto &obj_config = obj->config;
        if (speeds.size() > 1)
            obj_config.set_key_value("outer_wall_speed", new ConfigOptionFloatsNullable(1, tspd));
        if (accels.size() > 1)
            obj_config.set_key_value("outer_wall_acceleration", new ConfigOptionFloatsNullable(1, tacc));

        if (arranged_items[test_idx].bed_idx < 0)
            throw std::invalid_argument(_u8L("Pressure-advance pattern does not fit the printable bed"));
        layout.object_plates.push_back(plate_idx);
        const Vec3d obj_offset{unscale<double>(ai.translation(X)),
                               unscale<double>(ai.translation(Y)),
                               0};
        obj->instances[0]->set_offset(obj_offset + pa_pattern.handle_pos_offset());
        obj->ensure_on_bed();

    }

    layout.model.calib_pa_pattern = std::make_unique<CalibPressureAdvancePattern>(pa_pattern);
    return layout;
}

CustomGCode::Info generate_pressure_advance_pattern_gcode(CalibPressureAdvancePattern &pattern,
    const ModelObjectPtrs &objects, const DynamicPrintConfig &config, bool is_bbl_machine, const Vec3d &origin)
{
    CustomGCode::Info result;
    bool first = true;
    for (const auto *object : objects) {
        auto info = pattern.generate_custom_gcodes(config, is_bbl_machine, *object, origin);
        if (first) {
            result = std::move(info);
            first = false;
        } else {
            if (result.gcodes.size() != info.gcodes.size())
                throw std::invalid_argument(_u8L("Pressure-advance patterns must have equal layer counts on a plate"));
            for (size_t i = 0; i < result.gcodes.size(); ++i)
                result.gcodes[i].extra += info.gcodes[i].extra;
        }
    }
    return result;
}

} // namespace Slic3r
