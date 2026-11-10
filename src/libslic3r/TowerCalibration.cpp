#include "TowerCalibration.hpp"
#include "calib.hpp"
#include "Config.hpp"
#include "CutUtils.hpp"

#include "BoundingBox.hpp"
#include "Flow.hpp"
#include "Geometry.hpp"
#include "Model.hpp"
#include "ParameterUtils.hpp"
#include "Point.hpp"
#include "PrintConfig.hpp"
#include "libslic3r.h"

#include <boost/log/trivial.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace Slic3r {
namespace {
// Orca: a calibration print sets pressure advance explicitly to 0 on each extruder variant that
// has it disabled, so the value stored in the printer does not skew the result. Variants with
// pressure advance enabled keep their own value.
static void zero_pressure_advance_where_disabled(DynamicPrintConfig *filament_config)
{
    auto enable_pa   = filament_config->option<ConfigOptionBools>("enable_pressure_advance");
    auto pa          = filament_config->option<ConfigOptionFloats>("pressure_advance");
    auto adaptive_pa = filament_config->option<ConfigOptionBools>("adaptive_pressure_advance");
    if (!enable_pa || !pa || !adaptive_pa || pa->empty() || adaptive_pa->empty())
        return;
    // All variant keys share the variant count; widen a short one rather than index past it.
    if (pa->size() < enable_pa->size())
        pa->resize(enable_pa->size());
    if (adaptive_pa->size() < enable_pa->size())
        adaptive_pa->resize(enable_pa->size());
    for (size_t variant = 0; variant < enable_pa->size(); ++variant) {
        if (enable_pa->get_at(variant))
            continue;
        enable_pa->values[variant]   = true;
        pa->values[variant]          = 0.0;
        adaptive_pa->values[variant] = false;
    }
}

} // namespace


void cut_calibration_model(Model &model, double z, ModelObjectCutAttributes attributes)
{
    if (model.objects.size() != 1 || model.objects.front()->instances.empty())
        throw std::invalid_argument("A tower calibration requires one object with an instance");
    const Vec3d instance_offset = model.objects.front()->instances.front()->get_offset();
    Cut cut(model.objects.front(), 0, Geometry::translation_transform(z * Vec3d::UnitZ() - instance_offset), attributes);
    const auto &objects = cut.perform_with_plane();
    model.delete_object(size_t(0));
    for (const ModelObject *object : objects)
        model.add_object(*object);
}

Calib_Params apply_max_volumetric_speed_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                              DynamicPrintConfig &printer, const Calib_Params &params,
                                              const CalibrationCut &cut)
{
    auto print_config = &print;
    auto filament_config = &filament;
    auto printer_config = &printer;
    auto obj = model.objects[0];
    auto& obj_cfg = obj->config;

    auto bed_shape = printer_config->option<ConfigOptionPoints>("printable_area")->values;
    BoundingBoxf bed_ext = get_extents(bed_shape);
    auto scale_obj = (bed_ext.size().x() - 10) / obj->bounding_box_exact().size().x();
    if (scale_obj < 1.0)
        obj->scale(scale_obj, 1, 1);

    const ConfigOptionFloats* nozzle_diameter_config = printer_config->option<ConfigOptionFloats>("nozzle_diameter");
    assert(nozzle_diameter_config->values.size() > 0);
    double nozzle_diameter = nozzle_diameter_config->values[0];
    double line_width = nozzle_diameter * 1.75;
    double layer_height = nozzle_diameter * 0.8;

    auto max_lh = printer_config->option<ConfigOptionFloats>("max_layer_height");
    for (size_t i = 0; i < max_lh->values.size(); ++i) {
        if (max_lh->values[i] < layer_height)
            max_lh->values[i] = layer_height;
    }

    const double filament_max_volumetric_speed = filament_config->option<ConfigOptionFloats>("filament_max_volumetric_speed")->get_at(0);
    set_config_values<double, ConfigOptionFloats>(filament_config, "filament_max_volumetric_speed", std::max(filament_max_volumetric_speed, 200.0));
    filament_config->set_key_value("slow_down_layer_time", new ConfigOptionFloats{0.0});
    printer_config->set_key_value("resonance_avoidance", new ConfigOptionBool{false});
    obj_cfg.set_key_value("enable_overhang_speed", new ConfigOptionBoolsNullable(1, false));
    obj_cfg.set_key_value("wall_loops", new ConfigOptionInt(1));
    obj_cfg.set_key_value("alternate_extra_wall", new ConfigOptionBool(false));
    obj_cfg.set_key_value("top_shell_layers", new ConfigOptionInt(0));
    obj_cfg.set_key_value("bottom_shell_layers", new ConfigOptionInt(0));
    obj_cfg.set_key_value("sparse_infill_density", new ConfigOptionPercent(0));
    obj_cfg.set_key_value("outer_wall_line_width", new ConfigOptionFloatOrPercent(line_width, false));
    obj_cfg.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
    obj_cfg.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterAndInner));
    obj_cfg.set_key_value("brim_width", new ConfigOptionFloat(5.0));
    obj_cfg.set_key_value("brim_object_gap", new ConfigOptionFloat(0.0));
    obj_cfg.set_key_value("precise_z_height", new ConfigOptionBool(false));
    print_config->set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlTraditional));
    print_config->set_key_value("spiral_mode", new ConfigOptionBool(true));
    print_config->set_key_value("max_volumetric_extrusion_rate_slope", new ConfigOptionFloat(0));
    print_config->set_key_value("enable_wrapping_detection", new ConfigOptionBool(false));

    //  cut upper
    auto obj_bb = obj->bounding_box_exact();
    auto height = (params.end - params.start + 1) / params.step;
    if (height < obj_bb.size().z()) {
        cut(height, ModelObjectCutAttribute::KeepLower);
    }


    auto new_params  = params;
    auto mm3_per_mm  = Flow(line_width, layer_height, nozzle_diameter).mm3_per_mm() * filament_config->option<ConfigOptionFloats>("filament_flow_ratio")->get_at(0);
    new_params.end   = params.end / mm3_per_mm;
    new_params.start = params.start / mm3_per_mm;
    new_params.step  = params.step / mm3_per_mm;

    return new_params;
}

Calib_Params apply_retraction_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                              DynamicPrintConfig &printer, const Calib_Params &params,
                                              const CalibrationCut &cut)
{
    auto print_config = &print;
    auto printer_config = &printer;
    auto obj = model.objects[0];

    print_config->set_key_value("enable_wrapping_detection", new ConfigOptionBool(false));
    print_config->set_key_value("wipe_inward", new ConfigOptionBool(false));

    float nozzle_diameter = printer_config->option<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
    float layer_height;
    if (nozzle_diameter <= 0.1f) {
        layer_height = 0.05f;
    } else if (nozzle_diameter <= 0.2f) {
        layer_height = 0.1f;
    } else {
        layer_height = 0.2f;
    }

    auto max_lh = printer_config->option<ConfigOptionFloats>("max_layer_height");
    for (size_t i = 0; i < max_lh->values.size(); ++i) {
        if (max_lh->values[i] < layer_height) max_lh->values[i] = layer_height;
    }

    printer_config->set_key_value("resonance_avoidance", new ConfigOptionBool{false});
    printer_config->set_key_value("use_firmware_retraction", new ConfigOptionBool(false));
    obj->config.set_key_value("wall_loops", new ConfigOptionInt(2));
    obj->config.set_key_value("top_shell_layers", new ConfigOptionInt(0));
    obj->config.set_key_value("bottom_shell_layers", new ConfigOptionInt(3));
    obj->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(0));
    print_config->set_key_value("initial_layer_print_height", new ConfigOptionFloat(layer_height));
    obj->config.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
    obj->config.set_key_value("alternate_extra_wall", new ConfigOptionBool(false));
    obj->config.set_key_value("seam_position", new ConfigOptionEnum<SeamPosition>(spAligned));
    obj->config.set_key_value("wall_sequence", new ConfigOptionEnum<WallSequence>(WallSequence::InnerOuter));
    obj->config.set_key_value("overhang_reverse", new ConfigOptionBool(false));
    obj->config.set_key_value("precise_z_height", new ConfigOptionBool(false));
    obj->config.set_key_value("seam_slope_type", new ConfigOptionEnum<SeamScarfType>(SeamScarfType::None));


    //  cut upper
    auto obj_bb = obj->bounding_box_exact();
    auto height = 1.0 + 0.4 + ((params.end - params.start)) / params.step - EPSILON;
    if (height < obj_bb.size().z()) {
        cut(height, ModelObjectCutAttribute::KeepLower);
    }

    return params;
}

Calib_Params apply_vfa_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                              DynamicPrintConfig &printer, const Calib_Params &params,
                                              const CalibrationCut &cut)
{
    auto print_config = &print;
    auto filament_config = &filament;
    auto printer_config = &printer;

    const ConfigOptionFloats* nozzle_diameter_config = printer_config->option<ConfigOptionFloats>("nozzle_diameter");
    size_t nozzle_id = static_cast<size_t>(std::max(params.extruder_id, 0));
    double nozzle_diameter = vfa_base_nozzle_diameter;
    if (nozzle_diameter_config && !nozzle_diameter_config->values.empty()) {
        nozzle_id = std::min(nozzle_id, nozzle_diameter_config->values.size() - 1);
        nozzle_diameter = nozzle_diameter_config->values[nozzle_id];
    }
    if (nozzle_diameter <= 0.0)
        nozzle_diameter = vfa_base_nozzle_diameter;

    // Resolved layer height: use the (possibly auto-adjusted) value from the dialog, else default to nozzle/2.
    double layer_height = params.vfa_layer_height > 0.0 ? params.vfa_layer_height : nozzle_diameter / 2.0;

    // cut upper (on the unscaled model, using the base block height); the scaling below keeps the physical
    // block height (vfa_layers_per_block * layer_height) in sync with the speed stepping in GCode::process_layer.
    // Subtract EPSILON (as the temperature tower does) so the cut lands just below the flat block surface instead
    // of exactly on it, which would otherwise add a degenerate extra layer.
    auto obj_bb = model.objects[0]->bounding_box_exact();
    auto height = vfa_base_block_height * ((params.end - params.start) / params.step + 1) - EPSILON;
    if (height < obj_bb.size().z()) {
        cut(height, ModelObjectCutAttribute::KeepLower);
    }

    // When resizing is enabled, XY scales with the nozzle (footprint / line width) and Z scales so each base
    // block becomes vfa_layers_per_block layers of the resolved layer height. When disabled the 0.4 mm / 0.2 mm
    // reference model is printed as-is (preset layer height kept).
    if (params.nozzle_based_resize) {
        const double xy_scale = nozzle_diameter / vfa_base_nozzle_diameter;
        const double z_scale  = (vfa_layers_per_block * layer_height) / vfa_base_block_height;
        if (std::abs(xy_scale - 1.0) > EPSILON || std::abs(z_scale - 1.0) > EPSILON)
            model.objects[0]->scale(xy_scale, xy_scale, z_scale);
    }

    model.objects[0]->ensure_on_bed();

    printer_config->set_key_value("resonance_avoidance", new ConfigOptionBool{false});
    filament_config->set_key_value("slow_down_layer_time", new ConfigOptionFloats { 0.0 });
    set_config_values<bool, ConfigOptionBoolsNullable>(print_config, "enable_overhang_speed", false);
    print_config->set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlTraditional));
    print_config->set_key_value("wall_loops", new ConfigOptionInt(1));
    print_config->set_key_value("alternate_extra_wall", new ConfigOptionBool(false));
    print_config->set_key_value("top_shell_layers", new ConfigOptionInt(0));
    print_config->set_key_value("bottom_shell_layers", new ConfigOptionInt(1));
    print_config->set_key_value("sparse_infill_density", new ConfigOptionPercent(0));
    print_config->set_key_value("detect_thin_wall", new ConfigOptionBool(false));
    print_config->set_key_value("spiral_mode", new ConfigOptionBool(true));
    print_config->set_key_value("enable_wrapping_detection", new ConfigOptionBool(false));
    print_config->set_key_value("precise_z_height", new ConfigOptionBool(false));
    if (params.nozzle_based_resize) {
        print_config->set_key_value("initial_layer_print_height", new ConfigOptionFloat(layer_height));
        model.objects[0]->config.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
    }
    model.objects[0]->config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
    model.objects[0]->config.set_key_value("brim_width", new ConfigOptionFloat(3.0));
    model.objects[0]->config.set_key_value("brim_object_gap", new ConfigOptionFloat(0.0));

    // Pass the resolved layer height on (only meaningful when resized). GCode's VFA stepping is layer-based, so
    // it does not require it, but keep it consistent with the geometry.
    Calib_Params calib_params = params;
    calib_params.vfa_layer_height = params.nozzle_based_resize ? layer_height : 0.0;
    return calib_params;
}

void apply_tower_blocks_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &printer,
                                    double bottom_z, double top_z, const Calib_Params &params,
                                    const CalibrationCut &cut)
{
    constexpr double base_temp_tower_nozzle_diameter = 0.4;
    auto printer_config = &printer;
    const ConfigOptionFloats* nozzle_diameter_config = printer_config->option<ConfigOptionFloats>("nozzle_diameter");
    size_t nozzle_id = static_cast<size_t>(std::max(params.extruder_id, 0));
    double nozzle_diameter = base_temp_tower_nozzle_diameter;
    if (nozzle_diameter_config && !nozzle_diameter_config->values.empty()) {
        nozzle_id = std::min(nozzle_id, nozzle_diameter_config->values.size() - 1);
        nozzle_diameter = nozzle_diameter_config->values[nozzle_id];
    }
    if (nozzle_diameter <= 0.0)
        nozzle_diameter = base_temp_tower_nozzle_diameter;

    const double nozzle_scale = nozzle_diameter / base_temp_tower_nozzle_diameter;

    // cut upper
    auto obj_bb = model.objects[0]->bounding_box_exact();
    // subtract EPSILON offset to avoid cutting at the exact location where the flat surface is
    if (top_z - EPSILON < obj_bb.size().z())
        cut( top_z - EPSILON, ModelObjectCutAttribute::KeepLower);

    // cut bottom
    obj_bb = model.objects[0]->bounding_box_exact();
    if (bottom_z > 0 && bottom_z + EPSILON < obj_bb.size().z())
        cut( bottom_z + EPSILON, ModelObjectCutAttribute::KeepUpper);

    if (params.nozzle_based_resize && std::abs(nozzle_scale - 1.0) > EPSILON)
        model.objects[0]->scale(nozzle_scale, nozzle_scale, nozzle_scale);

    model.objects[0]->ensure_on_bed();

    printer_config->set_key_value("resonance_avoidance", new ConfigOptionBool{false});
    // When resizing is disabled the 0.4 mm / 0.2 mm reference model is printed as-is (preset layer height kept).
    if (params.nozzle_based_resize)
        model.objects[0]->config.set_key_value("layer_height", new ConfigOptionFloat(nozzle_diameter/2));
    model.objects[0]->config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
    model.objects[0]->config.set_key_value("brim_width", new ConfigOptionFloat(5.0));
    model.objects[0]->config.set_key_value("brim_object_gap", new ConfigOptionFloat(0.0));
    model.objects[0]->config.set_key_value("alternate_extra_wall", new ConfigOptionBool(false));
    model.objects[0]->config.set_key_value("seam_slope_type", new ConfigOptionEnum<SeamScarfType>(SeamScarfType::None));
    model.objects[0]->config.set_key_value("overhang_reverse", new ConfigOptionBool(false));
    model.objects[0]->config.set_key_value("precise_z_height", new ConfigOptionBool(false));
    // Supports would fill the overhangs and bridges the towers exist to show.
    model.objects[0]->config.set_key_value("enable_support", new ConfigOptionBool(false));

    auto print_config = &print;
    print_config->set_key_value("enable_wrapping_detection", new ConfigOptionBool(false));
    if (params.nozzle_based_resize)
        print_config->set_key_value("initial_layer_print_height", new ConfigOptionFloat(nozzle_diameter/2));
}

Calib_Params apply_temperature_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                         DynamicPrintConfig &printer, const Calib_Params &params,
                                         const CalibrationCut &cut)
{
    // The model has one 10 mm block per 5 degrees, from 500 at its base.
    const double bottom = (500 - std::lround(params.start)) / 5 * 10.0;
    const double top = ((500 - std::lround(params.end)) / 5 + 1) * 10.0;
    apply_tower_blocks_calibration(model, print, printer, bottom, top, params, cut);
    const int start_temp = int(std::lround(params.start));
    set_config_values<int, ConfigOptionInts>(&filament, "nozzle_temperature_initial_layer", start_temp);
    set_config_values<int, ConfigOptionInts>(&filament, "nozzle_temperature", start_temp);
    return params;
}

Calib_Params apply_fan_tower_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                       DynamicPrintConfig &printer, const Calib_Params &params,
                                       const CalibrationCut &cut)
{
    apply_tower_blocks_calibration(model, print, printer, params.start, params.end + 10.0, params, cut);
    // Overhang markers only reach the cooling pass with this enabled. The test slope must qualify
    // while its stringing pillars remain below the pinned overhang threshold.
    if (fan_tower_steps_overhang_fan(params.mode))
        set_config_values<bool, ConfigOptionBools>(&filament, "enable_overhang_bridge_fan", true);
    if (params.mode == CalibMode::Calib_Overhang_Fan_Tower)
        set_config_values<int, ConfigOptionEnumsGeneric>(&filament, "overhang_fan_threshold", Overhang_threshold_2_4);
    return params;
}

Calib_Params apply_input_shaping_frequency_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                                 DynamicPrintConfig &printer, const Calib_Params &params)
{
    auto print_config = &print;
    auto filament_config = &filament;
    auto printer_config = &printer;
    const auto gcode_flavor_option = printer_config->option<ConfigOptionEnum<GCodeFlavor>>("gcode_flavor");

    if (has_junction_deviation(printer_config)) {
        printer_config->set_key_value("machine_max_junction_deviation", new ConfigOptionFloats {(std::max(printer_config->option<ConfigOptionFloats>("machine_max_junction_deviation")->values.front(), 0.25))});
        set_config_values<double, ConfigOptionFloatsNullable>(print_config, "default_junction_deviation", 0);
    } else {
        const double jerk_value = (gcode_flavor_option && gcode_flavor_option->value == GCodeFlavor::gcfKlipper) ? 5.0 : 10.0;
        printer_config->set_key_value("machine_max_jerk_x", new ConfigOptionFloats{std::max(printer_config->option<ConfigOptionFloats>("machine_max_jerk_x")->values.front(), jerk_value)});
        printer_config->set_key_value("machine_max_jerk_y", new ConfigOptionFloats{std::max(printer_config->option<ConfigOptionFloats>("machine_max_jerk_y")->values.front(), jerk_value)});
        set_config_values<double, ConfigOptionFloatsNullable>(print_config, "default_jerk", 0);
    }

    zero_pressure_advance_where_disabled(filament_config);

    printer_config->set_key_value("resonance_avoidance", new ConfigOptionBool{false});
    printer_config->set_key_value("input_shaping_emit", new ConfigOptionBool{false});
    filament_config->set_key_value("slow_down_layer_time", new ConfigOptionFloats { 0.0 });
    filament_config->set_key_value("slow_down_min_speed", new ConfigOptionFloats { 0.0 });
    filament_config->set_key_value("slow_down_for_layer_cooling", new ConfigOptionBools{false});
    print_config->set_key_value("layer_height", new ConfigOptionFloat(0.2));
    set_config_values<bool, ConfigOptionBools>(print_config, "enable_overhang_speed", false);
    // A 100% factor sends Klipper MINIMUM_CRUISE_RATIO=0 with every acceleration change, for the whole test.
    print_config->set_key_value("accel_to_decel_enable", new ConfigOptionBool(true));
    print_config->set_key_value("accel_to_decel_factor", new ConfigOptionPercent(100));
    print_config->set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlTraditional));
    print_config->set_key_value("wall_loops", new ConfigOptionInt(1));
    print_config->set_key_value("top_shell_layers", new ConfigOptionInt(0));
    print_config->set_key_value("bottom_shell_layers", new ConfigOptionInt(1));
    print_config->set_key_value("sparse_infill_density", new ConfigOptionPercent(0));
    print_config->set_key_value("detect_thin_wall", new ConfigOptionBool(false));
    print_config->set_key_value("spiral_mode", new ConfigOptionBool(true));
    print_config->set_key_value("spiral_mode_smooth", new ConfigOptionBool(false));
    print_config->set_key_value("bottom_surface_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
    const double machine_max_speed = std::min(printer_config->option<ConfigOptionFloats>("machine_max_speed_x")->get_at(0), printer_config->option<ConfigOptionFloats>("machine_max_speed_y")->get_at(0));
    const double machine_max_acceleration = printer_config->option<ConfigOptionFloats>("machine_max_acceleration_extruding")->get_at(0);
    set_config_values<double, ConfigOptionFloatsNullable>(print_config, "outer_wall_speed", machine_max_speed);
    set_config_values<double, ConfigOptionFloatsNullable>(print_config, "default_acceleration", machine_max_acceleration);
    set_config_values<double, ConfigOptionFloatsNullable>(print_config, "outer_wall_acceleration", machine_max_acceleration);
    print_config->set_key_value("precise_z_height", new ConfigOptionBool(false));
    model.objects[0]->config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
    model.objects[0]->config.set_key_value("brim_width", new ConfigOptionFloat(3.0));
    model.objects[0]->config.set_key_value("brim_object_gap", new ConfigOptionFloat(0.0));

    return params;
}

Calib_Params apply_input_shaping_damping_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                                 DynamicPrintConfig &printer, const Calib_Params &params)
{
    auto print_config = &print;
    auto filament_config = &filament;
    auto printer_config = &printer;
    const auto gcode_flavor_option = printer_config->option<ConfigOptionEnum<GCodeFlavor>>("gcode_flavor");

    if (has_junction_deviation(printer_config)) {
        printer_config->set_key_value("machine_max_junction_deviation", new ConfigOptionFloats {(std::max(printer_config->option<ConfigOptionFloats>("machine_max_junction_deviation")->values.front(), 0.25))});
        set_config_values<double, ConfigOptionFloatsNullable>(print_config, "default_junction_deviation", 0);
    } else {
        const double jerk_value = (gcode_flavor_option && gcode_flavor_option->value == GCodeFlavor::gcfKlipper) ? 5.0 : 10.0;
        printer_config->set_key_value("machine_max_jerk_x", new ConfigOptionFloats{std::max(printer_config->option<ConfigOptionFloats>("machine_max_jerk_x")->values.front(), jerk_value)});
        printer_config->set_key_value("machine_max_jerk_y", new ConfigOptionFloats{std::max(printer_config->option<ConfigOptionFloats>("machine_max_jerk_y")->values.front(), jerk_value)});
        set_config_values<double, ConfigOptionFloatsNullable>(print_config, "default_jerk", 0);
    }

    zero_pressure_advance_where_disabled(filament_config);

    printer_config->set_key_value("resonance_avoidance", new ConfigOptionBool{false});
    printer_config->set_key_value("input_shaping_emit", new ConfigOptionBool{false});
    filament_config->set_key_value("slow_down_layer_time", new ConfigOptionFloats { 0.0 });
    filament_config->set_key_value("slow_down_min_speed", new ConfigOptionFloats { 0.0 });
    filament_config->set_key_value("slow_down_for_layer_cooling", new ConfigOptionBools{false});
    set_config_values<bool, ConfigOptionBools>(print_config, "enable_overhang_speed", false);
    // A 100% factor sends Klipper MINIMUM_CRUISE_RATIO=0 with every acceleration change, for the whole test.
    print_config->set_key_value("accel_to_decel_enable", new ConfigOptionBool(true));
    print_config->set_key_value("accel_to_decel_factor", new ConfigOptionPercent(100));
    print_config->set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlTraditional));
    print_config->set_key_value("wall_loops", new ConfigOptionInt(1));
    print_config->set_key_value("top_shell_layers", new ConfigOptionInt(0));
    print_config->set_key_value("bottom_shell_layers", new ConfigOptionInt(1));
    print_config->set_key_value("sparse_infill_density", new ConfigOptionPercent(0));
    print_config->set_key_value("detect_thin_wall", new ConfigOptionBool(false));
    print_config->set_key_value("spiral_mode", new ConfigOptionBool(true));
    print_config->set_key_value("spiral_mode_smooth", new ConfigOptionBool(false));
    print_config->set_key_value("bottom_surface_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
    const double machine_max_speed = std::min(printer_config->option<ConfigOptionFloats>("machine_max_speed_x")->get_at(0), printer_config->option<ConfigOptionFloats>("machine_max_speed_y")->get_at(0));
    const double machine_max_acceleration = printer_config->option<ConfigOptionFloats>("machine_max_acceleration_extruding")->get_at(0);
    set_config_values<double, ConfigOptionFloatsNullable>(print_config, "outer_wall_speed", machine_max_speed);
    set_config_values<double, ConfigOptionFloatsNullable>(print_config, "default_acceleration", machine_max_acceleration);
    set_config_values<double, ConfigOptionFloatsNullable>(print_config, "outer_wall_acceleration", machine_max_acceleration);
    print_config->set_key_value("precise_z_height", new ConfigOptionBool(false));
    model.objects[0]->config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
    model.objects[0]->config.set_key_value("brim_width", new ConfigOptionFloat(3.0));
    model.objects[0]->config.set_key_value("brim_object_gap", new ConfigOptionFloat(0.0));

    return params;
}

Calib_Params apply_cornering_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                                 DynamicPrintConfig &printer, const Calib_Params &params)
{
    auto print_config = &print;
    auto filament_config = &filament;
    auto printer_config = &printer;

    if (has_junction_deviation(printer_config)) {
        printer_config->set_key_value("machine_max_junction_deviation", new ConfigOptionFloats{params.end});
        set_config_values<double, ConfigOptionFloatsNullable>(print_config, "default_junction_deviation", 0);
    } else {
        printer_config->set_key_value("machine_max_jerk_x", new ConfigOptionFloats{params.end});
        printer_config->set_key_value("machine_max_jerk_y", new ConfigOptionFloats{params.end});
        set_config_values<double, ConfigOptionFloatsNullable>(print_config, "default_jerk", 0);
    }

    zero_pressure_advance_where_disabled(filament_config);

    printer_config->set_key_value("resonance_avoidance", new ConfigOptionBool{false});
    printer_config->set_key_value("input_shaping_emit", new ConfigOptionBool{true});
    printer_config->set_key_value("input_shaping_type", new ConfigOptionEnum<InputShaperType>(InputShaperType::Disable));
    filament_config->set_key_value("slow_down_layer_time", new ConfigOptionFloats { 0.0 });
    filament_config->set_key_value("slow_down_min_speed", new ConfigOptionFloats { 0.0 });
    filament_config->set_key_value("slow_down_for_layer_cooling", new ConfigOptionBools{false});
    const double filament_max_volumetric_speed = filament_config->option<ConfigOptionFloats>("filament_max_volumetric_speed")->get_at(0);
    filament_config->set_key_value("filament_max_volumetric_speed", new ConfigOptionFloats{std::max(filament_max_volumetric_speed, 200.0)});
    set_config_values<bool, ConfigOptionBools>(print_config, "enable_overhang_speed", false);
    print_config->set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlTraditional));
    print_config->set_key_value("wall_loops", new ConfigOptionInt(1));
    print_config->set_key_value("top_shell_layers", new ConfigOptionInt(0));
    print_config->set_key_value("bottom_shell_layers", new ConfigOptionInt(1));
    print_config->set_key_value("sparse_infill_density", new ConfigOptionPercent(0));
    print_config->set_key_value("detect_thin_wall", new ConfigOptionBool(false));
    print_config->set_key_value("spiral_mode", new ConfigOptionBool(true));
    print_config->set_key_value("spiral_mode_smooth", new ConfigOptionBool(false));
    print_config->set_key_value("bottom_surface_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
    const double machine_max_speed = std::min(printer_config->option<ConfigOptionFloats>("machine_max_speed_x")->get_at(0), printer_config->option<ConfigOptionFloats>("machine_max_speed_y")->get_at(0));
    const double machine_max_acceleration = printer_config->option<ConfigOptionFloats>("machine_max_acceleration_extruding")->get_at(0);
    set_config_values<double, ConfigOptionFloatsNullable>(print_config, "outer_wall_speed", machine_max_speed);
    set_config_values<double, ConfigOptionFloatsNullable>(print_config, "default_acceleration", machine_max_acceleration);
    set_config_values<double, ConfigOptionFloatsNullable>(print_config, "outer_wall_acceleration", machine_max_acceleration);
    print_config->set_key_value("precise_z_height", new ConfigOptionBool(false));
    model.objects[0]->config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
    model.objects[0]->config.set_key_value("brim_width", new ConfigOptionFloat(3.0));
    model.objects[0]->config.set_key_value("brim_object_gap", new ConfigOptionFloat(0.0));

    return params;
}

std::string tower_calibration_asset(CalibMode mode, int test_model)
{
    switch (mode) {
    case CalibMode::Calib_Vol_speed_Tower: return "volumetric_speed/SpeedTestStructure.drc";
    case CalibMode::Calib_Retraction_tower: return "retraction/retraction_tower.drc";
    case CalibMode::Calib_VFA_Tower: return "vfa/vfa.drc";
    case CalibMode::Calib_Temp_Tower: return "temperature_tower/temperature_tower.drc";
    case CalibMode::Calib_Overhang_Fan_Tower:
    case CalibMode::Calib_Part_Fan_Tower: return "fan_tower/fan_tower.drc";
    case CalibMode::Calib_Bridge_Fan_Tower: return "fan_tower/bridge_fan_tower.drc";
    case CalibMode::Calib_Small_Feature_Fan_Tower: return "fan_tower/small_feature_fan_tower.drc";
    case CalibMode::Calib_Input_shaping_freq:
    case CalibMode::Calib_Input_shaping_damp:
        return test_model < 1 ? "input_shaping/ringing_tower.drc" : "input_shaping/fast_tower_test.drc";
    case CalibMode::Calib_Cornering:
        return test_model == 0 ? "input_shaping/ringing_tower.drc" :
            test_model == 1 ? "input_shaping/fast_tower_test.drc" : "cornering/SCV-V2.drc";
    default: throw std::invalid_argument("Unsupported calibration tower model");
    }
}

} // namespace Slic3r
