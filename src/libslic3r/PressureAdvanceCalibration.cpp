#include "PressureAdvanceCalibration.hpp"

#include "Config.hpp"
#include "CutUtils.hpp"
#include "Flow.hpp"
#include "Model.hpp"
#include "ParameterUtils.hpp"
#include "PrintConfig.hpp"
#include "TowerCalibration.hpp"
#include "calib.hpp"

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
    default: throw std::invalid_argument("Unsupported pressure-advance calibration model");
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

} // namespace Slic3r
