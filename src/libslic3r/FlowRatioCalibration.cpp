#include "FlowRatioCalibration.hpp"

#include "Flow.hpp"
#include "ParameterUtils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace Slic3r {
namespace {
void validate_pass(int pass)
{
    if (pass != 1 && pass != 2)
        throw std::invalid_argument("Flow-ratio calibration pass must be 1 or 2");
}
}

double flow_ratio_calibration_modifier(const std::string &object_name)
{
    const std::string prefix = "flowrate_";
    if (object_name.compare(0, prefix.size(), prefix) != 0)
        throw std::invalid_argument("Unexpected flow-ratio calibration object: " + object_name);
    std::string number = object_name.substr(prefix.size());
    if (!number.empty() && number.front() == 'm')
        number.front() = '-';
    std::istringstream input(number);
    input.imbue(std::locale::classic());
    double modifier = 0.;
    if (!(input >> modifier) || !input.eof() || !std::isfinite(modifier))
        throw std::invalid_argument("Invalid flow-ratio calibration modifier: " + object_name);
    return modifier;
}

std::string flow_ratio_calibration_asset(bool linear, int pass)
{
    validate_pass(pass);
    if (linear)
        return pass == 1 ? "filament_flow/Orca-LinearFlow.3mf" : "filament_flow/Orca-LinearFlow_fine.3mf";
    return pass == 1 ? "filament_flow/flowrate-test-pass1.3mf" : "filament_flow/flowrate-test-pass2.3mf";
}

FlowRatioCalibrationPlan plan_flow_ratio_calibration(const DynamicPrintConfig &config, bool linear, int pass)
{
    validate_pass(pass);
    const auto &nozzles = config.option<ConfigOptionFloats>("nozzle_diameter")->values;
    const auto &extruders = config.option<ConfigOptionEnumsGeneric>("extruder_type")->values;
    const auto &volumes = config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type")->values;
    if (nozzles.empty() || extruders.size() < nozzles.size() || volumes.size() < nozzles.size() || nozzles.front() <= 0.)
        throw std::invalid_argument("Flow-ratio calibration requires a configured nozzle and extruder");
    FlowRatioCalibrationPlan plan;
    plan.nozzle_diameter = nozzles.front();
    plan.layer_height = plan.nozzle_diameter / 2.;
    plan.first_layer_height = std::max(config.opt_float("initial_layer_print_height"), plan.layer_height);
    plan.flow_ratio = config.option<ConfigOptionFloats>("filament_flow_ratio")->get_at(0);
    if (!(plan.flow_ratio > 0.) || !std::isfinite(plan.flow_ratio))
        throw std::invalid_argument("Flow-ratio calibration requires a positive filament flow ratio");
    const double xy_scale = plan.nozzle_diameter / 0.6;
    plan.scale = Vec3d(xy_scale > 1.2 ? xy_scale : 1., xy_scale > 1.2 ? xy_scale : 1.,
                       (plan.first_layer_height + 9. * plan.layer_height) / 2.);
    // Cap the speeds for the extrusion the calibration actually prints.
    const Flow flow(plan.nozzle_diameter * 1.2, plan.layer_height, plan.nozzle_diameter);
    auto speeds = [&](const std::string &key) {
        std::vector<double> values;
        for (size_t i = 0; i < nozzles.size(); ++i) {
            int index = get_index_for_extruder_parameter(config, "filament_max_volumetric_speed", int(i),
                                                         ExtruderType(extruders[i]), NozzleVolumeType(volumes[i]));
            const double mvs = config.option<ConfigOptionFloats>("filament_max_volumetric_speed")->get_at(index);
            const double ratio = config.option<ConfigOptionFloats>("filament_flow_ratio")->get_at(index);
            if (!(mvs > 0.) || !(ratio > 0.))
                throw std::invalid_argument("Flow-ratio calibration requires positive volumetric speed and flow ratio");
            const double ceiling = linear ? mvs / (flow.mm3_per_mm() * (ratio + (pass == 2 ? 0.035 : 0.05)) / ratio)
                                          : mvs / (flow.mm3_per_mm() * (pass == 1 ? 1.2 : 1.));
            index = get_index_for_extruder_parameter(config, key, int(i), ExtruderType(extruders[i]), NozzleVolumeType(volumes[i]));
            const double speed = std::floor(std::min(config.option<ConfigOptionFloatsNullable>(key)->get_at(index), ceiling));
            if (!(speed > 0.))
                throw std::invalid_argument("Flow-ratio calibration speed must be positive");
            for (size_t j = 0; j < volumes.size(); ++j)
                values.push_back(speed);
        }
        return values;
    };
    plan.internal_solid_speeds = speeds("internal_solid_infill_speed");
    plan.top_surface_speeds = speeds("top_surface_speed");
    return plan;
}

void apply_flow_ratio_calibration(ModelObjectPtrs &objects, DynamicPrintConfig &print_config,
                                 const FlowRatioCalibrationPlan &plan, bool linear, int pass, InfillPattern pattern)
{
    validate_pass(pass);
    for (const auto *object : objects) {
        const double modifier = flow_ratio_calibration_modifier(object->name);
        if ((linear ? plan.flow_ratio + modifier : 1. + modifier / 100.) <= 0.)
            throw std::invalid_argument("Flow-ratio calibration patch would extrude no material");
    }
    // adjust parameters
    for (auto _obj : objects) {
        _obj->ensure_on_bed();
        _obj->config.set_key_value("wall_loops", new ConfigOptionInt(1));
        _obj->config.set_key_value("only_one_wall_top", new ConfigOptionBool(true));
        _obj->config.set_key_value("thick_internal_bridges", new ConfigOptionBool(false));
        _obj->config.set_key_value("enable_extra_bridge_layer", new ConfigOptionEnum<EnableExtraBridgeLayer>(eblDisabled));
        _obj->config.set_key_value("internal_bridge_density", new ConfigOptionPercent(100));
        _obj->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(35));
        _obj->config.set_key_value("min_width_top_surface", new ConfigOptionFloatOrPercent(100,true));
        _obj->config.set_key_value("bottom_shell_layers", new ConfigOptionInt(2));
        _obj->config.set_key_value("top_shell_layers", new ConfigOptionInt(5));
        _obj->config.set_key_value("top_shell_thickness", new ConfigOptionFloat(0));
        _obj->config.set_key_value("bottom_shell_thickness", new ConfigOptionFloat(0));
        _obj->config.set_key_value("detect_thin_wall", new ConfigOptionBool(true));
        _obj->config.set_key_value("filter_out_gap_fill", new ConfigOptionFloat(0));
        _obj->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
        _obj->config.set_key_value("top_surface_line_width", new ConfigOptionFloatOrPercent(plan.nozzle_diameter * 1.2f, false));
        _obj->config.set_key_value("internal_solid_infill_line_width", new ConfigOptionFloatOrPercent(plan.nozzle_diameter * 1.2f, false));
        // ORCA: use the pattern parameter
        _obj->config.set_key_value("top_surface_pattern", new ConfigOptionEnum<InfillPattern>(pattern));
        const auto *top_solid_flow = dynamic_cast<const ConfigOptionFloatsNullable *>(_obj->config.option("top_solid_infill_flow_ratio"));
        _obj->config.set_key_value("top_solid_infill_flow_ratio",
                                   new ConfigOptionFloatsNullable(top_solid_flow ? top_solid_flow->size() : 1, 1.0f));
        _obj->config.set_key_value("infill_direction", new ConfigOptionFloat(45));
        _obj->config.set_key_value("solid_infill_direction", new ConfigOptionFloat(135));
        _obj->config.set_key_value("center_of_surface_pattern", new ConfigOptionEnum<CenterOfSurfacePattern>(CenterOfSurfacePattern::Each_Surface));
        _obj->config.set_key_value("separated_infills", new ConfigOptionBool(false));
        _obj->config.set_key_value("align_infill_direction_to_model", new ConfigOptionBool(true));
        _obj->config.set_key_value("ironing_type", new ConfigOptionEnum<IroningType>(IroningType::NoIroning));
        _obj->config.set_key_value("internal_solid_infill_speed", new ConfigOptionFloatsNullable(plan.internal_solid_speeds));
        _obj->config.set_key_value("top_surface_speed", new ConfigOptionFloatsNullable(plan.top_surface_speeds));
        _obj->config.set_key_value("seam_slope_type", new ConfigOptionEnum<SeamScarfType>(SeamScarfType::None));
        _obj->config.set_key_value("gap_fill_target", new ConfigOptionEnum<GapFillTarget>(GapFillTarget::gftNowhere));
        print_config.set_key_value("max_volumetric_extrusion_rate_slope", new ConfigOptionFloat(0));
        // ORCA: request the calibration's special toolpath order (chords first, center spiral
        // last and inside-out) so opposing directions collide into the tactile lip the test
        // reads. The special order only applies while the fill order is Default, so reset the
        // profile's fill order on the calibration objects; changing the setting on the object
        // afterwards deliberately overrides the special order.
        _obj->config.set_key_value("calib_flowrate_topinfill_special_order", new ConfigOptionBool(true));
        _obj->config.set_key_value("top_surface_fill_order", new ConfigOptionEnum<SurfaceFillOrder>(SurfaceFillOrder::Default));

        const double modifier = flow_ratio_calibration_modifier(_obj->name);
        if(linear)
            _obj->config.set_key_value("print_flow_ratio", new ConfigOptionFloat((plan.flow_ratio + modifier)/plan.flow_ratio));
        else
            _obj->config.set_key_value("print_flow_ratio", new ConfigOptionFloat(1.0f + modifier/100.f));

    }

    print_config.set_key_value("layer_height", new ConfigOptionFloat(plan.layer_height));
    print_config.set_key_value("alternate_extra_wall", new ConfigOptionBool(false));
    print_config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(plan.first_layer_height));
    print_config.set_key_value("reduce_crossing_wall", new ConfigOptionBool(true));
    print_config.set_key_value("enable_wrapping_detection", new ConfigOptionBool(false));
}
} // namespace Slic3r
