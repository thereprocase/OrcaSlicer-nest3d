#include "CalibrationRecipe.hpp"
#include "calib.hpp"
#include "Config.hpp"
#include "CutUtils.hpp"
#include "Point.hpp"
#include "FlowRatioCalibration.hpp"
#include "BoundingBox.hpp"
#include "Model.hpp"
#include "PrintConfig.hpp"
#include "TowerCalibration.hpp"
#include "ParameterUtils.hpp"
#include "PressureAdvanceCalibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <istream>
#include <locale>
#include <sstream>
#include <string>
#include <utility>
#include <stdexcept>

namespace Slic3r {

const std::vector<CalibrationRecipe> &calibration_recipes()
{
    static const std::vector<CalibrationRecipe> recipes {
        {"flow-ratio", CalibrationGenerator::FlowRatio, true, false, 1},
        {"flow-ratio-fine", CalibrationGenerator::FlowRatio, true, false, 2},
        {"flow-ratio-yolo", CalibrationGenerator::FlowRatio, true, true, 1},
        {"flow-ratio-yolo-fine", CalibrationGenerator::FlowRatio, true, true, 2},
        {"pressure-advance-line", CalibrationGenerator::PressureAdvance, true, false, 1, CalibMode::Calib_PA_Line},
        {"pressure-advance-pattern", CalibrationGenerator::PressureAdvance, true, false, 1, CalibMode::Calib_PA_Pattern},
        {"pressure-advance-tower", CalibrationGenerator::PressureAdvance, true, false, 1, CalibMode::Calib_PA_Tower},
        {"temperature", CalibrationGenerator::Temperature, true, false, 1, CalibMode::Calib_Temp_Tower},
        {"max-volumetric-speed", CalibrationGenerator::MaxVolumetricSpeed, true, false, 1, CalibMode::Calib_Vol_speed_Tower},
        {"retraction", CalibrationGenerator::Retraction, true, false, 1, CalibMode::Calib_Retraction_tower},
        {"vfa", CalibrationGenerator::VFA, true, false, 1, CalibMode::Calib_VFA_Tower},
        {"cornering", CalibrationGenerator::Cornering, true, false, 1, CalibMode::Calib_Cornering},
        {"input-shaping-frequency", CalibrationGenerator::InputShapingFrequency, true, false, 1, CalibMode::Calib_Input_shaping_freq},
        {"input-shaping-damping", CalibrationGenerator::InputShapingDamping, true, false, 1, CalibMode::Calib_Input_shaping_damp},
        {"overhang-fan", CalibrationGenerator::FanTower, true, false, 1, CalibMode::Calib_Overhang_Fan_Tower},
        {"part-fan", CalibrationGenerator::FanTower, true, false, 1, CalibMode::Calib_Part_Fan_Tower},
        {"bridge-fan", CalibrationGenerator::FanTower, true, false, 1, CalibMode::Calib_Bridge_Fan_Tower},
        {"small-feature-fan", CalibrationGenerator::FanTower, true, false, 1, CalibMode::Calib_Small_Feature_Fan_Tower},
    };
    return recipes;
}

const CalibrationRecipe *find_calibration_recipe(const std::string &name)
{
    const auto &recipes = calibration_recipes();
    const auto it = std::find_if(recipes.begin(), recipes.end(), [&name](const CalibrationRecipe &recipe) {
        return recipe.name == name;
    });
    return it == recipes.end() ? nullptr : &*it;
}

std::string calibration_recipe_asset(const CalibrationRecipe &recipe, const Calib_Params &params)
{
    if (recipe.cli_supported && recipe.generator == CalibrationGenerator::FlowRatio)
        return flow_ratio_calibration_asset(recipe.linear, recipe.pass);
    if (recipe.cli_supported && recipe.mode == CalibMode::Calib_PA_Pattern)
        return {}; // The shared pattern generator builds its anchor geometry natively.
    if (recipe.cli_supported && recipe.generator == CalibrationGenerator::PressureAdvance)
        return pressure_advance_calibration_asset(recipe.mode);
    if (recipe.cli_supported && recipe.mode != CalibMode::Calib_None)
        return tower_calibration_asset(recipe.mode, params.test_model);
    throw std::invalid_argument("Native CLI calibration generation is unavailable for " + recipe.name);
}

std::vector<double> parse_calibration_values(const std::string &text)
{
    std::vector<double> values;
    if (text.empty())
        return values;
    if (text.back() == ',')
        throw std::invalid_argument("Calibration values must be comma-separated finite numbers");
    std::istringstream list(text);
    std::string token;
    while (std::getline(list, token, ',')) {
        std::istringstream number(token);
        number.imbue(std::locale::classic());
        double value = 0.;
        if (!(number >> value) || !(number >> std::ws).eof() || !std::isfinite(value))
            throw std::invalid_argument("Calibration values must be comma-separated finite numbers");
        values.push_back(value);
    }
    return values;
}

CalibrationResult apply_calibration_recipe(Model &model, DynamicPrintConfig &config, const CalibrationRecipe &recipe,
                                     const std::string &surface_pattern, const Calib_Params &params)
{
    // Keep dispatch explicit: adding an advertised recipe must also implement its real generator.
    if (!recipe.cli_supported)
        throw std::invalid_argument("Native CLI calibration generation is unavailable for " + recipe.name);
    if (config.opt_bool("belt_printer"))
        throw std::invalid_argument("CLI calibration currently requires a flat-bed printer");
    if (recipe.generator != CalibrationGenerator::FlowRatio) {
        const bool frequency = recipe.generator == CalibrationGenerator::InputShapingFrequency;
        const bool damping = recipe.generator == CalibrationGenerator::InputShapingDamping;
        const bool cornering = recipe.generator == CalibrationGenerator::Cornering;
        const bool continuous = frequency || damping || cornering;
        if (!std::isfinite(params.start) || !std::isfinite(params.end) || !std::isfinite(params.step) ||
            params.start < 0. || params.end < 0.)
            throw std::invalid_argument("Tower calibration requires finite nonnegative values");
        if (!continuous && (params.step <= 0. || params.step > std::abs(params.end - params.start)))
            throw std::invalid_argument("Tower calibration requires a positive step within the range");
        if (recipe.generator == CalibrationGenerator::Temperature) {
            if (params.start <= params.end || params.start > 500. || params.end < 155. || params.step != 5. ||
                std::fmod(params.start, 5.) != 0. || std::fmod(params.end, 5.) != 0.)
                throw std::invalid_argument("Temperature calibration requires a descending range of multiples of 5 from 155 to 500 C and step 5");
        } else if (!frequency && params.end <= params.start) {
            throw std::invalid_argument("This calibration requires an increasing range");
        }
        if (recipe.generator == CalibrationGenerator::FanTower &&
            (params.end > 100. || params.step != 10. || std::fmod(params.start, 10.) != 0. || std::fmod(params.end, 10.) != 0.))
            throw std::invalid_argument("Fan calibration requires multiples of 10 from 0 to 100 and step 10");
        if (recipe.mode != CalibMode::Calib_PA_Pattern && (model.objects.size() != 1 || model.objects.front()->instances.empty()))
            throw std::invalid_argument("Tower calibration requires exactly one stock model with an instance");
        if (params.extruder_id != 0 || !std::isfinite(params.vfa_layer_height) || params.vfa_layer_height < 0.)
            throw std::invalid_argument("CLI tower calibration requires extruder 0 and a nonnegative layer height");
        const auto *nozzles = config.option<ConfigOptionFloats>("nozzle_diameter");
        if (nozzles == nullptr || nozzles->values.empty() || !std::isfinite(nozzles->get_at(0)) || nozzles->get_at(0) <= 0.)
            throw std::invalid_argument("Tower calibration requires a positive nozzle diameter");
        if (recipe.generator == CalibrationGenerator::MaxVolumetricSpeed) {
            const auto *ratio = config.option<ConfigOptionFloats>("filament_flow_ratio");
            const auto *bed = config.option<ConfigOptionPoints>("printable_area");
            if (ratio == nullptr || ratio->values.empty() || !std::isfinite(ratio->get_at(0)) || ratio->get_at(0) <= 0.)
                throw std::invalid_argument("Volumetric-speed calibration requires a positive filament flow ratio");
            if (bed == nullptr || bed->values.size() < 3 || get_extents(bed->values).size().x() <= 10.)
                throw std::invalid_argument("Volumetric-speed calibration requires a printable bed wider than 10 mm");
        }
        Calib_Params runtime = params;
        runtime.mode = recipe.mode;
        if (continuous) {
            const auto flavor = config.opt_enum<GCodeFlavor>("gcode_flavor");
            if (flavor != gcfKlipper && flavor != gcfMarlinFirmware && flavor != gcfRepRapFirmware)
                throw std::invalid_argument("Cornering and input-shaping calibration require Klipper, Marlin 2 or RepRapFirmware");
            if (params.test_model < 0 || params.test_model > (cornering ? 2 : 1))
                throw std::invalid_argument("Invalid calibration model variant");
            if (cornering && params.end > (has_junction_deviation(&config) ? 0.3 : 100.))
                throw std::invalid_argument("Cornering range exceeds the GUI's firmware-specific limit");
            if (frequency || damping) {
                auto valid_frequency = [](double value) { return std::isfinite(value) && value >= 0. && value <= 500.; };
                if (!valid_frequency(params.freqStartX) || !valid_frequency(params.freqStartY) ||
                    (frequency && (!valid_frequency(params.freqEndX) || !valid_frequency(params.freqEndY) ||
                                   params.freqEndX <= params.freqStartX || params.freqEndY <= params.freqStartY)))
                    throw std::invalid_argument("Input-shaping calibration requires frequencies from 0 to 500 Hz and increasing frequency ramps");
                if ((frequency && params.start >= 1.) || (damping && params.end > 1.))
                    throw std::invalid_argument("Input-shaping damping values must lie between 0 and 1");
                if (flavor == gcfRepRapFirmware && (params.freqStartX != params.freqStartY ||
                    (frequency && params.freqEndX != params.freqEndY)))
                    throw std::invalid_argument("RepRapFirmware input shaping requires equal X and Y frequencies");
                const auto allowed = calibration_input_shaper_types(flavor);
                const auto &names = print_config_def.get("input_shaping_type")->enum_values;
                if (runtime.shaper_type.empty())
                    runtime.shaper_type = names.at(static_cast<size_t>(allowed.front()));
                const bool valid_type = std::any_of(allowed.begin(), allowed.end(), [&](InputShaperType type) {
                    return type != InputShaperType::Disable && names.at(static_cast<size_t>(type)) == runtime.shaper_type;
                });
                if (!valid_type)
                    throw std::invalid_argument("Input shaper is not available for the selected firmware");
            }
        }
        const CalibrationCut cut = [&model](double z, ModelObjectCutAttributes attributes) {
            cut_calibration_model(model, z, attributes);
        };
        switch (recipe.generator) {
        case CalibrationGenerator::PressureAdvance:
            if (runtime.mode == CalibMode::Calib_PA_Pattern) {
                for (const auto *values : {&params.speeds, &params.accelerations})
                    if (std::any_of(values->begin(), values->end(), [](double value) { return !std::isfinite(value) || value <= 0.; }))
                        throw std::invalid_argument("PA pattern speeds and accelerations must be positive finite values");
                configure_pressure_advance_calibration(config, config);
                const auto plan = configure_pressure_advance_pattern(config, config, config, [&config] { return config; }, runtime);
                const bool is_bbl = config.opt_string("printer_model").compare(0, 9, "Bambu Lab") == 0;
                auto layout = build_pressure_advance_pattern(plan, config, runtime, is_bbl);
                model = std::move(layout.model);
                return {runtime, std::move(layout.object_plates)};
            }
            configure_pressure_advance_calibration(config, config);
            if (runtime.mode == CalibMode::Calib_PA_Tower)
                return {apply_pressure_advance_tower_calibration(model, config, config, config, config, runtime, cut), {}};
            return {runtime, {}};
        case CalibrationGenerator::MaxVolumetricSpeed:
            return {apply_max_volumetric_speed_calibration(model, config, config, config, runtime, cut), {}};
        case CalibrationGenerator::Retraction:
            return {apply_retraction_calibration(model, config, config, config, runtime, cut), {}};
        case CalibrationGenerator::VFA:
            return {apply_vfa_calibration(model, config, config, config, runtime, cut), {}};
        case CalibrationGenerator::Temperature:
            return {apply_temperature_calibration(model, config, config, config, runtime, cut), {}};
        case CalibrationGenerator::FanTower:
            return {apply_fan_tower_calibration(model, config, config, config, runtime, cut), {}};
        case CalibrationGenerator::InputShapingFrequency:
            return {apply_input_shaping_frequency_calibration(model, config, config, config, runtime), {}};
        case CalibrationGenerator::InputShapingDamping:
            return {apply_input_shaping_damping_calibration(model, config, config, config, runtime), {}};
        case CalibrationGenerator::Cornering:
            return {apply_cornering_calibration(model, config, config, config, runtime), {}};
        default: throw std::invalid_argument("Native CLI calibration generation is unavailable for " + recipe.name);
        }
    }
    if (surface_pattern != "archimedeanchords" && surface_pattern != "monotonic")
        throw std::invalid_argument("Flow-ratio calibration pattern must be archimedeanchords or monotonic");
    const InfillPattern pattern = surface_pattern == "monotonic" ? ipMonotonic : ipArchimedeanChords;
    const FlowRatioCalibrationPlan plan = plan_flow_ratio_calibration(config, recipe.linear, recipe.pass);
    apply_flow_ratio_calibration(model.objects, config, plan, recipe.linear, recipe.pass, pattern);
    for (ModelObject *object : model.objects) {
        object->scale(plan.scale);
        object->ensure_on_bed();
    }
    config.set_key_value("resonance_avoidance", new ConfigOptionBool(false));
    return {};
}

} // namespace Slic3r
