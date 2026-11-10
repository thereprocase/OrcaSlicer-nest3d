#include "CalibrationRecipe.hpp"
#include "FlowRatioCalibration.hpp"
#include "Model.hpp"
#include "PrintConfig.hpp"

#include <algorithm>
#include <stdexcept>

namespace Slic3r {

const std::vector<CalibrationRecipe> &calibration_recipes()
{
    static const std::vector<CalibrationRecipe> recipes {
        {"flow-ratio", CalibrationGenerator::FlowRatio, true, false, 1},
        {"flow-ratio-fine", CalibrationGenerator::FlowRatio, true, false, 2},
        {"flow-ratio-yolo", CalibrationGenerator::FlowRatio, true, true, 1},
        {"flow-ratio-yolo-fine", CalibrationGenerator::FlowRatio, true, true, 2},
        {"pressure-advance-line", CalibrationGenerator::PressureAdvance, false},
        {"pressure-advance-pattern", CalibrationGenerator::PressureAdvance, false},
        {"pressure-advance-tower", CalibrationGenerator::PressureAdvance, false},
        {"temperature", CalibrationGenerator::Temperature, false},
        {"max-volumetric-speed", CalibrationGenerator::MaxVolumetricSpeed, false},
        {"retraction", CalibrationGenerator::Retraction, false},
        {"vfa", CalibrationGenerator::VFA, false},
        {"cornering", CalibrationGenerator::Cornering, false},
        {"input-shaping-frequency", CalibrationGenerator::InputShapingFrequency, false},
        {"input-shaping-damping", CalibrationGenerator::InputShapingDamping, false},
        {"overhang-fan", CalibrationGenerator::FanTower, false},
        {"part-fan", CalibrationGenerator::FanTower, false},
        {"bridge-fan", CalibrationGenerator::FanTower, false},
        {"small-feature-fan", CalibrationGenerator::FanTower, false},
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

std::string calibration_recipe_asset(const CalibrationRecipe &recipe)
{
    if (recipe.cli_supported && recipe.generator == CalibrationGenerator::FlowRatio)
        return flow_ratio_calibration_asset(recipe.linear, recipe.pass);
    throw std::invalid_argument("Native CLI calibration generation is unavailable for " + recipe.name);
}

void apply_calibration_recipe(Model &model, DynamicPrintConfig &config, const CalibrationRecipe &recipe)
{
    // Keep dispatch explicit: adding an advertised recipe must also implement its real generator.
    if (!recipe.cli_supported || recipe.generator != CalibrationGenerator::FlowRatio)
        throw std::invalid_argument("Native CLI calibration generation is unavailable for " + recipe.name);
    if (config.opt_bool("belt_printer"))
        throw std::invalid_argument("CLI flow-ratio calibration currently requires a flat-bed printer");
    const FlowRatioCalibrationPlan plan = plan_flow_ratio_calibration(config, recipe.linear, recipe.pass);
    apply_flow_ratio_calibration(model.objects, config, plan, recipe.linear, recipe.pass);
    for (ModelObject *object : model.objects) {
        object->scale(plan.scale);
        object->ensure_on_bed();
    }
    config.set_key_value("resonance_avoidance", new ConfigOptionBool(false));
}

} // namespace Slic3r
