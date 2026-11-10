#pragma once

#include <string>
#include <vector>

namespace Slic3r {

class Model;
class DynamicPrintConfig;

enum class CalibrationGenerator {
    FlowRatio, PressureAdvance, Temperature, MaxVolumetricSpeed, Retraction, VFA,
    Cornering, InputShapingFrequency, InputShapingDamping, FanTower
};

struct CalibrationRecipe {
    std::string name;
    CalibrationGenerator generator;
    bool cli_supported;
    bool linear = false;
    int pass = 1;
};

// Catalogue of the GUI's model-generating calibration entrypoints. Unsupported CLI recipes are
// explicit, so automation cannot silently fall back to a superficially similar model or settings.
const std::vector<CalibrationRecipe> &calibration_recipes();
const CalibrationRecipe *find_calibration_recipe(const std::string &name);
std::string calibration_recipe_asset(const CalibrationRecipe &recipe);
void apply_calibration_recipe(Model &model, DynamicPrintConfig &config, const CalibrationRecipe &recipe);

} // namespace Slic3r
