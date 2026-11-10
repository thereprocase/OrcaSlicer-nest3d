#pragma once

#include "PrintConfig.hpp"
#include "Model.hpp"
#include "CustomGCode.hpp"
#include "Point.hpp"
#include "TowerCalibration.hpp"
#include "calib.hpp"

#include <string>
#include <functional>
#include <cstddef>
#include <vector>

namespace Slic3r {

std::string pressure_advance_calibration_asset(CalibMode mode);
void configure_pressure_advance_calibration(DynamicPrintConfig &print, DynamicPrintConfig &printer);
void set_pressure_advance_calibration_speed(DynamicPrintConfig &print, const DynamicPrintConfig &filament,
                                          const DynamicPrintConfig &printer, const DynamicPrintConfig &project,
                                          const std::string &key);
Calib_Params apply_pressure_advance_tower_calibration(Model &model, DynamicPrintConfig &print,
                                                    DynamicPrintConfig &filament, DynamicPrintConfig &printer,
                                                    const DynamicPrintConfig &project, const Calib_Params &params,
                                                    const CalibrationCut &cut);

struct PressureAdvancePatternPlan {
    std::vector<double> speeds;
    std::vector<double> accelerations;
    bool default_speed_used = false;
    bool default_acceleration_used = false;
};
struct PressureAdvancePatternLayout {
    Model model;
    std::vector<size_t> object_plates;
};
Model make_pressure_advance_pattern_seed();
PressureAdvancePatternPlan configure_pressure_advance_pattern(DynamicPrintConfig &print,
    DynamicPrintConfig &filament, DynamicPrintConfig &printer,
    const std::function<DynamicPrintConfig()> &effective_config, const Calib_Params &params);
PressureAdvancePatternLayout build_pressure_advance_pattern(const PressureAdvancePatternPlan &plan,
    const DynamicPrintConfig &full_config, const Calib_Params &params, bool is_bbl_machine);
CustomGCode::Info generate_pressure_advance_pattern_gcode(CalibPressureAdvancePattern &pattern,
    const ModelObjectPtrs &objects, const DynamicPrintConfig &config, bool is_bbl_machine, const Vec3d &origin);

} // namespace Slic3r
