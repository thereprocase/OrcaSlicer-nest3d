#pragma once

#include "CutUtils.hpp"
#include "PrintConfig.hpp"
#include "calib.hpp"

#include <functional>
#include <string>

namespace Slic3r {

// GUI supplies its scene-aware cut callback; CLI supplies the same Cut engine without UI updates.
using CalibrationCut = std::function<void(double, ModelObjectCutAttributes)>;
void cut_calibration_model(Model &model, double z, ModelObjectCutAttributes attributes);

Calib_Params apply_max_volumetric_speed_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                              DynamicPrintConfig &printer, const Calib_Params &params,
                                              const CalibrationCut &cut);
Calib_Params apply_retraction_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                              DynamicPrintConfig &printer, const Calib_Params &params,
                                              const CalibrationCut &cut);
Calib_Params apply_vfa_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                              DynamicPrintConfig &printer, const Calib_Params &params,
                                              const CalibrationCut &cut);

std::string tower_calibration_asset(CalibMode mode, int test_model = 0);
void apply_tower_blocks_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &printer,
                                    double bottom_z, double top_z, const Calib_Params &params,
                                    const CalibrationCut &cut);
Calib_Params apply_temperature_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                         DynamicPrintConfig &printer, const Calib_Params &params,
                                         const CalibrationCut &cut);
Calib_Params apply_fan_tower_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                       DynamicPrintConfig &printer, const Calib_Params &params,
                                       const CalibrationCut &cut);
Calib_Params apply_input_shaping_frequency_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                                 DynamicPrintConfig &printer, const Calib_Params &params);
Calib_Params apply_input_shaping_damping_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                                 DynamicPrintConfig &printer, const Calib_Params &params);
Calib_Params apply_cornering_calibration(Model &model, DynamicPrintConfig &print, DynamicPrintConfig &filament,
                                                 DynamicPrintConfig &printer, const Calib_Params &params);

} // namespace Slic3r
