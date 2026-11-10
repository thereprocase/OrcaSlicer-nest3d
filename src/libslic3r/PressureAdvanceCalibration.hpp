#pragma once

#include "PrintConfig.hpp"
#include "TowerCalibration.hpp"
#include "calib.hpp"

#include <string>

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

} // namespace Slic3r
