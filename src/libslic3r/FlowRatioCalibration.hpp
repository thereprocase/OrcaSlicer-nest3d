#pragma once

#include "Model.hpp"
#include "PrintConfig.hpp"
#include "Point.hpp"

#include <string>
#include <vector>

namespace Slic3r {

struct FlowRatioCalibrationPlan {
    double nozzle_diameter;
    double layer_height;
    double first_layer_height;
    double flow_ratio;
    Vec3d scale;
    std::vector<double> internal_solid_speeds;
    std::vector<double> top_surface_speeds;
};

// These recipes are shared by the Calibration menu and CLI. Invalid requests fail before mutating a model.
FlowRatioCalibrationPlan plan_flow_ratio_calibration(const DynamicPrintConfig &config, bool linear, int pass);
double flow_ratio_calibration_modifier(const std::string &object_name);
std::string flow_ratio_calibration_asset(bool linear, int pass);
void apply_flow_ratio_calibration(ModelObjectPtrs &objects, DynamicPrintConfig &print_config,
                                 const FlowRatioCalibrationPlan &plan, bool linear, int pass,
                                 InfillPattern pattern = ipArchimedeanChords);

} // namespace Slic3r
