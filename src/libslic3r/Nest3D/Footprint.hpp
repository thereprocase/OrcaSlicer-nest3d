#pragma once

// What an object prints on the first layer around its geometry: Orca's own brim and the skirt it will
// be given. Nest3DArrange adds these to each part's footprint so the arrangement reserves the room they
// take (docs/HLSD/nest3d.md).

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"

#include <functional>

namespace Slic3r {

class DynamicPrintConfig;
class ModelInstance;
class ModelObject;
class Print;

namespace nest3d {

// The skirt settings as the print makes them.
struct SkirtPlan
{
    bool   enabled    = false;
    bool   per_object = false; // each object gets its own skirt; otherwise one skirt encloses the plate
    double distance   = 0.;    // from the outermost brim to the first loop, mm
    double extent     = 0.;    // from the outermost brim to the outer edge of the last loop, mm
};

SkirtPlan skirt_plan(const DynamicPrintConfig &config);

// Whether the object prints a brim, with the per-object overrides on top of `config`.
bool prints_brim(ModelObject &object, const DynamicPrintConfig &config);

// Fills `print` with one copy of `instance` posed by `transform` (its translation is dropped, and the
// lowest point rests on the bed) on a bed too large to clip anything, with the slicer's settings from
// `config`. Nothing is checked against the real bed; the arrangement does that.
void apply_scratch_print(Print &print, const ModelInstance &instance, const Transform3d &transform,
                         const DynamicPrintConfig &config);

// An object's first-layer brim as Orca generates it (brim type, width, ears, gap, auto widths and the
// object's overrides), in the frame of the pose it was made for, where the object's own origin is (0, 0).
struct ObjectBrim
{
    ExPolygons areas;
    double     reach = 0.; // the farthest the brim extends beyond the first layer along an axis, mm
};

// Slices the object in `pose` (the instance transform without its offset) and runs the brim generator
// on the first layer. Throws CanceledException when `canceled` says so between steps.
ObjectBrim object_brim(const ModelInstance &instance, const Transform3d &pose, const DynamicPrintConfig &config,
                       const std::function<bool()> &canceled);

}} // namespace Slic3r::nest3d
