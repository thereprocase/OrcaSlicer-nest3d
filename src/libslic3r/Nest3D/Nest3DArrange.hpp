#pragma once

#include "libslic3r/Arrange.hpp"
#include "libslic3r/Point.hpp"
#include "Nest3D.hpp"

namespace Slic3r {

class DynamicPrintConfig;

namespace nest3d {

// User-facing switches of the shape-true arranger (see docs/HLSD/nest3d.md).
struct ArrangeSettings
{
    bool   enabled         = false;
    bool   allow_stacking  = false; // let short parts sit under unsupported overhangs
    double vertical_gap_mm = 2.;
    bool   reorient        = false; // also try lay-flat orientations
    int    rotation_steps  = 8;     // Z rotations tried when rotation is allowed
    int    effort          = 1;     // 0 fast, 1 balanced, 2 thorough
};

// Arranges `selected` around `fixed` like arrangement::arrange() does, with the same inputs
// and the same meaning of bed_idx, translation and rotation on output. Items must carry
// their ModelInstance to be arranged by shape; others are treated as prisms of their polygon.
// Parts laid on another face get a setter that also applies that orientation.
Stats arrange_polygons(arrangement::ArrangePolygons       &selected,
                       const arrangement::ArrangePolygons &fixed,
                       const Points                       &bed,
                       const arrangement::ArrangeParams   &params,
                       const ArrangeSettings              &settings,
                       const DynamicPrintConfig           &config,
                       int                                 max_plates);

}} // namespace Slic3r::nest3d
