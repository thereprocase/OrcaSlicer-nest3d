#pragma once

#include "libslic3r/Arrange.hpp"
#include "libslic3r/Point.hpp"

#include <vector>

namespace Slic3r {

class DynamicPrintConfig;

namespace nest3d {

// The arrange dialog's switches for shape-true nesting.
struct ArrangeSettings
{
    bool  enabled         = false;
    bool  allow_stacking  = false; // let short parts sit under unsupported overhangs
    float vertical_gap_mm = 2.f;   // clearance under an overhang for the nozzle and fan duct (docs/HLSD/nest3d.md)
    bool  generate_supports = false; // slice each candidate first and reserve its generated support footprint
};

// Z angles current + offset for every offset, in [0, 2 pi) and sorted, so copies an earlier arrange left
// a whole number of steps apart are offered the same angles in the same order and pack as copies.
std::vector<double> candidate_angles(double current, const std::vector<double> &offsets);

// Arranges `selected` around `fixed` like arrangement::arrange(), with the same inputs and the same
// meaning of bed_idx, translation and rotation on output. Without generated supports, the outline
// arranger guards against using more plates or leaving more parts out. Its model-only footprints are
// not a safe fallback when generate_supports is enabled.
void arrange_polygons(arrangement::ArrangePolygons       &selected,
                      const arrangement::ArrangePolygons &fixed,
                      const Points                       &bed,
                      const arrangement::ArrangeParams   &params,
                      const ArrangeSettings              &settings,
                      const DynamicPrintConfig           &config,
                      int                                 max_plates);

}} // namespace Slic3r::nest3d
