#pragma once

#include "libslic3r/Arrange.hpp"
#include "libslic3r/Point.hpp"
#include "Nest3D.hpp"

#include <vector>

namespace Slic3r {

class DynamicPrintConfig;

namespace nest3d {

// Z angles current + offset for every offset, normalised to [0, 2 pi) and sorted. Copies whose
// current angles differ by a multiple of an evenly spaced offset list get the same angles in the
// same order, so they can share a Part::shape_key.
std::vector<double> candidate_angles(double current, const std::vector<double> &offsets);

// Arranges `selected` around `fixed` like arrangement::arrange() does, with the same inputs
// and the same meaning of bed_idx, translation and rotation on output. Items must carry
// their ModelInstance to be arranged by shape; others are treated as prisms of their polygon.
// Parts laid on another face get a setter that also applies that orientation.
void arrange_polygons(arrangement::ArrangePolygons       &selected,
                      const arrangement::ArrangePolygons &fixed,
                      const Points                       &bed,
                      const arrangement::ArrangeParams   &params,
                      const ArrangeSettings              &settings,
                      const DynamicPrintConfig           &config,
                      int                                 max_plates);

}} // namespace Slic3r::nest3d
