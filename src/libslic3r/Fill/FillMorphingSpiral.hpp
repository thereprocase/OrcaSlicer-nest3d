#ifndef slic3r_FillMorphingSpiral_hpp_
#define slic3r_FillMorphingSpiral_hpp_
#include "FillSpiralInset.hpp"
#include <string>

namespace Slic3r {
// Geometry-only result, exposed to independent coverage/regression tests.
struct MorphingSpiralResult {
    ThickPolylines paths;
    std::string fallback_reason;
    double uncovered_fraction = 0.;
    double outside_fraction = 0.;
    double excess_fraction = 0.;
    double largest_gap_mm2 = 0.;
    double largest_spill_mm2 = 0.;
};
MorphingSpiralResult morphing_spiral(const ExPolygon& bead_domain, double spacing_mm,
                                   double min_spacing_mm, double max_spacing_mm);

class FillMorphingSpiral : public FillSpiralInset {
protected:
    Fill* clone() const override { return new FillMorphingSpiral(*this); }
    using FillSpiralInset::_fill_surface_single;
    void _fill_surface_single(const FillParams&, unsigned int, const std::pair<float, Point>&,
                              ExPolygon, ThickPolylines&) override;
};
}
#endif
