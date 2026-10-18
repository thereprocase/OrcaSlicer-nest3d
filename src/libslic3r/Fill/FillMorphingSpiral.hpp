#ifndef slic3r_FillMorphingSpiral_hpp_
#define slic3r_FillMorphingSpiral_hpp_
#include "FillSpiralInset.hpp"
#include <optional>
#include <string>
#include <vector>

namespace Slic3r {
// Geometry-only result, exposed to independent coverage/regression tests.
struct MorphingSpiralResult
{
    ThickPolylines paths;
    std::string fallback_reason;
    double uncovered_fraction    = 0.;
    double outside_fraction      = 0.;
    double excess_fraction       = 0.;
    double max_width_change_rate = 0.;
    double largest_gap_mm2       = 0.;
    double largest_spill_mm2     = 0.;
};
// One radial spiral with no U-turns; islands with at most one hole and a bounded thickness ratio.
MorphingSpiralResult morphing_spiral(const ExPolygon& bead_domain, double spacing_mm, double min_spacing_mm, double max_spacing_mm);
// Connected Fermat spiral (FillFermatSpiral.cpp): any island, holes included, still one path.
MorphingSpiralResult fermat_spiral(const ExPolygon& bead_domain, double spacing_mm, double min_spacing_mm, double max_spacing_mm);

namespace morphing_spiral_detail {
// Appends p (mm) with its width (mm) to a variable-width path.
void append_point(ThickPolyline& path, const Vec2d& p, double width_mm);
struct Coverage
{
    ExPolygons gaps;
    ExPolygons spills;
};
// Swept-footprint audit of paths over the domain: fills result's fractions and largest gap and
// spill, and returns the gaps and spills for inscribed-width checks. Nothing for an empty domain.
std::optional<Coverage> measure_coverage(const ExPolygon& domain, const ThickPolylines& paths, MorphingSpiralResult& result);
} // namespace morphing_spiral_detail

class FillMorphingSpiral : public FillSpiralInset
{
public:
    // Why islands of this fill fell back to Spiral Inset, one entry per island, so the layer can
    // report them as a slicing warning instead of only a log line.
    std::vector<std::string> fallback_reasons;

protected:
    Fill* clone() const override { return new FillMorphingSpiral(*this); }
    using FillSpiralInset::_fill_surface_single;
    void _fill_surface_single(const FillParams&, unsigned int, const std::pair<float, Point>&, ExPolygon, ThickPolylines&) override;
};
} // namespace Slic3r
#endif
