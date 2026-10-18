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
    int beads                    = 0; // loops of a radial spiral
    // Before slivers under 0.05 mm are opened away (Fermat and band audits only).
    double raw_uncovered_fraction = 0.;
    double raw_largest_gap_mm2    = 0.;
};
// One radial spiral with no U-turns; islands with at most one hole and a bounded thickness ratio.
MorphingSpiralResult morphing_spiral(const ExPolygon& bead_domain, double spacing_mm, double min_spacing_mm, double max_spacing_mm);
// Connected Fermat spiral (FillFermatSpiral.cpp): any island, holes included, still one path.
MorphingSpiralResult fermat_spiral(const ExPolygon& bead_domain, double spacing_mm, double min_spacing_mm, double max_spacing_mm);
// Accordion spiral (FillFermatSpiral.cpp), for an island with one hole that one radial spiral cannot
// fill: outline rings, one middle loop and hole rings as one inward spiral. Where the middle strip
// is wider than a bead (the ends of a rectangle round a square hole) the middle loop folds back and
// forth across it instead of running along it.
MorphingSpiralResult accordion_spiral(const ExPolygon& bead_domain, double spacing_mm, double min_spacing_mm, double max_spacing_mm);

namespace morphing_spiral_detail {
// The island in polar form around its hole centroid (or, for a disk, a centre that sees the whole
// boundary): edge radii and their slopes at polar_angle(k) for k < polar_samples().
struct PolarFrame
{
    Vec2d center;
    bool disk = false;
    std::vector<double> outer, outer_slope, inner, inner_slope;
};
int polar_samples();
double polar_angle(int k);
std::optional<PolarFrame> polar_frame(const ExPolygon& domain, double spacing_mm, std::string& reason);
// The radial spiral; with seam_allowed (one entry per polar sample) its lane-change zone stays
// inside the allowed angles.
MorphingSpiralResult radial_spiral(const ExPolygon& domain, double spacing_mm, double min_spacing_mm, double max_spacing_mm,
                                   const std::vector<char>* seam_allowed);

// Appends p (mm) with its width (mm) to a variable-width path.
void append_point(ThickPolyline& path, const Vec2d& p, double width_mm);
// Limits |dw/ds| along a path to limit (mm per mm); segment[i] is the length from point i to i + 1.
void limit_width_change(std::vector<double>& widths, const std::vector<double>& segment, double limit);
// The width change limit both fills enforce, in mm of width per mm of path.
constexpr double max_width_change = .20;
struct Coverage
{
    ExPolygons gaps;
    ExPolygons spills;
};
// Swept-footprint audit of paths over the domain: fills result's fractions and largest gap and
// spill, and returns the gaps and spills for inscribed-width checks. Nothing for an empty domain.
// per_segment sweeps each segment on its own, which stays exact through tight U-turns.
std::optional<Coverage> measure_coverage(const ExPolygon& domain, const ThickPolylines& paths, MorphingSpiralResult& result,
                                         bool per_segment = false);
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
