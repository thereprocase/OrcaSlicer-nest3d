#pragma once

namespace Slic3r {

// The nozzle shape assumed when printing beside material that stands taller than the nozzle tip
// (batched support interface prints some lines below their already printed neighbours): a flat
// tip of radius 1.25 x nozzle diameter under a 45 degree cone. Only the lowest nozzle_height of
// the hotend is predictable; above it sit the heater block and the silicone sock.
struct NozzleClearance
{
    // Kept between the nozzle and taller material on top of the geometric clearance.
    static constexpr double margin = 0.15;

    double tip_radius  = 0.;
    double cone_height = 0.;

    // Tallest step beside the tip that stays within the predictable cone.
    double max_lead() const { return cone_height - margin; }
    // Smallest distance from the nozzle centre to the edge of material standing `delta` above the
    // tip. The 45 degree flank rises one unit per unit of distance beyond the flat tip.
    double required_distance(double delta) const { return tip_radius + delta + margin; }
    // Gap between a line of `line_width` and material `delta` taller that keeps the nozzle clear:
    // the nozzle centre runs half a line width inside the line's edge.
    double required_gap(double delta, double line_width) const { return required_distance(delta) - 0.5 * line_width; }
};

inline NozzleClearance nozzle_clearance(double nozzle_diameter, double nozzle_height)
{
    return { 1.25 * nozzle_diameter, nozzle_height };
}

} // namespace Slic3r
