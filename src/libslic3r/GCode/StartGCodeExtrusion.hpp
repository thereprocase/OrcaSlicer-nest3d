#pragma once

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"

#include <string>

namespace Slic3r {

class DynamicPrintConfig;

// The plate area the machine start G-code lays material on, such as a purge line, in plate coordinates
// (mm): every extruding G0/G1/G2/G3 move of the processed template as wide as its bead, shifted by the
// extruder offset as the G-code viewer shows it. `initial_filament` is the zero-based filament the print
// starts with. Placeholders that depend on the finished layout (first_layer_print_min, bed mesh bounds)
// read as the whole printable area. Empty when the start G-code extrudes nowhere, or when it cannot be
// evaluated before slicing; `error`, when given, then receives the reason.
ExPolygons start_gcode_extrusion_area(const DynamicPrintConfig &config, unsigned int initial_filament,
                                      std::string *error = nullptr);

// The extruding moves of already processed G-code as in start_gcode_extrusion_area(). The bead is never
// narrower than `min_width`; `filament_area` converts E to volume. Material extruded without moving
// counts only over `plate`, the printable area: beside it, it falls into a purge chute.
ExPolygons gcode_extrusion_area(const std::string &gcode, const Polygon &plate, const Vec2d &offset, double min_width,
                                double filament_area);

} // namespace Slic3r
