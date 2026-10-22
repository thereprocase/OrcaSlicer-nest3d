# Experimental staggered perimeters

The `staggered_perimeters` process option raises alternate Arachne wall insets
by half a layer. It is off by default and appears under Strength / Walls.

After perimeter generation, the planner matches closed inner walls across
adjacent layers using their exact XY segments and widths. Matching ignores the
start vertex and traversal direction. A track must repeat for at least four
layers at constant height. Changed widths, slopes, open walls, missing layers
and ambiguous coincident walls break the track. These walls use ordinary paths.
Exact matching is deliberately conservative: even equivalent curves with
different segmentation may not match.

Each track begins at nominal Z. Its next bead rises half a layer, making that
bead 1.5 layers tall. Subsequent raised beads retain ordinary height. The
penultimate bead returns to nominal Z with half height, and the final bead is
ordinary. This applies independently to local starts and tops, rather than
using the object's overall height. Bead volume uses the rounded-rectangle
cross section at its actual height; stored height and volume follow the path
through seam splitting and feed the existing volumetric speed calculation.

Nominal walls precede raised walls within each island. Travels in a print with
staggering lift above the maximum possible staggered layer height plus 0.2 mm,
follow the planned XY route, then descend at the destination. A move exceeding
the configured printable height is rejected.

## Supported combinations

Use Arachne and constant layer height. Scarf seams, Z contouring, adaptive
layer-height profiles, rafts, spiral vase, belt printing and mixed sublayers
are rejected. The initial tall bead must not exceed 80% of the smallest
configured nozzle diameter, or its own width. Thus a 0.4 mm nozzle with
0.2 mm layers can produce the 0.3 mm transition bead.

An object with no qualifying tracks produces a slicing warning and retains
ordinary walls. An object with some qualifying tracks may contain both kinds.
No settings are silently changed to enable the feature.

Scarf seams and staggering both alter extrusion height along a path, but
composing them requires a shared height and volume calculation. They are not
combined in this experiment. Support-interface geometry, detailed nozzle-body
clearance, physical adhesion and strength still need separate evaluation;
passing software tests does not establish those properties.

## Tests

Planner tests cover local starts and stops, short tracks, changed geometry,
missing layers, reversed paths, duplicate tracks, bead height and volume, and
seam splitting. Print-level tests cover stepped objects, disabled behavior,
incompatible settings and lift / XY / descent ordering in emitted G-code.

Native Linux Release validation on 2026-10-21 passed 17 staggered-perimeter
cases (999 assertions) and 23 existing perimeter/extrusion cases (6,260
assertions). The focused executable links the full headless slicing library.
This was not a Windows GUI build or a physical print test.

## Provenance

Initially ported from [OrcaSlicer PR #8181](https://github.com/OrcaSlicer/OrcaSlicer/pull/8181),
head `f3b0c40d703c2ba2f8df77c0e3df41c2b1eeaebd`, onto personal `repro-dev`
`4c0d6e61c5`. Original implementation by Vipul Rajan and contributors.
The experiment replaces its generator-time staggering with the closed-wall
track planner described above.
