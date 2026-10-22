# Experimental staggered perimeters

The `staggered_perimeters` process option raises alternate Arachne wall insets
by half a layer. It is off by default and appears under Strength / Walls.
Even insets retain nominal Z and precede odd insets, with stable ordering
within each group. Classic walls do not implement this feature.

For objects with at least four layers, the first and final layers remain at
nominal Z. The first raised bead uses 1.5 times the nominal extrusion; the
penultimate layer returns to nominal Z with half extrusion. Intermediate
raised beads use nominal extrusion. Shorter objects are not raised.
Extrusion paths carry the Z fraction and extrusion multiplier through copies
and moves. A settings change invalidates perimeter generation.

The GUI offers to select Arachne, disable spiral vase, match first-layer and
regular heights, and match top-surface and outer-wall widths. This does not
validate every combination in the CLI or a project with per-object overrides.

## Experimental limits

Use constant layer height. Adaptive layers, rafts, scarf seams, contoured
paths, sloping walls, changes in wall count, local tops, and internal holes
need dedicated slicing checks before use. The upstream implementation also
reports fragmented layer previews and issues with one-wall first layers.
The endpoint logic is object-wide, not a local surface-height calculation.
The extrusion multiplier requires a volumetric-flow-limit audit.

Compiler checks are not evidence of collision-free travel or stronger parts.
Compare generated Z moves and extrusion volumes against an ordinary-wall
control before printing strength coupons. In particular, travel between raised
and nominal walls must be checked for crossing already printed raised beads.

## Provenance

Ported from [OrcaSlicer PR #8181](https://github.com/OrcaSlicer/OrcaSlicer/pull/8181),
head `f3b0c40d703c2ba2f8df77c0e3df41c2b1eeaebd`, onto personal `repro-dev`
`4c0d6e61c5`. Original implementation by Vipul Rajan and contributors.
The port preserves current contouring and spiral-vase behavior, fixes the
non-strict sorting comparator and pointer comparison in settings validation,
and leaves the feature disabled by default.
