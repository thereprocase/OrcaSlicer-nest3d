# Morphing spiral solid fill

`Morphing spiral (experimental)` is an opt-in top, bottom and internal solid
fill pattern. Select it in the process profile's surface-pattern controls, or
set `top_surface_pattern`, `bottom_surface_pattern` or
`internal_solid_infill_pattern` to `morphingspiral` in a CLI settings JSON.
Existing defaults and saved pattern identifiers are unchanged. The first layer
uses the bottom surface pattern.

## What it prints

An island with one hole, or a disk, is filled with one continuous variable-width
path. The island is described in polar form around a centre: the hole centroid,
or for a disk the deepest candidate point that sees the whole boundary. The
bead loops sit at even fractions between the outer edge and the hole (a pinhole
for a disk). Each bead's width is its local share of the thickness along the
boundary normal, so beads widen where the island is thick and thin where it is
narrow.

The path is a start taper on the outer loop, then each loop with a lane change
to the next, then an end taper on the innermost loop. All lane changes share
one seam zone, placed in the smoothest part of the boundary, so neighbouring
beads stay parallel and the island is covered without widening any bead. A disk
ends on its last loop, which already covers the pinhole. Loops are evaluated at
every polygon vertex angle, so they turn real corners.

The bead count is chosen inside the window where every width fits between 0.75
and 1.5 nozzle diameters (less the rounded-bead correction), as close as
possible to the requested line width. The requested width is not replaced by
the nozzle width.

## When it falls back

Rejected islands are filled with Spiral Inset, and the reason is logged at
warning level. The fallback cases:

- more than one hole
- a boundary that a ray from the centre crosses more than once
- a region too narrow for two beads
- a thickness ratio around the island larger than the width-bound ratio

The last case is a limit of any single spiral in which every ray crosses the
same number of loops, for example a wide ellipse or a triangle around a small
hole. Steep teeth also fail the width bounds with this radial
parameterisation; a field-based one is a possible follow-up.

## Checks

- Every bead outside the two tapers stays inside the nozzle width bounds after
  smoothing.
- Width changes along the path are rate-limited to 0.20 mm per mm. Where the
  geometry asks for more, at the centre of a lumpy disk for example, the widths
  are relaxed toward the limit.
- Taper tips thinner than 10% of the spacing are not printed.
- The swept footprint must pass the coverage audit:
  - uncovered and outside fractions each below 0.2%
  - connected gaps at most 0.25 mm², connected outside patches at most 0.10 mm²
  - inscribed-width limits on both
  - deposition within 0.5% of the island area

These are geometric checks of swept spacing, not predictions of molten plastic
height, bonding or surface finish. No physical print qualification is claimed.
Inspect the width/flow preview and use a small trial before applying it to a
finished part.

The offline polygon tool under `experiments/morphing_spiral` predates this
design. It produces geometry previews and raster coverage measurements of the
earlier endpoint approach, not printable G-code.
