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

## Islands one radial spiral cannot fill

When the radial spiral rejects an island, two other single-path methods try, and the fill keeps
the one that leaves less of the island uncovered, counting half its deposition error.

- Radial band with Fermat pockets, for an island with at most one hole: the radial spiral is capped
  around the hole at the thickness its bead count can cover, so the narrow sides stay one clean
  morphing spiral. The parts beyond the cap (a square's corners, a star's arms) are pockets, each
  filled by a Fermat spiral spliced into the band's outer loop. The band takes everything up to two
  beads past its cap and steps down over a short smoothstep, so no pocket starts as a wedge thinner
  than two beads.
- Connected Fermat spiral (after Zhao et al., "Connected Fermat Spirals for Layered Fabrication",
  SIGGRAPH 2016), for any island, holes included. The island is offset into rings, each opened by
  half a spacing so that every ring end is a U-turn of full track width rather than a pinched
  hairpin. Chains of nested rings become spirals: the outermost is a plain inward spiral, the others
  go in on alternate rings and come back out on the rest, and each is spliced into a neighbour with
  a U-turn in and a two-lane smoothstep out. Where rings of different families meet, the leftover
  strip is shared by every bead outward along the same line, so widths change gradually.

Both are audited against a looser budget than the radial spiral, because every ring end and splice
leaves a small gap: largest gap 0.45 mm2, no gap wider than half a spacing, uncovered at most 1.2%,
spill as for the radial spiral, deposition within 3% of the island area.

A known limit: a band only a few beads wide that needs an odd bead count (three beads across a
1.4 mm side, for example) cannot be met by two ring families, so such islands fall back.

## When it falls back

Islands that none of the three methods can fill are filled with Spiral Inset.
Each object then gets one non-critical slicing warning that gives the number of
islands, the layer range and the reasons with counts, for example "Morphing
spiral used Spiral Inset for 3 island(s) on layers 1-3 of frame: bead footprint
coverage or deposition budget failed (3);". The reason given is the Fermat
spiral's, the last method to try.

Where the warning appears:

- In the GUI, in the slicing notifications.
- In a CLI run, as class `slicing_warning_non_critical` in the `warnings` array
  of `result.json`. The CLI writes `result.json` on Linux only. Under `--strict`
  a fallback ends the run non-zero, like any non-critical warning.
- Each fallback is also logged at warning level. On Windows and macOS the CLI
  shows it only in the log, with `--logfile PATH`.

Islands deliberately skipped are not reported, because they are never spiral
candidates: sparse density, `dont_adjust` and bridges. The usual fallback
cases:

- an island smaller than one ring
- a narrow band that needs an odd bead count, as above
- coverage outside the Fermat budget
- a region that cannot be spliced to its neighbours (no stretch of three
  spacings where their rings run side by side)

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
