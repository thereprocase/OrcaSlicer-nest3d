# Morphing spiral solid fill

`Morphing spiral (experimental)` is an opt-in top, bottom and internal solid
fill pattern. Select it in the process profile's surface-pattern controls, or
set `top_surface_pattern`, `bottom_surface_pattern` or
`internal_solid_infill_pattern` to `morphingspiral` in a CLI settings JSON.
Existing defaults and saved pattern identifiers are unchanged.

For a star-shaped region with exactly one hole, the generator interpolates
radial boundary samples around the hole centroid. It emits two boundary paths,
one continuous interior spiral and two supplemental tapered gap arcs. This is five
extrusion paths, not a promise of a travel-free layer. The ordinary extrusion
writer converts spacing to bead width and retains material flow limits,
acceleration, cooling and retraction behavior.

The generator checks nozzle width bounds, width change per distance, swept
footprint coverage and connected local gap/spill limits. Rejected islands use
Spiral Inset, with a debug-log reason. Disks, multiple holes, unsuitable radial
geometry and bridges currently use that fallback. The implementation does not
solve a harmonic field and cannot represent arbitrary outlines.

## Experimental limits

The overlapping first and last turns are removed. Two tapered arcs cover the
resulting crescent gaps between the main spiral and its boundary paths. Near a
thin arc tip, the boundary bead widens to absorb the residual until the arc
can take over. The handover spans nine target spacings. Arc tips below 10% of
the target spacing are omitted; the resulting void still has to pass the
local area and width gates. Narrow tips remain a physical-print limitation.

Gap arcs remain last for both surface fill orders, including bottom surfaces.
The extrusion collection prohibits path sorting, so the later planner retains
this sequence. Main and boundary paths retain nozzle width bounds; gap arcs
are permitted to taper below the main-path minimum. Every path has a maximum
width-change rate of 0.20 mm/mm.

The geometric deposition-budget ceiling is 0.5% excess or deficit; uncovered
and outside footprint fractions must each remain below 0.2%. Connected gaps
must be at most 0.25 mm², outside patches at most 0.10 mm²; inscribed-width
checks also apply. These are swept-spacing checks, not predictions of molten
plastic height, bonding or surface finish. No physical print qualification is
claimed. Inspect the width/flow preview and use a small trial before applying
it to a finished part.

The offline polygon tool under `experiments/morphing_spiral` produces geometry
previews and independent raster coverage measurements, not printable G-code.
It represents the initial uncorrected endpoint approach, not the native tapered-arc revision. Its widths describe path spacing; production bead width additionally includes
the layer-height-dependent rounded cross-section correction.
