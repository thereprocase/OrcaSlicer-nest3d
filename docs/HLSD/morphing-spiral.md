# Morphing spiral solid fill

`Morphing spiral (experimental)` is an opt-in top, bottom and internal solid
fill pattern. Select it in the process profile's surface-pattern controls, or
set `top_surface_pattern`, `bottom_surface_pattern` or
`internal_solid_infill_pattern` to `morphingspiral` in a CLI settings JSON.
Existing defaults and saved pattern identifiers are unchanged.

For a star-shaped region with exactly one hole, the generator interpolates
radial boundary samples around the hole centroid. It emits two boundary paths
and one continuous interior spiral with variable spacing. This is three
extrusion paths, not a promise of a travel-free layer. The ordinary extrusion
writer converts spacing to bead width and retains material flow limits,
acceleration, cooling and retraction behavior.

The generator checks nozzle width bounds, width change per distance, swept
footprint coverage and connected local gap/spill limits. Rejected islands use
Spiral Inset, with a debug-log reason. Disks, multiple holes, unsuitable radial
geometry and bridges currently use that fallback. The implementation does not
solve a harmonic field and cannot represent arbitrary outlines.

## Experimental limits

The first and last spiral turns overlap the boundary paths. Current synthetic
80 mm circular/eight-lobed annulus checks measure approximately 2.3–2.5% excess
swept-strip area; the acceptance ceiling is 3%. These area checks do not predict
local molten-plastic height or establish that the endpoints print cleanly.
Endpoint deposition remains an unresolved limitation of this preview build.
Inspect the width/flow preview and use a small trial before applying it to a
finished part. No physical print qualification is claimed.

The offline polygon tool under `experiments/morphing_spiral` produces geometry
previews and independent raster coverage measurements, not printable G-code.
Its widths describe path spacing; production bead width additionally includes
the layer-height-dependent rounded cross-section correction.
