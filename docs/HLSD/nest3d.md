# Nest3D shape-true arrange — High Level Design

## Purpose and scope

The polygon arranger packs the 2D convex hull of every part, so a part never enters another
part's hole, notch or concave side, and nothing ever sits under an overhang. Nest3D arranges
parts by their real 3D shape instead. It runs in place of the polygon arranger when the user
turns on "Nest by 3D shape" in the arrange options, where it applies (see Gating). The option is
off by default; with it off, the polygon arranger runs.

The engine (`src/libslic3r/Nest3D/Nest3D.{hpp,cpp}`, `Grid`, `Search`, `Pockets` and `Lattice`) uses
no libslic3r types, so tests can run it alone. Policy that needs the model (poses,
Z angles, spacing, copies, compatibility groups) belongs to the adapter
(`Nest3DArrange.{hpp,cpp}`), which turns `ArrangePolygon`s into engine shapes, parts and obstacles
and writes the result back into the same fields the polygon arranger fills, so
`ArrangeJob::finalize` and the plate bookkeeping work unchanged.

## Model

Every part is a column grid, the *height map*: one cell per `cell_mm` square of the bed, holding
the z-interval `[lo, hi]` the part occupies above that square. A cell's interval is the union of
the part's triangles crossing it, so a part with two levels above one cell keeps the gap between
them filled; the model only ever over-estimates the space a part takes.

- **Cells.** 0.5 mm, or the longest bed side over 720 on larger beds, which bounds the cost of a
  scan. Cells of 0.5 mm divide whole and half millimetres, so an axis-parallel edge at such a size
  and spacing falls on a cell boundary and keeps its spacing exactly.
- **Spacing.** Every map is grown in XY by the item's inflation, the same value the polygon
  arranger uses (the user's spacing, or the brim and support allowance when spacing is automatic).
  Growth claims every cell that comes closer than the inflation to the part, so two parts whose
  maps share no cell keep at least the sum of their inflations apart. Distances are measured from
  the extent of the geometry inside each occupied cell along the axes and the diagonals, which is
  exact for edges in those directions, so the clearance is not rounded up to whole cells. A cell
  whose eight neighbours are filled and hold its interval claims nothing they do not, and is
  skipped, which keeps the 12 mm growth of tree supports near the cost of the outline.
- **Support.** A part that prints with support (`enable_support`) has every column filled down to
  the bed, because the support below it occupies that space.
- **Fixed items.** Unselected instances are rasterized from their meshes in their current pose.
  The wipe tower and exclusion areas are full-height columns over their polygons. On a bed that is
  not a rectangle, the cells whose centre lies outside its outline are a full-height obstacle on
  every plate.

Two parts may share a cell only when their intervals are at least the vertical clearance apart.
With "Nest under overhangs" off, any occupied cell blocks its whole column, which still lets parts
interlock in plan view (one part in another's hole or notch) but never stack. One collision test,
`Board::fits`, decides every placement, pocket, lattice no-fit set and recentring.

## Physical rules for stacking

A part may only be placed in empty space under another part, which by the model means under an
overhang printed without support: supported parts fill their columns to the bed. The parts under
it are shorter than the overhang by at least the vertical clearance (2 mm), so the nozzle and the
fan duct pass over them while printing the overhang's first layers.

## Candidates

Each part offers its current Z angle, or with "Auto rotate" eight angles over a full turn (or the
item's `allowed_rotations` when it lists more than one). The angles are taken modulo a full turn
and sorted, so copies that an earlier arrange left a quarter turn apart are offered the same angles
and pack as copies. The pose is the one `ModelInstance::get_arrange_polygon` uses, the instance
transform without its offset and Z angle, so a placement's rotation and translation mean what they
mean for the polygon arranger. Instances of one object posed alike, with the same angles and
spacing, share one shape: they are rasterized once and are copies to the engine.

## Algorithm

Placement is greedy first fit. Parts are ordered, higher priority first, then by a size key, and
each part goes onto the first open plate of its compatibility group that holds it; a part that
fits nowhere opens the next plate, up to the plate limit, and is otherwise reported unplaced.

- **Pockets.** A part first tries the free regions the pile encloses: the 4-connected regions of
  free cells inside the bounding box of the parts placed so far that do not reach its border, such
  as the hole of a ring or the bay of a C-clip closed by a neighbour. It goes into the smallest one
  that holds it, in the pose with the fewest cells, at the first fit in raster order, which leaves
  the rest of the pocket in one piece. Contact plays no part here: the staircase edge of a part
  turned 45 degrees touches more cells than a straight one, and preferring it turns small parts
  askew in round holes, where fewer of them fit. With stacking, a cell with room under its occupant
  counts as free, so a ring's hole under a table top is still a pocket. Regions are labelled by a
  union-find over the free runs of each row.
- **Envelope.** Otherwise the pile grows from an anchor, the point the polygon arranger aligns its
  pile to (`best_object_pos`, the bed centre by default). A position scores the growth envelope of
  the pile and the part minus a fixed area (16 mm² per mm) of the part's outline that touches an
  occupied cell or the bed edge, so contact picks between positions of equal area. The *box*
  envelope is the bounding-box area; the *ring* envelope is the smallest bed-proportioned rectangle
  centred on the pile's height-weighted centroid that holds pile and part, which grows piles round
  instead of into strips or corners. Remaining ties go to the position nearest the envelope's
  centre.
- **Search.** Both envelopes are convex along each axis, so rows and columns are scanned outward
  from where the envelope is least, and a side stops once a lower bound on its score loses. Contact
  is at most the outline length, and at most the occupied cells around the part's rectangle, which
  a summed-area table gives in O(1). A part is a few runs of cells per row; a run collides when the
  prefix count of its board row shows an occupied cell under it, and it also collides at the next
  positions while those cells stay under it, so one lookup usually rules out several positions.
  The search is exact: pruning never changes the result. Candidates and row bands are parallel
  tasks reduced in task order, so the result does not depend on scheduling.
- **Order.** The size keys are the cells on the floor (with stacking, a table that leaves room
  under it comes after the parts that could go there), the height, the longer side, the volume, the
  sum of the sides and the bounding-box area. Ties fall to the other keys, then the shape and the
  id, so the order of the selection does not change the result.
- **Copies.** A shape with copies gets lattice units: up to four of its candidates that fill their
  bounding rectangle best, and the tightest pair of one of them with itself or its half turn (the
  touching offset with the smallest common bounding box, then the most contact). The raster no-fit
  set of a unit against itself yields its densest collision-free lattice, from pairs of offsets
  where a translate just touches; the unit's bounding-box lattice is always offered beside it,
  because on a finite bed a looser lattice can hold more copies. When the first copy of a shape
  reaches a plate in the lattice pass, its copies are laid on the lattice phase that holds them all
  in the smallest pile, or failing that the most of them: first the phase that centres a site on
  the anchor, and when that falls short, every phase, counted in one pass over the bed in which each
  cell where a copy fits votes for its phase. Copies take the sites that keep the pile roundest
  about the anchor, then smallest.
- **Passes.** A box pass runs for each distinct ordering, a ring pass tallest first, and a lattice
  pass when there are copies, in parallel. When they leave parts unplaced or need several plates, a
  last pass grows the pile from the bed's min corner, for parts that only fit pushed into opposite
  corners. Each pass's piles are then moved as one onto the anchor, halving the shift until every
  part clears the obstacles, the bed outline and the fixed parts of its plate.
- **Selection.** The best passes have the fewest unplaced parts, then the fewest plates, then the
  most parts on the first plate, then on the second, and so on. Among them, the one whose parts lie
  nearest the anchor, by the second moment of their areas weighted by 1 + height / tallest, wins
  if it is within 5% (`CentreAllowance`) of the smallest total bounding-box area; the earliest pass
  wins a tie. Taller parts therefore end nearer the centre when all else is equal, which keeps
  travel moves over them short.

Parts that the polygon arranger would refuse to put together never share a plate: high- and
low-temperature filaments are kept apart when the plate's filaments are not compatible, and with
"Allow multiple materials on same plate" off, every distinct filament set is a group of its own. A
plate holding fixed parts takes their group.

The result never uses more plates or leaves more parts unplaced than the polygon arranger: when it
does not fit one plate, the adapter runs the polygon arranger on the same input and keeps the
better of the two. The polygon arranger packs every item's convex hull without overlap, so it
needs at least as many plates as the hulls' total area fills beds; a result already there is kept
without running it. A cancelled arrange keeps the parts it placed and runs no fallback: under the
same stop condition the polygon arranger would return every part on top of the others.

## Complexity

Every step is polynomial. With `O` passes (at most 6 orderings plus a ring, a lattice and a corner
pass), `N` parts, `L` open plates, `C` candidates per part and `P` grid positions per plate, the
search costs `O(O * N * L * C * P)` run lookups in the worst case, and far fewer where the
envelope bound ends a side; a position that survives costs a collision and a contact test, 64
cells at a time. Keeping the plate's summed-area table current costs `O(P)` per placed part, and
labelling its pockets `O(P + R)` for `R` free runs in the pile's bounding box. Rasterizing
costs `O(C * (T + K))` per shape of `T` triangles crossing `K` cells, plus `O(B * g)` cells per
growth of `g` cells for `B` cells near the outline. A shape with copies costs a bounded number of
raster no-fit sets and lattice searches over its units' cells; the lattice search tries the 2048
shortest touching offsets, and phase counting reads each bed cell once per unit and lattice.
Cancelling is polled between parts.

## Gating

`ArrangeJob::process` uses Nest3D only when it is enabled and the arrange is not by-object, not on
a belt printer, and no IDEX/IQEX zone is active. By-object printing finishes every object before
the next starts, so the toolhead would sweep through finished parts nested around it; belt
printers and IDEX/IQEX zones place parts by rules of their own. The arrange menu offers the two
options, "Nest by 3D shape" and "Nest under overhangs", only for layer-by-layer FFF printing on a
flat bed, and stores them in the `arrange` section of the app config as `nest3d_enabled_fff` and
`nest3d_stacking_fff`.

## Testing

The `[Nest3D]` cases in `tests/libslic3r/test_nest3d.cpp` drive the adapter the arrange job uses and
check outcomes on the arranged model's exact printed geometry, leaving the engine's internal choices
(the winning pass, exact coordinates) free:

- every part prints inside the bed outline, rectangular or round, keeps its spacing on the printed
  mesh with or without mirroring, and stays off exclusion regions and fixed parts;
- a short part goes under an overhang only with stacking on and the clearance met;
- the result never uses more plates or leaves more parts unplaced than the polygon arranger;
- small parts fill the hole of a ring, and copies pack at least as densely as a grid of their boxes;
- the result is centred on the anchor, does not depend on input order or threading, and a cancelled
  arrange leaves only valid placements.
