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
- **Spacing.** Every map is grown in XY by the item's inflation: the user's spacing halved, or with
  automatic spacing one millimetre for a part whose brim is in its footprint (see Brim and skirt), and
  the caller's allowance for brim and support otherwise. Growth claims every cell that comes closer than
  the inflation to the part, so two parts whose maps share no cell keep at least the sum of their
  inflations apart. Distances are measured from the extent of the geometry inside each occupied cell
  along the axes and the diagonals, which is exact for edges in those directions, so the clearance is not
  rounded up to whole cells. A cell whose eight neighbours are filled and hold its interval claims
  nothing they do not, and is skipped, which keeps the 12 mm growth of tree supports near the cost of the
  outline.
- **Support.** By default a part that prints with support (`enable_support`) has every column
  filled down to the bed. With "Generate supports before arranging", the adapter instead combines
  native sliced model geometry with support extrusion envelopes, including their widths and actual
  extrusion heights. Thick bridges and combined infill can extend below the nominal layer bottom.
  Z contouring also extends the envelope by the generated paths' upward and downward offsets.
  Model brims, support base, interface, tree brim and raft paths occupy space; unused
  columns remain available. Automatic spacing retains 1 mm per part after replacing the estimated
  support allowance with actual geometry; an explicit gap applies outside that geometry.
- **Fixed items.** Unselected instances use their current pose, with native model/support envelopes
  when support generation is enabled and mesh geometry otherwise.
  The wipe tower and exclusion areas are full-height columns over their polygons. On a bed that is
  not a rectangle, the cells whose centre lies outside its outline are a full-height obstacle on
  every plate.
- **Bed edge.** The printable area is the bed: callers hand over that area pulled in by a bed
  shrink, and the adapter takes the shrink back, since the half spacing every part is grown by already
  keeps parts apart and would count a second time against the edge. A part's footprint may reach the
  boundary. What prints beyond the footprint stays inside: with one skirt for the whole plate, its distance
  and the width of its loops. The bed moves by the least of (inflation - that extent) over the selected
  parts. A part whose brim could not be shaped keeps the caller's allowance (its brim width, the skirt's
  distance and loops) in that sum instead. A bed that does not match the printable area less the
  shrink, as one the caller reshaped, is used as it comes.
- **Reserved plate area.** Two more full-height obstacles on every plate keep the spacing of the
  excluded regions (1 mm, or half an explicit gap). The first is `bed_exclude_area` read as the one
  polygon the plate draws: the plate hands the arrangers the bounding box of every four points, which
  drops the rest of an L-shaped or other many-cornered area. The second is the material the machine
  start G-code lays on the plate, such as a purge line (`GCode/StartGCodeExtrusion`). The template is
  processed with the placeholder parser and the variables the G-code export sets before it; those
  that depend on the finished layout (`first_layer_print_min`, bed mesh bounds) read as the whole
  printable area, so a purge placed relative to the print is not predicted. The processed moves are
  then followed through G90/G91, M82/M83, G92, homing and arcs. Every extruding move from a known
  position is a bead as wide as its volume over the nozzle height gives, never narrower than the
  nozzle; retraction and unretraction deposit nothing, and a flush standing still counts only over the
  printable area. G-code coordinates plus the extruder offset are plate coordinates, as in the G-code
  viewer. A start G-code that extrudes nowhere, or that cannot be evaluated before slicing, reserves
  nothing and leaves the arrangement as it was.

Two parts may share a cell only when their intervals are at least the vertical clearance apart.
With "Nest under overhangs" off, any occupied cell blocks its whole column, which still lets parts
interlock in plan view (one part in another's hole or notch) but never stack. One collision test,
`Board::fits`, decides every placement, pocket, lattice no-fit set and recentring.

## Brim and skirt

An object's footprint includes what it prints on the first layer around its geometry, as the slicer will
generate it.

- **Brim.** The adapter slices a copy of the object in its arrange pose, with its own settings over the
  print's (brim type, width, ears and their angle, gap to the object, auto widths, painted ears), and
  runs the slicer's own brim generator (`make_brim`) on the first layer. Only the slice and the brim run:
  no perimeters, infill or support. The brim's areas join the object's triangles as caps from the bed to
  the first layer's height, so they enter every candidate angle's height map like the geometry does. The
  brim is made once per object and pose and shared by every instance posed alike, fixed parts
  included, whatever the number of angles. An object without a brim costs nothing. With generated
  supports the slice of each candidate already carries the brim, so only the skirt is added.
- **Spacing against brims.** With the brim in the footprint, automatic spacing leaves one millimetre
  around it, so two brims keep two millimetres apart and never overlap; an explicit gap applies outside
  the brims. Tree supports keep the caller's branch-sized allowance.
- **Skirt per object.** The skirt that goes around each object is the ring from the skirt distance to
  the last loop around the convex hull of the object and its brim, from the bed to the skirt's height.
  It joins the footprint like the brim. The ring leaves the hull's inside free, so small parts can still
  go into the object's pockets.
- **One skirt for the plate.** A combined skirt encircles everything on the plate, which no single
  footprint can see. The bed margin keeps it on the printable area, and once the parts are placed the ring
  around the hull of every part on the plate (fixed parts with their brim) is checked against the excluded
  area and the start G-code purge strip. If it crosses one, the parts are packed again with the skirt's
  width added to the room they keep from those areas. A skirt that still crosses a limit after that, or
  leaves the bed, is reported.

The hull of a skirt is that of the whole object, not only of the layers up to the skirt height, which
over-reserves a little for an object that widens above the skirt.

## Messages

When a part gets no place, or goes to a later plate while others of its filament group sit on an earlier
one, the adapter returns an `Issue` for it: the part's name, the size of its footprint with brim and
per-object skirt at the angle that gives the smallest box, the limit it hit, and a hint. The limit comes
from placing the part alone on an empty plate and adding the limits one at a time: the bed outline, the
excluded area, the start G-code purge strip. Each reports the largest free rectangle that limit leaves.
A part that fits an empty plate is told that the others leave it no room, or that every plate is full.
A skirt around the plate that still crosses a limit is an issue of its own.

Every sentence is one translatable msgid with placeholders; `issue_text` fills them in after the caller
translates, so no translated fragments are concatenated. The arrange job shows the first five issues in
a notification and logs the rest. The command line writes them to the arrange report under `messages`.

## Physical rules for stacking

A part may only be placed in empty space under another part. Without support generation,
supported parts fill their columns to the bed; with it, the generated supports block their actual
columns. The parts under
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

Support preparation clones the object with its volumes, modifiers, painted support data, object
settings and variable layer heights. Each candidate orientation runs the native perimeter, infill
and support pipeline because world-aligned support patterns cannot simply be rotated afterward.
A temporary expanded bed avoids clipping desired branches to an arbitrary initial position; the
nesting engine tests the resulting envelope against the actual bed and exclusions. Candidates run
sequentially while each slice uses its own internal parallelism, bounding temporary slicing memory.
Cancellation reaches the native slicing loops and leaves the original placements untouched.
Sliced model geometry includes material shrinkage and XY contour compensation. The option requires
matching XY and Z shrinkage compensation among used filaments, because otherwise the slicer can
change scale when parts are combined. Unused filament slots do not impose this restriction.

The preliminary slice is per part. Final plate slicing can change supports in response to neighbors
or placement, so final toolpath checks remain necessary, especially at small gaps. Clearance between
parts also does not establish that supports will detach cleanly from their own supported surfaces.

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

Without support generation, the result never uses more plates or leaves more parts unplaced than the polygon arranger: when it
does not fit one plate, the adapter runs the polygon arranger on the same input and keeps the
better of the two. The polygon arranger packs every item's convex hull without overlap, so it
needs at least as many plates as the hulls' total area fills beds; a result already there is kept
without running it. A cancelled arrange keeps the parts it placed and runs no fallback: under the
same stop condition the polygon arranger would return every part on top of the others.
With generated supports the polygon fallback is disabled because its model-only hulls do not reserve
the generated support space.

## Complexity

Every step is polynomial. With `O` passes (at most 6 orderings plus a ring, a lattice and a corner
pass), `N` parts, `L` open plates, `C` candidates per part and `P` grid positions per plate, the
search costs `O(O * N * L * C * P)` run lookups in the worst case, and far fewer where the
envelope bound ends a side; a position that survives costs a collision and a contact test, 64
cells at a time. Keeping the plate's summed-area table current costs `O(P)` per placed part, and
labelling its pockets `O(P + R)` for `R` free runs in the pile's bounding box. A brim costs one slice
of the first layers and one run of the brim generator per object and pose, and a skirt the convex hull of its
object. Rasterizing costs `O(C * (T + K))` per shape of `T` triangles crossing `K` cells, plus `O(B * g)` cells per
growth of `g` cells for `B` cells near the outline. A shape with copies costs a bounded number of
raster no-fit sets and lattice searches over its units' cells; the lattice search tries the 2048
shortest touching offsets, and phase counting reads each bed cell once per unit and lattice.
Cancelling is polled between parts.

## Gating

`ArrangeJob::process` uses Nest3D only when it is enabled and the arrange is not by-object, not on
a belt printer, and no IDEX/IQEX zone is active. By-object printing finishes every object before
the next starts, so the toolhead would sweep through finished parts nested around it; belt
printers and IDEX/IQEX zones place parts by rules of their own. The arrange menu offers
"Nest by 3D shape", "Generate supports before arranging", and "Nest under overhangs" for layer-by-layer FFF printing on a
flat bed, and stores them in the `arrange` section of the app config as `nest3d_enabled_fff`,
`nest3d_supports_fff`, and `nest3d_stacking_fff`. Support generation is off by default and does not
change print profiles or the project format.

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
  arrange leaves only valid placements;
- the brim shape is the slicer's for each brim type and per-object override, neighbouring brims stay apart,
  a per-object skirt is reserved around each object and a combined one around the plate and clear of the
  excluded area;
- an unplaceable part and a part moved to a later plate are explained with the part's name, its footprint
  and the limit that stopped it.

The `[Nest3DSupport]` pipeline tests in `tests/fff_print/test_nest3d_support.cpp` check generated
support and raft footprints, bead clearance for rotated copies, object overrides and painted
enforcers, fixed supported overhangs, thick bridge clearance, material compensation, model brims,
and cancellation without source edits.
