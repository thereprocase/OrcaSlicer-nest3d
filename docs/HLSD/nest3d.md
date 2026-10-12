# Nest3D shape-true arrange — High Level Design

## Purpose and scope

The polygon arranger packs the 2D convex hull of every part, so a part never enters another
part's hole, notch or concave side, and nothing ever sits under an overhang. Nest3D arranges
parts by their real 3D shape instead. It runs in place of the polygon arranger when the user
turns on "Nest by 3D shape" in the arrange options, and only for layer-by-layer printing on a
flat bed. The option is off by default; with it off, arrange behaves exactly as before.

The engine (`src/libslic3r/Nest3D/Nest3D.{hpp,cpp}`) depends only on the standard library and is
unit tested on its own. The adapter (`Nest3DArrange.{hpp,cpp}`) turns `ArrangePolygon`s into engine
parts and obstacles and writes the result back into the same fields the polygon arranger fills,
so `ArrangeJob::finalize` and the plate bookkeeping work unchanged.

## Model

Every part is a column grid, the *height map*: one cell per `cell_mm` square of the bed, holding
the z-interval `[lo, hi]` the part occupies above that square. A cell's interval is the union of
the part's triangles crossing it, so a part with two levels above one cell keeps the gap between
them filled; the model only ever over-estimates the space a part takes.

- **Spacing.** Every map is grown in XY by the item's inflation, the same value the polygon
  arranger uses (`update_selected_items_inflation` / `update_unselected_items_inflation`: the
  user's spacing, or the brim and support allowance when spacing is automatic). The growth
  claims every cell that comes closer than the inflation to the part, so two parts whose maps
  share no cell keep at least the sum of their inflations apart: a point between two parts
  closer than that lies within the inflation of both. Distances are measured from the extent
  of the geometry inside each occupied cell along the axes and the diagonals, which is exact
  for edges in those directions, so the clearance is not rounded up to whole cells. The map
  starts one inflation before the part, so an axis-parallel edge of a part whose sizes are
  multiples of the cell, grown by a multiple of half a cell, keeps exactly the requested
  spacing; elsewhere, as for parts at arbitrary sizes and angles, a gap costs up to a cell more.
  Only cells near the part's outline are grown: a cell whose eight neighbours hold its interval
  and are filled by the part claims nothing beyond what they claim.
- **Support.** A part that prints with support (`enable_support`) has every column filled down
  to the bed, because the support tower below it occupies that space.
- **Fixed items.** Unselected instances are rasterized from their meshes in their current pose.
  The wipe tower, exclusion areas, the calibration region and belt strips are full-height
  columns over their polygons.
- **Bed.** The bed is the bounding box of the shrunk bed shape. Cells whose centre lies outside a
  non-rectangular bed (circular or polygonal) are masked off.

Two parts may share a cell only when their intervals are at least the vertical clearance apart.
With "Nest under overhangs" off, any occupied cell blocks its whole column, which still lets parts
interlock in plan view (one part in another's hole or notch) but never stack.

## Physical rules for stacking

A part may only be placed in empty space under another part, which by the model means under an
overhang printed without support: supported parts fill their columns to the bed. The parts under
it are shorter than the overhang by at least the vertical clearance (2 mm by default), so the
nozzle and the fan duct pass over them while printing the overhang's first layers. Stacking is
never used for by-object printing, where every object is finished before the next starts and
the toolhead would sweep through the finished ones; the arrange job keeps the polygon arranger
there.

## Candidates

Each part offers a constant number of candidate poses, each with its own height map:

- **Z rotations.** Only the part's current angle unless "Auto rotate" is on; then
  `rotation_steps` evenly spaced angles, or the item's `allowed_rotations` when it lists more than
  one. The angles are taken modulo a full turn and sorted, so copies that an earlier arrange left
  a rotation step apart are offered the same angles in the same order.
- **Lay-flat orientations** ("Try lay-flat orientations"). Up to five poses resting the part on
  the largest faces of its convex hull. A pose is kept only if its unsupported overhang area
  (downward facets steeper than the object's support threshold angle, not counting those on the
  bed) is at most 10% above that of the current pose, and the increase is charged as a penalty.

The pose is expressed like `ModelInstance::get_arrange_polygon`: the instance transform without
its XY offset and Z angle. A placement's rotation and translation therefore mean what they mean
for the polygon arranger. A lay-flat pose is stored as the instance's X and Y Euler angles plus
the Z offset that puts it on the bed; the item's setter applies them after the usual move and Z
rotation.

## Algorithm

Placement is greedy first-fit-decreasing. Parts are ordered (higher priority first, then by a
size key), and each part tries every candidate at every grid position of every open plate, in
plate order, keeping the best-scoring fit; a part that fits nowhere opens the next plate, up to
the plate limit, and is otherwise reported unplaced.

- **Pockets.** A part first tries the free regions the pile already encloses: the 4-connected
  components of free cells inside the bounding box of the parts placed so far that do not reach
  its border, such as the hole of a ring or the bay of a C-clip closed by a neighbour. A part
  placed there leaves the pile's extent unchanged and uses space only a part that small could
  use. It goes to the smallest pocket that holds it (best fit), in the pose that claims the
  fewest cells, as close to the pocket's min corner as it fits, which fills a pocket from one
  side and leaves the rest of it in one piece. Contact does not count here: the staircase edge of
  a part turned 45 degrees touches more cells than a straight one, and preferring it turns small
  parts askew in round holes, where fewer of them fit. Pockets are found with a union-find over
  the free runs of each row of the bounding box, once per placement. With "Nest under overhangs"
  on, they are regions of the bed: a cell with room under its occupant counts as free, so a
  ring's hole under a table top is still a pocket, and the collision test decides what fits
  under the top. A pocket is scanned row by row from its min corner, and the scan stops once
  the distance from that corner exceeds that of a fit already found, so a small part in a large
  pocket tests only the positions nearer the corner than the first one where it fits.
- **Score.** Otherwise the pile grows from an anchor: the point the polygon arranger aligns its
  pile to (`best_object_pos`, the bed centre by default). A position scores the area of the
  bounding box of the pile plus the part, which lets the pile grow lopsided where that is more
  compact, minus 16 mm^2 per mm of the part's outline that touches an occupied cell or the bed
  edge. Many positions tie on area, and contact picks the one where the part nests against what
  is there. Remaining ties go to the position nearest the anchor. A lay-flat pose with more
  overhang scales the area by its penalty. Copies are scored like unique parts: growing each
  group of copies evenly around the anchor instead left mixed piles 12% larger on median (on 24
  random bags of rings, clips, stars, combs, brackets and pegs; up to 2.8 times on bags with few
  parts) and placed no more copies on fill-the-bed sets, where the lattices below do the work.
  The area is a product of one term per axis, convex in the position, which is what lets the
  scan below stop early.
- **Order.** The size keys are the occupied cells, the height, the longer side, the volume, the
  sum of the sides and the bounding-box area. With "Nest under overhangs" on, the first key
  counts only the cells with no room under them (those whose column starts within the vertical
  clearance of the bed, a clearance of 0 included), so a part that leaves room below it (a
  table) comes after the parts that could go there and can be placed over them. On 24 random
  bags with stacking this changes 9 piles, each through a pass with this key: 8 come out 6% to
  11% smaller and one 2% larger, 2.7% smaller on average over all 24. Ties on a pass's key fall
  to the other keys, then the shape key and the id, so the order of the selection does not
  matter: with input order breaking ties, shuffling the input of two mixed bags spread their
  piles over 7% and 11%.
- **Pruning.** Contact can lower a score by at most the part's outline length, and by at most the
  number of occupied cells around the part's rectangle, which a summed-area table of the plate
  gives in O(1). Positions that cannot beat the best score found so far are skipped before the
  collision test. The scan goes outward from the rows and columns where the area is smallest,
  nearest the anchor, and stops on each side once the area alone loses. Pruning never changes
  the result. After a placement only the rows the part covers are recounted; the rows above take
  the change in their cumulative count.
- **Copies.** Instances of one object posed alike share a shape key (the same transform apart from
  the XY offset and Z angle, the same candidate Z angles and spacing), so their candidates are
  identical. Each group of copies gets *clusters*: one copy in a candidate pose, and the tightest
  pair, which is the touching offset of two copies with the smallest common bounding box, then the
  most contact. Only the four candidates that fill their bounding rectangle best (scaled down by
  their penalty) get clusters, and the pair is searched among each of them with itself and with
  its half turn, so the cost stays the same whether a part has 4 or 432 candidates. The raster
  no-fit set of a cluster against itself (every offset at which a translate collides) yields its
  densest collision-free lattice. Basis vectors come from the offsets where a translate just
  touches. A basis is accepted when no lattice vector within the cluster's size collides, and the
  cluster's bounding-box lattice is the fallback. In the seeded pass, the first copy of a group to
  reach a plate lays a lattice there. The lattice runs through the position where the whole
  cluster scores best, scored by the smallest anchor-centred rectangle holding the pile and the
  cluster, which puts the first cluster on the anchor so that no wall pins it where the lattice
  cannot grow around it. Lattices are tried densest per copy first, a penalised pose's lattice
  counting as less dense by its penalty, as in the score. When no lattice holds all copies that
  way (the copies fill the plate), the cosets of every cluster's lattice are counted (at most
  16384 of them, evenly spread, for very large clusters), and the one that fits the most copies
  wins, because the lattice phase decides how much the bed edges cut off. A site whose rectangle
  holds no occupied cell, read from the summed-area table, needs no collision test. Copies take
  the sites nearest the anchor, and the normal search places any copies left over.
- **Passes.** Each distinct part ordering gets a pass: 1, 3 or 6 orderings with the Fast, Balanced
  and Thorough effort settings, and orderings that coincide, as they do for copies, run once. A
  further pass seeds copies on lattices. If these leave parts unplaced or need more than one
  plate, a last pass grows the pile from the bed's min corner, for parts that only fit pushed into
  opposite corners. The best result wins: fewest unplaced parts, then fewest plates, then the most
  parts on the first plate, then on the second, and so on, so that parts overflowing one plate
  leave it as full as a single-plate arrange would, then the smallest total cost. A plate costs
  its bounding-box area, scaled by its parts' mean penalty, plus the free area left in its
  pockets: that area lies inside the bounding box already and counts once more as waste, the
  measure the pocket search minimises, so a pass that fills its pockets beats one whose bounding
  box is smaller by less than the area it left empty. The pile is then centred on the anchor, as
  the polygon arranger centres its pile. With final alignment off, every pass grows from the min
  corner and the pile stays there.

Parts that the polygon arranger would refuse to put together never share a plate: high- and
low-temperature filaments are kept apart, and with "Allow multiple materials on same plate" off,
every distinct filament set is a group of its own. A plate holding fixed parts takes their
group, so new parts join it only when compatible with what is already there.

## Complexity

Every step is polynomial; nothing enumerates permutations or subsets. With `O` passes (at most 6
orderings plus a lattice-seeded and a corner-grown pass), `N` parts, `L` open plates, `C`
candidates per part, `P` grid positions per plate and `A` cells per part map, placement costs
`O(O * N * L * C * P * A / 64)`: a position costs O(1) when the bounds prune it, otherwise a
collision test and a contact count, both 64 cells at a time. The pocket search tests at most the
positions inside the pile's bounding box, the same bound, and in a pocket larger than the part
only those nearer its corner than the first fit. Keeping the plate's summed-area table current
costs `O(P)` per placed part, and finding its pockets `O(P / 64 + R)` for `R` free runs in the
bounding box. Rasterizing costs `O(C * T)` per part of `T` triangles, plus `O(C * (A + B * g^2))`
to grow a map with `B` cells near its outline by `g` cells. Parts and candidates are prepared in
parallel with TBB.

The effort setting chooses the cell size and the number of orderings: Fast 1 mm and 1 ordering,
Balanced 0.5 mm and 3, Thorough 0.5 mm and 6. On beds longer than 720 cells the cell grows with
the bed, which bounds `P`. Cells of 1 and 0.5 mm divide whole and half millimetres, so an
axis-parallel edge at such a size and spacing falls on a cell boundary and keeps its spacing
exactly. On a fill-the-bed benchmark (eight shapes that tile the plane, sizes in whole
millimetres, 256 mm bed, 1 mm spacing), 0.6 mm cells place up to 10% fewer parts than 0.5 mm
cells and 0.43 mm cells up to 6% fewer. With the shapes scaled off that grid (by 1.037 or
1.113), turned 17 degrees, or both, 0.43, 0.5 and 0.6 mm cells place 74.8%, 74.6% and 72.8% of
the area bound on average, so the choice of 0.5 mm costs little where sizes are arbitrary.
0.25 mm cells took 4 to 8 times as long for at most 4% more parts and 2% smaller mixed piles, so
Thorough spends its time on orderings instead. On a 510 mm bed, the 0.71 mm cells Balanced uses
there place 2% to 5% fewer parts than 0.5 mm cells in 37% to 62% less time. The default of 8
rotation steps places as many tetrominoes as 4 and more of a part that tiles at 45 degrees (a
chevron: 96 against 90 of a bound of 116).

For a group of copies whose clusters span `W x H` cells with `r` runs of cells per row:

- The clusters and pair search build at most 9 raster no-fit sets of
  `O((W + H) * H * r^2 + W * H)` each, whatever the number of candidates `C`.
- Each of the at most 5 lattice searches tries `O(B^2)` bases, with `B` at most 2048 touching
  offsets, and validates only bases denser than the best so far, each against
  `O(1 + W * H / det)` lattice points.
- Seeding a plate costs one search per cluster. When the copies fill the plate, it also counts
  `min(det, 16384)` cosets of `O(1 + P / det)` sites per lattice, at most `O(P)` collision tests
  per cluster.

Cancelling is polled between parts and inside the cluster and coset tasks, so it takes effect
within one placement search or one lattice search.

## Gating

`ArrangeJob::process` uses Nest3D only when it is enabled and the arrange is not by-object (the
global sequence, or the plate's own when arranging one plate), not on a belt printer, and no
IDEX/IQEX zone is active. Every other case runs the polygon arranger with the same inputs as
before. The settings are stored in the `arrange` section of the app config with the same
per-mode postfixes as the other arrange options (`_fff`, `_fff_seq_print`, `_sla`).
