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
  covers every cell that comes within the inflation of an occupied cell, so two parts always
  keep at least the sum of their inflations apart, at the price of up to a cell more.
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

- **Score.** The pile grows from an anchor: the point the polygon arranger aligns its pile to
  (`best_object_pos`, the bed centre by default). A position scores an area, minus 16 mm^2 per mm
  of the part's outline that touches an occupied cell or the bed edge. For a copy (see below) the
  area is that of the smallest anchor-centred rectangle that holds the parts placed so far plus
  this one: it puts the first copy on the anchor and makes the pile grow evenly outward, so no
  wall pins the first copy where its partner cannot reach it. For a unique part it is the plain
  bounding box of the pile plus the part, which lets the pile grow lopsided where that is more
  compact; on random sets of 15 unique parts the anchor-centred rectangle leaves the pile 13%
  larger on median (3% for 30 parts). Many positions tie on area, and contact picks the one where
  the part nests against what is there. Remaining ties go to the position nearest the anchor. A
  lay-flat pose with more overhang scales the area by its penalty. Both areas are products of one
  convex term per axis, which is what lets the scan below stop early.
- **Pruning.** Contact can lower a score by at most the part's outline length, and by at most the
  number of occupied cells around the part's rectangle, which a summed-area table of the plate
  gives in O(1). Positions that cannot beat the best score found so far are skipped before the
  collision test. The scan goes outward from the rows and columns where the area is smallest,
  nearest the anchor, and stops on each side once the area alone loses. Pruning never changes
  the result.
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
  cluster scores best. Lattices are tried densest per copy first, a penalised pose's lattice
  counting as less dense by its penalty, as in the score. When no lattice holds all copies that
  way (the copies fill the plate), the cosets of every cluster's lattice are counted (at most
  16384 of them, evenly spread, for very large clusters), and the one that fits the most copies
  wins, because the lattice phase decides how much the bed edges cut off. Copies take the sites
  nearest the anchor, and the normal search places any copies left over.
- **Passes.** Each distinct part ordering gets a pass: 1, 3 or 6 orderings with the Fast, Balanced
  and Thorough effort settings, and orderings that coincide, as they do for copies, run once. A
  further pass seeds copies on lattices. If these leave parts unplaced or need more than one
  plate, a last pass grows the pile from the bed's min corner, for parts that only fit pushed into
  opposite corners. The best result wins: fewest unplaced parts, then fewest plates, then the most
  parts on the first plate, then on the second, and so on, so that parts overflowing one plate
  leave it as full as a single-plate arrange would, then the smallest total bounding-box area,
  each plate's scaled by its parts' mean penalty. The pile is then centred on the anchor, as the
  polygon arranger centres its pile. With final alignment off, every pass grows from the min
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
collision test and a contact count, both 64 cells at a time. Keeping the plate's summed-area table
current costs `O(P)` per placed part. Rasterizing costs `O(C * T)` per part of `T` triangles. The
effort setting chooses a cell size of 1, 0.6 or 0.4 mm, coarsened so the bed is at most 600 cells
per side, which bounds `P`. Parts and candidates are prepared in parallel with TBB.

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
