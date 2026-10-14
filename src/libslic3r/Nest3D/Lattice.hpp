#pragma once

// The identical-copy lattice: copies of one shape laid on the densest lattice their footprint tiles, or
// as interlocked pairs on a pair lattice.

#include "Grid.hpp"
#include "Nest3D.hpp"

#include <vector>

namespace Slic3r { namespace nest3d {

// A lattice in Hermite normal form, spanned by (a, 0) and (b, c) with a, c > 0 and 0 <= b < a: rows c
// apart, each a run of points a apart, one point per a * c cells.
struct Lattice { long long a = 1, b = 0, c = 1; };

// One copy, or two interlocked copies, laid as one on a lattice. A member is a candidate and its tight
// rect's offset in the unit's tight rect.
struct Unit
{
    struct Member { int candidate = 0, dx = 0, dy = 0; };
    std::vector<Member>  members;
    Footprint            fp;       // union of the members
    std::vector<Lattice> lattices; // the bounding-box lattice, then the densest when denser
};

// Units for copies of a shape with candidates `cands` and their footprints `fps`: one per distinct
// candidate that fills its rect best, and the best interlocked pair of one of them with itself or its
// half turn.
std::vector<Unit> lattice_units(const std::vector<Footprint> &fps, const std::vector<Candidate> &cands,
                                const Params &p);

// Up to cap copies on the best phase of the units' lattices around the anchor, in cells, for a plate
// whose pile so far is `pile`: every spot fits the board and none collide. Empty when no copy fits.
std::vector<Spot> lay_lattice(const std::vector<Unit> &units, const std::vector<Footprint> &fps, const Board &board,
                              const Rect &pile, double anchor_x, double anchor_y, int cap, const ParallelFor &pf);

}} // namespace Slic3r::nest3d
