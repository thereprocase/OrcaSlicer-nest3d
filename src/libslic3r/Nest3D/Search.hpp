#pragma once

// Search: where one part goes on one plate.
//
// The search finds the exhaustive minimum of the score over every position of every candidate. It
// scans outward from where the envelope is least and gives up on a position, a run of positions or
// the rest of a row only when a lower bound on their score is strictly above a hit already found.
// Strictly, so a position that ties the best is still visited, and the result never depends on which
// task found what first.
//
// Collisions are tested by runs. A footprint is a short list of runs, the stretches of low cells along
// its rows, and a run collides when the board holds a low cell under it, one prefix-count lookup per
// run. A run with n low cells under it also collides at the next n - 1 positions, since each step moves
// one cell out from under it, and the widest runs go first, so the first lookup usually both decides
// and jumps. Without stacking every cell is low, so the runs are the whole footprint and fits() is
// never called; with stacking fits() has the last word where the runs find nothing.

#include "Grid.hpp"
#include "Nest3D.hpp"

#include <algorithm>
#include <tuple>
#include <vector>

namespace Slic3r { namespace nest3d {

// A position, ordered by score (envelope - contact_value * contact), then the squared distance from the
// footprint's centre to the envelope centre, then y, x and candidate: a total order, so the best does not
// depend on which task found it.
struct Hit
{
    double score = Inf, dist = 0.;
    int    y = 0, x = 0, candidate = -1;
    bool   operator<(const Hit &o) const
    {
        return std::tie(score, dist, y, x, candidate) < std::tie(o.score, o.dist, o.y, o.x, o.candidate);
    }
};

struct SearchArgs
{
    const Board &board;
    Rect         pile;               // bbox of this run's parts on the plate
    double       centre_x, centre_y; // envelope centre, cells: the anchor, or the pile's weighted centroid
    bool         ring;               // false: box envelope; true: ring envelope
    double       contact_value;      // score per contact cell

    // Growth envelope of the pile with a w x h footprint at (x, y), convex in x and in y. Box: the bbox
    // area of pile and part. Ring: the smallest bed-proportioned rectangle centred on the centre that
    // holds pile and part, which grows a pile round about its centre instead of along a wall or into a
    // strip.
    double envelope(int x, int y, int w, int h) const
    {
        const Rect r = pile.united({x, y, x + w, y + h});
        if (!ring)
            return double(r.x1 - r.x0) * double(r.y1 - r.y0);
        const double reach_x = std::max(centre_x - r.x0, r.x1 - centre_x) / board.w;
        const double reach_y = std::max(centre_y - r.y0, r.y1 - centre_y) / board.h;
        const double q       = std::max(reach_x, reach_y);
        return 4. * board.w * board.h * q * q;
    }
};

// The best position of any of the candidates `fps` on the board, or a Hit with candidate -1 when none fits.
Hit search(const std::vector<Footprint> &fps, const SearchArgs &args, const ParallelFor &pf);

}} // namespace Slic3r::nest3d
