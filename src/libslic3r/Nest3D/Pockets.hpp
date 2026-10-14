#pragma once

// Pockets: the free regions a pile encloses, and best-fit placement into them.
//
// A region is a 4-connected set of cells that are free at the floor inside the pile's bbox. Regions
// are built from runs: each row's free cells are cut into runs, and each run joins the runs it overlaps
// in the row above, a union-find over a few runs per row in place of a flood over every cell.

#include "Grid.hpp"

#include <cstddef>
#include <vector>

namespace Slic3r { namespace nest3d {

// Closed: no cell of the region lies on the border of the scanned rect, so the pile encloses it.
struct Region
{
    Rect box;
    int  cells  = 0;
    bool closed = false;
};

struct Pockets
{
    Rect                box;     // the pile bbox clipped to the bed
    std::vector<int>    label;   // per cell of box: region index, -1 where not free at the floor
    std::vector<Region> regions; // in raster order of their first cell

    int region_at(int x, int y) const { return label[size_t(y - box.y0) * (box.x1 - box.x0) + (x - box.x0)]; }
};

// The regions of the plate's pile bbox, labelled cell by cell.
Pockets find_pockets(const Board &board, const Rect &pile);

// The pocket spot for one of the candidates `fps`, or a Spot with candidate -1 when no closed region holds one.
Spot pocket_search(const std::vector<Footprint> &fps, const Board &board, const Pockets &pockets);

}} // namespace Slic3r::nest3d
