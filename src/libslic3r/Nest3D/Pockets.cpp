#include "Pockets.hpp"
#include "Grid.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>

namespace Slic3r { namespace nest3d {
namespace {

// The runs of cells free at the floor in `box`, in raster order, each a rect one row tall.
std::vector<Rect> free_runs(const Board &board, const Rect &box)
{
    std::vector<Rect> runs;
    for (int y = box.y0; y < box.y1; ++y) {
        int x = box.x0;
        while (x < box.x1) {
            const int start = x;
            while (x < box.x1 && board.free_at_floor(x, y))
                ++x;
            if (x > start)
                runs.push_back({start, y, x, y + 1});
            ++x;
        }
    }
    return runs;
}

// Union-find over runs. A set's root is always its earliest run, so a raster scan meets the root first
// and every later run of the set finds its region already numbered.
class RunSets
{
public:
    explicit RunSets(size_t runs) : m_parent(runs) { std::iota(m_parent.begin(), m_parent.end(), 0); }

    int root(int run)
    {
        while (m_parent[run] != run)
            run = m_parent[run] = m_parent[m_parent[run]];
        return run;
    }

    void join(int run, int other)
    {
        const int a = root(run), b = root(other);
        m_parent[std::max(a, b)] = std::min(a, b);
    }

private:
    std::vector<int> m_parent;
};

// Joins every run to the runs of the row above that share a column with it. Both rows are sorted by
// x, so `above` walks the row above once, past the runs that end before the current run starts.
void join_touching_runs(const std::vector<Rect> &runs, RunSets &sets)
{
    int above = 0;
    for (int run = 0; run < int(runs.size()); ++run) {
        const Rect &r = runs[size_t(run)];
        while (std::make_pair(runs[size_t(above)].y1, runs[size_t(above)].x1) <= std::make_pair(r.y0, r.x0))
            ++above;
        for (int k = above; k < run && runs[size_t(k)].y1 == r.y0 && runs[size_t(k)].x0 < r.x1; ++k)
            sets.join(k, run);
    }
}

// A fit, ordered best first: the smallest region, then the candidate with the fewest cells, then the
// position nearest the region's min corner, then y, x and candidate. The default is above every fit.
struct PocketFit
{
    int region_cells = std::numeric_limits<int>::max();
    int part_cells   = 0;
    int corner_dist2 = 0;
    int y = 0, x = 0, candidate = -1;

    bool operator<(const PocketFit &o) const
    {
        return std::tie(region_cells, part_cells, corner_dist2, y, x, candidate) <
               std::tie(o.region_cells, o.part_cells, o.corner_dist2, o.y, o.x, o.candidate);
    }
};

// Lowers `best` to the candidate's best fit in region `r`. The distance from the corner only grows along
// a row and from row to row, so a row whose nearest position cannot beat `best` ends the scan, and at
// once in a region too large to win.
void best_fit_in_region(const Footprint &fp, int candidate, const Board &board, const Pockets &pockets, int r,
                        PocketFit &best)
{
    const Region &region = pockets.regions[size_t(r)];
    for (int y = region.box.y0; y + fp.h <= region.box.y1; ++y) {
        const int       dy = y - region.box.y0;
        const PocketFit nearest_in_row{region.cells, fp.cells, dy * dy, 0, 0, 0};
        if (!(nearest_in_row < best))
            break;
        for (int x = region.box.x0; x + fp.w <= region.box.x1; ++x) {
            // The probe cell settles region membership before the full collision test.
            if (pockets.region_at(x + fp.first_x, y) != r || !board.fits(fp, x, y))
                continue;
            const int dx = x - region.box.x0;
            best         = std::min(best, PocketFit{region.cells, fp.cells, dx * dx + dy * dy, y, x, candidate});
        }
    }
}

// A part goes into the smallest closed region that holds it, in its pose with the fewest cells, at the fit
// nearest the region's min corner: a pocket fills from one side and keeps its free space in one piece,
// where raster order would start a round hole in the middle of its bottom row. Contact plays no part: the
// staircase edge of a part turned 45 degrees touches more cells and would turn pegs askew in round holes.
// Regions go smallest first, so the first fit cuts off every larger region; the bound is shared by all

} // namespace

Pockets find_pockets(const Board &board, const Rect &pile)
{
    const Rect box{std::max(pile.x0, 0), std::max(pile.y0, 0), std::min(pile.x1, board.w),
                   std::min(pile.y1, board.h)};
    Pockets    pockets{box, std::vector<int>(size_t(box.area()), -1), {}};
    const std::vector<Rect> runs = free_runs(board, box);
    RunSets                 sets(runs.size());
    join_touching_runs(runs, sets);
    std::vector<int> region_of(runs.size());
    for (int run = 0; run < int(runs.size()); ++run) {
        const Rect &r    = runs[size_t(run)];
        const int   root = sets.root(run);
        if (root == run) {
            pockets.regions.emplace_back();
            region_of[size_t(run)] = int(pockets.regions.size()) - 1;
        } else {
            region_of[size_t(run)] = region_of[size_t(root)];
        }
        Region &region = pockets.regions[size_t(region_of[size_t(run)])];
        region.box     = region.box.united(r);
        region.cells += int(r.area());
        // Judged again with every run, so the region's last run leaves the verdict for its whole bbox.
        region.closed = region.box.x0 > box.x0 && region.box.y0 > box.y0 && region.box.x1 < box.x1 &&
                        region.box.y1 < box.y1;
        int *first_label = &pockets.label[size_t(r.y0 - box.y0) * (box.x1 - box.x0) + (r.x0 - box.x0)];
        std::fill_n(first_label, r.area(), region_of[size_t(run)]);
    }
    return pockets;
}

Spot pocket_search(const std::vector<Footprint> &fps, const Board &board, const Pockets &pockets)
{
    std::vector<int> closed;
    for (int r = 0; r < int(pockets.regions.size()); ++r)
        if (pockets.regions[size_t(r)].closed)
            closed.push_back(r);
    const auto smaller = [&](int p, int q) {
        return pockets.regions[size_t(p)].cells < pockets.regions[size_t(q)].cells;
    };
    std::stable_sort(closed.begin(), closed.end(), smaller);
    PocketFit best;
    for (int candidate = 0; candidate < int(fps.size()); ++candidate) {
        const Footprint &fp = fps[size_t(candidate)];
        for (int r : closed) {
            const Region &region = pockets.regions[size_t(r)];
            if (region.cells > best.region_cells)
                break;
            if (fp.usable && region.cells >= fp.cells)
                best_fit_in_region(fp, candidate, board, pockets, r, best);
        }
    }
    return {best.candidate, best.x, best.y};
}

}} // namespace Slic3r::nest3d
