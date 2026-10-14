#pragma once

// Internal grid types of the engine: a candidate reduced to its occupied cells (Footprint) and the
// occupancy of one plate (Board). Every collision in the engine, placement, tucking under overhangs,
// lattice no-fit sets and recentring alike, goes through Board::fits.

#include "Nest3D.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace Slic3r { namespace nest3d {

// Bed cells [x0, x1) x [y0, y1).
struct Rect
{
    int  x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    bool empty() const { return x1 <= x0 || y1 <= y0; }
    long area() const { return empty() ? 0 : long(x1 - x0) * (y1 - y0); }
    Rect united(const Rect &other) const
    {
        if (empty() || other.empty())
            return empty() ? other : *this;
        return {std::min(x0, other.x0), std::min(y0, other.y0), std::max(x1, other.x1), std::max(y1, other.y1)};
    }
};

// Where a part landed: its candidate and the bed cell of its footprint's tight rect.
struct Spot { int candidate = -1, x = 0, y = 0; };

inline int popcount(uint64_t v) { return int(std::bitset<64>(v).count()); }

// ~v & (v - 1) keeps exactly the bits below the lowest set bit of v, which must not be 0.
inline int trailing_zeros(uint64_t v) { return popcount(~v & (v - 1)); }

// True when nothing fits under a column starting at height lo. Without stacking (gap Inf) every column
// stands on the floor.
inline bool on_floor(float lo, double gap) { return lo <= gap + 1e-3; }

// One candidate's height map reduced to its occupied cells, as spans for height comparisons and as one
// bit per cell, so a row is compared with the board 64 cells at a time.
struct Footprint
{
    int               x0 = 0, y0 = 0, w = 0, h = 0; // tight rect of the occupied cells in the candidate's map
    int               first_x = 0;                  // first occupied column of row 0: a pocket membership probe
    std::vector<Span> spans;                        // w * h, row-major

    int                   words = 0; // per row of bits
    std::vector<uint64_t> bits;      // h rows; cell (x, y) is bit x of row y

    // The empty cells 4-adjacent to the footprint, which are what it touches: the tight rect grown by one
    // cell, h + 2 rows of halo_words words, cell (x, y) at bit x + 1 of row y + 1.
    int                   halo_words = 0;
    std::vector<uint64_t> halo;

    int    cells = 0, floor_cells = 0, halo_cells = 0;
    float  top    = 0.f;
    double volume = 0.;
    bool   usable = false; // non-empty and no taller than the printable height

    bool occupied(int x, int y) const
    {
        if (x < 0 || y < 0 || x >= w || y >= h)
            return false;
        return (bits[size_t(y) * words + size_t(x / 64)] >> (x % 64)) & 1;
    }
    const Span &span(int x, int y) const { return spans[size_t(y) * w + x]; }
};

Footprint make_footprint(const HeightMap &m, double max_height, double vertical_gap);

// Occupancy of one plate: spans per cell, one bit per cell and a summed-area table of the occupied cells.
// Each bit row starts with a whole pad word and ends with the bits past the bed, all set, so cell x is
// bit x + 64 of its row and a 64-cell window may start before the bed or reach past it. The cells it
// finds there read as occupied: the bed's edges count as contact and block fits().
struct Board
{
    int                   w = 0, h = 0;
    int                   words        = 0;   // per row of bits
    double                vertical_gap = Inf; // as Params::vertical_gap
    std::vector<Span>     spans;
    std::vector<uint64_t> bits;
    std::vector<int>      summed_area; // (w + 1) x (h + 1): occupied cells below and left of each corner

    Board() = default;
    Board(int width, int height, double gap = Inf)
        : w(width), h(height), words((width + 63) / 64 + 2), vertical_gap(gap), spans(size_t(width) * height)
        , bits(size_t(height) * words), summed_area(size_t(width + 1) * (height + 1))
    {
        for (int y = 0; y < h; ++y) {
            uint64_t *row = &bits[size_t(y) * words];
            row[0]        = ~uint64_t(0);
            for (int bit = w + 64; bit < 64 * words; ++bit)
                row[bit / 64] |= uint64_t(1) << (bit % 64);
        }
    }

    const Span &span(int x, int y) const { return spans[size_t(y) * w + x]; }

    // The 64 cells of row y from cell x, -64 <= x <= w, as bits 0 to 63. Rows off the bed are full.
    uint64_t row_bits(int y, int x) const
    {
        if (y < 0 || y >= h)
            return ~uint64_t(0);
        const uint64_t *row   = &bits[size_t(y) * words];
        const int       bit   = x + 64;
        const int       word  = bit / 64;
        const int       shift = bit % 64;
        if (shift == 0)
            return row[word];
        return (row[word] >> shift) | (row[word + 1] << (64 - shift));
    }

    bool occupied(int x, int y) const { return (bits[size_t(y) * words + 1 + size_t(x / 64)] >> (x % 64)) & 1; }

    // The summed-area table is stale until update_summed_area().
    void add(int x, int y, const Span &s)
    {
        spans[size_t(y) * w + x].add(s);
        bits[size_t(y) * words + 1 + size_t(x / 64)] |= uint64_t(1) << (x % 64);
    }

    // Adds a footprint with its tight rect at (x, y), which must lie on the bed.
    void place(const Footprint &fp, int x, int y)
    {
        for (int row = 0; row < fp.h; ++row)
            for (int column = 0; column < fp.w; ++column)
                if (fp.occupied(column, row))
                    add(x + column, y + row, fp.span(column, row));
        update_summed_area(y, y + fp.h);
    }

    // After rows [y0, y1) changed: they are recounted from the row below them, and every row above takes
    // the change in its total.
    void update_summed_area(int y0, int y1)
    {
        const size_t           stride = size_t(w) + 1;
        const std::vector<int> old_row_above(summed_area.begin() + long(size_t(y1) * stride),
                                             summed_area.begin() + long(size_t(y1 + 1) * stride));
        for (int y = y0; y < y1; ++y) {
            const int *below  = &summed_area[size_t(y) * stride];
            int       *row    = &summed_area[size_t(y + 1) * stride];
            int        in_row = 0;
            for (int x = 0; x < w; ++x) {
                in_row += occupied(x, y);
                row[x + 1] = below[x + 1] + in_row;
            }
        }
        for (size_t y = size_t(y1) + 1; y <= size_t(h); ++y)
            for (size_t x = 0; x < stride; ++x)
                summed_area[y * stride + x] += summed_area[size_t(y1) * stride + x] - old_row_above[x];
    }

    // Occupied cells in [x0, x1) x [y0, y1), clipped to the bed.
    int occupied_in(int x0, int y0, int x1, int y1) const
    {
        x0 = std::max(x0, 0);
        y0 = std::max(y0, 0);
        x1 = std::min(x1, w);
        y1 = std::min(y1, h);
        if (x0 >= x1 || y0 >= y1)
            return 0;
        auto corner = [&](int x, int y) { return summed_area[size_t(y) * (w + 1) + x]; };
        return corner(x1, y1) - corner(x0, y1) - corner(x1, y0) + corner(x0, y0);
    }

    // True when the footprint with its tight rect at (x, y) lies on the bed and collides with nothing.
    bool fits(const Footprint &fp, int x, int y) const
    {
        if (!fp.usable || x < 0 || y < 0 || x + fp.w > w || y + fp.h > h)
            return false;
        // The summed-area table settles a rect with no occupied cell and, without stacking, one with more
        // occupied cells than the footprint leaves empty.
        const int occupied_under = occupied_in(x, y, x + fp.w, y + fp.h);
        if (occupied_under == 0)
            return true;
        if (std::isinf(vertical_gap) && occupied_under > fp.w * fp.h - fp.cells)
            return false;
        for (int i = 0; i < fp.h; ++i)
            if (collides_in_row(fp, x, y, middle_row_first(i, fp.h)))
                return false;
        return true;
    }

    // Halo cells of the footprint at (x, y) that are on occupied cells or past the bed's edge.
    int contact(const Footprint &fp, int x, int y) const
    {
        int n = 0;
        for (int row = 0; row < fp.h + 2; ++row)
            for (int k = 0; k < fp.halo_words; ++k)
                n += popcount(row_bits(y - 1 + row, x - 1 + 64 * k) & fp.halo[size_t(row) * fp.halo_words + k]);
        return n;
    }

    // Free at the floor: empty, or with room under its occupant (only with stacking).
    bool free_at_floor(int x, int y) const { return span(x, y).empty() || !on_floor(span(x, y).lo, vertical_gap); }

private:
    // The middle row first, where a hit is likeliest, then the rows below it, then the rows above.
    static int middle_row_first(int i, int rows)
    {
        if (i == 0)
            return rows / 2;
        return i <= rows / 2 ? i - 1 : i;
    }

    // A cell both occupy is allowed only when the two spans keep the vertical gap apart.
    bool collides_in_row(const Footprint &fp, int x, int y, int row) const
    {
        for (int k = 0; k < fp.words; ++k) {
            uint64_t shared = row_bits(y + row, x + 64 * k) & fp.bits[size_t(row) * fp.words + k];
            while (shared != 0) {
                const int   column = 64 * k + trailing_zeros(shared);
                const Span &part   = fp.span(column, row);
                const Span &board  = span(x + column, y + row);
                if (!(part.hi + vertical_gap <= board.lo || board.hi + vertical_gap <= part.lo))
                    return true;
                shared &= shared - 1;
            }
        }
        return false;
    }
};

}} // namespace Slic3r::nest3d
