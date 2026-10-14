#include "Search.hpp"
#include "Grid.hpp"
#include "Nest3D.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <vector>

namespace Slic3r { namespace nest3d {
namespace {

// Where on one axis the envelope is least: the part centred on `centre`, clamped to the bed. The envelope
// term is piecewise linear in t with slopes -1, 0 and +1 around that centre, so rounding lands on an
// argmin, and from an argmin of a convex term the envelope never falls going outward.
int pivot(double centre, int size, int last)
{
    return std::clamp(int(std::floor(centre - 0.5 * size + 0.5)), 0, last);
}

// Spans that start below the gap and end above the bed: two of them are never `gap` apart (a.hi + gap
// >= gap > b.lo, and the same the other way), so they collide whatever their heights.
bool is_low(const Span &s, double gap) { return !s.empty() && s.lo < gap && s.hi >= 0.f; }

// The runs of the footprint's low cells, each a rect one row tall, widest first.
std::vector<Rect> row_runs(const Footprint &fp, double gap)
{
    std::vector<Rect> runs;
    for (int y = 0; y < fp.h; ++y) {
        int x = 0;
        while (x < fp.w) {
            const int start = x;
            while (x < fp.w && is_low(fp.span(x, y), gap))
                ++x;
            if (x > start)
                runs.push_back({start, y, x, y + 1});
            ++x;
        }
    }
    const auto wider = [](const Rect &p, const Rect &q) { return p.x1 - p.x0 > q.x1 - q.x0; };
    std::stable_sort(runs.begin(), runs.end(), wider);
    return runs;
}

// The cells of a board row a run may not cover: without stacking the occupied cells, which the board's
// summed-area table counts, with stacking the low ones, counted by prefix sums along every row.
class BlockedCells
{
public:
    BlockedCells(const Board &board, const ParallelFor &pf) : m_board(board), m_stride(size_t(board.w) + 1)
    {
        if (std::isinf(board.vertical_gap))
            return;
        m_low_prefix.resize(size_t(board.h) * m_stride);
        pf(size_t(board.h), [&](size_t y) {
            int *row = &m_low_prefix[y * m_stride];
            for (int x = 0; x < board.w; ++x)
                row[x + 1] = row[x] + is_low(board.span(x, int(y)), board.vertical_gap);
        });
    }

    // Blocked cells of row y in [x0, x1).
    int count(int y, int x0, int x1) const
    {
        if (m_low_prefix.empty())
            return m_board.occupied_in(x0, y, x1, y + 1);
        return m_low_prefix[y * m_stride + x1] - m_low_prefix[y * m_stride + x0];
    }

private:
    const Board     &m_board;
    size_t           m_stride;
    std::vector<int> m_low_prefix;
};

// Lowers the shared bound to `score` when that is lower. The bound is only ever compared with a strict
// >, so it prunes positions that cannot win without deciding ties.
void lower_limit(std::atomic<double> &limit, double score)
{
    double current = limit.load();
    while (score < current && !limit.compare_exchange_weak(current, score)) {}
}

// How a row scan goes on after one position: the side is done, or the next position lies this many
// columns further out.
struct Step
{
    bool side_done = false;
    int  columns   = 1;
};

// The scan of one candidate's footprint over one band of rows. Rows are dealt to bands round-robin by
// their distance from the pivot row, so every band gets a share of the rows nearest the pile.
class FootprintScan
{
public:
    FootprintScan(const Footprint &fp, int candidate, const SearchArgs &args, const BlockedCells &blocked,
                  std::atomic<double> &limit)
        : m_fp(fp), m_candidate(candidate), m_args(args), m_board(args.board), m_blocked(blocked), m_limit(limit)
        , m_stacking(!std::isinf(m_board.vertical_gap)), m_last_x(m_board.w - fp.w), m_last_y(m_board.h - fp.h)
    {
        // Box growth is least with the part centred on the pile, ring growth with it on the centre.
        const bool   on_pile  = !args.ring && !args.pile.empty();
        const double centre_x = on_pile ? 0.5 * (args.pile.x0 + args.pile.x1) : args.centre_x;
        const double centre_y = on_pile ? 0.5 * (args.pile.y0 + args.pile.y1) : args.centre_y;
        m_pivot_x             = pivot(centre_x, fp.w, m_last_x);
        m_pivot_y             = pivot(centre_y, fp.h, m_last_y);
        m_runs                = row_runs(fp, m_board.vertical_gap);
    }

    // The best hit in the rows of `band` of `bands`.
    Hit scan_band(int band, int bands)
    {
        for (int y = m_pivot_y + band; y <= m_last_y; y += bands) {
            if (row_cannot_win(y))
                break;
            scan_row(y);
        }
        for (int y = m_pivot_y - 1 - band; y >= 0; y -= bands) {
            if (row_cannot_win(y))
                break;
            scan_row(y);
        }
        return m_best;
    }

private:
    // The lowest score a position with envelope `env` can reach with `cells` cells available for contact.
    double best_possible(double env, int cells) const
    {
        return env - m_args.contact_value * std::min(m_fp.halo_cells, cells);
    }

    // The pivot column minimises the envelope of every row, so it bounds the row, and every row further
    // out from the pivot row has a larger bound still.
    bool row_cannot_win(int y) const
    {
        return best_possible(m_args.envelope(m_pivot_x, y, m_fp.w, m_fp.h), m_fp.halo_cells) > m_limit;
    }

    // Every column of row y, outward from the pivot column: rightward, then leftward.
    void scan_row(int y)
    {
        for (int x = m_pivot_x; x <= m_last_x;) {
            const Step step = try_position(x, y, +1);
            if (step.side_done)
                break;
            x += step.columns;
        }
        for (int x = m_pivot_x - 1; x >= 0;) {
            const Step step = try_position(x, y, -1);
            if (step.side_done)
                break;
            x -= step.columns;
        }
    }

    // Judges the position (x, y), records it when it fits, and says how the scan goes on in direction
    // `dir` along the row.
    Step try_position(int x, int y, int dir)
    {
        const int    w = m_fp.w, h = m_fp.h;
        const double env   = m_args.envelope(x, y, w, h);
        const double limit = m_limit.load(std::memory_order_relaxed);
        // The envelope only grows further out along the row, so once it alone is over the limit the
        // rest of this side cannot win.
        if (best_possible(env, m_fp.halo_cells) > limit)
            return {true, 0};
        // Contact needs occupied cells, which lie inside the footprint's rect grown by one cell, or the
        // bed's edge: at most h cells per side edge the rect touches and w per top or bottom edge.
        const int around = m_board.occupied_in(x - 1, y - 1, x + w + 1, y + h + 1);
        const int edges  = h * ((x == 0) + (x == m_last_x)) + w * ((y == 0) + (y == m_last_y));
        if (best_possible(env, around + edges) > limit)
            return skip_to_more_contact(x, y, dir);
        // A run over n blocked cells stays over at least one of them for the next n - 1 positions.
        for (const Rect &run : m_runs) {
            const int under = m_blocked.count(y + run.y0, x + run.x0, x + run.x1);
            if (under > 0)
                return {false, under};
        }
        if (m_stacking && around > 0 && !m_board.fits(m_fp, x, y))
            return {false, 1};
        record_hit(x, y, env);
        return {false, 1};
    }

    // Reached when even full contact with everything around the footprint would not beat the limit.
    // With nothing occupied beyond the grown rect in its rows on side `dir`, each step only sheds
    // occupied cells from it until the bed's edge, whose contact may count again: the scan goes straight
    // there, or is done with this side when it is already there.
    Step skip_to_more_contact(int x, int y, int dir) const
    {
        const int w = m_fp.w, h = m_fp.h;
        const int beyond = dir > 0 ? m_board.occupied_in(x + w + 1, y - 1, m_board.w, y + h + 1)
                                   : m_board.occupied_in(0, y - 1, x - 1, y + h + 1);
        if (beyond > 0)
            return {false, 1};
        const int to_edge = dir > 0 ? m_last_x - x : x;
        return {to_edge == 0, to_edge};
    }

    void record_hit(int x, int y, double env)
    {
        const double dx = x + 0.5 * m_fp.w - m_args.centre_x;
        const double dy = y + 0.5 * m_fp.h - m_args.centre_y;
        const double score = env - m_args.contact_value * m_board.contact(m_fp, x, y);
        const Hit    hit{score, dx * dx + dy * dy, y, x, m_candidate};
        lower_limit(m_limit, hit.score);
        m_best = std::min(m_best, hit);
    }

    const Footprint     &m_fp;
    int                  m_candidate;
    const SearchArgs    &m_args;
    const Board         &m_board;
    const BlockedCells  &m_blocked;
    std::atomic<double> &m_limit;
    bool                 m_stacking;
    int                  m_last_x, m_last_y; // the last position on each axis that keeps the footprint on the bed
    int                  m_pivot_x = 0, m_pivot_y = 0;
    std::vector<Rect>    m_runs;
    Hit                  m_best;
};

} // namespace

Hit search(const std::vector<Footprint> &fps, const SearchArgs &args, const ParallelFor &pf)
{
    const BlockedCells  blocked(args.board, pf);
    const int           bands = std::clamp(args.board.h / 32, 1, 8);
    std::atomic<double> limit{Inf};
    std::vector<Hit>    hits(fps.size() * size_t(bands)); // per candidate and band
    pf(hits.size(), [&](size_t task) {
        const int        candidate = int(task) / bands;
        const int        band      = int(task) % bands;
        const Footprint &fp        = fps[size_t(candidate)];
        if (fp.usable && fp.w <= args.board.w && fp.h <= args.board.h)
            hits[task] = FootprintScan(fp, candidate, args, blocked, limit).scan_band(band, bands);
    });
    return *std::min_element(hits.begin(), hits.end());
}

}} // namespace Slic3r::nest3d
