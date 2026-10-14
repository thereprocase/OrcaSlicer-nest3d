// Placement: greedy passes over a few part orderings. Each part goes into the smallest enclosed pocket
// that holds it, else where the pile's envelope grows least, with contact as a bonus. Every pass is
// centred on the anchor, and the best pass wins.

#include "Grid.hpp"
#include "Lattice.hpp"
#include "Nest3D.hpp"
#include "Pockets.hpp"
#include "Search.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <functional>
#include <numeric>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace Slic3r { namespace nest3d {
namespace {

// Area in mm^2 that 1 mm of contact outweighs: small next to the area a part adds by moving a cell, so
// contact mostly decides between positions of equal area.
constexpr double ContactMM = 16.;
// The share of bbox area pass selection may give up for a pile nearer the anchor.
constexpr double CentreAllowance = 0.05;
constexpr int    Mixed           = -2; // group of a plate whose fixed parts belong to several groups
// ---------------------------------------------------------------------------------------------------
// Passes
// ---------------------------------------------------------------------------------------------------

// Box grows the pile's bbox least, Ring grows it round about its weighted centroid, Lattice is Box with
// copies seeded on lattices, and Corner is Box grown from the bed's min corner.
enum class Mode { Box, Ring, Lattice, Corner };

struct Plate
{
    int   index = 0;
    int   group = -1; // -1 while empty
    Board board;
    Rect  bbox; // of this run's parts
    // The weighted centroid of the parts, kept as sums: each part's weight times its area, and that
    // times its centre on each axis.
    double                 weight = 0., weighted_x = 0., weighted_y = 0.;
    std::vector<int>       placed;
    std::optional<Pockets> pockets; // labelled on demand, dropped by every placement
};

// How passes compare, best first: fewer parts left out, then fewer plates, then the earlier plates
// fuller, so parts overflowing a plate leave it as full as a single-plate arrange would.
struct PassRank
{
    int              unplaced = 0;
    int              plates   = 0;
    std::vector<int> parts_on_plate; // per plate, in plate order

    bool operator<(const PassRank &o) const
    {
        if (unplaced != o.unplaced)
            return unplaced < o.unplaced;
        if (plates != o.plates)
            return plates < o.plates;
        for (size_t i = 0; i < parts_on_plate.size() && i < o.parts_on_plate.size(); ++i)
            if (parts_on_plate[i] != o.parts_on_plate[i])
                return parts_on_plate[i] > o.parts_on_plate[i];
        return parts_on_plate.size() < o.parts_on_plate.size();
    }
    bool operator==(const PassRank &o) const
    {
        return unplaced == o.unplaced && plates == o.plates && parts_on_plate == o.parts_on_plate;
    }
};

struct Pass
{
    std::vector<Spot>             spots; // per part
    std::vector<Placement>        out;   // per part, once finished
    std::vector<Plate>            plates;
    std::set<std::pair<int, int>> full;   // (shape, plate) pairs without room for the shape
    std::set<std::pair<int, int>> seeded; // (shape, plate) pairs with the shape's lattice laid
    PassRank                      rank;
    double                        cost   = 0.; // the bbox area of every plate
    double                        moment = 0.; // the parts' weighted second moment about the anchor
};

struct PassPlan
{
    std::vector<int> order;
    Mode             mode;
};

class Arranger
{
public:
    Arranger(const std::vector<std::vector<Candidate>> &shapes, const std::vector<Part> &parts,
             const std::vector<Obstacle> &obstacles, const Params &params)
        : m_shapes(shapes), m_parts(parts), m_params(params), m_footprints(shapes.size()), m_units(shapes.size())
        , m_bed_w(std::max(0, int(std::floor(params.bed_w / params.cell_mm + 1e-6))))
        , m_bed_h(std::max(0, int(std::floor(params.bed_h / params.cell_mm + 1e-6))))
        , m_anchor_x(anchored() ? std::clamp(params.align_x, 0., 1.) * m_bed_w : 0.)
        , m_anchor_y(anchored() ? std::clamp(params.align_y, 0., 1.) * m_bed_h : 0.)
    {
        if (!m_params.parallel_for)
            m_params.parallel_for = [](size_t n, const std::function<void(size_t)> &fn) {
                for (size_t i = 0; i < n; ++i)
                    fn(i);
            };
        m_params.parallel_for(shapes.size(), [&](size_t s) {
            for (const Candidate &c : shapes[s])
                m_footprints[s].push_back(make_footprint(c.map, params.max_height, params.vertical_gap));
        });
        for (const Part &part : parts)
            m_tallest = std::max(m_tallest, double(m_footprints[part.shape][0].top));
        make_fixed_plates(obstacles);
        make_lattice_units();
    }

    std::vector<Placement> run()
    {
        const std::vector<PassPlan> plan = plan_passes();
        std::vector<Pass>           passes(plan.size());
        m_progress_total = int((plan.size() + 1) * m_parts.size()); // + 1: the corner pass, when it runs
        m_params.parallel_for(plan.size(), [&](size_t i) { passes[i] = greedy(plan[i].order, plan[i].mode); });
        // When the best pass leaves parts out or needs several plates, a pass grown from the bed's corner
        // may still fit them: parts that only fit pushed into opposite corners.
        const PassRank &lead = best_rank(passes);
        if (anchored() && (lead.unplaced > 0 || lead.plates > 1))
            passes.push_back(greedy(plan[0].order, Mode::Corner));
        return choose(passes).out;
    }

private:
    bool             anchored() const { return m_params.align_x >= 0.; }
    const Footprint &footprint(int part, const Spot &spot) const
    {
        return m_footprints[m_parts[part].shape][spot.candidate];
    }
    double           weight(const Footprint &fp) const { return 1. + fp.top / std::max(m_tallest, 1e-9); }
    int              own_plates() const { return int(m_fixed_plates.size()) - 1; }
    // The empty plate with the obstacles of plate `index`: its own, or the shared ones past those.
    const Plate &fixed_plate(int index) const
    {
        return m_fixed_plates[std::min(size_t(index), m_fixed_plates.size() - 1)];
    }

    // Empty plates with the obstacles of the plates that have their own, then one with the obstacles every
    // plate shares, for the rest.
    void make_fixed_plates(const std::vector<Obstacle> &obstacles)
    {
        int own = 0;
        for (const Obstacle &o : obstacles)
            own = std::max(own, o.plate + 1);
        m_fixed_plates.resize(size_t(own) + 1);
        m_params.parallel_for(m_fixed_plates.size(), [&](size_t k) {
            m_fixed_plates[k] = plate_with_obstacles(obstacles, int(k) < own ? int(k) : -1);
        });
    }

    // Lattice units for every shape with copies, of which a plate can hold at least two.
    void make_lattice_units()
    {
        std::vector<int> copies(m_shapes.size());
        for (const Part &part : m_parts)
            ++copies[size_t(part.shape)];
        m_params.parallel_for(m_shapes.size(), [&](size_t s) {
            if (copies[s] > 1 && 2 * m_footprints[s][0].cells <= m_bed_w * m_bed_h)
                m_units[s] = lattice_units(m_footprints[s], m_shapes[s], m_params);
        });
    }

    // A box pass per distinct part ordering, a ring pass tallest first and a lattice pass when there are
    // copies, all run at once.
    std::vector<PassPlan> plan_passes() const
    {
        std::vector<PassPlan> plan;
        for (int k = 0; k < 6; ++k) {
            std::vector<int> order  = part_order(k);
            const auto       same   = [&](const PassPlan &p) { return p.order == order; };
            if (std::none_of(plan.begin(), plan.end(), same))
                plan.push_back({std::move(order), Mode::Box});
        }
        if (anchored())
            plan.push_back({part_order(1), Mode::Ring});
        const auto has_units = [](const std::vector<Unit> &u) { return !u.empty(); };
        if (std::any_of(m_units.begin(), m_units.end(), has_units))
            plan.push_back({plan[0].order, Mode::Lattice});
        return plan;
    }

    static const PassRank &best_rank(const std::vector<Pass> &passes)
    {
        const auto by_rank = [](const Pass &a, const Pass &b) { return a.rank < b.rank; };
        return std::min_element(passes.begin(), passes.end(), by_rank)->rank;
    }

    // Of the passes ranked best, the one with the least moment among those within CentreAllowance of
    // the least cost; the earlier pass on a tie.
    static const Pass &choose(const std::vector<Pass> &passes)
    {
        const PassRank &rank       = best_rank(passes);
        double          least_cost = Inf;
        for (const Pass &pass : passes)
            if (pass.rank == rank)
                least_cost = std::min(least_cost, pass.cost);
        const Pass *best = nullptr;
        for (const Pass &pass : passes) {
            if (!(pass.rank == rank) || pass.cost > (1. + CentreAllowance) * least_cost)
                continue;
            if (!best || pass.moment < best->moment)
                best = &pass;
        }
        return *best;
    }

    // Part order k: priority first, then size key k, then the other size keys in turn, then the id, so
    // the order never follows the input order (the selection order in the GUI). With stacking, the first
    // size key counts only the cells with no room under them, so a part that leaves room under it (a
    // table) comes after the parts that could go there.
    std::vector<int> part_order(int k) const
    {
        using Key = std::pair<std::array<double, 7>, int>;
        auto key  = [&](int i) {
            const Footprint &fp = m_footprints[m_parts[i].shape][0];
            const double     longest      = std::max(fp.w, fp.h);
            const double     size_keys[6] = {-double(fp.floor_cells), -fp.top,              -longest,
                                             -fp.volume,              -double(fp.w + fp.h), -double(fp.w) * fp.h};
            Key              key{{-m_parts[i].priority, size_keys[k]}, m_parts[i].id};
            size_t           next = 2;
            for (int j = 0; j < 6; ++j)
                if (j != k)
                    key.first[next++] = size_keys[j];
            return key;
        };
        std::vector<int> order(m_parts.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int a, int b) { return key(a) < key(b); });
        return order;
    }

    // An empty plate with the obstacles of plate `index`, or with those on every plate for -1, each
    // obstacle cell resampled onto every bed cell it overlaps; it takes the group of its fixed parts.
    Plate plate_with_obstacles(const std::vector<Obstacle> &obstacles, int index) const
    {
        Plate plate{index, -1, Board(m_bed_w, m_bed_h, m_params.vertical_gap)};
        for (const Obstacle &o : obstacles) {
            if (o.plate != index && o.plate >= 0)
                continue;
            if (o.plate >= 0 && o.group >= 0)
                plate.group = plate.group == -1 || plate.group == o.group ? o.group : Mixed;
            const HeightMap &map = o.map;
            for (int y = 0; y < map.height; ++y)
                for (int x = 0; x < map.width; ++x) {
                    const Span &s = map.at(x, y);
                    if (s.empty())
                        continue;
                    const int by0 = cell_at(map.origin_y + y * map.cell_mm);
                    const int by1 = cell_ending_at(map.origin_y + (y + 1) * map.cell_mm, m_bed_h);
                    const int bx0 = cell_at(map.origin_x + x * map.cell_mm);
                    const int bx1 = cell_ending_at(map.origin_x + (x + 1) * map.cell_mm, m_bed_w);
                    for (int by = by0; by <= by1; ++by)
                        for (int bx = bx0; bx <= bx1; ++bx)
                            plate.board.add(bx, by, s);
                }
        }
        plate.board.update_summed_area(0, m_bed_h);
        return plate;
    }

    // The bed cell holding the coordinate mm, and the one ending there, both clamped to a bed of n cells.
    int cell_at(double mm) const { return std::max(0, int(std::floor(mm / m_params.cell_mm + 1e-6))); }
    int cell_ending_at(double mm, int n) const
    {
        return std::min(n - 1, int(std::floor(mm / m_params.cell_mm - 1e-6)));
    }

    void place(int part, const Spot &spot, Plate &plate, Pass &pass) const
    {
        const Footprint &fp   = footprint(part, spot);
        const double     mass = weight(fp) * fp.cells;
        plate.board.place(fp, spot.x, spot.y);
        plate.bbox = plate.bbox.united({spot.x, spot.y, spot.x + fp.w, spot.y + fp.h});
        plate.weight += mass;
        plate.weighted_x += mass * (spot.x + 0.5 * fp.w);
        plate.weighted_y += mass * (spot.y + 0.5 * fp.h);
        plate.group = m_parts[part].group;
        plate.placed.push_back(part);
        plate.pockets.reset();
        pass.spots[part] = spot;
    }

    // Part order[k] on `plate`: into a pocket, else with its copies seeded on a lattice, else where the
    // search puts it. False when it does not fit the plate.
    bool place_on(const std::vector<int> &order, size_t k, Plate &plate, Pass &pass, Mode mode)
    {
        const int                     part     = order[k];
        const int                     shape    = m_parts[part].shape;
        const std::vector<Footprint> &fps      = m_footprints[shape];
        const double                  centre_x = mode == Mode::Corner ? 0. : m_anchor_x;
        const double                  centre_y = mode == Mode::Corner ? 0. : m_anchor_y;
        Spot                          spot;
        if (!plate.bbox.empty()) {
            if (!plate.pockets)
                plate.pockets = find_pockets(plate.board, plate.bbox);
            spot = pocket_search(fps, plate.board, *plate.pockets);
        }
        if (spot.candidate < 0 && mode == Mode::Lattice && !m_units[shape].empty()) {
            // The first copy of a shape to reach a plate that no pocket takes lays its unplaced copies there.
            const bool first_copy_here = pass.seeded.insert({shape, plate.index}).second;
            if (first_copy_here && seed_lattice(order, k, plate, pass, centre_x, centre_y))
                return true;
        }
        if (spot.candidate < 0) {
            const double contact_value = ContactMM / m_params.cell_mm;
            SearchArgs   args{plate.board, plate.bbox, centre_x, centre_y, mode == Mode::Ring, contact_value};
            if (mode == Mode::Ring && plate.weight > 0.) {
                args.centre_x = plate.weighted_x / plate.weight;
                args.centre_y = plate.weighted_y / plate.weight;
            }
            const Hit hit = search(fps, args, m_params.parallel_for);
            spot          = {hit.candidate, hit.x, hit.y};
        }
        if (spot.candidate >= 0)
            place(part, spot, plate, pass);
        return spot.candidate >= 0;
    }

    // Lays the unplaced copies of part order[k]'s shape, from it on, on the plate's lattice. True when at
    // least one copy was laid.
    bool seed_lattice(const std::vector<int> &order, size_t k, Plate &plate, Pass &pass, double centre_x,
                      double centre_y)
    {
        const Part      &first = m_parts[order[k]];
        std::vector<int> copies;
        for (size_t j = k; j < order.size(); ++j) {
            const Part &part = m_parts[order[j]];
            if (part.shape == first.shape && part.group == first.group && pass.spots[order[j]].candidate < 0)
                copies.push_back(order[j]);
        }
        const std::vector<Spot> spots = lay_lattice(m_units[first.shape], m_footprints[first.shape], plate.board,
                                                    plate.bbox, centre_x, centre_y, int(copies.size()),
                                                    m_params.parallel_for);
        for (size_t j = 0; j < spots.size(); ++j)
            place(copies[j], spots[j], plate, pass);
        return !spots.empty();
    }

    // First fit over `order`. A (shape, plate) pair that found no room is not tried again: the board only
    // fills up.
    Pass greedy(const std::vector<int> &order, Mode mode)
    {
        Pass pass;
        pass.spots.resize(m_parts.size());
        for (const Part &part : m_parts)
            pass.out.push_back({part.id});
        pass.plates.assign(m_fixed_plates.begin(), m_fixed_plates.end() - 1);
        for (size_t k = 0; k < order.size(); ++k) {
            if (m_params.should_stop && m_params.should_stop())
                break;
            if (pass.spots[order[k]].candidate < 0) // not laid already as a copy of an earlier part
                place_on_any_plate(order, k, pass, mode);
            if (m_params.progress)
                m_params.progress(++m_progress_done, m_progress_total);
        }
        finish(pass);
        return pass;
    }

    // The first open plate of the part's group with room, else a new plate, which stays only with a part on it.
    void place_on_any_plate(const std::vector<int> &order, size_t k, Pass &pass, Mode mode)
    {
        const Part &part = m_parts[order[k]];
        for (int p = 0; p < m_params.max_plates; ++p) {
            if (p == int(pass.plates.size())) {
                pass.plates.push_back(fixed_plate(p));
                pass.plates.back().index = p;
            }
            Plate     &plate      = pass.plates[p];
            const bool compatible = plate.group == -1 || plate.group == part.group;
            if (compatible && !pass.full.count({part.shape, p}) && !place_on(order, k, plate, pass, mode))
                pass.full.insert({part.shape, p});
            if (pass.spots[order[k]].candidate >= 0)
                return;
            if (plate.placed.empty() && p >= own_plates()) {
                pass.plates.pop_back();
                return;
            }
        }
    }

    // Moves the plate's parts as one so that their bbox centres on the anchor, halving the shift until every
    // part fits the plate's fixed board: a pile never moves onto an exclusion zone, the bed's outline or a
    // fixed part, and its own parts move together, so they cannot collide.
    void recentre(Plate &plate, std::vector<Spot> &spots) const
    {
        auto shift_to_anchor = [](double anchor, int bed, int lo, int hi) {
            return std::clamp(int(std::lround(anchor - 0.5 * (lo + hi))), -lo, bed - hi);
        };
        int  dx    = shift_to_anchor(m_anchor_x, m_bed_w, plate.bbox.x0, plate.bbox.x1);
        int  dy    = shift_to_anchor(m_anchor_y, m_bed_h, plate.bbox.y0, plate.bbox.y1);
        auto clear = [&] {
            const Board &obstacles = fixed_plate(plate.index).board;
            const auto   fits      = [&](int i) {
                return obstacles.fits(footprint(i, spots[i]), spots[i].x + dx, spots[i].y + dy);
            };
            return std::all_of(plate.placed.begin(), plate.placed.end(), fits);
        };
        while ((dx != 0 || dy != 0) && !clear()) {
            dx /= 2;
            dy /= 2;
        }
        for (int i : plate.placed) {
            spots[i].x += dx;
            spots[i].y += dy;
        }
    }

    // A pass's rank, its cost and, once centred, its placements and its moment about the anchor.
    void finish(Pass &pass) const
    {
        const auto unplaced = [](const Spot &s) { return s.candidate < 0; };
        pass.rank.unplaced  = int(std::count_if(pass.spots.begin(), pass.spots.end(), unplaced));
        for (Plate &plate : pass.plates) {
            if (!plate.placed.empty())
                pass.rank.plates = plate.index + 1;
            pass.rank.parts_on_plate.push_back(int(plate.placed.size()));
            pass.cost += double(plate.bbox.area());
            if (anchored() && !plate.placed.empty())
                recentre(plate, pass.spots);
            for (int i : plate.placed) {
                const Spot      &spot = pass.spots[i];
                const Footprint &fp   = footprint(i, spot);
                const Candidate &c    = m_shapes[m_parts[i].shape][spot.candidate];
                const double     dx   = spot.x + 0.5 * fp.w - m_anchor_x;
                const double     dy   = spot.y + 0.5 * fp.h - m_anchor_y;
                pass.moment += weight(fp) * fp.cells * (dx * dx + dy * dy);
                // The bed cell of the map's cell (0, 0) puts the reference point on the bed.
                pass.out[i] = {m_parts[i].id, plate.index, (spot.x - fp.x0) * m_params.cell_mm - c.map.origin_x,
                               (spot.y - fp.y0) * m_params.cell_mm - c.map.origin_y, c.rotation_z};
            }
            plate.board = Board(); // a finished pass keeps its spots only
            plate.pockets.reset();
        }
    }

    const std::vector<std::vector<Candidate>> &m_shapes;
    const std::vector<Part>                   &m_parts;
    Params                                     m_params;
    std::vector<std::vector<Footprint>>        m_footprints;   // per shape, per candidate
    std::vector<std::vector<Unit>>             m_units;        // per shape: its lattice units, when it has copies
    // Empty plates 0, 1, ... with their own obstacles, then one with the obstacles every plate shares.
    std::vector<Plate>                         m_fixed_plates;
    int                                        m_bed_w, m_bed_h; // in cells
    double                                     m_anchor_x, m_anchor_y; // in cells
    double                                     m_tallest = 0.;         // the tallest part, mm
    int                                        m_progress_total = 0;
    std::atomic<int>                           m_progress_done{0};
};

} // namespace

std::vector<Placement> arrange(const std::vector<std::vector<Candidate>> &shapes, const std::vector<Part> &parts,
                               const std::vector<Obstacle> &obstacles, const Params &params)
{
    return Arranger(shapes, parts, obstacles, params).run();
}

}} // namespace Slic3r::nest3d
