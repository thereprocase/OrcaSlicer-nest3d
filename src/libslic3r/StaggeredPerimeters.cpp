#include "StaggeredPerimeters.hpp"
#include "Point.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <utility>
#include <tuple>

namespace Slic3r {
namespace {
using Edge = std::array<coord_t, 5>;
struct Key {
    size_t region;
    int inset;
    std::vector<Edge> edges;
    bool operator<(const Key &b) const {
        return std::tie(region, inset, edges) < std::tie(b.region, b.inset, b.edges);
    }
};

// No fuzzy matching across slopes or changed topology: equal bead footprints
// form a track, independent of its starting vertex or traversal direction.
Key key(const StaggeredWall &wall)
{
    Key out{wall.region, wall.inset, {}};
    if (wall.inset < 0 || wall.inset % 2 == 0 || wall.paths->empty() || 1.5 * wall.height > wall.maximum_bead_height + 1e-6)
        return out;
    for (const ExtrusionPath &path : *wall.paths)
        if (path.polyline.points.size() < 2) return out;
    if (wall.paths->front().first_point() != wall.paths->back().last_point())
        return out;
    for (size_t i = 1; i < wall.paths->size(); ++i)
        if ((*wall.paths)[i-1].last_point() != (*wall.paths)[i].first_point())
            return out;
    for (const ExtrusionPath &path : *wall.paths) {
        if (path.role() != erPerimeter || path.z_contoured || path.height <= 0 ||
            path.width < 1.5 * wall.height || std::abs(path.height - wall.height) > 1e-6)
            return {wall.region, wall.inset, {}};
        const auto &points = path.polyline.points;
        for (size_t i = 1; i < points.size(); ++i) {
            auto a = std::make_pair(points[i-1].x(), points[i-1].y());
            auto b = std::make_pair(points[i].x(), points[i].y());
            if (b < a) std::swap(a, b);
            if (a != b)
                out.edges.push_back({a.first, a.second, b.first, b.second, static_cast<coord_t>(scale_(path.width))});
        }
    }
    std::sort(out.edges.begin(), out.edges.end());
    return out;
}

double area(double width, double height)
{
    // Same rounded-rectangle cross section as Flow; do not scale height
    // linearly into volume at the 1.5-layer and half-layer transitions.
    return height * (width - height * (1.0 - 0.25 * std::acos(-1.0)));
}
} // namespace

size_t stagger_perimeters(const std::vector<std::vector<StaggeredWall>> &layers,
                          const std::function<void()> &cancel)
{
    struct Track { size_t last_layer; std::vector<const StaggeredWall *> walls; };
    std::map<Key, Track> active;
    size_t changed = 0;
    const auto finish = [&](Track &track) {
        if (track.walls.size() < 4) return;
        double previous_offset = 0.;
        for (size_t i = 0; i < track.walls.size(); ++i) {
            const StaggeredWall &wall = *track.walls[i];
            const double offset = i > 0 && i + 2 < track.walls.size() ? wall.height * 0.5 : 0.;
            const double bead_height = wall.height + offset - previous_offset;
            if (offset != 0. || previous_offset != 0.) {
                for (ExtrusionPath &path : *wall.paths) {
                    path.mm3_per_mm *= area(path.width, bead_height) / area(path.width, wall.height);
                    path.height = static_cast<float>(bead_height);
                    path.staggered_z_offset = static_cast<float>(offset);
                }
                ++changed;
            }
            previous_offset = offset;
        }
    };
    for (size_t layer = 0; layer < layers.size(); ++layer) {
        cancel();
        std::map<Key, std::vector<const StaggeredWall *>> current;
        for (const auto &wall : layers[layer]) {
            auto k = key(wall);
            if (!k.edges.empty()) current[std::move(k)].push_back(&wall);
        }
        for (auto it = active.begin(); it != active.end();) {
            const auto found = current.find(it->first);
            const auto &previous = *it->second.walls.back();
            if (found == current.end() || found->second.size() != 1 ||
                it->second.last_layer + 1 != layer ||
                std::abs(found->second.front()->height - previous.height) > 1e-6 ||
                std::abs(found->second.front()->print_z - previous.print_z - previous.height) > 1e-6) {
                finish(it->second);
                it = active.erase(it);
            } else ++it;
        }
        for (const auto &[k, walls] : current) {
            // Ambiguous coincident walls must not connect to an arbitrary track.
            if (walls.size() != 1) continue;
            auto &track = active[k];
            track.last_layer = layer;
            track.walls.push_back(walls.front());
        }
    }
    for (auto &[k, track] : active) { cancel(); finish(track); }
    return changed;
}
} // namespace Slic3r
