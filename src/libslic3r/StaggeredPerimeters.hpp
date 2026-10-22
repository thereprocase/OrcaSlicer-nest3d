#pragma once

#include "ExtrusionEntity.hpp"
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

namespace Slic3r {

// Closed wall tracks only. Unsupported/open/changing tracks remain nominal.
struct StaggeredWall {
    ExtrusionPaths *paths;
    size_t region;
    int inset;
    double print_z;
    double height;
    double maximum_bead_height = std::numeric_limits<double>::max();
};

// Input is ordered by object layer, not by global print layer or raft id.
// Returns the number of closed walls whose bead geometry was changed.
size_t stagger_perimeters(const std::vector<std::vector<StaggeredWall>> &layers,
                          const std::function<void()> &cancel = [] {});

} // namespace Slic3r
