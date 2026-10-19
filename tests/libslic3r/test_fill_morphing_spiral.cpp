#include "libslic3r/Fill/FillMorphingSpiral.hpp"
#include "libslic3r/Point.hpp"

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cstddef>
#include <vector>

TEST_CASE("Accordion scan lines keep interior samples across tangent vertices", "[FillMorphingSpiral]")
{
    using Slic3r::Vec2d;
    const std::vector<std::vector<Vec2d>> loops{
        {{0, -2}, {4, -2}, {4, 2}, {3, 2}, {2, 0}, {1, 2}, {0, 2}},
        {{0, -2}, {1, -2}, {2, 0}, {3, -2}, {4, -2}, {4, 2}, {0, 2}},
        {{0, 0}, {2, -2}, {4, 0}, {2, 2}},
        {{0, -2}, {4, -2}, {4, 2}, {3, 2}, {3, 0}, {1, 0}, {1, 2}, {0, 2}}
    };
    for (auto loop : loops) {
        // Both winding orders and every choice of first vertex must classify alike.
        for (int winding = 0; winding < 2; ++winding) {
            for (std::size_t start = 0; start < loop.size(); ++start) {
                auto hits = Slic3r::morphing_spiral_detail::line_crossings(loop, Vec2d(1, 0), Vec2d(0, 1), 0);
                std::sort(hits.begin(), hits.end());
                const auto inside = [&hits](double x) {
                    return (std::upper_bound(hits.begin(), hits.end(), x) - hits.begin()) % 2 == 1;
                };
                REQUIRE(inside(0.5));
                REQUIRE(inside(3.5));
                REQUIRE_FALSE(inside(-0.5));
                REQUIRE_FALSE(inside(4.5));
                std::rotate(loop.begin(), loop.begin() + 1, loop.end());
            }
            std::reverse(loop.begin(), loop.end());
        }
    }
}
