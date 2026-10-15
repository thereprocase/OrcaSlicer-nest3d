#include <catch2/catch_test_macros.hpp>
#include "libslic3r/GCode/RetractionLimiter.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <initializer_list>
#include <string>

using Slic3r::RetractionLimiter;

TEST_CASE("Retraction budget recovers only as fresh filament advances", "[RetractionLimiter]")
{
    RetractionLimiter limiter;
    REQUIRE(limiter.allows(10., 2, 1.));
    limiter.record();
    REQUIRE(limiter.allows(10.2, 2, 1.));
    limiter.record();
    CHECK_FALSE(limiter.allows(10.2, 2, 1.));
    CHECK_FALSE(limiter.allows(9.9, 2, 1.)); // Retract, then unretract the same stock.
    CHECK_FALSE(limiter.allows(10.2, 2, 1.));
    CHECK_FALSE(limiter.allows(10.99, 2, 1.));
    REQUIRE(limiter.allows(11., 2, 1.)); // The first cycle leaves the window.
    limiter.record();
    CHECK_FALSE(limiter.allows(11., 2, 1.)); // The second cycle still occupies it.
    CHECK(limiter.allows(11.21, 2, 1.));
}

TEST_CASE("Disabled limits and reloads clear retraction history", "[RetractionLimiter]")
{
    RetractionLimiter limiter;
    REQUIRE(limiter.allows(1., 1, 2.));
    limiter.record();
    CHECK_FALSE(limiter.allows(1., 1, 2.));
    CHECK(limiter.allows(1., 0, 2.));
    REQUIRE(limiter.allows(1., 1, 2.));
    limiter.record();
    CHECK(limiter.allows(1., 1, 0.));
    REQUIRE(limiter.allows(1., 1, 2.));
    limiter.record();
    limiter.reset();
    CHECK(limiter.allows(0., 1, 2.));
}

TEST_CASE("Hard spacing survives soft window expiry and overrides quality exceptions", "[RetractionLimiter]")
{
    RetractionLimiter limiter;
    REQUIRE(limiter.allows(5., 1, 0.1));
    REQUIRE(limiter.allows_minimum(5., 0.3));
    limiter.record();
    CHECK(limiter.allows(5.2, 1, 0.1)); // Soft window has expired.
    CHECK_FALSE(limiter.allows_minimum(5.2, 0.3)); // Hard spacing has not.
    CHECK_FALSE(limiter.allows_minimum(4.9, 0.3));
    CHECK(limiter.allows_minimum(5.31, 0.3));
}

TEST_CASE("Filaments inherit printer retraction guards unless explicitly overridden", "[RetractionLimiter][Config]")
{
    auto config = Slic3r::DynamicPrintConfig::full_print_config();
    for (const char *key : {"filament_retraction_count_max", "filament_retraction_extrusion_window", "filament_retraction_minimum_extrusion"}) {
        REQUIRE(config.option(key, true) != nullptr);
        CHECK(config.option(key)->is_nil());
    }
    Slic3r::DynamicPrintConfig updated = config;
    updated.set_deserialize_strict({{"retraction_count_max", "3"}, {"retraction_minimum_extrusion", "0.3"},
                                   {"filament_retraction_count_max", "2"}, {"filament_retraction_minimum_extrusion", "0.5"}});
    const auto *override_count = updated.option("filament_retraction_count_max");
    CHECK_FALSE(override_count->is_nil());
    CHECK(override_count->serialize() == "2");
}
