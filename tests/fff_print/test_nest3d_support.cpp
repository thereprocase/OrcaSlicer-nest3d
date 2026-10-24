#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Nest3D/Nest3DArrange.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/libslic3r.h"
#include "test_helpers.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;

namespace {

constexpr double Spacing = 0.8;

// A narrow stem carries a rectangular cap. Support expansion makes a real extrusion wing
// outside the cap's projected outline, which the old fill-to-bed approximation cannot see.
TriangleMesh supported_cap()
{
    TriangleMesh stem = make_cube(6, 6, 8);
    stem.translate(7, 3, 0);
    TriangleMesh cap = make_cube(20, 12, 2);
    cap.translate(0, 0, 7);
    stem.merge(cap);
    return stem;
}

DynamicPrintConfig support_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"nozzle_diameter", "0.4"},
        {"layer_height", "0.2"},
        {"initial_layer_print_height", "0.2"},
        {"enable_support", "1"},
        {"support_type", "normal(auto)"},
        {"support_style", "snug"},
        {"support_expansion", "5"},
        {"support_threshold_angle", "30"},
        {"support_on_build_plate_only", "1"},
        {"bridge_no_support", "0"},
        {"raft_layers", "0"},
        {"brim_type", "no_brim"},
        {"enable_prime_tower", "0"}
    });
    return config;
}

Points bed(double width, double height)
{
    return {Point(0, 0), Point(scaled(width), 0), Point(scaled(width), scaled(height)), Point(0, scaled(height))};
}

arrangement::ArrangePolygons items_of(Model &model, const DynamicPrintConfig &config)
{
    arrangement::ArrangePolygons items;
    for (ModelObject *object : model.objects)
        for (ModelInstance *instance : object->instances) {
            items.push_back(get_instance_arrange_poly(instance, config));
            items.back().inflation = scaled(Spacing / 2);
            items.back().itemid = int(items.size() - 1);
        }
    return items;
}

arrangement::ArrangeParams arrange_params()
{
    arrangement::ArrangeParams params;
    params.progressind = nullptr;
    params.allow_rotations = false;
    params.printable_height = 100;
    return params;
}

nest3d::ArrangeSettings support_settings()
{
    nest3d::ArrangeSettings settings;
    settings.enabled = true;
    settings.generate_supports = true;
    return settings;
}

// Slice the arranged model independently, including its copied/rotated instances. Width-covered
// polygons measure occupied extrusion beads rather than just their centre lines.
std::vector<Polygons> printed_footprints(Model &model, const DynamicPrintConfig &config, bool include_model = true)
{
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    std::vector<Polygons> footprints;
    for (const PrintObject *object : print.objects()) {
        Polygons support;
        for (const SupportLayer *layer : object->support_layers())
            layer->support_fills.polygons_covered_by_width(support, 0.f);
        if (include_model)
            for (const Layer *layer : object->layers())
                for (const LayerRegion *region : layer->regions()) {
                    region->perimeters.polygons_covered_by_width(support, 0.f);
                    region->fills.polygons_covered_by_width(support, 0.f);
                }
        for (const PrintInstance &instance : object->instances()) {
            Polygons footprint = support;
            for (Polygon &polygon : footprint)
                polygon.translate(instance.shift);
            footprints.push_back(union_(footprint));
        }
    }
    return footprints;
}

void paint_cap_underside(ModelVolume &volume)
{
    TriangleSelector selector(volume.mesh());
    const indexed_triangle_set &mesh = volume.mesh().its;
    for (size_t i = 0; i < mesh.indices.size(); ++i) {
        const auto &face = mesh.indices[i];
        const Vec3f &a = mesh.vertices[face[0]];
        const Vec3f &b = mesh.vertices[face[1]];
        const Vec3f &c = mesh.vertices[face[2]];
        if (a.z() > 1 && (b - a).cross(c - a).z() < 0)
            selector.set_facet(int(i), EnforcerBlockerType::ENFORCER);
    }
    volume.supported_facets.set(selector);
}

} // namespace

TEST_CASE("Generated support reserves space outside the model outline", "[Nest3DSupport]")
{
    DynamicPrintConfig config = support_config();
    Model model = Test::model("supported cap", supported_cap());
    const auto footprints = printed_footprints(model, config, false);
    REQUIRE(footprints.size() == 1);
    REQUIRE_FALSE(footprints.front().empty());
    // The model is 20 x 12 mm. Its generated support cannot fit this 22 x 14 mm bed.
    CHECK(unscaled(get_extents(footprints.front()).size().x()) > 22);

    nest3d::ArrangeSettings settings;
    CHECK_FALSE(settings.generate_supports);
    settings.enabled = true;
    auto without = items_of(model, config);
    nest3d::arrange_polygons(without, {}, bed(22, 14), arrange_params(), settings, config, 1);
    REQUIRE(without.front().is_arranged());

    settings.generate_supports = true;
    auto with = items_of(model, config);
    nest3d::arrange_polygons(with, {}, bed(22, 14), arrange_params(), settings, config, 1);
    // In particular, the model-only outline fallback must not replace this valid rejection.
    CHECK_FALSE(with.front().is_arranged());
}

TEST_CASE("Generated support keeps copied and rotated parts inside the bed with bead clearance", "[Nest3DSupport]")
{
    DynamicPrintConfig config = support_config();
    Model model = Test::model("supported copies", supported_cap());
    model.objects.front()->add_instance()->set_rotation(Vec3d(0, 0, PI / 2));
    auto items = items_of(model, config);
    const Points outline = bed(75, 45);
    nest3d::arrange_polygons(items, {}, outline, arrange_params(), support_settings(), config, 1);
    for (auto &item : items) {
        REQUIRE(item.is_arranged());
        item.apply();
    }
    const auto footprints = printed_footprints(model, config);
    REQUIRE(footprints.size() == 2);
    const Polygons bed_polygon{Polygon(outline)};
    for (const Polygons &footprint : footprints)
        CHECK(diff(footprint, bed_polygon).empty());
    // A 0.8 mm setting is measured between bead edges, not support centre lines.
    CHECK(intersection(offset(footprints[0], scaled(Spacing - 0.001)), footprints[1]).empty());
}

TEST_CASE("Generated support respects object overrides and painted enforcers without editing them", "[Nest3DSupport]")
{
    const bool painted = GENERATE(false, true);
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({{"enable_support", "0"}, {"support_type", "normal(manual)"}});
    Model model = Test::model("painted support", supported_cap());
    ModelObject *object = model.objects.front();
    object->config.set_key_value("enable_support", new ConfigOptionBool(true));
    ModelVolume *volume = object->volumes.front();
    if (painted)
        paint_cap_underside(*volume);
    REQUIRE(object->is_fdm_support_painted() == painted);
    std::vector<std::string> annotation;
    for (size_t i = 0; i < volume->mesh().its.indices.size(); ++i)
        annotation.push_back(volume->supported_facets.get_triangle_as_string(int(i)));
    const auto transformation = object->instances.front()->get_matrix();
    const auto support = printed_footprints(model, config, false);
    REQUIRE(support.size() == 1);
    CHECK(support.front().empty() == !painted);
    const auto fixed = items_of(model, config);
    Model small_model = Test::model("short neighbor", make_cube(4, 4, 2));
    auto items = items_of(small_model, config);
    auto settings = support_settings();
    settings.allow_stacking = true;
    settings.vertical_gap_mm = 0.5f;
    nest3d::arrange_polygons(items, fixed, bed(22, 14), arrange_params(), settings, config, 1);
    CHECK(items.front().is_arranged() == !painted);
    CHECK(object->config.get().opt_bool("enable_support"));
    CHECK(object->instances.front()->get_matrix().isApprox(transformation));
    REQUIRE(object->volumes.size() == 1);
    for (size_t i = 0; i < annotation.size(); ++i)
        CHECK(volume->supported_facets.get_triangle_as_string(int(i)) == annotation[i]);
}

TEST_CASE("Generated raft clearance applies even when supports are disabled", "[Nest3DSupport]")
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({{"enable_support", "0"}, {"raft_layers", "3"}, {"raft_expansion", "5"}});
    Model model = Test::model("rafted cube", make_cube(12, 12, 4));
    const auto footprints = printed_footprints(model, config, false);
    REQUIRE(footprints.size() == 1);
    REQUIRE_FALSE(footprints.front().empty());
    REQUIRE(unscaled(get_extents(footprints.front()).size().x()) > 14);
    auto items = items_of(model, config);
    nest3d::arrange_polygons(items, {}, bed(14, 14), arrange_params(), support_settings(), config, 1);
    CHECK_FALSE(items.front().is_arranged());
}

TEST_CASE("Support-aware nesting keeps shrinkage-compensated unsupported parts clear", "[Nest3DSupport]")
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"enable_support", "0"},
        {"filament_shrink", "98%"},
        {"xy_contour_compensation", "0"},
        {"elefant_foot_compensation", "0"}
    });
    Model model = Test::model("compensated copies", make_cube(20, 20, 2));
    model.objects.front()->add_instance();
    auto items = items_of(model, config);
    const Points outline = bed(80, 50);
    nest3d::arrange_polygons(items, {}, outline, arrange_params(), support_settings(), config, 1);
    for (auto &item : items) {
        REQUIRE(item.is_arranged());
        item.apply();
    }
    const auto footprints = printed_footprints(model, config);
    REQUIRE(footprints.size() == 2);
    for (const Polygons &footprint : footprints) {
        CHECK_THAT(unscaled(get_extents(footprint).size().x()), Catch::Matchers::WithinAbs(20 / 0.98, 0.02));
        CHECK(diff(footprint, Polygons{Polygon(outline)}).empty());
    }
    CHECK(intersection(offset(footprints[0], scaled(Spacing - 0.001)), footprints[1]).empty());
}

TEST_CASE("Support-aware nesting reserves a model brim even when supports are disabled", "[Nest3DSupport]")
{
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"enable_support", "0"}, {"brim_type", "outer_only"},
        {"brim_width", "5"}, {"brim_object_gap", "0.1"}
    });
    Model model = Test::model("brimmed cube", make_cube(12, 12, 2));
    Print reference;
    reference.set_status_silent();
    reference.apply(model, config);
    reference.process();
    Polygons brim;
    for (const auto &entry : reference.get_brimMap())
        entry.second.polygons_covered_by_width(brim, 0.f);
    REQUIRE_FALSE(brim.empty());
    REQUIRE(unscaled(get_extents(brim).size().x()) > 14);

    auto items = items_of(model, config);
    nest3d::arrange_polygons(items, {}, bed(14, 14), arrange_params(), support_settings(), config, 1);
    CHECK_FALSE(items.front().is_arranged());
}

TEST_CASE("Support-aware nesting rejects differing used shrinkage but permits unused filament differences", "[Nest3DSupport]")
{
    const bool use_second_filament = GENERATE(false, true);
    DynamicPrintConfig config = Test::multifilament_config(2, {
        {"enable_support", "0"}, {"brim_type", "no_brim"},
        {"filament_shrink", "98%,100%"},
        {"outer_wall_filament_id", "1"}, {"inner_wall_filament_id", "1"},
        {"sparse_infill_filament_id", "1"}, {"internal_solid_filament_id", "1"},
        {"top_surface_filament_id", "1"}, {"bottom_surface_filament_id", "1"}
    });
    Model model = Test::model("first filament", make_cube(12, 12, 2));
    ModelObject *second = model.add_object(*model.objects.front());
    second->instances.front()->set_offset(Vec3d(25, 11, 0));
    if (use_second_filament)
        for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "sparse_infill_filament_id",
                               "internal_solid_filament_id", "top_surface_filament_id", "bottom_surface_filament_id"})
            second->config.set_key_value(key, new ConfigOptionInt(2));
    const auto source_transform = second->instances.front()->get_matrix();
    auto items = items_of(model, config);
    const auto original = items;

    // The opt-in guard must not change the existing arrangement path.
    auto ordinary = items;
    auto disabled = support_settings();
    disabled.generate_supports = false;
    REQUIRE_NOTHROW(nest3d::arrange_polygons(ordinary, {}, bed(60, 35), arrange_params(), disabled, config, 1));
    if (use_second_filament) {
        REQUIRE_THROWS_AS(nest3d::arrange_polygons(items, {}, bed(60, 35), arrange_params(), support_settings(), config, 1), SlicingError);
        REQUIRE(items.size() == original.size());
        for (size_t i = 0; i < items.size(); ++i) {
            CHECK(items[i].translation == original[i].translation);
            CHECK(items[i].bed_idx == original[i].bed_idx);
            CHECK_THAT(items[i].rotation, Catch::Matchers::WithinAbs(original[i].rotation, 1e-12));
        }
    } else {
        REQUIRE_NOTHROW(nest3d::arrange_polygons(items, {}, bed(60, 35), arrange_params(), support_settings(), config, 1));
        for (const auto &item : items)
            CHECK(item.is_arranged());
    }
    CHECK(second->instances.front()->get_matrix().isApprox(source_transform));
    for (const auto &item : items)
        CHECK(item.inflation == scaled(Spacing / 2));
}

TEST_CASE("Generated support blocks a fixed overhang that otherwise admits a short part", "[Nest3DSupport]")
{
    const bool supports = GENERATE(false, true);
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({{"support_expansion", "0"}});
    config.set_key_value("enable_support", new ConfigOptionBool(supports));
    Model fixed_model = Test::model("fixed cap", supported_cap());
    auto fixed = items_of(fixed_model, config);
    Model selected_model = Test::model("short part", make_cube(4, 4, 2));
    auto items = items_of(selected_model, config);
    auto settings = support_settings();
    settings.allow_stacking = true;
    settings.vertical_gap_mm = 0.5f;
    // The two-millimetre margin outside the cap cannot hold the four-millimetre cube.
    // It can only fit below the cap, where generated support leaves insufficient room.
    nest3d::arrange_polygons(items, fixed, bed(22, 14), arrange_params(), settings, config, 1);
    CHECK(items.front().is_arranged() == !supports);
}

TEST_CASE("Support-aware stacking reserves the full height of thick bridge beads", "[Nest3DSupport]")
{
    constexpr double part_height = 5;
    constexpr double clearance = 2;
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({{"enable_support", "0"}, {"thick_bridges", "1"}, {"bridge_flow", "1"}});
    Model fixed_model = Test::model("thick bridge cap", supported_cap());
    Print reference;
    reference.set_status_silent();
    reference.apply(fixed_model, config);
    reference.process();
    bool thick_bridge = false;
    double bead_bottom = std::numeric_limits<double>::infinity();
    double nominal_bottom = bead_bottom;
    for (const PrintObject *object : reference.objects())
        for (const Layer *layer : object->layers())
            for (const LayerRegion *region : layer->regions()) {
                const auto inspect = [&](const ExtrusionPath &path) {
                    if (is_bridge(path.role()) && path.height > layer->height + EPSILON) {
                        thick_bridge = true;
                        const double bottom = layer->print_z - path.height;
                        if (bottom < bead_bottom) {
                            bead_bottom = bottom;
                            nominal_bottom = layer->print_z - layer->height;
                        }
                    }
                };
                for_each_extrusion_path(region->perimeters, inspect);
                for_each_extrusion_path(region->fills, inspect);
            }
    REQUIRE(thick_bridge);
    // The nominal layer bottom admits this part, but the real bridge bead hangs lower.
    REQUIRE(nominal_bottom >= part_height + clearance - EPSILON);
    REQUIRE(bead_bottom < part_height + clearance - EPSILON);

    auto fixed = items_of(fixed_model, config);
    Model selected_model = Test::model("part below bridge", make_cube(4, 4, part_height));
    auto items = items_of(selected_model, config);
    auto settings = support_settings();
    settings.allow_stacking = true;
    settings.vertical_gap_mm = float(clearance);
    auto nominal_items = items;
    auto nominal_settings = settings;
    nominal_settings.generate_supports = false;
    nest3d::arrange_polygons(nominal_items, fixed, bed(22, 14), arrange_params(), nominal_settings, config, 1);
    REQUIRE(nominal_items.front().is_arranged());
    nest3d::arrange_polygons(items, fixed, bed(22, 14), arrange_params(), settings, config, 1);
    CHECK_FALSE(items.front().is_arranged());
}

TEST_CASE("Cancelling support preparation leaves placements and source objects untouched", "[Nest3DSupport]")
{
    DynamicPrintConfig config = support_config();
    Model model = Test::model("cancelled support", supported_cap());
    model.objects.front()->instances.front()->set_offset(Vec3d(27, 31, 0));
    model.objects.front()->add_instance()->set_rotation(Vec3d(0, 0, PI / 2));
    auto items = items_of(model, config);
    const auto original = items;
    const auto transformation = model.objects.front()->instances.front()->get_matrix();
    std::atomic<int> polls{0};
    auto params = arrange_params();
    params.stopcondition = [&polls] { return ++polls > 3; };
    REQUIRE_THROWS_AS(nest3d::arrange_polygons(items, {}, bed(75, 45), params, support_settings(), config, 1), CanceledException);
    CHECK(polls.load() > 3);
    REQUIRE(items.size() == original.size());
    for (size_t i = 0; i < items.size(); ++i) {
        CHECK(items[i].translation == original[i].translation);
        CHECK(items[i].bed_idx == original[i].bed_idx);
        CHECK_THAT(items[i].rotation, Catch::Matchers::WithinAbs(original[i].rotation, 1e-12));
    }
    CHECK(model.objects.front()->instances.front()->get_matrix().isApprox(transformation));
    CHECK(model.objects.front()->instances.size() == 2);
    CHECK(model.objects.front()->volumes.size() == 1);
}

TEST_CASE("Support-aware nesting reserves upward Z contouring above nominal layer height", "[Nest3DSupport]")
{
    constexpr double layer_height = 0.2;
    constexpr double surface_height = 2.03;
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"enable_support", "0"}, {"precise_z_height", "0"},
        {"zaa_enabled", "1"}, {"zaa_min_z", "0.05"},
        {"zaa_minimize_perimeter_height", "0"}
    });
    Model model = Test::model("contoured top", make_cube(12, 12, surface_height));
    Print reference;
    reference.set_status_silent();
    reference.apply(model, config);
    reference.process();
    double nominal_top = 0, contoured_top = 0;
    for (const PrintObject *object : reference.objects())
        for (const Layer *layer : object->layers()) {
            nominal_top = std::max(nominal_top, layer->print_z);
            for (const LayerRegion *region : layer->regions()) {
                const auto inspect = [&](const ExtrusionPath &path) {
                    if (path.z_contoured)
                        for (const Point3 &point : path.polyline.points)
                            contoured_top = std::max(contoured_top, layer->print_z + unscale<double>(point.z()));
                };
                for_each_extrusion_path(region->perimeters, inspect);
                for_each_extrusion_path(region->fills, inspect);
            }
        }
    REQUIRE_THAT(nominal_top, Catch::Matchers::WithinAbs(10 * layer_height, EPSILON));
    REQUIRE_THAT(contoured_top, Catch::Matchers::WithinAbs(surface_height, EPSILON));
    REQUIRE(contoured_top > nominal_top);

    auto params = arrange_params();
    params.printable_height = float((nominal_top + contoured_top) / 2);
    auto items = items_of(model, config);
    auto nominal_items = items;
    DynamicPrintConfig nominal_config = config;
    nominal_config.set_key_value("zaa_enabled", new ConfigOptionBool(false));
    nest3d::arrange_polygons(nominal_items, {}, bed(14, 14), params, support_settings(), nominal_config, 1);
    REQUIRE(nominal_items.front().is_arranged());
    nest3d::arrange_polygons(items, {}, bed(14, 14), params, support_settings(), config, 1);
    CHECK_FALSE(items.front().is_arranged());
}

TEST_CASE("Support-aware stacking reserves downward Z contouring below nominal layer bottom", "[Nest3DSupport]")
{
    constexpr double cap_bottom = 7;
    constexpr double cap_height = 0.17;
    constexpr double part_height = 5;
    constexpr double clearance = 2;
    DynamicPrintConfig config = support_config();
    config.set_deserialize_strict({
        {"enable_support", "0"}, {"precise_z_height", "0"}, {"thick_bridges", "0"}, {"bridge_flow", "1"},
        {"zaa_enabled", "1"}, {"zaa_min_z", "0.05"}, {"zaa_minimize_perimeter_height", "0"}
    });
    TriangleMesh stem = make_cube(6, 6, cap_bottom + cap_height);
    stem.translate(7, 3, 0);
    TriangleMesh cap = make_cube(20, 12, cap_height);
    cap.translate(0, 0, cap_bottom);
    stem.merge(cap);
    Model fixed_model = Test::model("lowered contour cap", std::move(stem));
    Print reference;
    reference.set_status_silent();
    reference.apply(fixed_model, config);
    reference.process();
    double nominal_bottom = std::numeric_limits<double>::infinity();
    double contoured_bottom = nominal_bottom;
    for (const PrintObject *object : reference.objects())
        for (const Layer *layer : object->layers())
            for (const LayerRegion *region : layer->regions()) {
                const auto inspect = [&](const ExtrusionPath &path) {
                    if (path.z_contoured)
                        for (const Point3 &point : path.polyline.points) {
                            const double offset = unscale<double>(point.z());
                            if (offset < -EPSILON) {
                                nominal_bottom = std::min(nominal_bottom, layer->print_z - path.height);
                                contoured_bottom = std::min(contoured_bottom, layer->print_z + offset - path.height);
                            }
                        }
                };
                for_each_extrusion_path(region->perimeters, inspect);
                for_each_extrusion_path(region->fills, inspect);
            }
    REQUIRE_THAT(nominal_bottom, Catch::Matchers::WithinAbs(cap_bottom, EPSILON));
    REQUIRE(contoured_bottom < part_height + clearance - EPSILON);

    const auto fixed = items_of(fixed_model, config);
    Model selected_model = Test::model("part below contour", make_cube(4, 4, part_height));
    auto items = items_of(selected_model, config);
    auto settings = support_settings();
    settings.allow_stacking = true;
    settings.vertical_gap_mm = float(clearance);
    auto nominal_items = items;
    DynamicPrintConfig nominal_config = config;
    nominal_config.set_key_value("zaa_enabled", new ConfigOptionBool(false));
    nest3d::arrange_polygons(nominal_items, fixed, bed(22, 14), arrange_params(), settings, nominal_config, 1);
    REQUIRE(nominal_items.front().is_arranged());
    nest3d::arrange_polygons(items, fixed, bed(22, 14), arrange_params(), settings, config, 1);
    CHECK_FALSE(items.front().is_arranged());
}
