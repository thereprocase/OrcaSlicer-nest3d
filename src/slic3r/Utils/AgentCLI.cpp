#include "AgentCLI.hpp"

#include "Http.hpp"

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/libslic3r.h"

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/fstream.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <regex>
#include <set>
#include <sstream>

namespace Slic3r { namespace AgentCLI {

using nlohmann::json;
namespace fs = boost::filesystem;

namespace {

// Footprints are measured to this tolerance. It keeps an outline with smooth curves to a few hundred
// points, so the brute-force distances below stay cheap.
constexpr double OutlineToleranceMM = 0.01;

double round_to(double value, double step) { return std::round(value / step) * step; }
double mm(double scaled_length) { return round_to(unscale<double>(scaled_length), 0.001); }
double mm2(double scaled_area) { return round_to(scaled_area * SCALING_FACTOR * SCALING_FACTOR, 0.001); }

json point_json(const Point &p) { return json::array({mm(p.x()), mm(p.y())}); }

json bbox_json(const BoundingBox &bb)
{
    return json{{"min", point_json(bb.min)}, {"max", point_json(bb.max)}, {"size", point_json(bb.size())}};
}

// The outline of an instance's model parts seen from above, in world coordinates.
ExPolygons instance_footprint(const ModelObject &object, const ModelInstance &instance)
{
    Polygons projections;
    for (const ModelVolume *volume : object.volumes)
        if (volume->is_model_part()) {
            const Transform3d trafo = instance.get_matrix() * volume->get_matrix();
            append(projections, project_mesh(volume->mesh().its, trafo, []() {}));
        }
    return expolygons_simplify(union_ex(projections), scaled(OutlineToleranceMM));
}

ExPolygons translated(ExPolygons polygons, const Point &shift)
{
    for (ExPolygon &polygon : polygons)
        polygon.translate(shift);
    return polygons;
}

double bbox_distance(const BoundingBox &a, const BoundingBox &b)
{
    const double dx = std::max({0., double(b.min.x() - a.max.x()), double(a.min.x() - b.max.x())});
    const double dy = std::max({0., double(b.min.y() - a.max.y()), double(a.min.y() - b.max.y())});
    return std::hypot(dx, dy);
}

// Smallest distance between the outlines of `a` and `b`, scaled. With `overlap_is_zero`, shapes that overlap
// are 0 apart; without it only the outlines count, as for a part inside the bed outline.
double outline_distance(const ExPolygons &a, const ExPolygons &b, bool overlap_is_zero)
{
    if (a.empty() || b.empty())
        return std::numeric_limits<double>::infinity();
    if (overlap_is_zero && !intersection_ex(a, b).empty())
        return 0.;
    const Lines lines_a = to_lines(a), lines_b = to_lines(b);
    double      best    = std::numeric_limits<double>::infinity();
    // The closest pair of two polygon outlines always includes a vertex of one of them.
    const auto scan = [&best](const Lines &from, const Lines &to) {
        for (const Line &vertex : from)
            for (const Line &edge : to)
                best = std::min(best, edge.distance_to_squared(vertex.a));
    };
    scan(lines_a, lines_b);
    scan(lines_b, lines_a);
    return std::sqrt(best);
}

json distance_json(double scaled_distance)
{
    return std::isfinite(scaled_distance) ? json(mm(scaled_distance)) : json(nullptr);
}

Polygon scaled_polygon(const Pointfs &points)
{
    Polygon polygon;
    for (const Vec2d &pt : points)
        polygon.points.emplace_back(scaled(pt.x()), scaled(pt.y()));
    return polygon;
}

std::string trimmed(const std::string &text)
{
    const size_t begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
        return std::string();
    const size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

} // namespace

PlateBed plate_bed(const DynamicPrintConfig &config)
{
    PlateBed bed;
    if (const auto *area = config.option<ConfigOptionPoints>("printable_area"))
        bed.printable = scaled_polygon(area->values);
    // The excluded area is one polygon of plate coordinates, as the GUI draws it.
    if (const auto *excluded = config.option<ConfigOptionPoints>("bed_exclude_area"); excluded && excluded->values.size() >= 3) {
        Polygon polygon = scaled_polygon(excluded->values);
        if (std::abs(polygon.area()) > 0.)
            bed.excluded.push_back(std::move(polygon));
    }
    return bed;
}

json arrange_report(const Model &model, const std::vector<Placement> &placements, const PlateBed &bed)
{
    struct Item
    {
        const Placement *placement;
        ExPolygons       footprint; // plate coordinates
        BoundingBox      bbox;
    };
    std::vector<Item> items;
    items.reserve(placements.size());
    for (const Placement &placement : placements) {
        const ModelObject   &object   = *model.objects[placement.object_idx];
        const ModelInstance &instance = *object.instances[placement.instance_idx];
        Item item{&placement, translated(instance_footprint(object, instance), Point::new_scale(-placement.plate_origin.x(), -placement.plate_origin.y())), {}};
        item.bbox = get_extents(item.footprint);
        items.push_back(std::move(item));
    }

    const ExPolygons excluded = to_expolygons(bed.excluded);
    const ExPolygons printable{ExPolygon(bed.printable)};
    json             objects  = json::array();
    json             unplaced = json::array();
    int              plates   = 0;
    for (const Item &item : items) {
        const Placement     &placement = *item.placement;
        const ModelObject   &object    = *model.objects[placement.object_idx];
        const ModelInstance &instance  = *object.instances[placement.instance_idx];
        json                 entry{{"name", object.name},
                                   {"source_file", object.input_file},
                                   {"object_index", placement.object_idx},
                                   {"instance_index", placement.instance_idx}};
        if (placement.plate < 0) {
            unplaced.push_back(entry);
            continue;
        }
        plates = std::max(plates, placement.plate + 1);

        double      nearest      = std::numeric_limits<double>::infinity();
        std::string nearest_name;
        for (const Item &other : items) {
            if (&other == &item || other.placement->plate != placement.plate || bbox_distance(item.bbox, other.bbox) >= nearest)
                continue;
            const double d = outline_distance(item.footprint, other.footprint, true);
            if (d < nearest) {
                nearest      = d;
                nearest_name = model.objects[other.placement->object_idx]->name;
            }
        }
        const Vec3d offset = instance.get_offset();
        entry["plate"]                = placement.plate + 1;
        entry["x"]                    = round_to(offset.x() - placement.plate_origin.x(), 0.001);
        entry["y"]                    = round_to(offset.y() - placement.plate_origin.y(), 0.001);
        entry["rotation_deg"]         = round_to(Geometry::rad2deg(instance.get_rotation().z()), 0.001);
        entry["footprint_bbox"]       = bbox_json(item.bbox);
        entry["footprint_area_mm2"]   = mm2(area(item.footprint));
        entry["nearest_neighbour_mm"] = distance_json(nearest);
        entry["nearest_neighbour"]    = nearest_name.empty() ? json(nullptr) : json(nearest_name);
        entry["nearest_exclusion_mm"] = distance_json(outline_distance(item.footprint, excluded, true));
        entry["bed_edge_mm"]          = distance_json(outline_distance(item.footprint, printable, false));
        objects.push_back(std::move(entry));
    }

    json bed_json{{"printable_area", json::array()}, {"exclude_area", json::array()}};
    for (const Point &pt : bed.printable.points)
        bed_json["printable_area"].push_back(point_json(pt));
    for (const Polygon &polygon : bed.excluded) {
        json points = json::array();
        for (const Point &pt : polygon.points)
            points.push_back(point_json(pt));
        bed_json["exclude_area"].push_back(std::move(points));
    }
    return json{{"plates_used", plates}, {"objects", std::move(objects)}, {"unplaced", std::move(unplaced)}, {"bed", std::move(bed_json)}};
}

json first_layer_check(const Print &print, const PlateBed &bed, double min_island_mm2, double min_exclusion_mm)
{
    const Vec3d      origin   = print.get_plate_origin();
    const Point      to_plate = Point::new_scale(-origin.x(), -origin.y());
    const ExPolygons excluded = to_expolygons(bed.excluded);
    json             objects  = json::array();
    for (const PrintObject *object : print.objects()) {
        if (object->layers().empty())
            continue;
        const ExPolygons &islands = object->layers().front()->lslices;
        for (const PrintInstance &instance : object->instances()) {
            const ExPolygons placed  = translated(islands, Point(instance.shift + to_plate));
            double           contact = 0., smallest = std::numeric_limits<double>::infinity();
            json             small   = json::array();
            for (const ExPolygon &island : placed) {
                const double a = island.area();
                contact += a;
                smallest = std::min(smallest, a);
                if (a * SCALING_FACTOR * SCALING_FACTOR < min_island_mm2)
                    small.push_back(json{{"area_mm2", mm2(a)}, {"center", point_json(get_extents(island).center())}});
            }
            const double exclusion = outline_distance(placed, excluded, true);
            json         flags     = json::array();
            if (!small.empty())
                flags.push_back("small_island");
            if (std::isfinite(exclusion) && unscale<double>(exclusion) < min_exclusion_mm)
                flags.push_back("near_exclusion_area");
            objects.push_back(json{{"name", object->model_object()->name},
                                   {"island_count", placed.size()},
                                   {"contact_area_mm2", mm2(contact)},
                                   {"smallest_island_mm2", std::isfinite(smallest) ? json(mm2(smallest)) : json(nullptr)},
                                   {"small_islands", std::move(small)},
                                   {"nearest_exclusion_mm", distance_json(exclusion)},
                                   {"bbox", bbox_json(get_extents(placed))},
                                   {"flags", std::move(flags)}});
        }
    }
    return json{{"min_island_mm2", min_island_mm2}, {"min_exclusion_mm", min_exclusion_mm}, {"objects", std::move(objects)}};
}

json plate_summary(int plate_id, const Print &print, const GCodeProcessorResult &result)
{
    const PrintEstimatedStatistics &stats = result.print_statistics;
    const PrintConfig              &config = print.config();

    json filaments = json::array();
    for (const auto &[filament, volume] : stats.total_volumes_per_extruder) {
        const double diameter = filament < result.filament_diameters.size() ? result.filament_diameters[filament] : 1.75;
        const double density  = filament < result.filament_densities.size() ? result.filament_densities[filament] : 1.;
        const double section  = PI * sqr(0.5 * diameter);
        json         entry{{"id", filament + 1},
                           {"length_m", round_to(section > EPSILON ? 0.001 * volume / section : 0., 0.001)},
                           {"weight_g", round_to(0.001 * volume * density, 0.001)}};
        if (filament < config.filament_type.values.size())
            entry["type"] = config.filament_type.values[filament];
        if (filament < config.filament_colour.values.size())
            entry["color"] = config.filament_colour.values[filament];
        filaments.push_back(std::move(entry));
    }

    // What the G-code processor read from the exported G-code: the toolpaths the plate prints.
    std::set<ExtrusionRole> roles;
    for (const auto &[role, used] : stats.used_filaments_per_role)
        if (used.first > 0.)
            roles.insert(role);
    json role_names = json::array();
    for (ExtrusionRole role : roles)
        role_names.push_back(ExtrusionEntity::role_to_string(role));
    const bool has_support = std::any_of(roles.begin(), roles.end(), [](ExtrusionRole role) { return is_support(role); });

    json objects = json::array();
    for (const PrintObject *object : print.objects())
        for (size_t i = 0; i < object->instances().size(); ++i)
            objects.push_back(object->model_object()->name);

    const auto &normal = stats.modes[static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal)];
    return json{{"plate", plate_id},
                {"estimated_time_s", round_to(normal.time, 1.)},
                {"filaments", std::move(filaments)},
                {"objects", std::move(objects)},
                {"has_brim", roles.count(erBrim) > 0},
                {"has_skirt", roles.count(erSkirt) > 0},
                {"has_support", has_support},
                {"feature_types", std::move(role_names)}};
}

bool apply_object_settings(Model &model, const std::string &path, std::string &error)
{
    json overrides;
    try {
        boost::nowide::ifstream in(path);
        if (!in) {
            error = "cannot open " + path;
            return false;
        }
        in >> overrides;
    } catch (const std::exception &ex) {
        error = path + ": " + ex.what();
        return false;
    }
    if (!overrides.is_object()) {
        error = path + ": expected an object keyed by object or file name";
        return false;
    }

    for (const auto &[key, settings] : overrides.items()) {
        if (!settings.is_object()) {
            error = path + ": settings of \"" + key + "\" are not an object";
            return false;
        }
        std::vector<ModelObject *> targets;
        for (ModelObject *object : model.objects)
            if (object->name == key || (!object->input_file.empty() && fs::path(object->input_file).filename().string() == key))
                targets.push_back(object);
        if (targets.empty()) {
            error = path + ": no object or input file named \"" + key + "\"";
            return false;
        }
        for (const auto &[opt_key, value] : settings.items()) {
            // Only settings the slicer reads per object can be overridden on one.
            if (!PrintObjectConfig::defaults().has(opt_key) && !PrintRegionConfig::defaults().has(opt_key)) {
                error = path + ": \"" + opt_key + "\" is not a per-object setting";
                return false;
            }
            const std::string text = value.is_string() ? value.get<std::string>() :
                                     value.is_boolean() ? std::string(value.get<bool>() ? "1" : "0") :
                                                          value.dump();
            for (ModelObject *object : targets) {
                try {
                    ConfigSubstitutionContext context(ForwardCompatibilitySubstitutionRule::Disable);
                    object->config.set_deserialize(opt_key, text, context);
                } catch (const std::exception &ex) {
                    error = path + ": " + key + ": " + opt_key + "=" + text + ": " + ex.what();
                    return false;
                }
            }
            BOOST_LOG_TRIVIAL(info) << "object settings: " << key << " (" << targets.size() << " objects): " << opt_key << " = " << text;
        }
    }
    return true;
}

std::string unindent_tool_selects(const std::string &gcode)
{
    // A tool select alone on its line: T0, T[initial_extruder] or T{...}, with an optional comment.
    static const std::regex tool_select(R"(T(\d+|\[[^\]]*\]|\{[^}]*\})[ \t]*(;.*)?\r?)");
    std::string out;
    out.reserve(gcode.size());
    size_t start = 0;
    while (start <= gcode.size()) {
        const size_t      end    = std::min(gcode.find('\n', start), gcode.size());
        const std::string line   = gcode.substr(start, end - start);
        const size_t      indent = line.find_first_not_of(" \t");
        if (indent != std::string::npos && indent > 0 && std::regex_match(line.begin() + indent, line.end(), tool_select))
            out += line.substr(indent);
        else
            out += line;
        if (end == gcode.size())
            break;
        out += '\n';
        start = end + 1;
    }
    return out;
}

std::string printer_model_id(const std::string &printer_model)
{
    if (printer_model.empty())
        return std::string();
    // The vendor trees the GUI reads model ids from: installed ones first, then the bundled profiles.
    for (const fs::path &root : {fs::path(data_dir()) / PRESET_SYSTEM_DIR, fs::path(resources_dir()) / "profiles"}) {
        boost::system::error_code ec;
        if (!fs::is_directory(root, ec))
            continue;
        for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
            const fs::path file = it->path() / "machine" / (printer_model + ".json");
            if (!fs::is_regular_file(file, ec))
                continue;
            try {
                boost::nowide::ifstream in(file.string());
                json                    j;
                in >> j;
                if (j.value("type", "") == "machine_model" && j.contains("model_id"))
                    return j["model_id"].get<std::string>();
            } catch (const std::exception &ex) {
                BOOST_LOG_TRIVIAL(warning) << "printer model id: cannot read " << file.string() << ": " << ex.what();
            }
        }
    }
    return std::string();
}

bool parse_tray_list(const std::string &text, std::vector<int> &trays)
{
    trays.clear();
    std::stringstream stream(text);
    std::string       item;
    while (std::getline(stream, item, ',')) {
        item = trimmed(item);
        if (item.empty() || item.find_first_not_of("0123456789") != std::string::npos || item.size() > 3)
            return false;
        trays.push_back(std::stoi(item));
    }
    return !trays.empty();
}

bool send_to_bridge(const std::string &url, const std::string &token_file, const std::string &ams_mapping,
                    const std::string &ca_file, bool insecure,
                    const std::vector<std::string> &files, json &results, std::string &error)
{
    std::string token;
    {
        boost::nowide::ifstream in(token_file, std::ios::binary);
        if (!in) {
            error = "cannot read the bridge token file " + token_file;
            return false;
        }
        std::stringstream buffer;
        buffer << in.rdbuf();
        token = trimmed(buffer.str());
    }
    if (token.empty()) {
        error = "the bridge token file " + token_file + " is empty";
        return false;
    }
    // Whatever a server echoes back is kept for the report, but never the token itself.
    const auto scrub = [&token](std::string text) {
        for (size_t pos = text.find(token); pos != std::string::npos; pos = text.find(token, pos))
            text.replace(pos, token.size(), "<token>");
        if (text.size() > 2000)
            text = text.substr(0, 2000) + "...";
        return text;
    };

    bool all_ok = true;
    results     = json::array();
    for (const std::string &file : files) {
        json result{{"file", file}, {"ok", false}};
        // The shared Http client defaults to no certificate checks; a bearer token must not go out that way.
        Http http = Http::post(url);
        http.tls_verify(!insecure);
        if (!insecure && !ca_file.empty())
            http.ca_file(ca_file);
        http.header("Authorization", "Bearer " + token)
            .form_add("ams_mapping", ams_mapping)
            .form_add_file("file", fs::path(file), fs::path(file).filename().string())
            .on_complete([&](std::string body, unsigned status) {
                result["ok"]       = status >= 200 && status < 300;
                result["status"]   = status;
                result["response"] = scrub(body);
            })
            .on_error([&](std::string body, std::string err, unsigned status) {
                result["status"]   = status;
                result["error"]    = scrub(err);
                result["response"] = scrub(body);
            })
            .perform_sync();
        all_ok &= result["ok"].get<bool>();
        BOOST_LOG_TRIVIAL(info) << "bridge upload of " << file << ": status " << result.value("status", 0u) << (result["ok"].get<bool>() ? " ok" : " failed");
        results.push_back(std::move(result));
    }
    if (!all_ok)
        error = "the bridge rejected at least one upload";
    return all_ok;
}

}} // namespace Slic3r::AgentCLI
