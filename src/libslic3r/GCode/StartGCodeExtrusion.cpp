#include "StartGCodeExtrusion.hpp"

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/PlaceholderParser.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

#include <boost/log/trivial.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r {

namespace {

// The letter words of one G-code line, comments removed. `code` is the command number (29.2 for G29.2).
struct Words
{
    char                                 command = 0;
    double                               code    = -1.;
    std::array<std::optional<double>, 26> value;

    const std::optional<double> &operator[](char letter) const { return value[size_t(letter - 'A')]; }
    bool is(char c, int n) const { return command == c && code == double(n); }
};

Words parse_words(const std::string &line)
{
    Words       words;
    const char  *p   = line.c_str();
    const size_t cut = line.find(';');
    const char  *end = p + (cut == std::string::npos ? line.size() : cut);
    while (p < end) {
        const char letter = char(std::toupper(static_cast<unsigned char>(*p)));
        if (letter < 'A' || letter > 'Z') {
            ++p;
            continue;
        }
        char        *after = nullptr;
        const double v     = std::strtod(p + 1, &after);
        if (after == p + 1 || after > end) {
            ++p; // a letter without a number: a macro argument or text, not a word
            continue;
        }
        if (words.command == 0) {
            words.command = letter;
            words.code    = v;
        } else
            words.value[size_t(letter - 'A')] = v;
        p = after;
    }
    return words;
}

// The firmware state a start G-code moves through, in G-code coordinates. An axis is unknown from the
// start and after homing, probing or anything else that moves it to where the slicer cannot tell.
struct Machine
{
    std::array<std::optional<double>, 3> pos;                  // X, Y, Z as the G-code names them
    std::array<std::optional<double>, 3> g92_shift{0., 0., 0.}; // homed position minus that name, set by G92
    bool                                  relative_xyz = false;
    bool                                  relative_e   = false;
    bool                                  e_mode_set   = false; // M82/M83 seen: G90/G91 leave E alone
    double                                e            = 0.;
    double                                retracted    = 0.; // pulled back, so the next push refills the nozzle
};

// Appends the plan view of one extruding move: a bead around `path`, or a blob when the nozzle stands still.
void add_bead(Polygons &out, const Points &path, double volume, std::optional<double> z, double min_width,
              const Polygon &plate)
{
    const double length = Polyline(path).length() * SCALING_FACTOR;
    // A start G-code purge is printed on the plate, so its height is the nozzle height; without a known
    // height the bead is taken to be as high as the nozzle is wide.
    const double h = z && *z > EPSILON ? *z : min_width;
    if (length < EPSILON) {
        // A flush beside the plate drops into the purge chute; over the plate it spreads into a puddle
        // about as high as the nozzle.
        if (!plate.contains(path.front()))
            return;
        const double  radius   = std::max(0.5 * min_width, std::sqrt(volume / (PI * h)));
        constexpr int segments = 24;
        Polygon       blob     = make_circle_num_segments(scaled(radius / std::cos(PI / segments)), segments);
        blob.translate(path.front());
        out.push_back(std::move(blob));
        return;
    }
    // The flow model the slicer itself uses: a rectangle with rounded sides, area w * h - h^2 (1 - pi / 4).
    // Thinner than the nozzle would be under-extrusion the bead still spreads over; the cap keeps a slow
    // move high above the plate from reading as a wide line.
    const double width = std::clamp(volume / (length * h) + h * (1. - PI / 4.), min_width, 3. * min_width);
    append(out, offset(Polyline(path), float(scaled(0.5 * width)), jtSquare, 0., etOpenSquare));
}

// Points of an arc from `from` about `center` to `to` (a full turn when they coincide), at most 0.5 mm apart.
Points arc_points(const Vec2d &from, const Vec2d &to, const Vec2d &center, bool ccw)
{
    const double radius = (from - center).norm();
    const double a0     = std::atan2(from.y() - center.y(), from.x() - center.x());
    double       sweep  = std::atan2(to.y() - center.y(), to.x() - center.x()) - a0;
    if (ccw && sweep <= EPSILON)
        sweep += 2. * PI;
    if (!ccw && sweep >= -EPSILON)
        sweep -= 2. * PI;
    const int n = std::clamp(int(std::ceil(std::abs(sweep) * radius / 0.5)), 4, 720);
    Points    out;
    for (int i = 0; i <= n; ++i) {
        const double a = a0 + sweep * i / n;
        out.push_back(scaled(Vec2d(center + radius * Vec2d(std::cos(a), std::sin(a)))));
    }
    out.back() = scaled(to);
    return out;
}

} // namespace

ExPolygons gcode_extrusion_area(const std::string &gcode, const Polygon &plate, const Vec2d &offset, double min_width,
                                double filament_area)
{
    Polygons           beads;
    Machine            m;
    std::istringstream lines(gcode);
    std::string        line;
    while (std::getline(lines, line)) {
        const Words w = parse_words(line);
        if (w.command == 'M' && (w.code == 82. || w.code == 83.)) {
            m.relative_e = w.code == 83.;
            m.e_mode_set = true;
            continue;
        }
        if (w.command != 'G')
            continue;
        if (w.is('G', 90) || w.is('G', 91)) {
            m.relative_xyz = w.code == 91.;
            if (!m.e_mode_set)
                m.relative_e = m.relative_xyz;
            continue;
        }
        if (w.is('G', 92)) {
            for (int axis = 0; axis < 3; ++axis)
                if (const auto &v = w["XYZ"[axis]]) {
                    // The nozzle stays where it is; only the number that names that place changes.
                    if (m.pos[axis] && m.g92_shift[axis])
                        *m.g92_shift[axis] += *m.pos[axis] - *v;
                    else
                        m.g92_shift[axis].reset();
                    m.pos[axis] = *v;
                }
            if (const auto &e = w['E'])
                m.e = *e;
            continue;
        }
        const bool line_move = w.is('G', 0) || w.is('G', 1);
        const bool arc       = w.is('G', 2) || w.is('G', 3);
        if (!line_move && !arc) {
            // Homing, probing, bed levelling and other motion end at a place this reading cannot know.
            const bool all = w.is('G', 28) && !w['X'] && !w['Y'] && !w['Z'];
            for (int axis = 0; axis < 3; ++axis)
                if (all || w["XYZ"[axis]]) {
                    m.pos[axis].reset();
                    if (w.is('G', 28))
                        m.g92_shift[axis] = 0.;
                }
            continue;
        }

        const std::array<std::optional<double>, 3> from = m.pos;
        for (int axis = 0; axis < 3; ++axis)
            if (const auto &v = w["XYZ"[axis]])
                m.pos[axis] = m.relative_xyz ? (m.pos[axis] ? std::optional<double>(*m.pos[axis] + *v) : std::nullopt) :
                                               std::optional<double>(*v);
        double extruded = 0.;
        if (const auto &e = w['E']) {
            const double pushed = m.relative_e ? *e : *e - m.e;
            m.e                 = m.relative_e ? m.e + *e : *e;
            // A retraction leaves nothing behind and its unretraction only refills the nozzle.
            extruded    = std::max(0., pushed - m.retracted);
            m.retracted = std::max(0., m.retracted - pushed);
        }
        if (extruded <= 0. || !from[0] || !from[1] || !m.pos[0] || !m.pos[1] || !m.g92_shift[0] || !m.g92_shift[1])
            continue;

        // G-code coordinates are plate coordinates less the extruder offset (GCode::point_to_gcode).
        const auto to_plate = [&](double x, double y) -> Vec2d {
            return Vec2d(x + *m.g92_shift[0], y + *m.g92_shift[1]) + offset;
        };
        const Vec2d start = to_plate(*from[0], *from[1]), end = to_plate(*m.pos[0], *m.pos[1]);
        Points      path;
        if (arc && (w['I'] || w['J']))
            path = arc_points(start, end, start + Vec2d(w['I'].value_or(0.), w['J'].value_or(0.)), w.code == 3.);
        else
            path = {scaled(start), scaled(end)}; // a straight move, or an arc given by radius as its chord
        add_bead(beads, path, extruded * filament_area, m.pos[2] ? m.pos[2] : from[2], min_width, plate);
    }
    return union_ex(beads);
}

ExPolygons start_gcode_extrusion_area(const DynamicPrintConfig &input, unsigned int initial_filament, std::string *error)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.apply(input, true);
    const auto &start = config.opt_string("machine_start_gcode");
    if (start.empty())
        return {};

    const size_t filaments = std::max<size_t>(1, config.option<ConfigOptionStrings>("filament_type")->values.size());
    const int    filament  = int(std::min<size_t>(initial_filament, filaments - 1));
    const auto  *map       = config.option<ConfigOptionInts>("filament_map");
    const int    extruders = int(config.option<ConfigOptionFloats>("nozzle_diameter")->values.size());
    const int    extruder  = map && size_t(filament) < map->values.size() ? std::clamp(map->values[size_t(filament)] - 1, 0, extruders - 1) : 0;

    // The variables GCode::_do_export() sets before the start G-code that a layout does not decide, and
    // for those it does decide, the whole printable area as the print area.
    PlaceholderParser parser(&config);
    for (const char *key : {"initial_extruder", "initial_tool", "initial_filament_id", "initial_no_support_extruder",
                            "initial_no_support_tool", "initial_no_support_filament_id", "current_extruder",
                            "current_filament_id"})
        parser.set(key, filament);
    for (const char *key : {"current_extruder_id", "current_nozzle_id", "initial_nozzle_id", "current_hotend",
                            "initial_no_support_hotend"})
        parser.set(key, extruder);
    for (const char *key : {"first_tools", "first_filaments", "first_non_support_tools", "first_non_support_filaments"})
        parser.set(key, new ConfigOptionInts(size_t(extruders), filament));
    parser.set("first_non_support_hotend", new ConfigOptionInts(size_t(extruders), extruder));
    std::vector<unsigned char> used(std::max<size_t>(MAXIMUM_EXTRUDER_NUMBER, filaments), 0);
    used[size_t(filament)] = 1;
    parser.set("is_extruder_used", new ConfigOptionBools(used));
    parser.set("num_extruders", extruders);
    parser.set("has_wipe_tower", false);
    parser.set("wipe_tower_center_pos_x", 0.);
    parser.set("wipe_tower_center_pos_y", 0.);
    parser.set("wipe_tower_center_pos_valid", false);
    parser.set("has_single_extruder_multi_material_priming", false);
    parser.set("total_toolchanges", 0);
    parser.set("total_layer_count", 0);
    parser.set("current_object_idx", 0);
    parser.set("max_additional_fan", 0);
    parser.set("temperature", new ConfigOptionInts(config.option<ConfigOptionInts>("nozzle_temperature")->values));
    parser.set("retract_length", new ConfigOptionFloats(config.option<ConfigOptionFloats>("retraction_length")->values));
    parser.set("first_layer_temperature", new ConfigOptionInts(config.option<ConfigOptionInts>("nozzle_temperature_initial_layer")->values));
    parser.set("first_layer_height", config.opt_float("initial_layer_print_height"));
    parser.set("max_print_height", int(config.opt_float("printable_height")));
    parser.set("max_print_z", int(std::ceil(config.opt_float("printable_height"))));
    parser.set("in_head_wrap_detect_zone", false);
    parser.set("is_all_bbl_filament", false);
    parser.set("has_tpu_in_first_layer", false);
    parser.set("hold_chamber_temp_for_flat_print", false);
    parser.set("enable_high_low_temp_mix", true);
    parser.set("model_name", "");
    parser.set("plate_name", "");
    parser.set("plate_number", "1");
    parser.set("outer_wall_volumetric_speed", 0.);
    parser.set("bbl_bed_temperature_gcode", false);
    parser.set("imex_mode", "");
    parser.set("imex_mode_index", 0);
    parser.set("imex_mode_gcode", "");
    parser.set("bed_temperature_initial_layer_vector", "");

    const BedType bed_type     = config.opt_enum<BedType>("curr_bed_type");
    const auto   *first_bed    = config.option<ConfigOptionInts>(get_bed_temp_1st_layer_key(bed_type));
    const auto   *bed          = config.option<ConfigOptionInts>(get_bed_temp_key(bed_type));
    const auto   *chamber      = config.option<ConfigOptionInts>("chamber_temperature");
    const auto   *vitrification = config.option<ConfigOptionInts>("temperature_vitrification");
    parser.set("bed_temperature_initial_layer", first_bed->clone());
    parser.set("first_layer_bed_temperature", first_bed->clone());
    parser.set("bed_temperature", bed->clone());
    parser.set("bed_temperature_initial_layer_single", first_bed->get_at(size_t(filament)));
    parser.set("overall_chamber_temperature", chamber->get_at(size_t(filament)));
    parser.set("min_vitrification_temperature", vitrification->get_at(size_t(filament)));

    std::vector<double> flush_speeds(filaments);
    std::vector<int>    flush_temps(filaments);
    const bool          fast = config.opt_enum<PrimeVolumeMode>("prime_volume_mode") == PrimeVolumeMode::pvmFast;
    for (size_t i = 0; i < filaments; ++i) {
        flush_speeds[i] = config.option<ConfigOptionFloats>("filament_flush_volumetric_speed")->get_at(i);
        if (flush_speeds[i] == 0.)
            flush_speeds[i] = config.option<ConfigOptionFloats>("filament_max_volumetric_speed")->get_at(i);
        flush_temps[i] = config.option<ConfigOptionInts>(fast ? "filament_flush_temp_fast" : "filament_flush_temp")->get_at(i);
        if (flush_temps[i] == 0)
            flush_temps[i] = config.option<ConfigOptionInts>("nozzle_temperature_range_high")->get_at(i);
    }
    parser.set("flush_volumetric_speeds", new ConfigOptionFloats(flush_speeds));
    parser.set("flush_temperatures", new ConfigOptionInts(flush_temps));

    const BoundingBoxf plate(config.option<ConfigOptionPoints>("printable_area")->values);
    const auto         pair = [](const Vec2d &v) { return new ConfigOptionFloats({v.x(), v.y()}); };
    for (const char *key : {"print_bed_min", "first_layer_print_min", "adaptive_bed_mesh_min"})
        parser.set(key, pair(plate.min));
    for (const char *key : {"print_bed_max", "first_layer_print_max", "adaptive_bed_mesh_max"})
        parser.set(key, pair(plate.max));
    for (const char *key : {"print_bed_size", "first_layer_print_size"})
        parser.set(key, pair(plate.size()));
    parser.set("first_layer_center_no_wipe_tower", pair(plate.center()));
    parser.set("first_layer_print_convex_hull", new ConfigOptionPoints({plate.min, Vec2d(plate.max.x(), plate.min.y()), plate.max,
                                                                        Vec2d(plate.min.x(), plate.max.y())}));
    parser.set("bed_mesh_probe_count", new ConfigOptionInts({3, 3}));
    parser.set("bed_mesh_algo", "bicubic");
    parser.update_timestamp();

    std::string gcode;
    try {
        gcode = parser.process(start, unsigned(filament));
    } catch (const std::exception &ex) {
        BOOST_LOG_TRIVIAL(warning) << "Start G-code extrusion area: the start G-code cannot be evaluated before slicing: " << ex.what();
        if (error)
            *error = ex.what();
        return {};
    }

    const double diameter      = config.option<ConfigOptionFloats>("filament_diameter")->get_at(size_t(filament));
    const double filament_area = 0.25 * PI * diameter * diameter;
    const double nozzle        = config.option<ConfigOptionFloats>("nozzle_diameter")->get_at(size_t(extruder));
    const Vec2d  offset        = config.option<ConfigOptionPoints>("extruder_offset")->get_at(size_t(extruder));
    Polygon      plate_outline;
    for (const Vec2d &pt : config.option<ConfigOptionPoints>("printable_area")->values)
        plate_outline.points.push_back(scaled(pt));
    return gcode_extrusion_area(gcode, plate_outline, offset, nozzle, filament_area);
}

} // namespace Slic3r
