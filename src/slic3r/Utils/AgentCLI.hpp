#pragma once

#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace Slic3r {

class DynamicPrintConfig;
class Model;
class Print;
struct GCodeProcessorResult;

// Helpers behind the command-line options for scripted and agent-driven use: arrange reports, plate
// summaries, the first-layer check, per-object settings and the print bridge upload. The options and
// the JSON they write are described in docs/agent-cli.md.
namespace AgentCLI {

// A plate's bed in plate coordinates, scaled: its printable outline and its excluded areas.
struct PlateBed
{
    Polygon  printable;
    Polygons excluded;
};
PlateBed plate_bed(const DynamicPrintConfig &config);

// Where an instance ended up: its plate (0-based, -1 when it lies on none) and that plate's origin.
struct Placement
{
    size_t object_idx   = 0;
    size_t instance_idx = 0;
    int    plate        = -1;
    Vec2d  plate_origin = Vec2d::Zero();
};

// The --arrange-report JSON for the instances in `placements`, measured on their footprints: the outlines
// of their meshes seen from above.
nlohmann::json arrange_report(const Model &model, const std::vector<Placement> &placements, const PlateBed &bed);

// Per printed instance of a sliced plate: first-layer contact area and islands, islands under
// `min_island_mm2`, and the distance to the excluded areas, flagged below `min_exclusion_mm`.
nlohmann::json first_layer_check(const Print &print, const PlateBed &bed, double min_island_mm2, double min_exclusion_mm);

// The summary of a sliced plate, from the G-code processor's statistics of its exported G-code.
nlohmann::json plate_summary(int plate_id, const Print &print, const GCodeProcessorResult &result);

// Applies the per-object overrides in the --object-settings JSON file. Every key of the file must name
// an object or the file name of an input, and every setting must be a known per-object setting.
bool apply_object_settings(Model &model, const std::string &path, std::string &error);

// The machine start G-code with tool selects (T0, T[initial_extruder], ...) moved to the start of their
// line. Some printer bridges only recognise a tool select at the start of a line.
std::string unindent_tool_selects(const std::string &gcode);

// The printer model id ("C12" for "Bambu Lab P1S") a vendor profile gives `printer_model`, or "".
std::string printer_model_id(const std::string &printer_model);

// Parses "2" or "2,0" into tray indices. Rejects anything but non-negative integers.
bool parse_tray_list(const std::string &text, std::vector<int> &trays);

// Uploads each of `files` to `url` as multipart/form-data with fields `file` and `ams_mapping`, authorised
// by the bearer token in `token_file`. One result per file goes to `results`. Never prints the token.
bool send_to_bridge(const std::string &url, const std::string &token_file, const std::string &ams_mapping,
                    const std::vector<std::string> &files, nlohmann::json &results, std::string &error);

} // namespace AgentCLI
} // namespace Slic3r
