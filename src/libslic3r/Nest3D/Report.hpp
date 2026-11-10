#pragma once

// What Nest3D tells the user when a part does not get the place it asked for. A message is a few
// sentences, each a complete translatable msgid with placeholders, so a translator can reorder and
// inflect it; the arranger fills in the numbers and names, and the caller translates.

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r { namespace nest3d {

struct Sentence
{
    const char              *msgid = ""; // marked with L(), translated by the caller
    std::vector<std::string> args;       // substituted for %1%, %2%, ...
};

struct Issue
{
    enum class Kind { Unplaced, Spilled, Skirt };
    enum class Limit { Bed, ExcludedArea, PurgeStrip, OtherParts, Plates };

    Kind                  kind  = Kind::Unplaced;
    Limit                 limit = Limit::Bed;
    std::string           part; // the object's name
    std::vector<Sentence> sentences;
};

// Looks a msgid up in the caller's catalog; the identity when empty.
using Translate = std::function<std::string(const char *msgid)>;

// The issue's sentences, translated, filled in and joined with a space.
std::string issue_text(const Issue &issue, const Translate &translate = {});

// Width and height in mm of the largest axis-parallel rectangle inside `area` (given as the polygons
// that bound it) that lies within `bed`, found on a grid of `cell_mm`.
std::pair<double, double> largest_free_rectangle(const ExPolygons &area, const BoundingBox &bed, double cell_mm);

// A length for a message: one decimal, without a trailing ".0" (5 mm, 208.6 mm).
std::string length_text(double mm);

}} // namespace Slic3r::nest3d
