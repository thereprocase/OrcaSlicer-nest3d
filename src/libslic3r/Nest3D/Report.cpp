#include "Report.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

#include <boost/format.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstddef>

namespace Slic3r { namespace nest3d {

std::string length_text(double mm)
{
    char text[32];
    std::snprintf(text, sizeof text, "%.1f", mm);
    std::string out = text;
    if (out.size() > 2 && out.compare(out.size() - 2, 2, ".0") == 0)
        out.resize(out.size() - 2);
    return out;
}

std::string issue_text(const Issue &issue, const Translate &translate)
{
    std::string text;
    for (const Sentence &sentence : issue.sentences) {
        std::string pattern = translate ? translate(sentence.msgid) : std::string(sentence.msgid);
        try {
            boost::format format(pattern);
            for (const std::string &arg : sentence.args)
                format % arg;
            pattern = format.str();
        } catch (const boost::io::format_error &) {
            // A catalog entry with the wrong placeholders must not hide the message: show it as it is.
        }
        if (!text.empty())
            text += ' ';
        text += pattern;
    }
    return text;
}

std::pair<double, double> largest_free_rectangle(const ExPolygons &area, const BoundingBox &bed, double cell_mm)
{
    const int w = std::max(1, int(std::ceil(unscaled(bed.size().x()) / cell_mm)));
    const int h = std::max(1, int(std::ceil(unscaled(bed.size().y()) / cell_mm)));
    // A cell is free when its centre lies inside the area.
    std::vector<char> free_cell(size_t(w) * size_t(h), 0);
    for (const ExPolygon &polygon : area) {
        const BoundingBox box = get_extents(polygon);
        const int x0 = std::max(0, int(std::floor(unscaled(box.min.x() - bed.min.x()) / cell_mm)));
        const int x1 = std::min(w - 1, int(std::ceil(unscaled(box.max.x() - bed.min.x()) / cell_mm)));
        const int y0 = std::max(0, int(std::floor(unscaled(box.min.y() - bed.min.y()) / cell_mm)));
        const int y1 = std::min(h - 1, int(std::ceil(unscaled(box.max.y() - bed.min.y()) / cell_mm)));
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                if (polygon.contains(bed.min + Point(scaled((x + 0.5) * cell_mm), scaled((y + 0.5) * cell_mm))))
                    free_cell[size_t(y) * size_t(w) + size_t(x)] = 1;
    }
    // Largest rectangle under a histogram, row by row.
    std::vector<int> heights(size_t(w), 0);
    std::pair<int, int> best{0, 0};
    long best_area = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x)
            heights[size_t(x)] = free_cell[size_t(y) * size_t(w) + size_t(x)] ? heights[size_t(x)] + 1 : 0;
        for (int x = 0; x < w; ++x) {
            int low = heights[size_t(x)];
            for (int right = x; right < w && heights[size_t(right)] > 0; ++right) {
                low = std::min(low, heights[size_t(right)]);
                const long rect = long(low) * long(right - x + 1);
                if (rect > best_area) {
                    best_area = rect;
                    best      = {right - x + 1, low};
                }
            }
        }
    }
    return {best.first * cell_mm, best.second * cell_mm};
}

}} // namespace Slic3r::nest3d
