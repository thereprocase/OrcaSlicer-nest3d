#pragma once

#include <algorithm>
#include <deque>
#include <cstddef>
#include <optional>

namespace Slic3r {

// Positions are net millimetres of filament, not the E axis (which G92 resets).
// Retraction/unretraction cannot refresh the budget by moving the same stock.
class RetractionLimiter
{
public:
    bool allows(double filament_position, int maximum_count, double extrusion_window)
    {
        m_forward_position = std::max(m_forward_position, filament_position);
        if (maximum_count <= 0 || extrusion_window <= 0.) {
            m_retractions.clear();
            return true;
        }
        while (!m_retractions.empty() && m_forward_position - m_retractions.front() >= extrusion_window)
            m_retractions.pop_front();
        return m_retractions.size() < static_cast<std::size_t>(maximum_count);
    }

    bool allows_minimum(double filament_position, double minimum_extrusion)
    {
        m_forward_position = std::max(m_forward_position, filament_position);
        return minimum_extrusion <= 0. || !m_last_retraction_position ||
            m_forward_position - *m_last_retraction_position >= minimum_extrusion;
    }

    void record()
    {
        m_retractions.push_back(m_forward_position);
        m_last_retraction_position = m_forward_position;
    }
    void reset()
    {
        m_retractions.clear();
        m_forward_position = 0.;
        m_last_retraction_position.reset();
    }

private:
    double m_forward_position {0.};
    std::deque<double> m_retractions;
    std::optional<double> m_last_retraction_position;
};

} // namespace Slic3r
