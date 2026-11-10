#ifndef slic3r_Parameter_Utils_hpp_
#define slic3r_Parameter_Utils_hpp_

#include <boost/log/trivial.hpp>

#include <cstddef>
#include <utility>
#include <string>
#include <vector>
#include <map>
#include "PrintConfig.hpp"

namespace Slic3r {

template <typename T, typename OptionType>
void set_config_values(DynamicPrintConfig *config, const std::string &key, T value)
{
    auto config_opt = config->option<OptionType>(key);
    if (config_opt) {
        for (size_t i = 0; i < config_opt->values.size(); ++i) {
            config_opt->values[i] = value;
        }
    } else {
        BOOST_LOG_TRIVIAL(info) << "set_config_values: missing setting " << key;
    }
}

std::vector<InputShaperType> calibration_input_shaper_types(GCodeFlavor flavor);
bool has_junction_deviation(const DynamicPrintConfig *printer_config);

using LayerPrintSequence = std::pair<std::pair<int, int>, std::vector<int>>;
std::vector<LayerPrintSequence> get_other_layers_print_sequence(int sequence_nums, const std::vector<int> &sequence);
void get_other_layers_print_sequence(const std::vector<LayerPrintSequence> &customize_sequences, int &sequence_nums, std::vector<int> &sequence);

extern int get_index_for_extruder_parameter(const DynamicPrintConfig &config, const std::string &opt_key, int cur_extruder_id, ExtruderType extruder_type, NozzleVolumeType nozzle_volume_type);
} // namespace Slic3r

#endif // slic3r_Parameter_Utils_hpp_
