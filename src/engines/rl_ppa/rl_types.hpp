#pragma once

#include "core/db/chip_db.hpp"
#include <cstdint>
#include <vector>

namespace vlsi::rl {

struct PPAState {
    double wns_ns{0.0};              // Worst Negative Slack (WNS)
    double tns_ns{0.0};              // Total Negative Slack (TNS)
    double wirelength_um{0.0};       // Total routed wirelength
    double cell_area_um2{0.0};       // Total standard cell area
    double dynamic_power_mw{0.0};    // Switching + internal dynamic power
    double leakage_power_mw{0.0};    // Static sub-threshold leakage power
    double peak_temperature_c{25.0}; // Peak junction temperature
    double max_ir_drop_mv{0.0};      // Maximum PDN IR-drop
    uint32_t drc_violations{0};      // Design rule violation count
    int32_t congestion_overflow{0};  // Routing congestion overflow
};

struct PPAWeights {
    double w_freq{0.4};  // Weight on frequency / timing closure
    double w_power{0.3}; // Weight on power minimization
    double w_area{0.15}; // Weight on silicon area minimization
    double w_drc{0.15};  // Weight on DRC cleanliness
};

struct PPAAction {
    double delta_placement_density{0.0}; // Adjustment to electrostatic placement density weight lambda
    double delta_timing_weight{0.0};     // Adjustment to timing criticality exponent alpha
    int32_t ulvt_cell_swaps{0};          // Number of critical cells converted to ULVT (speed boost)
    int32_t hvt_cell_swaps{0};           // Number of non-critical cells converted to HVT (leakage reduction)
};

struct PPAOptimizationResult {
    PPAState initial_state;
    PPAState final_state;
    double initial_reward{0.0};
    double final_reward{0.0};
    uint32_t iterations_run{0};
    uint32_t total_ulvt_swaps{0};
    uint32_t total_hvt_swaps{0};
    bool converged{false};
    double runtime_seconds{0.0};
};

} // namespace vlsi::rl
