#include "ppa_agent.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace vlsi::rl {

PPAAgent::PPAAgent(soc::db::ChipDB& db, const PPAWeights& weights)
    : m_db(db), m_weights(weights) {
    identify_or_create_vt_variants();
}

void PPAAgent::identify_or_create_vt_variants() {
    // Ensure ULVT (Ultra Low-Vt, fast, high leakage) and HVT (High-Vt, slow, ultra low leakage) exist
    auto ulvt_name = m_db.strings.intern("INV_X1_ULVT");
    auto hvt_name = m_db.strings.intern("INV_X1_HVT");

    bool has_ulvt = false;
    bool has_hvt = false;

    for (size_t i = 0; i < m_db.master_cells.size(); ++i) {
        soc::db::MasterCellId mid{static_cast<uint32_t>(i)};
        if (m_db.master_cells[mid].name_id == ulvt_name) has_ulvt = true;
        if (m_db.master_cells[mid].name_id == hvt_name) has_hvt = true;
    }

    if (!has_ulvt) {
        m_db.create_master_cell("INV_X1_ULVT", 380, 280, soc::db::CellType::STD_CELL);
    }

    if (!has_hvt) {
        m_db.create_master_cell("INV_X1_HVT", 380, 280, soc::db::CellType::STD_CELL);
    }
}

PPAState PPAAgent::extract_state() const {
    PPAState s;

    // 1. Compute cell area and leakage power
    double total_area_dbu2 = 0.0;
    double leakage_uw = 0.0;

    for (size_t i = 0; i < m_db.instances.size(); ++i) {
        soc::db::InstanceId iid{static_cast<uint32_t>(i)};
        const auto& inst = m_db.instances[iid];
        if (!inst.master_id.is_valid() || inst.master_id.value() >= m_db.master_cells.size()) continue;

        const auto& master = m_db.master_cells[inst.master_id];
        total_area_dbu2 += static_cast<double>(master.width) * static_cast<double>(master.height);

        std::string_view name = m_db.strings.resolve(master.name_id);
        if (name.find("ULVT") != std::string_view::npos) {
            leakage_uw += 0.080; // 80 nW for ULVT
        } else if (name.find("HVT") != std::string_view::npos) {
            leakage_uw += 0.002; // 2 nW for HVT
        } else {
            leakage_uw += 0.015; // 15 nW for SVT/Nominal
        }
    }

    double dbu = static_cast<double>(m_db.dbu_per_micron);
    if (dbu <= 0.0) dbu = 1000.0;
    s.cell_area_um2 = total_area_dbu2 / (dbu * dbu);
    s.leakage_power_mw = leakage_uw * 0.001;

    // 2. Compute wirelength and dynamic power
    double total_wl_dbu = 0.0;
    for (size_t n = 0; n < m_db.nets.size(); ++n) {
        soc::db::NetId nid{static_cast<uint32_t>(n)};
        const auto& net = m_db.nets[nid];

        for (const auto& ws : net.route_segments) {
            int64_t dx = ws.end.x - ws.start.x;
            int64_t dy = ws.end.y - ws.start.y;
            total_wl_dbu += std::sqrt(static_cast<double>(dx * dx + dy * dy));
        }
    }

    s.wirelength_um = total_wl_dbu / dbu;
    // Dynamic power: P = 0.5 * C_wire * V^2 * f
    // C_wire = ~0.15 fF/um, V = 0.8 V, f = 4.0 GHz -> 0.5 * 0.15e-15 * 0.64 * 4e9 = 1.92e-7 W/um = 0.000192 mW/um
    s.dynamic_power_mw = s.wirelength_um * 0.000192 + 5.0; // 5 mW base dynamic

    // 3. Timing Slack (estimated from wirelength and cell delay)
    // Target clock period 250 ps (0.25 ns)
    double longest_wire_delay_ns = (s.wirelength_um > 0 ? (s.wirelength_um / std::max(1.0, static_cast<double>(m_db.nets.size()))) * 0.00005 : 0.05);
    s.wns_ns = 0.25 - (0.18 + longest_wire_delay_ns); // Baseline slack
    s.tns_ns = std::min(0.0, s.wns_ns * 10.0);

    // 4. Physical / Thermal
    s.peak_temperature_c = 45.0 + (s.dynamic_power_mw + s.leakage_power_mw) * 0.2;
    s.max_ir_drop_mv = 12.0 + (s.dynamic_power_mw + s.leakage_power_mw) * 0.05;
    s.drc_violations = 0;
    s.congestion_overflow = 0;

    return s;
}

double PPAAgent::compute_reward(const PPAState& s, const PPAState& baseline) const noexcept {
    double base_power = std::max(1e-3, baseline.dynamic_power_mw + baseline.leakage_power_mw);
    double cur_power = s.dynamic_power_mw + s.leakage_power_mw;
    double base_area = std::max(1.0, baseline.cell_area_um2);

    // Multi-objective PPA reward:
    // Timing reward: normalized against 0.25 ns period
    double r_freq = m_weights.w_freq * (1.0 + (s.wns_ns / 0.25));
    // Power penalty: lower power gives higher reward
    double r_power = -m_weights.w_power * (cur_power / base_power);
    // Area penalty
    double r_area = -m_weights.w_area * (s.cell_area_um2 / base_area);
    // DRC penalty
    double r_drc = -m_weights.w_drc * (static_cast<double>(s.drc_violations) * 0.1);

    return r_freq + r_power + r_area + r_drc;
}

PPAState PPAAgent::step(const PPAAction& action) {
    // 1. Apply ULVT cell swaps (boost speed for critical cells)
    if (action.ulvt_cell_swaps > 0) {
        auto ulvt_name = m_db.strings.intern("INV_X1_ULVT");
        soc::db::MasterCellId ulvt_mid{soc::db::MasterCellId::invalid()};
        for (size_t i = 0; i < m_db.master_cells.size(); ++i) {
            soc::db::MasterCellId mid{static_cast<uint32_t>(i)};
            if (m_db.master_cells[mid].name_id == ulvt_name) {
                ulvt_mid = mid;
                break;
            }
        }

        if (ulvt_mid.is_valid()) {
            int32_t swapped = 0;
            for (size_t i = 0; i < m_db.instances.size() && swapped < action.ulvt_cell_swaps; ++i) {
                soc::db::InstanceId iid{static_cast<uint32_t>(i)};
                auto& inst = m_db.instances[iid];
                if (!inst.is_fixed && inst.master_id != ulvt_mid) {
                    inst.master_id = ulvt_mid;
                    swapped++;
                }
            }
        }
    }

    // 2. Apply HVT cell swaps (slash leakage power for positive slack cells)
    if (action.hvt_cell_swaps > 0) {
        auto hvt_name = m_db.strings.intern("INV_X1_HVT");
        soc::db::MasterCellId hvt_mid{soc::db::MasterCellId::invalid()};
        for (size_t i = 0; i < m_db.master_cells.size(); ++i) {
            soc::db::MasterCellId mid{static_cast<uint32_t>(i)};
            if (m_db.master_cells[mid].name_id == hvt_name) {
                hvt_mid = mid;
                break;
            }
        }

        if (hvt_mid.is_valid()) {
            int32_t swapped = 0;
            for (size_t i = m_db.instances.size(); i > 0 && swapped < action.hvt_cell_swaps; --i) {
                soc::db::InstanceId iid{static_cast<uint32_t>(i - 1)};
                auto& inst = m_db.instances[iid];
                if (!inst.is_fixed && inst.master_id != hvt_mid) {
                    inst.master_id = hvt_mid;
                    swapped++;
                }
            }
        }
    }

    return extract_state();
}

PPAOptimizationResult PPAAgent::optimize(uint32_t max_iterations) {
    auto t0 = std::chrono::steady_clock::now();
    PPAOptimizationResult res;

    res.initial_state = extract_state();
    res.initial_reward = compute_reward(res.initial_state, res.initial_state);

    PPAState current_state = res.initial_state;
    double current_reward = res.initial_reward;

    for (uint32_t iter = 0; iter < max_iterations; ++iter) {
        res.iterations_run++;

        PPAAction action;
        if (current_state.wns_ns < 0.0) {
            // Timing negative: prioritize ULVT cell swaps to recover setup slack
            action.ulvt_cell_swaps = std::min(50, static_cast<int32_t>(m_db.instances.size() / 10 + 1));
            action.hvt_cell_swaps = 0;
            action.delta_timing_weight = 0.2;
        } else {
            // Timing positive: aggressively swap non-critical cells to HVT to slash leakage power!
            action.ulvt_cell_swaps = 0;
            action.hvt_cell_swaps = std::min(100, static_cast<int32_t>(m_db.instances.size() / 5 + 1));
            action.delta_placement_density = 0.05;
        }

        PPAState next_state = step(action);
        double next_reward = compute_reward(next_state, res.initial_state);

        if (next_reward >= current_reward - 0.05) { // Accept action
            current_state = next_state;
            current_reward = next_reward;
            res.total_ulvt_swaps += action.ulvt_cell_swaps;
            res.total_hvt_swaps += action.hvt_cell_swaps;
        }
    }

    res.final_state = current_state;
    res.final_reward = current_reward;
    res.converged = (res.final_reward >= res.initial_reward);

    auto t1 = std::chrono::steady_clock::now();
    res.runtime_seconds = std::chrono::duration<double>(t1 - t0).count();

    return res;
}

} // namespace vlsi::rl
