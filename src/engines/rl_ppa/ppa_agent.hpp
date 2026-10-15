#pragma once

#include "rl_types.hpp"
#include "core/db/chip_db.hpp"

namespace vlsi::rl {

class PPAAgent {
public:
    explicit PPAAgent(soc::db::ChipDB& db, const PPAWeights& weights = {});

    // Extract current physical and electrical state tensor from ChipDB
    [[nodiscard]] PPAState extract_state() const;

    // Evaluate scalar multi-objective PPA reward relative to baseline
    [[nodiscard]] double compute_reward(const PPAState& state, const PPAState& baseline) const noexcept;

    // Apply an action to ChipDB (e.g. ULVT/HVT cell swaps, density adjustments)
    PPAState step(const PPAAction& action);

    // Run iterative reinforcement learning PPA optimization loop
    PPAOptimizationResult optimize(uint32_t max_iterations = 10);

    [[nodiscard]] const PPAWeights& weights() const noexcept { return m_weights; }

private:
    soc::db::ChipDB& m_db;
    PPAWeights m_weights;

    // Master cell IDs for fast ULVT vs low-power HVT cell variants
    void identify_or_create_vt_variants();
};

} // namespace vlsi::rl
