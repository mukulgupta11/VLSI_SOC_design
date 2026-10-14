#pragma once

#include "core/db/chip_db.hpp"
#include <vector>
#include <cstdint>

namespace vlsi::place {

struct LegalizerConfig {
    int32_t fin_pitch{24};            // Sub-3nm / 2nm fin pitch = 24 nm
    int32_t row_height{240};          // Standard cell row height = 240 nm
    int32_t core_x_min{0};
    int32_t core_y_min{0};
    int32_t core_x_max{10000000};
    int32_t core_y_max{10000000};
    int32_t ntsv_keepout_radius{120}; // BSPDN nano-TSV exclusion keepout radius in nm
    int32_t search_rows{4};           // Search window +/- K candidate rows
    double y_displacement_weight{2.5}; // Penalty weight for moving rows
};

struct LegalizerStats {
    uint32_t cells_legalized{0};
    double total_displacement_nm{0.0};
    double max_displacement_nm{0.0};
    double avg_displacement_nm{0.0};
    uint32_t overlaps_detected{0};
    uint32_t fin_grid_violations{0};
    uint32_t row_alignment_violations{0};
    uint32_t ntsv_violations{0};
    uint32_t macro_violations{0};
    double runtime_seconds{0.0};
};

class DetailedLegalizer {
public:
    explicit DetailedLegalizer(soc::db::ChipDB& db, LegalizerConfig config = {});

    // Execute Abacus dynamic-programming row legalization with FinFET quantization and BSPDN keepout
    LegalizerStats legalize();

    // Verify legal placement criteria (zero overlaps, fin grid, row height, nTSV keepouts)
    [[nodiscard]] LegalizerStats verify_legal_placement() const;

    [[nodiscard]] const LegalizerConfig& config() const noexcept { return m_config; }

private:
    soc::db::ChipDB& m_db;
    LegalizerConfig m_config;

    // Sub-row interval representation
    struct SubRowInterval {
        int32_t x_min{0};
        int32_t x_max{0};
        int32_t remaining_width{0};
    };

    // Abacus Cluster
    struct Cluster {
        std::vector<uint32_t> cell_indices;
        double total_weight{0.0};
        double total_q{0.0};
        int32_t width{0};
        int32_t x_pos{0};
    };

    // Sub-row state during Abacus DP
    struct SubRowState {
        int32_t x_start{0};
        int32_t x_end{0};
        std::vector<Cluster> clusters;
    };

    // Row representation
    struct Row {
        int32_t row_index{0};
        int32_t y_coord{0};
        std::vector<SubRowState> sub_rows;
    };

    // Helper to snap coordinate to fin pitch grid
    [[nodiscard]] int32_t snap_to_fin(int32_t coord) const noexcept {
        int32_t p = m_config.fin_pitch;
        int32_t rem = coord % p;
        if (rem < 0) rem += p;
        if (rem >= p / 2) {
            return coord + (p - rem);
        } else {
            return coord - rem;
        }
    }
};

} // namespace vlsi::place
