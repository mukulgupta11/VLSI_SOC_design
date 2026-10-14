#pragma once

#include "core/db/chip_db.hpp"
#include <vector>
#include <string>
#include <cstdint>

namespace vlsi::cts {

struct CtsConfig {
    int32_t max_wire_length_um{400};   // Buffer insertion distance threshold (um)
    int32_t htree_depth{2};            // Depth of top-level symmetric H-Tree (1 = 4 clusters, 2 = 16 clusters)
    double target_skew_ps{10.0};       // Target max clock skew across all sinks (ps)
    double unit_r_ohm_per_um{0.05};    // Fat clock metal resistance (Ohms/um)
    double unit_c_ff_per_um{0.14};     // Fat clock metal capacitance (fF/um)
    double buffer_delay_ps{22.0};      // Clock buffer nominal delay (ps)
    double sink_input_cap_ff{1.5};     // Flip-flop clock pin input capacitance (fF)
};

struct CtsStats {
    uint32_t total_clock_sinks{0};
    uint32_t buffers_inserted{0};
    uint32_t clock_levels{0};
    double max_insertion_delay_ps{0.0};
    double min_insertion_delay_ps{0.0};
    double clock_skew_ps{0.0};
    double total_clock_wirelength_um{0.0};
    double runtime_seconds{0.0};
};

class CtsEngine {
public:
    explicit CtsEngine(soc::db::ChipDB& db, const CtsConfig& config = {});

    // Execute Zero-Skew Clock Tree Synthesis: Symmetric H-Tree + Deferred-Merge Embedding (DME)
    CtsStats synthesize_clock_tree(std::string_view clock_net_name = "clk_master");

    [[nodiscard]] const CtsConfig& config() const noexcept { return m_config; }

private:
    soc::db::ChipDB& m_db;
    CtsConfig m_config;

    // DME Tree Internal Node
    struct DmeNode {
        uint32_t id{0};
        soc::db::Point2D pos{0, 0};
        soc::db::Rect2D merge_segment; // Segment of zero-skew points
        double subtree_delay_ps{0.0};
        double subtree_cap_ff{0.0};
        int32_t left_child{-1};
        int32_t right_child{-1};
        soc::db::PinId sink_pin_id{soc::db::PinId::invalid()};
        bool is_leaf{false};
    };

    std::vector<DmeNode> m_dme_nodes;

    // Helper to get or create MasterCell for Clock Buffers
    soc::db::MasterCellId get_or_create_clk_buffer();

    // Bottom-Up DME Phase: compute zero-skew merging segments
    void dme_bottom_up(int32_t node_idx);

    // Top-Down DME Phase: embed exact node coordinates on merging segments
    void dme_top_down(int32_t node_idx, soc::db::Point2D parent_pos);
};

} // namespace vlsi::cts
