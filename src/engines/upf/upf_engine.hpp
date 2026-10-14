#pragma once

#include "core/db/chip_db.hpp"
#include <vector>
#include <string>
#include <string_view>
#include <unordered_set>

namespace vlsi::upf {

struct UpfConfig {
    double cpu_voltage{1.15};       // P-Core island VDD_CPU
    double gpu_voltage{0.90};       // Switchable GPU island VDD_GPU
    double soc_voltage{0.65};       // Always-on SOC island VDD_SOC
    int32_t switch_grid_cols{8};    // Header switch grid columns
    int32_t switch_grid_rows{8};    // Header switch grid rows
    int32_t switch_width{480};      // Header switch width (DBU)
    int32_t switch_height{240};     // Header switch height (DBU)
    double target_ir_drop_v{0.010}; // Max 10 mV IR-drop across switch grid
    double switch_cell_ron_ohms{50.0}; // Individual PMOS switch ON-resistance (Ohms)
    double cell_peak_current_ua{2.0};  // Average standard cell peak switching current (uA)
};

struct UpfStats {
    uint32_t boundary_nets_scanned{0};
    uint32_t level_shifters_inserted{0};
    uint32_t isolation_cells_inserted{0};
    uint32_t power_switches_inserted{0};
    double total_switch_conductance_mho{0.0};
    double estimated_ir_drop_mv{0.0};
    uint32_t unconstrained_crossings{0};
    double runtime_seconds{0.0};
};

class UpfEngine {
public:
    explicit UpfEngine(soc::db::ChipDB& db, const UpfConfig& config = {});

    // Parse and establish multi-voltage power domains
    void configure_power_domains();

    // Scan hypergraph nets crossing voltage boundaries and insert Level Shifters & Isolation Cells
    UpfStats insert_boundary_cells();

    // Stitch coarse-grain PMOS Header Sleep Switch grid in switchable power domains
    uint32_t insert_header_switch_grid(soc::db::PowerDomainId domain_id,
                                       std::string_view sleep_ctrl_net_name);

    // Verify all domain crossings have legal UPF protection
    [[nodiscard]] bool verify_power_intent() const;

    [[nodiscard]] const UpfConfig& config() const noexcept { return m_config; }

private:
    soc::db::ChipDB& m_db;
    UpfConfig m_config;

    // Set of switchable power domain IDs
    std::unordered_set<uint32_t> m_switchable_domains;

    // Helper to get or create MasterCell for UPF cells
    soc::db::MasterCellId get_or_create_master(std::string_view name,
                                              soc::db::CellType type,
                                              int32_t width, int32_t height);
};

} // namespace vlsi::upf
