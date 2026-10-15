#pragma once

#include "noc_types.hpp"
#include "core/db/chip_db.hpp"
#include <vector>

namespace vlsi::noc {

class NoCCompiler {
public:
    explicit NoCCompiler(soc::db::ChipDB& db, const NoCConfig& config = {});

    // Synthesize mesh NoC topology, compute wire delays, and insert pipeline repeater registers
    NoCStats compile();

    [[nodiscard]] const std::vector<NoCNode>& nodes() const noexcept { return m_nodes; }
    [[nodiscard]] const std::vector<NoCLink>& links() const noexcept { return m_links; }
    [[nodiscard]] const NoCConfig& config() const noexcept { return m_config; }

private:
    soc::db::ChipDB& m_db;
    NoCConfig m_config;

    std::vector<NoCNode> m_nodes;
    std::vector<NoCLink> m_links;

    void discover_nodes();
    void synthesize_mesh_topology();
    void insert_pipeline_repeaters(NoCLink& link, NoCStats& stats);
    soc::db::MasterCellId get_or_create_dff_master();
};

} // namespace vlsi::noc
