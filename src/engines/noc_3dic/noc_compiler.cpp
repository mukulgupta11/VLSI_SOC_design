#include "noc_compiler.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace vlsi::noc {

NoCCompiler::NoCCompiler(soc::db::ChipDB& db, const NoCConfig& config)
    : m_db(db), m_config(config) {}

soc::db::MasterCellId NoCCompiler::get_or_create_dff_master() {
    auto dff_name_id = m_db.strings.intern("DFF_X1");
    for (size_t i = 0; i < m_db.master_cells.size(); ++i) {
        soc::db::MasterCellId mid{static_cast<uint32_t>(i)};
        if (m_db.master_cells[mid].name_id == dff_name_id) {
            return mid;
        }
    }

    // Create DFF_X1 master cell if not present
    soc::db::MasterCellId dff_mid = m_db.create_master_cell("DFF_X1", 1200, 280, soc::db::CellType::STD_CELL);
    auto& dff = m_db.master_cells[dff_mid];

    // Add D, Q, CLK ports
    auto port_d = m_db.strings.intern("D");
    auto port_q = m_db.strings.intern("Q");
    auto port_clk = m_db.strings.intern("CLK");

    dff.ports.push_back({port_d, soc::db::SignalType::SIGNAL, soc::db::LayerId{1}, {100, 50, 200, 150}});
    dff.ports.push_back({port_q, soc::db::SignalType::SIGNAL, soc::db::LayerId{1}, {1000, 50, 1100, 150}});
    dff.ports.push_back({port_clk, soc::db::SignalType::CLOCK, soc::db::LayerId{1}, {500, 50, 600, 150}});

    return dff_mid;
}

void NoCCompiler::discover_nodes() {
    m_nodes.clear();
    uint32_t next_id = 0;

    // Scan block abstracts from ChipDB
    for (size_t i = 0; i < m_db.block_abstracts.size(); ++i) {
        soc::db::BlockAbstractId bid{static_cast<uint32_t>(i)};
        const auto& block = m_db.block_abstracts[bid];

        NoCNode node;
        node.id = next_id++;
        node.name = std::string(m_db.strings.resolve(block.name_id));
        node.bbox = block.boundary;
        node.port_pos = soc::db::Point2D{
            (block.boundary.x_min + block.boundary.x_max) / 2,
            (block.boundary.y_min + block.boundary.y_max) / 2
        };
        node.power_domain_id = block.power_domain_id;

        if (block.macro_type == "P_CORE") {
            node.type = NoCNodeType::CPU_PCORE;
            node.max_bandwidth_gbps = 512.0;
        } else if (block.macro_type == "E_CORE") {
            node.type = NoCNodeType::CPU_ECORE;
            node.max_bandwidth_gbps = 256.0;
        } else if (block.macro_type == "GPU_CORE") {
            node.type = NoCNodeType::GPU;
            node.max_bandwidth_gbps = 1024.0;
        } else if (block.macro_type == "NPU") {
            node.type = NoCNodeType::NPU;
            node.max_bandwidth_gbps = 512.0;
        } else if (block.macro_type == "SLC") {
            node.type = NoCNodeType::SLC;
            node.max_bandwidth_gbps = 2048.0;
        } else {
            node.type = NoCNodeType::ROUTER;
            node.max_bandwidth_gbps = 256.0;
        }

        m_nodes.push_back(node);
    }

    // If no block abstracts exist, synthesize a default 4x4 mesh across die
    if (m_nodes.empty()) {
        int32_t cols = 4;
        int32_t rows = 4;
        int32_t dx = m_db.die_area.width() / cols;
        int32_t dy = m_db.die_area.height() / rows;

        for (int32_t r = 0; r < rows; ++r) {
            for (int32_t c = 0; c < cols; ++c) {
                NoCNode node;
                node.id = next_id++;
                node.name = "NOC_NODE_R" + std::to_string(r) + "_C" + std::to_string(c);
                node.bbox = soc::db::Rect2D{
                    m_db.die_area.x_min + c * dx,
                    m_db.die_area.y_min + r * dy,
                    m_db.die_area.x_min + (c + 1) * dx,
                    m_db.die_area.y_min + (r + 1) * dy
                };
                node.port_pos = soc::db::Point2D{
                    node.bbox.x_min + dx / 2,
                    node.bbox.y_min + dy / 2
                };
                node.type = NoCNodeType::ROUTER;
                node.max_bandwidth_gbps = 512.0;
                m_nodes.push_back(node);
            }
        }
    }
}

void NoCCompiler::synthesize_mesh_topology() {
    m_links.clear();
    uint32_t link_id = 0;

    // Connect nodes based on spatial proximity (Manhattan nearest neighbors)
    for (size_t i = 0; i < m_nodes.size(); ++i) {
        for (size_t j = i + 1; j < m_nodes.size(); ++j) {
            const auto& na = m_nodes[i];
            const auto& nb = m_nodes[j];

            int64_t dx = std::abs(static_cast<int64_t>(nb.port_pos.x - na.port_pos.x));
            int64_t dy = std::abs(static_cast<int64_t>(nb.port_pos.y - na.port_pos.y));
            double length_um = static_cast<double>(dx + dy) / static_cast<double>(m_db.dbu_per_micron);

            // Connect if either:
            // 1. Manhattan distance <= 2500 um (adjacent in mesh)
            // 2. High-bandwidth link between compute core and SLC
            bool connect = (length_um <= 2500.0) ||
                           (na.type == NoCNodeType::SLC && (nb.type == NoCNodeType::CPU_PCORE || nb.type == NoCNodeType::GPU)) ||
                           (nb.type == NoCNodeType::SLC && (na.type == NoCNodeType::CPU_PCORE || na.type == NoCNodeType::GPU));

            if (connect) {
                NoCLink link;
                link.id = link_id++;
                link.src_node_id = na.id;
                link.dst_node_id = nb.id;
                link.bit_width = 256;
                link.length_um = length_um;

                // Wire flight RC delay: tau = 0.5 * r0 * c0 * L^2 (in picoseconds)
                // r0 in Ohm/um, c0 in fF/um, L in um -> 0.5 * 0.3 * 0.15 * L^2 * 1e-3 ps
                double tau_ps = 0.5 * m_config.unit_r_ohm_per_um * m_config.unit_c_ff_per_um * length_um * length_um * 0.001;
                link.wire_flight_delay_ps = tau_ps;

                m_links.push_back(link);
            }
        }
    }
}

void NoCCompiler::insert_pipeline_repeaters(NoCLink& link, NoCStats& stats) {
    if (link.wire_flight_delay_ps <= m_config.max_unpipelined_delay_ps) {
        link.repeater_stages = 0;
        return;
    }

    // Number of pipeline stages needed to close timing at 4.0 GHz
    uint32_t k = static_cast<uint32_t>(std::ceil(link.wire_flight_delay_ps / m_config.max_unpipelined_delay_ps));
    link.repeater_stages = k;

    soc::db::MasterCellId dff_mid = get_or_create_dff_master();
    const auto& na = m_nodes[link.src_node_id];
    const auto& nb = m_nodes[link.dst_node_id];

    for (uint32_t stage = 1; stage <= k; ++stage) {
        double ratio = static_cast<double>(stage) / static_cast<double>(k + 1);
        soc::db::Point2D rep_pos{
            static_cast<int32_t>(na.port_pos.x + ratio * (nb.port_pos.x - na.port_pos.x)),
            static_cast<int32_t>(na.port_pos.y + ratio * (nb.port_pos.y - na.port_pos.y))
        };

        std::string inst_name = "NOC_REP_L" + std::to_string(link.id) + "_S" + std::to_string(stage);
        soc::db::InstanceId inst_id = m_db.create_instance(inst_name, dff_mid, rep_pos);
        m_db.instances[inst_id].is_fixed = true;
        link.inserted_repeaters.push_back(inst_id);

        stats.total_repeaters_inserted++;
    }

    // Recompute segmented delay per stage: tau_seg = tau / (k+1)^2 + repeater_delay
    double tau_seg = (link.wire_flight_delay_ps / ((k + 1) * (k + 1))) + m_config.repeater_delay_ps;
    link.wire_flight_delay_ps = tau_seg;
}

NoCStats NoCCompiler::compile() {
    auto t0 = std::chrono::steady_clock::now();
    NoCStats stats;

    // 1. Discover nodes
    discover_nodes();
    stats.total_nodes = static_cast<uint32_t>(m_nodes.size());

    // 2. Synthesize mesh links
    synthesize_mesh_topology();
    stats.total_links = static_cast<uint32_t>(m_links.size());

    // 3. Insert pipeline repeaters for timing closure
    double max_delay = 0.0;
    double total_bw = 0.0;

    for (auto& link : m_links) {
        insert_pipeline_repeaters(link, stats);
        if (link.wire_flight_delay_ps > max_delay) {
            max_delay = link.wire_flight_delay_ps;
        }
        total_bw += (link.bit_width * m_config.target_freq_ghz) / 8000.0; // TB/s
    }

    stats.max_wire_delay_ps = max_delay;
    stats.total_bandwidth_tbps = total_bw;

    auto t1 = std::chrono::steady_clock::now();
    stats.runtime_seconds = std::chrono::duration<double>(t1 - t0).count();

    return stats;
}

} // namespace vlsi::noc
