#include "core/db/chip_db.hpp"
#include "engines/sta/sta_engine.hpp"
#include "engines/cts/cts_engine.hpp"

#include <iostream>
#include <cassert>
#include <chrono>

using namespace soc::db;
using namespace vlsi::sta;
using namespace vlsi::cts;

#if defined(__GNUC__) && defined(_WIN32)
__attribute__((force_align_arg_pointer))
#endif
int main() {
    std::cout << "========================================================\n";
    std::cout << "  VLSI SoC Studio: Phase 4 STA & CTS Verification      \n";
    std::cout << "  Target: MCMM POCV Statistical STA + Zero-Skew CTS     \n";
    std::cout << "========================================================\n\n";

    ChipDB db;
    db.die_area = Rect2D{0, 0, 8000000, 8000000}; // 8 mm x 8 mm
    db.dbu_per_micron = 1000;

    // 1. Create Routing Layers
    for (int l = 1; l <= 15; ++l) {
        db.create_layer("metal" + std::to_string(l), l, 0, 48 * l, 24 * l, 24 * l, true, false);
    }

    // 2. Library Master Cells
    MasterCellId m_inv   = db.create_master_cell("INV_X1", 240, 240, CellType::STD_CELL);
    MasterCellId m_nand2 = db.create_master_cell("NAND2_X1", 480, 240, CellType::STD_CELL);
    MasterCellId m_dff   = db.create_master_cell("DFF_X1", 960, 240, CellType::STD_CELL);

    // Add ports to master cells
    LayerId m1{1};
    db.master_cells[m_dff].ports.push_back(MasterPort{db.strings.intern("CLK"), SignalType::CLOCK, m1, Rect2D{20, 60, 60, 180}});
    db.master_cells[m_dff].ports.push_back(MasterPort{db.strings.intern("D"), SignalType::SIGNAL, m1, Rect2D{100, 60, 140, 180}});
    db.master_cells[m_dff].ports.push_back(MasterPort{db.strings.intern("Q"), SignalType::SIGNAL, m1, Rect2D{880, 60, 920, 180}});

    db.master_cells[m_inv].ports.push_back(MasterPort{db.strings.intern("A"), SignalType::SIGNAL, m1, Rect2D{20, 60, 60, 180}});
    db.master_cells[m_inv].ports.push_back(MasterPort{db.strings.intern("Y"), SignalType::SIGNAL, m1, Rect2D{180, 60, 220, 180}});

    db.master_cells[m_nand2].ports.push_back(MasterPort{db.strings.intern("A"), SignalType::SIGNAL, m1, Rect2D{20, 60, 60, 180}});
    db.master_cells[m_nand2].ports.push_back(MasterPort{db.strings.intern("B"), SignalType::SIGNAL, m1, Rect2D{100, 60, 140, 180}});
    db.master_cells[m_nand2].ports.push_back(MasterPort{db.strings.intern("Y"), SignalType::SIGNAL, m1, Rect2D{400, 60, 440, 180}});

    // 3. Power Domains & Block Abstracts
    std::cout << "[1/4] Building Multi-Corner Multi-Mode (MCMM) Pipeline Netlist...\n";
    PowerDomainId dom_soc = db.create_power_domain("VDD_SOC", 0.80, db.die_area);
    BlockAbstractId pcore = db.create_block_abstract("PCORE_0", "P_CORE", Rect2D{500000, 4500000, 2500000, 6500000}, dom_soc);

    // Create boundary pins on PCORE abstract for ETM test
    PinId bp_in1 = db.create_pin("inst_in1", InstanceId::invalid(), NetId::invalid(), m1, Rect2D{500000, 5000000, 500050, 5000050});
    PinId bp_in2 = db.create_pin("inst_in2", InstanceId::invalid(), NetId::invalid(), m1, Rect2D{500000, 5200000, 500050, 5200050});
    PinId bp_out = db.create_pin("inst_out", InstanceId::invalid(), NetId::invalid(), m1, Rect2D{2500000, 5100000, 2500050, 5100050});
    db.block_abstracts[pcore].boundary_pins.push_back(bp_in1);
    db.block_abstracts[pcore].boundary_pins.push_back(bp_in2);
    db.block_abstracts[pcore].boundary_pins.push_back(bp_out);

    // 4. Construct a Realistic 16-Stage High-Frequency Pipeline Path
    // Flip-Flop FF0 -> INV -> NAND2 -> BUF -> NOR -> ... -> Flip-Flop FF1
    const int num_stages = 16;
    std::vector<InstanceId> dff_insts;
    dff_insts.reserve(1024);

    // Global master clock net
    NetId clk_net = db.create_net("clk_master", SignalType::CLOCK, dom_soc);

    // Create 512 distributed flip-flops for CTS zero-skew verification
    for (int i = 0; i < 512; ++i) {
        int32_t fx = 200000 + (i % 32) * 230000;
        int32_t fy = 200000 + (i / 32) * 450000;
        InstanceId dff_id = db.create_instance("u_reg_" + std::to_string(i), m_dff, Point2D{fx, fy}, Orientation::R0, dom_soc);
        dff_insts.push_back(dff_id);

        PinId clk_p = db.create_pin("CLK", dff_id, clk_net, m1, Rect2D{20, 60, 60, 180}, SignalType::CLOCK);
        db.nets[clk_net].pins.push_back(clk_p);
    }

    // Build critical datapath between FF0 and FF1
    InstanceId ff_src = dff_insts[0];
    InstanceId ff_dst = dff_insts[1];

    PinId q_src = db.create_pin("Q", ff_src, NetId::invalid(), m1, Rect2D{880, 60, 920, 180});
    PinId d_dst = db.create_pin("D", ff_dst, NetId::invalid(), m1, Rect2D{100, 60, 140, 180});

    PinId prev_out = q_src;
    std::vector<InstanceId> datapath_gates;

    for (int s = 0; s < num_stages; ++s) {
        MasterCellId m_gate = (s % 2 == 0) ? m_inv : m_nand2;
        int32_t gx = 250000 + s * 150000;
        int32_t gy = 200000;
        InstanceId gate_id = db.create_instance("u_dp_gate_" + std::to_string(s), m_gate, Point2D{gx, gy}, Orientation::R0, dom_soc);
        datapath_gates.push_back(gate_id);

        NetId net_stage = db.create_net("dp_net_" + std::to_string(s), SignalType::SIGNAL, dom_soc);
        db.pins[prev_out].net_id = net_stage;
        db.nets[net_stage].pins.push_back(prev_out);

        PinId in_p = db.create_pin("A", gate_id, net_stage, m1, Rect2D{20, 60, 60, 180});
        db.nets[net_stage].pins.push_back(in_p);

        if (m_gate == m_nand2) {
            NetId tie_net = db.create_net("dp_tie_" + std::to_string(s), SignalType::SIGNAL, dom_soc);
            PinId b_p = db.create_pin("B", gate_id, tie_net, m1, Rect2D{100, 60, 140, 180});
            db.nets[tie_net].pins.push_back(b_p);
        }

        prev_out = db.create_pin("Y", gate_id, NetId::invalid(), m1, Rect2D{180, 60, 220, 180});
    }

    // Connect last gate output to destination flip-flop D pin
    NetId last_net = db.create_net("dp_net_last", SignalType::SIGNAL, dom_soc);
    db.pins[prev_out].net_id = last_net;
    db.nets[last_net].pins.push_back(prev_out);
    db.pins[d_dst].net_id = last_net;
    db.nets[last_net].pins.push_back(d_dst);

    std::cout << "  Synthesized 512 register sinks and a " << num_stages << "-stage combinational datapath.\n";

    // 5. Test Step 1: Multi-Corner Multi-Mode (MCMM) Statistical STA with POCV
    std::cout << "\n[2/4] Executing Multi-Corner Multi-Mode POCV Static Timing Analysis...\n";
    StaConfig sta_cfg;
    sta_cfg.target_clock_freq_ghz = 3.0; // 3.0 GHz nominal signoff
    sta_cfg.clock_uncertainty_ps = 12.0; // 12 ps clock jitter margin
    sta_cfg.enable_pocv = true;

    StaEngine sta(db, sta_cfg);
    StaSummary summary = sta.update_timing();

    std::cout << "  Timing Update completed in " << summary.runtime_seconds << " s.\n";
    std::cout << "  Analyzed " << summary.total_endpoints << " timing check endpoints across 3 PVT corners.\n";
    std::cout << "  Worst Negative Slack (WNS 3-sigma): " << summary.wns_ps << " ps\n";
    std::cout << "  Total Negative Slack (TNS): " << summary.tns_ps << " ps\n";
    std::cout << "  Achievable Max Frequency: " << summary.max_frequency_ghz << " GHz\n";

    // Extract Critical Path
    TimingPath crit_path = sta.get_critical_path(CornerType::NOMINAL_080V_25C);
    std::cout << "  Critical Path from Pin #" << crit_path.startpoint.value()
              << " to Pin #" << crit_path.endpoint.value()
              << " (" << crit_path.path_pins.size() << " pins traversed):\n";
    std::cout << "    - Path delay (Gaussian): " << crit_path.total_delay.mean << " +/- "
              << crit_path.total_delay.sigma << " ps\n";
    std::cout << "    - Slack (Gaussian): " << crit_path.slack.mean << " +/- "
              << crit_path.slack.sigma << " ps (3-sigma yield: "
              << (crit_path.slack.mean - 3.0 * crit_path.slack.sigma) << " ps)\n";

    assert(!crit_path.path_pins.empty());
    assert(crit_path.startpoint.is_valid());
    assert(crit_path.endpoint.is_valid());

    // Compute Slack Histogram
    SlackHistogram hist = sta.compute_slack_histogram(CornerType::NOMINAL_080V_25C);
    std::cout << "  Slack Histogram: " << hist.bin_counts.size() << " bins, Worst Slack: "
              << hist.worst_negative_slack_ps << " ps, Violating: "
              << hist.total_violating_endpoints << "/" << hist.total_endpoints << "\n";

    // 6. Test Step 2: Extracted Timing Model (ETM) Abstractor
    std::cout << "\n[3/4] Extracting Hierarchical Timing Model (ETM) for Block Abstract...\n";
    ExtractedTimingModel etm = sta.extract_timing_model(pcore);
    std::cout << "  Generated ETM for '" << etm.block_name << "' at "
              << etm.core_frequency_ghz << " GHz (Clock period: " << etm.clock_period_ps << " ps):\n";
    std::cout << "    - Abstracted " << etm.boundary_pins.size() << " boundary pin timing arcs.\n";
    for (const auto& bp : etm.boundary_pins) {
        std::string_view pname = db.strings.resolve(bp.pin_name);
        if (bp.is_input) {
            std::cout << "      * Input Pin '" << pname << "': setup = "
                      << bp.setup_time.mean << " ps, hold = " << bp.hold_time.mean << " ps\n";
        } else {
            std::cout << "      * Output Pin '" << pname << "': clk_to_q = "
                      << bp.clk_to_q.mean << " ps, max_slew = " << bp.max_slew_ps << " ps\n";
        }
    }
    assert(etm.boundary_pins.size() == 3);

    // 7. Test Step 3: Zero-Skew Clock Tree Synthesis (DME + Symmetric H-Tree)
    std::cout << "\n[4/4] Executing Clock Tree Synthesis (Symmetric H-Tree + DME)...\n";
    CtsConfig cts_cfg;
    cts_cfg.target_skew_ps = 10.0;
    cts_cfg.htree_depth = 1;
    cts_cfg.max_wire_length_um = 350;

    CtsEngine cts(db, cts_cfg);
    CtsStats cts_stats = cts.synthesize_clock_tree("clk_master");

    std::cout << "  Synthesized clock tree in " << cts_stats.runtime_seconds << " s:\n";
    std::cout << "    - Sinks balanced: " << cts_stats.total_clock_sinks << "\n";
    std::cout << "    - Clock buffers inserted: " << cts_stats.buffers_inserted << "\n";
    std::cout << "    - Clock tree levels: " << cts_stats.clock_levels << "\n";
    std::cout << "    - Max Insertion Delay: " << cts_stats.max_insertion_delay_ps << " ps\n";
    std::cout << "    - Min Insertion Delay: " << cts_stats.min_insertion_delay_ps << " ps\n";
    std::cout << "    - Clock Skew: " << cts_stats.clock_skew_ps << " ps (Target: < "
              << cts_cfg.target_skew_ps << " ps)\n";
    std::cout << "    - Total Clock Wirelength: " << cts_stats.total_clock_wirelength_um << " um\n";

    assert(cts_stats.total_clock_sinks == 512);
    assert(cts_stats.buffers_inserted >= 4);
    assert(cts_stats.clock_skew_ps < cts_cfg.target_skew_ps);

    std::cout << "\n========================================================\n";
    std::cout << "  PHASE 4 STA & CTS VERIFICATION TESTS PASSED 100%!     \n";
    std::cout << "========================================================\n";
    return 0;
}
