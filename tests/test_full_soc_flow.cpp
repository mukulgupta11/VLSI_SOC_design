#include "core/db/chip_db.hpp"
#include "core/parsers/gdsii_stream.hpp"
#include "engines/place/floorplanner.hpp"
#include "engines/place/global_placer.hpp"
#include "engines/place/legalizer.hpp"
#include "engines/upf/upf_engine.hpp"
#include "engines/cts/cts_engine.hpp"
#include "engines/sta/sta_engine.hpp"
#include "engines/route/global_router.hpp"
#include "engines/route/detailed_router.hpp"
#include "engines/noc_3dic/noc_compiler.hpp"
#include "engines/noc_3dic/interposer_router.hpp"
#include "engines/noc_3dic/thermal_ir_solver.hpp"
#include "engines/drc_lvs/sweep_line_drc.hpp"
#include "engines/drc_lvs/lvs_engine.hpp"
#include "engines/rl_ppa/ppa_agent.hpp"

#include <iostream>
#include <cassert>
#include <chrono>
#include <random>
#include <filesystem>
#include <map>

using namespace soc::db;
using namespace vlsi::place;
using namespace vlsi::upf;
using namespace vlsi::cts;
using namespace vlsi::sta;
using namespace vlsi::route;
using namespace vlsi::noc;
using namespace vlsi::drc_lvs;
using namespace vlsi::rl;

#if defined(__GNUC__) && defined(_WIN32)
__attribute__((force_align_arg_pointer))
#endif
int main() {
    std::cout << std::unitbuf;
    std::cout << "===================================================================\n";
    std::cout << "  VLSI SoC Studio: Phase 5 End-to-End RTL-to-GDSII Physical Flow   \n";
    std::cout << "  Target: Synthetic Apple M-Series SoC (Sub-3nm FinFET / GAAFET)  \n";
    std::cout << "===================================================================\n\n";

    ChipDB db;
    db.die_area = Rect2D{0, 0, 8000000, 8000000}; // 8 mm x 8 mm die
    db.dbu_per_micron = 1000;

    // 1. Setup Metal 1 to Metal 15 Layers
    for (int l = 1; l <= 15; ++l) {
        std::string name = "metal" + std::to_string(l);
        int32_t pitch = (l <= 3) ? 48 : (l <= 8 ? 96 : 192);
        db.create_layer(name, l, 0, pitch, pitch / 2, pitch / 2, true, false);
    }

    // 2. Setup Library Master Cells (FinFET quantized)
    MasterCellId m_inv   = db.create_master_cell("INV_X1", 240, 240, CellType::STD_CELL);
    MasterCellId m_nand2 = db.create_master_cell("NAND2_X1", 480, 240, CellType::STD_CELL);
    MasterCellId m_nor2  = db.create_master_cell("NOR2_X1", 480, 240, CellType::STD_CELL);
    MasterCellId m_dff   = db.create_master_cell("DFF_X1", 960, 240, CellType::STD_CELL);
    MasterCellId m_buf4  = db.create_master_cell("BUF_X4", 720, 240, CellType::STD_CELL);
    [[maybe_unused]] MasterCellId m_ls    = db.create_master_cell("LS_HL", 960, 240, CellType::LEVEL_SHIFTER);
    [[maybe_unused]] MasterCellId m_iso   = db.create_master_cell("ISO_AND", 720, 240, CellType::ISOLATION_CELL);

    // Add ports to master cells
    LayerId m1{0};
    db.master_cells[m_dff].ports.push_back(MasterPort{db.strings.intern("CLK"), SignalType::CLOCK, m1, Rect2D{20, 60, 60, 180}});
    db.master_cells[m_dff].ports.push_back(MasterPort{db.strings.intern("D"), SignalType::SIGNAL, m1, Rect2D{100, 60, 140, 180}});
    db.master_cells[m_dff].ports.push_back(MasterPort{db.strings.intern("Q"), SignalType::SIGNAL, m1, Rect2D{880, 60, 920, 180}});

    db.master_cells[m_inv].ports.push_back(MasterPort{db.strings.intern("A"), SignalType::SIGNAL, m1, Rect2D{20, 60, 60, 180}});
    db.master_cells[m_inv].ports.push_back(MasterPort{db.strings.intern("Y"), SignalType::SIGNAL, m1, Rect2D{180, 60, 220, 180}});

    db.master_cells[m_nand2].ports.push_back(MasterPort{db.strings.intern("A"), SignalType::SIGNAL, m1, Rect2D{20, 60, 60, 180}});
    db.master_cells[m_nand2].ports.push_back(MasterPort{db.strings.intern("B"), SignalType::SIGNAL, m1, Rect2D{100, 60, 140, 180}});
    db.master_cells[m_nand2].ports.push_back(MasterPort{db.strings.intern("Y"), SignalType::SIGNAL, m1, Rect2D{400, 60, 440, 180}});

    std::vector<MasterCellId> std_masters = {m_inv, m_nand2, m_nor2, m_dff, m_buf4};

    // 3. Define IEEE 1801 Multi-Voltage Power Domains
    std::cout << "[Step 1/14] Setting up IEEE 1801 Power Domains & Hierarchical Blocks...\n";
    PowerDomainId dom_cpu = db.create_power_domain("VDD_CPU", 1.15, Rect2D{0, 4000000, 4000000, 8000000});
    PowerDomainId dom_gpu = db.create_power_domain("VDD_GPU", 0.90, Rect2D{0, 0, 8000000, 4000000});
    PowerDomainId dom_soc = db.create_power_domain("VDD_SOC", 0.65, Rect2D{4000000, 4000000, 8000000, 8000000});

    // 4. Create Hierarchical Macro Blocks
    db.create_block_abstract("SLC_0", "SLC", Rect2D{3000000, 3600000, 5000000, 4400000}, dom_soc);
    db.create_block_abstract("PCORE_0", "P_CORE", Rect2D{400000, 4500000, 1800000, 5900000}, dom_cpu);
    db.create_block_abstract("PCORE_1", "P_CORE", Rect2D{2000000, 4500000, 3400000, 5900000}, dom_cpu);
    db.create_block_abstract("ECORE_0", "E_CORE", Rect2D{4500000, 4500000, 5500000, 5500000}, dom_soc);
    db.create_block_abstract("ECORE_1", "E_CORE", Rect2D{6000000, 4500000, 7000000, 5500000}, dom_soc);
    db.create_block_abstract("GPU_0", "GPU_CORE", Rect2D{500000, 500000, 2000000, 2000000}, dom_gpu);
    db.create_block_abstract("GPU_1", "GPU_CORE", Rect2D{2500000, 500000, 4000000, 2000000}, dom_gpu);
    db.create_block_abstract("NPU_0", "NPU", Rect2D{5500000, 1000000, 7500000, 3000000}, dom_soc);

    // 5. Backside Power Delivery (BSPDN) Nano-TSVs
    uint32_t tsv_idx = 0;
    for (int32_t x = 500000; x < 7500000; x += 1000000) {
        for (int32_t y = 500000; y < 7500000; y += 1000000) {
            db.nano_tsvs.push_back(NanoTSV{tsv_idx++, Point2D{x, y}, 40, NetId::invalid(), true});
        }
    }

    // 6. Synthesize Standard Cell Netlist
    std::cout << "[Step 2/14] Synthesizing Multi-Domain Standard Cell Netlist...\n";
    constexpr int NUM_CELLS = 1200;
    std::mt19937_64 rng(42);
    std::uniform_int_distribution<int> rand_master(0, static_cast<int>(std_masters.size() - 1));
    std::uniform_int_distribution<int> rand_x(100000, 7800000);
    std::uniform_int_distribution<int> rand_y(100000, 7800000);

    std::vector<InstanceId> inst_ids;
    inst_ids.reserve(NUM_CELLS);

    for (int i = 0; i < NUM_CELLS; ++i) {
        MasterCellId m = std_masters[rand_master(rng)];
        int32_t x = rand_x(rng);
        int32_t y = rand_y(rng);

        PowerDomainId dom = dom_soc;
        if (x < 4000000 && y >= 4000000) dom = dom_cpu;
        else if (y < 4000000) dom = dom_gpu;

        std::string name = "inst_" + std::to_string(i);
        InstanceId id = db.create_instance(name, m, Point2D{x, y}, Orientation::R0, dom);
        inst_ids.push_back(id);
    }

    // Create interconnect nets between distinct cell instances
    // Standard cell output pin 'Y' connects to another cell's input pin 'A'
    constexpr int NUM_NETS = 500;
    for (int n = 0; n < NUM_NETS; ++n) {
        std::string net_name = "sig_net_" + std::to_string(n);
        NetId net_id = db.create_net(net_name);

        InstanceId src_inst = inst_ids[n * 2];
        InstanceId dst_inst = inst_ids[n * 2 + 1];

        db.create_pin("Y", src_inst, net_id, LayerId{0}, Rect2D{180, 60, 220, 180});
        db.create_pin("A", dst_inst, net_id, LayerId{0}, Rect2D{20, 60, 60, 180});
    }

    // Add global master clock net and distributed flip-flops for CTS & STA
    NetId clk_net = db.create_net("clk_master", SignalType::CLOCK, dom_soc);
    std::vector<InstanceId> dff_pipeline;
    dff_pipeline.reserve(64);

    for (int i = 0; i < 64; ++i) {
        int32_t fx = 400000 + (i % 8) * 900000;
        int32_t fy = 400000 + (i / 8) * 900000;
        InstanceId dff_id = db.create_instance("u_reg_" + std::to_string(i), m_dff, Point2D{fx, fy}, Orientation::R0, dom_soc);
        dff_pipeline.push_back(dff_id);

        db.create_pin("CLK", dff_id, clk_net, m1, Rect2D{20, 60, 60, 180}, SignalType::CLOCK);
    }

    // Connect sequential datapath stages
    for (size_t i = 0; i < 63; ++i) {
        NetId dp_net = db.create_net("dp_stage_" + std::to_string(i), SignalType::SIGNAL, dom_soc);
        db.create_pin("Q", dff_pipeline[i], dp_net, m1, Rect2D{880, 60, 920, 180});
        db.create_pin("D", dff_pipeline[i + 1], dp_net, m1, Rect2D{100, 60, 140, 180});
    }

    // 7. Step 3: Hierarchical Floorplanning & Boundary Pin Assignment
    std::cout << "[Step 3/14] Running Hierarchical Floorplanner & Pin Budgeting...\n";
    HierarchicalFloorplanner floorplanner(db);
    bool fp_ok = floorplanner.floorplan_macros();
    bool pins_ok = floorplanner.assign_boundary_pins();
    std::cout << "  -> Floorplanning complete. Macros planned: " << (fp_ok ? "YES" : "NO")
              << ", Boundary pins assigned: " << (pins_ok ? "YES" : "NO") << "\n";
    assert(fp_ok);

    // 8. Step 4: Analytical Global Placement (WA-HPWL + Electrostatic FFT)
    std::cout << "[Step 4/14] Running Analytical Global Placement...\n";
    GlobalPlacerConfig gp_config;
    gp_config.max_iterations = 15;
    GlobalPlacer placer(db, gp_config);
    auto gp_stats = placer.place();
    std::cout << "  -> Global placement complete. Initial HPWL: " << gp_stats.initial_hpwl
              << " DBU, Final HPWL: " << gp_stats.final_hpwl
              << " DBU, Runtime: " << gp_stats.runtime_seconds << "s\n";
    assert(gp_stats.final_hpwl > 0);

    // 9. Step 5: Sub-3nm FinFET Detailed Legalization (Abacus)
    std::cout << "[Step 5/14] Running FinFET/GAAFET Detailed Row Legalization...\n";
    LegalizerConfig leg_config;
    leg_config.core_x_min = 100000;
    leg_config.core_x_max = 7900000;
    leg_config.core_y_min = 100000;
    leg_config.core_y_max = 7900000;
    DetailedLegalizer legalizer(db, leg_config);
    auto leg_stats = legalizer.legalize();
    std::cout << "  -> Legalization complete. Cells legalized: " << leg_stats.cells_legalized
              << ", Overlaps: " << leg_stats.overlaps_detected << "\n";
    auto leg_verify = legalizer.verify_legal_placement();
    assert(leg_verify.overlaps_detected == 0);

    // 10. Step 6: IEEE 1801 UPF Multi-Voltage Engine
    std::cout << "[Step 6/14] Running IEEE 1801 UPF Multi-Voltage Engine...\n";
    UpfEngine upf(db);
    upf.configure_power_domains();
    auto upf_stats = upf.insert_boundary_cells();
    uint32_t switches_inserted = upf.insert_header_switch_grid(dom_gpu, "sleep_gpu");
    assert(switches_inserted > 0);
    assert(upf_stats.unconstrained_crossings == 0);

    // Re-legalize to pack newly inserted Level Shifter and Isolation cells with zero overlaps
    auto post_upf_leg = legalizer.legalize();
    std::cout << "  -> Post-UPF Legalization complete. Cells legalized: "
              << post_upf_leg.cells_legalized << ", Overlaps: " << post_upf_leg.overlaps_detected << "\n";
    assert(post_upf_leg.overlaps_detected == 0);

    // 11. Step 7: Apple-Style Mesh NoC Interconnect & 4.0 GHz Repeater Synthesis
    std::cout << "[Step 7/14] Synthesizing Mesh NoC Interconnect & 4.0 GHz Repeaters...\n";
    NoCConfig noc_cfg;
    noc_cfg.target_freq_ghz = 4.0;
    noc_cfg.clock_period_ps = 250.0;
    noc_cfg.max_unpipelined_delay_ps = 180.0;
    NoCCompiler noc(db, noc_cfg);
    auto noc_stats = noc.compile();
    std::cout << "  -> NoC Interconnect synthesized. Nodes: " << noc_stats.total_nodes
              << ", Links: " << noc_stats.total_links
              << ", Repeaters inserted: " << noc_stats.total_repeaters_inserted
              << ", Max flight delay: " << noc_stats.max_wire_delay_ps << " ps\n";
    assert(noc_stats.total_nodes >= 8);
    assert(noc_stats.total_links > 0);
    assert(noc_stats.max_wire_delay_ps <= 250.0); // Timing closed at 4.0 GHz!

    // 12. Step 8: 3D-IC UltraFusion Micro-Bump & RDL Length-Matched Router
    std::cout << "[Step 8/14] Routing 3D-IC UltraFusion Micro-Bump & RDL Array...\n";
    InterposerRouter interposer(db, noc_cfg);
    auto if_stats = interposer.route_interposer();
    std::cout << "  -> UltraFusion complete. Microbumps routed: " << if_stats.microbumps_routed
              << ", Max RDL skew: " << if_stats.max_rdl_skew_ps << " ps\n";
    assert(if_stats.microbumps_routed == 64);
    assert(if_stats.max_rdl_skew_ps < 2.0); // Sub-2ps length-matched skew budget met!

    // 13. Step 9: Zero-Skew Clock Tree Synthesis (DME & H-Tree)
    std::cout << "[Step 9/14] Running Zero-Skew Clock Tree Synthesis...\n";
    CtsEngine cts(db);
    auto cts_stats = cts.synthesize_clock_tree("clk_master");
    std::cout << "  -> CTS complete. Clock sinks: " << cts_stats.total_clock_sinks
              << ", Skew: " << cts_stats.clock_skew_ps << " ps\n";
    assert(cts_stats.clock_skew_ps < 10.0);

    // 14. Step 10: Multi-Corner Multi-Mode STA (POCV)
    std::cout << "[Step 10/14] Running Multi-Corner Multi-Mode STA with POCV...\n";
    StaConfig sta_cfg;
    sta_cfg.target_clock_freq_ghz = 3.0;
    StaEngine sta(db, sta_cfg);
    sta.build_timing_graph();
    auto sta_summary = sta.update_timing();
    std::cout << "  -> STA complete. WNS: " << sta_summary.wns_ps
              << " ps, Endpoints: " << sta_summary.total_endpoints
              << ", Achievable Fmax: " << sta_summary.max_frequency_ghz << " GHz\n";
    assert(sta_summary.total_endpoints > 0);

    // 15. Step 11: 3D Congestion Global Maze Router (PathFinder)
    std::cout << "[Step 11/14] Running 3D GCell Global Router (PathFinder)...\n";
    GlobalRouterConfig gr_cfg;
    gr_cfg.gcell_width_um = 50;
    gr_cfg.gcell_height_um = 50;
    gr_cfg.max_iterations = 5;
    GlobalRouter global_router(db, gr_cfg);
    auto gr_stats = global_router.route();
    std::cout << "  -> Global routing complete. Routed nets: " << gr_stats.routed_nets
              << ", Total wirelength: " << gr_stats.total_wirelength_um << " um\n";
    assert(gr_stats.routed_nets > 0);

    // 16. Step 12: Track-Level Detailed Router & BSPDN Nano-TSV Generator
    std::cout << "[Step 12/14] Running Detailed Router & BSPDN Power Rails...\n";
    DetailedRouterConfig dr_cfg;
    dr_cfg.gcell_width_um = 50;
    dr_cfg.gcell_height_um = 50;
    DetailedRouter detailed_router(db, dr_cfg);
    auto dr_stats = detailed_router.route(global_router.routes());
    std::cout << "  -> Detailed routing complete. Wires placed, Total vias (inc. BSPDN): "
              << dr_stats.total_vias << "\n";
    assert(dr_stats.total_vias > 0);

    // 17. Step 13: 3D Finite-Difference Thermal & IR-Drop Solver
    std::cout << "[Step 13/14] Running 3D Thermal & PDN IR-Drop Finite-Difference Solver...\n";
    ThermalIRSolver thermal_ir(db, noc_cfg);
    auto therm_stats = thermal_ir.solve();
    std::cout << "  -> Thermal/IR-drop solve complete. Peak temp: " << therm_stats.peak_temp_c
              << " C, Worst IR-drop: " << therm_stats.worst_ir_drop_mv << " mV\n";
    assert(therm_stats.peak_temp_c <= 105.0); // Within Apple thermal ceiling
    assert(therm_stats.worst_ir_drop_mv <= 35.0); // Within 35 mV IR-drop budget

    // 18. Step 14: Bentley-Ottmann Sweep-Line DRC & LVS Verification
    std::cout << "[Step 14/14] Running Bentley-Ottmann DRC & LVS Verification...\n";
    SweepLineDRC drc(db);
    auto drc_report = drc.run();
    std::cout << "  -> DRC complete. Shorts: " << drc_report.short_violations
              << ", Opens: " << drc_report.open_violations
              << ", Total violations: " << drc_report.total_violations << "\n";
    std::map<uint32_t, int> shorts_by_layer;
    for (const auto& v : drc_report.violations) {
        if (v.type == DRCViolationType::SHORT_CIRCUIT) {
            shorts_by_layer[v.layer.value()]++;
        }
    }
    for (const auto& [l, c] : shorts_by_layer) {
        std::cout << "    [Layer " << l << " Shorts] " << c << "\n";
    }
    int s_cnt = 0, o_cnt = 0;
    for (const auto& v : drc_report.violations) {
        if (v.type == DRCViolationType::SHORT_CIRCUIT && s_cnt < 20) {
            std::cout << "    [DRC SHORT] Layer " << v.layer.value() << ": " << v.message
                      << " at [" << v.bbox.x_min << "," << v.bbox.y_min << "," << v.bbox.x_max << "," << v.bbox.y_max << "]\n";
            auto print_net_info = [&](NetId nid) {
                std::cout << "      Net " << nid.value() << " pins (" << db.nets[nid].pins.size() << "):";
                for (auto p : db.nets[nid].pins) {
                    if (!p.is_valid()) continue;
                    auto& pin = db.pins[p];
                    if (pin.instance_id.is_valid()) {
                        auto& inst = db.instances[pin.instance_id];
                        std::cout << " [" << db.strings.resolve(inst.name_id) << " orig=(" << inst.origin.x << "," << inst.origin.y << ") pin_box_center=(" << pin.offset_box.center().x << "," << pin.offset_box.center().y << ")]";
                    }
                }
                std::cout << "\n";
            };
            print_net_info(v.net_a);
            print_net_info(v.net_b);
            s_cnt++;
        }
        if (v.type == DRCViolationType::OPEN_CIRCUIT && o_cnt < 8) {
            std::cout << "    [DRC OPEN] Net " << v.net_a.value() << ": " << v.message << "\n";
            o_cnt++;
        }
    }
    assert(drc_report.short_violations == 0);
    assert(drc_report.open_violations == 0);

    LVSEngine lvs(db);
    auto lvs_report = lvs.run();
    std::cout << "  -> LVS complete. Matched nets: " << lvs_report.matched_nets
              << ", Matched instances: " << lvs_report.matched_instances
              << ", LVS Clean: " << (lvs_report.is_lvs_match ? "YES" : "NO") << "\n";
    assert(lvs_report.is_lvs_match);

    // 19. Bonus: Autonomous RL PPA Optimizer
    std::cout << "\n[RL PPA Optimizer] Running Autonomous Multi-Objective PPA Policy Optimization...\n";
    PPAAgent ppa_agent(db);
    auto ppa_res = ppa_agent.optimize(5);
    std::cout << "  -> Initial Reward: " << ppa_res.initial_reward
              << " -> Final Reward: " << ppa_res.final_reward
              << " (ULVT Swaps: " << ppa_res.total_ulvt_swaps
              << ", HVT Swaps: " << ppa_res.total_hvt_swaps << ")\n";
    assert(ppa_res.final_reward >= ppa_res.initial_reward);

    // 20. Binary GDSII Stream-Out
    std::string gds_file = "apple_m_series_soc_signoff.gds";
    std::cout << "\nStreaming out binary GDSII to " << gds_file << "...\n";
    bool gds_ok = soc::parsers::GdsiiWriter::export_chip_db(db, gds_file);
    assert(gds_ok);
    assert(std::filesystem::exists(gds_file));
    auto file_size = std::filesystem::file_size(gds_file);
    std::cout << "GDSII written successfully! File size: " << file_size << " bytes.\n";
    assert(file_size > 1000);

    std::cout << "\n===================================================================\n";
    std::cout << "  [SUCCESS] All 14 Physical Flow Steps & DRC/LVS/PPA Signoff Passed! \n";
    std::cout << "===================================================================\n";
    return 0;
}
