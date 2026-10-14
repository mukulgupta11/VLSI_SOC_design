#include "core/db/chip_db.hpp"
#include "engines/place/floorplanner.hpp"
#include "engines/place/global_placer.hpp"
#include "engines/place/legalizer.hpp"
#include "engines/upf/upf_engine.hpp"

#include <iostream>
#include <cassert>
#include <chrono>
#include <random>

using namespace soc::db;
using namespace vlsi::place;
using namespace vlsi::upf;

int main() {
    std::cout << "========================================================\n";
    std::cout << "  VLSI SoC Studio: Phase 3 Place & UPF Verification    \n";
    std::cout << "  Target: 50,000-Cell FinFET/GAAFET Multi-Voltage SoC   \n";
    std::cout << "========================================================\n\n";

    ChipDB db;
    db.die_area = Rect2D{0, 0, 8000000, 8000000}; // 8 mm x 8 mm
    db.dbu_per_micron = 1000;

    // 1. Create Physical Layers (Metal1..Metal15)
    for (int l = 1; l <= 15; ++l) {
        std::string name = "metal" + std::to_string(l);
        int32_t pitch = (l <= 3) ? 48 : (l <= 8 ? 96 : 192);
        db.create_layer(name, l, 0, pitch, pitch / 2, pitch / 2, true, false);
    }

    // 2. Standard Cell Masters (quantized to 24 nm fin pitch, 240 nm row height)
    MasterCellId m_inv   = db.create_master_cell("INV_X1", 240, 240, CellType::STD_CELL);   // 10 fins x 1 row
    MasterCellId m_nand2 = db.create_master_cell("NAND2_X1", 480, 240, CellType::STD_CELL); // 20 fins x 1 row
    MasterCellId m_nor2  = db.create_master_cell("NOR2_X1", 480, 240, CellType::STD_CELL);  // 20 fins x 1 row
    MasterCellId m_dff   = db.create_master_cell("DFF_X1", 960, 240, CellType::STD_CELL);   // 40 fins x 1 row
    MasterCellId m_buf4  = db.create_master_cell("BUF_X4", 720, 240, CellType::STD_CELL);   // 30 fins x 1 row

    std::vector<MasterCellId> std_masters = {m_inv, m_nand2, m_nor2, m_dff, m_buf4};

    // 3. Define IEEE 1801 Multi-Voltage Power Domains
    std::cout << "[1/5] Initializing Multi-Voltage Power Domains & Block Abstracts...\n";
    PowerDomainId dom_cpu = db.create_power_domain("VDD_CPU", 1.15, Rect2D{0, 4000000, 4000000, 8000000});
    PowerDomainId dom_gpu = db.create_power_domain("VDD_GPU", 0.90, Rect2D{0, 0, 8000000, 4000000});
    PowerDomainId dom_soc = db.create_power_domain("VDD_SOC", 0.65, Rect2D{4000000, 4000000, 8000000, 8000000});

    // 4. Create Hierarchical Block Abstracts
    BlockAbstractId slc     = db.create_block_abstract("SLC_0", "SLC", Rect2D{3000000, 3600000, 5000000, 4400000}, dom_soc);
    BlockAbstractId pcore_0 = db.create_block_abstract("PCORE_0", "P_CORE", Rect2D{400000, 4500000, 1800000, 5900000}, dom_cpu);
    BlockAbstractId pcore_1 = db.create_block_abstract("PCORE_1", "P_CORE", Rect2D{2000000, 4500000, 3400000, 5900000}, dom_cpu);
    BlockAbstractId ecore_0 = db.create_block_abstract("ECORE_0", "E_CORE", Rect2D{4500000, 4500000, 5500000, 5500000}, dom_soc);
    BlockAbstractId ecore_1 = db.create_block_abstract("ECORE_1", "E_CORE", Rect2D{6000000, 4500000, 7000000, 5500000}, dom_soc);
    BlockAbstractId gpu_0   = db.create_block_abstract("GPU_0", "GPU_CORE", Rect2D{500000, 500000, 2000000, 2000000}, dom_gpu);
    BlockAbstractId gpu_1   = db.create_block_abstract("GPU_1", "GPU_CORE", Rect2D{2500000, 500000, 4000000, 2000000}, dom_gpu);

    // 5. Backside Power Delivery (BSPDN) Nano-TSVs
    std::cout << "  Generating Backside Power Delivery (BSPDN) Nano-TSV grid...\n";
    uint32_t tsv_id = 0;
    for (int32_t tx = 500000; tx < 7500000; tx += 1000000) {
        for (int32_t ty = 500000; ty < 7500000; ty += 1000000) {
            NanoTSV tsv;
            tsv.id = ++tsv_id;
            tsv.pos = Point2D{tx, ty};
            tsv.diameter = 80; // 80 nm diameter
            tsv.is_vdd = (tsv_id % 2 == 0);
            db.nano_tsvs.push_back(tsv);
        }
    }
    std::cout << "  Inserted " << db.nano_tsvs.size() << " BSPDN Nano-TSVs with 120 nm keepout zones.\n";

    // 6. Synthesize 50,000 Standard Cells across Power Domains
    std::cout << "[2/5] Synthesizing 50,000 standard cells across 3 voltage islands...\n";
    const uint32_t total_cells = 50000;
    std::mt19937 rng(42);
    std::uniform_int_distribution<size_t> master_dist(0, std_masters.size() - 1);
    std::uniform_real_distribution<double> rand_frac(0.0, 1.0);

    std::vector<InstanceId> cpu_cells;
    std::vector<InstanceId> gpu_cells;
    std::vector<InstanceId> soc_cells;
    std::vector<InstanceId> all_movable_cells;

    cpu_cells.reserve(15000);
    gpu_cells.reserve(20000);
    soc_cells.reserve(15000);
    all_movable_cells.reserve(total_cells);

    for (uint32_t i = 0; i < total_cells; ++i) {
        MasterCellId m_id = std_masters[master_dist(rng)];
        std::string name = "u_std_" + std::to_string(i);

        PowerDomainId dom;
        Point2D init_pt;

        // Partition: 30% CPU (1.15V), 40% GPU (0.90V), 30% SOC (0.65V)
        if (i < 15000) {
            dom = dom_cpu;
            init_pt.x = static_cast<int32_t>(200000 + rand_frac(rng) * 3600000);
            init_pt.y = static_cast<int32_t>(4200000 + rand_frac(rng) * 3600000);
        } else if (i < 35000) {
            dom = dom_gpu;
            init_pt.x = static_cast<int32_t>(200000 + rand_frac(rng) * 7600000);
            init_pt.y = static_cast<int32_t>(200000 + rand_frac(rng) * 3600000);
        } else {
            dom = dom_soc;
            init_pt.x = static_cast<int32_t>(4200000 + rand_frac(rng) * 3600000);
            init_pt.y = static_cast<int32_t>(4200000 + rand_frac(rng) * 3600000);
        }

        InstanceId inst_id = db.create_instance(name, m_id, init_pt, Orientation::R0, dom);
        all_movable_cells.push_back(inst_id);

        if (dom == dom_cpu) cpu_cells.push_back(inst_id);
        else if (dom == dom_gpu) gpu_cells.push_back(inst_id);
        else soc_cells.push_back(inst_id);
    }
    std::cout << "  Created " << all_movable_cells.size() << " instances ("
              << cpu_cells.size() << " CPU, " << gpu_cells.size() << " GPU, "
              << soc_cells.size() << " SOC).\n";

    // 7. Synthesize Hypergraph Nets (Internal nets + Cross-Domain Nets)
    std::cout << "  Synthesizing 20,000 hypergraph nets with cross-voltage boundary crossings...\n";
    LayerId m1{1};
    Rect2D pin_box{20, 20, 60, 60};

    // 7.1 Cross-voltage nets: SOC (0.65V) -> CPU (1.15V) (Low-to-High)
    for (uint32_t n = 0; n < 300; ++n) {
        NetId net_id = db.create_net("net_soc_to_cpu_" + std::to_string(n), SignalType::SIGNAL, dom_soc);
        InstanceId drv = soc_cells[n % soc_cells.size()];
        InstanceId sink = cpu_cells[n % cpu_cells.size()];

        PinId p_drv = db.create_pin("Y", drv, net_id, m1, pin_box);
        PinId p_sink = db.create_pin("A", sink, net_id, m1, pin_box);
        db.nets[net_id].pins.push_back(p_drv);
        db.nets[net_id].pins.push_back(p_sink);
    }

    // 7.2 Cross-voltage nets: GPU (0.90V switchable) -> SOC (0.65V) (Needs Isolation + High-to-Low)
    for (uint32_t n = 0; n < 300; ++n) {
        NetId net_id = db.create_net("net_gpu_to_soc_" + std::to_string(n), SignalType::SIGNAL, dom_gpu);
        InstanceId drv = gpu_cells[n % gpu_cells.size()];
        InstanceId sink = soc_cells[(n + 1000) % soc_cells.size()];

        PinId p_drv = db.create_pin("Y", drv, net_id, m1, pin_box);
        PinId p_sink = db.create_pin("A", sink, net_id, m1, pin_box);
        db.nets[net_id].pins.push_back(p_drv);
        db.nets[net_id].pins.push_back(p_sink);
    }

    // 7.3 Local nets within each domain to establish connectivity for global placer
    for (uint32_t n = 0; n < 15000; ++n) {
        NetId net_id = db.create_net("local_net_" + std::to_string(n), SignalType::SIGNAL);
        uint32_t c1 = n * 2;
        uint32_t c2 = (n * 2 + 1) % all_movable_cells.size();
        uint32_t c3 = (n * 2 + 2) % all_movable_cells.size();

        PinId p1 = db.create_pin("Y", all_movable_cells[c1], net_id, m1, pin_box);
        PinId p2 = db.create_pin("A", all_movable_cells[c2], net_id, m1, pin_box);
        PinId p3 = db.create_pin("B", all_movable_cells[c3], net_id, m1, pin_box);

        db.nets[net_id].pins.push_back(p1);
        db.nets[net_id].pins.push_back(p2);
        db.nets[net_id].pins.push_back(p3);
    }

    // 8. Test Step 1: Hierarchical Floorplanning & Pin Budgeting
    std::cout << "[3/5] Executing Hierarchical Floorplanner & Boundary Pin Budgeting...\n";
    HierarchicalFloorplanner floorplanner(db);
    bool fp_ok = floorplanner.floorplan_macros();
    assert(fp_ok);
    std::cout << "  Macro floorplanning completed successfully.\n";

    // Create 32-bit P-Core to SLC AXI Request Bus
    std::vector<NetId> pcore_bus;
    for (int b = 0; b < 32; ++b) {
        pcore_bus.push_back(db.create_net("pcore0_axi_req_" + std::to_string(b), SignalType::SIGNAL, dom_cpu));
    }
    floorplanner.order_bus_pins(pcore_0, slc, pcore_bus);

    // Create 32-bit GPU to SLC Memory Bus
    std::vector<NetId> gpu_bus;
    for (int b = 0; b < 32; ++b) {
        gpu_bus.push_back(db.create_net("gpu0_slc_data_" + std::to_string(b), SignalType::SIGNAL, dom_gpu));
    }
    floorplanner.order_bus_pins(gpu_0, slc, gpu_bus);

    (void)pcore_1; (void)ecore_0; (void)ecore_1; (void)gpu_1;

    bool pin_ok = floorplanner.assign_boundary_pins();
    assert(pin_ok);
    std::cout << "  Assigned " << (db.block_abstracts[pcore_0].boundary_pins.size() +
                                   db.block_abstracts[gpu_0].boundary_pins.size() +
                                   db.block_abstracts[slc].boundary_pins.size())
              << " ordered boundary pins across hierarchical block edges.\n";
    assert(db.block_abstracts[pcore_0].boundary_pins.size() == 32);
    assert(db.block_abstracts[gpu_0].boundary_pins.size() == 32);
    assert(db.block_abstracts[slc].boundary_pins.size() == 64);

    // 9. Test Step 2: Electrostatic Global Placement
    std::cout << "[4/5] Executing Electrostatic Analytical Global Placer (WA-HPWL + 2D Poisson)...\n";
    GlobalPlacerConfig gp_cfg;
    gp_cfg.max_iterations = 25;
    gp_cfg.target_density = 0.70;
    gp_cfg.learning_rate = 120.0;
    gp_cfg.gamma_wa = 4000.0;
    gp_cfg.bin_grid_x = 64;
    gp_cfg.bin_grid_y = 64;

    GlobalPlacer global_placer(db, gp_cfg);
    PlacerStats p_stats = global_placer.place();
    std::cout << "  Executed " << p_stats.iterations_executed << " Nesterov iterations in "
              << p_stats.runtime_seconds << " s.\n";
    std::cout << "  Initial HPWL: " << static_cast<int64_t>(p_stats.initial_hpwl)
              << " DBU -> Final HPWL: " << static_cast<int64_t>(p_stats.final_hpwl) << " DBU\n";
    std::cout << "  Density Overflow: " << (p_stats.initial_overflow * 100.0) << "% -> "
              << (p_stats.final_overflow * 100.0) << "%\n";

    // 10. Test Step 3: Sub-3nm FinFET / 2nm GAAFET Detailed Legalizer
    std::cout << "[5/5] Executing Sub-3nm FinFET Detailed Legalizer (Abacus DP + BSPDN Keepouts)...\n";
    LegalizerConfig leg_cfg;
    leg_cfg.fin_pitch = 24;          // 24 nm Fin-pitch
    leg_cfg.row_height = 240;        // 240 nm Row-height
    leg_cfg.ntsv_keepout_radius = 120; // 120 nm Nano-TSV keepout
    leg_cfg.search_rows = 6;

    DetailedLegalizer legalizer(db, leg_cfg);
    LegalizerStats l_stats = legalizer.legalize();

    std::cout << "  Legalized " << l_stats.cells_legalized << " cells in "
              << l_stats.runtime_seconds << " s.\n";
    std::cout << "  Average displacement: " << l_stats.avg_displacement_nm << " nm, Max: "
              << l_stats.max_displacement_nm << " nm.\n";
    std::cout << "  Verification Results:\n";
    std::cout << "    - Cell overlaps: " << l_stats.overlaps_detected << "\n";
    std::cout << "    - Fin-grid violations (x % 24 != 0): " << l_stats.fin_grid_violations << "\n";
    std::cout << "    - Row-alignment violations (y % 240 != 0): " << l_stats.row_alignment_violations << "\n";
    std::cout << "    - BSPDN Nano-TSV keepout violations: " << l_stats.ntsv_violations << "\n";
    std::cout << "    - Macro boundary violations: " << l_stats.macro_violations << "\n";

    assert(l_stats.overlaps_detected == 0);
    assert(l_stats.fin_grid_violations == 0);
    assert(l_stats.row_alignment_violations == 0);
    assert(l_stats.ntsv_violations == 0);
    assert(l_stats.macro_violations == 0);

    // 11. Test Step 4: IEEE 1801 UPF Multi-Voltage Engine
    std::cout << "\nExecuting IEEE 1801 UPF Multi-Voltage Engine...\n";
    UpfEngine upf(db);
    UpfStats u_stats = upf.insert_boundary_cells();

    std::cout << "  Scanned " << u_stats.boundary_nets_scanned << " boundary crossing nets in "
              << u_stats.runtime_seconds << " s.\n";
    std::cout << "  Inserted " << u_stats.level_shifters_inserted << " Level Shifter cells.\n";
    std::cout << "  Inserted " << u_stats.isolation_cells_inserted << " Isolation Clamp cells.\n";

    assert(u_stats.level_shifters_inserted > 0);
    assert(u_stats.isolation_cells_inserted > 0);

    // Stitch coarse-grain Header Sleep Switch Grid for switchable GPU domain
    uint32_t sw_count = upf.insert_header_switch_grid(dom_gpu, "gpu_sleep_en");
    std::cout << "  Stitched coarse-grain PMOS Header Sleep Switch grid ("
              << sw_count << " switches in VDD_GPU).\n";
    assert(sw_count == 64); // 8x8 grid

    // Verify 100% legal power intent compliance
    bool power_intent_valid = upf.verify_power_intent();
    std::cout << "  Power Intent Verification: " << (power_intent_valid ? "PASSED (100% Legal)" : "FAILED") << "\n";
    assert(power_intent_valid);

    std::cout << "\n========================================================\n";
    std::cout << "  PHASE 3 GLOBAL PLACER, LEGALIZER & UPF TESTS PASSED!  \n";
    std::cout << "========================================================\n";
    return 0;
}
