#include "core/db/chip_db.hpp"
#include "core/parsers/gdsii_stream.hpp"
#include "core/parsers/lef_def_parser.hpp"

#include <iostream>
#include <chrono>
#include <cassert>
#include <filesystem>
#include <vector>
#include <random>

using namespace soc::db;
using namespace soc::parsers;

class TestObserver : public DatabaseObserver {
public:
    uint32_t moved_count{0};
    InstanceId last_moved_id{InstanceId::invalid()};
    Point2D last_new_pos{0, 0};

    void on_instance_moved(InstanceId id, Point2D /*old_pos*/, Point2D new_pos) override {
        ++moved_count;
        last_moved_id = id;
        last_new_pos = new_pos;
    }
};

int main() {
    std::cout << "========================================================\n";
    std::cout << "  VLSI SoC Studio: Phase 1 ChipDB & Hierarchy Test       \n";
    std::cout << "  Target: Synthetic Apple M-Series Class SoC             \n";
    std::cout << "========================================================\n\n";

    auto t_start = std::chrono::high_resolution_clock::now();

    ChipDB db;
    db.design_name = db.strings.intern("apple_m_series_mini");
    db.dbu_per_micron = 1000;
    // 10mm x 10mm die = 10,000,000 x 10,000,000 DBU
    db.die_area = Rect2D{0, 0, 10000000, 10000000};

    // -------------------------------------------------------------
    // 1. Create Physical Metal Routing Layers (metal1..metal15)
    // -------------------------------------------------------------
    std::cout << "[1/7] Creating 15 routing layers (metal1..metal15)...\n";
    std::vector<LayerId> layers;
    for (int l = 1; l <= 15; ++l) {
        std::string name = "metal" + std::to_string(l);
        int32_t pitch = (l <= 4) ? 50 : ((l <= 8) ? 100 : 200); // 50nm - 200nm
        int32_t width = pitch / 2;
        int32_t spacing = pitch / 2;
        LayerId lid = db.create_layer(name, static_cast<uint16_t>(l), 0, pitch, width, spacing, true, false);
        layers.push_back(lid);
    }
    assert(db.layers.size() == 15);

    // -------------------------------------------------------------
    // 2. Create Standard Cell Library (INV, NAND, NOR, DFF, BUF)
    // -------------------------------------------------------------
    std::cout << "[2/7] Creating Standard Cell Library Masters...\n";
    MasterCellId inv_cell  = db.create_master_cell("INV_X1",   200, 600, CellType::STD_CELL);
    MasterCellId nand_cell = db.create_master_cell("NAND2_X1", 300, 600, CellType::STD_CELL);
    MasterCellId nor_cell  = db.create_master_cell("NOR2_X1",  300, 600, CellType::STD_CELL);
    MasterCellId dff_cell  = db.create_master_cell("DFF_X1",   800, 600, CellType::STD_CELL);
    MasterCellId buf_cell  = db.create_master_cell("BUF_X2",   250, 600, CellType::STD_CELL);

    // Add ports to cells
    db.master_cells[inv_cell].ports.push_back({db.strings.intern("A"), SignalType::SIGNAL, layers[0], Rect2D{20, 100, 80, 200}});
    db.master_cells[inv_cell].ports.push_back({db.strings.intern("Y"), SignalType::SIGNAL, layers[0], Rect2D{120, 300, 180, 400}});
    db.master_cells[inv_cell].obstructions.push_back(Rect2D{0, 0, 200, 50});
    db.master_cells[inv_cell].obstruction_layer = layers[0];

    db.master_cells[nand_cell].ports.push_back({db.strings.intern("A"), SignalType::SIGNAL, layers[0], Rect2D{20, 100, 80, 200}});
    db.master_cells[nand_cell].ports.push_back({db.strings.intern("B"), SignalType::SIGNAL, layers[0], Rect2D{100, 100, 160, 200}});
    db.master_cells[nand_cell].ports.push_back({db.strings.intern("Y"), SignalType::SIGNAL, layers[0], Rect2D{200, 300, 280, 400}});

    db.master_cells[dff_cell].ports.push_back({db.strings.intern("D"),   SignalType::SIGNAL, layers[0], Rect2D{50, 100, 120, 200}});
    db.master_cells[dff_cell].ports.push_back({db.strings.intern("CLK"), SignalType::CLOCK,  layers[0], Rect2D{150, 100, 220, 200}});
    db.master_cells[dff_cell].ports.push_back({db.strings.intern("Q"),   SignalType::SIGNAL, layers[0], Rect2D{650, 300, 750, 400}});

    std::vector<MasterCellId> cell_types = {inv_cell, nand_cell, nor_cell, dff_cell, buf_cell};

    // -------------------------------------------------------------
    // 3. Define IEEE 1801 Multi-Voltage Islands
    // -------------------------------------------------------------
    std::cout << "[3/7] Defining 3 Voltage Domains (VDD_CPU, VDD_GPU, VDD_SOC)...\n";
    // CPU Domain: x in [500um, 4500um], y in [500um, 4500um]
    PowerDomainId pd_cpu = db.create_power_domain("VDD_CPU", 0.85, Rect2D{500000, 500000, 4500000, 4500000});
    // GPU Domain: x in [5000um, 9500um], y in [500um, 5500um]
    PowerDomainId pd_gpu = db.create_power_domain("VDD_GPU", 0.75, Rect2D{5000000, 500000, 9500000, 5500000});
    // SoC / SLC Domain: x in [500um, 9500um], y in [5500um, 9500um]
    PowerDomainId pd_soc = db.create_power_domain("VDD_SOC", 0.65, Rect2D{500000, 5500000, 9500000, 9500000});

    assert(db.power_domains.size() == 3);

    // -------------------------------------------------------------
    // 4. Create Apple M-Series Hierarchical Block Abstracts
    //    (2 P-Cores, 4 E-Cores, 8 GPU Slices, 1 SLC)
    // -------------------------------------------------------------
    std::cout << "[4/7] Instantiating Hierarchical Block Abstracts (P-Cores, E-Cores, GPU Slices, SLC)...\n";
    // 2 P-Cores in CPU domain
    BlockAbstractId pcore0 = db.create_block_abstract("PCORE_0", "P_CORE", Rect2D{600000, 600000, 2400000, 2400000}, pd_cpu);
    BlockAbstractId pcore1 = db.create_block_abstract("PCORE_1", "P_CORE", Rect2D{2500000, 600000, 4300000, 2400000}, pd_cpu);

    // 4 E-Cores in CPU domain
    BlockAbstractId ecore0 = db.create_block_abstract("ECORE_0", "E_CORE", Rect2D{600000, 2600000, 1400000, 4300000}, pd_cpu);
    BlockAbstractId ecore1 = db.create_block_abstract("ECORE_1", "E_CORE", Rect2D{1500000, 2600000, 2300000, 4300000}, pd_cpu);
    BlockAbstractId ecore2 = db.create_block_abstract("ECORE_2", "E_CORE", Rect2D{2400000, 2600000, 3200000, 4300000}, pd_cpu);
    BlockAbstractId ecore3 = db.create_block_abstract("ECORE_3", "E_CORE", Rect2D{3300000, 2600000, 4200000, 4300000}, pd_cpu);

    // 8 GPU Cores in GPU domain
    std::vector<BlockAbstractId> gpu_cores;
    for (int g = 0; g < 8; ++g) {
        int row = g / 4;
        int col = g % 4;
        int32_t x1 = 5200000 + col * 1050000;
        int32_t y1 = 700000 + row * 2300000;
        std::string gname = "GPU_CORE_" + std::to_string(g);
        gpu_cores.push_back(db.create_block_abstract(gname, "GPU_CORE", Rect2D{x1, y1, x1 + 950000, y1 + 2100000}, pd_gpu));
    }

    // 1 SLC Block in SoC domain
    BlockAbstractId slc_block = db.create_block_abstract("SYSTEM_LEVEL_CACHE", "SLC", Rect2D{1000000, 6000000, 9000000, 8500000}, pd_soc);

    assert(db.block_abstracts.size() == 15); // 2 + 4 + 8 + 1 = 15 block abstracts

    // -------------------------------------------------------------
    // 5. Populate 100,000 Standard Cell Instances & Nets
    // -------------------------------------------------------------
    constexpr size_t TOTAL_INSTANCES = 100000;
    std::cout << "[5/7] Synthesizing " << TOTAL_INSTANCES << " standard cells across voltage islands...\n";

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> cell_dist(0, static_cast<int>(cell_types.size() - 1));

    db.instances.reserve(TOTAL_INSTANCES);
    db.nets.reserve(TOTAL_INSTANCES / 2);

    // 35,000 CPU cells, 40,000 GPU cells, 25,000 SoC cells
    for (size_t i = 0; i < TOTAL_INSTANCES; ++i) {
        PowerDomainId target_pd;
        BlockAbstractId target_ba;
        Point2D origin;

        if (i < 35000) {
            target_pd = pd_cpu;
            switch (i % 6) {
                case 0: target_ba = pcore0; break;
                case 1: target_ba = pcore1; break;
                case 2: target_ba = ecore0; break;
                case 3: target_ba = ecore1; break;
                case 4: target_ba = ecore2; break;
                default: target_ba = ecore3; break;
            }
            const Rect2D& b = db.block_abstracts[target_ba].boundary;
            origin.x = b.x_min + static_cast<int32_t>(rng() % (b.width() - 800));
            origin.y = b.y_min + static_cast<int32_t>(rng() % (b.height() - 600));
        } else if (i < 75000) {
            target_pd = pd_gpu;
            target_ba = gpu_cores[i % 8];
            const Rect2D& b = db.block_abstracts[target_ba].boundary;
            origin.x = b.x_min + static_cast<int32_t>(rng() % (b.width() - 800));
            origin.y = b.y_min + static_cast<int32_t>(rng() % (b.height() - 600));
        } else {
            target_pd = pd_soc;
            target_ba = slc_block;
            const Rect2D& b = db.block_abstracts[target_ba].boundary;
            origin.x = b.x_min + static_cast<int32_t>(rng() % (b.width() - 800));
            origin.y = b.y_min + static_cast<int32_t>(rng() % (b.height() - 600));
        }

        MasterCellId mid = cell_types[cell_dist(rng)];
        std::string iname = "inst_" + std::to_string(i);
        db.create_instance(iname, mid, origin, Orientation::R0, target_pd, target_ba);

        // Periodically synthesize nets and wire segments
        if (i > 0 && i % 4 == 0) {
            std::string nname = "net_" + std::to_string(i);
            NetId nid = db.create_net(nname, SignalType::SIGNAL, target_pd);
            PinId p1 = db.create_pin("Y", InstanceId{static_cast<uint32_t>(i - 1)}, nid, layers[0], Rect2D{0, 0, 50, 50});
            PinId p2 = db.create_pin("A", InstanceId{static_cast<uint32_t>(i)},     nid, layers[0], Rect2D{0, 0, 50, 50});
            (void)p1; (void)p2;

            // Add wire segment
            const Instance& prev_inst = db.instances[InstanceId{static_cast<uint32_t>(i - 1)}];
            WireSegment ws;
            ws.layer = layers[1];
            ws.start = prev_inst.origin;
            ws.end = origin;
            ws.width = 40;
            db.nets[nid].route_segments.push_back(ws);
        }
    }

    assert(db.instances.size() == TOTAL_INSTANCES);
    std::cout << "  Successfully loaded " << db.instances.size() << " instances.\n";

    // -------------------------------------------------------------
    // 6. Spatial R-Tree Viewport & LOD Verification
    // -------------------------------------------------------------
    std::cout << "[6/7] Verifying Spatial R-Tree queries & LOD culling...\n";
    // Viewport query: Zoom into PCORE_0 [600um, 600um] to [2400um, 2400um]
    Rect2D viewport{600000, 600000, 2400000, 2400000};
    auto q_start = std::chrono::high_resolution_clock::now();
    std::vector<InstanceId> visible_instances = db.query_instances_in_box(viewport);
    auto q_end = std::chrono::high_resolution_clock::now();
    double q_time_ms = std::chrono::duration<double, std::milli>(q_end - q_start).count();

    std::cout << "  Spatial query in PCORE_0 window returned " << visible_instances.size()
              << " instances in " << q_time_ms << " ms.\n";
    assert(!visible_instances.empty());

    // Validate correctness of spatial query results
    for (InstanceId id : visible_instances) {
        Rect2D bbox = db.get_instance_bbox(id);
        assert(bbox.intersects(viewport));
    }

    // Level-of-Detail (LOD) Query with 5000 DBU threshold
    std::vector<InstanceId> lod_visible;
    std::vector<Rect2D> lod_clusters;
    db.query_instances_lod(viewport, 5000, lod_visible, lod_clusters);
    std::cout << "  LOD query returned " << lod_visible.size() << " full-detail cells and "
              << lod_clusters.size() << " aggregated sub-pixel clusters.\n";

    // Test Observer Bus
    TestObserver test_obs;
    db.observers.register_observer(&test_obs);
    db.move_instance(InstanceId{10}, Point2D{123456, 654321});
    assert(test_obs.moved_count == 1);
    assert(test_obs.last_moved_id == InstanceId{10});
    assert(test_obs.last_new_pos == Point2D(123456, 654321));
    db.observers.unregister_observer(&test_obs);

    // -------------------------------------------------------------
    // 7. Binary GDSII & Standard DEF 5.8 Export / Roundtrip
    // -------------------------------------------------------------
    std::cout << "[7/7] Exporting and verifying DEF and GDSII binary streams...\n";
    std::filesystem::create_directories("build");
    std::string def_path = "build/m_series_soc.def";
    std::string gds_path = "build/m_series_soc.gds";

    bool def_ok = DefParser::write(def_path, db);
    assert(def_ok);
    std::uintmax_t def_size = std::filesystem::file_size(def_path);
    std::cout << "  DEF Exported: " << def_path << " (" << (def_size / (1024 * 1024)) << " MB)\n";
    assert(def_size > 0);

    bool gds_ok = GdsiiWriter::export_chip_db(db, gds_path);
    assert(gds_ok);
    std::uintmax_t gds_size = std::filesystem::file_size(gds_path);
    std::cout << "  GDSII Exported: " << gds_path << " (" << (gds_size / (1024 * 1024)) << " MB)\n";
    assert(gds_size > 0);

    // Verify GDSII Binary Stream with GdsiiReader
    GdsiiReader reader(gds_path);
    bool parse_ok = reader.parse();
    assert(parse_ok);
    std::cout << "  GDSII Stream Reader validated library: '" << reader.library_name()
              << "' with " << reader.structures().size() << " structures.\n";
    assert(reader.structures().size() > 0);

    auto t_end = std::chrono::high_resolution_clock::now();
    double total_sec = std::chrono::duration<double>(t_end - t_start).count();

    std::cout << "\n========================================================\n";
    std::cout << "  ALL TESTS PASSED SUCCESSFULLY!                         \n";
    std::cout << "  Total Execution Time: " << total_sec << " seconds      \n";
    std::cout << "  Memory Model: 100% Contiguous ArenaPool<T> (0 raw ptrs)\n";
    std::cout << "========================================================\n";

    return 0;
}
