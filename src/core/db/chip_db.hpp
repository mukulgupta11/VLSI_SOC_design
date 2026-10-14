#pragma once

#include "id_types.hpp"
#include "arena_pool.hpp"
#include "rtree.hpp"
#include "observer.hpp"

#include <string>
#include <vector>
#include <unordered_map>
#include <string_view>
#include <optional>
#include <memory>

namespace soc::db {

// High-speed string interner mapping string to 32-bit StringId
class StringInterner {
public:
    StringId intern(std::string_view str) {
        auto it = m_map.find(str);
        if (it != m_map.end()) {
            return StringId{it->second};
        }
        uint32_t id = static_cast<uint32_t>(m_strings.size());
        m_strings.emplace_back(str);
        m_map[m_strings.back()] = id;
        return StringId{id};
    }

    [[nodiscard]] std::string_view resolve(StringId id) const noexcept {
        if (!id.is_valid() || id.value() >= m_strings.size()) {
            return "";
        }
        return m_strings[id.value()];
    }

    void clear() {
        m_strings.clear();
        m_map.clear();
    }

    [[nodiscard]] size_t size() const noexcept { return m_strings.size(); }

private:
    std::vector<std::string> m_strings;
    // Transparent string hash map
    struct StringViewHash {
        using is_transparent = void;
        size_t operator()(std::string_view sv) const noexcept {
            return std::hash<std::string_view>{}(sv);
        }
    };
    std::unordered_map<std::string, uint32_t, StringViewHash, std::equal_to<>> m_map;
};

// Physical / Logical Layer definition
struct Layer {
    LayerId id{LayerId::invalid()};
    StringId name_id{StringId::invalid()};
    uint16_t gds_layer{0};
    uint16_t gds_datatype{0};
    int32_t pitch{0};
    int32_t min_width{0};
    int32_t min_spacing{0};
    bool is_routing{false};
    bool is_cut{false};
};

// Cell Master Port definition
struct MasterPort {
    StringId pin_name{StringId::invalid()};
    SignalType signal_type{SignalType::SIGNAL};
    LayerId layer_id{LayerId::invalid()};
    Rect2D box; // Offset relative to master cell origin
};

// Library Master Cell (standard cell, macro, or IO)
struct MasterCell {
    MasterCellId id{MasterCellId::invalid()};
    StringId name_id{StringId::invalid()};
    int32_t width{0};
    int32_t height{0};
    CellType cell_type{CellType::STD_CELL};
    std::vector<MasterPort> ports;
    std::vector<Rect2D> obstructions;
    LayerId obstruction_layer{LayerId::invalid()};
};

// Physical Instance of a MasterCell
struct Instance {
    InstanceId id{InstanceId::invalid()};
    StringId name_id{StringId::invalid()};
    MasterCellId master_id{MasterCellId::invalid()};
    Point2D origin{0, 0};
    Orientation orientation{Orientation::R0};
    PowerDomainId power_domain_id{PowerDomainId::invalid()};
    BlockAbstractId parent_abstract_id{BlockAbstractId::invalid()};
    bool is_fixed{false};

    [[nodiscard]] Rect2D compute_bbox(int32_t width, int32_t height) const noexcept {
        // Compute oriented bounding box
        switch (orientation) {
            case Orientation::R90:
            case Orientation::R270:
            case Orientation::MY90:
            case Orientation::MX90:
                return Rect2D{origin.x, origin.y, origin.x + height, origin.y + width};
            default:
                return Rect2D{origin.x, origin.y, origin.x + width, origin.y + height};
        }
    }
};

// Pin connected to an Instance or Module boundary
struct Pin {
    PinId id{PinId::invalid()};
    StringId name_id{StringId::invalid()};
    InstanceId instance_id{InstanceId::invalid()};
    NetId net_id{NetId::invalid()};
    LayerId layer_id{LayerId::invalid()};
    Rect2D offset_box;
    SignalType signal_type{SignalType::SIGNAL};
};

// Routed wire segment on a specific physical layer
struct WireSegment {
    LayerId layer{LayerId::invalid()};
    Point2D start{0, 0};
    Point2D end{0, 0};
    int32_t width{0};
};

// Logical / Physical Net
struct Net {
    NetId id{NetId::invalid()};
    StringId name_id{StringId::invalid()};
    SignalType signal_type{SignalType::SIGNAL};
    PowerDomainId power_domain_id{PowerDomainId::invalid()};
    std::vector<PinId> pins;
    std::vector<WireSegment> route_segments;
    double lumped_cap_pf{0.0};
};

// IEEE 1801 Multi-Voltage Power Domain
struct PowerDomain {
    PowerDomainId id{PowerDomainId::invalid()};
    StringId name_id{StringId::invalid()};
    double nominal_voltage{0.8};
    Rect2D boundary;
    std::vector<InstanceId> member_instances;
    std::vector<NetId> internal_nets;
};

// Hierarchical Black-Box Block Abstract (P-Core, E-Core, GPU Slice, SLC, NPU)
struct BlockAbstract {
    BlockAbstractId id{BlockAbstractId::invalid()};
    StringId name_id{StringId::invalid()};
    std::string macro_type; // e.g. "P_CORE", "E_CORE", "GPU_CORE", "SLC", "NPU"
    Rect2D boundary;
    PowerDomainId power_domain_id{PowerDomainId::invalid()};
    std::vector<Rect2D> obstructions;
    std::vector<PinId> boundary_pins;
    std::vector<InstanceId> internal_instances;

    // Extracted Timing Model (ETM) parameters
    double input_setup_ns{0.050};
    double output_clk_to_q_ns{0.120};
};

// Routing Pitch Tracks
struct TrackGrid {
    TrackId id{TrackId::invalid()};
    LayerId layer_id{LayerId::invalid()};
    int32_t start_coord{0};
    int32_t step{0};
    uint32_t num_tracks{0};
    bool is_horizontal{false};
};

// Standard Via
struct Via {
    ViaId id{ViaId::invalid()};
    StringId name_id{StringId::invalid()};
    LayerId bottom_layer{LayerId::invalid()};
    LayerId top_layer{LayerId::invalid()};
    Rect2D bottom_box;
    Rect2D top_box;
    Rect2D cut_box;
};

// Multi-Die Interposer Micro-Bump (UltraFusion-style packaging)
struct MicroBump {
    uint32_t id{0};
    Point2D pos{0, 0};
    int32_t diameter{0};
    NetId net_id{NetId::invalid()};
    uint32_t source_die{0};
    uint32_t target_die{1};
};

// Backside Power Delivery Network (BSPDN) Nano-TSV
struct NanoTSV {
    uint32_t id{0};
    Point2D pos{0, 0};
    int32_t diameter{0};
    NetId power_net_id{NetId::invalid()};
    bool is_vdd{true};
};

// Forward declaration of ChipDB
class ChipDB {
public:
    ChipDB();
    ~ChipDB() = default;

    ChipDB(const ChipDB&) = delete;
    ChipDB& operator=(const ChipDB&) = delete;

    ChipDB(ChipDB&&) noexcept = default;
    ChipDB& operator=(ChipDB&&) noexcept = default;

    // Direct ArenaPool collections for all entities
    ArenaPool<MasterCell>    master_cells;
    ArenaPool<Instance>      instances;
    ArenaPool<Net>           nets;
    ArenaPool<Pin>           pins;
    ArenaPool<PowerDomain>   power_domains;
    ArenaPool<BlockAbstract> block_abstracts;
    ArenaPool<Layer>         layers;
    ArenaPool<TrackGrid>     tracks;
    ArenaPool<Via>           vias;

    // 3D-IC and Packaging Substrates
    std::vector<MicroBump>   micro_bumps;
    std::vector<NanoTSV>     nano_tsvs;

    // Spatial Indexing
    RTree<InstanceId>                  instance_rtree;
    std::unordered_map<uint32_t, RTree<uint32_t>> layer_rtrees; // layer_id.value() -> RTree

    // String Interner and Observer Bus
    StringInterner      strings;
    DatabaseObserverBus observers;

    // Die boundary & design metrics
    Rect2D   die_area{0, 0, 0, 0};
    uint32_t dbu_per_micron{1000};
    StringId design_name{StringId::invalid()};

    // Factory methods
    LayerId create_layer(std::string_view name, uint16_t gds_layer, uint16_t gds_datatype,
                         int32_t pitch, int32_t min_width, int32_t min_spacing,
                         bool is_routing, bool is_cut);

    MasterCellId create_master_cell(std::string_view name, int32_t width, int32_t height,
                                    CellType type = CellType::STD_CELL);

    InstanceId create_instance(std::string_view name, MasterCellId master_id,
                               Point2D origin, Orientation orient = Orientation::R0,
                               PowerDomainId domain = PowerDomainId::invalid(),
                               BlockAbstractId abstract_id = BlockAbstractId::invalid());

    void move_instance(InstanceId id, Point2D new_pos);

    NetId create_net(std::string_view name, SignalType signal_type = SignalType::SIGNAL,
                     PowerDomainId domain = PowerDomainId::invalid());

    PinId create_pin(std::string_view name, InstanceId inst_id, NetId net_id,
                     LayerId layer, Rect2D offset_box, SignalType sig_type = SignalType::SIGNAL);

    PowerDomainId create_power_domain(std::string_view name, double voltage, Rect2D boundary);

    BlockAbstractId create_block_abstract(std::string_view name, std::string_view macro_type,
                                          Rect2D boundary, PowerDomainId domain);

    // Bounding Box Calculation
    [[nodiscard]] Rect2D get_instance_bbox(InstanceId inst_id) const;

    // Spatial Queries
    void rebuild_instance_spatial_index();
    [[nodiscard]] std::vector<InstanceId> query_instances_in_box(const Rect2D& search_box) const;
    void query_instances_lod(const Rect2D& search_box, int32_t min_pixel_size_dbu,
                             std::vector<InstanceId>& visible_instances,
                             std::vector<Rect2D>& aggregated_clusters) const;

    // Database Reset
    void clear();
};

} // namespace soc::db
