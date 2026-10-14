#pragma once

#include "core/db/chip_db.hpp"
#include <vector>
#include <string_view>

namespace vlsi::place {

enum class BlockEdge : uint8_t {
    NORTH = 0,
    SOUTH,
    EAST,
    WEST
};

struct FloorplanConfig {
    int32_t die_width{10000000};   // 10 mm (in DBU)
    int32_t die_height{10000000};  // 10 mm
    int32_t core_margin_x{200000}; // 200 um margin
    int32_t core_margin_y{200000}; // 200 um margin
    int32_t macro_halo{40000};     // 40 um keepout around macros
    int32_t pin_pitch{2000};       // 2 um pitch between boundary pins
    soc::db::LayerId pin_layer_h{soc::db::LayerId{3}}; // Metal3 (horizontal tracks)
    soc::db::LayerId pin_layer_v{soc::db::LayerId{4}}; // Metal4 (vertical tracks)
};

struct BoundaryPinBudget {
    soc::db::PinId pin_id{soc::db::PinId::invalid()};
    soc::db::BlockAbstractId block_id{soc::db::BlockAbstractId::invalid()};
    soc::db::NetId net_id{soc::db::NetId::invalid()};
    BlockEdge edge{BlockEdge::NORTH};
    soc::db::Point2D location{0, 0};
    double timing_budget_ps{0.0};
};

class HierarchicalFloorplanner {
public:
    explicit HierarchicalFloorplanner(soc::db::ChipDB& db, const FloorplanConfig& config = {});

    // Compute floorplan layout for hierarchical block abstracts (P-Cores, E-Cores, GPU, NPU, SLC)
    bool floorplan_macros();

    // Assign boundary pins along edges of block abstracts to minimize bus wirelength and crossings
    bool assign_boundary_pins();

    // Order bus pins along block edges topologically to prevent wire crossings
    void order_bus_pins(soc::db::BlockAbstractId block_a,
                        soc::db::BlockAbstractId block_b,
                        const std::vector<soc::db::NetId>& bus_nets);

    [[nodiscard]] const FloorplanConfig& config() const noexcept { return m_config; }
    [[nodiscard]] const std::vector<BoundaryPinBudget>& pin_budgets() const noexcept { return m_budgets; }

private:
    soc::db::ChipDB& m_db;
    FloorplanConfig m_config;
    std::vector<BoundaryPinBudget> m_budgets;

    // Helper to determine best edge of block facing a target point
    [[nodiscard]] BlockEdge determine_best_edge(const soc::db::Rect2D& block_box,
                                                soc::db::Point2D target) const noexcept;

    // Helper to project a target point to a specific edge of a box
    [[nodiscard]] soc::db::Point2D project_to_edge(const soc::db::Rect2D& block_box,
                                                   BlockEdge edge,
                                                   soc::db::Point2D target,
                                                   int32_t slot_offset) const noexcept;
};

} // namespace vlsi::place
