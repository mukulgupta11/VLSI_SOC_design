#include "floorplanner.hpp"
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <map>

namespace vlsi::place {

HierarchicalFloorplanner::HierarchicalFloorplanner(soc::db::ChipDB& db, const FloorplanConfig& config)
    : m_db(db), m_config(config) {}

BlockEdge HierarchicalFloorplanner::determine_best_edge(const soc::db::Rect2D& block_box,
                                                        soc::db::Point2D target) const noexcept {
    soc::db::Point2D center = block_box.center();
    int32_t dx = target.x - center.x;
    int32_t dy = target.y - center.y;

    if (std::abs(dx) >= std::abs(dy)) {
        return (dx >= 0) ? BlockEdge::EAST : BlockEdge::WEST;
    } else {
        return (dy >= 0) ? BlockEdge::NORTH : BlockEdge::SOUTH;
    }
}

soc::db::Point2D HierarchicalFloorplanner::project_to_edge(const soc::db::Rect2D& block_box,
                                                           BlockEdge edge,
                                                           soc::db::Point2D target,
                                                           int32_t slot_offset) const noexcept {
    int32_t x = 0;
    int32_t y = 0;
    int32_t halo = m_config.macro_halo;

    switch (edge) {
        case BlockEdge::NORTH:
            x = std::clamp(target.x + slot_offset, block_box.x_min + halo, block_box.x_max - halo);
            // Snap to pin pitch
            x = (x / m_config.pin_pitch) * m_config.pin_pitch;
            y = block_box.y_max;
            break;
        case BlockEdge::SOUTH:
            x = std::clamp(target.x + slot_offset, block_box.x_min + halo, block_box.x_max - halo);
            x = (x / m_config.pin_pitch) * m_config.pin_pitch;
            y = block_box.y_min;
            break;
        case BlockEdge::EAST:
            x = block_box.x_max;
            y = std::clamp(target.y + slot_offset, block_box.y_min + halo, block_box.y_max - halo);
            y = (y / m_config.pin_pitch) * m_config.pin_pitch;
            break;
        case BlockEdge::WEST:
            x = block_box.x_min;
            y = std::clamp(target.y + slot_offset, block_box.y_min + halo, block_box.y_max - halo);
            y = (y / m_config.pin_pitch) * m_config.pin_pitch;
            break;
    }
    return {x, y};
}

bool HierarchicalFloorplanner::floorplan_macros() {
    if (m_db.block_abstracts.empty()) {
        return true;
    }

    // Set chip die area if not configured
    if (m_db.die_area.width() == 0 || m_db.die_area.height() == 0) {
        m_db.die_area = soc::db::Rect2D{0, 0, m_config.die_width, m_config.die_height};
    }

    // Group abstracts by macro_type
    std::vector<soc::db::BlockAbstractId> slc_blocks;
    std::vector<soc::db::BlockAbstractId> pcore_blocks;
    std::vector<soc::db::BlockAbstractId> ecore_blocks;
    std::vector<soc::db::BlockAbstractId> gpu_blocks;
    std::vector<soc::db::BlockAbstractId> npu_blocks;
    std::vector<soc::db::BlockAbstractId> other_blocks;

    for (size_t i = 0; i < m_db.block_abstracts.size(); ++i) {
        soc::db::BlockAbstractId id{static_cast<uint32_t>(i)};
        const auto& block = m_db.block_abstracts[id];
        if (block.macro_type == "SLC") {
            slc_blocks.push_back(id);
        } else if (block.macro_type == "P_CORE") {
            pcore_blocks.push_back(id);
        } else if (block.macro_type == "E_CORE") {
            ecore_blocks.push_back(id);
        } else if (block.macro_type == "GPU_CORE" || block.macro_type == "GPU_SLICE") {
            gpu_blocks.push_back(id);
        } else if (block.macro_type == "NPU") {
            npu_blocks.push_back(id);
        } else {
            other_blocks.push_back(id);
        }
    }

    // Floorplan layout geometry parameters
    int32_t core_min_x = m_config.core_margin_x;
    int32_t core_min_y = m_config.core_margin_y;
    int32_t core_max_x = m_db.die_area.x_max - m_config.core_margin_x;
    int32_t core_max_y = m_db.die_area.y_max - m_config.core_margin_y;
    int32_t center_x = (core_min_x + core_max_x) / 2;
    int32_t center_y = (core_min_y + core_max_y) / 2;

    // 1. Place SLC in the exact center (System Level Cache hub)
    for (auto id : slc_blocks) {
        auto& block = m_db.block_abstracts[id];
        int32_t w = block.boundary.width();
        int32_t h = block.boundary.height();
        if (w == 0) w = 3000000; // 3 mm default
        if (h == 0) h = 800000;  // 800 um default
        block.boundary = soc::db::Rect2D{center_x - w / 2, center_y - h / 2,
                                         center_x + w / 2, center_y + h / 2};
    }

    // 2. Place P-Cores on West / Northwest (High performance compute cluster)
    int32_t p_cur_x = core_min_x;
    int32_t p_cur_y = center_y + 600000;
    for (auto id : pcore_blocks) {
        auto& block = m_db.block_abstracts[id];
        int32_t w = block.boundary.width();
        int32_t h = block.boundary.height();
        if (w == 0) w = 1800000;
        if (h == 0) h = 1800000;
        block.boundary = soc::db::Rect2D{p_cur_x, p_cur_y, p_cur_x + w, p_cur_y + h};
        p_cur_x += w + m_config.macro_halo;
        if (p_cur_x + w > center_x) {
            p_cur_x = core_min_x;
            p_cur_y += h + m_config.macro_halo;
        }
    }

    // 3. Place E-Cores on North / Northeast (Energy efficient compute cluster)
    int32_t e_cur_x = center_x + 200000;
    int32_t e_cur_y = center_y + 600000;
    for (auto id : ecore_blocks) {
        auto& block = m_db.block_abstracts[id];
        int32_t w = block.boundary.width();
        int32_t h = block.boundary.height();
        if (w == 0) w = 900000;
        if (h == 0) h = 900000;
        block.boundary = soc::db::Rect2D{e_cur_x, e_cur_y, e_cur_x + w, e_cur_y + h};
        e_cur_x += w + m_config.macro_halo;
        if (e_cur_x + w > core_max_x) {
            e_cur_x = center_x + 200000;
            e_cur_y += h + m_config.macro_halo;
        }
    }

    // 4. Place GPU Cores / Slices across Southern quadrant
    int32_t g_cur_x = core_min_x;
    int32_t g_cur_y = core_min_y;
    for (auto id : gpu_blocks) {
        auto& block = m_db.block_abstracts[id];
        int32_t w = block.boundary.width();
        int32_t h = block.boundary.height();
        if (w == 0) w = 1100000;
        if (h == 0) h = 1100000;
        block.boundary = soc::db::Rect2D{g_cur_x, g_cur_y, g_cur_x + w, g_cur_y + h};
        g_cur_x += w + m_config.macro_halo;
        if (g_cur_x + w > core_max_x) {
            g_cur_x = core_min_x;
            g_cur_y += h + m_config.macro_halo;
        }
    }

    // 5. Place NPU in Southwest above GPU
    int32_t npu_cur_x = core_min_x;
    int32_t npu_cur_y = g_cur_y + 1200000;
    for (auto id : npu_blocks) {
        auto& block = m_db.block_abstracts[id];
        int32_t w = block.boundary.width();
        int32_t h = block.boundary.height();
        if (w == 0) w = 1500000;
        if (h == 0) h = 1200000;
        block.boundary = soc::db::Rect2D{npu_cur_x, npu_cur_y, npu_cur_x + w, npu_cur_y + h};
        npu_cur_x += w + m_config.macro_halo;
    }

    // Rebuild spatial index and notify observers
    m_db.rebuild_instance_spatial_index();
    m_db.observers.notify_dirty_region(m_db.die_area, soc::db::LayerId{0});
    return true;
}

bool HierarchicalFloorplanner::assign_boundary_pins() {
    m_budgets.clear();

    // Map each block abstract's 4 edges to slot occupancy counters
    struct EdgeAllocator {
        int32_t north_slots{0};
        int32_t south_slots{0};
        int32_t east_slots{0};
        int32_t west_slots{0};
    };
    std::unordered_map<uint32_t, EdgeAllocator> allocators;

    // Scan all nets connecting between block abstracts or outside
    for (size_t net_idx = 0; net_idx < m_db.nets.size(); ++net_idx) {
        soc::db::NetId net_id{static_cast<uint32_t>(net_idx)};
        auto& net = m_db.nets[net_id];
        if (net.pins.size() < 2) continue;

        // Collect all blocks associated with pins in this net
        std::vector<std::pair<soc::db::BlockAbstractId, soc::db::PinId>> block_pins;
        for (auto pin_id : net.pins) {
            const auto& pin = m_db.pins[pin_id];
            if (pin.instance_id.is_valid()) {
                const auto& inst = m_db.instances[pin.instance_id];
                if (inst.parent_abstract_id.is_valid()) {
                    block_pins.emplace_back(inst.parent_abstract_id, pin_id);
                }
            }
        }

        // If net spans across 2 different block abstracts, assign boundary pins on both
        if (block_pins.size() >= 2 && block_pins.front().first != block_pins.back().first) {
            auto block_a_id = block_pins.front().first;
            auto block_b_id = block_pins.back().first;
            auto& block_a = m_db.block_abstracts[block_a_id];
            auto& block_b = m_db.block_abstracts[block_b_id];

            // Edge from A facing B
            BlockEdge edge_a = determine_best_edge(block_a.boundary, block_b.boundary.center());
            // Edge from B facing A
            BlockEdge edge_b = determine_best_edge(block_b.boundary, block_a.boundary.center());

            auto& alloc_a = allocators[block_a_id.value()];
            auto& alloc_b = allocators[block_b_id.value()];

            int32_t slot_offset_a = 0;
            switch (edge_a) {
                case BlockEdge::NORTH: slot_offset_a = (alloc_a.north_slots++) * m_config.pin_pitch; break;
                case BlockEdge::SOUTH: slot_offset_a = (alloc_a.south_slots++) * m_config.pin_pitch; break;
                case BlockEdge::EAST:  slot_offset_a = (alloc_a.east_slots++)  * m_config.pin_pitch; break;
                case BlockEdge::WEST:  slot_offset_a = (alloc_a.west_slots++)  * m_config.pin_pitch; break;
            }

            int32_t slot_offset_b = 0;
            switch (edge_b) {
                case BlockEdge::NORTH: slot_offset_b = (alloc_b.north_slots++) * m_config.pin_pitch; break;
                case BlockEdge::SOUTH: slot_offset_b = (alloc_b.south_slots++) * m_config.pin_pitch; break;
                case BlockEdge::EAST:  slot_offset_b = (alloc_b.east_slots++)  * m_config.pin_pitch; break;
                case BlockEdge::WEST:  slot_offset_b = (alloc_b.west_slots++)  * m_config.pin_pitch; break;
            }

            soc::db::Point2D pin_pos_a = project_to_edge(block_a.boundary, edge_a,
                                                         block_b.boundary.center(), slot_offset_a);
            soc::db::Point2D pin_pos_b = project_to_edge(block_b.boundary, edge_b,
                                                         block_a.boundary.center(), slot_offset_b);

            // Layer selection: horizontal edges use pin_layer_v, vertical edges use pin_layer_h
            soc::db::LayerId layer_a = (edge_a == BlockEdge::NORTH || edge_a == BlockEdge::SOUTH)
                                           ? m_config.pin_layer_v : m_config.pin_layer_h;
            soc::db::LayerId layer_b = (edge_b == BlockEdge::NORTH || edge_b == BlockEdge::SOUTH)
                                           ? m_config.pin_layer_v : m_config.pin_layer_h;

            // Pin dimensions (e.g. 50 nm x 50 nm pin contact box)
            soc::db::Rect2D pin_box{-25, -25, 25, 25};

            std::string pin_a_name = "BP_" + std::string(m_db.strings.resolve(net.name_id)) + "_A";
            std::string pin_b_name = "BP_" + std::string(m_db.strings.resolve(net.name_id)) + "_B";

            soc::db::PinId bp_a = m_db.create_pin(pin_a_name, soc::db::InstanceId::invalid(),
                                                  net_id, layer_a,
                                                  pin_box.translated(pin_pos_a));
            soc::db::PinId bp_b = m_db.create_pin(pin_b_name, soc::db::InstanceId::invalid(),
                                                  net_id, layer_b,
                                                  pin_box.translated(pin_pos_b));

            block_a.boundary_pins.push_back(bp_a);
            block_b.boundary_pins.push_back(bp_b);

            // Compute Euclidean timing budget (assume 10 ps per 100 um interconnect flight time)
            double dist_um = std::hypot(pin_pos_a.x - pin_pos_b.x, pin_pos_a.y - pin_pos_b.y) / 1000.0;
            double budget_ps = 50.0 + (dist_um * 0.1);

            m_budgets.push_back(BoundaryPinBudget{bp_a, block_a_id, net_id, edge_a, pin_pos_a, budget_ps});
            m_budgets.push_back(BoundaryPinBudget{bp_b, block_b_id, net_id, edge_b, pin_pos_b, budget_ps});
        }
    }

    return true;
}

void HierarchicalFloorplanner::order_bus_pins(soc::db::BlockAbstractId block_a_id,
                                              soc::db::BlockAbstractId block_b_id,
                                              const std::vector<soc::db::NetId>& bus_nets) {
    if (bus_nets.empty()) return;
    auto& block_a = m_db.block_abstracts[block_a_id];
    auto& block_b = m_db.block_abstracts[block_b_id];

    BlockEdge edge_a = determine_best_edge(block_a.boundary, block_b.boundary.center());
    BlockEdge edge_b = determine_best_edge(block_b.boundary, block_a.boundary.center());

    // Bus lines ordered monotonically along the edges to prevent wire crossings
    int32_t count = static_cast<int32_t>(bus_nets.size());
    int32_t start_offset = -(count / 2) * m_config.pin_pitch;

    for (int32_t i = 0; i < count; ++i) {
        int32_t offset = start_offset + i * m_config.pin_pitch;
        soc::db::Point2D pos_a = project_to_edge(block_a.boundary, edge_a,
                                                block_b.boundary.center(), offset);
        soc::db::Point2D pos_b = project_to_edge(block_b.boundary, edge_b,
                                                block_a.boundary.center(), offset);

        soc::db::LayerId layer_a = (edge_a == BlockEdge::NORTH || edge_a == BlockEdge::SOUTH)
                                       ? m_config.pin_layer_v : m_config.pin_layer_h;
        soc::db::LayerId layer_b = (edge_b == BlockEdge::NORTH || edge_b == BlockEdge::SOUTH)
                                       ? m_config.pin_layer_v : m_config.pin_layer_h;

        soc::db::Rect2D pin_box{-25, -25, 25, 25};
        std::string name_a = "BUS_" + std::to_string(i) + "_A";
        std::string name_b = "BUS_" + std::to_string(i) + "_B";

        soc::db::PinId bp_a = m_db.create_pin(name_a, soc::db::InstanceId::invalid(),
                                              bus_nets[i], layer_a, pin_box.translated(pos_a));
        soc::db::PinId bp_b = m_db.create_pin(name_b, soc::db::InstanceId::invalid(),
                                              bus_nets[i], layer_b, pin_box.translated(pos_b));

        block_a.boundary_pins.push_back(bp_a);
        block_b.boundary_pins.push_back(bp_b);
    }
}

} // namespace vlsi::place
