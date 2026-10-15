#pragma once

#include "core/db/chip_db.hpp"
#include <vector>
#include <cstdint>
#include <limits>
#include <string>

namespace vlsi::route {

// 3D GCell Grid Node
struct GCellCoord {
    int32_t x{0};
    int32_t y{0};
    int32_t z{0}; // Layer index: 0 = metal1, 14 = metal15

    constexpr bool operator==(const GCellCoord& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct GCellCoordHash {
    size_t operator()(const GCellCoord& c) const noexcept {
        size_t h1 = std::hash<int32_t>{}(c.x);
        size_t h2 = std::hash<int32_t>{}(c.y);
        size_t h3 = std::hash<int32_t>{}(c.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// Routing direction for track allocation
enum class RouteDirection : uint8_t {
    HORIZONTAL = 0,
    VERTICAL,
    VIA_UP,
    VIA_DOWN
};

// GCell data holding capacity, demand, and congestion penalties
struct GCell {
    GCellCoord coord;
    soc::db::Rect2D bbox;
    soc::db::LayerId layer_id{soc::db::LayerId::invalid()};

    // Capacities (number of routing tracks available across boundaries)
    int32_t cap_h{0};
    int32_t cap_v{0};
    int32_t cap_via{16}; // Max via cuts between adjacent layers

    // Current demand (number of nets crossing boundary)
    int32_t demand_h{0};
    int32_t demand_v{0};
    int32_t demand_via{0};

    // Historical congestion penalty (PathFinder negotiated congestion)
    double history_cost_h{1.0};
    double history_cost_v{1.0};
    double history_cost_via{1.0};

    // Blockage flag (e.g. macro obstruction, BSPDN nano-TSV keepout)
    bool is_blocked{false};
};

// 3D Routing segment connecting two GCells
struct GCellSegment {
    GCellCoord from;
    GCellCoord to;
    RouteDirection direction{RouteDirection::HORIZONTAL};
};

// Global route path for a net
struct GlobalRoute {
    soc::db::NetId net_id{soc::db::NetId::invalid()};
    std::vector<GCellSegment> segments;
    double wirelength_um{0.0};
    uint32_t via_count{0};
    bool is_routed{false};
};

// Global Router Configuration
struct GlobalRouterConfig {
    int32_t gcell_width_um{10};    // 10 um GCell width
    int32_t gcell_height_um{10};   // 10 um GCell height
    uint32_t max_iterations{20};   // Max rip-up and reroute iterations
    double alpha_congestion{1.5};  // Present congestion penalty weight
    double beta_history{0.5};      // Historic congestion accumulation rate
    double via_cost{4.0};          // Cost penalty for changing layers (via)
    bool enable_bspdn{true};       // Backside Power Delivery Network support
    int32_t bspdn_keepout_radius_um{1}; // Keepout around BSPDN nano-TSVs on M1-M3
};

// Routing Quality Metrics
struct RouteStats {
    uint32_t total_nets{0};
    uint32_t routed_nets{0};
    uint32_t failed_nets{0};
    double total_wirelength_um{0.0};
    uint32_t total_vias{0};
    int32_t max_overflow{0};
    int32_t total_overflow{0};
    uint32_t ripup_iterations{0};
    double runtime_seconds{0.0};
};

// Detailed Router Configuration
struct DetailedRouterConfig {
    int32_t tile_size_um{100};     // 100 um x 100 um parallel routing tiles
    int32_t gcell_width_um{50};    // 50 um GCell width
    int32_t gcell_height_um{50};   // 50 um GCell height
    bool enforce_stacked_vias{true};
    bool generate_bspdn_rails{true};
};

} // namespace vlsi::route
