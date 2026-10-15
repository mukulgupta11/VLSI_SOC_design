#pragma once

#include "router_types.hpp"
#include "core/db/chip_db.hpp"
#include <vector>
#include <unordered_map>
#include <memory>

namespace vlsi::route {

class GlobalRouter {
public:
    explicit GlobalRouter(soc::db::ChipDB& db, const GlobalRouterConfig& config = {});

    // Execute 3D GCell Global Maze Routing with Negotiated Congestion (PathFinder)
    RouteStats route();

    [[nodiscard]] const GlobalRouterConfig& config() const noexcept { return m_config; }
    [[nodiscard]] const std::vector<GlobalRoute>& routes() const noexcept { return m_routes; }

    [[nodiscard]] int32_t grid_dim_x() const noexcept { return m_nx; }
    [[nodiscard]] int32_t grid_dim_y() const noexcept { return m_ny; }
    [[nodiscard]] int32_t grid_dim_z() const noexcept { return m_nz; }

    [[nodiscard]] const GCell& get_gcell(int32_t x, int32_t y, int32_t z) const {
        return m_grid[grid_index(x, y, z)];
    }

private:
    soc::db::ChipDB& m_db;
    GlobalRouterConfig m_config;

    int32_t m_nx{0};
    int32_t m_ny{0};
    int32_t m_nz{0}; // Number of routing layers (up to 15)
    int32_t m_gcell_w{10000}; // in DBU
    int32_t m_gcell_h{10000}; // in DBU
    soc::db::Rect2D m_die_box;

    std::vector<GCell> m_grid;
    std::vector<GlobalRoute> m_routes;

    [[nodiscard]] size_t grid_index(int32_t x, int32_t y, int32_t z) const noexcept {
        return static_cast<size_t>((z * m_ny + y) * m_nx + x);
    }

    [[nodiscard]] GCellCoord point_to_gcell(soc::db::Point2D pt, int32_t z = 0) const noexcept;
    [[nodiscard]] soc::db::Point2D gcell_to_point(GCellCoord coord) const noexcept;

    void init_grid();
    void apply_obstructions();
    bool route_two_pin(GCellCoord start, GCellCoord target, double alpha,
                       std::vector<GCellSegment>& out_segments);
    bool route_net(soc::db::NetId net_id, double alpha, GlobalRoute& route);

    void add_route_demand(const GlobalRoute& route);
    void rip_up_route_demand(const GlobalRoute& route);
    void update_congestion_history(double beta);
    [[nodiscard]] std::pair<int32_t, int32_t> calculate_overflow() const noexcept;
};

} // namespace vlsi::route
