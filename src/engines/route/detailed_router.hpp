#pragma once

#include "router_types.hpp"
#include "core/db/chip_db.hpp"
#include <vector>

namespace vlsi::route {

class DetailedRouter {
public:
    explicit DetailedRouter(soc::db::ChipDB& db, const DetailedRouterConfig& config = {});

    // Detailed track and via assignment from 3D global routes
    RouteStats route(const std::vector<GlobalRoute>& global_routes);

    [[nodiscard]] const DetailedRouterConfig& config() const noexcept { return m_config; }

private:
    soc::db::ChipDB& m_db;
    DetailedRouterConfig m_config;

    void route_net_detailed(soc::db::NetId net_id, const GlobalRoute& groute, RouteStats& stats);
    void connect_pins_to_tracks(soc::db::NetId net_id, RouteStats& stats);
    void generate_bspdn_nano_tsvs(RouteStats& stats);
    [[nodiscard]] soc::db::Point2D gcell_coord_to_point(GCellCoord coord) const noexcept;
};

} // namespace vlsi::route
