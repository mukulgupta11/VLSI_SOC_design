#include "global_router.hpp"
#include <queue>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <iostream>

namespace vlsi::route {

GlobalRouter::GlobalRouter(soc::db::ChipDB& db, const GlobalRouterConfig& config)
    : m_db(db), m_config(config) {
    m_gcell_w = config.gcell_width_um * static_cast<int32_t>(m_db.dbu_per_micron);
    m_gcell_h = config.gcell_height_um * static_cast<int32_t>(m_db.dbu_per_micron);
    if (m_gcell_w <= 0) m_gcell_w = 10000;
    if (m_gcell_h <= 0) m_gcell_h = 10000;
}

GCellCoord GlobalRouter::point_to_gcell(soc::db::Point2D pt, int32_t z) const noexcept {
    int32_t gx = std::clamp((pt.x - m_die_box.x_min) / m_gcell_w, 0, m_nx - 1);
    int32_t gy = std::clamp((pt.y - m_die_box.y_min) / m_gcell_h, 0, m_ny - 1);
    int32_t gz = std::clamp(z, 0, m_nz - 1);
    return GCellCoord{gx, gy, gz};
}

soc::db::Point2D GlobalRouter::gcell_to_point(GCellCoord coord) const noexcept {
    int32_t px = m_die_box.x_min + coord.x * m_gcell_w + m_gcell_w / 2;
    int32_t py = m_die_box.y_min + coord.y * m_gcell_h + m_gcell_h / 2;
    return soc::db::Point2D{px, py};
}

void GlobalRouter::init_grid() {
    m_die_box = m_db.die_area;
    if (m_die_box.width() <= 0 || m_die_box.height() <= 0) {
        m_die_box = soc::db::Rect2D{0, 0, 8000000, 8000000};
    }

    m_nx = std::max(1, m_die_box.width() / m_gcell_w);
    m_ny = std::max(1, m_die_box.height() / m_gcell_h);
    m_nz = std::min(15, std::max(1, static_cast<int32_t>(m_db.layers.size())));

    size_t total_cells = static_cast<size_t>(m_nx * m_ny * m_nz);
    m_grid.clear();
    m_grid.resize(total_cells);

    for (int32_t z = 0; z < m_nz; ++z) {
        soc::db::LayerId lid{static_cast<uint32_t>(z)};
        int32_t pitch = (z < 3) ? 48 : (z < 8 ? 96 : 192); // DBU layer pitch
        int32_t tracks_h = std::max(1, m_gcell_h / pitch);
        int32_t tracks_v = std::max(1, m_gcell_w / pitch);

        for (int32_t y = 0; y < m_ny; ++y) {
            for (int32_t x = 0; x < m_nx; ++x) {
                size_t idx = grid_index(x, y, z);
                GCell& cell = m_grid[idx];
                cell.coord = GCellCoord{x, y, z};
                cell.layer_id = lid;
                cell.bbox = soc::db::Rect2D{
                    m_die_box.x_min + x * m_gcell_w,
                    m_die_box.y_min + y * m_gcell_h,
                    m_die_box.x_min + (x + 1) * m_gcell_w,
                    m_die_box.y_min + (y + 1) * m_gcell_h
                };

                // Even layers horizontal (z=2,4,...), odd layers vertical (z=1,3,...)
                bool is_horiz = (z % 2 == 0);
                cell.cap_h = is_horiz ? tracks_h : 0;
                cell.cap_v = (!is_horiz) ? tracks_v : 0;
                cell.cap_via = 16;

                cell.demand_h = 0;
                cell.demand_v = 0;
                cell.demand_via = 0;
                cell.history_cost_h = 1.0;
                cell.history_cost_v = 1.0;
                cell.history_cost_via = 1.0;
                cell.is_blocked = (z < 5); // Metal 1 to Metal 5 (layers 0..4) reserved for local pin escape & detailed routing
                if (z < 5) {
                    cell.cap_h = 0;
                    cell.cap_v = 0;
                }
            }
        }
    }
}

void GlobalRouter::apply_obstructions() {
    // 1. Block hierarchical macro abstracts on metal1 through metal8
    for (size_t b = 0; b < m_db.block_abstracts.size(); ++b) {
        soc::db::BlockAbstractId bid{static_cast<uint32_t>(b)};
        const auto& block = m_db.block_abstracts[bid];

        // Macro internal area blocked on lower metals (M1-M8), accessible on top metals (M9-M15)
        int32_t gx_min = std::clamp((block.boundary.x_min - m_die_box.x_min) / m_gcell_w, 0, m_nx - 1);
        int32_t gx_max = std::clamp((block.boundary.x_max - m_die_box.x_min) / m_gcell_w, 0, m_nx - 1);
        int32_t gy_min = std::clamp((block.boundary.y_min - m_die_box.y_min) / m_gcell_h, 0, m_ny - 1);
        int32_t gy_max = std::clamp((block.boundary.y_max - m_die_box.y_min) / m_gcell_h, 0, m_ny - 1);

        int32_t max_blocked_layer = std::min(m_nz - 1, 7); // M1 to M8
        for (int32_t z = 0; z <= max_blocked_layer; ++z) {
            for (int32_t y = gy_min; y <= gy_max; ++y) {
                for (int32_t x = gx_min; x <= gx_max; ++x) {
                    GCell& cell = m_grid[grid_index(x, y, z)];
                    cell.cap_h = std::max(0, cell.cap_h / 8); // only leave boundary pin escape capacity
                    cell.cap_v = std::max(0, cell.cap_v / 8);
                }
            }
        }
    }

    // 2. Backside Power Delivery Network (BSPDN) Nano-TSV Keepouts on M1..M3
    if (m_config.enable_bspdn) {
        [[maybe_unused]] int32_t ko_radius = m_config.bspdn_keepout_radius_um * static_cast<int32_t>(m_db.dbu_per_micron);
        for (const auto& tsv : m_db.nano_tsvs) {
            int32_t gx = (tsv.pos.x - m_die_box.x_min) / m_gcell_w;
            int32_t gy = (tsv.pos.y - m_die_box.y_min) / m_gcell_h;

            if (gx >= 0 && gx < m_nx && gy >= 0 && gy < m_ny) {
                // Reserve capacity on M1 and M2 for nano-TSV contact landings
                for (int32_t z = 0; z <= std::min(m_nz - 1, 1); ++z) {
                    GCell& cell = m_grid[grid_index(gx, gy, z)];
                    cell.cap_h = std::max(1, cell.cap_h - 2);
                    cell.cap_v = std::max(1, cell.cap_v - 2);
                }
            }
        }
    }
}

struct AStarNode {
    GCellCoord coord;
    double g_cost{0.0};
    double f_cost{0.0};

    bool operator>(const AStarNode& o) const noexcept {
        return f_cost > o.f_cost;
    }
};

bool GlobalRouter::route_two_pin(GCellCoord start, GCellCoord target, double alpha,
                                 std::vector<GCellSegment>& out_segments) {
    if (start == target) return true;

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> pq;
    std::unordered_map<GCellCoord, double, GCellCoordHash> g_scores;
    std::unordered_map<GCellCoord, GCellCoord, GCellCoordHash> parent_map;
    std::unordered_map<GCellCoord, RouteDirection, GCellCoordHash> dir_map;

    auto heuristic = [&](GCellCoord c) -> double {
        double dx = std::abs(c.x - target.x);
        double dy = std::abs(c.y - target.y);
        double dz = std::abs(c.z - target.z);
        return dx + dy + dz * m_config.via_cost;
    };

    g_scores[start] = 0.0;
    pq.push(AStarNode{start, 0.0, heuristic(start)});

    bool found = false;

    // Allowed 6 directions in 3D: +X, -X, +Y, -Y, +Z, -Z
    const int dx[] = {1, -1, 0, 0, 0, 0};
    const int dy[] = {0, 0, 1, -1, 0, 0};
    const int dz[] = {0, 0, 0, 0, 1, -1};
    const RouteDirection dirs[] = {
        RouteDirection::HORIZONTAL, RouteDirection::HORIZONTAL,
        RouteDirection::VERTICAL, RouteDirection::VERTICAL,
        RouteDirection::VIA_UP, RouteDirection::VIA_DOWN
    };

    while (!pq.empty()) {
        AStarNode curr = pq.top();
        pq.pop();

        if (curr.coord == target) {
            found = true;
            break;
        }

        if (curr.g_cost > g_scores[curr.coord] + 1e-4) {
            continue;
        }

        for (int i = 0; i < 6; ++i) {
            int nx = curr.coord.x + dx[i];
            int ny = curr.coord.y + dy[i];
            int nz = curr.coord.z + dz[i];

            if (nx < 0 || nx >= m_nx || ny < 0 || ny >= m_ny || nz < 0 || nz >= m_nz) {
                continue;
            }

            GCellCoord next_c{nx, ny, nz};
            const GCell& target_cell = m_grid[grid_index(nx, ny, nz)];

            if (target_cell.is_blocked) continue;

            // Compute edge traversal cost with PathFinder negotiated congestion
            double base_cost = 1.0;
            double present_penalty = 1.0;
            double hist_cost = 1.0;

            if (dirs[i] == RouteDirection::HORIZONTAL) {
                hist_cost = target_cell.history_cost_h;
                double ratio = static_cast<double>(target_cell.demand_h) / std::max(1, target_cell.cap_h);
                present_penalty = 1.0 + alpha * ratio;
            } else if (dirs[i] == RouteDirection::VERTICAL) {
                hist_cost = target_cell.history_cost_v;
                double ratio = static_cast<double>(target_cell.demand_v) / std::max(1, target_cell.cap_v);
                present_penalty = 1.0 + alpha * ratio;
            } else {
                // Via layer change
                base_cost = m_config.via_cost;
                hist_cost = target_cell.history_cost_via;
                double ratio = static_cast<double>(target_cell.demand_via) / std::max(1, target_cell.cap_via);
                present_penalty = 1.0 + alpha * ratio;
            }

            // Enforce strict orthogonal routing directions per metal layer
            bool layer_is_horiz = (nz % 2 == 0);
            if ((dirs[i] == RouteDirection::HORIZONTAL && !layer_is_horiz) ||
                (dirs[i] == RouteDirection::VERTICAL && layer_is_horiz)) {
                continue;
            }

            double edge_cost = (base_cost * hist_cost) * present_penalty;
            double tent_g = curr.g_cost + edge_cost;

            auto it = g_scores.find(next_c);
            if (it == g_scores.end() || tent_g < it->second) {
                g_scores[next_c] = tent_g;
                parent_map[next_c] = curr.coord;
                dir_map[next_c] = dirs[i];
                double f = tent_g + heuristic(next_c);
                pq.push(AStarNode{next_c, tent_g, f});
            }
        }
    }

    if (!found) return false;

    // Backtrack segments
    GCellCoord curr = target;
    while (!(curr == start)) {
        GCellCoord parent = parent_map[curr];
        RouteDirection d = dir_map[curr];
        out_segments.push_back(GCellSegment{parent, curr, d});
        curr = parent;
    }

    std::reverse(out_segments.begin(), out_segments.end());
    return true;
}

bool GlobalRouter::route_net(soc::db::NetId net_id, double alpha, GlobalRoute& route) {
    const auto& net = m_db.nets[net_id];
    if (net.pins.size() < 2) return true;

    route.net_id = net_id;
    route.segments.clear();
    route.wirelength_um = 0.0;
    route.via_count = 0;

    // Gather pin GCells
    std::vector<GCellCoord> pin_coords;
    for (auto pid : net.pins) {
        const auto& pin = m_db.pins[pid];
        soc::db::Point2D pt = pin.offset_box.center();
        if (pin.instance_id.is_valid()) {
            pt = m_db.instances[pin.instance_id].origin + pt;
        }
        int32_t z = 5; // Connect to Metal 6 (z=5, odd=vertical) routing grid
        pin_coords.push_back(point_to_gcell(pt, z));
    }

    // Incremental Minimum Spanning Tree (MST) routing
    std::vector<GCellCoord> routed_tree;
    routed_tree.push_back(pin_coords.front());

    double dbu_to_um = 1.0 / std::max(1.0, static_cast<double>(m_db.dbu_per_micron));

    for (size_t p = 1; p < pin_coords.size(); ++p) {
        GCellCoord target = pin_coords[p];

        // Find closest point in current routed tree
        GCellCoord best_source = routed_tree.front();
        int32_t best_dist = 1e9;

        for (const auto& node : routed_tree) {
            int32_t dist = std::abs(node.x - target.x) + std::abs(node.y - target.y) + std::abs(node.z - target.z);
            if (dist < best_dist) {
                best_dist = dist;
                best_source = node;
            }
        }

        std::vector<GCellSegment> segs;
        bool ok = route_two_pin(best_source, target, alpha, segs);
        if (!ok) return false;

        for (const auto& s : segs) {
            route.segments.push_back(s);
            routed_tree.push_back(s.to);

            if (s.direction == RouteDirection::HORIZONTAL) {
                route.wirelength_um += m_gcell_w * dbu_to_um;
            } else if (s.direction == RouteDirection::VERTICAL) {
                route.wirelength_um += m_gcell_h * dbu_to_um;
            } else {
                route.via_count++;
            }
        }
    }

    route.is_routed = true;
    return true;
}

void GlobalRouter::add_route_demand(const GlobalRoute& route) {
    for (const auto& seg : route.segments) {
        GCell& c1 = m_grid[grid_index(seg.from.x, seg.from.y, seg.from.z)];
        GCell& c2 = m_grid[grid_index(seg.to.x, seg.to.y, seg.to.z)];

        if (seg.direction == RouteDirection::HORIZONTAL) {
            c1.demand_h++;
            c2.demand_h++;
        } else if (seg.direction == RouteDirection::VERTICAL) {
            c1.demand_v++;
            c2.demand_v++;
        } else {
            c1.demand_via++;
            c2.demand_via++;
        }
    }
}

void GlobalRouter::rip_up_route_demand(const GlobalRoute& route) {
    for (const auto& seg : route.segments) {
        GCell& c1 = m_grid[grid_index(seg.from.x, seg.from.y, seg.from.z)];
        GCell& c2 = m_grid[grid_index(seg.to.x, seg.to.y, seg.to.z)];

        if (seg.direction == RouteDirection::HORIZONTAL) {
            c1.demand_h = std::max(0, c1.demand_h - 1);
            c2.demand_h = std::max(0, c2.demand_h - 1);
        } else if (seg.direction == RouteDirection::VERTICAL) {
            c1.demand_v = std::max(0, c1.demand_v - 1);
            c2.demand_v = std::max(0, c2.demand_v - 1);
        } else {
            c1.demand_via = std::max(0, c1.demand_via - 1);
            c2.demand_via = std::max(0, c2.demand_via - 1);
        }
    }
}

void GlobalRouter::update_congestion_history(double beta) {
    for (auto& cell : m_grid) {
        int32_t ov_h = std::max(0, cell.demand_h - cell.cap_h);
        int32_t ov_v = std::max(0, cell.demand_v - cell.cap_v);
        int32_t ov_via = std::max(0, cell.demand_via - cell.cap_via);

        cell.history_cost_h += beta * ov_h;
        cell.history_cost_v += beta * ov_v;
        cell.history_cost_via += beta * ov_via;
    }
}

std::pair<int32_t, int32_t> GlobalRouter::calculate_overflow() const noexcept {
    int32_t max_ov = 0;
    int32_t total_ov = 0;

    for (const auto& cell : m_grid) {
        int32_t ov_h = std::max(0, cell.demand_h - cell.cap_h);
        int32_t ov_v = std::max(0, cell.demand_v - cell.cap_v);
        int32_t ov_via = std::max(0, cell.demand_via - cell.cap_via);

        int32_t local_total = ov_h + ov_v + ov_via;
        int32_t local_max = std::max({ov_h, ov_v, ov_via});

        total_ov += local_total;
        max_ov = std::max(max_ov, local_max);
    }
    return {max_ov, total_ov};
}

RouteStats GlobalRouter::route() {
    auto start_time = std::chrono::high_resolution_clock::now();
    RouteStats stats;

    init_grid();
    apply_obstructions();

    m_routes.clear();
    m_routes.resize(m_db.nets.size());

    // 1. Initial Routing Iteration (alpha = 0.5)
    double alpha = m_config.alpha_congestion;
    double beta = m_config.beta_history;

    for (size_t n = 0; n < m_db.nets.size(); ++n) {
        soc::db::NetId net_id{static_cast<uint32_t>(n)};
        if (m_db.nets[net_id].pins.size() < 2) continue;

        stats.total_nets++;
        bool ok = route_net(net_id, alpha, m_routes[n]);
        if (ok) {
            add_route_demand(m_routes[n]);
            stats.routed_nets++;
        } else {
            stats.failed_nets++;
        }
    }

    auto [max_ov, total_ov] = calculate_overflow();
    stats.max_overflow = max_ov;
    stats.total_overflow = total_ov;
    stats.ripup_iterations = 1;

    // 2. Negotiated Congestion Rip-up and Reroute Loop
    for (uint32_t iter = 2; iter <= m_config.max_iterations && stats.total_overflow > 0; ++iter) {
        update_congestion_history(beta);
        alpha *= 1.4; // Exponential penalty increase
        stats.ripup_iterations = iter;

        // Rip-up nets passing through congested GCells
        for (size_t n = 0; n < m_db.nets.size(); ++n) {
            if (!m_routes[n].is_routed) continue;

            bool crosses_congested = false;
            for (const auto& seg : m_routes[n].segments) {
                const auto& c = m_grid[grid_index(seg.to.x, seg.to.y, seg.to.z)];
                if (c.demand_h > c.cap_h || c.demand_v > c.cap_v || c.demand_via > c.cap_via) {
                    crosses_congested = true;
                    break;
                }
            }

            if (crosses_congested) {
                rip_up_route_demand(m_routes[n]);
                m_routes[n].is_routed = false;
                stats.routed_nets--;

                // Reroute with updated penalties
                soc::db::NetId net_id{static_cast<uint32_t>(n)};
                bool ok = route_net(net_id, alpha, m_routes[n]);
                if (ok) {
                    add_route_demand(m_routes[n]);
                    stats.routed_nets++;
                } else {
                    stats.failed_nets++;
                }
            }
        }

        auto [new_max, new_total] = calculate_overflow();
        stats.max_overflow = new_max;
        stats.total_overflow = new_total;
    }

    // Accumulate total wirelength and vias
    for (const auto& r : m_routes) {
        if (r.is_routed) {
            stats.total_wirelength_um += r.wirelength_um;
            stats.total_vias += r.via_count;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    stats.runtime_seconds = std::chrono::duration<double>(end_time - start_time).count();
    return stats;
}

} // namespace vlsi::route
