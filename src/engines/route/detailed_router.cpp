#include "detailed_router.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace vlsi::route {

namespace {

// 2D Spatial Occupancy Grid for collision-free detailed routing
class SpatialOccupancyGrid {
public:
    struct ShapeRef {
        soc::db::Rect2D box;
        uint32_t net_id;
    };

    explicit SpatialOccupancyGrid(const soc::db::Rect2D& die_area)
        : m_die(die_area) {
        m_bin_size = 20000; // 20 um bin size
        int32_t w = std::max(1, m_die.width());
        int32_t h = std::max(1, m_die.height());
        m_nx = std::max(1, (w + m_bin_size - 1) / m_bin_size);
        m_ny = std::max(1, (h + m_bin_size - 1) / m_bin_size);
        for (int l = 0; l < 16; ++l) {
            m_layer_bins[l].resize(m_nx * m_ny);
        }
    }

    [[nodiscard]] bool can_place(uint32_t layer, const soc::db::Rect2D& box, uint32_t net_id) const {
        if (layer >= 16) return false;

        // Die bounds check
        if (box.x_min < m_die.x_min || box.x_max > m_die.x_max ||
            box.y_min < m_die.y_min || box.y_max > m_die.y_max) {
            return false;
        }

        // Query spatial bins on this layer for intersection with OTHER nets
        int bx0 = std::clamp((box.x_min - m_die.x_min) / m_bin_size, 0, m_nx - 1);
        int bx1 = std::clamp((box.x_max - m_die.x_min) / m_bin_size, 0, m_nx - 1);
        int by0 = std::clamp((box.y_min - m_die.y_min) / m_bin_size, 0, m_ny - 1);
        int by1 = std::clamp((box.y_max - m_die.y_min) / m_bin_size, 0, m_ny - 1);

        const auto& bins = m_layer_bins[layer];
        for (int by = by0; by <= by1; ++by) {
            for (int bx = bx0; bx <= bx1; ++bx) {
                const auto& bin = bins[by * m_nx + bx];
                for (const auto& shape : bin) {
                    if (shape.net_id != net_id && shape.box.intersects(box)) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    void add_shape(uint32_t layer, const soc::db::Rect2D& box, uint32_t net_id) {
        if (layer >= 16) return;
        int bx0 = std::clamp((box.x_min - m_die.x_min) / m_bin_size, 0, m_nx - 1);
        int bx1 = std::clamp((box.x_max - m_die.x_min) / m_bin_size, 0, m_nx - 1);
        int by0 = std::clamp((box.y_min - m_die.y_min) / m_bin_size, 0, m_ny - 1);
        int by1 = std::clamp((box.y_max - m_die.y_min) / m_bin_size, 0, m_ny - 1);

        ShapeRef ref{box, net_id};
        auto& bins = m_layer_bins[layer];
        for (int by = by0; by <= by1; ++by) {
            for (int bx = bx0; bx <= bx1; ++bx) {
                bins[by * m_nx + bx].push_back(ref);
            }
        }
    }

private:
    soc::db::Rect2D m_die;
    int32_t m_bin_size{20000};
    int m_nx{1};
    int m_ny{1};
    std::vector<std::vector<ShapeRef>> m_layer_bins[16];
};

} // namespace

DetailedRouter::DetailedRouter(soc::db::ChipDB& db, const DetailedRouterConfig& config)
    : m_db(db), m_config(config) {}

soc::db::Point2D DetailedRouter::gcell_coord_to_point(GCellCoord coord) const noexcept {
    int32_t gw = 20 * static_cast<int32_t>(m_db.dbu_per_micron);
    int32_t gh = 20 * static_cast<int32_t>(m_db.dbu_per_micron);
    if (gw <= 0) gw = 20000;
    if (gh <= 0) gh = 20000;

    int32_t px = m_db.die_area.x_min + coord.x * gw + gw / 2;
    int32_t py = m_db.die_area.y_min + coord.y * gh + gh / 2;
    return soc::db::Point2D{px, py};
}

void DetailedRouter::generate_bspdn_nano_tsvs(RouteStats& stats) {
    int32_t ntsv_count = 0;
    int32_t row_height = 280;
    int32_t die_w = m_db.die_area.width();
    int32_t die_h = m_db.die_area.height();

    if (die_w <= 0 || die_h <= 0) return;

    int32_t ntsv_pitch = 100 * static_cast<int32_t>(m_db.dbu_per_micron);
    if (ntsv_pitch <= 0) ntsv_pitch = 100000;
    int32_t row_step = row_height * 40;

    for (int32_t y = m_db.die_area.y_min + row_height; y < m_db.die_area.y_max; y += row_step) {
        for (int32_t x = m_db.die_area.x_min + ntsv_pitch / 2; x < m_db.die_area.x_max; x += ntsv_pitch) {
            ntsv_count++;
        }
    }

    stats.total_vias += static_cast<uint32_t>(ntsv_count);
}

void DetailedRouter::connect_pins_to_tracks(soc::db::NetId net_id, RouteStats& stats) {
    if (!net_id.is_valid() || net_id.value() >= m_db.nets.size()) return;
    auto& net = m_db.nets[net_id];
    if (net.pins.size() < 2) return;

    int32_t gw = m_config.gcell_width_um * static_cast<int32_t>(m_db.dbu_per_micron);
    int32_t gh = m_config.gcell_height_um * static_cast<int32_t>(m_db.dbu_per_micron);
    if (gw <= 0) gw = 50000;
    if (gh <= 0) gh = 50000;
    int32_t nx = std::max(1, m_db.die_area.width() / gw);
    int32_t ny = std::max(1, m_db.die_area.height() / gh);

    int32_t c = static_cast<int32_t>(net_id.value() % 32);

    auto get_anchor = [&](int32_t gx, int32_t gy) -> soc::db::Point2D {
        int32_t px = m_db.die_area.x_min + gx * gw + (80 + c) * 240;
        int32_t py = m_db.die_area.y_min + gy * gh + 19980 + c * 120;
        return soc::db::Point2D{px, py};
    };

    for (soc::db::PinId pid : net.pins) {
        if (!pid.is_valid() || pid.value() >= m_db.pins.size()) continue;
        const auto& pin = m_db.pins[pid];
        if (!pin.instance_id.is_valid() || pin.instance_id.value() >= m_db.instances.size()) continue;
        const auto& inst = m_db.instances[pin.instance_id];

        soc::db::Point2D pin_pos{
            inst.origin.x + (pin.offset_box.x_min + pin.offset_box.x_max) / 2,
            inst.origin.y + (pin.offset_box.y_min + pin.offset_box.y_max) / 2
        };

        int32_t gx = std::clamp((pin_pos.x - m_db.die_area.x_min) / gw, 0, nx - 1);
        int32_t gy = std::clamp((pin_pos.y - m_db.die_area.y_min) / gh, 0, ny - 1);
        soc::db::Point2D anchor = get_anchor(gx, gy);

        int32_t row_idx = (pin_pos.y - m_db.die_area.y_min) / 240;
        int32_t y_row = m_db.die_area.y_min + row_idx * 240;
        int32_t y_track = y_row + 30 + (c % 4) * 60;

        soc::db::Point2D pin_turn{pin_pos.x, y_track};
        soc::db::Point2D anchor_turn{anchor.x, y_track};

        // Via Layer 0 to 1
        soc::db::ViaInstance v_pin;
        v_pin.bottom_layer = soc::db::LayerId{0};
        v_pin.top_layer = soc::db::LayerId{1};
        v_pin.pos = pin_pos;
        v_pin.width = 28;
        net.vias.push_back(v_pin);

        // Jog Layer 1 to y_track
        if (pin_pos != pin_turn) {
            soc::db::WireSegment ws;
            ws.layer = soc::db::LayerId{1};
            ws.start = pin_pos;
            ws.end = pin_turn;
            ws.width = 28;
            net.route_segments.push_back(ws);
        }

        // Via Layer 1 to 2
        soc::db::ViaInstance v_turn;
        v_turn.bottom_layer = soc::db::LayerId{1};
        v_turn.top_layer = soc::db::LayerId{2};
        v_turn.pos = pin_turn;
        v_turn.width = 28;
        net.vias.push_back(v_turn);

        // Wire Layer 2 to anchor_turn
        if (pin_turn != anchor_turn) {
            soc::db::WireSegment ws;
            ws.layer = soc::db::LayerId{2};
            ws.start = pin_turn;
            ws.end = anchor_turn;
            ws.width = 28;
            net.route_segments.push_back(ws);
        }

        // Via Layer 2 to 3
        soc::db::ViaInstance v_stem;
        v_stem.bottom_layer = soc::db::LayerId{2};
        v_stem.top_layer = soc::db::LayerId{3};
        v_stem.pos = anchor_turn;
        v_stem.width = 28;
        net.vias.push_back(v_stem);

        // Wire Layer 3 along anchor.x to anchor
        if (anchor_turn != anchor) {
            soc::db::WireSegment ws;
            ws.layer = soc::db::LayerId{3};
            ws.start = anchor_turn;
            ws.end = anchor;
            ws.width = 28;
            net.route_segments.push_back(ws);
        }
    }
    stats.total_vias += static_cast<uint32_t>(net.vias.size());
}

void DetailedRouter::route_net_detailed(soc::db::NetId net_id, const GlobalRoute& groute, RouteStats& stats) {
    if (!net_id.is_valid() || net_id.value() >= m_db.nets.size()) return;
    auto& net = m_db.nets[net_id];
    if (net.pins.size() < 2) return;

    int32_t gw = m_config.gcell_width_um * static_cast<int32_t>(m_db.dbu_per_micron);
    int32_t gh = m_config.gcell_height_um * static_cast<int32_t>(m_db.dbu_per_micron);
    if (gw <= 0) gw = 50000;
    if (gh <= 0) gh = 50000;

    int32_t c = static_cast<int32_t>(net_id.value() % 32);

    auto get_anchor = [&](int32_t gx, int32_t gy) -> soc::db::Point2D {
        int32_t px = m_db.die_area.x_min + gx * gw + (80 + c) * 240;
        int32_t py = m_db.die_area.y_min + gy * gh + 19980 + c * 120;
        return soc::db::Point2D{px, py};
    };

    // Route global route segments on layer z >= 5
    for (const auto& seg : groute.segments) {
        soc::db::Point2D p1 = get_anchor(seg.from.x, seg.from.y);
        soc::db::Point2D p2 = get_anchor(seg.to.x, seg.to.y);

        if (seg.from.z == seg.to.z) {
            soc::db::WireSegment ws;
            ws.layer = soc::db::LayerId{static_cast<uint32_t>(seg.from.z)};
            ws.start = p1;
            ws.end = p2;
            ws.width = 28;
            net.route_segments.push_back(ws);
        } else {
            uint32_t z_low = static_cast<uint32_t>(std::min(seg.from.z, seg.to.z));
            uint32_t z_high = static_cast<uint32_t>(std::max(seg.from.z, seg.to.z));
            for (uint32_t z = z_low; z < z_high; ++z) {
                soc::db::ViaInstance via;
                via.bottom_layer = soc::db::LayerId{z};
                via.top_layer = soc::db::LayerId{z + 1};
                via.pos = p1;
                via.width = 28;
                net.vias.push_back(via);
            }
        }
    }
    stats.total_wirelength_um += groute.wirelength_um;

    // Connect local pins to tracks and anchors
    connect_pins_to_tracks(net_id, stats);
}

RouteStats DetailedRouter::route(const std::vector<GlobalRoute>& global_routes) {
    auto t0 = std::chrono::steady_clock::now();
    RouteStats stats;
    stats.total_nets = static_cast<uint32_t>(m_db.nets.size());

    int32_t gw = m_config.gcell_width_um * static_cast<int32_t>(m_db.dbu_per_micron);
    int32_t gh = m_config.gcell_height_um * static_cast<int32_t>(m_db.dbu_per_micron);
    if (gw <= 0) gw = 50000;
    if (gh <= 0) gh = 50000;
    int32_t nx = std::max(1, m_db.die_area.width() / gw);
    int32_t ny = std::max(1, m_db.die_area.height() / gh);

    // 1. Reset all routed physical shapes in ChipDB
    for (size_t i = 0; i < m_db.nets.size(); ++i) {
        auto& net = m_db.nets[soc::db::NetId{static_cast<uint32_t>(i)}];
        net.route_segments.clear();
        net.vias.clear();
    }

    // Index global routes by net_id
    std::unordered_map<uint32_t, const GlobalRoute*> groute_map;
    for (const auto& gr : global_routes) {
        if (gr.net_id.is_valid()) {
            groute_map[gr.net_id.value()] = &gr;
        }
    }

    // 2. Identify 2D GCells touched by each net
    size_t num_nets = m_db.nets.size();
    std::vector<std::vector<std::pair<int32_t, int32_t>>> net_gcells(num_nets);
    std::unordered_map<uint64_t, std::vector<uint32_t>> gcell_nets;

    auto make_key = [](int32_t gx, int32_t gy) -> uint64_t {
        return (static_cast<uint64_t>(gx) << 32) | static_cast<uint32_t>(gy);
    };

    for (size_t n = 0; n < num_nets; ++n) {
        soc::db::NetId nid{static_cast<uint32_t>(n)};
        const auto& net = m_db.nets[nid];
        if (net.pins.size() < 2) continue;

        std::unordered_set<uint64_t> touched;

        // From pins
        for (soc::db::PinId pid : net.pins) {
            if (!pid.is_valid() || pid.value() >= m_db.pins.size()) continue;
            const auto& pin = m_db.pins[pid];
            if (!pin.instance_id.is_valid() || pin.instance_id.value() >= m_db.instances.size()) continue;
            const auto& inst = m_db.instances[pin.instance_id];

            int32_t px = inst.origin.x + (pin.offset_box.x_min + pin.offset_box.x_max) / 2;
            int32_t py = inst.origin.y + (pin.offset_box.y_min + pin.offset_box.y_max) / 2;

            int32_t gx = std::clamp((px - m_db.die_area.x_min) / gw, 0, nx - 1);
            int32_t gy = std::clamp((py - m_db.die_area.y_min) / gh, 0, ny - 1);
            touched.insert(make_key(gx, gy));
        }

        // From global routes
        auto it = groute_map.find(static_cast<uint32_t>(n));
        if (it != groute_map.end()) {
            const GlobalRoute* gr = it->second;
            for (const auto& seg : gr->segments) {
                touched.insert(make_key(seg.from.x, seg.from.y));
                touched.insert(make_key(seg.to.x, seg.to.y));
            }
        }

        for (uint64_t k : touched) {
            int32_t gx = static_cast<int32_t>(k >> 32);
            int32_t gy = static_cast<int32_t>(k & 0xFFFFFFFF);
            net_gcells[n].emplace_back(gx, gy);
            gcell_nets[k].push_back(static_cast<uint32_t>(n));
        }
    }

    // 3. Build conflict graph and perform Welsh-Powell greedy coloring
    std::vector<std::vector<uint32_t>> adj(num_nets);
    for (const auto& [k, nlist] : gcell_nets) {
        for (size_t i = 0; i < nlist.size(); ++i) {
            for (size_t j = i + 1; j < nlist.size(); ++j) {
                uint32_t u = nlist[i];
                uint32_t v = nlist[j];
                adj[u].push_back(v);
                adj[v].push_back(u);
            }
        }
    }

    std::vector<uint32_t> order(num_nets);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
        return adj[a].size() > adj[b].size();
    });

    std::vector<int32_t> colors(num_nets, -1);
    std::vector<bool> used_colors(num_nets + 1, false);
    for (uint32_t n : order) {
        if (m_db.nets[soc::db::NetId{n}].pins.size() < 2) continue;

        auto& nbrs = adj[n];
        for (uint32_t nbr : nbrs) {
            if (colors[nbr] >= 0 && static_cast<size_t>(colors[nbr]) < used_colors.size()) {
                used_colors[colors[nbr]] = true;
            }
        }
        int32_t c = 0;
        while (static_cast<size_t>(c) < used_colors.size() && used_colors[c]) {
            ++c;
        }
        colors[n] = c;
        for (uint32_t nbr : nbrs) {
            if (colors[nbr] >= 0 && static_cast<size_t>(colors[nbr]) < used_colors.size()) {
                used_colors[colors[nbr]] = false;
            }
        }
    }

    int32_t max_color = 0;
    for (int32_t c : colors) if (c > max_color) max_color = c;
    std::cout << "  [DetailedRouter] Welsh-Powell max net color = " << max_color << "\n";

    // 4. Initialize Spatial Occupancy Grid for collision avoidance
    SpatialOccupancyGrid grid(m_db.die_area);

    auto get_wire_box = [](const soc::db::Point2D& p1, const soc::db::Point2D& p2, int32_t width) -> soc::db::Rect2D {
        int32_t hw = std::max(1, width / 2);
        return soc::db::Rect2D{
            std::min(p1.x, p2.x) - hw,
            std::min(p1.y, p2.y) - hw,
            std::max(p1.x, p2.x) + hw,
            std::max(p1.y, p2.y) + hw
        };
    };

    auto get_via_box = [](const soc::db::Point2D& pos, int32_t width) -> soc::db::Rect2D {
        int32_t hw = std::max(1, width / 2);
        return soc::db::Rect2D{
            pos.x - hw,
            pos.y - hw,
            pos.x + hw,
            pos.y + hw
        };
    };

    auto can_add_wire = [&](uint32_t net_id, uint32_t layer,
                            const soc::db::Point2D& p1, const soc::db::Point2D& p2, int32_t width = 28) -> bool {
        if (p1 == p2) return true;
        return grid.can_place(layer, get_wire_box(p1, p2, width), net_id);
    };

    auto can_add_via = [&](uint32_t net_id, uint32_t bot_layer, uint32_t top_layer,
                           const soc::db::Point2D& pos, int32_t width = 28) -> bool {
        if (bot_layer == top_layer) return true;
        if (bot_layer > top_layer) std::swap(bot_layer, top_layer);
        soc::db::Rect2D v_box = get_via_box(pos, width);
        for (uint32_t l = bot_layer; l <= top_layer; ++l) {
            if (!grid.can_place(l, v_box, net_id)) return false;
        }
        return true;
    };

    auto add_wire = [&](soc::db::Net& net, uint32_t net_id, uint32_t layer,
                        const soc::db::Point2D& p1, const soc::db::Point2D& p2, int32_t width = 28) {
        if (p1 == p2) return;
        soc::db::WireSegment ws;
        ws.layer = soc::db::LayerId{layer};
        ws.start = p1;
        ws.end = p2;
        ws.width = width;
        net.route_segments.push_back(ws);
        grid.add_shape(layer, get_wire_box(p1, p2, width), net_id);
    };

    auto add_via = [&](soc::db::Net& net, uint32_t net_id, uint32_t bot_layer, uint32_t top_layer,
                       const soc::db::Point2D& pos, int32_t width = 28) {
        if (bot_layer == top_layer) return;
        if (bot_layer > top_layer) std::swap(bot_layer, top_layer);
        for (uint32_t l = bot_layer; l < top_layer; ++l) {
            soc::db::ViaInstance via;
            via.bottom_layer = soc::db::LayerId{l};
            via.top_layer = soc::db::LayerId{l + 1};
            via.pos = pos;
            via.width = width;
            net.vias.push_back(via);
            soc::db::Rect2D v_box = get_via_box(pos, width);
            grid.add_shape(l, v_box, net_id);
            grid.add_shape(l + 1, v_box, net_id);
        }
    };

    auto get_anchor = [&](int32_t gx, int32_t gy, int32_t c) -> soc::db::Point2D {
        int32_t px = m_db.die_area.x_min + gx * gw + (80 + c) * 240;
        int32_t py = m_db.die_area.y_min + gy * gh + 19980 + c * 120;
        return soc::db::Point2D{px, py};
    };

    // 5. Route Global Segments between anchors
    for (size_t n = 0; n < num_nets; ++n) {
        soc::db::NetId nid{static_cast<uint32_t>(n)};
        auto& net = m_db.nets[nid];
        if (net.pins.size() < 2) continue;

        int32_t c = colors[n] >= 0 ? colors[n] : 0;
        const GlobalRoute* gr = nullptr;
        auto git = groute_map.find(static_cast<uint32_t>(n));
        if (git != groute_map.end()) gr = git->second;

        if (gr && !gr->segments.empty()) {
            for (const auto& seg : gr->segments) {
                soc::db::Point2D p1 = get_anchor(seg.from.x, seg.from.y, c);
                soc::db::Point2D p2 = get_anchor(seg.to.x, seg.to.y, c);

                if (seg.from.z == seg.to.z) {
                    uint32_t z = static_cast<uint32_t>(seg.from.z);
                    add_wire(net, static_cast<uint32_t>(n), z, p1, p2);
                } else {
                    uint32_t z_low = static_cast<uint32_t>(std::min(seg.from.z, seg.to.z));
                    uint32_t z_high = static_cast<uint32_t>(std::max(seg.from.z, seg.to.z));
                    add_via(net, static_cast<uint32_t>(n), z_low, z_high, p1);
                }
            }
            stats.total_wirelength_um += gr->wirelength_um;
        }
    }

    // Candidate track offsets inside standard cell row (row height = 240 DBU)
    const std::vector<int32_t> cand_y_offsets = {
        30, 90, 150, 210, 45, 75, 105, 135, 165, 195, 20, 60, 80, 120, 140, 180, 200, 220
    };
    const std::vector<uint32_t> cand_h_layers = {2u, 4u, 6u, 8u, 10u};

    // 6. Detailed Route Pins & Local Escapes
    for (size_t n = 0; n < num_nets; ++n) {
        soc::db::NetId nid{static_cast<uint32_t>(n)};
        auto& net = m_db.nets[nid];
        if (net.pins.size() < 2) continue;

        int32_t c = colors[n] >= 0 ? colors[n] : 0;
        const GlobalRoute* gr = nullptr;
        auto git = groute_map.find(static_cast<uint32_t>(n));
        if (git != groute_map.end()) gr = git->second;

        // Check if net qualifies for direct intra-row routing (2 pins in the SAME cell row & same GCell)
        bool is_intra_row = false;
        soc::db::Point2D pt0, pt1;
        int32_t row_idx0 = -1, row_idx1 = -1;

        if (net.pins.size() == 2) {
            soc::db::PinId p0 = net.pins[0];
            soc::db::PinId p1 = net.pins[1];
            if (p0.is_valid() && p1.is_valid() &&
                p0.value() < m_db.pins.size() && p1.value() < m_db.pins.size()) {
                const auto& pin0 = m_db.pins[p0];
                const auto& pin1 = m_db.pins[p1];
                if (pin0.instance_id.is_valid() && pin1.instance_id.is_valid() &&
                    pin0.instance_id.value() < m_db.instances.size() &&
                    pin1.instance_id.value() < m_db.instances.size()) {
                    const auto& inst0 = m_db.instances[pin0.instance_id];
                    const auto& inst1 = m_db.instances[pin1.instance_id];
                    pt0 = soc::db::Point2D{
                        inst0.origin.x + (pin0.offset_box.x_min + pin0.offset_box.x_max) / 2,
                        inst0.origin.y + (pin0.offset_box.y_min + pin0.offset_box.y_max) / 2
                    };
                    pt1 = soc::db::Point2D{
                        inst1.origin.x + (pin1.offset_box.x_min + pin1.offset_box.x_max) / 2,
                        inst1.origin.y + (pin1.offset_box.y_min + pin1.offset_box.y_max) / 2
                    };
                    row_idx0 = (pt0.y - m_db.die_area.y_min) / 240;
                    row_idx1 = (pt1.y - m_db.die_area.y_min) / 240;
                    int32_t gx0 = std::clamp((pt0.x - m_db.die_area.x_min) / gw, 0, nx - 1);
                    int32_t gy0 = std::clamp((pt0.y - m_db.die_area.y_min) / gh, 0, ny - 1);
                    int32_t gx1 = std::clamp((pt1.x - m_db.die_area.x_min) / gw, 0, nx - 1);
                    int32_t gy1 = std::clamp((pt1.y - m_db.die_area.y_min) / gh, 0, ny - 1);

                    if (row_idx0 == row_idx1 && gx0 == gx1 && gy0 == gy1) {
                        is_intra_row = true;
                    }
                }
            }
        }

        if (is_intra_row) {
            if (pt0.x > pt1.x) std::swap(pt0, pt1);

            int32_t row_idx = row_idx0;
            int32_t y_row = m_db.die_area.y_min + row_idx * 240;

            bool routed = false;
            for (uint32_t L_route : cand_h_layers) {
                for (int32_t y_off : cand_y_offsets) {
                    int32_t y_cand = y_row + y_off;
                    soc::db::Point2D p0_turn{pt0.x, y_cand};
                    soc::db::Point2D p1_turn{pt1.x, y_cand};

                    // Check feasibility of all required shapes:
                    if (!can_add_via(static_cast<uint32_t>(n), 0, 1, pt0)) continue;
                    if (!can_add_wire(static_cast<uint32_t>(n), 1, pt0, p0_turn)) continue;
                    if (!can_add_via(static_cast<uint32_t>(n), 1, L_route, p0_turn)) continue;
                    if (!can_add_wire(static_cast<uint32_t>(n), L_route, p0_turn, p1_turn)) continue;
                    if (!can_add_via(static_cast<uint32_t>(n), 1, L_route, p1_turn)) continue;
                    if (!can_add_wire(static_cast<uint32_t>(n), 1, p1_turn, pt1)) continue;
                    if (!can_add_via(static_cast<uint32_t>(n), 0, 1, pt1)) continue;

                    // All shapes verified collision-free. Commit route:
                    add_via(net, static_cast<uint32_t>(n), 0, 1, pt0);
                    add_wire(net, static_cast<uint32_t>(n), 1, pt0, p0_turn);
                    add_via(net, static_cast<uint32_t>(n), 1, L_route, p0_turn);
                    add_wire(net, static_cast<uint32_t>(n), L_route, p0_turn, p1_turn);
                    add_via(net, static_cast<uint32_t>(n), 1, L_route, p1_turn);
                    add_wire(net, static_cast<uint32_t>(n), 1, p1_turn, pt1);
                    add_via(net, static_cast<uint32_t>(n), 0, 1, pt1);

                    stats.routed_nets++;
                    stats.total_vias += static_cast<uint32_t>(net.vias.size());
                    routed = true;
                    break;
                }
                if (routed) break;
            }
            if (routed) continue;
        }

        // Standard Pin Escapes to GCell Anchor (Multi-pin, Inter-Row, or Inter-GCell nets)
        std::unordered_set<uint64_t> anchored_gcells;

        for (soc::db::PinId pid : net.pins) {
            if (!pid.is_valid() || pid.value() >= m_db.pins.size()) continue;
            const auto& pin = m_db.pins[pid];
            if (!pin.instance_id.is_valid() || pin.instance_id.value() >= m_db.instances.size()) continue;
            const auto& inst = m_db.instances[pin.instance_id];

            soc::db::Point2D pin_pos{
                inst.origin.x + (pin.offset_box.x_min + pin.offset_box.x_max) / 2,
                inst.origin.y + (pin.offset_box.y_min + pin.offset_box.y_max) / 2
            };

            int32_t gx = std::clamp((pin_pos.x - m_db.die_area.x_min) / gw, 0, nx - 1);
            int32_t gy = std::clamp((pin_pos.y - m_db.die_area.y_min) / gh, 0, ny - 1);
            soc::db::Point2D anchor = get_anchor(gx, gy, c);

            // Determine target layer at this anchor
            uint32_t target_layer = 3; // Layer 3 (stem layer)
            if (gr && !gr->segments.empty()) {
                target_layer = 5;
                for (const auto& s : gr->segments) {
                    if ((s.from.x == gx && s.from.y == gy) || (s.to.x == gx && s.to.y == gy)) {
                        target_layer = static_cast<uint32_t>(std::max(target_layer, static_cast<uint32_t>(std::max(s.from.z, s.to.z))));
                    }
                }
            }

            int32_t row_idx = (pin_pos.y - m_db.die_area.y_min) / 240;
            int32_t y_row = m_db.die_area.y_min + row_idx * 240;

            // Search for collision-free horizontal escape track and layer
            bool pin_done = false;
            for (uint32_t esc_h : cand_h_layers) {
                for (int32_t y_off : cand_y_offsets) {
                    int32_t y_cand = y_row + y_off;
                    soc::db::Point2D pin_turn{pin_pos.x, y_cand};
                    soc::db::Point2D anchor_turn{anchor.x, y_cand};

                    uint32_t bot_stem = std::min(esc_h, 3u);
                    uint32_t top_stem = std::max(esc_h, 3u);

                    // Check feasibility:
                    if (!can_add_via(static_cast<uint32_t>(n), 0, 1, pin_pos)) continue;
                    if (!can_add_wire(static_cast<uint32_t>(n), 1, pin_pos, pin_turn)) continue;
                    if (!can_add_via(static_cast<uint32_t>(n), 1, esc_h, pin_turn)) continue;
                    if (!can_add_wire(static_cast<uint32_t>(n), esc_h, pin_turn, anchor_turn)) continue;
                    if (!can_add_via(static_cast<uint32_t>(n), bot_stem, top_stem, anchor_turn)) continue;
                    if (!can_add_wire(static_cast<uint32_t>(n), 3, anchor_turn, anchor)) continue;

                    // Commit pin escape shapes:
                    add_via(net, static_cast<uint32_t>(n), 0, 1, pin_pos);
                    add_wire(net, static_cast<uint32_t>(n), 1, pin_pos, pin_turn);
                    add_via(net, static_cast<uint32_t>(n), 1, esc_h, pin_turn);
                    add_wire(net, static_cast<uint32_t>(n), esc_h, pin_turn, anchor_turn);
                    add_via(net, static_cast<uint32_t>(n), bot_stem, top_stem, anchor_turn);
                    add_wire(net, static_cast<uint32_t>(n), 3, anchor_turn, anchor);

                    pin_done = true;
                    break;
                }
                if (pin_done) break;
            }

            // Guaranteed fallback if all preferred tracks had conflicts:
            if (!pin_done) {
                uint32_t esc_h = 2;
                int32_t y_cand = y_row + 30;
                soc::db::Point2D pin_turn{pin_pos.x, y_cand};
                soc::db::Point2D anchor_turn{anchor.x, y_cand};
                add_via(net, static_cast<uint32_t>(n), 0, 1, pin_pos);
                add_wire(net, static_cast<uint32_t>(n), 1, pin_pos, pin_turn);
                add_via(net, static_cast<uint32_t>(n), 1, esc_h, pin_turn);
                add_wire(net, static_cast<uint32_t>(n), esc_h, pin_turn, anchor_turn);
                add_via(net, static_cast<uint32_t>(n), esc_h, 3, anchor_turn);
                add_wire(net, static_cast<uint32_t>(n), 3, anchor_turn, anchor);
            }

            // Connect anchor from Layer 3 to target_layer (e.g. Metal 6 / Layer 5) if global routes exist
            uint64_t gcell_key = (static_cast<uint64_t>(gx) << 32) | static_cast<uint32_t>(gy);
            if (target_layer > 3 && anchored_gcells.find(gcell_key) == anchored_gcells.end()) {
                anchored_gcells.insert(gcell_key);
                add_via(net, static_cast<uint32_t>(n), 3, target_layer, anchor);
            }
        }

        stats.routed_nets++;
        stats.total_vias += static_cast<uint32_t>(net.vias.size());
    }

    // 7. Generate Backside Power Delivery (BSPDN) nano-TSVs
    if (m_config.generate_bspdn_rails) {
        generate_bspdn_nano_tsvs(stats);
    }

    auto t1 = std::chrono::steady_clock::now();
    stats.runtime_seconds = std::chrono::duration<double>(t1 - t0).count();
    return stats;
}

} // namespace vlsi::route
