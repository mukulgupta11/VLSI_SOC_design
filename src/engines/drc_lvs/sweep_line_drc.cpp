#include "sweep_line_drc.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

namespace vlsi::drc_lvs {

namespace {

struct RectShape {
    soc::db::Rect2D box;
    soc::db::NetId net_id{soc::db::NetId::invalid()};
    int32_t width{0};
    bool is_obstruction{false};
};

enum class EventType : uint8_t {
    START = 0,
    END
};

struct SweepEvent {
    int32_t x{0};
    EventType type{EventType::START};
    size_t shape_index{0};

    bool operator<(const SweepEvent& o) const noexcept {
        if (x != o.x) return x < o.x;
        return type < o.type; // START before END
    }
};

struct DSU {
    std::vector<int32_t> parent;
    explicit DSU(size_t n) : parent(n) {
        std::iota(parent.begin(), parent.end(), 0);
    }
    int32_t find(int32_t i) {
        if (parent[i] == i) return i;
        return parent[i] = find(parent[i]);
    }
    void unite(int32_t i, int32_t j) {
        int32_t root_i = find(i);
        int32_t root_j = find(j);
        if (root_i != root_j) {
            parent[root_i] = root_j;
        }
    }
};

} // namespace

SweepLineDRC::SweepLineDRC(soc::db::ChipDB& db) : m_db(db) {}

void SweepLineDRC::check_layer_geometry(soc::db::LayerId layer_id, DRCReport& report) {
    int32_t min_width = 24;
    int32_t min_spacing = 24;

    if (layer_id.is_valid() && layer_id.value() < m_db.layers.size()) {
        const auto& layer = m_db.layers[layer_id];
        if (layer.min_width > 0) min_width = layer.min_width;
        if (layer.min_spacing > 0) min_spacing = layer.min_spacing;
    }

    std::vector<RectShape> shapes;

    // 1. Collect wire segments on this layer
    for (size_t n = 0; n < m_db.nets.size(); ++n) {
        soc::db::NetId nid{static_cast<uint32_t>(n)};
        const auto& net = m_db.nets[nid];

        for (const auto& ws : net.route_segments) {
            if (ws.layer != layer_id) continue;

            // Check min width rule
            if (ws.width < min_width) {
                DRCViolation v;
                v.type = DRCViolationType::MIN_WIDTH;
                v.layer = layer_id;
                v.net_a = nid;
                v.actual_val = ws.width;
                v.required_val = min_width;
                v.bbox = soc::db::Rect2D{
                    std::min(ws.start.x, ws.end.x) - ws.width / 2,
                    std::min(ws.start.y, ws.end.y) - ws.width / 2,
                    std::max(ws.start.x, ws.end.x) + ws.width / 2,
                    std::max(ws.start.y, ws.end.y) + ws.width / 2
                };
                v.message = "Wire width (" + std::to_string(ws.width) + " DBU) violates layer min width (" + std::to_string(min_width) + " DBU)";
                report.violations.push_back(v);
                report.min_width_violations++;
            }

            int32_t half_w = std::max(1, ws.width / 2);
            RectShape rs;
            rs.box = soc::db::Rect2D{
                std::min(ws.start.x, ws.end.x) - half_w,
                std::min(ws.start.y, ws.end.y) - half_w,
                std::max(ws.start.x, ws.end.x) + half_w,
                std::max(ws.start.y, ws.end.y) + half_w
            };
            rs.net_id = nid;
            rs.width = ws.width;
            rs.is_obstruction = false;
            shapes.push_back(rs);
        }

        // Collect vias landing on or touching this layer
        for (const auto& via : net.vias) {
            if (via.bottom_layer == layer_id || via.top_layer == layer_id) {
                int32_t half_w = std::max(1, via.width / 2);
                RectShape rs;
                rs.box = soc::db::Rect2D{
                    via.pos.x - half_w,
                    via.pos.y - half_w,
                    via.pos.x + half_w,
                    via.pos.y + half_w
                };
                rs.net_id = nid;
                rs.width = via.width;
                rs.is_obstruction = false;
                shapes.push_back(rs);
            }
        }
    }

    // 2. Add macro blockages if layer <= 7 (Metal 1 to Metal 8)
    if (layer_id.value() <= 7) {
        for (size_t b = 0; b < m_db.block_abstracts.size(); ++b) {
            soc::db::BlockAbstractId bid{static_cast<uint32_t>(b)};
            const auto& block = m_db.block_abstracts[bid];

            RectShape rs;
            rs.box = block.boundary;
            rs.net_id = soc::db::NetId::invalid();
            rs.width = 0;
            rs.is_obstruction = true;
            shapes.push_back(rs);
        }
    }

    if (shapes.size() < 2) return;

    // 3. Bentley-Ottmann Sweep-Line Algorithm with Spatial Y-Binning
    std::vector<SweepEvent> events;
    events.reserve(shapes.size() * 2);

    for (size_t i = 0; i < shapes.size(); ++i) {
        events.push_back(SweepEvent{shapes[i].box.x_min, EventType::START, i});
        events.push_back(SweepEvent{shapes[i].box.x_max, EventType::END, i});
    }

    std::sort(events.begin(), events.end());

    constexpr int NUM_BINS = 128;
    int32_t y_min_bound = m_db.die_area.y_min;
    int32_t y_height = std::max(1, m_db.die_area.height());
    int32_t bin_h = std::max(1, y_height / NUM_BINS);

    auto get_bin = [&](int32_t y) -> int {
        return std::clamp((y - y_min_bound) / bin_h, 0, NUM_BINS - 1);
    };

    std::vector<std::vector<size_t>> bin_shapes(NUM_BINS);
    std::vector<bool> is_active(shapes.size(), false);

    for (const auto& ev : events) {
        size_t idx = ev.shape_index;
        const auto& cur = shapes[idx];

        if (ev.type == EventType::START) {
            int b_start = get_bin(cur.box.y_min - min_spacing);
            int b_end   = get_bin(cur.box.y_max + min_spacing);

            for (int b = b_start; b <= b_end; ++b) {
                auto& bin = bin_shapes[b];
                size_t w = 0;
                for (size_t r = 0; r < bin.size(); ++r) {
                    size_t act_idx = bin[r];
                    if (!is_active[act_idx]) {
                        continue; // Lazy removal
                    }
                    bin[w++] = act_idx;

                    const auto& act = shapes[act_idx];

                    // Spatial filtering along Y
                    if (std::max(cur.box.y_min, act.box.y_min) > std::min(cur.box.y_max, act.box.y_max) + min_spacing) {
                        continue;
                    }

                    // 1. Check for macro obstruction encroachment
                    if (cur.is_obstruction || act.is_obstruction) {
                        if (cur.is_obstruction != act.is_obstruction) {
                            const auto& sig = cur.is_obstruction ? act : cur;
                            const auto& obs = cur.is_obstruction ? cur : act;

                            if (sig.box.intersects(obs.box)) {
                                DRCViolation v;
                                v.type = DRCViolationType::OBSTRUCTION_ENCROACHMENT;
                                v.layer = layer_id;
                                v.net_a = sig.net_id;
                                v.bbox = sig.box;
                                v.message = "Signal routing encroaches on hierarchical macro obstruction on layer " + std::to_string(layer_id.value());
                                report.violations.push_back(v);
                            }
                        }
                        continue;
                    }

                    // If shapes belong to the same net, they are electrically connected
                    if (!cur.net_id.is_valid() || !act.net_id.is_valid() || cur.net_id == act.net_id) {
                        continue;
                    }

                    // 2. Short circuit check: overlapping boxes of different nets
                    if (cur.box.intersects(act.box)) {
                        DRCViolation v;
                        v.type = DRCViolationType::SHORT_CIRCUIT;
                        v.layer = layer_id;
                        v.net_a = cur.net_id;
                        v.net_b = act.net_id;
                        v.bbox = cur.box;
                        v.message = "Short circuit between net " + std::to_string(cur.net_id.value()) + " and net " + std::to_string(act.net_id.value());
                        report.violations.push_back(v);
                        report.short_violations++;
                    } else {
                        // 3. Min spacing check: distance between disjoint shapes
                        int32_t dx = std::max(0, std::max(cur.box.x_min - act.box.x_max, act.box.x_min - cur.box.x_max));
                        int32_t dy = std::max(0, std::max(cur.box.y_min - act.box.y_max, act.box.y_min - cur.box.y_max));
                        int32_t dist = std::max(dx, dy);

                        if (dist < min_spacing) {
                            DRCViolation v;
                            v.type = DRCViolationType::MIN_SPACING;
                            v.layer = layer_id;
                            v.net_a = cur.net_id;
                            v.net_b = act.net_id;
                            v.actual_val = dist;
                            v.required_val = min_spacing;
                            v.bbox = cur.box;
                            v.message = "Spacing (" + std::to_string(dist) + " DBU) violates layer min spacing (" + std::to_string(min_spacing) + " DBU)";
                            report.violations.push_back(v);
                            report.min_spacing_violations++;
                        }
                    }
                }
                bin.resize(w);
            }

            is_active[idx] = true;
            int cur_bin = get_bin((cur.box.y_min + cur.box.y_max) / 2);
            bin_shapes[cur_bin].push_back(idx);
        } else {
            is_active[idx] = false;
        }
    }
}

void SweepLineDRC::check_open_circuits(DRCReport& report) {
    // Check electrical connectivity for multi-pin nets
    for (size_t n = 0; n < m_db.nets.size(); ++n) {
        soc::db::NetId nid{static_cast<uint32_t>(n)};
        const auto& net = m_db.nets[nid];

        if (net.pins.size() <= 1) continue;

        // Collect pin positions
        std::vector<soc::db::Point2D> pin_positions;
        for (soc::db::PinId pid : net.pins) {
            if (!pid.is_valid() || pid.value() >= m_db.pins.size()) continue;
            const auto& pin = m_db.pins[pid];
            if (!pin.instance_id.is_valid() || pin.instance_id.value() >= m_db.instances.size()) continue;
            const auto& inst = m_db.instances[pin.instance_id];

            pin_positions.push_back(soc::db::Point2D{
                inst.origin.x + (pin.offset_box.x_min + pin.offset_box.x_max) / 2,
                inst.origin.y + (pin.offset_box.y_min + pin.offset_box.y_max) / 2
            });
        }

        if (pin_positions.size() <= 1) continue;

        if (net.route_segments.empty() && net.vias.empty()) {
            DRCViolation v;
            v.type = DRCViolationType::OPEN_CIRCUIT;
            v.net_a = nid;
            v.message = "Unrouted net has " + std::to_string(net.pins.size()) + " disconnected pins";
            report.violations.push_back(v);
            report.open_violations++;
            continue;
        }

        size_t num_pins = pin_positions.size();
        size_t num_segs = net.route_segments.size();
        size_t num_vias = net.vias.size();
        size_t total_elements = num_pins + num_segs + num_vias;
        DSU dsu(total_elements);

        auto pin_idx = [&](size_t p) { return static_cast<int32_t>(p); };
        auto seg_idx = [&](size_t s) { return static_cast<int32_t>(num_pins + s); };
        auto via_idx = [&](size_t v) { return static_cast<int32_t>(num_pins + num_segs + v); };

        // 1. Connect pins to segments and vias on Metal 1 (layer 0)
        for (size_t p = 0; p < num_pins; ++p) {
            // Check segments on Layer 0
            for (size_t s = 0; s < num_segs; ++s) {
                const auto& ws = net.route_segments[s];
                if (ws.layer.value() != 0) continue;
                int32_t tol = ws.width * 2 + 500;
                int32_t xmin = std::min(ws.start.x, ws.end.x) - tol;
                int32_t xmax = std::max(ws.start.x, ws.end.x) + tol;
                int32_t ymin = std::min(ws.start.y, ws.end.y) - tol;
                int32_t ymax = std::max(ws.start.y, ws.end.y) + tol;

                if (pin_positions[p].x >= xmin && pin_positions[p].x <= xmax &&
                    pin_positions[p].y >= ymin && pin_positions[p].y <= ymax) {
                    dsu.unite(pin_idx(p), seg_idx(s));
                }
            }

            // Check vias landing on Layer 0
            for (size_t v = 0; v < num_vias; ++v) {
                const auto& via = net.vias[v];
                if (via.bottom_layer.value() == 0 || via.top_layer.value() == 0) {
                    int32_t tol = via.width * 2 + 500;
                    if (std::abs(pin_positions[p].x - via.pos.x) <= tol &&
                        std::abs(pin_positions[p].y - via.pos.y) <= tol) {
                        dsu.unite(pin_idx(p), via_idx(v));
                    }
                }
            }
        }

        // 2. Connect touching segments on the SAME layer
        for (size_t s1 = 0; s1 < num_segs; ++s1) {
            const auto& w1 = net.route_segments[s1];
            int32_t tol1 = w1.width * 2 + 500;
            soc::db::Rect2D box1{
                std::min(w1.start.x, w1.end.x) - tol1,
                std::min(w1.start.y, w1.end.y) - tol1,
                std::max(w1.start.x, w1.end.x) + tol1,
                std::max(w1.start.y, w1.end.y) + tol1
            };

            for (size_t s2 = s1 + 1; s2 < num_segs; ++s2) {
                const auto& w2 = net.route_segments[s2];
                if (w1.layer != w2.layer) continue;

                soc::db::Rect2D box2{
                    std::min(w2.start.x, w2.end.x),
                    std::min(w2.start.y, w2.end.y),
                    std::max(w2.start.x, w2.end.x),
                    std::max(w2.start.y, w2.end.y)
                };

                if (box1.intersects(box2)) {
                    dsu.unite(seg_idx(s1), seg_idx(s2));
                }
            }
        }

        // 3. Connect vias to touching segments across layers
        // Group vias by position for O(V) stacked via connection
        auto via_pos_key = [](const soc::db::Point2D& p) -> uint64_t {
            return (static_cast<uint64_t>(p.x) << 32) | static_cast<uint32_t>(p.y);
        };
        std::unordered_map<uint64_t, std::vector<size_t>> vias_at_pos;
        for (size_t v = 0; v < num_vias; ++v) {
            vias_at_pos[via_pos_key(net.vias[v].pos)].push_back(v);
        }

        for (const auto& [pos_k, v_list] : vias_at_pos) {
            for (size_t i = 0; i < v_list.size(); ++i) {
                for (size_t j = i + 1; j < v_list.size(); ++j) {
                    size_t v1 = v_list[i];
                    size_t v2 = v_list[j];
                    const auto& via1 = net.vias[v1];
                    const auto& via2 = net.vias[v2];
                    if (via1.top_layer == via2.bottom_layer || via1.bottom_layer == via2.top_layer) {
                        dsu.unite(via_idx(v1), via_idx(v2));
                    }
                }
            }
        }

        for (size_t v = 0; v < num_vias; ++v) {
            const auto& via = net.vias[v];
            int32_t v_tol = via.width * 2 + 500;

            for (size_t s = 0; s < num_segs; ++s) {
                const auto& ws = net.route_segments[s];
                if (ws.layer.value() >= via.bottom_layer.value() && ws.layer.value() <= via.top_layer.value()) {
                    int32_t xmin = std::min(ws.start.x, ws.end.x) - v_tol;
                    int32_t xmax = std::max(ws.start.x, ws.end.x) + v_tol;
                    int32_t ymin = std::min(ws.start.y, ws.end.y) - v_tol;
                    int32_t ymax = std::max(ws.start.y, ws.end.y) + v_tol;

                    if (via.pos.x >= xmin && via.pos.x <= xmax && via.pos.y >= ymin && via.pos.y <= ymax) {
                        dsu.unite(via_idx(v), seg_idx(s));
                    }
                }
            }
        }

        // 4. Check if all pins share the same root
        int32_t root0 = dsu.find(pin_idx(0));
        bool is_open = false;
        for (size_t p = 1; p < num_pins; ++p) {
            if (dsu.find(pin_idx(p)) != root0) {
                is_open = true;
                break;
            }
        }

        if (is_open) {
            DRCViolation v;
            v.type = DRCViolationType::OPEN_CIRCUIT;
            v.net_a = nid;
            v.message = "Net " + std::to_string(nid.value()) + " has disjoint unconnected pin clusters";
            report.violations.push_back(v);
            report.open_violations++;
        }
    }
}

void SweepLineDRC::check_bspdn_keepouts(DRCReport& report) {
    if (m_db.nano_tsvs.empty()) return;

    // In sub-3nm FinFET and 2nm GAAFET BSPDN technologies, nano-TSVs connect backside power rails
    // to frontside power rails. Signal wires on Metal 1 (layer 0) and Metal 2 (layer 1) must respect
    // a strict physical keep-out exclusion radius R_ko around each nano-TSV.
    for (const auto& tsv : m_db.nano_tsvs) {
        int32_t tsv_r = std::max(1, tsv.diameter / 2);
        int32_t keepout_r = tsv_r + 50; // 50 DBU minimum physical isolation halo

        soc::db::Rect2D ko_box{
            tsv.pos.x - keepout_r,
            tsv.pos.y - keepout_r,
            tsv.pos.x + keepout_r,
            tsv.pos.y + keepout_r
        };

        // Check Metal 1 (layer 0) and Metal 2 (layer 1)
        for (uint32_t l = 0; l <= 1; ++l) {
            soc::db::LayerId lid{l};

            for (size_t n = 0; n < m_db.nets.size(); ++n) {
                soc::db::NetId nid{static_cast<uint32_t>(n)};
                const auto& net = m_db.nets[nid];
                // Power and ground nets are permitted to tap nano-TSVs; signal/clock nets are prohibited
                if (net.signal_type == soc::db::SignalType::POWER || net.signal_type == soc::db::SignalType::GROUND) {
                    continue;
                }

                for (const auto& ws : net.route_segments) {
                    if (ws.layer != lid) continue;

                    soc::db::Rect2D w_box{
                        std::min(ws.start.x, ws.end.x) - ws.width / 2,
                        std::min(ws.start.y, ws.end.y) - ws.width / 2,
                        std::max(ws.start.x, ws.end.x) + ws.width / 2,
                        std::max(ws.start.y, ws.end.y) + ws.width / 2
                    };

                    if (!w_box.intersects(ko_box)) continue;

                    // Point-to-segment minimum squared Euclidean distance calculation
                    double dx = ws.end.x - ws.start.x;
                    double dy = ws.end.y - ws.start.y;
                    double len_sq = dx * dx + dy * dy;

                    double dist_sq = 0.0;
                    if (len_sq < 1e-6) {
                        double px = ws.start.x - tsv.pos.x;
                        double py = ws.start.y - tsv.pos.y;
                        dist_sq = px * px + py * py;
                    } else {
                        double t = ((tsv.pos.x - ws.start.x) * dx + (tsv.pos.y - ws.start.y) * dy) / len_sq;
                        t = std::clamp(t, 0.0, 1.0);
                        double proj_x = ws.start.x + t * dx;
                        double proj_y = ws.start.y + t * dy;
                        double px = proj_x - tsv.pos.x;
                        double py = proj_y - tsv.pos.y;
                        dist_sq = px * px + py * py;
                    }

                    int32_t eff_radius = keepout_r + ws.width / 2;
                    if (dist_sq < static_cast<double>(eff_radius) * static_cast<double>(eff_radius)) {
                        DRCViolation v;
                        v.type = DRCViolationType::BSPDN_KEEPOUT_VIOLATION;
                        v.layer = lid;
                        v.net_a = nid;
                        v.bbox = ko_box;
                        v.actual_val = static_cast<int32_t>(std::sqrt(dist_sq));
                        v.required_val = eff_radius;
                        v.message = "Signal net " + std::to_string(n) + " encroaches on BSPDN nano-TSV " +
                                    std::to_string(tsv.id) + " keepout zone (distance: " +
                                    std::to_string(v.actual_val) + " DBU, min required: " +
                                    std::to_string(eff_radius) + " DBU)";
                        report.violations.push_back(v);
                        report.bspdn_keepout_violations++;
                    }
                }
            }
        }
    }
}

DRCReport SweepLineDRC::run() {
    auto t0 = std::chrono::steady_clock::now();
    DRCReport report;

    // Check layer geometry for all routing layers (Metal 1 to 15)
    uint32_t max_layers = static_cast<uint32_t>(std::min(15, static_cast<int32_t>(m_db.layers.size())));
    for (uint32_t l = 0; l < max_layers; ++l) {
        auto tl0 = std::chrono::steady_clock::now();
        check_layer_geometry(soc::db::LayerId{l}, report);
        auto tl1 = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(tl1 - tl0).count();
        if (dt > 0.05) {
            std::cout << "    [DRC Timing] Layer " << l << " check took " << dt << "s\n";
        }
    }

    auto to0 = std::chrono::steady_clock::now();
    // Check open circuit net connectivity
    check_open_circuits(report);
    auto to1 = std::chrono::steady_clock::now();
    double dto = std::chrono::duration<double>(to1 - to0).count();
    if (dto > 0.05) {
        std::cout << "    [DRC Timing] Open circuits check took " << dto << "s\n";
    }

    // Check BSPDN keepout zones
    check_bspdn_keepouts(report);

    report.total_violations = static_cast<uint32_t>(report.violations.size());

    auto t1 = std::chrono::steady_clock::now();
    report.runtime_seconds = std::chrono::duration<double>(t1 - t0).count();

    return report;
}

} // namespace vlsi::drc_lvs
