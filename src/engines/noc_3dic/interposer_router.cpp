#include "interposer_router.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace vlsi::noc {

InterposerRouter::InterposerRouter(soc::db::ChipDB& db, const NoCConfig& config)
    : m_db(db), m_config(config) {}

void InterposerRouter::generate_microbump_array(int32_t bump_count) {
    m_microbumps.clear();
    int32_t pitch_dbu = m_config.microbump_pitch_um * static_cast<int32_t>(m_db.dbu_per_micron);
    if (pitch_dbu <= 0) pitch_dbu = 25000;

    int32_t die_xmax = m_db.die_area.x_max;
    int32_t bump_x = die_xmax - (pitch_dbu * 2); // 2 pitches away from boundary
    int32_t start_y = m_db.die_area.y_min + pitch_dbu * 4;

    for (int32_t i = 0; i < bump_count; ++i) {
        MicroBump mb;
        mb.id = static_cast<uint32_t>(i);
        mb.pos = soc::db::Point2D{bump_x, start_y + i * pitch_dbu};
        mb.pitch_um = m_config.microbump_pitch_um;
        mb.diameter_um = 12;
        mb.signal_name = "UF_BUS_" + std::to_string(i);

        // Create or bind net in DB
        mb.net_id = m_db.create_net(mb.signal_name);

        m_microbumps.push_back(mb);
    }
}

void InterposerRouter::route_length_matched_rdl(NoCStats& stats) {
    if (m_microbumps.empty()) return;

    // Internal I/O driver column located 1000 um inboard from microbumps
    int32_t dbu = static_cast<int32_t>(m_db.dbu_per_micron);
    int32_t inboard_x = m_microbumps[0].pos.x - 1000 * dbu;
    soc::db::LayerId rdl_layer{static_cast<uint32_t>(std::min(15, static_cast<int32_t>(m_db.layers.size())))};

    // First pass: compute direct Manhattan length for each trace
    double max_length_um = 0.0;
    std::vector<double> direct_lengths(m_microbumps.size(), 0.0);

    for (size_t i = 0; i < m_microbumps.size(); ++i) {
        soc::db::Point2D src{inboard_x, m_microbumps[i].pos.y};
        soc::db::Point2D dst = m_microbumps[i].pos;
        double len_um = static_cast<double>(std::abs(dst.x - src.x) + std::abs(dst.y - src.y)) / static_cast<double>(dbu);
        direct_lengths[i] = len_um;
        if (len_um > max_length_um) {
            max_length_um = len_um;
        }
    }

    double target_length_um = max_length_um + 100.0; // Target length for serpentine matching
    double max_skew_ps = 0.0;

    // Propagation speed on RDL: ~1.5e8 m/s = 150 um/ps (6.67 ps/mm = 0.00667 ps/um)
    constexpr double rdl_delay_ps_per_um = 0.00667;

    for (size_t i = 0; i < m_microbumps.size(); ++i) {
        auto& mb = m_microbumps[i];
        soc::db::Point2D src{inboard_x, mb.pos.y};
        soc::db::Point2D dst = mb.pos;

        double needed_serpentine_um = target_length_um - direct_lengths[i];
        int32_t serpentine_dbu = static_cast<int32_t>(needed_serpentine_um * dbu);

        auto& net = m_db.nets[mb.net_id];

        if (serpentine_dbu > 0) {
            // Insert serpentine meander jog
            int32_t mid_x = (src.x + dst.x) / 2;
            int32_t jog_h = serpentine_dbu / 2;

            soc::db::Point2D p1{mid_x, src.y};
            soc::db::Point2D p2{mid_x, src.y + jog_h};
            soc::db::Point2D p3{mid_x + 50 * dbu, src.y + jog_h};
            soc::db::Point2D p4{mid_x + 50 * dbu, src.y};

            net.route_segments.push_back({rdl_layer, src, p1, 1000}); // 1 um wide RDL trace
            net.route_segments.push_back({rdl_layer, p1, p2, 1000});
            net.route_segments.push_back({rdl_layer, p2, p3, 1000});
            net.route_segments.push_back({rdl_layer, p3, p4, 1000});
            net.route_segments.push_back({rdl_layer, p4, dst, 1000});

            mb.escape_wirelength_um = target_length_um;
        } else {
            net.route_segments.push_back({rdl_layer, src, dst, 1000});
            mb.escape_wirelength_um = direct_lengths[i];
        }

        // Add microbump via
        soc::db::ViaInstance bump_via;
        bump_via.bottom_layer = rdl_layer;
        bump_via.top_layer = rdl_layer;
        bump_via.pos = dst;
        bump_via.width = mb.diameter_um * dbu;
        net.vias.push_back(bump_via);

        // Skew relative to target length
        double skew_ps = std::abs(mb.escape_wirelength_um - target_length_um) * rdl_delay_ps_per_um;
        mb.rdl_skew_ps = skew_ps;
        if (skew_ps > max_skew_ps) {
            max_skew_ps = skew_ps;
        }
    }

    stats.microbumps_routed = static_cast<uint32_t>(m_microbumps.size());
    stats.max_rdl_skew_ps = max_skew_ps;
}

NoCStats InterposerRouter::route_interposer() {
    auto t0 = std::chrono::steady_clock::now();
    NoCStats stats;

    // Generate UltraFusion 64-channel micro-bump array
    generate_microbump_array(64);

    // Route length-matched RDL traces
    route_length_matched_rdl(stats);

    auto t1 = std::chrono::steady_clock::now();
    stats.runtime_seconds = std::chrono::duration<double>(t1 - t0).count();

    return stats;
}

} // namespace vlsi::noc
