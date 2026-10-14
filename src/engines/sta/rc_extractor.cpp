#include "rc_extractor.hpp"
#include <cmath>
#include <algorithm>

namespace vlsi::sta {

ParasiticRCExtractor::ParasiticRCExtractor(soc::db::ChipDB& db) : m_db(db) {
    // Populate default FinFET / GAAFET sub-3nm layer parasitics (Metal1..Metal15)
    // Lower metals (M1-M3): high resistance, tight pitch
    for (uint32_t l = 1; l <= 3; ++l) {
        m_layer_parasitics[l] = LayerParasitics{0.85, 0.22, 3.5};
    }
    // Intermediate routing metals (M4-M8): balanced
    for (uint32_t l = 4; l <= 8; ++l) {
        m_layer_parasitics[l] = LayerParasitics{0.25, 0.16, 1.8};
    }
    // Top fat metals / global clock spine (M9-M15): ultra low resistance
    for (uint32_t l = 9; l <= 15; ++l) {
        m_layer_parasitics[l] = LayerParasitics{0.04, 0.12, 0.8};
    }
}

void ParasiticRCExtractor::set_layer_parasitics(soc::db::LayerId layer_id, LayerParasitics params) {
    if (layer_id.is_valid()) {
        m_layer_parasitics[layer_id.value()] = params;
    }
}

LayerParasitics ParasiticRCExtractor::get_layer_params(soc::db::LayerId layer_id) const noexcept {
    if (layer_id.is_valid()) {
        auto it = m_layer_parasitics.find(layer_id.value());
        if (it != m_layer_parasitics.end()) {
            return it->second;
        }
    }
    return LayerParasitics{0.25, 0.16, 1.8};
}

double ParasiticRCExtractor::compute_arnoldi_c_eff(double c_near_ff,
                                                   double r_wire_ohm,
                                                   double c_far_ff,
                                                   double input_slew_ps) const noexcept {
    // Arnoldi 2nd-order moment matching for resistive shielding
    if (r_wire_ohm < 1e-3 || c_far_ff < 1e-3 || input_slew_ps < 1.0) {
        return c_near_ff + c_far_ff;
    }

    // Convert to standard units: R in kOhms, C in pF, slew in ns
    double r_kohm = r_wire_ohm * 1e-3;
    double c_far_pf = c_far_ff * 1e-3;
    double c_near_pf = c_near_ff * 1e-3;
    double t_slew_ns = input_slew_ps * 1e-3;

    double rc_tau = r_kohm * c_far_pf; // in nanoseconds
    double ratio = t_slew_ns / rc_tau;

    // Shielding factor derived from 2nd-order Arnoldi expansion of driver admittance
    double shielding_factor = 0.0;
    if (ratio > 20.0) {
        // Slow slew compared to RC: full far-end capacitance seen
        shielding_factor = 1.0;
    } else if (ratio < 0.05) {
        // Very fast slew: heavy resistive shielding, far capacitance completely shielded
        shielding_factor = ratio * 0.5;
    } else {
        shielding_factor = 1.0 - (2.0 / ratio) * (1.0 - std::exp(-ratio));
    }

    shielding_factor = std::clamp(shielding_factor, 0.0, 1.0);
    double c_eff_pf = c_near_pf + c_far_pf * shielding_factor;
    return c_eff_pf * 1e3; // Return in fF
}

PiModel ParasiticRCExtractor::extract_net_arc_rc(soc::db::NetId net_id,
                                                soc::db::PinId sink_pin_id,
                                                double driver_slew_ps) const {
    PiModel model;
    if (!net_id.is_valid() || net_id.value() >= m_db.nets.size()) {
        return model;
    }

    const auto& net = m_db.nets[net_id];
    if (net.pins.empty()) {
        return model;
    }

    soc::db::PinId drv_pin_id = net.pins.front();
    const auto& drv_pin = m_db.pins[drv_pin_id];
    const auto& sink_pin = m_db.pins[sink_pin_id];

    // Compute driver physical coordinate
    soc::db::Point2D drv_pos = drv_pin.offset_box.center();
    if (drv_pin.instance_id.is_valid()) {
        drv_pos = m_db.instances[drv_pin.instance_id].origin + drv_pos;
    }

    // Compute sink physical coordinate
    soc::db::Point2D sink_pos = sink_pin.offset_box.center();
    if (sink_pin.instance_id.is_valid()) {
        sink_pos = m_db.instances[sink_pin.instance_id].origin + sink_pos;
    }

    double dbu_to_um = 1.0 / std::max(1.0, static_cast<double>(m_db.dbu_per_micron));

    // Receiver pin input capacitance (nominal 1.5 fF for standard cell inputs)
    double pin_cap_ff = 1.5;

    // 1. If detailed route segments exist in the net, extract segment-by-segment
    if (!net.route_segments.empty()) {
        double total_r = 0.0;
        double total_c = 0.0;

        for (const auto& seg : net.route_segments) {
            double seg_len_um = std::hypot(seg.end.x - seg.start.x, seg.end.y - seg.start.y) * dbu_to_um;
            LayerParasitics lp = get_layer_params(seg.layer);
            total_r += lp.r_unit_ohm_per_um * seg_len_um;
            total_c += lp.c_unit_ff_per_um * seg_len_um;
        }

        model.r_wire_ohm = std::max(0.1, total_r);
        model.c_near_ff = total_c * 0.5;
        model.c_far_ff = total_c * 0.5 + pin_cap_ff;
    } else {
        // 2. Pre-route Steiner / Manhattan estimate: driver to sink flight path
        double manhattan_dist_um = (std::abs(sink_pos.x - drv_pos.x) + std::abs(sink_pos.y - drv_pos.y)) * dbu_to_um;

        // Default to intermediate metal 4
        LayerParasitics lp = get_layer_params(soc::db::LayerId{4});

        double total_r = lp.r_unit_ohm_per_um * manhattan_dist_um;
        double total_c = lp.c_unit_ff_per_um * manhattan_dist_um;

        model.r_wire_ohm = std::max(0.1, total_r);
        model.c_near_ff = total_c * 0.5;
        model.c_far_ff = total_c * 0.5 + pin_cap_ff;
    }

    // Compute Elmore Delay in picoseconds: tau = R * (C_far + C_near / 2)
    // R in Ohms, C in fF -> tau in Ohms * fF = 1e-15 s = 0.001 ps
    double tau_s = (model.r_wire_ohm * (model.c_far_ff + 0.5 * model.c_near_ff)) * 1e-15;
    model.elmore_delay_ps = tau_s * 1e12; // in picoseconds

    // Compute Arnoldi Reduced-Order effective capacitance
    model.c_eff_ff = compute_arnoldi_c_eff(model.c_near_ff, model.r_wire_ohm, model.c_far_ff, driver_slew_ps);

    return model;
}

} // namespace vlsi::sta
