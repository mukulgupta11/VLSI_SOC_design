#pragma once

#include "core/db/chip_db.hpp"
#include <vector>
#include <unordered_map>

namespace vlsi::sta {

struct LayerParasitics {
    double r_unit_ohm_per_um{0.35}; // Wire resistance in Ohms per micron
    double c_unit_ff_per_um{0.18};   // Wire capacitance in fF per micron
    double via_res_ohm{1.5};         // Via contact resistance in Ohms
};

struct PiModel {
    double c_near_ff{0.0};  // Near-end capacitance seen by driver (fF)
    double r_wire_ohm{0.0}; // Interconnect resistance (Ohms)
    double c_far_ff{0.0};   // Far-end receiver capacitance (fF)
    double elmore_delay_ps{0.0}; // Elmore delay in picoseconds
    double c_eff_ff{0.0};   // Arnoldi effective capacitance with resistive shielding (fF)
};

class ParasiticRCExtractor {
public:
    explicit ParasiticRCExtractor(soc::db::ChipDB& db);

    // Set custom parasitic technology parameters for a layer
    void set_layer_parasitics(soc::db::LayerId layer_id, LayerParasitics params);

    // Extract RC Pi-model and Elmore delay for a specific net and sink pin
    [[nodiscard]] PiModel extract_net_arc_rc(soc::db::NetId net_id,
                                             soc::db::PinId sink_pin_id,
                                             double driver_slew_ps = 30.0) const;

    // Compute Arnoldi 2nd-order moment-matched effective capacitance C_eff
    [[nodiscard]] double compute_arnoldi_c_eff(double c_near_ff,
                                               double r_wire_ohm,
                                               double c_far_ff,
                                               double input_slew_ps) const noexcept;

private:
    soc::db::ChipDB& m_db;
    std::unordered_map<uint32_t, LayerParasitics> m_layer_parasitics;

    [[nodiscard]] LayerParasitics get_layer_params(soc::db::LayerId layer_id) const noexcept;
};

} // namespace vlsi::sta
