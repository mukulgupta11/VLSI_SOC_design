#pragma once

#include "noc_types.hpp"
#include "core/db/chip_db.hpp"
#include <vector>

namespace vlsi::noc {

class ThermalIRSolver {
public:
    explicit ThermalIRSolver(soc::db::ChipDB& db, const NoCConfig& config = {});

    // Solve 3D finite-difference heat equation and PDN resistive mesh IR-drop
    NoCStats solve();

    [[nodiscard]] double get_temperature(int32_t x_dbu, int32_t y_dbu) const noexcept;
    [[nodiscard]] double get_ir_drop_mv(int32_t x_dbu, int32_t y_dbu) const noexcept;

    [[nodiscard]] const std::vector<double>& temperature_grid() const noexcept { return m_temp_grid; }
    [[nodiscard]] const std::vector<double>& ir_drop_grid() const noexcept { return m_ir_drop_grid; }
    [[nodiscard]] int32_t grid_dim_x() const noexcept { return m_nx; }
    [[nodiscard]] int32_t grid_dim_y() const noexcept { return m_ny; }

private:
    soc::db::ChipDB& m_db;
    NoCConfig m_config;

    int32_t m_nx{32};
    int32_t m_ny{32};
    int32_t m_nz{4}; // 0: Silicon, 1: BEOL, 2: Backside/Bumps, 3: Heat Spreader

    std::vector<double> m_power_grid;    // Volumetric power density W/mm^3
    std::vector<double> m_temp_grid;     // Temperature (C) on active layer
    std::vector<double> m_ir_drop_grid;  // IR-drop (mV) across die

    void build_power_map();
    void solve_thermal_fd();
    void solve_pdn_ir_drop();
};

} // namespace vlsi::noc
