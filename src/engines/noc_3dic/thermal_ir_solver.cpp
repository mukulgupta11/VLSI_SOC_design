#include "thermal_ir_solver.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace vlsi::noc {

ThermalIRSolver::ThermalIRSolver(soc::db::ChipDB& db, const NoCConfig& config)
    : m_db(db), m_config(config) {
    m_power_grid.resize(m_nx * m_ny, 0.0);
    m_temp_grid.resize(m_nx * m_ny, 25.0);
    m_ir_drop_grid.resize(m_nx * m_ny, 0.0);
}

void ThermalIRSolver::build_power_map() {
    std::fill(m_power_grid.begin(), m_power_grid.end(), 0.1); // 0.1 W/mm2 baseline background leakage

    int32_t die_w = m_db.die_area.width();
    int32_t die_h = m_db.die_area.height();
    if (die_w <= 0 || die_h <= 0) return;

    double dbu_to_mm = 1.0 / (static_cast<double>(m_db.dbu_per_micron) * 1000.0);
    double cell_w_mm = (static_cast<double>(die_w) / m_nx) * dbu_to_mm;
    double cell_h_mm = (static_cast<double>(die_h) / m_ny) * dbu_to_mm;
    double cell_area_mm2 = std::max(1e-6, cell_w_mm * cell_h_mm);

    // 1. Map block abstracts to spatial power dissipation
    for (size_t i = 0; i < m_db.block_abstracts.size(); ++i) {
        soc::db::BlockAbstractId bid{static_cast<uint32_t>(i)};
        const auto& block = m_db.block_abstracts[bid];

        double block_power_w = 2.0;
        if (block.macro_type == "P_CORE") {
            block_power_w = 12.0; // 12 W per high-performance P-Core
        } else if (block.macro_type == "GPU_CORE") {
            block_power_w = 16.0; // 16 W per high-throughput GPU cluster
        } else if (block.macro_type == "NPU") {
            block_power_w = 6.0;  // 6 W Neural Engine
        } else if (block.macro_type == "E_CORE") {
            block_power_w = 1.5;  // 1.5 W per efficiency core
        } else if (block.macro_type == "SLC") {
            block_power_w = 2.5;  // 2.5 W System Level Cache
        }

        int32_t gx_min = std::clamp((block.boundary.x_min - m_db.die_area.x_min) * m_nx / die_w, 0, m_nx - 1);
        int32_t gx_max = std::clamp((block.boundary.x_max - m_db.die_area.x_min) * m_nx / die_w, 0, m_nx - 1);
        int32_t gy_min = std::clamp((block.boundary.y_min - m_db.die_area.y_min) * m_ny / die_h, 0, m_ny - 1);
        int32_t gy_max = std::clamp((block.boundary.y_max - m_db.die_area.y_min) * m_ny / die_h, 0, m_ny - 1);

        int32_t covered_cells = std::max(1, (gx_max - gx_min + 1) * (gy_max - gy_min + 1));
        double power_per_cell = block_power_w / covered_cells;
        double density_w_mm2 = power_per_cell / cell_area_mm2;

        for (int32_t y = gy_min; y <= gy_max; ++y) {
            for (int32_t x = gx_min; x <= gx_max; ++x) {
                m_power_grid[y * m_nx + x] += density_w_mm2;
            }
        }
    }
}

void ThermalIRSolver::solve_thermal_fd() {
    // 3D Finite-Difference Poisson Heat Diffusion:
    // nabla . (k nabla T) + Q = 0
    // Ambient temperature = 25.0 C, heatsink package thermal resistance = 0.15 C/W
    size_t total_3d_cells = static_cast<size_t>(m_nx * m_ny * m_nz);
    std::vector<double> t_3d(total_3d_cells, 25.0);

    // Thermal conductivities (W / (m * K)):
    // Layer 0 (Silicon): 149
    // Layer 1 (BEOL): 20
    // Layer 2 (BSPDN / Bumps): 50
    // Layer 3 (Copper Heat Spreader): 400
    const double k_layer[4] = {149.0, 20.0, 50.0, 400.0};

    constexpr int32_t max_iterations = 60;
    constexpr double omega = 1.3; // SOR over-relaxation factor

    for (int32_t iter = 0; iter < max_iterations; ++iter) {
        for (int32_t z = 0; z < m_nz; ++z) {
            double kz = k_layer[z];
            for (int32_t y = 0; y < m_ny; ++y) {
                for (int32_t x = 0; x < m_nx; ++x) {
                    size_t idx = static_cast<size_t>((z * m_ny + y) * m_nx + x);

                    double q_src = (z == 0) ? m_power_grid[y * m_nx + x] * 0.05 : 0.0; // Scaled volumetric heat source
                    double sum_neighbors = 0.0;
                    double denom = 0.0;

                    // Lateral conduction (x, y)
                    if (x > 0) { sum_neighbors += t_3d[idx - 1] * kz; denom += kz; }
                    if (x < m_nx - 1) { sum_neighbors += t_3d[idx + 1] * kz; denom += kz; }
                    if (y > 0) { sum_neighbors += t_3d[idx - m_nx] * kz; denom += kz; }
                    if (y < m_ny - 1) { sum_neighbors += t_3d[idx + m_nx] * kz; denom += kz; }

                    // Vertical conduction (z)
                    if (z > 0) {
                        size_t idx_below = static_cast<size_t>(((z - 1) * m_ny + y) * m_nx + x);
                        double k_inter = 2.0 * kz * k_layer[z - 1] / (kz + k_layer[z - 1]);
                        sum_neighbors += t_3d[idx_below] * k_inter;
                        denom += k_inter;
                    }
                    if (z < m_nz - 1) {
                        size_t idx_above = static_cast<size_t>(((z + 1) * m_ny + y) * m_nx + x);
                        double k_inter = 2.0 * kz * k_layer[z + 1] / (kz + k_layer[z + 1]);
                        sum_neighbors += t_3d[idx_above] * k_inter;
                        denom += k_inter;
                    } else {
                        // Top boundary: convection to ambient air (25.0 C) through heatsink
                        double h_sink = 40.0; // Convection coefficient
                        sum_neighbors += 25.0 * h_sink;
                        denom += h_sink;
                    }

                    if (denom > 0.0) {
                        double t_new = (sum_neighbors + q_src) / denom;
                        t_3d[idx] = t_3d[idx] + omega * (t_new - t_3d[idx]);
                    }
                }
            }
        }
    }

    // Extract active silicon junction temperature (Layer 0)
    for (int32_t y = 0; y < m_ny; ++y) {
        for (int32_t x = 0; x < m_nx; ++x) {
            m_temp_grid[y * m_nx + x] = t_3d[y * m_nx + x];
        }
    }
}

void ThermalIRSolver::solve_pdn_ir_drop() {
    // 2D/3D Power Delivery Network (PDN) Resistive Mesh Solver
    // G * V = I
    // Nominal supply voltage Vdd = 0.8 V
    // With BSPDN (Backside Power Delivery Network), power taps occur directly under rows,
    // greatly reducing IR-drop compared to frontside BEOL power delivery.
    double v_nom = 0.8;
    std::vector<double> v_grid(m_nx * m_ny, v_nom);

    double r_mesh = 0.02; // 0.02 Ohm mesh wire resistance
    double r_tap = 0.005; // 0.005 Ohm nano-TSV backside tap resistance

    constexpr int32_t max_iterations = 50;

    for (int32_t iter = 0; iter < max_iterations; ++iter) {
        for (int32_t y = 0; y < m_ny; ++y) {
            for (int32_t x = 0; x < m_nx; ++x) {
                size_t idx = static_cast<size_t>(y * m_nx + x);
                // Current drawn by local cells: I = P / V
                double p_density = m_power_grid[idx];
                double i_cell = (p_density * 0.002) / v_nom; // Current in Amps

                double g_sum = 1.0 / r_tap; // Direct tap to backside VDD
                double rhs = v_nom / r_tap - i_cell;

                if (x > 0) { g_sum += 1.0 / r_mesh; rhs += v_grid[idx - 1] / r_mesh; }
                if (x < m_nx - 1) { g_sum += 1.0 / r_mesh; rhs += v_grid[idx + 1] / r_mesh; }
                if (y > 0) { g_sum += 1.0 / r_mesh; rhs += v_grid[idx - m_nx] / r_mesh; }
                if (y < m_ny - 1) { g_sum += 1.0 / r_mesh; rhs += v_grid[idx + m_nx] / r_mesh; }

                if (g_sum > 0.0) {
                    v_grid[idx] = rhs / g_sum;
                }
            }
        }
    }

    // Compute IR-drop in millivolts: delta_V = (V_nom - V_node) * 1000 mV
    for (size_t i = 0; i < v_grid.size(); ++i) {
        double drop_mv = std::max(0.0, (v_nom - v_grid[i]) * 1000.0);
        m_ir_drop_grid[i] = drop_mv;
    }
}

NoCStats ThermalIRSolver::solve() {
    auto t0 = std::chrono::steady_clock::now();
    NoCStats stats;

    // 1. Build spatial power density distribution
    build_power_map();

    // 2. Solve 3D thermal finite-difference heat diffusion
    solve_thermal_fd();

    // 3. Solve PDN resistive mesh IR-drop
    solve_pdn_ir_drop();

    // 4. Extract metrics
    double peak_temp = 25.0;
    for (double t : m_temp_grid) {
        if (t > peak_temp) peak_temp = t;
    }

    double worst_ir = 0.0;
    for (double v : m_ir_drop_grid) {
        if (v > worst_ir) worst_ir = v;
    }

    stats.peak_temp_c = peak_temp;
    stats.worst_ir_drop_mv = worst_ir;

    auto t1 = std::chrono::steady_clock::now();
    stats.runtime_seconds = std::chrono::duration<double>(t1 - t0).count();

    return stats;
}

double ThermalIRSolver::get_temperature(int32_t x_dbu, int32_t y_dbu) const noexcept {
    int32_t die_w = m_db.die_area.width();
    int32_t die_h = m_db.die_area.height();
    if (die_w <= 0 || die_h <= 0) return 25.0;

    int32_t gx = std::clamp((x_dbu - m_db.die_area.x_min) * m_nx / die_w, 0, m_nx - 1);
    int32_t gy = std::clamp((y_dbu - m_db.die_area.y_min) * m_ny / die_h, 0, m_ny - 1);
    return m_temp_grid[gy * m_nx + gx];
}

double ThermalIRSolver::get_ir_drop_mv(int32_t x_dbu, int32_t y_dbu) const noexcept {
    int32_t die_w = m_db.die_area.width();
    int32_t die_h = m_db.die_area.height();
    if (die_w <= 0 || die_h <= 0) return 0.0;

    int32_t gx = std::clamp((x_dbu - m_db.die_area.x_min) * m_nx / die_w, 0, m_nx - 1);
    int32_t gy = std::clamp((y_dbu - m_db.die_area.y_min) * m_ny / die_h, 0, m_ny - 1);
    return m_ir_drop_grid[gy * m_nx + gx];
}

} // namespace vlsi::noc
