#include "global_placer.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <iostream>

namespace vlsi::place {

GlobalPlacer::GlobalPlacer(soc::db::ChipDB& db, GlobalPlacerConfig config)
    : m_db(db), m_config(config) {}

void GlobalPlacer::init_placement_data() {
    m_movable_cells.clear();
    m_inst_to_movable.clear();
    m_fast_nets.clear();

    m_core_box = m_db.die_area;
    if (m_core_box.width() <= 0 || m_core_box.height() <= 0) {
        m_core_box = soc::db::Rect2D{0, 0, 10000000, 10000000};
    }

    // 1. Identify all movable standard cell instances
    for (size_t i = 0; i < m_db.instances.size(); ++i) {
        soc::db::InstanceId inst_id{static_cast<uint32_t>(i)};
        const auto& inst = m_db.instances[inst_id];
        if (inst.is_fixed || !inst.master_id.is_valid()) {
            continue;
        }

        const auto& master = m_db.master_cells[inst.master_id];
        if (master.cell_type == soc::db::CellType::MACRO ||
            master.cell_type == soc::db::CellType::BLOCK_ABSTRACT) {
            continue;
        }

        uint32_t m_idx = static_cast<uint32_t>(m_movable_cells.size());
        int32_t w = (master.width > 0) ? master.width : 240;
        int32_t h = (master.height > 0) ? master.height : 240;
        double area = static_cast<double>(w) * static_cast<double>(h);

        m_movable_cells.push_back(MovableCell{inst_id, w, h, area});
        m_inst_to_movable[inst_id.value()] = m_idx;
    }

    // 2. Pre-process nets into FastNet structures
    for (size_t net_idx = 0; net_idx < m_db.nets.size(); ++net_idx) {
        soc::db::NetId net_id{static_cast<uint32_t>(net_idx)};
        const auto& net = m_db.nets[net_id];
        if (net.pins.size() < 2) continue;

        FastNet fn;
        fn.net_id = net_id;

        for (auto pin_id : net.pins) {
            const auto& pin = m_db.pins[pin_id];
            NetPinRef pref;

            if (pin.instance_id.is_valid()) {
                auto it = m_inst_to_movable.find(pin.instance_id.value());
                if (it != m_inst_to_movable.end()) {
                    pref.is_movable = true;
                    pref.movable_idx = it->second;
                    pref.offset_x = pin.offset_box.center().x;
                    pref.offset_y = pin.offset_box.center().y;
                } else {
                    // Fixed instance / macro
                    pref.is_movable = false;
                    const auto& inst = m_db.instances[pin.instance_id];
                    pref.fixed_x = inst.origin.x + pin.offset_box.center().x;
                    pref.fixed_y = inst.origin.y + pin.offset_box.center().y;
                }
            } else {
                // Top-level boundary pin / pad
                pref.is_movable = false;
                pref.fixed_x = pin.offset_box.center().x;
                pref.fixed_y = pin.offset_box.center().y;
            }
            fn.pins.push_back(pref);
        }

        if (fn.pins.size() >= 2) {
            m_fast_nets.push_back(std::move(fn));
        }
    }
}

double GlobalPlacer::compute_hpwl() const {
    double total_hpwl = 0.0;

    for (const auto& fn : m_fast_nets) {
        double min_x = 1e18, max_x = -1e18;
        double min_y = 1e18, max_y = -1e18;

        for (const auto& pin : fn.pins) {
            double px = 0.0, py = 0.0;
            if (pin.is_movable) {
                const auto& mc = m_movable_cells[pin.movable_idx];
                const auto& inst = m_db.instances[mc.inst_id];
                px = inst.origin.x + pin.offset_x;
                py = inst.origin.y + pin.offset_y;
            } else {
                px = pin.fixed_x;
                py = pin.fixed_y;
            }
            min_x = std::min(min_x, px);
            max_x = std::max(max_x, px);
            min_y = std::min(min_y, py);
            max_y = std::max(max_y, py);
        }

        if (max_x >= min_x && max_y >= min_y) {
            total_hpwl += (max_x - min_x) + (max_y - min_y);
        }
    }
    return total_hpwl;
}

double GlobalPlacer::compute_wa_hpwl_and_gradient(
    const std::vector<double>& cur_x,
    const std::vector<double>& cur_y,
    std::vector<double>& grad_x,
    std::vector<double>& grad_y) const {

    double total_wa = 0.0;
    const double gamma = m_config.gamma_wa;
    const double inv_gamma = 1.0 / gamma;

    for (const auto& fn : m_fast_nets) {
        if (fn.pins.empty()) continue;

        // 1. Gather coordinates
        double max_x = -1e18, min_x = 1e18;
        double max_y = -1e18, min_y = 1e18;

        size_t num_pins = fn.pins.size();
        std::vector<double> px(num_pins);
        std::vector<double> py(num_pins);

        for (size_t p = 0; p < num_pins; ++p) {
            const auto& pin = fn.pins[p];
            if (pin.is_movable) {
                px[p] = cur_x[pin.movable_idx] + pin.offset_x;
                py[p] = cur_y[pin.movable_idx] + pin.offset_y;
            } else {
                px[p] = pin.fixed_x;
                py[p] = pin.fixed_y;
            }
            max_x = std::max(max_x, px[p]);
            min_x = std::min(min_x, px[p]);
            max_y = std::max(max_y, py[p]);
            min_y = std::min(min_y, py[p]);
        }

        // 2. WA-HPWL for X dimension
        double sum_exp_pos_x = 0.0, sum_x_exp_pos_x = 0.0;
        double sum_exp_neg_x = 0.0, sum_x_exp_neg_x = 0.0;

        std::vector<double> a_x(num_pins);
        std::vector<double> b_x(num_pins);

        for (size_t p = 0; p < num_pins; ++p) {
            // Subtract max/min to avoid numerical floating overflow
            a_x[p] = std::exp((px[p] - max_x) * inv_gamma);
            b_x[p] = std::exp(-(px[p] - min_x) * inv_gamma);
            sum_exp_pos_x += a_x[p];
            sum_x_exp_pos_x += px[p] * a_x[p];
            sum_exp_neg_x += b_x[p];
            sum_x_exp_neg_x += px[p] * b_x[p];
        }

        double t_pos_x = sum_x_exp_pos_x / sum_exp_pos_x;
        double t_neg_x = sum_x_exp_neg_x / sum_exp_neg_x;
        total_wa += (t_pos_x - t_neg_x);

        // Accumulate gradients for X
        for (size_t p = 0; p < num_pins; ++p) {
            const auto& pin = fn.pins[p];
            if (!pin.is_movable) continue;

            double d_pos = (a_x[p] / sum_exp_pos_x) * (1.0 + (px[p] - t_pos_x) * inv_gamma);
            double d_neg = (b_x[p] / sum_exp_neg_x) * (1.0 - (px[p] - t_neg_x) * inv_gamma);
            grad_x[pin.movable_idx] += (d_pos - d_neg);
        }

        // 3. WA-HPWL for Y dimension
        double sum_exp_pos_y = 0.0, sum_y_exp_pos_y = 0.0;
        double sum_exp_neg_y = 0.0, sum_y_exp_neg_y = 0.0;

        std::vector<double> a_y(num_pins);
        std::vector<double> b_y(num_pins);

        for (size_t p = 0; p < num_pins; ++p) {
            a_y[p] = std::exp((py[p] - max_y) * inv_gamma);
            b_y[p] = std::exp(-(py[p] - min_y) * inv_gamma);
            sum_exp_pos_y += a_y[p];
            sum_y_exp_pos_y += py[p] * a_y[p];
            sum_exp_neg_y += b_y[p];
            sum_y_exp_neg_y += py[p] * b_y[p];
        }

        double t_pos_y = sum_y_exp_pos_y / sum_exp_pos_y;
        double t_neg_y = sum_y_exp_neg_y / sum_exp_neg_y;
        total_wa += (t_pos_y - t_neg_y);

        // Accumulate gradients for Y
        for (size_t p = 0; p < num_pins; ++p) {
            const auto& pin = fn.pins[p];
            if (!pin.is_movable) continue;

            double d_pos = (a_y[p] / sum_exp_pos_y) * (1.0 + (py[p] - t_pos_y) * inv_gamma);
            double d_neg = (b_y[p] / sum_exp_neg_y) * (1.0 - (py[p] - t_neg_y) * inv_gamma);
            grad_y[pin.movable_idx] += (d_pos - d_neg);
        }
    }

    return total_wa;
}

void GlobalPlacer::compute_density_force(
    const std::vector<double>& cur_x,
    const std::vector<double>& cur_y,
    std::vector<double>& force_x,
    std::vector<double>& force_y,
    double& total_overflow) const {

    uint32_t nx = m_config.bin_grid_x;
    uint32_t ny = m_config.bin_grid_y;
    double bin_w = static_cast<double>(m_core_box.width()) / nx;
    double bin_h = static_cast<double>(m_core_box.height()) / ny;
    double bin_area = bin_w * bin_h;
    double bin_cap = bin_area * m_config.target_density;

    std::vector<std::vector<double>> density(nx, std::vector<double>(ny, 0.0));
    std::vector<std::vector<double>> potential(nx, std::vector<double>(ny, 0.0));

    // 1. Spreading cell area into 2D density bins using bilinear distribution
    double total_cell_area = 0.0;
    for (size_t i = 0; i < m_movable_cells.size(); ++i) {
        const auto& mc = m_movable_cells[i];
        total_cell_area += mc.area;

        double cx = cur_x[i] + mc.width * 0.5;
        double cy = cur_y[i] + mc.height * 0.5;

        double u = (cx - m_core_box.x_min) / bin_w - 0.5;
        double v = (cy - m_core_box.y_min) / bin_h - 0.5;

        int32_t u0 = std::clamp(static_cast<int32_t>(std::floor(u)), 0, static_cast<int32_t>(nx - 1));
        int32_t v0 = std::clamp(static_cast<int32_t>(std::floor(v)), 0, static_cast<int32_t>(ny - 1));
        int32_t u1 = std::min(u0 + 1, static_cast<int32_t>(nx - 1));
        int32_t v1 = std::min(v0 + 1, static_cast<int32_t>(ny - 1));

        double fu = std::clamp(u - u0, 0.0, 1.0);
        double fv = std::clamp(v - v0, 0.0, 1.0);

        density[u0][v0] += mc.area * (1.0 - fu) * (1.0 - fv);
        density[u1][v0] += mc.area * fu * (1.0 - fv);
        density[u0][v1] += mc.area * (1.0 - fu) * fv;
        density[u1][v1] += mc.area * fu * fv;
    }

    // 2. Compute density overflow
    double overflow_area = 0.0;
    for (uint32_t u = 0; u < nx; ++u) {
        for (uint32_t v = 0; v < ny; ++v) {
            double excess = density[u][v] - bin_cap;
            if (excess > 0.0) {
                overflow_area += excess;
            }
        }
    }
    total_overflow = (total_cell_area > 0.0) ? (overflow_area / total_cell_area) : 0.0;

    // 3. Poisson Solver: Solve nabla^2 phi = -(density - bin_cap)
    // Run fast Jacobi/Gauss-Seidel relaxation with Neumann boundary conditions
    const int num_poisson_iters = 16;
    double h2 = (bin_w * bin_h);

    for (int it = 0; it < num_poisson_iters; ++it) {
        for (uint32_t u = 0; u < nx; ++u) {
            for (uint32_t v = 0; v < ny; ++v) {
                double left   = (u > 0) ? potential[u - 1][v] : potential[u][v];
                double right  = (u + 1 < nx) ? potential[u + 1][v] : potential[u][v];
                double bottom = (v > 0) ? potential[u][v - 1] : potential[u][v];
                double top    = (v + 1 < ny) ? potential[u][v + 1] : potential[u][v];

                double charge = density[u][v] - bin_cap;
                potential[u][v] = 0.25 * (left + right + bottom + top + (charge / bin_area) * h2 * 0.001);
            }
        }
    }

    // 4. Compute Electric Field E = -nabla phi and repulsive forces on cells
    for (size_t i = 0; i < m_movable_cells.size(); ++i) {
        const auto& mc = m_movable_cells[i];
        double cx = cur_x[i] + mc.width * 0.5;
        double cy = cur_y[i] + mc.height * 0.5;

        int32_t u = std::clamp(static_cast<int32_t>((cx - m_core_box.x_min) / bin_w), 0, static_cast<int32_t>(nx - 1));
        int32_t v = std::clamp(static_cast<int32_t>((cy - m_core_box.y_min) / bin_h), 0, static_cast<int32_t>(ny - 1));

        int32_t u_left  = std::max(0, u - 1);
        int32_t u_right = std::min(static_cast<int32_t>(nx - 1), u + 1);
        int32_t v_bot   = std::max(0, v - 1);
        int32_t v_top   = std::min(static_cast<int32_t>(ny - 1), v + 1);

        double ex = -(potential[u_right][v] - potential[u_left][v]) / (2.0 * bin_w);
        double ey = -(potential[u][v_top] - potential[u][v_bot]) / (2.0 * bin_h);

        // Repulsive force proportional to cell area
        force_x[i] = mc.area * ex * 0.0001;
        force_y[i] = mc.area * ey * 0.0001;
    }
}

PlacerStats GlobalPlacer::place() {
    auto start_time = std::chrono::high_resolution_clock::now();
    init_placement_data();

    PlacerStats stats;
    size_t num_cells = m_movable_cells.size();
    if (num_cells == 0) {
        return stats;
    }

    // Initialize position vectors
    std::vector<double> cur_x(num_cells);
    std::vector<double> cur_y(num_cells);
    std::vector<double> prev_x(num_cells);
    std::vector<double> prev_y(num_cells);
    std::vector<double> forecast_x(num_cells);
    std::vector<double> forecast_y(num_cells);

    for (size_t i = 0; i < num_cells; ++i) {
        const auto& inst = m_db.instances[m_movable_cells[i].inst_id];
        cur_x[i] = inst.origin.x;
        cur_y[i] = inst.origin.y;
        prev_x[i] = cur_x[i];
        prev_y[i] = cur_y[i];
    }

    stats.initial_hpwl = compute_hpwl();

    // Initial density overflow
    std::vector<double> dummy_fx(num_cells, 0.0);
    std::vector<double> dummy_fy(num_cells, 0.0);
    compute_density_force(cur_x, cur_y, dummy_fx, dummy_fy, stats.initial_overflow);

    double lambda = m_config.init_lambda;
    double step_size = m_config.learning_rate;

    std::vector<double> grad_wa_x(num_cells, 0.0);
    std::vector<double> grad_wa_y(num_cells, 0.0);
    std::vector<double> force_den_x(num_cells, 0.0);
    std::vector<double> force_den_y(num_cells, 0.0);

    // Nesterov Accelerated Gradient Descent Loop
    for (uint32_t iter = 0; iter < m_config.max_iterations; ++iter) {
        stats.iterations_executed = iter + 1;

        // 1. Nesterov momentum forecast: v^k = x^k + alpha_k (x^k - x^{k-1})
        double alpha = (iter > 0) ? (static_cast<double>(iter) / (iter + 3.0)) : 0.0;
        for (size_t i = 0; i < num_cells; ++i) {
            forecast_x[i] = cur_x[i] + alpha * (cur_x[i] - prev_x[i]);
            forecast_y[i] = cur_y[i] + alpha * (cur_y[i] - prev_y[i]);

            // Clamp forecast to die boundaries
            forecast_x[i] = std::clamp(forecast_x[i],
                                       static_cast<double>(m_core_box.x_min),
                                       static_cast<double>(m_core_box.x_max - m_movable_cells[i].width));
            forecast_y[i] = std::clamp(forecast_y[i],
                                       static_cast<double>(m_core_box.y_min),
                                       static_cast<double>(m_core_box.y_max - m_movable_cells[i].height));
        }

        // 2. Compute gradients at forecast position
        std::fill(grad_wa_x.begin(), grad_wa_x.end(), 0.0);
        std::fill(grad_wa_y.begin(), grad_wa_y.end(), 0.0);
        (void)compute_wa_hpwl_and_gradient(forecast_x, forecast_y, grad_wa_x, grad_wa_y);

        double cur_overflow = 0.0;
        compute_density_force(forecast_x, forecast_y, force_den_x, force_den_y, cur_overflow);
        stats.final_overflow = cur_overflow;

        // 3. Update positions with gradient descent step
        for (size_t i = 0; i < num_cells; ++i) {
            prev_x[i] = cur_x[i];
            prev_y[i] = cur_y[i];

            double total_grad_x = grad_wa_x[i] + lambda * force_den_x[i];
            double total_grad_y = grad_wa_y[i] + lambda * force_den_y[i];

            // Normalize or limit maximum per-iteration displacement
            double max_disp = 100000.0; // 100 um
            double disp_x = std::clamp(step_size * total_grad_x, -max_disp, max_disp);
            double disp_y = std::clamp(step_size * total_grad_y, -max_disp, max_disp);

            cur_x[i] = forecast_x[i] - disp_x;
            cur_y[i] = forecast_y[i] - disp_y;

            // Strict core area box projection
            cur_x[i] = std::clamp(cur_x[i],
                                  static_cast<double>(m_core_box.x_min),
                                  static_cast<double>(m_core_box.x_max - m_movable_cells[i].width));
            cur_y[i] = std::clamp(cur_y[i],
                                  static_cast<double>(m_core_box.y_min),
                                  static_cast<double>(m_core_box.y_max - m_movable_cells[i].height));
        }

        // 4. Adapt lambda (electrostatic penalty) and step size
        if (cur_overflow > m_config.overflow_threshold) {
            lambda *= 1.05;
        } else {
            step_size *= 0.98;
            if (iter > 15) {
                break; // Converged within overflow threshold!
            }
        }
    }

    // Write back final coordinates to ChipDB instances
    for (size_t i = 0; i < num_cells; ++i) {
        soc::db::Point2D new_pos{static_cast<int32_t>(std::round(cur_x[i])),
                                 static_cast<int32_t>(std::round(cur_y[i]))};
        m_db.move_instance(m_movable_cells[i].inst_id, new_pos);
    }

    stats.final_hpwl = compute_hpwl();
    m_db.rebuild_instance_spatial_index();
    m_db.observers.notify_dirty_region(m_core_box, soc::db::LayerId{0});

    auto end_time = std::chrono::high_resolution_clock::now();
    stats.runtime_seconds = std::chrono::duration<double>(end_time - start_time).count();

    return stats;
}

} // namespace vlsi::place
