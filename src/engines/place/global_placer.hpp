#pragma once

#include "core/db/chip_db.hpp"
#include <vector>
#include <cstdint>

namespace vlsi::place {

struct GlobalPlacerConfig {
    uint32_t max_iterations{50};
    double target_density{0.70};   // 70% target cell density
    double gamma_wa{5000.0};       // WA-HPWL smoothing parameter (in DBU)
    double init_lambda{0.005};     // Electrostatic penalty weight
    double learning_rate{150.0};   // Step size for Nesterov optimization
    uint32_t bin_grid_x{64};       // 2D density grid resolution X
    uint32_t bin_grid_y{64};       // 2D density grid resolution Y
    double overflow_threshold{0.10}; // Stop when overflow < 10%
    bool verbose{false};
};

struct PlacerStats {
    uint32_t iterations_executed{0};
    double initial_hpwl{0.0};
    double final_hpwl{0.0};
    double initial_overflow{0.0};
    double final_overflow{0.0};
    double runtime_seconds{0.0};
};

class GlobalPlacer {
public:
    explicit GlobalPlacer(soc::db::ChipDB& db, GlobalPlacerConfig config = {});

    // Execute electrostatic analytical global placement
    PlacerStats place();

    // Compute exact HPWL (Half-Perimeter Wirelength)
    [[nodiscard]] double compute_hpwl() const;

    // Compute WA-HPWL and analytical gradients
    [[nodiscard]] double compute_wa_hpwl_and_gradient(
        const std::vector<double>& cur_x,
        const std::vector<double>& cur_y,
        std::vector<double>& grad_x,
        std::vector<double>& grad_y) const;

    // Compute 2D electrostatic density field and forces on cells
    void compute_density_force(
        const std::vector<double>& cur_x,
        const std::vector<double>& cur_y,
        std::vector<double>& force_x,
        std::vector<double>& force_y,
        double& total_overflow) const;

    [[nodiscard]] const GlobalPlacerConfig& config() const noexcept { return m_config; }

private:
    soc::db::ChipDB& m_db;
    GlobalPlacerConfig m_config;

    // Internal cache of movable cells
    struct MovableCell {
        soc::db::InstanceId inst_id{soc::db::InstanceId::invalid()};
        int32_t width{0};
        int32_t height{0};
        double area{0.0};
    };
    std::vector<MovableCell> m_movable_cells;

    // Mapping from InstanceId to movable cell index
    std::unordered_map<uint32_t, uint32_t> m_inst_to_movable;

    // Net representation for fast wirelength and gradient calculation
    struct NetPinRef {
        uint32_t movable_idx{0xFFFFFFFF}; // index in m_movable_cells or invalid if fixed
        double fixed_x{0.0};              // fixed pin coordinate if not movable
        double fixed_y{0.0};
        double offset_x{0.0};             // pin offset from cell origin
        double offset_y{0.0};
        bool is_movable{false};
    };

    struct FastNet {
        soc::db::NetId net_id{soc::db::NetId::invalid()};
        std::vector<NetPinRef> pins;
    };
    std::vector<FastNet> m_fast_nets;

    // Core region boundaries
    soc::db::Rect2D m_core_box{0, 0, 0, 0};

    void init_placement_data();
};

} // namespace vlsi::place
