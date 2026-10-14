#pragma once

#include "core/db/id_types.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <numbers>

namespace vlsi::sta {

// Parametric On-Chip Variation (POCV) Gaussian distribution representation: (mean, sigma)
struct GaussianDelay {
    double mean{0.0};  // Nominal delay in picoseconds (ps)
    double sigma{0.0}; // Standard deviation (variation) in picoseconds (ps)

    constexpr GaussianDelay() noexcept = default;
    constexpr GaussianDelay(double m, double s = 0.0) noexcept : mean(m), sigma(std::abs(s)) {}

    // Statistical addition: independent random variables add variances
    GaussianDelay operator+(const GaussianDelay& o) const noexcept {
        return {mean + o.mean, std::sqrt(sigma * sigma + o.sigma * o.sigma)};
    }

    // Statistical subtraction: variances still add
    GaussianDelay operator-(const GaussianDelay& o) const noexcept {
        return {mean - o.mean, std::sqrt(sigma * sigma + o.sigma * o.sigma)};
    }

    GaussianDelay& operator+=(const GaussianDelay& o) noexcept {
        mean += o.mean;
        sigma = std::sqrt(sigma * sigma + o.sigma * o.sigma);
        return *this;
    }

    // Scalar scaling
    GaussianDelay operator*(double scalar) const noexcept {
        return {mean * scalar, sigma * std::abs(scalar)};
    }

    // 3-Sigma worst-case late arrival: mean + 3 * sigma
    [[nodiscard]] double late_3sigma() const noexcept {
        return mean + 3.0 * sigma;
    }

    // 3-Sigma worst-case early arrival: mean - 3 * sigma
    [[nodiscard]] double early_3sigma() const noexcept {
        return mean - 3.0 * sigma;
    }

    // Clark's statistical maximum operator for two Gaussian distributions
    static GaussianDelay clark_max(const GaussianDelay& x1, const GaussianDelay& x2) noexcept {
        if (x1.sigma < 1e-6 && x2.sigma < 1e-6) {
            return {std::max(x1.mean, x2.mean), 0.0};
        }

        double var1 = x1.sigma * x1.sigma;
        double var2 = x2.sigma * x2.sigma;
        double a = std::sqrt(std::max(1e-12, var1 + var2));
        double alpha = (x1.mean - x2.mean) / a;

        // Standard Normal PDF and CDF
        constexpr double inv_sqrt_2pi = 0.3989422804014327; // 1 / sqrt(2 * pi)
        constexpr double inv_sqrt_2 = 0.7071067811865475;
        double pdf = inv_sqrt_2pi * std::exp(-0.5 * alpha * alpha);
        double cdf1 = 0.5 * (1.0 + std::erf(alpha * inv_sqrt_2));
        double cdf2 = 1.0 - cdf1;

        double mu_y = x1.mean * cdf1 + x2.mean * cdf2 + a * pdf;
        double second_moment = (x1.mean * x1.mean + var1) * cdf1 +
                              (x2.mean * x2.mean + var2) * cdf2 +
                              (x1.mean + x2.mean) * a * pdf;
        double var_y = std::max(0.0, second_moment - mu_y * mu_y);

        return {mu_y, std::sqrt(var_y)};
    }

    // Clark's statistical minimum operator for two Gaussian distributions
    static GaussianDelay clark_min(const GaussianDelay& x1, const GaussianDelay& x2) noexcept {
        // min(X, Y) = -max(-X, -Y)
        GaussianDelay neg_x1{-x1.mean, x1.sigma};
        GaussianDelay neg_x2{-x2.mean, x2.sigma};
        GaussianDelay neg_res = clark_max(neg_x1, neg_x2);
        return {-neg_res.mean, neg_res.sigma};
    }
};

// Process-Voltage-Temperature (PVT) Corner Definitions
enum class CornerType : uint8_t {
    TURBO_115V_125C = 0, // High voltage, high temp (max clock freq, max leakage)
    NOMINAL_080V_25C,    // Nominal voltage, room temp
    LOWPOWER_065V_M40C   // Low voltage, cold temp (hold-time critical / inversion)
};

struct PvtCorner {
    CornerType type{CornerType::NOMINAL_080V_25C};
    std::string name{"Nominal_0.8V_25C"};
    double voltage{0.80};       // Supply voltage in Volts
    double temperature{25.0};   // Temperature in Celsius
    double derate_late{1.00};   // Late delay scaling factor
    double derate_early{1.00};  // Early delay scaling factor
    double pocv_sensitivity{0.04}; // 4% nominal sigma / mean sensitivity
};

// Timing Arc Classifications
enum class TimingArcType : uint8_t {
    CELL_COMBINATIONAL = 0, // In -> Out combinational gate arc
    CELL_CLK_TO_Q,          // Clk -> Q sequential flip-flop arc
    CELL_SETUP_CHECK,       // Clk -> D setup time check arc
    CELL_HOLD_CHECK,        // Clk -> D hold time check arc
    NET_PROPAGATION         // Driver Pin -> Sink Pin interconnect arc
};

// Individual Timing Arc between two graph nodes
struct TimingArc {
    uint32_t arc_id{0};
    soc::db::PinId from_pin{soc::db::PinId::invalid()};
    soc::db::PinId to_pin{soc::db::PinId::invalid()};
    TimingArcType type{TimingArcType::CELL_COMBINATIONAL};

    // Delay models per PVT corner
    std::vector<GaussianDelay> corner_delays;
    std::vector<double> output_slews_ps;
};

// Node in the levelized Timing DAG (one per Pin in ChipDB)
struct TimingNode {
    soc::db::PinId pin_id{soc::db::PinId::invalid()};
    int32_t level{-1};
    bool is_clock_root{false};
    bool is_register_clock{false};
    bool is_endpoint{false};

    // Multi-corner arrival times (AT), required times (RT), and slacks
    std::vector<GaussianDelay> arrival_times;
    std::vector<GaussianDelay> required_times;
    std::vector<GaussianDelay> slacks;
    std::vector<double> slews_ps;

    std::vector<uint32_t> fanin_arcs;
    std::vector<uint32_t> fanout_arcs;
};

// Critical Timing Path
struct TimingPath {
    soc::db::PinId startpoint{soc::db::PinId::invalid()};
    soc::db::PinId endpoint{soc::db::PinId::invalid()};
    CornerType corner{CornerType::NOMINAL_080V_25C};
    GaussianDelay slack{0.0, 0.0};
    GaussianDelay total_delay{0.0, 0.0};
    std::vector<soc::db::PinId> path_pins;
    bool is_setup_violation{false};
};

// Slack Histogram for distribution reporting
struct SlackHistogram {
    double min_slack_ps{0.0};
    double max_slack_ps{0.0};
    double bin_width_ps{50.0};
    std::vector<uint32_t> bin_counts;
    uint32_t total_violating_endpoints{0};
    uint32_t total_endpoints{0};
    double worst_negative_slack_ps{0.0};
    double total_negative_slack_ps{0.0};
};

// Extracted Timing Model (ETM) for Hierarchical Block Abstracts
struct EtmPinModel {
    soc::db::StringId pin_name{soc::db::StringId::invalid()};
    bool is_input{true};
    bool is_clock{false};
    GaussianDelay setup_time{0.0, 0.0};
    GaussianDelay hold_time{0.0, 0.0};
    GaussianDelay clk_to_q{0.0, 0.0};
    double input_cap_ff{2.0};
    double max_slew_ps{50.0};
};

struct ExtractedTimingModel {
    soc::db::BlockAbstractId block_id{soc::db::BlockAbstractId::invalid()};
    std::string block_name;
    double core_frequency_ghz{3.5};
    double clock_period_ps{285.714}; // 3.5 GHz
    std::vector<EtmPinModel> boundary_pins;
    std::vector<TimingArc> feedthrough_arcs;
};

} // namespace vlsi::sta
