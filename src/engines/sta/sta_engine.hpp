#pragma once

#include "core/db/chip_db.hpp"
#include "core/db/observer.hpp"
#include "timing_types.hpp"
#include "rc_extractor.hpp"

#include <vector>
#include <unordered_map>
#include <memory>
#include <string_view>

namespace vlsi::sta {

struct StaConfig {
    double target_clock_freq_ghz{3.0}; // 3.0 GHz nominal
    double clock_uncertainty_ps{15.0}; // 15 ps clock jitter/skew margin
    bool enable_pocv{true};            // Enable Gaussian (mu, sigma) propagation
    bool verbose{false};
};

struct StaSummary {
    double wns_ps{0.0};                // Worst Negative Slack (3-sigma) in ps
    double tns_ps{0.0};                // Total Negative Slack in ps
    uint32_t violating_endpoints{0};   // Number of setup/hold violating endpoints
    uint32_t total_endpoints{0};       // Total timing check endpoints
    double max_frequency_ghz{0.0};     // Maximum achievable f_max across all corners
    double runtime_seconds{0.0};
};

class StaEngine : public soc::db::DatabaseObserver {
public:
    explicit StaEngine(soc::db::ChipDB& db, const StaConfig& config = {});
    ~StaEngine() override;

    // Build the timing graph from ChipDB masters, instances, pins, and nets
    void build_timing_graph();

    // Run Multi-Corner Multi-Mode levelized timing analysis with POCV statistical propagation
    StaSummary update_timing();

    // Query critical path for a specific corner
    [[nodiscard]] TimingPath get_critical_path(CornerType corner = CornerType::NOMINAL_080V_25C) const;

    // Compute slack histogram for viewport/inspector display
    [[nodiscard]] SlackHistogram compute_slack_histogram(CornerType corner = CornerType::NOMINAL_080V_25C) const;

    // Get slack for a specific pin
    [[nodiscard]] GaussianDelay get_pin_slack(soc::db::PinId pin_id, CornerType corner = CornerType::NOMINAL_080V_25C) const;

    // Extract boundary-only Extracted Timing Model (ETM) for a hierarchical block abstract
    [[nodiscard]] ExtractedTimingModel extract_timing_model(soc::db::BlockAbstractId block_id) const;

    // DatabaseObserver callbacks for zero-copy incremental dirty updates
    void on_instance_moved(soc::db::InstanceId id, soc::db::Point2D old_pos, soc::db::Point2D new_pos) override;
    void on_net_modified(soc::db::NetId id) override;
    void on_database_cleared() override;

    [[nodiscard]] const StaConfig& config() const noexcept { return m_config; }
    [[nodiscard]] const std::vector<PvtCorner>& corners() const noexcept { return m_corners; }

private:
    soc::db::ChipDB& m_db;
    StaConfig m_config;
    ParasiticRCExtractor m_rc_extractor;

    std::vector<PvtCorner> m_corners;
    std::vector<TimingNode> m_nodes; // Indexed by PinId::value()
    std::vector<TimingArc>  m_arcs;

    // Levelized topological buckets: m_level_nodes[level] = vector of PinIds
    std::vector<std::vector<soc::db::PinId>> m_level_nodes;

    // Startpoints (primary inputs, flip-flop Q pins) and Endpoints (primary outputs, flip-flop D pins)
    std::vector<soc::db::PinId> m_startpoints;
    std::vector<soc::db::PinId> m_endpoints;

    // Dirty tracking for incremental analysis
    bool m_graph_dirty{true};
    std::vector<bool> m_node_dirty;

    void init_corners();
    void levelize_graph();
    void propagate_forward_at();
    void propagate_backward_rt();

    // Delay calculation using CCS and NLDM interpolation
    [[nodiscard]] GaussianDelay compute_gate_delay(soc::db::InstanceId inst_id,
                                                   const TimingArc& arc,
                                                   double input_slew_ps,
                                                   double load_cap_ff,
                                                   const PvtCorner& corner) const;
};

} // namespace vlsi::sta
