#include "sta_engine.hpp"
#include <cmath>
#include <chrono>
#include <queue>
#include <algorithm>
#include <iostream>

namespace vlsi::sta {

StaEngine::StaEngine(soc::db::ChipDB& db, const StaConfig& config)
    : m_db(db), m_config(config), m_rc_extractor(db) {
    init_corners();
    m_db.observers.register_observer(this);
}

StaEngine::~StaEngine() {
    m_db.observers.unregister_observer(this);
}

void StaEngine::init_corners() {
    m_corners.clear();

    // 1. Turbo Corner (1.15V, 125C, 4.0 GHz signoff)
    PvtCorner c_turbo;
    c_turbo.type = CornerType::TURBO_115V_125C;
    c_turbo.name = "Turbo_1.15V_125C";
    c_turbo.voltage = 1.15;
    c_turbo.temperature = 125.0;
    c_turbo.derate_late = 0.75;
    c_turbo.derate_early = 0.70;
    c_turbo.pocv_sensitivity = 0.045; // 4.5% local Gaussian variation
    m_corners.push_back(c_turbo);

    // 2. Nominal Corner (0.80V, 25C, 3.0 GHz signoff)
    PvtCorner c_nom;
    c_nom.type = CornerType::NOMINAL_080V_25C;
    c_nom.name = "Nominal_0.8V_25C";
    c_nom.voltage = 0.80;
    c_nom.temperature = 25.0;
    c_nom.derate_late = 1.00;
    c_nom.derate_early = 0.95;
    c_nom.pocv_sensitivity = 0.040;
    m_corners.push_back(c_nom);

    // 3. LowPower Corner (0.65V, -40C, 2.0 GHz signoff)
    PvtCorner c_lp;
    c_lp.type = CornerType::LOWPOWER_065V_M40C;
    c_lp.name = "LowPower_0.65V_-40C";
    c_lp.voltage = 0.65;
    c_lp.temperature = -40.0;
    c_lp.derate_late = 1.50;
    c_lp.derate_early = 1.40;
    c_lp.pocv_sensitivity = 0.050;
    m_corners.push_back(c_lp);
}

void StaEngine::build_timing_graph() {
    m_nodes.clear();
    m_arcs.clear();
    m_startpoints.clear();
    m_endpoints.clear();

    size_t num_pins = m_db.pins.size();
    size_t num_corners = m_corners.size();

    m_nodes.resize(num_pins);
    m_node_dirty.assign(num_pins, true);

    for (size_t i = 0; i < num_pins; ++i) {
        soc::db::PinId pid{static_cast<uint32_t>(i)};
        m_nodes[i].pin_id = pid;
        m_nodes[i].arrival_times.assign(num_corners, GaussianDelay{0.0, 0.0});
        m_nodes[i].required_times.assign(num_corners, GaussianDelay{1e9, 0.0});
        m_nodes[i].slacks.assign(num_corners, GaussianDelay{0.0, 0.0});
        m_nodes[i].slews_ps.assign(num_corners, 25.0);
    }

    // 1. Identify Sequential Elements & Build Cell Internal Timing Arcs
    for (size_t i = 0; i < m_db.instances.size(); ++i) {
        soc::db::InstanceId inst_id{static_cast<uint32_t>(i)};
        const auto& inst = m_db.instances[inst_id];
        if (!inst.master_id.is_valid()) continue;

        const auto& master = m_db.master_cells[inst.master_id];
        std::string_view m_name = m_db.strings.resolve(master.name_id);

        // Find pins belonging to this instance
        std::vector<soc::db::PinId> inst_pins;
        for (size_t p = 0; p < m_db.pins.size(); ++p) {
            soc::db::PinId pid{static_cast<uint32_t>(p)};
            if (m_db.pins[pid].instance_id == inst_id) {
                inst_pins.push_back(pid);
            }
        }

        bool is_dff = (m_name.find("DFF") != std::string_view::npos ||
                       m_name.find("RETENTION") != std::string_view::npos);

        if (is_dff) {
            soc::db::PinId clk_pin{soc::db::PinId::invalid()};
            soc::db::PinId d_pin{soc::db::PinId::invalid()};
            soc::db::PinId q_pin{soc::db::PinId::invalid()};

            for (auto pid : inst_pins) {
                std::string_view pname = m_db.strings.resolve(m_db.pins[pid].name_id);
                if (pname == "CLK" || pname == "CK" || pname == "clk") clk_pin = pid;
                else if (pname == "D" || pname == "d") d_pin = pid;
                else if (pname == "Q" || pname == "q") q_pin = pid;
            }

            if (clk_pin.is_valid()) {
                m_nodes[clk_pin.value()].is_register_clock = true;
            }
            if (d_pin.is_valid()) {
                m_nodes[d_pin.value()].is_endpoint = true;
                m_endpoints.push_back(d_pin);
            }
            if (q_pin.is_valid()) {
                m_startpoints.push_back(q_pin);
            }

            // Sequential Clk -> Q Arc
            if (clk_pin.is_valid() && q_pin.is_valid()) {
                uint32_t arc_id = static_cast<uint32_t>(m_arcs.size());
                TimingArc arc;
                arc.arc_id = arc_id;
                arc.from_pin = clk_pin;
                arc.to_pin = q_pin;
                arc.type = TimingArcType::CELL_CLK_TO_Q;

                for (const auto& c : m_corners) {
                    double nom_delay = 45.0 * c.derate_late; // 45 ps nominal clk-to-q
                    double sigma = nom_delay * c.pocv_sensitivity;
                    arc.corner_delays.push_back(GaussianDelay{nom_delay, sigma});
                    arc.output_slews_ps.push_back(20.0 * c.derate_late);
                }

                m_arcs.push_back(arc);
                m_nodes[clk_pin.value()].fanout_arcs.push_back(arc_id);
                m_nodes[q_pin.value()].fanin_arcs.push_back(arc_id);
            }

            // Setup Check Constraint Arc
            if (clk_pin.is_valid() && d_pin.is_valid()) {
                uint32_t arc_id = static_cast<uint32_t>(m_arcs.size());
                TimingArc arc;
                arc.arc_id = arc_id;
                arc.from_pin = clk_pin;
                arc.to_pin = d_pin;
                arc.type = TimingArcType::CELL_SETUP_CHECK;

                for (const auto& c : m_corners) {
                    double nom_setup = 25.0 * c.derate_late; // 25 ps setup requirement
                    double sigma = nom_setup * c.pocv_sensitivity;
                    arc.corner_delays.push_back(GaussianDelay{nom_setup, sigma});
                    arc.output_slews_ps.push_back(20.0);
                }

                m_arcs.push_back(arc);
            }
        } else {
            // Combinational Gate Arcs (Input pins -> Output pins)
            soc::db::PinId out_pin{soc::db::PinId::invalid()};
            std::vector<soc::db::PinId> in_pins;

            for (auto pid : inst_pins) {
                std::string_view pname = m_db.strings.resolve(m_db.pins[pid].name_id);
                if (pname == "Y" || pname == "Z" || pname == "out" || pname == "OUT") {
                    out_pin = pid;
                } else {
                    in_pins.push_back(pid);
                }
            }

            if (out_pin.is_valid()) {
                for (auto in_p : in_pins) {
                    uint32_t arc_id = static_cast<uint32_t>(m_arcs.size());
                    TimingArc arc;
                    arc.arc_id = arc_id;
                    arc.from_pin = in_p;
                    arc.to_pin = out_pin;
                    arc.type = TimingArcType::CELL_COMBINATIONAL;

                    double base_gate_delay = 18.0; // 18 ps nominal for sub-3nm inverter/NAND
                    if (m_name.find("NAND") != std::string_view::npos) base_gate_delay = 24.0;
                    else if (m_name.find("NOR") != std::string_view::npos) base_gate_delay = 28.0;
                    else if (m_name.find("BUF") != std::string_view::npos) base_gate_delay = 20.0;
                    else if (m_name.find("LS") != std::string_view::npos)  base_gate_delay = 35.0; // Level Shifter

                    for (const auto& c : m_corners) {
                        double nom_d = base_gate_delay * c.derate_late;
                        double sigma = nom_d * c.pocv_sensitivity;
                        arc.corner_delays.push_back(GaussianDelay{nom_d, sigma});
                        arc.output_slews_ps.push_back(18.0 * c.derate_late);
                    }

                    m_arcs.push_back(arc);
                    m_nodes[in_p.value()].fanout_arcs.push_back(arc_id);
                    m_nodes[out_pin.value()].fanin_arcs.push_back(arc_id);
                }
            }
        }
    }

    // 2. Build Interconnect / Net Timing Arcs
    for (size_t net_idx = 0; net_idx < m_db.nets.size(); ++net_idx) {
        soc::db::NetId net_id{static_cast<uint32_t>(net_idx)};
        const auto& net = m_db.nets[net_id];
        if (net.pins.size() < 2) continue;

        soc::db::PinId drv_pin = net.pins.front();

        for (size_t p = 1; p < net.pins.size(); ++p) {
            soc::db::PinId sink_pin = net.pins[p];

            uint32_t arc_id = static_cast<uint32_t>(m_arcs.size());
            TimingArc arc;
            arc.arc_id = arc_id;
            arc.from_pin = drv_pin;
            arc.to_pin = sink_pin;
            arc.type = TimingArcType::NET_PROPAGATION;

            // Extract Elmore + Arnoldi RC delay
            PiModel pi = m_rc_extractor.extract_net_arc_rc(net_id, sink_pin, 25.0);
            double base_wire_delay_ps = std::max(1.0, pi.elmore_delay_ps);

            for (const auto& c : m_corners) {
                double nom_d = base_wire_delay_ps * c.derate_late;
                double sigma = nom_d * (c.pocv_sensitivity * 0.75); // wire variation slightly lower than gates
                arc.corner_delays.push_back(GaussianDelay{nom_d, sigma});
                arc.output_slews_ps.push_back(std::max(15.0, base_wire_delay_ps * 0.8));
            }

            m_arcs.push_back(arc);
            m_nodes[drv_pin.value()].fanout_arcs.push_back(arc_id);
            m_nodes[sink_pin.value()].fanin_arcs.push_back(arc_id);
        }
    }

    levelize_graph();
    m_graph_dirty = false;
}

void StaEngine::levelize_graph() {
    m_level_nodes.clear();
    size_t num_nodes = m_nodes.size();

    std::vector<int32_t> in_degree(num_nodes, 0);

    // Compute in-degrees considering only forward combinational and net propagation arcs
    for (size_t i = 0; i < num_nodes; ++i) {
        int32_t deg = 0;
        for (auto arc_id : m_nodes[i].fanin_arcs) {
            const auto& arc = m_arcs[arc_id];
            if (arc.type == TimingArcType::CELL_COMBINATIONAL ||
                arc.type == TimingArcType::NET_PROPAGATION) {
                deg++;
            }
        }
        in_degree[i] = deg;
    }

    // Queue nodes with 0 in-degree: startpoints (register Q), primary inputs, or isolated pins
    std::queue<soc::db::PinId> q;
    for (size_t i = 0; i < num_nodes; ++i) {
        if (in_degree[i] == 0) {
            m_nodes[i].level = 0;
            q.push(soc::db::PinId{static_cast<uint32_t>(i)});
        }
    }

    int32_t max_level = 0;

    while (!q.empty()) {
        soc::db::PinId curr = q.front();
        q.pop();

        int32_t curr_lvl = m_nodes[curr.value()].level;
        max_level = std::max(max_level, curr_lvl);

        if (static_cast<size_t>(curr_lvl) >= m_level_nodes.size()) {
            m_level_nodes.resize(curr_lvl + 1);
        }
        m_level_nodes[curr_lvl].push_back(curr);

        for (auto arc_id : m_nodes[curr.value()].fanout_arcs) {
            const auto& arc = m_arcs[arc_id];
            if (arc.type == TimingArcType::CELL_COMBINATIONAL ||
                arc.type == TimingArcType::NET_PROPAGATION) {
                uint32_t dst_idx = arc.to_pin.value();
                in_degree[dst_idx]--;
                m_nodes[dst_idx].level = std::max(m_nodes[dst_idx].level, curr_lvl + 1);
                if (in_degree[dst_idx] == 0) {
                    q.push(arc.to_pin);
                }
            }
        }
    }
}

void StaEngine::propagate_forward_at() {
    size_t num_corners = m_corners.size();

    // Reset arrival times across all nodes
    for (auto& node : m_nodes) {
        for (size_t c = 0; c < num_corners; ++c) {
            node.arrival_times[c] = GaussianDelay{0.0, 0.0};
        }
    }

    // 1. Initialize startpoints with clk-to-q delay
    for (auto sp_id : m_startpoints) {
        for (size_t c = 0; c < num_corners; ++c) {
            double nom_clk_to_q = 45.0 * m_corners[c].derate_late;
            double sigma = nom_clk_to_q * m_corners[c].pocv_sensitivity;
            m_nodes[sp_id.value()].arrival_times[c] = GaussianDelay{nom_clk_to_q, sigma};
        }
    }

    // 2. Propagate forward level by level
    for (const auto& lvl_pins : m_level_nodes) {
        for (auto pin_id : lvl_pins) {
            auto& node = m_nodes[pin_id.value()];

            for (auto arc_id : node.fanout_arcs) {
                const auto& arc = m_arcs[arc_id];
                if (arc.type != TimingArcType::CELL_COMBINATIONAL &&
                    arc.type != TimingArcType::NET_PROPAGATION) {
                    continue;
                }
                auto& to_node = m_nodes[arc.to_pin.value()];

                for (size_t c = 0; c < num_corners; ++c) {
                    GaussianDelay arc_d = arc.corner_delays[c];
                    GaussianDelay cand_at = node.arrival_times[c] + arc_d;

                    // Clark's statistical maximum for late arrival time
                    to_node.arrival_times[c] = GaussianDelay::clark_max(to_node.arrival_times[c], cand_at);
                    to_node.slews_ps[c] = arc.output_slews_ps[c];
                }
            }
        }
    }
}

void StaEngine::propagate_backward_rt() {
    size_t num_corners = m_corners.size();

    // Reset required times across all nodes to infinity
    for (auto& node : m_nodes) {
        for (size_t c = 0; c < num_corners; ++c) {
            node.required_times[c] = GaussianDelay{1e9, 0.0};
        }
    }

    // 1. Initialize required times at endpoints based on corner clock period
    for (size_t c = 0; c < num_corners; ++c) {
        // Clock period in ps (e.g. 250 ps for Turbo 4 GHz, 333.3 ps for Nominal 3 GHz, 500 ps for LP 2 GHz)
        double f_ghz = m_config.target_clock_freq_ghz;
        if (m_corners[c].type == CornerType::TURBO_115V_125C) f_ghz = 4.0;
        else if (m_corners[c].type == CornerType::LOWPOWER_065V_M40C) f_ghz = 2.0;

        double t_clk_ps = 1000.0 / f_ghz;
        double uncertainty = m_config.clock_uncertainty_ps;

        for (auto ep_id : m_endpoints) {
            // Nominal setup time 25 ps
            double t_setup = 25.0 * m_corners[c].derate_late;
            double rt_nom = t_clk_ps - uncertainty - t_setup;
            double rt_sigma = t_setup * m_corners[c].pocv_sensitivity;

            m_nodes[ep_id.value()].required_times[c] = GaussianDelay{rt_nom, rt_sigma};
        }
    }

    // 2. Propagate backward in reverse topological order
    for (auto it = m_level_nodes.rbegin(); it != m_level_nodes.rend(); ++it) {
        for (auto pin_id : *it) {
            const auto& node = m_nodes[pin_id.value()];

            for (auto arc_id : node.fanin_arcs) {
                const auto& arc = m_arcs[arc_id];
                if (arc.type != TimingArcType::CELL_COMBINATIONAL &&
                    arc.type != TimingArcType::NET_PROPAGATION) {
                    continue;
                }
                auto& from_node = m_nodes[arc.from_pin.value()];

                for (size_t c = 0; c < num_corners; ++c) {
                    GaussianDelay arc_d = arc.corner_delays[c];
                    GaussianDelay cand_rt = node.required_times[c] - arc_d;

                    from_node.required_times[c] = GaussianDelay::clark_min(from_node.required_times[c], cand_rt);
                }
            }
        }
    }

    // 3. Compute slacks at all nodes across all corners
    for (auto& node : m_nodes) {
        for (size_t c = 0; c < num_corners; ++c) {
            if (node.required_times[c].mean > 1e8) {
                node.required_times[c] = node.arrival_times[c];
            }
            node.slacks[c] = node.required_times[c] - node.arrival_times[c];
        }
    }
}

StaSummary StaEngine::update_timing() {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (m_graph_dirty || m_nodes.empty()) {
        build_timing_graph();
    }

    propagate_forward_at();
    propagate_backward_rt();

    StaSummary summary;
    summary.total_endpoints = static_cast<uint32_t>(m_endpoints.size());

    // Worst Negative Slack (WNS) across all corners using 3-sigma yield slack (mu - 3 * sigma)
    double worst_wns = 1e9;
    double total_tns = 0.0;
    uint32_t total_viols = 0;

    for (size_t c = 0; c < m_corners.size(); ++c) {
        for (auto ep_id : m_endpoints) {
            const auto& slack_g = m_nodes[ep_id.value()].slacks[c];
            double slack_3sigma = slack_g.late_3sigma() - slack_g.mean * 2.0; // mu - 3 * sigma
            slack_3sigma = slack_g.mean - 3.0 * slack_g.sigma;

            if (slack_3sigma < 0.0) {
                total_tns += slack_3sigma;
                total_viols++;
                worst_wns = std::min(worst_wns, slack_3sigma);
            }
        }
    }

    summary.wns_ps = (worst_wns < 1e8) ? worst_wns : 0.0;
    summary.tns_ps = total_tns;
    summary.violating_endpoints = total_viols;

    // Achievable f_max calculation: T_clk_eff = T_clk - WNS
    double t_clk_nom = 1000.0 / m_config.target_clock_freq_ghz;
    double t_min_period = t_clk_nom - summary.wns_ps;
    summary.max_frequency_ghz = (t_min_period > 0.0) ? (1000.0 / t_min_period) : 0.0;

    auto end_time = std::chrono::high_resolution_clock::now();
    summary.runtime_seconds = std::chrono::duration<double>(end_time - start_time).count();

    return summary;
}

TimingPath StaEngine::get_critical_path(CornerType corner) const {
    TimingPath path;
    path.corner = corner;
    size_t corner_idx = static_cast<size_t>(corner);
    if (corner_idx >= m_corners.size()) corner_idx = 0;

    if (m_endpoints.empty()) return path;

    // 1. Locate worst endpoint (minimum 3-sigma slack)
    soc::db::PinId worst_ep{soc::db::PinId::invalid()};
    double worst_slack = 1e18;

    for (auto ep_id : m_endpoints) {
        const auto& s = m_nodes[ep_id.value()].slacks[corner_idx];
        double s_val = s.mean - 3.0 * s.sigma;
        if (s_val < worst_slack) {
            worst_slack = s_val;
            worst_ep = ep_id;
        }
    }

    if (!worst_ep.is_valid()) return path;

    path.endpoint = worst_ep;
    path.slack = m_nodes[worst_ep.value()].slacks[corner_idx];
    path.is_setup_violation = (worst_slack < 0.0);

    // 2. Trace backwards from endpoint to startpoint along worst fanin arc
    soc::db::PinId curr = worst_ep;
    path.path_pins.push_back(curr);

    const int max_trace_depth = 500;
    int depth = 0;

    while (depth++ < max_trace_depth) {
        const auto& node = m_nodes[curr.value()];
        if (node.fanin_arcs.empty()) break;

        // Pick fanin arc that contributed maximum arrival time
        uint32_t best_arc = 0xFFFFFFFF;
        double max_at = -1e18;

        for (auto arc_id : node.fanin_arcs) {
            const auto& arc = m_arcs[arc_id];
            const auto& from_node = m_nodes[arc.from_pin.value()];
            double at_val = from_node.arrival_times[corner_idx].mean + arc.corner_delays[corner_idx].mean;
            if (at_val > max_at) {
                max_at = at_val;
                best_arc = arc_id;
            }
        }

        if (best_arc == 0xFFFFFFFF) break;

        soc::db::PinId prev_pin = m_arcs[best_arc].from_pin;
        path.path_pins.push_back(prev_pin);
        curr = prev_pin;

        // If arrived at sequential startpoint
        if (std::find(m_startpoints.begin(), m_startpoints.end(), curr) != m_startpoints.end()) {
            path.startpoint = curr;
            break;
        }
    }

    std::reverse(path.path_pins.begin(), path.path_pins.end());
    path.total_delay = m_nodes[worst_ep.value()].arrival_times[corner_idx];
    return path;
}

SlackHistogram StaEngine::compute_slack_histogram(CornerType corner) const {
    SlackHistogram hist;
    size_t corner_idx = static_cast<size_t>(corner);
    if (corner_idx >= m_corners.size()) corner_idx = 0;

    hist.total_endpoints = static_cast<uint32_t>(m_endpoints.size());
    if (m_endpoints.empty()) return hist;

    double min_s = 1e18, max_s = -1e18;
    for (auto ep_id : m_endpoints) {
        double s = m_nodes[ep_id.value()].slacks[corner_idx].mean - 3.0 * m_nodes[ep_id.value()].slacks[corner_idx].sigma;
        min_s = std::min(min_s, s);
        max_s = std::max(max_s, s);
        if (s < 0.0) {
            hist.total_violating_endpoints++;
            hist.total_negative_slack_ps += s;
        }
    }

    hist.min_slack_ps = min_s;
    hist.max_slack_ps = max_s;
    hist.worst_negative_slack_ps = (min_s < 0.0) ? min_s : 0.0;

    hist.bin_width_ps = std::max(10.0, (max_s - min_s) / 20.0);
    hist.bin_counts.assign(20, 0);

    for (auto ep_id : m_endpoints) {
        double s = m_nodes[ep_id.value()].slacks[corner_idx].mean - 3.0 * m_nodes[ep_id.value()].slacks[corner_idx].sigma;
        int bin = std::clamp(static_cast<int>((s - min_s) / hist.bin_width_ps), 0, 19);
        hist.bin_counts[bin]++;
    }

    return hist;
}

GaussianDelay StaEngine::get_pin_slack(soc::db::PinId pin_id, CornerType corner) const {
    if (pin_id.is_valid() && pin_id.value() < m_nodes.size()) {
        size_t c_idx = static_cast<size_t>(corner);
        if (c_idx < m_corners.size()) {
            return m_nodes[pin_id.value()].slacks[c_idx];
        }
    }
    return {0.0, 0.0};
}

ExtractedTimingModel StaEngine::extract_timing_model(soc::db::BlockAbstractId block_id) const {
    ExtractedTimingModel etm;
    etm.block_id = block_id;

    if (!block_id.is_valid() || block_id.value() >= m_db.block_abstracts.size()) {
        return etm;
    }

    const auto& block = m_db.block_abstracts[block_id];
    etm.block_name = std::string(m_db.strings.resolve(block.name_id));
    etm.clock_period_ps = 250.0; // 4 GHz signoff

    // Extract pin models for boundary pins
    for (auto pin_id : block.boundary_pins) {
        if (!pin_id.is_valid() || pin_id.value() >= m_nodes.size()) continue;

        const auto& pin = m_db.pins[pin_id];
        const auto& node = m_nodes[pin_id.value()];

        EtmPinModel pm;
        pm.pin_name = pin.name_id;

        // If pin has fanout inside block -> Input pin with setup/hold requirements
        if (!node.fanout_arcs.empty()) {
            pm.is_input = true;
            pm.setup_time = GaussianDelay{35.0, 1.5}; // 35 ps input setup
            pm.hold_time  = GaussianDelay{10.0, 0.8}; // 10 ps hold
            pm.input_cap_ff = 2.5;
        } else {
            // Output pin -> Clock-to-Q delay
            pm.is_input = false;
            pm.clk_to_q = node.arrival_times[0]; // Arrival from internal clock
            if (pm.clk_to_q.mean < 1.0) pm.clk_to_q = GaussianDelay{65.0, 2.8};
            pm.max_slew_ps = 35.0;
        }

        etm.boundary_pins.push_back(pm);
    }

    return etm;
}

void StaEngine::on_instance_moved(soc::db::InstanceId id, soc::db::Point2D /*old_pos*/, soc::db::Point2D /*new_pos*/) {
    if (!id.is_valid()) return;
    // Invalidate affected timing nodes for incremental analysis
    m_graph_dirty = true;
}

void StaEngine::on_net_modified(soc::db::NetId id) {
    if (!id.is_valid()) return;
    m_graph_dirty = true;
}

void StaEngine::on_database_cleared() {
    m_nodes.clear();
    m_arcs.clear();
    m_graph_dirty = true;
}

} // namespace vlsi::sta
