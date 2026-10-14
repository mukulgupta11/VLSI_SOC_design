#include "upf_engine.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <iostream>

namespace vlsi::upf {

UpfEngine::UpfEngine(soc::db::ChipDB& db, UpfConfig config)
    : m_db(db), m_config(config) {}

soc::db::MasterCellId UpfEngine::get_or_create_master(std::string_view name,
                                                     soc::db::CellType type,
                                                     int32_t width, int32_t height) {
    for (size_t i = 0; i < m_db.master_cells.size(); ++i) {
        soc::db::MasterCellId id{static_cast<uint32_t>(i)};
        if (m_db.strings.resolve(m_db.master_cells[id].name_id) == name) {
            return id;
        }
    }

    soc::db::MasterCellId master_id = m_db.create_master_cell(name, width, height, type);
    auto& master = m_db.master_cells[master_id];

    // Create standard ports for UPF cells
    soc::db::LayerId m1{1};
    if (type == soc::db::CellType::LEVEL_SHIFTER) {
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("A"), soc::db::SignalType::SIGNAL, m1,
                                                   soc::db::Rect2D{20, 40, 80, 100}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("Y"), soc::db::SignalType::SIGNAL, m1,
                                                   soc::db::Rect2D{width - 80, 40, width - 20, 100}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("VDD_L"), soc::db::SignalType::POWER, m1,
                                                   soc::db::Rect2D{0, height - 40, width / 2, height}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("VDD_H"), soc::db::SignalType::POWER, m1,
                                                   soc::db::Rect2D{width / 2, height - 40, width, height}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("VSS"), soc::db::SignalType::GROUND, m1,
                                                   soc::db::Rect2D{0, 0, width, 40}});
    } else if (type == soc::db::CellType::ISOLATION_CELL) {
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("A"), soc::db::SignalType::SIGNAL, m1,
                                                   soc::db::Rect2D{20, 40, 80, 100}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("EN"), soc::db::SignalType::SIGNAL, m1,
                                                   soc::db::Rect2D{100, 40, 160, 100}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("Y"), soc::db::SignalType::SIGNAL, m1,
                                                   soc::db::Rect2D{width - 80, 40, width - 20, 100}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("VDD"), soc::db::SignalType::POWER, m1,
                                                   soc::db::Rect2D{0, height - 40, width, height}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("VSS"), soc::db::SignalType::GROUND, m1,
                                                   soc::db::Rect2D{0, 0, width, 40}});
    } else if (type == soc::db::CellType::POWER_SWITCH) {
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("SLEEP_EN"), soc::db::SignalType::SIGNAL, m1,
                                                   soc::db::Rect2D{20, 40, 80, 100}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("VDD_IN"), soc::db::SignalType::POWER, m1,
                                                   soc::db::Rect2D{0, height - 40, width, height}});
        master.ports.push_back(soc::db::MasterPort{m_db.strings.intern("VDD_OUT"), soc::db::SignalType::POWER, m1,
                                                   soc::db::Rect2D{0, 0, width, 40}});
    }

    return master_id;
}

void UpfEngine::configure_power_domains() {
    // If power domains already created in ChipDB, identify them; otherwise create standard domains
    soc::db::PowerDomainId dom_cpu{soc::db::PowerDomainId::invalid()};
    soc::db::PowerDomainId dom_gpu{soc::db::PowerDomainId::invalid()};
    soc::db::PowerDomainId dom_soc{soc::db::PowerDomainId::invalid()};

    for (size_t i = 0; i < m_db.power_domains.size(); ++i) {
        soc::db::PowerDomainId id{static_cast<uint32_t>(i)};
        std::string_view name = m_db.strings.resolve(m_db.power_domains[id].name_id);
        if (name == "VDD_CPU") {
            dom_cpu = id;
            m_db.power_domains[id].nominal_voltage = m_config.cpu_voltage;
        } else if (name == "VDD_GPU") {
            dom_gpu = id;
            m_db.power_domains[id].nominal_voltage = m_config.gpu_voltage;
            m_switchable_domains.insert(id.value());
        } else if (name == "VDD_SOC") {
            dom_soc = id;
            m_db.power_domains[id].nominal_voltage = m_config.soc_voltage;
        }
    }

    int32_t w = m_db.die_area.width();
    int32_t h = m_db.die_area.height();
    if (w == 0) w = 10000000;
    if (h == 0) h = 10000000;

    if (!dom_cpu.is_valid()) {
        dom_cpu = m_db.create_power_domain("VDD_CPU", m_config.cpu_voltage,
                                           soc::db::Rect2D{0, h / 2, w / 2, h});
    }
    if (!dom_gpu.is_valid()) {
        dom_gpu = m_db.create_power_domain("VDD_GPU", m_config.gpu_voltage,
                                           soc::db::Rect2D{0, 0, w, h / 2});
        m_switchable_domains.insert(dom_gpu.value());
    }
    if (!dom_soc.is_valid()) {
        dom_soc = m_db.create_power_domain("VDD_SOC", m_config.soc_voltage,
                                           soc::db::Rect2D{w / 2, h / 2, w, h});
    }

    // Ensure Master Cells exist for Level Shifters and Isolation cells
    get_or_create_master("LS_LOW_TO_HIGH", soc::db::CellType::LEVEL_SHIFTER, 480, 240);
    get_or_create_master("LS_HIGH_TO_LOW", soc::db::CellType::LEVEL_SHIFTER, 480, 240);
    get_or_create_master("ISO_CLAMP_LOW", soc::db::CellType::ISOLATION_CELL, 360, 240);
    get_or_create_master("HEADER_SWITCH_X1", soc::db::CellType::POWER_SWITCH,
                         m_config.switch_width, m_config.switch_height);
}

UpfStats UpfEngine::insert_boundary_cells() {
    auto start_time = std::chrono::high_resolution_clock::now();
    UpfStats stats;

    configure_power_domains();

    soc::db::MasterCellId m_ls_lh = get_or_create_master("LS_LOW_TO_HIGH", soc::db::CellType::LEVEL_SHIFTER, 480, 240);
    soc::db::MasterCellId m_ls_hl = get_or_create_master("LS_HIGH_TO_LOW", soc::db::CellType::LEVEL_SHIFTER, 480, 240);
    soc::db::MasterCellId m_iso   = get_or_create_master("ISO_CLAMP_LOW", soc::db::CellType::ISOLATION_CELL, 360, 240);

    // Create or find global isolation enable net
    soc::db::NetId iso_en_net = m_db.create_net("gpu_iso_en", soc::db::SignalType::SIGNAL);

    size_t initial_net_count = m_db.nets.size();

    for (size_t net_idx = 0; net_idx < initial_net_count; ++net_idx) {
        soc::db::NetId net_id{static_cast<uint32_t>(net_idx)};
        auto& net = m_db.nets[net_id];
        if (net.pins.size() < 2) continue;

        // Find driver pin (first pin by convention or output pin)
        soc::db::PinId drv_pin_id = net.pins.front();
        const auto& drv_pin = m_db.pins[drv_pin_id];
        if (!drv_pin.instance_id.is_valid()) continue;

        const auto& drv_inst = m_db.instances[drv_pin.instance_id];
        soc::db::PowerDomainId drv_domain_id = drv_inst.power_domain_id;
        if (!drv_domain_id.is_valid()) continue;

        double v_drv = m_db.power_domains[drv_domain_id].nominal_voltage;
        bool drv_is_switchable = (m_switchable_domains.find(drv_domain_id.value()) != m_switchable_domains.end());

        // Check sink pins for voltage / domain crossings
        std::vector<soc::db::PinId> crossing_sink_pins;
        for (size_t p = 1; p < net.pins.size(); ++p) {
            soc::db::PinId sink_pin_id = net.pins[p];
            const auto& sink_pin = m_db.pins[sink_pin_id];
            if (!sink_pin.instance_id.is_valid()) continue;

            const auto& sink_inst = m_db.instances[sink_pin.instance_id];
            soc::db::PowerDomainId sink_domain_id = sink_inst.power_domain_id;
            if (!sink_domain_id.is_valid()) continue;

            if (sink_domain_id != drv_domain_id) {
                crossing_sink_pins.push_back(sink_pin_id);
            }
        }

        if (crossing_sink_pins.empty()) continue;

        stats.boundary_nets_scanned++;

        // For each crossing sink group in a distinct destination domain
        for (auto sink_pin_id : crossing_sink_pins) {
            auto& sink_pin = m_db.pins[sink_pin_id];
            const auto& sink_inst = m_db.instances[sink_pin.instance_id];
            soc::db::PowerDomainId sink_domain_id = sink_inst.power_domain_id;
            double v_sink = m_db.power_domains[sink_domain_id].nominal_voltage;
            bool sink_is_switchable = (m_switchable_domains.find(sink_domain_id.value()) != m_switchable_domains.end());

            soc::db::NetId current_driver_net = net_id;

            // 1. Isolation Cell needed if driver is power-gated/switchable and sink is active
            if (drv_is_switchable && !sink_is_switchable) {
                std::string iso_inst_name = "ISO_" + std::string(m_db.strings.resolve(net.name_id)) +
                                           "_" + std::to_string(stats.isolation_cells_inserted);

                soc::db::Point2D iso_pos = sink_inst.origin;
                iso_pos.x -= 480; // place slightly before sink

                soc::db::InstanceId iso_inst = m_db.create_instance(
                    iso_inst_name, m_iso, iso_pos, soc::db::Orientation::R0, sink_domain_id);

                // Create intermediate isolated net
                std::string iso_net_name = "net_iso_" + std::to_string(stats.isolation_cells_inserted);
                soc::db::NetId iso_out_net = m_db.create_net(iso_net_name, soc::db::SignalType::SIGNAL, sink_domain_id);

                // Connect ISO pins
                soc::db::LayerId m1{1};
                m_db.create_pin("A", iso_inst, current_driver_net, m1, soc::db::Rect2D{20, 40, 80, 100});
                m_db.create_pin("EN", iso_inst, iso_en_net, m1, soc::db::Rect2D{100, 40, 160, 100});
                m_db.create_pin("Y", iso_inst, iso_out_net, m1, soc::db::Rect2D{280, 40, 340, 100});

                current_driver_net = iso_out_net;
                stats.isolation_cells_inserted++;
            }

            // 2. Level Shifter needed if voltage differential > 0.05 V
            if (std::abs(v_drv - v_sink) > 0.05) {
                bool is_lh = (v_drv < v_sink);
                soc::db::MasterCellId ls_master = is_lh ? m_ls_lh : m_ls_hl;

                std::string ls_inst_name = (is_lh ? "LS_LH_" : "LS_HL_") +
                                           std::string(m_db.strings.resolve(net.name_id)) +
                                           "_" + std::to_string(stats.level_shifters_inserted);

                soc::db::Point2D ls_pos = sink_inst.origin;
                ls_pos.x -= 240;

                soc::db::InstanceId ls_inst = m_db.create_instance(
                    ls_inst_name, ls_master, ls_pos, soc::db::Orientation::R0, sink_domain_id);

                // Create post-level-shifter net driving the destination sink
                std::string ls_out_net_name = "net_ls_" + std::to_string(stats.level_shifters_inserted);
                soc::db::NetId ls_out_net = m_db.create_net(ls_out_net_name, soc::db::SignalType::SIGNAL, sink_domain_id);

                // Connect LS pins
                soc::db::LayerId m1{1};
                m_db.create_pin("A", ls_inst, current_driver_net, m1, soc::db::Rect2D{20, 40, 80, 100});
                m_db.create_pin("Y", ls_inst, ls_out_net, m1, soc::db::Rect2D{400, 40, 460, 100});

                // Rewire destination sink pin to post-LS net
                sink_pin.net_id = ls_out_net;
                m_db.nets[ls_out_net].pins.push_back(sink_pin_id);

                // Remove sink pin from original net
                auto& old_pins = m_db.nets[net_id].pins;
                old_pins.erase(std::remove(old_pins.begin(), old_pins.end(), sink_pin_id), old_pins.end());

                stats.level_shifters_inserted++;
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    stats.runtime_seconds = std::chrono::duration<double>(end_time - start_time).count();
    return stats;
}

uint32_t UpfEngine::insert_header_switch_grid(soc::db::PowerDomainId domain_id,
                                              std::string_view sleep_ctrl_net_name) {
    if (!domain_id.is_valid()) return 0;
    auto& domain = m_db.power_domains[domain_id];

    soc::db::MasterCellId m_switch = get_or_create_master("HEADER_SWITCH_X1",
                                                         soc::db::CellType::POWER_SWITCH,
                                                         m_config.switch_width,
                                                         m_config.switch_height);

    soc::db::NetId sleep_net = m_db.create_net(sleep_ctrl_net_name, soc::db::SignalType::SIGNAL, domain_id);

    int32_t cols = m_config.switch_grid_cols;
    int32_t rows = m_config.switch_grid_rows;
    if (cols <= 0) cols = 8;
    if (rows <= 0) rows = 8;

    int32_t x_step = domain.boundary.width() / (cols + 1);
    int32_t y_step = domain.boundary.height() / (rows + 1);

    uint32_t switches_placed = 0;
    soc::db::LayerId m1{1};

    for (int32_t r = 1; r <= rows; ++r) {
        for (int32_t c = 1; c <= cols; ++c) {
            int32_t x = domain.boundary.x_min + c * x_step;
            int32_t y = domain.boundary.y_min + r * y_step;

            std::string name = "PWR_SW_" + std::to_string(switches_placed);
            soc::db::InstanceId sw_inst = m_db.create_instance(
                name, m_switch, soc::db::Point2D{x, y}, soc::db::Orientation::R0, domain_id);

            // Connect sleep enable pin
            m_db.create_pin("SLEEP_EN", sw_inst, sleep_net, m1, soc::db::Rect2D{20, 40, 80, 100});
            switches_placed++;
        }
    }

    return switches_placed;
}

bool UpfEngine::verify_power_intent() const {
    // Verify that every cross-domain net with a voltage differential has a level shifter,
    // and every switchable-to-active crossing has an isolation cell.
    for (size_t net_idx = 0; net_idx < m_db.nets.size(); ++net_idx) {
        soc::db::NetId net_id{static_cast<uint32_t>(net_idx)};
        const auto& net = m_db.nets[net_id];
        if (net.pins.size() < 2) continue;

        soc::db::PinId drv_pin_id = net.pins.front();
        const auto& drv_pin = m_db.pins[drv_pin_id];
        if (!drv_pin.instance_id.is_valid()) continue;

        const auto& drv_inst = m_db.instances[drv_pin.instance_id];
        soc::db::PowerDomainId drv_domain_id = drv_inst.power_domain_id;
        if (!drv_domain_id.is_valid()) continue;

        double v_drv = m_db.power_domains[drv_domain_id].nominal_voltage;
        bool drv_is_switchable = (m_switchable_domains.find(drv_domain_id.value()) != m_switchable_domains.end());

        const auto& drv_master = m_db.master_cells[drv_inst.master_id];
        bool drv_is_upf_cell = (drv_master.cell_type == soc::db::CellType::LEVEL_SHIFTER ||
                                drv_master.cell_type == soc::db::CellType::ISOLATION_CELL);

        for (size_t p = 1; p < net.pins.size(); ++p) {
            soc::db::PinId sink_pin_id = net.pins[p];
            const auto& sink_pin = m_db.pins[sink_pin_id];
            if (!sink_pin.instance_id.is_valid()) continue;

            const auto& sink_inst = m_db.instances[sink_pin.instance_id];
            soc::db::PowerDomainId sink_domain_id = sink_inst.power_domain_id;
            if (!sink_domain_id.is_valid()) continue;

            const auto& sink_master = m_db.master_cells[sink_inst.master_id];
            bool sink_is_upf_cell = (sink_master.cell_type == soc::db::CellType::LEVEL_SHIFTER ||
                                     sink_master.cell_type == soc::db::CellType::ISOLATION_CELL);

            if (sink_domain_id != drv_domain_id && !drv_is_upf_cell && !sink_is_upf_cell) {
                double v_sink = m_db.power_domains[sink_domain_id].nominal_voltage;
                bool sink_is_switchable = (m_switchable_domains.find(sink_domain_id.value()) != m_switchable_domains.end());

                if (std::abs(v_drv - v_sink) > 0.05) {
                    return false; // Unshielded voltage differential
                }
                if (drv_is_switchable && !sink_is_switchable) {
                    return false; // Un-isolated power-gated crossing
                }
            }
        }
    }
    return true;
}

} // namespace vlsi::upf
