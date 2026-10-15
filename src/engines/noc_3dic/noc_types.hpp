#pragma once

#include "core/db/chip_db.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace vlsi::noc {

enum class NoCNodeType : uint8_t {
    CPU_PCORE = 0,
    CPU_ECORE,
    GPU,
    NPU,
    SLC,
    MEMORY_CTRL,
    PCIE,
    ROUTER
};

struct NoCNode {
    uint32_t id{0};
    std::string name;
    NoCNodeType type{NoCNodeType::ROUTER};
    soc::db::Rect2D bbox;
    soc::db::Point2D port_pos{0, 0};
    soc::db::PowerDomainId power_domain_id{soc::db::PowerDomainId::invalid()};
    double max_bandwidth_gbps{256.0};
};

struct NoCLink {
    uint32_t id{0};
    uint32_t src_node_id{0};
    uint32_t dst_node_id{0};
    uint32_t bit_width{256}; // 256-bit wide flit bus
    double length_um{0.0};
    double wire_flight_delay_ps{0.0};
    uint32_t repeater_stages{0};
    std::vector<soc::db::InstanceId> inserted_repeaters;
    std::vector<soc::db::NetId> link_nets;
};

struct MicroBump {
    uint32_t id{0};
    soc::db::Point2D pos{0, 0};
    int32_t pitch_um{25}; // 25 um micro-bump pitch
    int32_t diameter_um{12};
    soc::db::NetId net_id{soc::db::NetId::invalid()};
    std::string signal_name;
    double escape_wirelength_um{0.0};
    double rdl_skew_ps{0.0};
};

struct ThermalCell {
    double temperature_celsius{25.0};
    double power_density_w_per_mm2{0.0};
};

struct IRDropNode {
    soc::db::Point2D pos{0, 0};
    double nominal_voltage{0.8};
    double actual_voltage{0.8};
    double ir_drop_mv{0.0};
};

struct NoCConfig {
    double target_freq_ghz{4.0};         // 4.0 GHz target operating frequency
    double clock_period_ps{250.0};        // 250 ps clock cycle
    double max_unpipelined_delay_ps{180.0}; // Max delay before inserting pipeline repeater register
    double unit_r_ohm_per_um{0.3};        // Distributed wire resistance per um
    double unit_c_ff_per_um{0.15};        // Distributed wire capacitance per um
    double repeater_delay_ps{25.0};       // DFF_X1 clock-to-Q + setup delay
    int32_t microbump_pitch_um{25};       // UltraFusion microbump pitch
    double max_rdl_skew_ps{2.0};          // Sub-2ps length-matched skew budget
    double max_ir_drop_mv{35.0};          // 35 mV max allowable IR-drop
    double max_temp_celsius{105.0};       // 105 C Apple thermal throttling boundary
};

struct NoCStats {
    uint32_t total_nodes{0};
    uint32_t total_links{0};
    uint32_t total_repeaters_inserted{0};
    double total_bandwidth_tbps{0.0};
    double max_wire_delay_ps{0.0};
    double peak_temp_c{0.0};
    double worst_ir_drop_mv{0.0};
    uint32_t microbumps_routed{0};
    double max_rdl_skew_ps{0.0};
    double runtime_seconds{0.0};
};

} // namespace vlsi::noc
