#include "lvs_engine.hpp"
#include <chrono>
#include <unordered_set>
#include <algorithm>
#include <iostream>

namespace vlsi::drc_lvs {

LVSEngine::LVSEngine(soc::db::ChipDB& db) : m_db(db) {}

LVSReport LVSEngine::run() {
    auto t0 = std::chrono::steady_clock::now();
    LVSReport report;

    // 1. Verify instances
    for (size_t i = 0; i < m_db.instances.size(); ++i) {
        soc::db::InstanceId iid{static_cast<uint32_t>(i)};
        const auto& inst = m_db.instances[iid];

        if (inst.master_id.is_valid() && inst.master_id.value() < m_db.master_cells.size()) {
            report.matched_instances++;
        } else {
            report.mismatched_instances++;
            report.is_lvs_match = false;
            report.error_messages.push_back("Instance " + std::to_string(i) + " has invalid master cell reference");
        }
    }

    // 2. Verify pins and net connectivity isomorphism
    for (size_t n = 0; n < m_db.nets.size(); ++n) {
        soc::db::NetId nid{static_cast<uint32_t>(n)};
        const auto& net = m_db.nets[nid];

        bool net_valid = true;

        for (soc::db::PinId pid : net.pins) {
            if (!pid.is_valid() || pid.value() >= m_db.pins.size()) {
                net_valid = false;
                report.mismatched_pins++;
                report.error_messages.push_back("Net " + std::to_string(n) + " references invalid pin ID");
                continue;
            }

            const auto& pin = m_db.pins[pid];
            if (pin.net_id != nid) {
                net_valid = false;
                report.mismatched_pins++;
                report.error_messages.push_back("Pin " + std::to_string(pid.value()) + " net_id mismatch with net " + std::to_string(n));
            } else {
                report.matched_pins++;
            }
        }

        if (net_valid) {
            report.matched_nets++;
        } else {
            report.mismatched_nets++;
            report.is_lvs_match = false;
        }
    }

    auto t1 = std::chrono::steady_clock::now();
    report.runtime_seconds = std::chrono::duration<double>(t1 - t0).count();

    return report;
}

} // namespace vlsi::drc_lvs
