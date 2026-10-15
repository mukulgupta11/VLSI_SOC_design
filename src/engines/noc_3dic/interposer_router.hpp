#pragma once

#include "noc_types.hpp"
#include "core/db/chip_db.hpp"
#include <vector>

namespace vlsi::noc {

class InterposerRouter {
public:
    explicit InterposerRouter(soc::db::ChipDB& db, const NoCConfig& config = {});

    // Generate UltraFusion-style micro-bump array and route length-matched RDL escape traces
    NoCStats route_interposer();

    [[nodiscard]] const std::vector<MicroBump>& microbumps() const noexcept { return m_microbumps; }
    [[nodiscard]] const NoCConfig& config() const noexcept { return m_config; }

private:
    soc::db::ChipDB& m_db;
    NoCConfig m_config;
    std::vector<MicroBump> m_microbumps;

    void generate_microbump_array(int32_t bump_count);
    void route_length_matched_rdl(NoCStats& stats);
};

} // namespace vlsi::noc
