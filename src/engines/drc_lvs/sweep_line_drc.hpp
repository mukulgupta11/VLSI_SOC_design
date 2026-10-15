#pragma once

#include "drc_types.hpp"
#include "core/db/chip_db.hpp"
#include <vector>

namespace vlsi::drc_lvs {

class SweepLineDRC {
public:
    explicit SweepLineDRC(soc::db::ChipDB& db);

    // Run 2D sweep-line Design Rule Checking across all layers
    DRCReport run();

private:
    soc::db::ChipDB& m_db;

    void check_layer_geometry(soc::db::LayerId layer_id, DRCReport& report);
    void check_open_circuits(DRCReport& report);
    void check_bspdn_keepouts(DRCReport& report);
};

} // namespace vlsi::drc_lvs
