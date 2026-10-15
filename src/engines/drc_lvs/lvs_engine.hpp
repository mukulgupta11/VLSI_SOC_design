#pragma once

#include "drc_types.hpp"
#include "core/db/chip_db.hpp"

namespace vlsi::drc_lvs {

class LVSEngine {
public:
    explicit LVSEngine(soc::db::ChipDB& db);

    // Extract physical layout netlist graph and perform LVS isomorphism verification against schematic
    LVSReport run();

private:
    soc::db::ChipDB& m_db;
};

} // namespace vlsi::drc_lvs
