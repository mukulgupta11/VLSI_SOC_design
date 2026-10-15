#pragma once

#include "core/db/chip_db.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace vlsi::drc_lvs {

enum class DRCViolationType : uint8_t {
    MIN_WIDTH = 0,
    MIN_SPACING,
    SHORT_CIRCUIT,
    OPEN_CIRCUIT,
    OBSTRUCTION_ENCROACHMENT,
    BSPDN_KEEPOUT_VIOLATION
};

struct DRCViolation {
    DRCViolationType type{DRCViolationType::SHORT_CIRCUIT};
    soc::db::LayerId layer{soc::db::LayerId::invalid()};
    soc::db::Rect2D bbox;
    soc::db::NetId net_a{soc::db::NetId::invalid()};
    soc::db::NetId net_b{soc::db::NetId::invalid()};
    std::string message;
    int32_t actual_val{0};
    int32_t required_val{0};
};

struct DRCReport {
    uint32_t total_violations{0};
    uint32_t min_width_violations{0};
    uint32_t min_spacing_violations{0};
    uint32_t short_violations{0};
    uint32_t open_violations{0};
    uint32_t bspdn_keepout_violations{0};
    std::vector<DRCViolation> violations;
    double runtime_seconds{0.0};
};

struct LVSReport {
    bool is_lvs_match{true};
    uint32_t matched_nets{0};
    uint32_t mismatched_nets{0};
    uint32_t matched_instances{0};
    uint32_t mismatched_instances{0};
    uint32_t matched_pins{0};
    uint32_t mismatched_pins{0};
    std::vector<std::string> error_messages;
    double runtime_seconds{0.0};
};

} // namespace vlsi::drc_lvs
