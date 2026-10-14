#pragma once

#include "../db/chip_db.hpp"
#include <string>
#include <string_view>
#include <vector>

namespace soc::parsers {

class LefParser {
public:
    static bool parse(std::string_view filepath, db::ChipDB& db);
    static bool write(std::string_view filepath, const db::ChipDB& db);
};

class DefParser {
public:
    static bool parse(std::string_view filepath, db::ChipDB& db);
    static bool write(std::string_view filepath, const db::ChipDB& db);
};

} // namespace soc::parsers
