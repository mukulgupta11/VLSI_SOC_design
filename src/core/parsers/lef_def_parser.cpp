#include "lef_def_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cctype>

namespace soc::parsers {

namespace {

// Lightweight tokenizer for LEF/DEF formats
class Tokenizer {
public:
    explicit Tokenizer(std::istream& in) : m_in(in) {}

    bool next_token(std::string& token) {
        token.clear();
        char c;
        while (m_in.get(c)) {
            if (c == '#') {
                // Comment until end of line
                std::string comment_line;
                std::getline(m_in, comment_line);
                continue;
            }
            if (std::isspace(static_cast<unsigned char>(c))) {
                continue;
            }
            if (c == ';' || c == '(' || c == ')') {
                token.push_back(c);
                return true;
            }
            // Read alphanumeric or symbol token
            token.push_back(c);
            while (m_in.get(c)) {
                if (std::isspace(static_cast<unsigned char>(c)) || c == ';' || c == '(' || c == ')' || c == '#') {
                    m_in.unget();
                    return true;
                }
                token.push_back(c);
            }
            return true;
        }
        return false;
    }

private:
    std::istream& m_in;
};

} // anonymous namespace

// -------------------------------------------------------------
// LEF Parser & Writer
// -------------------------------------------------------------
bool LefParser::write(std::string_view filepath, const db::ChipDB& db) {
    std::ofstream out{std::string(filepath)};
    if (!out.is_open()) return false;

    out << "VERSION 5.8 ;\n";
    out << "BUSBITCHARS \"[]\" ;\n";
    out << "DIVIDERCHAR \"/\" ;\n";
    out << "UNITS\n";
    out << "  DATABASE MICRONS " << db.dbu_per_micron << " ;\n";
    out << "END UNITS\n\n";

    // Write Layers
    for (size_t i = 0; i < db.layers.size(); ++i) {
        const auto& lyr = db.layers[static_cast<uint32_t>(i)];
        std::string_view lname = db.strings.resolve(lyr.name_id);
        if (lname.empty()) continue;

        out << "LAYER " << lname << "\n";
        out << "  TYPE " << (lyr.is_routing ? "ROUTING" : (lyr.is_cut ? "CUT" : "MASTERSLICE")) << " ;\n";
        if (lyr.pitch > 0) out << "  PITCH " << (static_cast<double>(lyr.pitch) / db.dbu_per_micron) << " ;\n";
        if (lyr.min_width > 0) out << "  WIDTH " << (static_cast<double>(lyr.min_width) / db.dbu_per_micron) << " ;\n";
        if (lyr.min_spacing > 0) out << "  SPACING " << (static_cast<double>(lyr.min_spacing) / db.dbu_per_micron) << " ;\n";
        out << "END " << lname << "\n\n";
    }

    // Write Master Cells
    for (size_t i = 0; i < db.master_cells.size(); ++i) {
        const auto& cell = db.master_cells[static_cast<uint32_t>(i)];
        std::string_view cname = db.strings.resolve(cell.name_id);
        if (cname.empty()) continue;

        out << "MACRO " << cname << "\n";
        out << "  CLASS " << (cell.cell_type == db::CellType::STD_CELL ? "CORE" : "BLOCK") << " ;\n";
        out << "  ORIGIN 0 0 ;\n";
        out << "  SIZE " << (static_cast<double>(cell.width) / db.dbu_per_micron)
            << " BY " << (static_cast<double>(cell.height) / db.dbu_per_micron) << " ;\n";

        for (const auto& port : cell.ports) {
            std::string_view pname = db.strings.resolve(port.pin_name);
            out << "  PIN " << pname << "\n";
            out << "    DIRECTION INOUT ;\n";
            out << "    USE SIGNAL ;\n";
            out << "    PORT\n";
            if (port.layer_id.is_valid() && port.layer_id.value() < db.layers.size()) {
                out << "      LAYER " << db.strings.resolve(db.layers[port.layer_id].name_id) << " ;\n";
            }
            out << "      RECT " << (static_cast<double>(port.box.x_min) / db.dbu_per_micron) << " "
                                 << (static_cast<double>(port.box.y_min) / db.dbu_per_micron) << " "
                                 << (static_cast<double>(port.box.x_max) / db.dbu_per_micron) << " "
                                 << (static_cast<double>(port.box.y_max) / db.dbu_per_micron) << " ;\n";
            out << "    END\n";
            out << "  END " << pname << "\n";
        }

        if (!cell.obstructions.empty()) {
            out << "  OBS\n";
            if (cell.obstruction_layer.is_valid() && cell.obstruction_layer.value() < db.layers.size()) {
                out << "    LAYER " << db.strings.resolve(db.layers[cell.obstruction_layer].name_id) << " ;\n";
            }
            for (const auto& obs : cell.obstructions) {
                out << "    RECT " << (static_cast<double>(obs.x_min) / db.dbu_per_micron) << " "
                                   << (static_cast<double>(obs.y_min) / db.dbu_per_micron) << " "
                                   << (static_cast<double>(obs.x_max) / db.dbu_per_micron) << " "
                                   << (static_cast<double>(obs.y_max) / db.dbu_per_micron) << " ;\n";
            }
            out << "  END\n";
        }

        out << "END " << cname << "\n\n";
    }

    out << "END LIBRARY\n";
    return true;
}

bool LefParser::parse(std::string_view filepath, db::ChipDB& db) {
    std::ifstream in{std::string(filepath)};
    if (!in.is_open()) return false;

    Tokenizer tok(in);
    std::string token;

    double dbu = 1000.0;
    while (tok.next_token(token)) {
        if (token == "DATABASE") {
            if (tok.next_token(token) && token == "MICRONS") {
                if (tok.next_token(token)) {
                    dbu = std::stod(token);
                    db.dbu_per_micron = static_cast<uint32_t>(dbu);
                }
            }
        } else if (token == "LAYER") {
            std::string lname;
            if (tok.next_token(lname)) {
                bool is_routing = false;
                bool is_cut = false;
                int32_t pitch = 0, width = 0, spacing = 0;
                while (tok.next_token(token) && token != "END") {
                    if (token == "TYPE") {
                        tok.next_token(token);
                        if (token == "ROUTING") is_routing = true;
                        else if (token == "CUT") is_cut = true;
                    } else if (token == "PITCH") {
                        tok.next_token(token);
                        pitch = static_cast<int32_t>(std::stod(token) * dbu);
                    } else if (token == "WIDTH") {
                        tok.next_token(token);
                        width = static_cast<int32_t>(std::stod(token) * dbu);
                    } else if (token == "SPACING") {
                        tok.next_token(token);
                        spacing = static_cast<int32_t>(std::stod(token) * dbu);
                    }
                }
                tok.next_token(token); // consume layer name after END
                uint16_t layer_idx = static_cast<uint16_t>(db.layers.size() + 1);
                db.create_layer(lname, layer_idx, 0, pitch, width, spacing, is_routing, is_cut);
            }
        } else if (token == "MACRO") {
            std::string macro_name;
            if (tok.next_token(macro_name)) {
                int32_t mwidth = 0, mheight = 0;
                db::CellType ctype = db::CellType::STD_CELL;

                while (tok.next_token(token) && token != "END") {
                    if (token == "CLASS") {
                        tok.next_token(token);
                        if (token == "BLOCK") ctype = db::CellType::MACRO;
                    } else if (token == "SIZE") {
                        tok.next_token(token);
                        mwidth = static_cast<int32_t>(std::stod(token) * dbu);
                        tok.next_token(token); // "BY"
                        tok.next_token(token);
                        mheight = static_cast<int32_t>(std::stod(token) * dbu);
                    }
                }
                tok.next_token(token); // consume macro name after END
                db.create_master_cell(macro_name, mwidth, mheight, ctype);
            }
        }
    }
    return true;
}

// -------------------------------------------------------------
// DEF Parser & Writer
// -------------------------------------------------------------
bool DefParser::write(std::string_view filepath, const db::ChipDB& db) {
    std::ofstream out{std::string(filepath)};
    if (!out.is_open()) return false;

    out << "VERSION 5.8 ;\n";
    out << "DIVIDERCHAR \"/\" ;\n";
    out << "BUSBITCHARS \"[]\" ;\n";
    std::string_view dname = db.strings.resolve(db.design_name);
    out << "DESIGN " << (dname.empty() ? "TOP_SOC" : dname) << " ;\n";
    out << "UNITS DISTANCE MICRONS " << db.dbu_per_micron << " ;\n\n";

    // DIEAREA
    out << "DIEAREA ( " << db.die_area.x_min << " " << db.die_area.y_min << " ) ( "
        << db.die_area.x_max << " " << db.die_area.y_max << " ) ;\n\n";

    // TRACKS
    for (size_t i = 0; i < db.tracks.size(); ++i) {
        const auto& trk = db.tracks[static_cast<uint32_t>(i)];
        std::string_view lname = db.strings.resolve(db.layers[trk.layer_id].name_id);
        out << "TRACKS " << (trk.is_horizontal ? "Y " : "X ")
            << trk.start_coord << " DO " << trk.num_tracks << " STEP " << trk.step
            << " LAYER " << lname << " ;\n";
    }
    if (!db.tracks.empty()) out << "\n";

    // BLOCKAGES (Block Abstracts)
    if (!db.block_abstracts.empty()) {
        out << "BLOCKAGES " << db.block_abstracts.size() << " ;\n";
        for (size_t i = 0; i < db.block_abstracts.size(); ++i) {
            const auto& ba = db.block_abstracts[static_cast<uint32_t>(i)];
            out << "  - PLACEMENT RECT ( " << ba.boundary.x_min << " " << ba.boundary.y_min << " ) ( "
                << ba.boundary.x_max << " " << ba.boundary.y_max << " ) ;\n";
        }
        out << "END BLOCKAGES\n\n";
    }

    // COMPONENTS
    out << "COMPONENTS " << db.instances.size() << " ;\n";
    for (size_t i = 0; i < db.instances.size(); ++i) {
        const auto& inst = db.instances[static_cast<uint32_t>(i)];
        std::string_view iname = db.strings.resolve(inst.name_id);
        std::string_view mname = "";
        if (inst.master_id.is_valid() && inst.master_id.value() < db.master_cells.size()) {
            mname = db.strings.resolve(db.master_cells[inst.master_id].name_id);
        }

        out << "  - " << iname << " " << mname << " + "
            << (inst.is_fixed ? "FIXED" : "PLACED")
            << " ( " << inst.origin.x << " " << inst.origin.y << " ) "
            << db::to_string(inst.orientation) << " ;\n";
    }
    out << "END COMPONENTS\n\n";

    // NETS
    out << "NETS " << db.nets.size() << " ;\n";
    for (size_t i = 0; i < db.nets.size(); ++i) {
        const auto& net = db.nets[static_cast<uint32_t>(i)];
        std::string_view nname = db.strings.resolve(net.name_id);
        out << "  - " << nname;

        // Output connected pins
        for (const auto& pid : net.pins) {
            if (!pid.is_valid() || pid.value() >= db.pins.size()) continue;
            const auto& pin = db.pins[pid];
            std::string_view pname = db.strings.resolve(pin.name_id);
            if (pin.instance_id.is_valid() && pin.instance_id.value() < db.instances.size()) {
                std::string_view iname = db.strings.resolve(db.instances[pin.instance_id].name_id);
                out << " ( " << iname << " " << pname << " )";
            } else {
                out << " ( PIN " << pname << " )";
            }
        }

        // Output routed wire segments if present
        if (!net.route_segments.empty()) {
            out << "\n    + ROUTED";
            for (const auto& seg : net.route_segments) {
                std::string_view lname = "metal1";
                if (seg.layer.is_valid() && seg.layer.value() < db.layers.size()) {
                    lname = db.strings.resolve(db.layers[seg.layer].name_id);
                }
                out << " " << lname << " ( " << seg.start.x << " " << seg.start.y << " ) ( "
                    << seg.end.x << " " << seg.end.y << " )";
            }
        }
        out << " ;\n";
    }
    out << "END NETS\n\n";

    out << "END DESIGN\n";
    return true;
}

bool DefParser::parse(std::string_view filepath, db::ChipDB& db) {
    std::ifstream in{std::string(filepath)};
    if (!in.is_open()) return false;

    Tokenizer tok(in);
    std::string token;

    while (tok.next_token(token)) {
        if (token == "DESIGN") {
            if (tok.next_token(token)) {
                db.design_name = db.strings.intern(token);
            }
        } else if (token == "UNITS") {
            if (tok.next_token(token) && token == "DISTANCE") {
                if (tok.next_token(token) && token == "MICRONS") {
                    if (tok.next_token(token)) {
                        db.dbu_per_micron = static_cast<uint32_t>(std::stoul(token));
                    }
                }
            }
        } else if (token == "DIEAREA") {
            // ( x1 y1 ) ( x2 y2 ) ;
            int32_t x1 = 0, y1 = 0, x2 = 0, y2 = 0;
            std::string t;
            if (tok.next_token(t) && t == "(" && tok.next_token(t)) x1 = std::stoi(t);
            if (tok.next_token(t)) y1 = std::stoi(t);
            tok.next_token(t); // ')'
            if (tok.next_token(t) && t == "(" && tok.next_token(t)) x2 = std::stoi(t);
            if (tok.next_token(t)) y2 = std::stoi(t);
            tok.next_token(t); // ')'
            db.die_area = db::Rect2D{x1, y1, x2, y2};
        } else if (token == "COMPONENTS") {
            std::string count_str;
            tok.next_token(count_str);
            while (tok.next_token(token) && token != "END") {
                if (token == "-") {
                    std::string iname, mname;
                    tok.next_token(iname);
                    tok.next_token(mname);

                    // Find or create master cell
                    db::MasterCellId mid{0xFFFFFFFF};
                    for (size_t i = 0; i < db.master_cells.size(); ++i) {
                        if (db.strings.resolve(db.master_cells[static_cast<uint32_t>(i)].name_id) == mname) {
                            mid = db::MasterCellId{static_cast<uint32_t>(i)};
                            break;
                        }
                    }

                    int32_t ox = 0, oy = 0;
                    db::Orientation orient = db::Orientation::R0;
                    bool is_fixed = false;

                    while (tok.next_token(token) && token != ";") {
                        if (token == "PLACED" || token == "FIXED") {
                            is_fixed = (token == "FIXED");
                            tok.next_token(token); // '('
                            tok.next_token(token);
                            ox = std::stoi(token);
                            tok.next_token(token);
                            oy = std::stoi(token);
                            tok.next_token(token); // ')'
                            tok.next_token(token); // Orientation
                            if (token == "N") orient = db::Orientation::R0;
                            else if (token == "W") orient = db::Orientation::R90;
                            else if (token == "S") orient = db::Orientation::R180;
                            else if (token == "E") orient = db::Orientation::R270;
                            else if (token == "FN") orient = db::Orientation::MY;
                            else if (token == "FS") orient = db::Orientation::MX;
                        }
                    }

                    db::InstanceId inst_id = db.create_instance(iname, mid, {ox, oy}, orient);
                    db.instances[inst_id].is_fixed = is_fixed;
                }
            }
            tok.next_token(token); // consume COMPONENTS after END
        } else if (token == "NETS") {
            std::string count_str;
            tok.next_token(count_str);
            while (tok.next_token(token) && token != "END") {
                if (token == "-") {
                    std::string net_name;
                    tok.next_token(net_name);
                    db::NetId nid = db.create_net(net_name);
                    (void)nid;

                    while (tok.next_token(token) && token != ";") {
                        if (token == "(") {
                            std::string iname, pname;
                            tok.next_token(iname);
                            tok.next_token(pname);
                            tok.next_token(token); // ')'
                            // Map to pin/instance if available
                        }
                    }
                }
            }
            tok.next_token(token); // consume NETS after END
        }
    }
    return true;
}

} // namespace soc::parsers
