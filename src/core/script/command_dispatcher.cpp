#include "command_dispatcher.hpp"
#include "../parsers/lef_def_parser.hpp"
#include <chrono>
#include <algorithm>
#include <iomanip>

namespace soc::script {

CommandDispatcher::CommandDispatcher(db::ChipDB& db)
    : m_db(db) {
    register_standard_commands();
}

std::vector<std::string> CommandDispatcher::tokenize(const std::string& cmd_line) {
    std::vector<std::string> tokens;
    std::string token;
    bool in_quotes = false;

    for (size_t i = 0; i < cmd_line.size(); ++i) {
        char c = cmd_line[i];
        if (c == '"' || c == '\'') {
            in_quotes = !in_quotes;
            continue;
        }
        if (!in_quotes && std::isspace(static_cast<unsigned char>(c))) {
            if (!token.empty()) {
                tokens.push_back(token);
                token.clear();
            }
        } else {
            token.push_back(c);
        }
    }
    if (!token.empty()) {
        tokens.push_back(token);
    }
    return tokens;
}

void CommandDispatcher::register_command(const std::string& name,
                                         const std::string& help_text,
                                         CommandHandler handler) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_commands[name] = {name, help_text, std::move(handler)};
}

CommandResult CommandDispatcher::execute(const std::string& command_line) {
    auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<std::string> tokens = tokenize(command_line);

    if (tokens.empty()) {
        return {true, "", 0.0};
    }

    const std::string& cmd_name = tokens[0];
    std::vector<std::string> args(tokens.begin() + 1, tokens.end());

    CommandHandler handler;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_history.push_back(command_line);
        auto it = m_commands.find(cmd_name);
        if (it == m_commands.end()) {
            return {false, "Unknown command: '" + cmd_name + "'. Type 'help' for available commands.", 0.0};
        }
        handler = it->second.handler;
    }

    CommandResult res = handler(args);
    auto t1 = std::chrono::high_resolution_clock::now();
    res.execution_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return res;
}

std::vector<std::string> CommandDispatcher::get_completions(const std::string& prefix) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<std::string> matches;
    for (const auto& [name, entry] : m_commands) {
        if (name.rfind(prefix, 0) == 0) {
            matches.push_back(name);
        }
    }
    std::sort(matches.begin(), matches.end());
    return matches;
}

std::string CommandDispatcher::get_help(const std::string& cmd_name) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_commands.find(cmd_name);
    if (it != m_commands.end()) {
        return it->second.name + " - " + it->second.help_text;
    }
    return "No help available for: " + cmd_name;
}

std::vector<std::pair<std::string, std::string>> CommandDispatcher::list_commands() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<std::pair<std::string, std::string>> list;
    for (const auto& [name, entry] : m_commands) {
        list.emplace_back(name, entry.help_text);
    }
    std::sort(list.begin(), list.end());
    return list;
}

void CommandDispatcher::register_standard_commands() {
    // 1. read_def
    register_command("read_def", "read_def <filepath> : Parse DEF physical floorplan into ChipDB",
        [this](const std::vector<std::string>& args) -> CommandResult {
            if (args.empty()) return {false, "Usage: read_def <filepath>", 0.0};
            bool ok = parsers::DefParser::parse(args[0], m_db);
            if (!ok) return {false, "Failed to parse DEF file: " + args[0], 0.0};
            m_db.rebuild_instance_spatial_index();
            std::stringstream ss;
            ss << "Successfully parsed DEF. Instances: " << m_db.instances.size()
               << ", Nets: " << m_db.nets.size()
               << ", Die: [" << m_db.die_area.x_min << ", " << m_db.die_area.y_min
               << "] to [" << m_db.die_area.x_max << ", " << m_db.die_area.y_max << "]";
            return {true, ss.str(), 0.0};
        }
    );

    // 2. zoom_to_bbox
    register_command("zoom_to_bbox", "zoom_to_bbox <x1> <y1> <x2> <y2> : Center viewport on bounding box",
        [this](const std::vector<std::string>& args) -> CommandResult {
            if (args.size() < 4) return {false, "Usage: zoom_to_bbox <x1> <y1> <x2> <y2>", 0.0};
            int32_t x1 = std::stoi(args[0]);
            int32_t y1 = std::stoi(args[1]);
            int32_t x2 = std::stoi(args[2]);
            int32_t y2 = std::stoi(args[3]);
            db::Rect2D bbox{x1, y1, x2, y2};
            if (m_zoom_cb) m_zoom_cb(bbox);
            return {true, "Zoomed to bounding box: [" + args[0] + ", " + args[1] + "] - [" + args[2] + ", " + args[3] + "]", 0.0};
        }
    );

    // 3. highlight_net
    register_command("highlight_net", "highlight_net <net_name> : Select and highlight routing net",
        [this](const std::vector<std::string>& args) -> CommandResult {
            if (args.empty()) return {false, "Usage: highlight_net <net_name>", 0.0};
            for (size_t i = 0; i < m_db.nets.size(); ++i) {
                db::NetId nid{static_cast<uint32_t>(i)};
                if (m_db.strings.resolve(m_db.nets[nid].name_id) == args[0]) {
                    if (m_highlight_cb) m_highlight_cb(nid);
                    return {true, "Highlighted net: " + args[0] + " (pins: " + std::to_string(m_db.nets[nid].pins.size()) + ")", 0.0};
                }
            }
            return {false, "Net not found: " + args[0], 0.0};
        }
    );

    // 4. set_layer_visible
    register_command("set_layer_visible", "set_layer_visible <layer_name> <1|0> : Toggle layer visibility",
        [this](const std::vector<std::string>& args) -> CommandResult {
            if (args.size() < 2) return {false, "Usage: set_layer_visible <layer_name> <1|0>", 0.0};
            bool visible = (args[1] == "1" || args[1] == "true" || args[1] == "on");
            if (m_layer_cb) m_layer_cb(args[0], visible);
            return {true, "Set layer '" + args[0] + "' visibility to " + (visible ? "ON" : "OFF"), 0.0};
        }
    );

    // 5. inspect_instance
    register_command("inspect_instance", "inspect_instance <instance_name> : Display physical & UPF attributes",
        [this](const std::vector<std::string>& args) -> CommandResult {
            if (args.empty()) return {false, "Usage: inspect_instance <instance_name>", 0.0};
            for (size_t i = 0; i < m_db.instances.size(); ++i) {
                db::InstanceId id{static_cast<uint32_t>(i)};
                const auto& inst = m_db.instances[id];
                if (m_db.strings.resolve(inst.name_id) == args[0]) {
                    std::string_view mname = "";
                    if (inst.master_id.is_valid() && inst.master_id.value() < m_db.master_cells.size()) {
                        mname = m_db.strings.resolve(m_db.master_cells[inst.master_id].name_id);
                    }
                    std::string_view pdname = "DEFAULT";
                    double voltage = 0.8;
                    if (inst.power_domain_id.is_valid() && inst.power_domain_id.value() < m_db.power_domains.size()) {
                        pdname = m_db.strings.resolve(m_db.power_domains[inst.power_domain_id].name_id);
                        voltage = m_db.power_domains[inst.power_domain_id].nominal_voltage;
                    }
                    db::Rect2D bbox = m_db.get_instance_bbox(id);

                    std::stringstream ss;
                    ss << "Instance: " << args[0] << "\n"
                       << "  Master Cell:  " << mname << "\n"
                       << "  Origin:       (" << inst.origin.x << ", " << inst.origin.y << ") DBU\n"
                       << "  Bounding Box: [" << bbox.x_min << ", " << bbox.y_min << "] - ["
                       << bbox.x_max << ", " << bbox.y_max << "]\n"
                       << "  Orientation:  " << db::to_string(inst.orientation) << "\n"
                       << "  Power Domain: " << pdname << " (" << std::fixed << std::setprecision(2) << voltage << "V)\n"
                       << "  Status:       " << (inst.is_fixed ? "FIXED" : "PLACED");
                    return {true, ss.str(), 0.0};
                }
            }
            return {false, "Instance not found: " + args[0], 0.0};
        }
    );

    // 6. show_heatmap
    register_command("show_heatmap", "show_heatmap <power|timing|thermal|congestion|none> : Select heatmap overlay",
        [this](const std::vector<std::string>& args) -> CommandResult {
            if (args.empty()) return {false, "Usage: show_heatmap <power|timing|thermal|congestion|none>", 0.0};
            std::string mode = args[0];
            std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
            if (mode != "power" && mode != "timing" && mode != "thermal" &&
                mode != "congestion" && mode != "none") {
                return {false, "Invalid heatmap mode. Choose from: power, timing, thermal, congestion, none", 0.0};
            }
            if (m_heatmap_cb) m_heatmap_cb(mode);
            return {true, "Active heatmap overlay set to: " + mode, 0.0};
        }
    );

    // 7. report_design
    register_command("report_design", "report_design : Summarize die metrics, block hierarchy, and power domains",
        [this](const std::vector<std::string>& /*args*/) -> CommandResult {
            std::stringstream ss;
            ss << "=== VLSI SoC Studio Design Summary ===\n"
               << "Design Name:    " << m_db.strings.resolve(m_db.design_name) << "\n"
               << "Die Area:       " << (m_db.die_area.width() / m_db.dbu_per_micron) << " x "
                                    << (m_db.die_area.height() / m_db.dbu_per_micron) << " um^2\n"
               << "Standard Cells: " << m_db.instances.size() << "\n"
               << "Nets:           " << m_db.nets.size() << "\n"
               << "Block Abstracts:" << m_db.block_abstracts.size() << "\n"
               << "Power Domains:  " << m_db.power_domains.size() << "\n"
               << "Routing Layers: " << m_db.layers.size();
            return {true, ss.str(), 0.0};
        }
    );

    // 8. help
    register_command("help", "help [cmd] : List available commands or show command help",
        [this](const std::vector<std::string>& args) -> CommandResult {
            if (!args.empty()) {
                return {true, get_help(args[0]), 0.0};
            }
            auto cmds = list_commands();
            std::stringstream ss;
            ss << "Available Commands (" << cmds.size() << "):\n";
            for (const auto& [name, help] : cmds) {
                ss << "  " << std::left << std::setw(20) << name << " : " << help << "\n";
            }
            return {true, ss.str(), 0.0};
        }
    );
}

} // namespace soc::script
