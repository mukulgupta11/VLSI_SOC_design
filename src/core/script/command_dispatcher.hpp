#pragma once

#include "../db/chip_db.hpp"
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <sstream>

namespace soc::script {

struct CommandResult {
    bool success{true};
    std::string message;
    double execution_time_ms{0.0};
};

using CommandHandler = std::function<CommandResult(const std::vector<std::string>& args)>;

// Headless-first central CommandDispatcher for GUI, Tcl, and Python bindings
class CommandDispatcher {
public:
    explicit CommandDispatcher(db::ChipDB& db);
    ~CommandDispatcher() = default;

    // Register a command with name, help string, and handler
    void register_command(const std::string& name,
                          const std::string& help_text,
                          CommandHandler handler);

    // Execute raw command string (e.g. "zoom_to_bbox 500 500 2000 2000")
    CommandResult execute(const std::string& command_line);

    // Autocomplete helper for CLI and GUI scripting console
    [[nodiscard]] std::vector<std::string> get_completions(const std::string& prefix) const;

    // Command documentation
    [[nodiscard]] std::string get_help(const std::string& cmd_name) const;
    [[nodiscard]] std::vector<std::pair<std::string, std::string>> list_commands() const;

    // Command History
    [[nodiscard]] const std::vector<std::string>& history() const noexcept { return m_history; }

    // Register all standard EDA commands
    void register_standard_commands();

    // Callbacks for GUI synchronization
    using ZoomCallback = std::function<void(const db::Rect2D&)>;
    using LayerVisibilityCallback = std::function<void(std::string_view, bool)>;
    using HighlightNetCallback = std::function<void(db::NetId)>;
    using HeatmapCallback = std::function<void(const std::string&)>;

    void set_zoom_callback(ZoomCallback cb) { m_zoom_cb = std::move(cb); }
    void set_layer_visibility_callback(LayerVisibilityCallback cb) { m_layer_cb = std::move(cb); }
    void set_highlight_net_callback(HighlightNetCallback cb) { m_highlight_cb = std::move(cb); }
    void set_heatmap_callback(HeatmapCallback cb) { m_heatmap_cb = std::move(cb); }

private:
    struct CommandEntry {
        std::string name;
        std::string help_text;
        CommandHandler handler;
    };

    db::ChipDB& m_db;
    mutable std::mutex m_mutex;
    std::unordered_map<std::string, CommandEntry> m_commands;
    std::vector<std::string> m_history;

    ZoomCallback            m_zoom_cb;
    LayerVisibilityCallback m_layer_cb;
    HighlightNetCallback    m_highlight_cb;
    HeatmapCallback         m_heatmap_cb;

    static std::vector<std::string> tokenize(const std::string& cmd_line);
};

} // namespace soc::script
