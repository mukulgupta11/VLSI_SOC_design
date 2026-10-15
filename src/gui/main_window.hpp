#pragma once

#include "core/db/chip_db.hpp"
#include "core/script/command_dispatcher.hpp"
#include "viewport/layout_viewport.hpp"
#include "display_control_panel.hpp"
#include "hierarchy_browser.hpp"
#include "inspector_panel.hpp"
#include "script_console.hpp"

#include <QMainWindow>
#include <QDockWidget>
#include <QLabel>
#include <QToolBar>
#include <QComboBox>

namespace soc::gui {

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(db::ChipDB& db, script::CommandDispatcher& dispatcher, QWidget* parent = nullptr);
    ~MainWindow() override = default;

    [[nodiscard]] LayoutViewport* viewport() noexcept { return m_viewport; }
    [[nodiscard]] DisplayControlPanel* display_control() noexcept { return m_display_control; }
    [[nodiscard]] HierarchyBrowser* hierarchy_browser() noexcept { return m_hierarchy; }
    [[nodiscard]] InspectorPanel* inspector() noexcept { return m_inspector; }
    [[nodiscard]] ScriptConsole* console() noexcept { return m_console; }

    // Execute engine command and incrementally update UI
    void run_engine_command(const QString& cmd);

private:
    db::ChipDB& m_db;
    script::CommandDispatcher& m_dispatcher;

    // Center viewport
    LayoutViewport* m_viewport{nullptr};

    // Dock panels
    DisplayControlPanel* m_display_control{nullptr};
    HierarchyBrowser*    m_hierarchy{nullptr};
    InspectorPanel*      m_inspector{nullptr};
    ScriptConsole*       m_console{nullptr};

    // Toolbar
    QToolBar*            m_toolbar{nullptr};
    QComboBox*           m_heatmap_combo{nullptr};

    // Status bar labels
    QLabel* m_status_coords{nullptr};
    QLabel* m_status_ruler{nullptr};
    QLabel* m_status_stats{nullptr};

    void setup_toolbar();
    void setup_docks();
    void setup_menus();
    void setup_status_bar();
    void connect_signals();
};

} // namespace soc::gui
