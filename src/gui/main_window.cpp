#include "main_window.hpp"
#include "core/parsers/gdsii_stream.hpp"
#include <QMenuBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QCoreApplication>

namespace soc::gui {

MainWindow::MainWindow(db::ChipDB& db, script::CommandDispatcher& dispatcher, QWidget* parent)
    : QMainWindow(parent)
    , m_db(db)
    , m_dispatcher(dispatcher) {
    setWindowTitle("VLSI SoC Studio — Apple A/M-Series Class Physical Design & Verification");
    resize(1600, 1000);

    // Dark EDA UI theme
    setStyleSheet(
        "QMainWindow { background-color: #1a1c23; }"
        "QDockWidget { color: #e0e4ec; font-weight: bold; titlebar-close-icon: none; }"
        "QDockWidget::title { background: #222631; padding: 6px; border: 1px solid #2e3444; }"
        "QMenuBar { background-color: #1e222d; color: #d0d4dc; border-bottom: 1px solid #2e3444; }"
        "QMenuBar::item:selected { background-color: #3b4252; }"
        "QStatusBar { background-color: #16181f; color: #a0a6b4; border-top: 1px solid #262a36; }"
        "QToolBar { background-color: #1e222d; border-bottom: 1px solid #2e3444; spacing: 4px; padding: 3px; }"
        "QToolButton { background-color: #2b303c; color: #d0d4dc; border: 1px solid #3b4252; border-radius: 3px; padding: 4px 8px; font-size: 11px; }"
        "QToolButton:hover { background-color: #3b4252; color: #ffffff; }"
        "QToolButton:pressed { background-color: #0e639c; color: #ffffff; }"
        "QComboBox { background-color: #2b303c; color: #d0d4dc; border: 1px solid #3b4252; border-radius: 3px; padding: 2px 6px; font-size: 11px; }"
    );

    // Central OpenGL 4.6 Viewport
    m_viewport = new LayoutViewport(m_db, m_dispatcher, this);
    setCentralWidget(m_viewport);

    setup_toolbar();
    setup_docks();
    setup_menus();
    setup_status_bar();
    connect_signals();

    // Wire incremental script redraw callback
    m_dispatcher.set_redraw_callback([this]() {
        m_display_control->populate_layers();
        m_hierarchy->refresh();
        m_viewport->update();
        QCoreApplication::processEvents();
    });
}

void MainWindow::run_engine_command(const QString& cmd) {
    m_console->append_output(QString("> %1").arg(cmd));
    auto res = m_dispatcher.execute(cmd.toStdString());
    if (res.success) {
        m_console->append_output(QString::fromStdString(res.message));
    } else {
        m_console->append_output(QString::fromStdString(res.message), true);
    }

    // Incremental visual and data refresh
    m_display_control->populate_layers();
    m_hierarchy->refresh();
    m_status_stats->setText(QString("Design: %1 cells | %2 nets | %3 blocks | %4 ms")
        .arg(m_db.instances.size())
        .arg(m_db.nets.size())
        .arg(m_db.block_abstracts.size())
        .arg(res.execution_time_ms, 0, 'f', 1));
    m_viewport->update();
}

void MainWindow::setup_toolbar() {
    m_toolbar = addToolBar("Physical Design Flow");
    m_toolbar->setMovable(false);

    // File I/O
    auto* act_open_def = m_toolbar->addAction("Open DEF");
    connect(act_open_def, &QAction::triggered, [this]() {
        QString file = QFileDialog::getOpenFileName(this, "Open DEF Floorplan", "", "Design Exchange Format (*.def)");
        if (!file.isEmpty()) {
            run_engine_command("read_def \"" + file + "\"");
            m_viewport->fit_all();
        }
    });

    auto* act_export_gds = m_toolbar->addAction("Export GDSII");
    connect(act_export_gds, &QAction::triggered, [this]() {
        QString file = QFileDialog::getSaveFileName(this, "Export GDSII Stream", "apple_soc_signoff.gds", "GDSII Stream (*.gds)");
        if (!file.isEmpty()) {
            run_engine_command("export_gdsii \"" + file + "\"");
        }
    });

    m_toolbar->addSeparator();

    // Individual Flow Steps
    auto* act_fp = m_toolbar->addAction("Floorplan");
    connect(act_fp, &QAction::triggered, [this]() { run_engine_command("run_floorplan"); });

    auto* act_place = m_toolbar->addAction("Placement");
    connect(act_place, &QAction::triggered, [this]() { run_engine_command("run_placement"); });

    auto* act_leg = m_toolbar->addAction("Legalize");
    connect(act_leg, &QAction::triggered, [this]() { run_engine_command("run_legalization"); });

    auto* act_upf = m_toolbar->addAction("UPF Power");
    connect(act_upf, &QAction::triggered, [this]() { run_engine_command("run_upf"); });

    auto* act_cts = m_toolbar->addAction("CTS");
    connect(act_cts, &QAction::triggered, [this]() { run_engine_command("run_cts"); });

    auto* act_sta = m_toolbar->addAction("STA");
    connect(act_sta, &QAction::triggered, [this]() { run_engine_command("run_sta"); });

    auto* act_gr = m_toolbar->addAction("Global Route");
    connect(act_gr, &QAction::triggered, [this]() { run_engine_command("run_global_route"); });

    auto* act_dr = m_toolbar->addAction("Detailed Route");
    connect(act_dr, &QAction::triggered, [this]() { run_engine_command("run_detailed_route"); });

    auto* act_noc = m_toolbar->addAction("NoC Mesh");
    connect(act_noc, &QAction::triggered, [this]() { run_engine_command("run_noc"); });

    auto* act_uf = m_toolbar->addAction("UltraFusion");
    connect(act_uf, &QAction::triggered, [this]() { run_engine_command("run_ultrafusion"); });

    auto* act_therm = m_toolbar->addAction("Thermal/IR");
    connect(act_therm, &QAction::triggered, [this]() { run_engine_command("run_thermal_ir"); });

    auto* act_drc = m_toolbar->addAction("DRC");
    connect(act_drc, &QAction::triggered, [this]() { run_engine_command("run_drc"); });

    auto* act_lvs = m_toolbar->addAction("LVS");
    connect(act_lvs, &QAction::triggered, [this]() { run_engine_command("run_lvs"); });

    auto* act_rl = m_toolbar->addAction("RL PPA");
    connect(act_rl, &QAction::triggered, [this]() { run_engine_command("run_rl_ppa 5"); });

    m_toolbar->addSeparator();

    // High-visibility One-Click Full Flow Button
    auto* act_full_flow = m_toolbar->addAction("RUN FULL FLOW");
    connect(act_full_flow, &QAction::triggered, [this]() { run_engine_command("run_full_flow"); });

    m_toolbar->addSeparator();

    // Heatmap Overlay Selector
    m_heatmap_combo = new QComboBox(m_toolbar);
    m_heatmap_combo->addItems({"Heatmap: None", "Heatmap: Power", "Heatmap: Timing", "Heatmap: Thermal", "Heatmap: Congestion"});
    connect(m_heatmap_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int idx) {
        static const char* modes[] = {"none", "power", "timing", "thermal", "congestion"};
        if (idx >= 0 && idx < 5) {
            run_engine_command(QString("show_heatmap %1").arg(modes[idx]));
        }
    });
    m_toolbar->addWidget(m_heatmap_combo);

    m_toolbar->addSeparator();

    // Viewport Controls
    auto* act_fit = m_toolbar->addAction("Fit [F]");
    connect(act_fit, &QAction::triggered, m_viewport, &LayoutViewport::fit_all);

    auto* act_ruler = m_toolbar->addAction("Ruler [K]");
    connect(act_ruler, &QAction::triggered, [this]() {
        m_viewport->ruler().active = !m_viewport->ruler().active;
        m_viewport->ruler().has_start = false;
        m_viewport->update();
    });
}

void MainWindow::setup_docks() {
    // 1. Left Dock: Display Control
    auto* left_dock = new QDockWidget("Display Control", this);
    left_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_display_control = new DisplayControlPanel(m_db, *m_viewport, left_dock);
    left_dock->setWidget(m_display_control);
    addDockWidget(Qt::LeftDockWidgetArea, left_dock);

    // 2. Right Dock Top: Hierarchy Browser
    auto* right_dock_top = new QDockWidget("Hierarchy Browser", this);
    right_dock_top->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_hierarchy = new HierarchyBrowser(m_db, right_dock_top);
    right_dock_top->setWidget(m_hierarchy);
    addDockWidget(Qt::RightDockWidgetArea, right_dock_top);

    // 3. Right Dock Bottom: Object & Timing Inspector
    auto* right_dock_bot = new QDockWidget("Object / Timing Inspector", this);
    right_dock_bot->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    m_inspector = new InspectorPanel(m_db, right_dock_bot);
    right_dock_bot->setWidget(m_inspector);
    addDockWidget(Qt::RightDockWidgetArea, right_dock_bot);

    // Split right docks vertically
    splitDockWidget(right_dock_top, right_dock_bot, Qt::Vertical);

    // 4. Bottom Dock: Interactive Scripting Console
    auto* bot_dock = new QDockWidget("Scripting Console", this);
    bot_dock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    m_console = new ScriptConsole(m_dispatcher, bot_dock);
    bot_dock->setWidget(m_console);
    addDockWidget(Qt::BottomDockWidgetArea, bot_dock);
}

void MainWindow::setup_menus() {
    auto* mb = menuBar();

    // File Menu
    auto* file_menu = mb->addMenu("&File");
    file_menu->addAction("&Open DEF...", [this]() {
        QString file = QFileDialog::getOpenFileName(this, "Open DEF Floorplan", "", "Design Exchange Format (*.def)");
        if (!file.isEmpty()) {
            run_engine_command("read_def \"" + file + "\"");
            m_viewport->fit_all();
        }
    });

    file_menu->addAction("&Export GDSII...", [this]() {
        QString file = QFileDialog::getSaveFileName(this, "Export GDSII Stream", "apple_soc_signoff.gds", "GDSII Stream (*.gds)");
        if (!file.isEmpty()) {
            run_engine_command("export_gdsii \"" + file + "\"");
        }
    });

    file_menu->addAction("&Initialize Apple M-Series Synthetic SoC", [this]() {
        run_engine_command("init_synthetic_soc");
        m_viewport->fit_all();
    });

    file_menu->addSeparator();
    file_menu->addAction("&Quit", this, &QWidget::close);

    // View Menu
    auto* view_menu = mb->addMenu("&View");
    view_menu->addAction("&Fit All [F]", QKeySequence(Qt::Key_F), m_viewport, &LayoutViewport::fit_all);
    view_menu->addAction("Toggle &Ruler [K]", QKeySequence(Qt::Key_K), [this]() {
        m_viewport->ruler().active = !m_viewport->ruler().active;
        m_viewport->ruler().has_start = false;
        m_viewport->update();
    });

    auto* hm_menu = view_menu->addMenu("&Heatmaps");
    hm_menu->addAction("None", [this]() { run_engine_command("show_heatmap none"); });
    hm_menu->addAction("Power Density", [this]() { run_engine_command("show_heatmap power"); });
    hm_menu->addAction("Timing Slack (WNS)", [this]() { run_engine_command("show_heatmap timing"); });
    hm_menu->addAction("Thermal (°C)", [this]() { run_engine_command("show_heatmap thermal"); });
    hm_menu->addAction("Global Routing Congestion", [this]() { run_engine_command("show_heatmap congestion"); });

    // Physical Design Flow Menu
    auto* flow_menu = mb->addMenu("&Flow");
    flow_menu->addAction("▶ &Run Full RTL-to-GDSII Signoff Flow", [this]() { run_engine_command("run_full_flow"); });
    flow_menu->addSeparator();
    flow_menu->addAction("Step 1: Hierarchical Floorplanning & Pin Budgeting", [this]() { run_engine_command("run_floorplan"); });
    flow_menu->addAction("Step 2: Analytical Global Placement (WA-HPWL + FFT)", [this]() { run_engine_command("run_placement"); });
    flow_menu->addAction("Step 3: FinFET/GAAFET Detailed Legalization", [this]() { run_engine_command("run_legalization"); });
    flow_menu->addAction("Step 4: IEEE 1801 UPF Multi-Voltage Power Domains", [this]() { run_engine_command("run_upf"); });
    flow_menu->addAction("Step 5: Apple-Style Mesh NoC & 4GHz Repeater Synthesis", [this]() { run_engine_command("run_noc"); });
    flow_menu->addAction("Step 6: 3D-IC UltraFusion Micro-Bump Alignment Router", [this]() { run_engine_command("run_ultrafusion"); });
    flow_menu->addAction("Step 7: Zero-Skew Clock Tree Synthesis (DME & H-Tree)", [this]() { run_engine_command("run_cts"); });
    flow_menu->addAction("Step 8: Multi-Corner Multi-Mode STA with POCV", [this]() { run_engine_command("run_sta"); });
    flow_menu->addAction("Step 9: 3D Congestion Global Maze Router", [this]() { run_engine_command("run_global_route"); });
    flow_menu->addAction("Step 10: Track Detailed Router & BSPDN Rails", [this]() { run_engine_command("run_detailed_route"); });
    flow_menu->addAction("Step 11: 3D Finite-Difference Thermal & IR-Drop Solver", [this]() { run_engine_command("run_thermal_ir"); });
    flow_menu->addAction("Step 12: Bentley-Ottmann Sweep-Line DRC Verification", [this]() { run_engine_command("run_drc"); });
    flow_menu->addAction("Step 13: LVS Netlist Graph Isomorphism", [this]() { run_engine_command("run_lvs"); });
    flow_menu->addAction("Step 14: Autonomous RL PPA Optimizer", [this]() { run_engine_command("run_rl_ppa 5"); });

    // Tools Menu
    auto* tools_menu = mb->addMenu("&Tools");
    tools_menu->addAction("Design &Summary Report", [this]() { run_engine_command("report_design"); });
    tools_menu->addAction("&Help", [this]() { run_engine_command("help"); });
}

void MainWindow::setup_status_bar() {
    auto* sb = statusBar();
    m_status_coords = new QLabel("X: 0.000 um, Y: 0.000 um  ", this);
    m_status_ruler = new QLabel("Ruler: Inactive  ", this);
    m_status_stats = new QLabel(QString("Design: %1 cells | 60 FPS Target").arg(m_db.instances.size()), this);

    sb->addPermanentWidget(m_status_coords);
    sb->addPermanentWidget(m_status_ruler);
    sb->addPermanentWidget(m_status_stats);
}

void MainWindow::connect_signals() {
    connect(m_viewport, &LayoutViewport::cursorWorldMoved, [this](db::Point2D p) {
        double x_um = static_cast<double>(p.x) / m_db.dbu_per_micron;
        double y_um = static_cast<double>(p.y) / m_db.dbu_per_micron;
        m_status_coords->setText(QString("X: %1 um, Y: %2 um  ")
            .arg(x_um, 0, 'f', 3).arg(y_um, 0, 'f', 3));
    });

    connect(m_viewport, &LayoutViewport::rulerUpdated, [this](double dist, double dx, double dy) {
        m_status_ruler->setText(QString("Ruler: dist=%1 um (dx=%2, dy=%3)  ")
            .arg(dist, 0, 'f', 2).arg(dx, 0, 'f', 2).arg(dy, 0, 'f', 2));
    });

    connect(m_viewport, &LayoutViewport::instanceSelected, [this](db::InstanceId id) {
        m_inspector->inspect_instance(id);
    });

    connect(m_viewport, &LayoutViewport::blockSelected, [this](db::BlockAbstractId id) {
        m_inspector->inspect_block(id);
    });

    connect(m_viewport, &LayoutViewport::netSelected, [this](db::NetId id) {
        m_inspector->inspect_net(id);
    });

    connect(m_hierarchy, &HierarchyBrowser::instanceSelected, [this](db::InstanceId id) {
        m_viewport->select_instance(id);
        m_inspector->inspect_instance(id);
    });

    connect(m_hierarchy, &HierarchyBrowser::blockSelected, [this](db::BlockAbstractId id) {
        m_inspector->inspect_block(id);
    });

    connect(m_hierarchy, &HierarchyBrowser::zoomRequested, [this](const db::Rect2D& box) {
        m_viewport->zoom_to_box(box);
    });
}

} // namespace soc::gui
