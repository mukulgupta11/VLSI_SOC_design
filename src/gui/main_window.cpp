#include "main_window.hpp"
#include "core/parsers/gdsii_stream.hpp"
#include <QMenuBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QMessageBox>

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
    );

    // Central OpenGL 4.6 Viewport
    m_viewport = new LayoutViewport(m_db, m_dispatcher, this);
    setCentralWidget(m_viewport);

    setup_docks();
    setup_menus();
    setup_status_bar();
    connect_signals();
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
            m_dispatcher.execute("read_def \"" + file.toStdString() + "\"");
            m_display_control->populate_layers();
            m_hierarchy->refresh();
            m_viewport->fit_all();
        }
    });

    file_menu->addAction("&Export GDSII...", [this]() {
        QString file = QFileDialog::getSaveFileName(this, "Export GDSII Stream", "layout.gds", "GDSII Stream (*.gds)");
        if (!file.isEmpty()) {
            bool ok = soc::parsers::GdsiiWriter::export_chip_db(m_db, file.toStdString());
            if (ok) {
                m_console->append_output("Exported GDSII successfully: " + file);
            } else {
                m_console->append_output("Failed to export GDSII: " + file, true);
            }
        }
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

    // Tools Menu
    auto* tools_menu = mb->addMenu("&Tools");
    tools_menu->addAction("Design &Summary Report", [this]() {
        auto res = m_dispatcher.execute("report_design");
        m_console->append_output(QString::fromStdString(res.message));
    });
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
