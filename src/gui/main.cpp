#include "main_window.hpp"
#include "core/parsers/lef_def_parser.hpp"
#include <QApplication>
#include <filesystem>
#include <iostream>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("VLSI SoC Studio");
    app.setOrganizationName("Apple-Class EDA Platform");

    soc::db::ChipDB db;
    soc::script::CommandDispatcher dispatcher(db);

    // If Phase 1 synthetic DEF exists, load it directly
    if (std::filesystem::exists("build/m_series_soc.def")) {
        dispatcher.execute("read_def build/m_series_soc.def");
    }

    soc::gui::MainWindow window(db, dispatcher);
    window.show();

    return app.exec();
}
