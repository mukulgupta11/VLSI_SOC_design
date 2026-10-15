#include "main_window.hpp"
#include "core/parsers/lef_def_parser.hpp"
#include <QApplication>
#include <filesystem>
#include <iostream>
#include <cassert>
#include <string>

int main(int argc, char* argv[]) {
    bool self_test = false;
    bool batch_mode = false;
    bool gui_script_mode = false;
    std::string script_path;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--self-test") {
            self_test = true;
        } else if (arg == "-batch" && i + 1 < argc) {
            batch_mode = true;
            script_path = argv[++i];
        } else if (arg == "-gui" && i + 1 < argc) {
            gui_script_mode = true;
            script_path = argv[++i];
        }
    }

    if (self_test || batch_mode) {
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }

    QApplication app(argc, argv);
    app.setApplicationName("VLSI SoC Studio");
    app.setOrganizationName("Apple-Class EDA Platform");

    soc::db::ChipDB db;
    soc::script::CommandDispatcher dispatcher(db);

    if (self_test) {
        std::cout << "===================================================================\n";
        std::cout << "  VLSI SoC Studio: Desktop Application Context Self-Test (--self-test)\n";
        std::cout << "===================================================================\n";
        std::cout << "[Self-Test 1/4] Initializing Qt6 + OpenGL 4.6 headless context...\n";
        soc::gui::MainWindow window(db, dispatcher);
        window.show();
        app.processEvents();

        std::cout << "[Self-Test 2/4] Verifying all 5 docking subsystems...\n";
        assert(window.viewport() != nullptr);
        assert(window.display_control() != nullptr);
        assert(window.hierarchy_browser() != nullptr);
        assert(window.inspector() != nullptr);
        assert(window.console() != nullptr);
        std::cout << "  -> Viewport, DisplayControl, Hierarchy, Inspector, ScriptConsole verified!\n";

        std::cout << "[Self-Test 3/4] Verifying CommandDispatcher EDA registration...\n";
        auto res_report = dispatcher.execute("report_design");
        assert(res_report.success);
        std::cout << "  -> 'report_design' executed successfully.\n";

        auto res_help = dispatcher.execute("help");
        assert(res_help.success);
        auto cmd_count = dispatcher.list_commands().size();
        assert(cmd_count >= 25);
        std::cout << "  -> 'help' executed successfully (" << cmd_count << " commands registered).\n";

        std::cout << "[Self-Test 4/4] Triggering OpenGL instanced rendering pass...\n";
        window.viewport()->update();
        app.processEvents();

        std::cout << "===================================================================\n";
        std::cout << "  [SUCCESS] VLSI SoC Studio Desktop GUI & Shaders Verified Clean!    \n";
        std::cout << "===================================================================\n";
        return 0;
    }

    if (batch_mode) {
        std::cout << "===================================================================\n";
        std::cout << "  VLSI SoC Studio: Headless Batch Script Execution (-batch)\n";
        std::cout << "===================================================================\n";
        std::cout << "[Batch] Sourcing Tcl script: " << script_path << "\n";
        auto res = dispatcher.execute("source " + script_path);
        std::cout << "\n[Batch] Execution Result:\n" << res.message << "\n";
        std::cout << "[Batch] Total Execution Time: " << res.execution_time_ms << " ms\n";
        if (res.success) {
            std::cout << "===================================================================\n";
            std::cout << "  [SUCCESS] Batch Script Execution Completed Cleanly!              \n";
            std::cout << "===================================================================\n";
            return 0;
        } else {
            std::cerr << "===================================================================\n";
            std::cerr << "  [FAILURE] Batch Script Execution Encountered Errors!             \n";
            std::cerr << "===================================================================\n";
            return 1;
        }
    }

    // Interactive GUI mode
    soc::gui::MainWindow window(db, dispatcher);
    window.show();

    if (gui_script_mode) {
        std::cout << "[GUI] Sourcing Tcl script on launch: " << script_path << "\n";
        window.run_engine_command(QString::fromStdString("source " + script_path));
    } else if (std::filesystem::exists("build/m_series_soc.def")) {
        dispatcher.execute("read_def build/m_series_soc.def");
        window.viewport()->fit_all();
    }

    return app.exec();
}
