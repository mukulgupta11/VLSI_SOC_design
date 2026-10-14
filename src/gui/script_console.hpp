#pragma once

#include "core/script/command_dispatcher.hpp"
#include <QWidget>
#include <QTextEdit>
#include <QLineEdit>
#include <QVBoxLayout>
#include <vector>

namespace soc::gui {

class ScriptConsole : public QWidget {
    Q_OBJECT

public:
    explicit ScriptConsole(script::CommandDispatcher& dispatcher, QWidget* parent = nullptr);
    ~ScriptConsole() override = default;

    void append_output(const QString& text, bool is_error = false);

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
    void on_command_entered();

private:
    script::CommandDispatcher& m_dispatcher;
    QTextEdit* m_output_log{nullptr};
    QLineEdit* m_input_line{nullptr};

    std::vector<QString> m_history;
    int m_history_index{-1};

    void setup_ui();
};

} // namespace soc::gui
