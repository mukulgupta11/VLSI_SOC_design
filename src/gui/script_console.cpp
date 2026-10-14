#include "script_console.hpp"
#include <QLabel>
#include <QHBoxLayout>
#include <QKeyEvent>

namespace soc::gui {

ScriptConsole::ScriptConsole(script::CommandDispatcher& dispatcher, QWidget* parent)
    : QWidget(parent)
    , m_dispatcher(dispatcher) {
    setup_ui();
}

void ScriptConsole::setup_ui() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(2);

    m_output_log = new QTextEdit(this);
    m_output_log->setReadOnly(true);
    m_output_log->setStyleSheet("background-color: #0c0e12; color: #d0d4dc; font-family: 'Consolas', monospace; font-size: 10pt;");
    layout->addWidget(m_output_log, 1);

    auto* prompt_layout = new QHBoxLayout();
    auto* prompt_label = new QLabel("soc> ", this);
    prompt_label->setStyleSheet("color: #4CAF50; font-family: 'Consolas', monospace; font-weight: bold; font-size: 10pt;");
    prompt_layout->addWidget(prompt_label);

    m_input_line = new QLineEdit(this);
    m_input_line->setStyleSheet("background-color: #141820; color: #ffffff; font-family: 'Consolas', monospace; font-size: 10pt; border: 1px solid #303642;");
    m_input_line->installEventFilter(this);
    connect(m_input_line, &QLineEdit::returnPressed, this, &ScriptConsole::on_command_entered);
    prompt_layout->addWidget(m_input_line, 1);

    layout->addLayout(prompt_layout);

    append_output("VLSI SoC Studio Scripting Bus (Type 'help' for available commands)");
}

void ScriptConsole::append_output(const QString& text, bool is_error) {
    QString color = is_error ? "#ff5252" : "#98c379";
    m_output_log->append(QString("<span style='color:%1;'>%2</span>").arg(color, text.toHtmlEscaped().replace("\n", "<br>")));
}

void ScriptConsole::on_command_entered() {
    QString cmd = m_input_line->text().trimmed();
    if (cmd.isEmpty()) return;

    m_output_log->append(QString("<span style='color:#61afef;'><b>soc&gt;</b> %1</span>").arg(cmd.toHtmlEscaped()));
    m_history.push_back(cmd);
    m_history_index = -1;
    m_input_line->clear();

    script::CommandResult res = m_dispatcher.execute(cmd.toStdString());
    if (!res.message.empty()) {
        append_output(QString::fromStdString(res.message), !res.success);
    }
}

bool ScriptConsole::eventFilter(QObject* obj, QEvent* event) {
    if (obj == m_input_line && event->type() == QEvent::KeyPress) {
        auto* keyEvent = static_cast<QKeyEvent*>(event);
        if (keyEvent->key() == Qt::Key_Up) {
            if (!m_history.empty()) {
                if (m_history_index == -1) m_history_index = static_cast<int>(m_history.size()) - 1;
                else if (m_history_index > 0) --m_history_index;
                m_input_line->setText(m_history[m_history_index]);
                return true;
            }
        } else if (keyEvent->key() == Qt::Key_Down) {
            if (m_history_index != -1) {
                if (m_history_index < static_cast<int>(m_history.size()) - 1) {
                    ++m_history_index;
                    m_input_line->setText(m_history[m_history_index]);
                } else {
                    m_history_index = -1;
                    m_input_line->clear();
                }
                return true;
            }
        } else if (keyEvent->key() == Qt::Key_Tab) {
            // Autocomplete
            QString current = m_input_line->text();
            std::vector<std::string> matches = m_dispatcher.get_completions(current.toStdString());
            if (matches.size() == 1) {
                m_input_line->setText(QString::fromStdString(matches[0]) + " ");
            } else if (matches.size() > 1) {
                QString list = "Completions: ";
                for (const auto& m : matches) list += QString::fromStdString(m) + "  ";
                append_output(list);
            }
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

} // namespace soc::gui
