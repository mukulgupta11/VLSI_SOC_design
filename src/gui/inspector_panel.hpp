#pragma once

#include "core/db/chip_db.hpp"
#include <QWidget>
#include <QTableWidget>
#include <QVBoxLayout>

namespace soc::gui {

class InspectorPanel : public QWidget {
    Q_OBJECT

public:
    explicit InspectorPanel(db::ChipDB& db, QWidget* parent = nullptr);
    ~InspectorPanel() override = default;

    void inspect_instance(db::InstanceId id);
    void inspect_block(db::BlockAbstractId id);
    void inspect_net(db::NetId id);
    void clear();

private:
    db::ChipDB& m_db;
    QTableWidget* m_table{nullptr};

    void add_row(const QString& property, const QString& value);
};

} // namespace soc::gui
