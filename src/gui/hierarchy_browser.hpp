#pragma once

#include "core/db/chip_db.hpp"
#include <QWidget>
#include <QTreeWidget>
#include <QLineEdit>
#include <QVBoxLayout>

namespace soc::gui {

class HierarchyBrowser : public QWidget {
    Q_OBJECT

public:
    explicit HierarchyBrowser(db::ChipDB& db, QWidget* parent = nullptr);
    ~HierarchyBrowser() override = default;

    void refresh();

signals:
    void instanceSelected(db::InstanceId id);
    void blockSelected(db::BlockAbstractId id);
    void zoomRequested(const db::Rect2D& box);

private:
    db::ChipDB& m_db;
    QLineEdit* m_search_filter{nullptr};
    QTreeWidget* m_tree{nullptr};

    void setup_ui();
    void populate_tree(const QString& filter = "");
};

} // namespace soc::gui
