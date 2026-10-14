#include "hierarchy_browser.hpp"
#include <QHeaderView>

namespace soc::gui {

HierarchyBrowser::HierarchyBrowser(db::ChipDB& db, QWidget* parent)
    : QWidget(parent)
    , m_db(db) {
    setup_ui();
    refresh();
}

void HierarchyBrowser::setup_ui() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);

    m_search_filter = new QLineEdit(this);
    m_search_filter->setPlaceholderText("Filter instances / blocks...");
    connect(m_search_filter, &QLineEdit::textChanged, this, &HierarchyBrowser::populate_tree);
    layout->addWidget(m_search_filter);

    m_tree = new QTreeWidget(this);
    m_tree->setHeaderLabels({"Hierarchy Entity", "Type / Spec"});
    m_tree->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    layout->addWidget(m_tree, 1);

    connect(m_tree, &QTreeWidget::itemDoubleClicked, [this](QTreeWidgetItem* item, int /*column*/) {
        uint32_t type = item->data(0, Qt::UserRole).toUInt();
        uint32_t id_val = item->data(1, Qt::UserRole).toUInt();

        if (type == 1) { // Block Abstract
            db::BlockAbstractId ba_id{id_val};
            emit blockSelected(ba_id);
            if (ba_id.is_valid() && ba_id.value() < m_db.block_abstracts.size()) {
                emit zoomRequested(m_db.block_abstracts[ba_id].boundary);
            }
        } else if (type == 2) { // Instance
            db::InstanceId inst_id{id_val};
            emit instanceSelected(inst_id);
            if (inst_id.is_valid() && inst_id.value() < m_db.instances.size()) {
                emit zoomRequested(m_db.get_instance_bbox(inst_id));
            }
        }
    });
}

void HierarchyBrowser::refresh() {
    populate_tree(m_search_filter ? m_search_filter->text() : "");
}

void HierarchyBrowser::populate_tree(const QString& filter) {
    m_tree->clear();

    // 1. Block Abstracts Category
    auto* blocks_root = new QTreeWidgetItem(m_tree, {"Hierarchical Blocks", QString::number(m_db.block_abstracts.size())});
    blocks_root->setExpanded(true);

    for (size_t i = 0; i < m_db.block_abstracts.size(); ++i) {
        db::BlockAbstractId ba_id{static_cast<uint32_t>(i)};
        const auto& ba = m_db.block_abstracts[ba_id];
        std::string_view name = m_db.strings.resolve(ba.name_id);
        QString qname = QString::fromUtf8(name.data(), name.size());

        if (!filter.isEmpty() && !qname.contains(filter, Qt::CaseInsensitive)) {
            continue;
        }

        auto* item = new QTreeWidgetItem(blocks_root, {qname, QString::fromStdString(ba.macro_type)});
        item->setData(0, Qt::UserRole, 1); // Type 1: Block Abstract
        item->setData(1, Qt::UserRole, ba_id.value());
    }

    // 2. Voltage Domains Category
    auto* domains_root = new QTreeWidgetItem(m_tree, {"Voltage Domains", QString::number(m_db.power_domains.size())});
    domains_root->setExpanded(true);

    for (size_t i = 0; i < m_db.power_domains.size(); ++i) {
        db::PowerDomainId pd_id{static_cast<uint32_t>(i)};
        const auto& pd = m_db.power_domains[pd_id];
        std::string_view name = m_db.strings.resolve(pd.name_id);
        QString qname = QString::fromUtf8(name.data(), name.size());
        QString vspec = QString("%1 V").arg(pd.nominal_voltage, 0, 'f', 2);

        auto* item = new QTreeWidgetItem(domains_root, {qname, vspec});
        item->setData(0, Qt::UserRole, 3); // Type 3: Power Domain
        item->setData(1, Qt::UserRole, pd_id.value());
    }

    // 3. Standard Cell Instances Category (display first 1000 matching items for performance)
    auto* insts_root = new QTreeWidgetItem(m_tree, {"Instances", QString("%1 cells").arg(m_db.instances.size())});
    size_t shown = 0;
    for (size_t i = 0; i < m_db.instances.size() && shown < 500; ++i) {
        db::InstanceId id{static_cast<uint32_t>(i)};
        const auto& inst = m_db.instances[id];
        std::string_view iname = m_db.strings.resolve(inst.name_id);
        QString qname = QString::fromUtf8(iname.data(), iname.size());

        if (!filter.isEmpty() && !qname.contains(filter, Qt::CaseInsensitive)) {
            continue;
        }

        std::string_view mname = "";
        if (inst.master_id.is_valid() && inst.master_id.value() < m_db.master_cells.size()) {
            mname = m_db.strings.resolve(m_db.master_cells[inst.master_id].name_id);
        }

        auto* item = new QTreeWidgetItem(insts_root, {qname, QString::fromUtf8(mname.data(), mname.size())});
        item->setData(0, Qt::UserRole, 2); // Type 2: Instance
        item->setData(1, Qt::UserRole, id.value());
        ++shown;
    }
}

} // namespace soc::gui
