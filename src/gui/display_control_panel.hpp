#pragma once

#include "core/db/chip_db.hpp"
#include "gui_types.hpp"
#include <QWidget>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QScrollArea>
#include <vector>

namespace soc::gui {

class LayoutViewport;

class DisplayControlPanel : public QWidget {
    Q_OBJECT

public:
    explicit DisplayControlPanel(db::ChipDB& db, LayoutViewport& viewport, QWidget* parent = nullptr);
    ~DisplayControlPanel() override = default;

    void populate_layers();

signals:
    void layerVisibilityToggled(std::string_view name, bool visible);
    void heatmapModeChanged(HeatmapMode mode);

private:
    db::ChipDB& m_db;
    LayoutViewport& m_viewport;

    QVBoxLayout* m_layers_layout{nullptr};
    std::vector<QCheckBox*> m_layer_checkboxes;
    QComboBox* m_heatmap_combo{nullptr};
    QPushButton* m_ruler_btn{nullptr};
};

} // namespace soc::gui
