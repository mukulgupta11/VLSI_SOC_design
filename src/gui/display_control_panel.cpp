#include "display_control_panel.hpp"
#include "viewport/layout_viewport.hpp"
#include <QGroupBox>
#include <QLabel>

namespace soc::gui {

DisplayControlPanel::DisplayControlPanel(db::ChipDB& db, LayoutViewport& viewport, QWidget* parent)
    : QWidget(parent)
    , m_db(db)
    , m_viewport(viewport) {
    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(4, 4, 4, 4);

    // 1. Heatmap Mode Selector
    auto* heatmap_group = new QGroupBox("Heatmap Overlays", this);
    auto* heatmap_layout = new QVBoxLayout(heatmap_group);
    m_heatmap_combo = new QComboBox(heatmap_group);
    m_heatmap_combo->addItem("None (Standard Layout)", static_cast<int>(HeatmapMode::NONE));
    m_heatmap_combo->addItem("Power Density (Dynamic & Leakage)", static_cast<int>(HeatmapMode::POWER_DENSITY));
    m_heatmap_combo->addItem("Timing Slack (Critical Paths)", static_cast<int>(HeatmapMode::TIMING_SLACK));
    m_heatmap_combo->addItem("3D Thermal Heatmap", static_cast<int>(HeatmapMode::THERMAL_3D));
    m_heatmap_combo->addItem("Routing Congestion", static_cast<int>(HeatmapMode::CONGESTION));

    connect(m_heatmap_combo, &QComboBox::currentIndexChanged, [this](int idx) {
        auto mode = static_cast<HeatmapMode>(m_heatmap_combo->itemData(idx).toInt());
        m_viewport.set_heatmap_mode(mode);
    });
    heatmap_layout->addWidget(m_heatmap_combo);
    main_layout->addWidget(heatmap_group);

    // 2. Interactive Tools
    auto* tools_group = new QGroupBox("Tools & Navigation", this);
    auto* tools_layout = new QVBoxLayout(tools_group);
    m_ruler_btn = new QPushButton("Toggle Ruler [K]", tools_group);
    connect(m_ruler_btn, &QPushButton::clicked, [this]() {
        m_viewport.ruler().active = !m_viewport.ruler().active;
        m_viewport.ruler().has_start = false;
        m_viewport.update();
    });
    tools_layout->addWidget(m_ruler_btn);

    auto* fit_btn = new QPushButton("Fit Design [F]", tools_group);
    connect(fit_btn, &QPushButton::clicked, [this]() {
        m_viewport.fit_all();
    });
    tools_layout->addWidget(fit_btn);
    main_layout->addWidget(tools_group);

    // 3. Physical Metal Routing Layers
    auto* layers_group = new QGroupBox("Metal Routing Layers", this);
    auto* scroll = new QScrollArea(layers_group);
    scroll->setWidgetResizable(true);
    auto* scroll_widget = new QWidget();
    m_layers_layout = new QVBoxLayout(scroll_widget);

    populate_layers();

    scroll->setWidget(scroll_widget);
    auto* group_layout = new QVBoxLayout(layers_group);
    group_layout->addWidget(scroll);
    main_layout->addWidget(layers_group, 1);
}

void DisplayControlPanel::populate_layers() {
    m_layer_checkboxes.clear();
    for (size_t i = 0; i < m_db.layers.size(); ++i) {
        db::LayerId lid{static_cast<uint32_t>(i)};
        std::string_view lname = m_db.strings.resolve(m_db.layers[lid].name_id);
        auto* cb = new QCheckBox(QString::fromUtf8(lname.data(), lname.size()));
        cb->setChecked(true);

        std::string layer_str(lname);
        connect(cb, &QCheckBox::toggled, [this, layer_str](bool checked) {
            m_viewport.set_layer_visible(layer_str, checked);
        });

        m_layers_layout->addWidget(cb);
        m_layer_checkboxes.push_back(cb);
    }
}

} // namespace soc::gui
