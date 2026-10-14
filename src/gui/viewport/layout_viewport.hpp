#pragma once

#include "../gui_types.hpp"
#include "core/db/chip_db.hpp"
#include "core/script/command_dispatcher.hpp"

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <memory>
#include <vector>

namespace soc::gui {

class LayoutViewport : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core {
    Q_OBJECT

public:
    explicit LayoutViewport(db::ChipDB& db, script::CommandDispatcher& dispatcher, QWidget* parent = nullptr);
    ~LayoutViewport() override;

    [[nodiscard]] Camera2D& camera() noexcept { return m_camera; }
    [[nodiscard]] const Camera2D& camera() const noexcept { return m_camera; }

    [[nodiscard]] Ruler& ruler() noexcept { return m_ruler; }
    [[nodiscard]] const Ruler& ruler() const noexcept { return m_ruler; }

    [[nodiscard]] SelectionState& selection() noexcept { return m_selection; }
    [[nodiscard]] const SelectionState& selection() const noexcept { return m_selection; }

    void set_layer_visible(std::string_view name, bool visible);
    void set_heatmap_mode(HeatmapMode mode);
    void zoom_to_box(const db::Rect2D& box);
    void fit_all();
    void select_instance(db::InstanceId id);
    void highlight_net(db::NetId id);

signals:
    void instanceSelected(db::InstanceId id);
    void netSelected(db::NetId id);
    void blockSelected(db::BlockAbstractId id);
    void cursorWorldMoved(db::Point2D world_pos);
    void rulerUpdated(double dist_um, double dx_um, double dy_um);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    struct InstanceDrawData {
        float x{0.0f}, y{0.0f}, w{0.0f}, h{0.0f};
        float r{1.0f}, g{1.0f}, b{1.0f}, a{1.0f};
        uint32_t flags{0};
    };

    db::ChipDB& m_db;
    script::CommandDispatcher& m_dispatcher;

    Camera2D        m_camera;
    Ruler           m_ruler;
    SelectionState  m_selection;
    HeatmapMode     m_heatmap_mode{HeatmapMode::NONE};

    std::vector<DisplayLayer> m_display_layers;

    // OpenGL Objects
    std::unique_ptr<QOpenGLShaderProgram> m_program;
    GLuint m_vao{0};
    GLuint m_quad_vbo{0};
    GLuint m_instance_vbo{0};
    size_t m_instance_buffer_capacity{0};

    // Pan interaction state
    bool m_is_panning{false};
    QPoint m_last_mouse_pos;

    void init_display_layers();
    void build_render_instances(std::vector<InstanceDrawData>& out_instances);
    void render_ruler_overlay();
};

} // namespace soc::gui
