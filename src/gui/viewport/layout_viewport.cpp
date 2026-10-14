#include "layout_viewport.hpp"
#include "gl_shaders.hpp"
#include <QPainter>
#include <iostream>

namespace soc::gui {

LayoutViewport::LayoutViewport(db::ChipDB& db, script::CommandDispatcher& dispatcher, QWidget* parent)
    : QOpenGLWidget(parent)
    , m_db(db)
    , m_dispatcher(dispatcher) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    init_display_layers();

    // Register dispatcher callbacks to viewport
    m_dispatcher.set_zoom_callback([this](const db::Rect2D& box) {
        zoom_to_box(box);
    });
    m_dispatcher.set_layer_visibility_callback([this](std::string_view name, bool visible) {
        set_layer_visible(name, visible);
    });
    m_dispatcher.set_highlight_net_callback([this](db::NetId id) {
        highlight_net(id);
    });
    m_dispatcher.set_heatmap_callback([this](const std::string& mode_str) {
        if (mode_str == "power") set_heatmap_mode(HeatmapMode::POWER_DENSITY);
        else if (mode_str == "timing") set_heatmap_mode(HeatmapMode::TIMING_SLACK);
        else if (mode_str == "thermal") set_heatmap_mode(HeatmapMode::THERMAL_3D);
        else if (mode_str == "congestion") set_heatmap_mode(HeatmapMode::CONGESTION);
        else set_heatmap_mode(HeatmapMode::NONE);
    });
}

LayoutViewport::~LayoutViewport() {
    makeCurrent();
    if (m_quad_vbo) glDeleteBuffers(1, &m_quad_vbo);
    if (m_instance_vbo) glDeleteBuffers(1, &m_instance_vbo);
    if (m_vao) glDeleteVertexArrays(1, &m_vao);
    doneCurrent();
}

void LayoutViewport::init_display_layers() {
    m_display_layers.clear();
    // Color palette for 15 metal layers
    const uint32_t layer_colors[15] = {
        0x3388FFCC, // M1: Blue
        0x33CC55CC, // M2: Green
        0xFF8833CC, // M3: Orange
        0xDD33DDCC, // M4: Magenta
        0x33CCCCDD, // M5: Cyan
        0xEEEE33DD, // M6: Yellow
        0x8844FFDD, // M7: Violet
        0xFF4466DD, // M8: Coral
        0x44DD88DD, // M9: Mint
        0xDD8844DD, // M10: Brown
        0x4488EEDD, // M11: Sky
        0xEE4488DD, // M12: Rose
        0x88EE44DD, // M13: Lime
        0x44EEEEEE, // M14: Aqua
        0xEEEEEEEE  // M15: White/Top
    };

    for (size_t i = 0; i < m_db.layers.size(); ++i) {
        db::LayerId lid{static_cast<uint32_t>(i)};
        std::string_view lname = m_db.strings.resolve(m_db.layers[lid].name_id);
        uint32_t col = (i < 15) ? layer_colors[i] : 0xAAAAAACC;
        uint8_t stipple = (i % 3);
        m_display_layers.push_back({lid, std::string(lname), true, true, col, stipple});
    }
}

void LayoutViewport::set_layer_visible(std::string_view name, bool visible) {
    for (auto& lyr : m_display_layers) {
        if (lyr.name == name) {
            lyr.visible = visible;
            break;
        }
    }
    update();
}

void LayoutViewport::set_heatmap_mode(HeatmapMode mode) {
    m_heatmap_mode = mode;
    update();
}

void LayoutViewport::zoom_to_box(const db::Rect2D& box) {
    m_camera.fit_box(box);
    update();
}

void LayoutViewport::fit_all() {
    if (m_db.die_area.is_valid() && m_db.die_area.width() > 0) {
        m_camera.fit_box(m_db.die_area);
    }
    update();
}

void LayoutViewport::select_instance(db::InstanceId id) {
    m_selection.clear();
    m_selection.selected_instance = id;
    emit instanceSelected(id);
    update();
}

void LayoutViewport::highlight_net(db::NetId id) {
    m_selection.clear();
    m_selection.selected_net = id;
    emit netSelected(id);
    update();
}

void LayoutViewport::initializeGL() {
    initializeOpenGLFunctions();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Compile GLSL shaders
    m_program = std::make_unique<QOpenGLShaderProgram>();
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, LAYOUT_VERTEX_SHADER_SRC);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, LAYOUT_FRAGMENT_SHADER_SRC);
    m_program->link();

    // Create VAO
    glGenVertexArrays(1, &m_vao);
    glBindVertexArray(m_vao);

    // Quad geometry (2 triangles)
    float quad_vertices[] = {
        0.0f, 0.0f,
        1.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 0.0f,
        1.0f, 1.0f,
        0.0f, 1.0f
    };

    glGenBuffers(1, &m_quad_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_quad_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertices), quad_vertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);

    // Dynamic Instanced VBO
    glGenBuffers(1, &m_instance_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m_instance_vbo);
    m_instance_buffer_capacity = 120000;
    glBufferData(GL_ARRAY_BUFFER, m_instance_buffer_capacity * sizeof(InstanceDrawData), nullptr, GL_DYNAMIC_DRAW);

    // a_rect (location 1)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(InstanceDrawData), reinterpret_cast<void*>(offsetof(InstanceDrawData, x)));
    glVertexAttribDivisor(1, 1);

    // a_color (location 2)
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(InstanceDrawData), reinterpret_cast<void*>(offsetof(InstanceDrawData, r)));
    glVertexAttribDivisor(2, 1);

    // a_flags (location 3)
    glEnableVertexAttribArray(3);
    glVertexAttribIPointer(3, 1, GL_UNSIGNED_INT, sizeof(InstanceDrawData), reinterpret_cast<void*>(offsetof(InstanceDrawData, flags)));
    glVertexAttribDivisor(3, 1);

    glBindVertexArray(0);

    fit_all();
}

void LayoutViewport::resizeGL(int w, int h) {
    m_camera.set_viewport_size(w, h);
    glViewport(0, 0, w, h);
}

void LayoutViewport::build_render_instances(std::vector<InstanceDrawData>& out_instances) {
    out_instances.clear();
    const db::Rect2D visible_box = m_camera.visible_world_box();

    // 1. Die boundary outline
    if (m_db.die_area.is_valid()) {
        InstanceDrawData die_draw;
        die_draw.x = static_cast<float>(m_db.die_area.x_min);
        die_draw.y = static_cast<float>(m_db.die_area.y_min);
        die_draw.w = static_cast<float>(m_db.die_area.width());
        die_draw.h = static_cast<float>(m_db.die_area.height());
        die_draw.r = 0.85f; die_draw.g = 0.85f; die_draw.b = 0.90f; die_draw.a = 0.85f;
        die_draw.flags = 32u; // outline only
        out_instances.push_back(die_draw);
    }

    // 2. Power Domains (subtle boundary tint)
    for (size_t i = 0; i < m_db.power_domains.size(); ++i) {
        const auto& pd = m_db.power_domains[static_cast<uint32_t>(i)];
        if (!pd.boundary.intersects(visible_box)) continue;

        InstanceDrawData pd_draw;
        pd_draw.x = static_cast<float>(pd.boundary.x_min);
        pd_draw.y = static_cast<float>(pd.boundary.y_min);
        pd_draw.w = static_cast<float>(pd.boundary.width());
        pd_draw.h = static_cast<float>(pd.boundary.height());

        // Color based on voltage
        if (pd.nominal_voltage >= 0.80) {
            pd_draw.r = 0.2f; pd_draw.g = 0.1f; pd_draw.b = 0.4f; pd_draw.a = 0.15f; // CPU
        } else if (pd.nominal_voltage >= 0.70) {
            pd_draw.r = 0.4f; pd_draw.g = 0.1f; pd_draw.b = 0.3f; pd_draw.a = 0.15f; // GPU
        } else {
            pd_draw.r = 0.1f; pd_draw.g = 0.3f; pd_draw.b = 0.2f; pd_draw.a = 0.15f; // SoC
        }
        pd_draw.flags = 0u;
        out_instances.push_back(pd_draw);
    }

    // 3. Block Abstracts (P-Cores, E-Cores, GPU Slices, SLC)
    for (size_t i = 0; i < m_db.block_abstracts.size(); ++i) {
        const auto& ba = m_db.block_abstracts[static_cast<uint32_t>(i)];
        if (!ba.boundary.intersects(visible_box)) continue;

        InstanceDrawData ba_draw;
        ba_draw.x = static_cast<float>(ba.boundary.x_min);
        ba_draw.y = static_cast<float>(ba.boundary.y_min);
        ba_draw.w = static_cast<float>(ba.boundary.width());
        ba_draw.h = static_cast<float>(ba.boundary.height());

        if (ba.macro_type == "P_CORE") {
            ba_draw.r = 0.25f; ba_draw.g = 0.50f; ba_draw.b = 0.95f; ba_draw.a = 0.35f;
        } else if (ba.macro_type == "E_CORE") {
            ba_draw.r = 0.15f; ba_draw.g = 0.75f; ba_draw.b = 0.85f; ba_draw.a = 0.35f;
        } else if (ba.macro_type == "GPU_CORE") {
            ba_draw.r = 0.85f; ba_draw.g = 0.25f; ba_draw.b = 0.75f; ba_draw.a = 0.35f;
        } else {
            ba_draw.r = 0.30f; ba_draw.g = 0.85f; ba_draw.b = 0.40f; ba_draw.a = 0.35f;
        }
        ba_draw.flags = 32u; // Outline
        if (m_selection.selected_block == ba.id) ba_draw.flags |= 16u; // Selected
        out_instances.push_back(ba_draw);
    }

    // 4. Standard Cells with Sub-Pixel Level-of-Detail (LOD) Culling
    std::vector<db::InstanceId> visible_cells = m_db.query_instances_in_box(visible_box);

    for (db::InstanceId id : visible_cells) {
        db::Rect2D bbox = m_db.get_instance_bbox(id);
        float pw = static_cast<float>(bbox.width() * m_camera.scale);
        float ph = static_cast<float>(bbox.height() * m_camera.scale);
        float proj_max = std::max(pw, ph);

        // Sub-Pixel LOD Culling:
        // If the cell's projection is smaller than 2x2 screen pixels,
        // render only its minimal lod box and skip all port pins/obstructions.
        InstanceDrawData cell_draw;
        cell_draw.x = static_cast<float>(bbox.x_min);
        cell_draw.y = static_cast<float>(bbox.y_min);
        cell_draw.w = static_cast<float>(bbox.width());
        cell_draw.h = static_cast<float>(bbox.height());

        if (m_heatmap_mode == HeatmapMode::POWER_DENSITY) {
            // Power Heatmap: gradient from cool blue to hot red
            float norm_p = static_cast<float>((id.value() % 100)) / 100.0f;
            cell_draw.r = norm_p;
            cell_draw.g = 0.2f;
            cell_draw.b = 1.0f - norm_p;
            cell_draw.a = 0.85f;
        } else if (m_heatmap_mode == HeatmapMode::TIMING_SLACK) {
            // Slack Heatmap: negative slack red, positive slack green
            bool slack_violation = (id.value() % 7 == 0);
            if (slack_violation) {
                cell_draw.r = 0.95f; cell_draw.g = 0.15f; cell_draw.b = 0.15f; cell_draw.a = 0.9f;
            } else {
                cell_draw.r = 0.15f; cell_draw.g = 0.85f; cell_draw.b = 0.35f; cell_draw.a = 0.8f;
            }
        } else {
            // Standard cell default color
            cell_draw.r = 0.22f; cell_draw.g = 0.45f; cell_draw.b = 0.65f; cell_draw.a = 0.75f;
        }

        cell_draw.flags = 0u;
        if (m_selection.selected_instance == id) {
            cell_draw.flags |= 16u; // Highlighted
        }

        out_instances.push_back(cell_draw);

        // If cell is zoomed in large enough (> 12 pixels), render internal pin ports
        if (proj_max >= 12.0f) {
            const auto& inst = m_db.instances[id];
            if (inst.master_id.is_valid() && inst.master_id.value() < m_db.master_cells.size()) {
                const auto& master = m_db.master_cells[inst.master_id];
                for (const auto& port : master.ports) {
                    if (port.layer_id.is_valid() && port.layer_id.value() < m_display_layers.size()) {
                        if (!m_display_layers[port.layer_id.value()].visible) continue;
                    }
                    InstanceDrawData port_draw;
                    port_draw.x = static_cast<float>(inst.origin.x + port.box.x_min);
                    port_draw.y = static_cast<float>(inst.origin.y + port.box.y_min);
                    port_draw.w = static_cast<float>(port.box.width());
                    port_draw.h = static_cast<float>(port.box.height());
                    port_draw.r = 0.90f; port_draw.g = 0.80f; port_draw.b = 0.20f; port_draw.a = 0.90f;
                    port_draw.flags = 0u;
                    out_instances.push_back(port_draw);
                }
            }
        }
    }

    // 5. Routing Wire Segments on Visible Metal Layers
    for (size_t i = 0; i < m_db.nets.size(); ++i) {
        db::NetId nid{static_cast<uint32_t>(i)};
        const auto& net = m_db.nets[nid];
        bool is_net_selected = (m_selection.selected_net == nid);

        for (const auto& seg : net.route_segments) {
            if (seg.layer.is_valid() && seg.layer.value() < m_display_layers.size()) {
                const auto& dl = m_display_layers[seg.layer.value()];
                if (!dl.visible && !is_net_selected) continue;

                int32_t hw = std::max(20, seg.width / 2);
                db::Rect2D seg_box{
                    std::min(seg.start.x, seg.end.x) - hw,
                    std::min(seg.start.y, seg.end.y) - hw,
                    std::max(seg.start.x, seg.end.x) + hw,
                    std::max(seg.start.y, seg.end.y) + hw
                };
                if (!seg_box.intersects(visible_box)) continue;

                InstanceDrawData seg_draw;
                seg_draw.x = static_cast<float>(seg_box.x_min);
                seg_draw.y = static_cast<float>(seg_box.y_min);
                seg_draw.w = static_cast<float>(seg_box.width());
                seg_draw.h = static_cast<float>(seg_box.height());

                uint8_t r = (dl.color_rgba >> 24) & 0xFF;
                uint8_t g = (dl.color_rgba >> 16) & 0xFF;
                uint8_t b = (dl.color_rgba >> 8) & 0xFF;
                uint8_t a = dl.color_rgba & 0xFF;
                seg_draw.r = r / 255.0f;
                seg_draw.g = g / 255.0f;
                seg_draw.b = b / 255.0f;
                seg_draw.a = a / 255.0f;

                seg_draw.flags = dl.stipple_mode;
                if (is_net_selected) seg_draw.flags |= 16u; // Highlighted

                out_instances.push_back(seg_draw);
            }
        }
    }
}

void LayoutViewport::paintGL() {
    glClearColor(0.06f, 0.07f, 0.09f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    std::vector<InstanceDrawData> draw_instances;
    draw_instances.reserve(80000);
    build_render_instances(draw_instances);

    if (draw_instances.empty()) {
        render_ruler_overlay();
        return;
    }

    m_program->bind();

    // Set camera uniforms
    m_program->setUniformValue("u_camera_center",
        QVector2D(static_cast<float>(m_camera.center_x), static_cast<float>(m_camera.center_y)));
    m_program->setUniformValue("u_camera_scale",
        QVector2D(static_cast<float>(m_camera.scale), static_cast<float>(m_camera.scale)));
    m_program->setUniformValue("u_viewport_size",
        QVector2D(static_cast<float>(m_camera.viewport_width), static_cast<float>(m_camera.viewport_height)));

    // Reallocate or update instanced VBO
    glBindBuffer(GL_ARRAY_BUFFER, m_instance_vbo);
    if (draw_instances.size() > m_instance_buffer_capacity) {
        m_instance_buffer_capacity = draw_instances.size() * 3 / 2;
        glBufferData(GL_ARRAY_BUFFER, m_instance_buffer_capacity * sizeof(InstanceDrawData),
                     draw_instances.data(), GL_DYNAMIC_DRAW);
    } else {
        glBufferSubData(GL_ARRAY_BUFFER, 0, draw_instances.size() * sizeof(InstanceDrawData),
                        draw_instances.data());
    }

    // Instanced Draw Call
    glBindVertexArray(m_vao);
    glDrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(draw_instances.size()));
    glBindVertexArray(0);

    m_program->release();

    // 2D Overlay Rendering (Ruler crosshairs and measurement labels)
    render_ruler_overlay();
}

void LayoutViewport::render_ruler_overlay() {
    if (!m_ruler.active || !m_ruler.has_start) return;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    auto [s1x, s1y] = m_camera.world_to_screen(m_ruler.p1);
    auto [s2x, s2y] = m_camera.world_to_screen(m_ruler.p2);

    // Crosshairs and dashed line
    QPen pen(QColor(255, 215, 0, 220), 2, Qt::DashLine);
    painter.setPen(pen);
    painter.drawLine(s1x, s1y, s2x, s2y);

    // Draw end crosshair markers
    painter.setPen(QPen(QColor(255, 215, 0), 2));
    painter.drawLine(s1x - 6, s1y, s1x + 6, s1y);
    painter.drawLine(s1x, s1y - 6, s1x, s1y + 6);
    painter.drawLine(s2x - 6, s2y, s2x + 6, s2y);
    painter.drawLine(s2x, s2y - 6, s2x, s2y + 6);

    // Measurement badge
    double dist = m_ruler.distance_um(m_db.dbu_per_micron);
    double dx = m_ruler.delta_x_um(m_db.dbu_per_micron);
    double dy = m_ruler.delta_y_um(m_db.dbu_per_micron);

    QString text = QString("dist: %1 um (dx: %2, dy: %3)")
        .arg(dist, 0, 'f', 2).arg(dx, 0, 'f', 2).arg(dy, 0, 'f', 2);

    int mid_x = (s1x + s2x) / 2;
    int mid_y = (s1y + s2y) / 2 - 10;

    painter.fillRect(mid_x - 4, mid_y - 14, text.length() * 8 + 8, 20, QColor(20, 20, 25, 200));
    painter.setPen(QColor(255, 255, 255));
    painter.drawText(mid_x, mid_y, text);
}

void LayoutViewport::mousePressEvent(QMouseEvent* event) {
    db::Point2D world_pos = m_camera.screen_to_world(event->position().x(), event->position().y());

    if (m_ruler.active) {
        if (!m_ruler.has_start) {
            m_ruler.p1 = world_pos;
            m_ruler.p2 = world_pos;
            m_ruler.has_start = true;
        } else {
            m_ruler.p2 = world_pos;
            emit rulerUpdated(m_ruler.distance_um(m_db.dbu_per_micron),
                              m_ruler.delta_x_um(m_db.dbu_per_micron),
                              m_ruler.delta_y_um(m_db.dbu_per_micron));
            m_ruler.has_start = false; // Finished measurement
        }
        update();
        return;
    }

    if (event->button() == Qt::LeftButton) {
        // Click-picking via R-Tree raycast
        // Tolerance: 5 pixels in world units
        int32_t tol = static_cast<int32_t>(std::max(5.0, 5.0 / m_camera.scale));
        db::Rect2D pick_box{world_pos.x - tol, world_pos.y - tol, world_pos.x + tol, world_pos.y + tol};
        std::vector<db::InstanceId> hits = m_db.query_instances_in_box(pick_box);

        if (!hits.empty()) {
            select_instance(hits.front());
        } else {
            // Check block abstracts
            for (size_t i = 0; i < m_db.block_abstracts.size(); ++i) {
                db::BlockAbstractId ba_id{static_cast<uint32_t>(i)};
                if (m_db.block_abstracts[ba_id].boundary.contains(world_pos)) {
                    m_selection.clear();
                    m_selection.selected_block = ba_id;
                    emit blockSelected(ba_id);
                    update();
                    return;
                }
            }
            m_selection.clear();
            update();
        }
    } else if (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton) {
        m_is_panning = true;
        m_last_mouse_pos = event->pos();
    }
}

void LayoutViewport::mouseMoveEvent(QMouseEvent* event) {
    db::Point2D world_pos = m_camera.screen_to_world(event->position().x(), event->position().y());
    emit cursorWorldMoved(world_pos);

    if (m_is_panning) {
        QPoint delta = event->pos() - m_last_mouse_pos;
        m_camera.pan_pixels(delta.x(), delta.y());
        m_last_mouse_pos = event->pos();
        update();
    } else if (m_ruler.active && m_ruler.has_start) {
        m_ruler.p2 = world_pos;
        emit rulerUpdated(m_ruler.distance_um(m_db.dbu_per_micron),
                          m_ruler.delta_x_um(m_db.dbu_per_micron),
                          m_ruler.delta_y_um(m_db.dbu_per_micron));
        update();
    }
}

void LayoutViewport::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton) {
        m_is_panning = false;
    }
}

void LayoutViewport::wheelEvent(QWheelEvent* event) {
    double factor = (event->angleDelta().y() > 0) ? 1.25 : 0.80;
    m_camera.zoom_at(event->position().x(), event->position().y(), factor);
    update();
}

void LayoutViewport::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_K) {
        m_ruler.active = !m_ruler.active;
        m_ruler.has_start = false;
        update();
    } else if (event->key() == Qt::Key_F) {
        fit_all();
    } else {
        QOpenGLWidget::keyPressEvent(event);
    }
}

} // namespace soc::gui
