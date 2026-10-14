#pragma once

#include "core/db/id_types.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

namespace soc::gui {

enum class HeatmapMode : uint8_t {
    NONE = 0,
    POWER_DENSITY,
    TIMING_SLACK,
    THERMAL_3D,
    CONGESTION
};

struct DisplayLayer {
    db::LayerId layer_id{db::LayerId::invalid()};
    std::string name;
    bool visible{true};
    bool selectable{true};
    uint32_t color_rgba{0xFFFFFFFF}; // 0xRRGGBBAA
    uint8_t stipple_mode{0}; // 0 = solid, 1 = diagonal cross, 2 = dots, 3 = border-only
};

// 2D Camera supporting smooth cursor-centered zoom and pan
class Camera2D {
public:
    double center_x{5000000.0}; // DBU
    double center_y{5000000.0}; // DBU
    double scale{0.0001};       // pixels per DBU (zoom level)
    int viewport_width{1280};
    int viewport_height{800};

    Camera2D() = default;

    void set_viewport_size(int w, int h) noexcept {
        viewport_width = std::max(1, w);
        viewport_height = std::max(1, h);
    }

    // Convert screen pixel coordinate to world DBU coordinate
    [[nodiscard]] db::Point2D screen_to_world(int sx, int sy) const noexcept {
        double dx = (static_cast<double>(sx) - viewport_width * 0.5) / scale;
        double dy = (viewport_height * 0.5 - static_cast<double>(sy)) / scale; // Inverted Y in OpenGL/screen
        return db::Point2D{
            static_cast<int32_t>(std::round(center_x + dx)),
            static_cast<int32_t>(std::round(center_y + dy))
        };
    }

    // Convert world DBU coordinate to screen pixel coordinate
    [[nodiscard]] std::pair<int, int> world_to_screen(db::Point2D p) const noexcept {
        double sx = (p.x - center_x) * scale + viewport_width * 0.5;
        double sy = viewport_height * 0.5 - (p.y - center_y) * scale;
        return {static_cast<int>(std::round(sx)), static_cast<int>(std::round(sy))};
    }

    // World bounding box currently visible in viewport
    [[nodiscard]] db::Rect2D visible_world_box() const noexcept {
        db::Point2D bot_left = screen_to_world(0, viewport_height);
        db::Point2D top_right = screen_to_world(viewport_width, 0);
        return db::Rect2D{bot_left.x, bot_left.y, top_right.x, top_right.y};
    }

    // Smooth cursor-centered zoom
    // Keeps the world coordinate under the cursor stationary during scale change
    void zoom_at(int cursor_sx, int cursor_sy, double factor) noexcept {
        if (factor <= 0.0) return;
        db::Point2D world_before = screen_to_world(cursor_sx, cursor_sy);

        scale *= factor;
        // Clamp scale to reasonable limits
        if (scale < 1e-6) scale = 1e-6;
        if (scale > 10.0) scale = 10.0;

        db::Point2D world_after = screen_to_world(cursor_sx, cursor_sy);
        center_x += (world_before.x - world_after.x);
        center_y += (world_before.y - world_after.y);
    }

    // Pan camera by screen pixel delta
    void pan_pixels(int dx_pixels, int dy_pixels) noexcept {
        center_x -= dx_pixels / scale;
        center_y += dy_pixels / scale;
    }

    // Fit camera frustum to target bounding box
    void fit_box(const db::Rect2D& box) noexcept {
        if (!box.is_valid() || box.width() <= 0 || box.height() <= 0) return;
        center_x = box.x_min + box.width() * 0.5;
        center_y = box.y_min + box.height() * 0.5;

        double scale_x = static_cast<double>(viewport_width) / box.width();
        double scale_y = static_cast<double>(viewport_height) / box.height();
        scale = std::min(scale_x, scale_y) * 0.90; // 10% margin
        if (scale < 1e-6) scale = 1e-6;
    }
};

// Interactive measurement ruler (toggled with 'K' key)
struct Ruler {
    bool active{false};
    bool has_start{false};
    db::Point2D p1{0, 0};
    db::Point2D p2{0, 0};

    [[nodiscard]] double delta_x_um(uint32_t dbu_per_micron = 1000) const noexcept {
        return std::abs(p2.x - p1.x) / static_cast<double>(dbu_per_micron);
    }

    [[nodiscard]] double delta_y_um(uint32_t dbu_per_micron = 1000) const noexcept {
        return std::abs(p2.y - p1.y) / static_cast<double>(dbu_per_micron);
    }

    [[nodiscard]] double distance_um(uint32_t dbu_per_micron = 1000) const noexcept {
        double dx = delta_x_um(dbu_per_micron);
        double dy = delta_y_um(dbu_per_micron);
        return std::sqrt(dx * dx + dy * dy);
    }

    void reset() noexcept {
        active = false;
        has_start = false;
        p1 = {0, 0};
        p2 = {0, 0};
    }
};

// Selection State for Viewport & Inspector
struct SelectionState {
    db::InstanceId      selected_instance{db::InstanceId::invalid()};
    db::NetId           selected_net{db::NetId::invalid()};
    db::BlockAbstractId selected_block{db::BlockAbstractId::invalid()};

    void clear() noexcept {
        selected_instance = db::InstanceId::invalid();
        selected_net = db::NetId::invalid();
        selected_block = db::BlockAbstractId::invalid();
    }

    [[nodiscard]] bool has_selection() const noexcept {
        return selected_instance.is_valid() || selected_net.is_valid() || selected_block.is_valid();
    }
};

// Critical Timing Path Overlay rendered in bright orange/red
struct TimingPathOverlay {
    bool visible{false};
    std::vector<db::Point2D> path_points; // Path pin coordinates
    std::vector<db::Rect2D>  path_boxes;  // Pin/Instance bounding boxes
    double slack_ps{0.0};
};

} // namespace soc::gui
