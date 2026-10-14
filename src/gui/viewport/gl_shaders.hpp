#pragma once

namespace soc::gui {

// GLSL Shaders for 60 FPS OpenGL 4.6 Core instanced layout rendering
inline constexpr const char* LAYOUT_VERTEX_SHADER_SRC = R"glsl(#version 450 core

layout(location = 0) in vec2 a_quad_pos; // Unit quad vertex: (0,0), (1,0), (1,1)...

// Per-instance attributes (instanced array with divisor = 1)
layout(location = 1) in vec4 a_rect;     // (x, y, width, height) in world DBU
layout(location = 2) in vec4 a_color;    // (r, g, b, a) normalized [0, 1]
layout(location = 3) in uint a_flags;    // bit0..2: stipple mode, bit4: selected, bit5: wire/outline

// Uniform camera parameters
uniform vec2 u_camera_center; // World DBU center
uniform vec2 u_camera_scale;  // Pixels per DBU
uniform vec2 u_viewport_size; // Viewport width, height in pixels

out vec2 v_local_uv;
out vec4 v_color;
flat out uint v_flags;

void main() {
    v_local_uv = a_quad_pos;
    v_color = a_color;
    v_flags = a_flags;

    // Calculate world position
    vec2 world_pos = a_rect.xy + a_quad_pos * a_rect.zw;

    // Transform world to screen coordinates (pixels relative to viewport center)
    vec2 screen_delta = (world_pos - u_camera_center) * u_camera_scale;

    // Convert to Normalized Device Coordinates (NDC) in [-1, 1]
    vec2 ndc = vec2(
        screen_delta.x / (u_viewport_size.x * 0.5),
        screen_delta.y / (u_viewport_size.y * 0.5)
    );

    gl_Position = vec4(ndc, 0.0, 1.0);
}
)glsl";

inline constexpr const char* LAYOUT_FRAGMENT_SHADER_SRC = R"glsl(#version 450 core

in vec2 v_local_uv;
in vec4 v_color;
flat in uint v_flags;

out vec4 frag_color;

void main() {
    uint stipple_mode = v_flags & 7u;
    bool is_selected = (v_flags & 16u) != 0u;
    bool is_outline_only = (v_flags & 32u) != 0u;

    vec4 base_color = v_color;

    // Selection highlight: golden glow
    if (is_selected) {
        base_color = mix(base_color, vec4(1.0, 0.84, 0.0, 1.0), 0.65);
    }

    // Border outline detection
    float border_w = 0.04;
    bool is_border = (v_local_uv.x < border_w || v_local_uv.x > (1.0 - border_w) ||
                      v_local_uv.y < border_w || v_local_uv.y > (1.0 - border_w));

    if (is_outline_only) {
        if (!is_border) {
            discard;
        }
        frag_color = vec4(base_color.rgb, 1.0);
        return;
    }

    // Stipple pattern simulation based on screen pixel coordinates
    if (stipple_mode == 1u) {
        // Diagonal cross-hatch stipple
        int diag = int(gl_FragCoord.x + gl_FragCoord.y) % 6;
        if (diag > 2 && !is_border) {
            discard;
        }
    } else if (stipple_mode == 2u) {
        // Dot stipple
        int dx = int(gl_FragCoord.x) % 4;
        int dy = int(gl_FragCoord.y) % 4;
        if ((dx != 0 || dy != 0) && !is_border) {
            discard;
        }
    }

    // Slightly darken borders for geometric clarity
    if (is_border) {
        frag_color = vec4(base_color.rgb * 0.75, 1.0);
    } else {
        frag_color = base_color;
    }
}
)glsl";

} // namespace soc::gui
