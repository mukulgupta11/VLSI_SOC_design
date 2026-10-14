#pragma once

#include <cstdint>
#include <compare>
#include <algorithm>
#include <limits>
#include <string>

namespace soc::db {

// Generic strongly-typed 32-bit handle for zero 64-bit pointer overhead
template<typename Tag>
struct ID {
    uint32_t index{0xFFFFFFFF};

    constexpr ID() noexcept = default;
    constexpr explicit ID(uint32_t idx) noexcept : index(idx) {}

    [[nodiscard]] constexpr bool is_valid() const noexcept { return index != 0xFFFFFFFF; }
    [[nodiscard]] constexpr uint32_t value() const noexcept { return index; }
    constexpr explicit operator uint32_t() const noexcept { return index; }

    constexpr bool operator==(const ID& o) const noexcept = default;
    constexpr auto operator<=>(const ID& o) const noexcept = default;

    static constexpr ID invalid() noexcept { return ID{0xFFFFFFFF}; }
};

// Strongly-typed entities
using MasterCellId    = ID<struct MasterCellTag>;
using InstanceId      = ID<struct InstanceTag>;
using NetId           = ID<struct NetTag>;
using PinId           = ID<struct PinTag>;
using LayerId         = ID<struct LayerTag>;
using PowerDomainId   = ID<struct PowerDomainTag>;
using BlockAbstractId = ID<struct BlockAbstractTag>;
using ViaId           = ID<struct ViaTag>;
using TrackId         = ID<struct TrackTag>;
using StringId        = ID<struct StringTag>;

// Physical orientations
enum class Orientation : uint8_t {
    R0 = 0,    // North (default)
    R90,       // West
    R180,      // South
    R270,      // East
    MY,        // Flipped North (mirrored about Y axis)
    MY90,      // Flipped West
    MX,        // Flipped South (mirrored about X axis)
    MX90       // Flipped East
};

[[nodiscard]] constexpr const char* to_string(Orientation o) noexcept {
    switch (o) {
        case Orientation::R0:   return "N";
        case Orientation::R90:  return "W";
        case Orientation::R180: return "S";
        case Orientation::R270: return "E";
        case Orientation::MY:   return "FN";
        case Orientation::MY90: return "FW";
        case Orientation::MX:   return "FS";
        case Orientation::MX90: return "FE";
        default:                return "N";
    }
}

// Net and pin signal semantics
enum class SignalType : uint8_t {
    SIGNAL = 0,
    CLOCK,
    POWER,
    GROUND,
    ANALOG,
    RESET,
    SCAN
};

// Master cell classifications
enum class CellType : uint8_t {
    STD_CELL = 0,
    MACRO,
    BLOCK_ABSTRACT,
    IO_PAD,
    LEVEL_SHIFTER,
    ISOLATION_CELL,
    POWER_SWITCH,
    RETENTION_FF,
    TSV
};

// 2D Point in integer Database Units (DBU)
struct Point2D {
    int32_t x{0};
    int32_t y{0};

    constexpr Point2D() noexcept = default;
    constexpr Point2D(int32_t px, int32_t py) noexcept : x(px), y(py) {}

    constexpr bool operator==(const Point2D& o) const noexcept = default;
    constexpr auto operator<=>(const Point2D& o) const noexcept = default;

    constexpr Point2D operator+(const Point2D& o) const noexcept { return {x + o.x, y + o.y}; }
    constexpr Point2D operator-(const Point2D& o) const noexcept { return {x - o.x, y - o.y}; }
};

// 2D Bounding Box in integer Database Units (DBU)
struct Rect2D {
    int32_t x_min{0};
    int32_t y_min{0};
    int32_t x_max{0};
    int32_t y_max{0};

    constexpr Rect2D() noexcept = default;
    constexpr Rect2D(int32_t x1, int32_t y1, int32_t x2, int32_t y2) noexcept
        : x_min(std::min(x1, x2)), y_min(std::min(y1, y2)),
          x_max(std::max(x1, x2)), y_max(std::max(y1, y2)) {}

    constexpr bool operator==(const Rect2D& o) const noexcept = default;

    [[nodiscard]] constexpr int32_t width() const noexcept { return x_max - x_min; }
    [[nodiscard]] constexpr int32_t height() const noexcept { return y_max - y_min; }
    [[nodiscard]] constexpr int64_t area() const noexcept {
        return static_cast<int64_t>(width()) * static_cast<int64_t>(height());
    }

    [[nodiscard]] constexpr bool is_valid() const noexcept {
        return x_max >= x_min && y_max >= y_min;
    }

    [[nodiscard]] constexpr bool intersects(const Rect2D& o) const noexcept {
        return !(x_max < o.x_min || x_min > o.x_max || y_max < o.y_min || y_min > o.y_max);
    }

    [[nodiscard]] constexpr bool contains(const Point2D& p) const noexcept {
        return p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max;
    }

    [[nodiscard]] constexpr bool contains(const Rect2D& o) const noexcept {
        return o.x_min >= x_min && o.x_max <= x_max && o.y_min >= y_min && o.y_max <= y_max;
    }

    constexpr void merge(const Rect2D& o) noexcept {
        x_min = std::min(x_min, o.x_min);
        y_min = std::min(y_min, o.y_min);
        x_max = std::max(x_max, o.x_max);
        y_max = std::max(y_max, o.y_max);
    }

    constexpr void merge(const Point2D& p) noexcept {
        x_min = std::min(x_min, p.x);
        y_min = std::min(y_min, p.y);
        x_max = std::max(x_max, p.x);
        y_max = std::max(y_max, p.y);
    }

    [[nodiscard]] constexpr Rect2D translated(Point2D offset) const noexcept {
        return {x_min + offset.x, y_min + offset.y, x_max + offset.x, y_max + offset.y};
    }

    [[nodiscard]] constexpr Point2D center() const noexcept {
        return {x_min + width() / 2, y_min + height() / 2};
    }
};

} // namespace soc::db
