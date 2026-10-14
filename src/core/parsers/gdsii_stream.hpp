#pragma once

#include "../db/chip_db.hpp"
#include <string>
#include <vector>
#include <cstdint>
#include <fstream>
#include <string_view>

namespace soc::parsers {

// Standard GDSII Record Types (Release 6.0)
enum class GdsiiRecordType : uint8_t {
    HEADER    = 0x00,
    BGNLIB    = 0x01,
    LIBNAME   = 0x02,
    UNITS     = 0x03,
    ENDLIB    = 0x04,
    BGNSTR    = 0x05,
    STRNAME   = 0x06,
    ENDSTR    = 0x07,
    BOUNDARY  = 0x08,
    PATH      = 0x09,
    SREF      = 0x0A,
    AREF      = 0x0B,
    TEXT      = 0x0C,
    LAYER     = 0x0D,
    DATATYPE  = 0x0E,
    WIDTH     = 0x0F,
    XY        = 0x10,
    ENDEL     = 0x11,
    SNAME     = 0x12,
    COLROW    = 0x13,
    TEXTNODE  = 0x14,
    NODE      = 0x15,
    TEXTTYPE  = 0x16,
    PRESENTATION = 0x17,
    STRING    = 0x19,
    STRANS    = 0x1A,
    MAG       = 0x1B,
    ANGLE     = 0x1C,
    BOX       = 0x2D,
    BOXTYPE   = 0x2E
};

#ifdef NO_DATA
#undef NO_DATA
#endif

enum class GdsiiDataType : uint8_t {
    NONE      = 0x00,
    BIT_ARRAY = 0x01,
    INT_16    = 0x02,
    INT_32    = 0x03,
    REAL_32   = 0x04,
    REAL_64   = 0x05,
    ASCII     = 0x06
};

// Raw GDSII Record
struct GdsiiRecord {
    GdsiiRecordType type{GdsiiRecordType::HEADER};
    GdsiiDataType data_type{GdsiiDataType::NONE};
    std::vector<uint8_t> data;
};

// Binary GDSII Stream Writer
class GdsiiWriter {
public:
    explicit GdsiiWriter(const std::string& filepath);
    ~GdsiiWriter();

    void write_header(int16_t version = 600);
    void write_bgnlib(std::string_view lib_name, double user_unit = 0.001, double db_unit_meters = 1e-9);
    void write_bgnstr(std::string_view cell_name);
    void write_endstr();
    void write_endlib();

    void write_boundary(uint16_t layer, uint16_t datatype, const std::vector<::soc::db::Point2D>& points);
    void write_box(uint16_t layer, uint16_t datatype, const ::soc::db::Rect2D& box);
    void write_sref(std::string_view target_cell, ::soc::db::Point2D origin, ::soc::db::Orientation orient = ::soc::db::Orientation::R0);

    // Export entire ChipDB to binary GDSII
    static bool export_chip_db(const ::soc::db::ChipDB& chip_db, const std::string& filepath);

    [[nodiscard]] bool is_open() const noexcept { return m_stream.is_open(); }
    void close();

private:
    std::ofstream m_stream;

    void write_record_header(GdsiiRecordType type, GdsiiDataType data_type, uint16_t length);
    void write_int16(int16_t val);
    void write_int32(int32_t val);
    void write_real64(double val);
    void write_string(std::string_view str);

    static uint64_t double_to_gdsii_real(double val) noexcept;
};

// Parsed GDSII Element representation
struct GdsiiBoundary {
    uint16_t layer{0};
    uint16_t datatype{0};
    std::vector<::soc::db::Point2D> points;
};

struct GdsiiSRef {
    std::string target_structure;
    ::soc::db::Point2D origin{0, 0};
    double angle{0.0};
    bool mirrored{false};
};

struct GdsiiStructure {
    std::string name;
    std::vector<GdsiiBoundary> boundaries;
    std::vector<GdsiiSRef> srefs;
};

// Binary GDSII Stream Reader
class GdsiiReader {
public:
    explicit GdsiiReader(const std::string& filepath);
    ~GdsiiReader() = default;

    bool parse();
    [[nodiscard]] const std::vector<GdsiiStructure>& structures() const noexcept { return m_structures; }
    [[nodiscard]] std::string_view library_name() const noexcept { return m_libname; }
    [[nodiscard]] double user_unit() const noexcept { return m_user_unit; }
    [[nodiscard]] double db_unit_meters() const noexcept { return m_db_unit_meters; }

    static double gdsii_real_to_double(uint64_t gds_real) noexcept;

private:
    std::string m_filepath;
    std::string m_libname;
    double m_user_unit{0.001};
    double m_db_unit_meters{1e-9};
    std::vector<GdsiiStructure> m_structures;
};

} // namespace soc::parsers
