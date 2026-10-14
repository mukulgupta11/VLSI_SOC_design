#include "gdsii_stream.hpp"
#include <cmath>
#include <iostream>
#include <cstring>
#include <ctime>

namespace soc::parsers {

GdsiiWriter::GdsiiWriter(const std::string& filepath)
    : m_stream(filepath, std::ios::binary | std::ios::out | std::ios::trunc) {}

GdsiiWriter::~GdsiiWriter() {
    close();
}

void GdsiiWriter::close() {
    if (m_stream.is_open()) {
        m_stream.flush();
        m_stream.close();
    }
}

uint64_t GdsiiWriter::double_to_gdsii_real(double val) noexcept {
    if (val == 0.0) return 0ULL;

    bool negative = (val < 0.0);
    double abs_val = std::abs(val);

    int exp = 0;
    double normalized = abs_val;
    while (normalized >= 1.0) {
        normalized /= 16.0;
        ++exp;
    }
    while (normalized < 0.0625) { // 1/16
        normalized *= 16.0;
        --exp;
    }

    int e = exp + 64;
    if (e < 0) return 0ULL;
    if (e > 127) e = 127;

    uint64_t mantissa = static_cast<uint64_t>(normalized * 72057594037927936.0 + 0.5); // 2^56
    if (mantissa >= (1ULL << 56)) {
        mantissa >>= 4;
        ++e;
    }

    uint64_t result = 0ULL;
    if (negative) result |= (1ULL << 63);
    result |= (static_cast<uint64_t>(e & 0x7F) << 56);
    result |= (mantissa & 0x00FFFFFFFFFFFFFFULL);
    return result;
}

double GdsiiReader::gdsii_real_to_double(uint64_t gds_real) noexcept {
    if (gds_real == 0ULL) return 0.0;

    bool negative = ((gds_real >> 63) & 1) != 0;
    int e = static_cast<int>((gds_real >> 56) & 0x7F) - 64;
    uint64_t mantissa = gds_real & 0x00FFFFFFFFFFFFFFULL;

    double frac = static_cast<double>(mantissa) / 72057594037927936.0; // 2^56
    double val = frac * std::pow(16.0, e);
    return negative ? -val : val;
}

void GdsiiWriter::write_record_header(GdsiiRecordType type, GdsiiDataType data_type, uint16_t length) {
    uint8_t header[4];
    header[0] = static_cast<uint8_t>((length >> 8) & 0xFF);
    header[1] = static_cast<uint8_t>(length & 0xFF);
    header[2] = static_cast<uint8_t>(type);
    header[3] = static_cast<uint8_t>(data_type);
    m_stream.write(reinterpret_cast<const char*>(header), 4);
}

void GdsiiWriter::write_int16(int16_t val) {
    uint8_t bytes[2];
    bytes[0] = static_cast<uint8_t>((val >> 8) & 0xFF);
    bytes[1] = static_cast<uint8_t>(val & 0xFF);
    m_stream.write(reinterpret_cast<const char*>(bytes), 2);
}

void GdsiiWriter::write_int32(int32_t val) {
    uint8_t bytes[4];
    bytes[0] = static_cast<uint8_t>((val >> 24) & 0xFF);
    bytes[1] = static_cast<uint8_t>((val >> 16) & 0xFF);
    bytes[2] = static_cast<uint8_t>((val >> 8) & 0xFF);
    bytes[3] = static_cast<uint8_t>(val & 0xFF);
    m_stream.write(reinterpret_cast<const char*>(bytes), 4);
}

void GdsiiWriter::write_real64(double val) {
    uint64_t raw = double_to_gdsii_real(val);
    uint8_t bytes[8];
    for (int i = 7; i >= 0; --i) {
        bytes[7 - i] = static_cast<uint8_t>((raw >> (i * 8)) & 0xFF);
    }
    m_stream.write(reinterpret_cast<const char*>(bytes), 8);
}

void GdsiiWriter::write_string(std::string_view str) {
    m_stream.write(str.data(), str.size());
    if (str.size() % 2 != 0) {
        uint8_t pad = 0x00;
        m_stream.write(reinterpret_cast<const char*>(&pad), 1);
    }
}

void GdsiiWriter::write_header(int16_t version) {
    write_record_header(GdsiiRecordType::HEADER, GdsiiDataType::INT_16, 6);
    write_int16(version);
}

void GdsiiWriter::write_bgnlib(std::string_view lib_name, double user_unit, double db_unit_meters) {
    // 12 2-byte integers for date
    write_record_header(GdsiiRecordType::BGNLIB, GdsiiDataType::INT_16, 28);
    for (int i = 0; i < 12; ++i) write_int16(2026);

    uint16_t name_len = static_cast<uint16_t>(lib_name.size() + (lib_name.size() % 2));
    write_record_header(GdsiiRecordType::LIBNAME, GdsiiDataType::ASCII, 4 + name_len);
    write_string(lib_name);

    write_record_header(GdsiiRecordType::UNITS, GdsiiDataType::REAL_64, 20);
    write_real64(user_unit);
    write_real64(db_unit_meters);
}

void GdsiiWriter::write_bgnstr(std::string_view cell_name) {
    write_record_header(GdsiiRecordType::BGNSTR, GdsiiDataType::INT_16, 28);
    for (int i = 0; i < 12; ++i) write_int16(2026);

    uint16_t name_len = static_cast<uint16_t>(cell_name.size() + (cell_name.size() % 2));
    write_record_header(GdsiiRecordType::STRNAME, GdsiiDataType::ASCII, 4 + name_len);
    write_string(cell_name);
}

void GdsiiWriter::write_endstr() {
    write_record_header(GdsiiRecordType::ENDSTR, GdsiiDataType::NONE, 4);
}

void GdsiiWriter::write_endlib() {
    write_record_header(GdsiiRecordType::ENDLIB, GdsiiDataType::NONE, 4);
}

void GdsiiWriter::write_boundary(uint16_t layer, uint16_t datatype, const std::vector<db::Point2D>& points) {
    if (points.size() < 4) return;

    write_record_header(GdsiiRecordType::BOUNDARY, GdsiiDataType::NONE, 4);

    write_record_header(GdsiiRecordType::LAYER, GdsiiDataType::INT_16, 6);
    write_int16(static_cast<int16_t>(layer));

    write_record_header(GdsiiRecordType::DATATYPE, GdsiiDataType::INT_16, 6);
    write_int16(static_cast<int16_t>(datatype));

    uint16_t xy_len = static_cast<uint16_t>(4 + points.size() * 8);
    write_record_header(GdsiiRecordType::XY, GdsiiDataType::INT_32, xy_len);
    for (const auto& pt : points) {
        write_int32(pt.x);
        write_int32(pt.y);
    }

    write_record_header(GdsiiRecordType::ENDEL, GdsiiDataType::NONE, 4);
}

void GdsiiWriter::write_box(uint16_t layer, uint16_t datatype, const db::Rect2D& box) {
    // 5 points forming closed loop
    std::vector<db::Point2D> pts = {
        {box.x_min, box.y_min},
        {box.x_max, box.y_min},
        {box.x_max, box.y_max},
        {box.x_min, box.y_max},
        {box.x_min, box.y_min}
    };
    write_boundary(layer, datatype, pts);
}

void GdsiiWriter::write_sref(std::string_view target_cell, db::Point2D origin, db::Orientation orient) {
    write_record_header(GdsiiRecordType::SREF, GdsiiDataType::NONE, 4);

    uint16_t name_len = static_cast<uint16_t>(target_cell.size() + (target_cell.size() % 2));
    write_record_header(GdsiiRecordType::SNAME, GdsiiDataType::ASCII, 4 + name_len);
    write_string(target_cell);

    // Orientation / Strans if rotated
    if (orient != db::Orientation::R0) {
        write_record_header(GdsiiRecordType::STRANS, GdsiiDataType::BIT_ARRAY, 6);
        bool mirrored = (orient == db::Orientation::MY || orient == db::Orientation::MY90 ||
                         orient == db::Orientation::MX || orient == db::Orientation::MX90);
        write_int16(mirrored ? static_cast<int16_t>(0x8000) : static_cast<int16_t>(0x0000));

        double angle = 0.0;
        switch (orient) {
            case db::Orientation::R90:
            case db::Orientation::MY90:
                angle = 90.0; break;
            case db::Orientation::R180:
            case db::Orientation::MX:
                angle = 180.0; break;
            case db::Orientation::R270:
            case db::Orientation::MX90:
                angle = 270.0; break;
            default: break;
        }
        if (angle > 0.0) {
            write_record_header(GdsiiRecordType::ANGLE, GdsiiDataType::REAL_64, 12);
            write_real64(angle);
        }
    }

    // Origin
    write_record_header(GdsiiRecordType::XY, GdsiiDataType::INT_32, 12);
    write_int32(origin.x);
    write_int32(origin.y);

    write_record_header(GdsiiRecordType::ENDEL, GdsiiDataType::NONE, 4);
}

bool GdsiiWriter::export_chip_db(const db::ChipDB& chip_db, const std::string& filepath) {
    GdsiiWriter writer(filepath);
    if (!writer.is_open()) return false;

    writer.write_header(600);
    writer.write_bgnlib("SOC_DESIGN_LIB", 0.001, 1e-9);

    // 1. Write structures for all MasterCells
    for (size_t i = 0; i < chip_db.master_cells.size(); ++i) {
        const auto& cell = chip_db.master_cells[static_cast<uint32_t>(i)];
        std::string_view cell_name = chip_db.strings.resolve(cell.name_id);
        if (cell_name.empty()) continue;

        writer.write_bgnstr(cell_name);
        // Cell boundary outline on layer 0 (boundary)
        writer.write_box(0, 0, db::Rect2D{0, 0, cell.width, cell.height});

        // Ports / Pins
        for (const auto& port : cell.ports) {
            uint16_t layer_num = 1;
            if (port.layer_id.is_valid() && port.layer_id.value() < chip_db.layers.size()) {
                layer_num = chip_db.layers[port.layer_id].gds_layer;
            }
            writer.write_box(layer_num, 0, port.box);
        }

        // Obstructions
        for (const auto& obs : cell.obstructions) {
            uint16_t layer_num = 1;
            if (cell.obstruction_layer.is_valid() && cell.obstruction_layer.value() < chip_db.layers.size()) {
                layer_num = chip_db.layers[cell.obstruction_layer].gds_layer;
            }
            writer.write_box(layer_num, 1, obs);
        }
        writer.write_endstr();
    }

    // 2. Write structures for all BlockAbstracts (P-Core, E-Core, GPU, etc.)
    for (size_t i = 0; i < chip_db.block_abstracts.size(); ++i) {
        const auto& ba = chip_db.block_abstracts[static_cast<uint32_t>(i)];
        std::string_view ba_name = chip_db.strings.resolve(ba.name_id);
        if (ba_name.empty()) continue;

        writer.write_bgnstr(ba_name);
        db::Rect2D local_box{0, 0, ba.boundary.width(), ba.boundary.height()};
        writer.write_box(100, 0, local_box); // Layer 100 for block boundary

        for (const auto& obs : ba.obstructions) {
            writer.write_box(101, 0, obs); // Layer 101 for block obstructions
        }
        writer.write_endstr();
    }

    // 3. Write Top Structure
    std::string_view top_name = chip_db.strings.resolve(chip_db.design_name);
    if (top_name.empty()) top_name = "TOP_CHIP";
    writer.write_bgnstr(top_name);

    // Die Boundary
    writer.write_box(0, 0, chip_db.die_area);

    // Instances of BlockAbstracts
    for (size_t i = 0; i < chip_db.block_abstracts.size(); ++i) {
        const auto& ba = chip_db.block_abstracts[static_cast<uint32_t>(i)];
        std::string_view ba_name = chip_db.strings.resolve(ba.name_id);
        writer.write_sref(ba_name, db::Point2D{ba.boundary.x_min, ba.boundary.y_min});
    }

    // Instances of Standard Cells
    for (size_t i = 0; i < chip_db.instances.size(); ++i) {
        const auto& inst = chip_db.instances[static_cast<uint32_t>(i)];
        if (!inst.master_id.is_valid()) continue;
        const auto& master = chip_db.master_cells[inst.master_id];
        std::string_view mname = chip_db.strings.resolve(master.name_id);
        writer.write_sref(mname, inst.origin, inst.orientation);
    }

    // Routed wire segments
    for (size_t i = 0; i < chip_db.nets.size(); ++i) {
        const auto& net = chip_db.nets[static_cast<uint32_t>(i)];
        for (const auto& seg : net.route_segments) {
            uint16_t lnum = 1;
            if (seg.layer.is_valid() && seg.layer.value() < chip_db.layers.size()) {
                lnum = chip_db.layers[seg.layer].gds_layer;
            }
            int32_t hw = seg.width / 2;
            db::Rect2D seg_box{
                std::min(seg.start.x, seg.end.x) - hw,
                std::min(seg.start.y, seg.end.y) - hw,
                std::max(seg.start.x, seg.end.x) + hw,
                std::max(seg.start.y, seg.end.y) + hw
            };
            writer.write_box(lnum, 0, seg_box);
        }
    }

    writer.write_endstr();
    writer.write_endlib();
    writer.close();
    return true;
}

// -------------------------------------------------------------
// GdsiiReader Implementation
// -------------------------------------------------------------
GdsiiReader::GdsiiReader(const std::string& filepath)
    : m_filepath(filepath) {}

bool GdsiiReader::parse() {
    std::ifstream stream(m_filepath, std::ios::binary | std::ios::in);
    if (!stream.is_open()) return false;

    m_structures.clear();
    GdsiiStructure* current_str = nullptr;
    GdsiiBoundary* current_boundary = nullptr;
    GdsiiSRef* current_sref = nullptr;

    while (stream.peek() != EOF) {
        uint8_t header[4];
        stream.read(reinterpret_cast<char*>(header), 4);
        if (stream.gcount() < 4) break;

        uint16_t length = (static_cast<uint16_t>(header[0]) << 8) | static_cast<uint16_t>(header[1]);
        if (length < 4) return false;

        GdsiiRecordType rec_type = static_cast<GdsiiRecordType>(header[2]);
        GdsiiDataType data_type = static_cast<GdsiiDataType>(header[3]);
        (void)data_type;

        size_t data_len = length - 4;
        std::vector<uint8_t> data(data_len);
        if (data_len > 0) {
            stream.read(reinterpret_cast<char*>(data.data()), data_len);
            if (static_cast<size_t>(stream.gcount()) < data_len) break;
        }

        switch (rec_type) {
            case GdsiiRecordType::LIBNAME: {
                m_libname = std::string(reinterpret_cast<const char*>(data.data()),
                                        data.size() - (!data.empty() && data.back() == 0 ? 1 : 0));
                break;
            }
            case GdsiiRecordType::UNITS: {
                if (data.size() >= 16) {
                    uint64_t u1 = 0, u2 = 0;
                    for (int i = 0; i < 8; ++i) u1 = (u1 << 8) | data[i];
                    for (int i = 0; i < 8; ++i) u2 = (u2 << 8) | data[8 + i];
                    m_user_unit = gdsii_real_to_double(u1);
                    m_db_unit_meters = gdsii_real_to_double(u2);
                }
                break;
            }
            case GdsiiRecordType::STRNAME: {
                std::string sname(reinterpret_cast<const char*>(data.data()),
                                  data.size() - (!data.empty() && data.back() == 0 ? 1 : 0));
                m_structures.push_back({sname, {}, {}});
                current_str = &m_structures.back();
                break;
            }
            case GdsiiRecordType::BOUNDARY: {
                if (current_str) {
                    current_str->boundaries.push_back({});
                    current_boundary = &current_str->boundaries.back();
                }
                break;
            }
            case GdsiiRecordType::SREF: {
                if (current_str) {
                    current_str->srefs.push_back({});
                    current_sref = &current_str->srefs.back();
                }
                break;
            }
            case GdsiiRecordType::SNAME: {
                if (current_sref) {
                    current_sref->target_structure = std::string(
                        reinterpret_cast<const char*>(data.data()),
                        data.size() - (!data.empty() && data.back() == 0 ? 1 : 0));
                }
                break;
            }
            case GdsiiRecordType::LAYER: {
                if (current_boundary && data.size() >= 2) {
                    current_boundary->layer = (static_cast<uint16_t>(data[0]) << 8) | data[1];
                }
                break;
            }
            case GdsiiRecordType::DATATYPE: {
                if (current_boundary && data.size() >= 2) {
                    current_boundary->datatype = (static_cast<uint16_t>(data[0]) << 8) | data[1];
                }
                break;
            }
            case GdsiiRecordType::XY: {
                if (current_boundary && data.size() >= 8) {
                    size_t num_pts = data.size() / 8;
                    for (size_t p = 0; p < num_pts; ++p) {
                        int32_t x = (static_cast<int32_t>(data[p * 8 + 0]) << 24) |
                                    (static_cast<int32_t>(data[p * 8 + 1]) << 16) |
                                    (static_cast<int32_t>(data[p * 8 + 2]) << 8)  |
                                    static_cast<int32_t>(data[p * 8 + 3]);
                        int32_t y = (static_cast<int32_t>(data[p * 8 + 4]) << 24) |
                                    (static_cast<int32_t>(data[p * 8 + 5]) << 16) |
                                    (static_cast<int32_t>(data[p * 8 + 6]) << 8)  |
                                    static_cast<int32_t>(data[p * 8 + 7]);
                        current_boundary->points.push_back({x, y});
                    }
                } else if (current_sref && data.size() >= 8) {
                    int32_t x = (static_cast<int32_t>(data[0]) << 24) |
                                (static_cast<int32_t>(data[1]) << 16) |
                                (static_cast<int32_t>(data[2]) << 8)  |
                                static_cast<int32_t>(data[3]);
                    int32_t y = (static_cast<int32_t>(data[4]) << 24) |
                                (static_cast<int32_t>(data[5]) << 16) |
                                (static_cast<int32_t>(data[6]) << 8)  |
                                static_cast<int32_t>(data[7]);
                    current_sref->origin = {x, y};
                }
                break;
            }
            case GdsiiRecordType::ENDEL: {
                current_boundary = nullptr;
                current_sref = nullptr;
                break;
            }
            case GdsiiRecordType::ENDSTR: {
                current_str = nullptr;
                break;
            }
            case GdsiiRecordType::ENDLIB: {
                return true;
            }
            default:
                break;
        }
    }

    return true;
}

} // namespace soc::parsers
