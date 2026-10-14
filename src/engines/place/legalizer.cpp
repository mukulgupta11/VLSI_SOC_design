#include "legalizer.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <iostream>

namespace vlsi::place {

DetailedLegalizer::DetailedLegalizer(soc::db::ChipDB& db, const LegalizerConfig& config)
    : m_db(db), m_config(config) {}

LegalizerStats DetailedLegalizer::legalize() {
    auto start_time = std::chrono::high_resolution_clock::now();
    LegalizerStats stats;

    int32_t fin = m_config.fin_pitch;
    int32_t row_h = m_config.row_height;

    // 1. Establish core rows
    int32_t core_y_min = m_config.core_y_min;
    int32_t core_y_max = m_config.core_y_max;
    int32_t core_x_min = m_config.core_x_min;
    int32_t core_x_max = m_config.core_x_max;

    if (m_db.die_area.width() > 0 && m_db.die_area.height() > 0) {
        core_x_min = m_db.die_area.x_min;
        core_y_min = m_db.die_area.y_min;
        core_x_max = m_db.die_area.x_max;
        core_y_max = m_db.die_area.y_max;
    }

    int32_t num_rows = (core_y_max - core_y_min) / row_h;
    if (num_rows <= 0) {
        return stats;
    }

    std::vector<Row> rows(num_rows);
    for (int32_t r = 0; r < num_rows; ++r) {
        rows[r].row_index = r;
        rows[r].y_coord = core_y_min + r * row_h;

        // Initial single sub-row covering full width
        SubRowState init_sr;
        init_sr.x_start = snap_to_fin(core_x_min);
        init_sr.x_end = snap_to_fin(core_x_max);
        rows[r].sub_rows.push_back(init_sr);
    }

    // 2. Punch holes in sub-rows for Macro Abstracts
    for (size_t b = 0; b < m_db.block_abstracts.size(); ++b) {
        soc::db::BlockAbstractId blk_id{static_cast<uint32_t>(b)};
        const auto& block = m_db.block_abstracts[blk_id];
        const auto& box = block.boundary;

        int32_t r_start = std::max(0, (box.y_min - core_y_min) / row_h);
        int32_t r_end = std::min(num_rows - 1, (box.y_max - core_y_min) / row_h);

        for (int32_t r = r_start; r <= r_end; ++r) {
            std::vector<SubRowState> new_sub_rows;
            for (const auto& sr : rows[r].sub_rows) {
                // If obstruction completely covers sub-row
                if (box.x_min <= sr.x_start && box.x_max >= sr.x_end) {
                    continue; // Sub-row eliminated
                }
                // If obstruction falls strictly inside sub-row, split into two
                if (box.x_min > sr.x_start && box.x_max < sr.x_end) {
                    int32_t snap_floor = (box.x_min / fin) * fin;
                    int32_t snap_ceil = ((box.x_max + fin - 1) / fin) * fin;

                    SubRowState left_sr;
                    left_sr.x_start = sr.x_start;
                    left_sr.x_end = snap_floor;
                    if (left_sr.x_end - left_sr.x_start >= fin) {
                        new_sub_rows.push_back(left_sr);
                    }

                    SubRowState right_sr;
                    right_sr.x_start = snap_ceil;
                    right_sr.x_end = sr.x_end;
                    if (right_sr.x_end - right_sr.x_start >= fin) {
                        new_sub_rows.push_back(right_sr);
                    }
                }
                // Overlaps left side of sub-row
                else if (box.x_min <= sr.x_start && box.x_max > sr.x_start && box.x_max < sr.x_end) {
                    SubRowState trimmed;
                    trimmed.x_start = ((box.x_max + fin - 1) / fin) * fin;
                    trimmed.x_end = sr.x_end;
                    if (trimmed.x_end - trimmed.x_start >= fin) {
                        new_sub_rows.push_back(trimmed);
                    }
                }
                // Overlaps right side of sub-row
                else if (box.x_min > sr.x_start && box.x_min < sr.x_end && box.x_max >= sr.x_end) {
                    SubRowState trimmed;
                    trimmed.x_start = sr.x_start;
                    trimmed.x_end = (box.x_min / fin) * fin;
                    if (trimmed.x_end - trimmed.x_start >= fin) {
                        new_sub_rows.push_back(trimmed);
                    }
                } else {
                    new_sub_rows.push_back(sr);
                }
            }
            rows[r].sub_rows = std::move(new_sub_rows);
        }
    }

    // 3. Punch holes in sub-rows for BSPDN Nano-TSV Keep-Out Zones
    int32_t ko_radius = m_config.ntsv_keepout_radius;
    for (const auto& tsv : m_db.nano_tsvs) {
        soc::db::Rect2D ko_box{tsv.pos.x - ko_radius, tsv.pos.y - ko_radius,
                               tsv.pos.x + ko_radius, tsv.pos.y + ko_radius};

        int32_t r_start = std::max(0, (ko_box.y_min - core_y_min) / row_h);
        int32_t r_end = std::min(num_rows - 1, (ko_box.y_max - core_y_min) / row_h);

        for (int32_t r = r_start; r <= r_end; ++r) {
            std::vector<SubRowState> new_sub_rows;
            for (const auto& sr : rows[r].sub_rows) {
                if (ko_box.x_min <= sr.x_start && ko_box.x_max >= sr.x_end) {
                    continue;
                }
                if (ko_box.x_min > sr.x_start && ko_box.x_max < sr.x_end) {
                    SubRowState left_sr;
                    left_sr.x_start = sr.x_start;
                    left_sr.x_end = (ko_box.x_min / fin) * fin;
                    if (left_sr.x_end - left_sr.x_start >= fin) new_sub_rows.push_back(left_sr);

                    SubRowState right_sr;
                    right_sr.x_start = ((ko_box.x_max + fin - 1) / fin) * fin;
                    right_sr.x_end = sr.x_end;
                    if (right_sr.x_end - right_sr.x_start >= fin) new_sub_rows.push_back(right_sr);
                } else if (ko_box.x_min <= sr.x_start && ko_box.x_max > sr.x_start && ko_box.x_max < sr.x_end) {
                    SubRowState trimmed;
                    trimmed.x_start = ((ko_box.x_max + fin - 1) / fin) * fin;
                    trimmed.x_end = sr.x_end;
                    if (trimmed.x_end - trimmed.x_start >= fin) new_sub_rows.push_back(trimmed);
                } else if (ko_box.x_min > sr.x_start && ko_box.x_min < sr.x_end && ko_box.x_max >= sr.x_end) {
                    SubRowState trimmed;
                    trimmed.x_start = sr.x_start;
                    trimmed.x_end = (ko_box.x_min / fin) * fin;
                    if (trimmed.x_end - trimmed.x_start >= fin) new_sub_rows.push_back(trimmed);
                } else {
                    new_sub_rows.push_back(sr);
                }
            }
            rows[r].sub_rows = std::move(new_sub_rows);
        }
    }

    // 4. Gather and sort movable cells by initial global placement X
    struct CellData {
        soc::db::InstanceId id{soc::db::InstanceId::invalid()};
        int32_t init_x{0};
        int32_t init_y{0};
        int32_t width{0};
        int32_t height{0};
    };

    std::vector<CellData> cells;
    cells.reserve(m_db.instances.size());

    for (size_t i = 0; i < m_db.instances.size(); ++i) {
        soc::db::InstanceId inst_id{static_cast<uint32_t>(i)};
        const auto& inst = m_db.instances[inst_id];
        if (inst.is_fixed || !inst.master_id.is_valid()) continue;

        const auto& master = m_db.master_cells[inst.master_id];
        if (master.cell_type == soc::db::CellType::MACRO ||
            master.cell_type == soc::db::CellType::BLOCK_ABSTRACT) continue;

        int32_t w = (master.width > 0) ? master.width : 240;
        int32_t h = (master.height > 0) ? master.height : 240;

        // Fin pitch quantization of width
        int32_t q_w = std::max(fin, ((w + fin - 1) / fin) * fin);

        cells.push_back(CellData{inst_id, inst.origin.x, inst.origin.y, q_w, h});
    }

    std::sort(cells.begin(), cells.end(), [](const CellData& a, const CellData& b) {
        if (a.init_x != b.init_x) return a.init_x < b.init_x;
        return a.init_y < b.init_y;
    });

    // 5. Abacus Dynamic Programming Legalization
    // Structure to track placed cell results
    struct PlacedCell {
        soc::db::InstanceId id;
        int32_t final_x{0};
        int32_t final_y{0};
    };
    std::vector<PlacedCell> final_placements;
    final_placements.reserve(cells.size());

    for (size_t c_idx = 0; c_idx < cells.size(); ++c_idx) {
        const auto& cell = cells[c_idx];
        int32_t ideal_r = std::clamp((cell.init_y - core_y_min) / row_h, 0, num_rows - 1);

        int32_t best_r = -1;
        int32_t best_sr = -1;
        double best_cost = 1e30;
        int32_t best_x = 0;

        int32_t k = m_config.search_rows;
        int32_t r_start = std::max(0, ideal_r - k);
        int32_t r_end = std::min(num_rows - 1, ideal_r + k);

        // Try candidate rows
        for (int32_t r = r_start; r <= r_end; ++r) {
            int32_t y_r = rows[r].y_coord;

            for (size_t sr_idx = 0; sr_idx < rows[r].sub_rows.size(); ++sr_idx) {
                auto& sr = rows[r].sub_rows[sr_idx];
                int32_t sr_cap = sr.x_end - sr.x_start;

                // Calculate current occupied width in this sub-row
                int32_t occupied_w = 0;
                for (const auto& cl : sr.clusters) {
                    occupied_w += cl.width;
                }
                if (occupied_w + cell.width > sr_cap) {
                    continue; // Not enough remaining capacity
                }

                // Simulate cluster insertion
                double trial_q = cell.width * cell.init_x;
                double trial_w = cell.width;
                int32_t trial_width = cell.width;
                int32_t trial_x = snap_to_fin(static_cast<int32_t>(trial_q / trial_w));
                trial_x = std::clamp(trial_x, sr.x_start, sr.x_end - trial_width);

                // Cluster collapse simulation against existing clusters
                int32_t sim_x = trial_x;
                if (!sr.clusters.empty()) {
                    const auto& last_cl = sr.clusters.back();
                    if (sim_x < last_cl.x_pos + last_cl.width) {
                        sim_x = last_cl.x_pos + last_cl.width;
                        sim_x = snap_to_fin(sim_x);
                    }
                }

                if (sim_x + trial_width > sr.x_end) {
                    sim_x = sr.x_end - trial_width;
                    sim_x = snap_to_fin(sim_x);
                }

                double dx = sim_x - cell.init_x;
                double dy = y_r - cell.init_y;
                double cost = (dx * dx) + m_config.y_displacement_weight * (dy * dy);

                if (cost < best_cost) {
                    best_cost = cost;
                    best_r = r;
                    best_sr = static_cast<int32_t>(sr_idx);
                    best_x = sim_x;
                }
            }
        }

        // If no candidate row found in search window, widen search to all rows
        if (best_r == -1) {
            for (int32_t r = 0; r < num_rows; ++r) {
                int32_t y_r = rows[r].y_coord;
                for (size_t sr_idx = 0; sr_idx < rows[r].sub_rows.size(); ++sr_idx) {
                    auto& sr = rows[r].sub_rows[sr_idx];
                    int32_t occupied_w = 0;
                    for (const auto& cl : sr.clusters) occupied_w += cl.width;
                    if (occupied_w + cell.width <= sr.x_end - sr.x_start) {
                        int32_t trial_x = sr.x_start + occupied_w;
                        trial_x = snap_to_fin(trial_x);
                        double dx = trial_x - cell.init_x;
                        double dy = y_r - cell.init_y;
                        double cost = (dx * dx) + m_config.y_displacement_weight * (dy * dy);
                        if (cost < best_cost) {
                            best_cost = cost;
                            best_r = r;
                            best_sr = static_cast<int32_t>(sr_idx);
                            best_x = trial_x;
                        }
                    }
                }
            }
        }

        // Commit cell to best row and sub-row
        if (best_r != -1 && best_sr != -1) {
            auto& sr = rows[best_r].sub_rows[best_sr];

            // Create cluster for this cell
            Cluster new_cl;
            new_cl.cell_indices.push_back(static_cast<uint32_t>(c_idx));
            new_cl.total_weight = cell.width;
            new_cl.total_q = cell.width * cell.init_x;
            new_cl.width = cell.width;
            new_cl.x_pos = best_x;

            // Merge if overlapping with previous cluster
            sr.clusters.push_back(new_cl);
            size_t c_pos = sr.clusters.size() - 1;
            while (c_pos > 0) {
                auto& prev = sr.clusters[c_pos - 1];
                auto& curr = sr.clusters[c_pos];
                if (curr.x_pos < prev.x_pos + prev.width) {
                    // Merge prev and curr
                    curr.total_q = prev.total_q + curr.total_q - curr.total_weight * prev.width;
                    curr.total_weight = prev.total_weight + curr.total_weight;
                    curr.width = prev.width + curr.width;
                    curr.x_pos = snap_to_fin(static_cast<int32_t>(curr.total_q / curr.total_weight));
                    curr.x_pos = std::clamp(curr.x_pos, sr.x_start, sr.x_end - curr.width);

                    // Prepend cell indices from prev
                    curr.cell_indices.insert(curr.cell_indices.begin(),
                                             prev.cell_indices.begin(), prev.cell_indices.end());
                    sr.clusters.erase(sr.clusters.begin() + (c_pos - 1));
                    --c_pos;
                } else {
                    break;
                }
            }
        }
    }

    // 6. Unpack clusters and apply legal coordinates
    double total_disp = 0.0;
    double max_disp = 0.0;

    for (int32_t r = 0; r < num_rows; ++r) {
        int32_t y_r = rows[r].y_coord;
        for (const auto& sr : rows[r].sub_rows) {
            for (const auto& cl : sr.clusters) {
                int32_t cur_cell_x = cl.x_pos;
                for (auto c_idx : cl.cell_indices) {
                    const auto& cell = cells[c_idx];
                    cur_cell_x = snap_to_fin(cur_cell_x);

                    m_db.move_instance(cell.id, soc::db::Point2D{cur_cell_x, y_r});

                    double disp = std::hypot(cur_cell_x - cell.init_x, y_r - cell.init_y);
                    total_disp += disp;
                    max_disp = std::max(max_disp, disp);

                    cur_cell_x += cell.width;
                }
            }
        }
    }

    stats.cells_legalized = static_cast<uint32_t>(cells.size());
    stats.total_displacement_nm = total_disp;
    stats.max_displacement_nm = max_disp;
    stats.avg_displacement_nm = (cells.empty()) ? 0.0 : (total_disp / cells.size());

    m_db.rebuild_instance_spatial_index();
    m_db.observers.notify_dirty_region(m_db.die_area, soc::db::LayerId{0});

    // Run verification pass
    LegalizerStats verify_stats = verify_legal_placement();
    stats.overlaps_detected = verify_stats.overlaps_detected;
    stats.fin_grid_violations = verify_stats.fin_grid_violations;
    stats.row_alignment_violations = verify_stats.row_alignment_violations;
    stats.ntsv_violations = verify_stats.ntsv_violations;
    stats.macro_violations = verify_stats.macro_violations;

    auto end_time = std::chrono::high_resolution_clock::now();
    stats.runtime_seconds = std::chrono::duration<double>(end_time - start_time).count();

    return stats;
}

LegalizerStats DetailedLegalizer::verify_legal_placement() const {
    LegalizerStats stats;
    int32_t fin = m_config.fin_pitch;
    int32_t row_h = m_config.row_height;
    int32_t core_y_min = m_config.core_y_min;

    if (m_db.die_area.width() > 0 && m_db.die_area.height() > 0) {
        core_y_min = m_db.die_area.y_min;
    }

    // Partition cells by row for overlap checking
    std::unordered_map<int32_t, std::vector<soc::db::Rect2D>> row_boxes;

    for (size_t i = 0; i < m_db.instances.size(); ++i) {
        soc::db::InstanceId inst_id{static_cast<uint32_t>(i)};
        const auto& inst = m_db.instances[inst_id];
        if (inst.is_fixed || !inst.master_id.is_valid()) continue;

        const auto& master = m_db.master_cells[inst.master_id];
        if (master.cell_type == soc::db::CellType::MACRO ||
            master.cell_type == soc::db::CellType::BLOCK_ABSTRACT) continue;

        stats.cells_legalized++;
        soc::db::Rect2D box = m_db.get_instance_bbox(inst_id);

        // 1. Fin grid check: origin X must be integer multiple of fin_pitch
        if (box.x_min % fin != 0) {
            stats.fin_grid_violations++;
        }

        // 2. Row height check: origin Y must be integer multiple of row_height
        if ((box.y_min - core_y_min) % row_h != 0) {
            stats.row_alignment_violations++;
        }

        auto has_area_overlap = [](const soc::db::Rect2D& a, const soc::db::Rect2D& b) noexcept {
            return (a.x_max > b.x_min && a.x_min < b.x_max &&
                    a.y_max > b.y_min && a.y_min < b.y_max);
        };

        // 3. Macro abstract overlap check
        for (size_t b = 0; b < m_db.block_abstracts.size(); ++b) {
            soc::db::BlockAbstractId blk_id{static_cast<uint32_t>(b)};
            const auto& block = m_db.block_abstracts[blk_id];
            if (has_area_overlap(box, block.boundary)) {
                stats.macro_violations++;
            }
        }

        // 4. BSPDN Nano-TSV keepout check
        int32_t ko_radius = m_config.ntsv_keepout_radius;
        for (const auto& tsv : m_db.nano_tsvs) {
            soc::db::Rect2D ko_box{tsv.pos.x - ko_radius, tsv.pos.y - ko_radius,
                                   tsv.pos.x + ko_radius, tsv.pos.y + ko_radius};
            if (has_area_overlap(box, ko_box)) {
                stats.ntsv_violations++;
            }
        }

        row_boxes[box.y_min].push_back(box);
    }

    // 5. Intra-row overlap checking
    for (auto& [y, boxes] : row_boxes) {
        std::sort(boxes.begin(), boxes.end(), [](const soc::db::Rect2D& a, const soc::db::Rect2D& b) {
            return a.x_min < b.x_min;
        });

        for (size_t j = 0; j + 1 < boxes.size(); ++j) {
            if (boxes[j].x_max > boxes[j + 1].x_min) {
                stats.overlaps_detected++;
            }
        }
    }

    return stats;
}

} // namespace vlsi::place
