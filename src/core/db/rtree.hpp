#pragma once

#include "id_types.hpp"
#include <vector>
#include <cstdint>
#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

namespace soc::db {

// 2D Spatial R-Tree supporting O(log N) bounding-box queries and sub-pixel LOD culling
template<typename T, size_t MaxEntries = 16, size_t MinEntries = 4>
class RTree {
    static_assert(MaxEntries > MinEntries * 2, "MaxEntries must be at least 2 * MinEntries + 1");

public:
    struct Entry {
        Rect2D bbox;
        T value{};
        uint32_t child_node{0xFFFFFFFF}; // 0xFFFFFFFF if leaf entry

        [[nodiscard]] bool is_leaf() const noexcept { return child_node == 0xFFFFFFFF; }
    };

    struct Node {
        Rect2D bbox;
        uint32_t entry_count{0};
        bool is_leaf{true};
        Entry entries[MaxEntries + 1];
    };

    RTree() {
        create_node(true); // Root node at index 0
    }

    void clear() {
        m_nodes.clear();
        m_size = 0;
        create_node(true);
    }

    [[nodiscard]] size_t size() const noexcept { return m_size; }
    [[nodiscard]] bool empty() const noexcept { return m_size == 0; }

    [[nodiscard]] Rect2D root_bbox() const noexcept {
        if (m_nodes.empty() || m_nodes[0].entry_count == 0) {
            return Rect2D{0, 0, 0, 0};
        }
        return m_nodes[0].bbox;
    }

    void insert(const Rect2D& bbox, const T& value) {
        Entry e;
        e.bbox = bbox;
        e.value = value;
        e.child_node = 0xFFFFFFFF;

        uint32_t split_node = 0xFFFFFFFF;
        Entry split_entry;

        insert_recursive(0, e, split_node, split_entry);

        if (split_node != 0xFFFFFFFF) {
            // Root was split. Create a new child to hold current root's contents
            bool root_is_leaf = m_nodes[0].is_leaf;
            uint32_t old_root_idx = create_node(root_is_leaf);

            // Copy old root content to new node
            m_nodes[old_root_idx] = m_nodes[0];

            // Reinitialize root as internal node with 2 children
            m_nodes[0].is_leaf = false;
            m_nodes[0].entry_count = 2;

            m_nodes[0].entries[0].bbox = m_nodes[old_root_idx].bbox;
            m_nodes[0].entries[0].child_node = old_root_idx;

            m_nodes[0].entries[1] = split_entry;
            m_nodes[0].bbox = m_nodes[0].entries[0].bbox;
            m_nodes[0].bbox.merge(m_nodes[0].entries[1].bbox);
        }

        ++m_size;
    }

    void query_intersect(const Rect2D& search_box, std::vector<T>& results) const {
        if (m_nodes.empty() || m_size == 0) return;
        query_node_intersect(0, search_box, results);
    }

    void query_lod(const Rect2D& search_box,
                   int32_t min_pixel_size_dbu,
                   std::vector<T>& visible_items,
                   std::vector<Rect2D>& culled_clusters) const {
        if (m_nodes.empty() || m_size == 0) return;
        query_node_lod(0, search_box, min_pixel_size_dbu, visible_items, culled_clusters);
    }

private:
    std::vector<Node> m_nodes;
    size_t m_size{0};

    uint32_t create_node(bool is_leaf) {
        uint32_t idx = static_cast<uint32_t>(m_nodes.size());
        Node n;
        n.is_leaf = is_leaf;
        n.entry_count = 0;
        n.bbox = Rect2D{0, 0, 0, 0};
        m_nodes.push_back(n);
        return idx;
    }

    void recalculate_bbox(uint32_t node_idx) noexcept {
        Node& node = m_nodes[node_idx];
        if (node.entry_count == 0) {
            node.bbox = Rect2D{0, 0, 0, 0};
            return;
        }
        node.bbox = node.entries[0].bbox;
        for (uint32_t i = 1; i < node.entry_count; ++i) {
            node.bbox.merge(node.entries[i].bbox);
        }
    }

    static int64_t enlargement_cost(const Rect2D& current, const Rect2D& additional) noexcept {
        Rect2D merged = current;
        merged.merge(additional);
        return merged.area() - current.area();
    }

    uint32_t choose_best_child(uint32_t node_idx, const Rect2D& entry_bbox) const noexcept {
        const Node& node = m_nodes[node_idx];
        int64_t best_enlargement = std::numeric_limits<int64_t>::max();
        int64_t best_area = std::numeric_limits<int64_t>::max();
        uint32_t best_idx = 0;

        for (uint32_t i = 0; i < node.entry_count; ++i) {
            int64_t enl = enlargement_cost(node.entries[i].bbox, entry_bbox);
            int64_t cur_area = node.entries[i].bbox.area();
            if (enl < best_enlargement || (enl == best_enlargement && cur_area < best_area)) {
                best_enlargement = enl;
                best_area = cur_area;
                best_idx = i;
            }
        }
        return best_idx;
    }

    void insert_recursive(uint32_t node_idx, const Entry& entry, uint32_t& split_node, Entry& split_entry) {
        if (m_nodes[node_idx].is_leaf) {
            if (m_nodes[node_idx].entry_count == 0) {
                m_nodes[node_idx].bbox = entry.bbox;
            } else {
                m_nodes[node_idx].bbox.merge(entry.bbox);
            }
            m_nodes[node_idx].entries[m_nodes[node_idx].entry_count++] = entry;

            if (m_nodes[node_idx].entry_count > MaxEntries) {
                split_node = split_node_quadratic(node_idx, split_entry);
            } else {
                split_node = 0xFFFFFFFF;
            }
            return;
        }

        // Internal node: choose subtree
        uint32_t child_entry_idx = choose_best_child(node_idx, entry.bbox);
        uint32_t child_node_idx = m_nodes[node_idx].entries[child_entry_idx].child_node;

        uint32_t child_split_node = 0xFFFFFFFF;
        Entry child_split_entry;

        insert_recursive(child_node_idx, entry, child_split_node, child_split_entry);

        // Update bbox for child entry (re-index m_nodes)
        m_nodes[node_idx].entries[child_entry_idx].bbox = m_nodes[child_node_idx].bbox;
        recalculate_bbox(node_idx);

        if (child_split_node != 0xFFFFFFFF) {
            m_nodes[node_idx].entries[m_nodes[node_idx].entry_count++] = child_split_entry;
            m_nodes[node_idx].bbox.merge(child_split_entry.bbox);

            if (m_nodes[node_idx].entry_count > MaxEntries) {
                split_node = split_node_quadratic(node_idx, split_entry);
            } else {
                split_node = 0xFFFFFFFF;
            }
        } else {
            split_node = 0xFFFFFFFF;
        }
    }

    uint32_t split_node_quadratic(uint32_t node_idx, Entry& out_split_entry) {
        const uint32_t total_entries = m_nodes[node_idx].entry_count;
        const bool is_leaf = m_nodes[node_idx].is_leaf;

        // Temporary copy of all entries
        Entry all_entries[MaxEntries + 1];
        for (uint32_t i = 0; i < total_entries; ++i) {
            all_entries[i] = m_nodes[node_idx].entries[i];
        }

        // 1. Pick Seeds
        int64_t max_waste = std::numeric_limits<int64_t>::min();
        uint32_t seed1 = 0;
        uint32_t seed2 = 1;

        for (uint32_t i = 0; i < total_entries; ++i) {
            for (uint32_t j = i + 1; j < total_entries; ++j) {
                Rect2D combo = all_entries[i].bbox;
                combo.merge(all_entries[j].bbox);
                int64_t waste = combo.area() - all_entries[i].bbox.area() - all_entries[j].bbox.area();
                if (waste > max_waste) {
                    max_waste = waste;
                    seed1 = i;
                    seed2 = j;
                }
            }
        }

        // Allocate node2
        uint32_t node2_idx = create_node(is_leaf);

        // Assign seeds
        m_nodes[node_idx].entries[0] = all_entries[seed1];
        m_nodes[node_idx].entry_count = 1;
        m_nodes[node_idx].bbox = all_entries[seed1].bbox;

        m_nodes[node2_idx].entries[0] = all_entries[seed2];
        m_nodes[node2_idx].entry_count = 1;
        m_nodes[node2_idx].bbox = all_entries[seed2].bbox;

        bool assigned[MaxEntries + 1] = {false};
        assigned[seed1] = true;
        assigned[seed2] = true;
        uint32_t remaining = total_entries - 2;

        // 2. Distribute remaining entries
        while (remaining > 0) {
            if (m_nodes[node_idx].entry_count + remaining == MinEntries) {
                for (uint32_t i = 0; i < total_entries; ++i) {
                    if (!assigned[i]) {
                        m_nodes[node_idx].entries[m_nodes[node_idx].entry_count++] = all_entries[i];
                        m_nodes[node_idx].bbox.merge(all_entries[i].bbox);
                        assigned[i] = true;
                        --remaining;
                    }
                }
                break;
            }
            if (m_nodes[node2_idx].entry_count + remaining == MinEntries) {
                for (uint32_t i = 0; i < total_entries; ++i) {
                    if (!assigned[i]) {
                        m_nodes[node2_idx].entries[m_nodes[node2_idx].entry_count++] = all_entries[i];
                        m_nodes[node2_idx].bbox.merge(all_entries[i].bbox);
                        assigned[i] = true;
                        --remaining;
                    }
                }
                break;
            }

            int64_t max_diff = -1;
            uint32_t next_entry = 0xFFFFFFFF;
            bool prefer_group1 = true;

            for (uint32_t i = 0; i < total_entries; ++i) {
                if (assigned[i]) continue;
                int64_t d1 = enlargement_cost(m_nodes[node_idx].bbox, all_entries[i].bbox);
                int64_t d2 = enlargement_cost(m_nodes[node2_idx].bbox, all_entries[i].bbox);
                int64_t diff = std::abs(d1 - d2);
                if (diff > max_diff) {
                    max_diff = diff;
                    next_entry = i;
                    prefer_group1 = (d1 < d2) || (d1 == d2 && m_nodes[node_idx].bbox.area() < m_nodes[node2_idx].bbox.area());
                }
            }

            if (next_entry != 0xFFFFFFFF) {
                if (prefer_group1) {
                    m_nodes[node_idx].entries[m_nodes[node_idx].entry_count++] = all_entries[next_entry];
                    m_nodes[node_idx].bbox.merge(all_entries[next_entry].bbox);
                } else {
                    m_nodes[node2_idx].entries[m_nodes[node2_idx].entry_count++] = all_entries[next_entry];
                    m_nodes[node2_idx].bbox.merge(all_entries[next_entry].bbox);
                }
                assigned[next_entry] = true;
                --remaining;
            }
        }

        recalculate_bbox(node_idx);
        recalculate_bbox(node2_idx);

        out_split_entry.bbox = m_nodes[node2_idx].bbox;
        out_split_entry.child_node = node2_idx;
        return node2_idx;
    }

    void query_node_intersect(uint32_t node_idx, const Rect2D& search_box, std::vector<T>& results) const {
        const Node& node = m_nodes[node_idx];
        if (!node.bbox.intersects(search_box)) return;

        for (uint32_t i = 0; i < node.entry_count; ++i) {
            const Entry& e = node.entries[i];
            if (e.bbox.intersects(search_box)) {
                if (node.is_leaf) {
                    results.push_back(e.value);
                } else {
                    query_node_intersect(e.child_node, search_box, results);
                }
            }
        }
    }

    void query_node_lod(uint32_t node_idx,
                        const Rect2D& search_box,
                        int32_t min_pixel_size_dbu,
                        std::vector<T>& visible_items,
                        std::vector<Rect2D>& culled_clusters) const {
        const Node& node = m_nodes[node_idx];
        if (!node.bbox.intersects(search_box)) return;

        if (!node.is_leaf && node.bbox.width() < min_pixel_size_dbu && node.bbox.height() < min_pixel_size_dbu) {
            culled_clusters.push_back(node.bbox);
            return;
        }

        for (uint32_t i = 0; i < node.entry_count; ++i) {
            const Entry& e = node.entries[i];
            if (e.bbox.intersects(search_box)) {
                if (node.is_leaf) {
                    if (e.bbox.width() < min_pixel_size_dbu && e.bbox.height() < min_pixel_size_dbu) {
                        culled_clusters.push_back(e.bbox);
                    } else {
                        visible_items.push_back(e.value);
                    }
                } else {
                    query_node_lod(e.child_node, search_box, min_pixel_size_dbu, visible_items, culled_clusters);
                }
            }
        }
    }
};

} // namespace soc::db
