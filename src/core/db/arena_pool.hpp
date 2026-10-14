#pragma once

#include "id_types.hpp"
#include <vector>
#include <memory>
#include <cassert>
#include <stdexcept>
#include <utility>

namespace soc::db {

// High-performance chunked arena pool indexed by 32-bit ID
// Avoids pointer invalidation on reallocation and maintains high cache locality
template<typename T, size_t ChunkSize = 65536>
class ArenaPool {
public:
    static_assert((ChunkSize & (ChunkSize - 1)) == 0, "ChunkSize must be a power of 2");
    static constexpr size_t CHUNK_MASK = ChunkSize - 1;
    static constexpr size_t CHUNK_SHIFT = []() constexpr {
        size_t shift = 0;
        size_t val = ChunkSize;
        while (val > 1) { val >>= 1; ++shift; }
        return shift;
    }();

    ArenaPool() = default;
    ~ArenaPool() = default;

    ArenaPool(const ArenaPool&) = delete;
    ArenaPool& operator=(const ArenaPool&) = delete;

    ArenaPool(ArenaPool&&) noexcept = default;
    ArenaPool& operator=(ArenaPool&&) noexcept = default;

    template<typename... Args>
    uint32_t emplace_back(Args&&... args) {
        const uint32_t id = m_size++;
        const size_t chunk_idx = id >> CHUNK_SHIFT;
        const size_t offset = id & CHUNK_MASK;

        if (chunk_idx >= m_chunks.size()) {
            m_chunks.emplace_back(std::make_unique<T[]>(ChunkSize));
        }

        m_chunks[chunk_idx][offset] = T(std::forward<Args>(args)...);
        return id;
    }

    template<typename Tag, typename... Args>
    ID<Tag> emplace_back_typed(Args&&... args) {
        return ID<Tag>{emplace_back(std::forward<Args>(args)...)};
    }

    [[nodiscard]] T& operator[](uint32_t id) noexcept {
        assert(id < m_size);
        const size_t chunk_idx = id >> CHUNK_SHIFT;
        const size_t offset = id & CHUNK_MASK;
        return m_chunks[chunk_idx][offset];
    }

    [[nodiscard]] const T& operator[](uint32_t id) const noexcept {
        assert(id < m_size);
        const size_t chunk_idx = id >> CHUNK_SHIFT;
        const size_t offset = id & CHUNK_MASK;
        return m_chunks[chunk_idx][offset];
    }

    template<typename Tag>
    [[nodiscard]] T& operator[](ID<Tag> id) noexcept {
        return operator[](id.value());
    }

    template<typename Tag>
    [[nodiscard]] const T& operator[](ID<Tag> id) const noexcept {
        return operator[](id.value());
    }

    [[nodiscard]] T& at(uint32_t id) {
        if (id >= m_size) {
            throw std::out_of_range("ArenaPool index out of range");
        }
        return operator[](id);
    }

    [[nodiscard]] const T& at(uint32_t id) const {
        if (id >= m_size) {
            throw std::out_of_range("ArenaPool index out of range");
        }
        return operator[](id);
    }

    template<typename Tag>
    [[nodiscard]] T& at(ID<Tag> id) {
        return at(id.value());
    }

    template<typename Tag>
    [[nodiscard]] const T& at(ID<Tag> id) const {
        return at(id.value());
    }

    [[nodiscard]] size_t size() const noexcept { return m_size; }
    [[nodiscard]] bool empty() const noexcept { return m_size == 0; }
    [[nodiscard]] size_t capacity() const noexcept { return m_chunks.size() * ChunkSize; }

    void reserve(size_t n) {
        const size_t required_chunks = (n + ChunkSize - 1) >> CHUNK_SHIFT;
        while (m_chunks.size() < required_chunks) {
            m_chunks.emplace_back(std::make_unique<T[]>(ChunkSize));
        }
    }

    void clear() noexcept {
        m_size = 0;
        m_chunks.clear();
    }

    template<typename Func>
    void for_each(Func&& f) {
        for (uint32_t i = 0; i < m_size; ++i) {
            f(operator[](i));
        }
    }

    template<typename Func>
    void for_each(Func&& f) const {
        for (uint32_t i = 0; i < m_size; ++i) {
            f(operator[](i));
        }
    }

private:
    uint32_t m_size{0};
    std::vector<std::unique_ptr<T[]>> m_chunks;
};

} // namespace soc::db
