#pragma once

#include <vector>
#include <cstdint>
#include <cassert>
#include <utility>

namespace SolidMesh {

// SlotMap<T>: O(1) insert/erase/lookup, cache-friendly dense iteration.
//
// IDs are stable uint32_t sparse indices. Deletion uses swap-with-last on
// the dense array to keep storage packed. A generation counter per slot
// lets callers detect stale IDs (optional — check with is_alive()).
//
// Layout:
//   data_[i]              — dense packed storage
//   dense_to_sparse_[i]   — data_[i]'s sparse index
//   sparse_[id].dense_idx — where id lives in data_ (valid only when alive)
//   sparse_[id].gen       — bumped on each erase; 0 = slot never used
//   free_list_            — recycled sparse indices

template<typename T>
class SlotMap {
public:
    static constexpr uint32_t INVALID = ~uint32_t(0);

    // Insert a copy/move of val; returns its stable ID.
    uint32_t insert(T val) {
        uint32_t sparse_idx;
        if (!free_list_.empty()) {
            sparse_idx = free_list_.back();
            free_list_.pop_back();
            sparse_[sparse_idx].dense_idx = static_cast<uint32_t>(data_.size());
            // generation was already bumped on erase
        } else {
            sparse_idx = static_cast<uint32_t>(sparse_.size());
            sparse_.push_back({static_cast<uint32_t>(data_.size()), 1u});
        }
        data_.push_back(std::move(val));
        dense_to_sparse_.push_back(sparse_idx);
        return sparse_idx;
    }

    // Erase element with given ID. O(1) via swap-with-last.
    void erase(uint32_t id) {
        assert(is_alive(id));
        uint32_t di = sparse_[id].dense_idx;
        uint32_t last_di = static_cast<uint32_t>(data_.size()) - 1;

        if (di != last_di) {
            // Move last element into the erased slot
            data_[di] = std::move(data_[last_di]);
            uint32_t moved_sparse = dense_to_sparse_[last_di];
            dense_to_sparse_[di] = moved_sparse;
            sparse_[moved_sparse].dense_idx = di;
        }

        data_.pop_back();
        dense_to_sparse_.pop_back();

        // Invalidate the slot
        sparse_[id].dense_idx = INVALID;
        ++sparse_[id].gen;
        free_list_.push_back(id);
    }

    T& operator[](uint32_t id) {
        assert(is_alive(id));
        return data_[sparse_[id].dense_idx];
    }
    const T& operator[](uint32_t id) const {
        assert(is_alive(id));
        return data_[sparse_[id].dense_idx];
    }

    bool is_alive(uint32_t id) const {
        return id < sparse_.size()
            && sparse_[id].gen > 0
            && sparse_[id].dense_idx != INVALID;
    }

    uint32_t size() const { return static_cast<uint32_t>(data_.size()); }
    bool     empty() const { return data_.empty(); }

    // Dense iteration — iterates live elements in unspecified order.
    T*       begin()       { return data_.data(); }
    T*       end()         { return data_.data() + data_.size(); }
    const T* begin() const { return data_.data(); }
    const T* end()   const { return data_.data() + data_.size(); }

    // Get the sparse ID of the element at dense index i.
    uint32_t id_of(uint32_t dense_idx) const {
        assert(dense_idx < dense_to_sparse_.size());
        return dense_to_sparse_[dense_idx];
    }

    void clear() {
        data_.clear();
        dense_to_sparse_.clear();
        sparse_.clear();
        free_list_.clear();
    }

    void reserve(uint32_t n) {
        data_.reserve(n);
        dense_to_sparse_.reserve(n);
        sparse_.reserve(n);
    }

private:
    struct Slot {
        uint32_t dense_idx; // INVALID when dead
        uint32_t gen;       // 0 = never used; bumped on each erase
    };

    std::vector<T>        data_;
    std::vector<uint32_t> dense_to_sparse_;
    std::vector<Slot>     sparse_;
    std::vector<uint32_t> free_list_;
};

} // namespace SolidMesh
