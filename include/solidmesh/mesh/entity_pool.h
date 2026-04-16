#pragma once
#include <vector>
#include <cassert>
#include <cstdint>
#include "solidmesh/mesh/handles.h"

namespace SolidMesh {

// Dense-storage slot-map with generation counters.
//
// Guarantees:
//   - O(1) insert / erase / lookup
//   - Stable handles across insertions; stale handles detectable via generation
//   - Dense iteration (no holes) — erase uses swap-with-last
//   - dense index is NOT stable; never expose it as a public ID
//
// ID must be one of the SM_DEFINE_ID types: {uint32_t slot, uint32_t generation}.

template<typename EntityType, typename EntityID>
class EntityPool {
public:
    // ---- mutation -------------------------------------------------------

    EntityID insert(EntityType value) {
        uint32_t slot;
        if (!free_list_.empty()) {
            slot = free_list_.back();
            free_list_.pop_back();
            sparse_[slot].dense_index = static_cast<uint32_t>(dense_.size());
            sparse_[slot].alive       = true;
            // generation was already bumped on erase
        } else {
            slot = static_cast<uint32_t>(sparse_.size());
            Slot s;
            s.dense_index = static_cast<uint32_t>(dense_.size());
            s.generation  = 0;
            s.alive       = true;
            sparse_.push_back(s);
        }
        dense_.push_back(std::move(value));
        dense_to_sparse_.push_back(slot);
        return EntityID{slot, sparse_[slot].generation};
    }

    // Erase by ID. No-op if id is stale or invalid.
    void erase(EntityID id) {
        if (!alive(id)) return;
        uint32_t di = sparse_[id.slot].dense_index;

        // Swap with last dense element
        if (di != static_cast<uint32_t>(dense_.size()) - 1) {
            uint32_t last_slot = dense_to_sparse_.back();
            dense_[di]              = std::move(dense_.back());
            dense_to_sparse_[di]    = last_slot;
            sparse_[last_slot].dense_index = di;
        }
        dense_.pop_back();
        dense_to_sparse_.pop_back();

        // Invalidate slot
        sparse_[id.slot].alive = false;
        ++sparse_[id.slot].generation;
        free_list_.push_back(id.slot);
    }

    // ---- query ----------------------------------------------------------

    bool alive(EntityID id) const noexcept {
        if (!id.is_valid()) return false;
        if (id.slot >= sparse_.size()) return false;
        const Slot& s = sparse_[id.slot];
        return s.alive && s.generation == id.generation;
    }

    EntityType& get(EntityID id) {
        assert(alive(id));
        return dense_[sparse_[id.slot].dense_index];
    }

    const EntityType& get(EntityID id) const {
        assert(alive(id));
        return dense_[sparse_[id.slot].dense_index];
    }

    size_t size() const noexcept { return dense_.size(); }
    bool   empty() const noexcept { return dense_.empty(); }

    // ---- iteration ------------------------------------------------------
    // Iterates over IDs of all alive elements in dense order.

    struct IDIterator {
        const EntityPool* pool;
        size_t            di;   // dense index

        EntityID operator*() const {
            uint32_t slot = pool->dense_to_sparse_[di];
            return EntityID{slot, pool->sparse_[slot].generation};
        }
        IDIterator& operator++() { ++di; return *this; }
        bool operator!=(const IDIterator& o) const { return di != o.di; }
        bool operator==(const IDIterator& o) const { return di == o.di; }
    };

    struct IDRange {
        const EntityPool* pool;
        IDIterator begin() const { return {pool, 0}; }
        IDIterator end()   const { return {pool, pool->dense_.size()}; }
    };

    IDRange ids() const { return {this}; }

    // Direct dense access (for internal use by Mesh)
    const std::vector<EntityType>& dense() const { return dense_; }
    std::vector<EntityType>&       dense()       { return dense_; }

    // Reconstruct ID for a dense index (internal use)
    EntityID id_at(size_t di) const {
        uint32_t slot = dense_to_sparse_[di];
        return EntityID{slot, sparse_[slot].generation};
    }

private:
    struct Slot {
        uint32_t dense_index;
        uint32_t generation;
        bool     alive;
    };

    std::vector<EntityType>        dense_;
    std::vector<uint32_t> dense_to_sparse_;
    std::vector<Slot>     sparse_;
    std::vector<uint32_t> free_list_;
};

} // namespace SolidMesh
