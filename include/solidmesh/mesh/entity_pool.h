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
        if (!free_list_.empty()) {  // 如果有可用的slot，就复用它
            slot = free_list_.back();
            free_list_.pop_back();
            sparse_[slot].dense_index = static_cast<uint32_t>(dense_.size());   // 新元素在dense_的末尾插入
            sparse_[slot].activate       = true;    // 标记这个slot被占用了
            // generation was already bumped on erase
        } else {
            // 将数据插入dense_的末尾
            slot = static_cast<uint32_t>(sparse_.size());
            EntitySlot s;
            s.dense_index = static_cast<uint32_t>(dense_.size());
            s.generation  = 0;
            s.activate       = true;
            sparse_.push_back(s);
        }
        dense_.push_back(std::move(value));
        dense_to_sparse_.push_back(slot);       // 记录dense_到sparse_的关系
        return EntityID{slot, sparse_[slot].generation};
    }

    // Erase by ID. No-op if id is stale or invalid.
    void erase(EntityID id) {
        if (!exist(id)) return;
        uint32_t di = sparse_[id.slot].dense_index;

        // 为了不让 dense_ 数组中间出现空洞，当删除一个元素时，它将 待删除元素 与 dense_最后一个元素 进行交换。
        if (di != static_cast<uint32_t>(dense_.size()) - 1) {
            uint32_t last_slot = dense_to_sparse_.back();
            dense_[di]              = std::move(dense_.back());
            dense_to_sparse_[di]    = last_slot;    // 更新dense_到sparse_的关系
            sparse_[last_slot].dense_index = di;    // 更新dense_中被交换元素在sparse_中的索引
        }
        dense_.pop_back();
        dense_to_sparse_.pop_back();

        // Invalidate slot
        sparse_[id.slot].activate = false;     // 当前槽位已经失效了
        ++sparse_[id.slot].generation;         
        free_list_.push_back(id.slot);
    }

    // ---- query ----------------------------------------------------------

    bool exist(EntityID id) const noexcept {
        if (!id.has_value()) return false;
        if (id.slot >= sparse_.size()) return false;
        const EntitySlot& s = sparse_[id.slot];
        return s.activate && s.generation == id.generation;
    }

    EntityType& get(EntityID id) {
        assert(exist(id));
        return dense_[sparse_[id.slot].dense_index];
    }

    const EntityType& get(EntityID id) const {
        assert(exist(id));
        return dense_[sparse_[id.slot].dense_index];
    }

    size_t size() const noexcept { return dense_.size(); }
    bool   empty() const noexcept { return dense_.empty(); }

    // ---- iteration ------------------------------------------------------
    // Iterates over IDs of all exist elements in dense order.

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
    struct EntitySlot {
        uint32_t dense_index;
        uint32_t generation;    // generation计数器，slot被用过一次后，generation会增加
        bool     activate;      // 这个槽位当前是否被占用
    };

    std::vector<EntityType>       dense_;               // 连续存储实际的元素
    std::vector<uint32_t>         dense_to_sparse_;     // 存储了dense_中每个元素在sparse_中的索引
    std::vector<EntitySlot>       sparse_;              // 存储了数据在dense_中的索引
    std::vector<uint32_t>         free_list_;           // 存储sparse_中的空闲槽位索引
};

} // namespace SolidMesh
