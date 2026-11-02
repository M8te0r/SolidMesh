#pragma once
#include "entity_pool.h"
#include "handle.h"
#include <cstddef>
#include <utility>

namespace SolidMesh::detail
{
    // Protocol adapter only: EntityPool still owns all storage, allocation and erase logic.
    template <class T>
    class PoolStorage
    {
        using Pool = EntityPool<T, PoolID<T>>;

    public:
        struct key
        {
            using id_type = std::uint64_t;
            id_type raw = 0;
        };
        using size_type = std::size_t;
        using IDIterator = typename Pool::IDIterator;
        template <class... Args>
        key emplace(Args &&...args)
        {
            return {encode(pool_.insert(T{std::forward<Args>(args)...}))};
        }
        const T *get(key k) const noexcept
        {
            const auto id = decode<T>(k.raw);
            return pool_.exist(id) ? &pool_.get(id) : nullptr;
        }
        T *get(key k) noexcept
        {
            const auto id = decode<T>(k.raw);
            return pool_.exist(id) ? &pool_.get(id) : nullptr;
        }
        void erase(key k) { pool_.erase(decode<T>(k.raw)); }
        size_type size() const noexcept { return pool_.size(); }
        auto ids() const noexcept { return pool_.ids(); }
        void swap(PoolStorage &other) noexcept
        {
            using std::swap;
            swap(pool_, other.pool_);
        }

    private:
        Pool pool_;
    };
}
