#pragma once
#include <cstdint>
#include <limits>

namespace SolidMesh
{
    class Mesh;
    namespace detail
    {
        // Adapter for EntityPool's unchanged slot/generation protocol, not a public handle.
        template <class Tag>
        struct PoolID
        {
            using id_type = std::uint64_t;
            std::uint32_t slot = std::numeric_limits<std::uint32_t>::max();
            std::uint32_t generation = 0;
            bool has_value() const noexcept { return slot != std::numeric_limits<std::uint32_t>::max(); }
            bool operator==(PoolID other) const noexcept { return slot == other.slot && generation == other.generation; }
            bool operator!=(PoolID other) const noexcept { return !(*this == other); }
        };
        template <class ID>
        constexpr std::uint64_t encode(ID id) noexcept
        {
            // Pool starts at slot=0,generation=0; reserve zero for an empty public handle.
            return ((std::uint64_t(id.generation) << 32) | id.slot) + 1;
        }
        template <class Tag>
        constexpr PoolID<Tag> decode(std::uint64_t raw) noexcept
        {
            if (!raw)
                return {};
            const auto value = raw - 1;
            return {std::uint32_t(value), std::uint32_t(value >> 32)};
        }
    }
    template <class Tag, typename HandleSize>
    class Handle
    {
    public:
        constexpr Handle() noexcept = default;
        // Empty is not the same as live; use the Mesh element lookup to check existence.
        constexpr bool empty() const noexcept { return raw_ == 0; }
        friend constexpr bool operator==(Handle a, Handle b) noexcept { return a.raw_ == b.raw_; }
        friend constexpr bool operator!=(Handle a, Handle b) noexcept { return !(a == b); }

    private:
        explicit constexpr Handle(HandleSize raw) noexcept : raw_(raw) {}
        HandleSize raw_ = 0;
        friend class Mesh;
        template <class Storage, class H>
        friend class HandleRange;
    };
    template <class Storage, class H>
    class HandleRange
    {
    public:
        explicit HandleRange(const Storage &storage) noexcept : storage_(&storage) {}
        struct Iterator
        {
            typename Storage::IDIterator it;
            H operator*() const noexcept { return H{detail::encode(*it)}; }
            Iterator &operator++() noexcept
            {
                ++it;
                return *this;
            }
            bool operator!=(const Iterator &other) const noexcept { return it != other.it; }
        };
        Iterator begin() const noexcept { return {storage_->ids().begin()}; }
        Iterator end() const noexcept { return {storage_->ids().end()}; }

    private:
        const Storage *storage_;
    };
}
